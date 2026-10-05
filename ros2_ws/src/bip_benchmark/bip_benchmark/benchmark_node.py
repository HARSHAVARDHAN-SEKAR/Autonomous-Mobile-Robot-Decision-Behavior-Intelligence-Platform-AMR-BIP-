#!/usr/bin/env python3
"""Controlled benchmark runner for BT/FSM/utility executive comparison.

For a valid comparison, launch the executive with ``wait_for_start:=true``.
The runner waits for the executive, restores the Gazebo world pose, clears
virtual obstacles, stops charging, resets battery to 100%, clears emergency,
then releases the benchmark start gate. Fault times are measured from that
same release instant for every engine.
"""
import csv
import json
import os
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, SetBool, Trigger

from bip_interfaces.srv import AddObstacle, SetBattery

BATTERY_RESUME = 80.0

SCENARIOS = {
    'baseline': [],
    'battery_drop': [(20.0, 'battery', 18.0)],
    'emergency': [(15.0, 'emergency_on', None),
                  (23.0, 'emergency_off', None)],
    'blocked_path': [(10.0, 'block', (1.2, -2.0, 1.0, 1.0))],
    'full_chaos': [(10.0, 'block', (1.2, -2.0, 1.0, 1.0)),
                   (20.0, 'battery', 18.0),
                   (40.0, 'emergency_on', None),
                   (48.0, 'emergency_off', None)],
}


class Benchmark(Node):
    def __init__(self):
        super().__init__('benchmark_node')
        self.declare_parameter('scenario', 'baseline')
        self.declare_parameter('timeout_s', 600.0)
        self.declare_parameter('out_dir', 'results')
        self.declare_parameter('startup_timeout_s', 30.0)
        self.declare_parameter('settle_s', 2.0)
        self.declare_parameter('reset_world', True)
        self.declare_parameter('initial_battery_pct', 100.0)

        self.scenario = self.get_parameter('scenario').value
        if self.scenario not in SCENARIOS:
            raise ValueError(f'unknown scenario {self.scenario}; '
                             f'options: {list(SCENARIOS)}')
        self.events = sorted(SCENARIOS[self.scenario])
        self.fired = [False] * len(self.events)

        cb = ReentrantCallbackGroup()

        # Fault injection services.
        self.batt = self.create_client(SetBattery, 'chaos/set_battery',
                                       callback_group=cb)
        self.em = self.create_client(SetBool, 'chaos/emergency',
                                     callback_group=cb)
        self.block = self.create_client(AddObstacle, 'chaos/block_path',
                                        callback_group=cb)

        # Deterministic pre-run reset services.
        self.reset_world_cli = self.create_client(Empty, 'reset_world',
                                                   callback_group=cb)
        self.batt_reset_cli = self.create_client(SetBattery, 'battery/set',
                                                  callback_group=cb)
        self.stop_charge_cli = self.create_client(
            Trigger, 'battery/stop_charging', callback_group=cb)
        self.clear_obs_cli = self.create_client(
            Trigger, 'nav/clear_obstacles', callback_group=cb)
        self.em_reset_cli = self.create_client(SetBool, 'chaos/emergency',
                                                callback_group=cb)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.start_pub = self.create_publisher(Bool, 'benchmark/start', qos)

        self.create_subscription(String, 'executive/status', self.on_status,
                                 10, callback_group=cb)
        self.create_subscription(String, 'nav/events', self.on_nav_event,
                                 10, callback_group=cb)

        self.engine = 'unknown'
        self.waiting_seen = False
        self.run_started = False
        self.t0 = None
        self.last = None
        self.rows = []
        self.state_switches = 0
        self.fault_records = []
        self.finished = False

        self.startup_started = time.monotonic()
        self.reset_sent = False
        self.reset_futures = []
        self.settle_until = None

        # Publish a durable false immediately so a waiting executive cannot be
        # released by accident before the controlled reset completes.
        self._publish_start(False)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'benchmark scenario={self.scenario}; waiting for executive gate')

    # ------------------------------------------------------------------ QoS
    def _publish_start(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.start_pub.publish(msg)

    # ---------------------------------------------------------------- status
    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self.engine = data.get('engine', self.engine)
        if str(data.get('state', '')).upper() == 'WAITING_START':
            self.waiting_seen = True

        if not self.run_started or self.t0 is None:
            return

        t = time.monotonic() - self.t0
        if self.last and data.get('state') != self.last.get('state'):
            self.state_switches += 1
        self.last = data

        self.rows.append([
            round(t, 2), data.get('state'), data.get('battery'),
            data.get('charging'), data.get('emergency'), data.get('nav'),
            data.get('mission'), data.get('step'),
            len(data.get('completed', [])), len(data.get('failed', [])),
        ])

        self._update_status_recoveries(data, t)

        # MissionProgress exposes mission=None only after every mission has
        # completed or been explicitly skipped/failed.
        if data.get('mission') is None and (data.get('completed') or
                                             data.get('failed')):
            all_ok = len(data.get('failed', [])) == 0
            self.finish(success=all_ok, reason='mission_set_finished')

    def on_nav_event(self, msg):
        if not self.run_started or self.t0 is None:
            return
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        t = time.monotonic() - self.t0
        if event.get('event') == 'replan_succeeded':
            rec = self._oldest_pending('block')
            if rec is not None and t >= rec['injected_s']:
                self._mark_recovered(rec, t, 'replanned')

    # -------------------------------------------------------------- startup
    def _startup_services_ready(self):
        services = [
            ('battery/set', self.batt_reset_cli),
            ('battery/stop_charging', self.stop_charge_cli),
            ('nav/clear_obstacles', self.clear_obs_cli),
            ('chaos/emergency', self.em_reset_cli),
        ]
        if self.get_parameter('reset_world').value:
            services.append(('reset_world', self.reset_world_cli))
        missing = [name for name, cli in services if not cli.service_is_ready()]
        return missing

    def _send_resets(self):
        self.reset_futures = []
        if self.get_parameter('reset_world').value:
            self.reset_futures.append(
                ('reset_world', self.reset_world_cli.call_async(Empty.Request())))

        batt_req = SetBattery.Request()
        batt_req.level = float(self.get_parameter('initial_battery_pct').value)
        self.reset_futures.append(
            ('battery/set', self.batt_reset_cli.call_async(batt_req)))
        self.reset_futures.append(
            ('battery/stop_charging',
             self.stop_charge_cli.call_async(Trigger.Request())))
        self.reset_futures.append(
            ('nav/clear_obstacles',
             self.clear_obs_cli.call_async(Trigger.Request())))
        em_req = SetBool.Request()
        em_req.data = False
        self.reset_futures.append(
            ('chaos/emergency', self.em_reset_cli.call_async(em_req)))
        self.reset_sent = True
        self.get_logger().info('controlled pre-run reset requested')

    def _validate_resets(self):
        if not self.reset_futures or not all(f.done() for _, f in self.reset_futures):
            return False
        for name, fut in self.reset_futures:
            try:
                res = fut.result()
            except Exception as exc:  # noqa: BLE001
                self._startup_fail(f'{name} reset failed: {exc}')
                return False
            if hasattr(res, 'success') and not res.success:
                self._startup_fail(f'{name} reset returned failure')
                return False
        return True

    def _startup_fail(self, reason):
        if self.finished:
            return
        self.finished = True
        self.get_logger().error(f'benchmark startup invalid: {reason}')
        rclpy.shutdown()

    def _start_run(self):
        self.rows.clear()
        self.last = None
        self.state_switches = 0
        self.fault_records.clear()
        self.t0 = time.monotonic()
        self.run_started = True
        self._publish_start(True)
        self.get_logger().info(
            f'RUN START engine={self.engine} scenario={self.scenario}')

    # ------------------------------------------------------------------ tick
    def tick(self):
        if self.finished:
            return

        now = time.monotonic()
        if not self.run_started:
            startup_timeout = float(
                self.get_parameter('startup_timeout_s').value)
            if now - self.startup_started > startup_timeout:
                self._startup_fail(
                    'timed out waiting for WAITING_START executive/reset services')
                return
            if not self.waiting_seen:
                return
            if not self.reset_sent:
                missing = self._startup_services_ready()
                if missing:
                    return
                self._send_resets()
                return
            if self.settle_until is None:
                if not self._validate_resets():
                    return
                self.settle_until = now + float(
                    self.get_parameter('settle_s').value)
                self.get_logger().info('reset complete; settling robot state')
                return
            if now >= self.settle_until:
                self._start_run()
            return

        t = now - self.t0
        for i, (scheduled_t, kind, arg) in enumerate(self.events):
            if not self.fired[i] and t >= scheduled_t:
                self.fired[i] = True
                self.inject(kind, arg, scheduled_t, t)

        if t > float(self.get_parameter('timeout_s').value):
            self.get_logger().error('TIMEOUT')
            self.finish(success=False, reason='timeout')

    # ------------------------------------------------------------- injection
    def _new_fault(self, fault_type, scheduled_t, injected_t):
        rec = {
            'id': f'{fault_type}_{len(self.fault_records) + 1}',
            'fault_type': fault_type,
            'scheduled_s': round(float(scheduled_t), 2),
            'injected_s': round(float(injected_t), 2),
            'released_s': None,
            'recovered_s': None,
            'recovery_time_s': None,
            'outcome': 'pending',
        }
        self.fault_records.append(rec)
        return rec

    def _oldest_pending(self, fault_type):
        for rec in self.fault_records:
            if rec['fault_type'] == fault_type and rec['outcome'] == 'pending':
                return rec
        return None

    def _mark_recovered(self, rec, t, outcome):
        if rec is None or rec['outcome'] != 'pending':
            return
        rec['recovered_s'] = round(float(t), 2)
        rec['recovery_time_s'] = round(float(t) - rec['injected_s'], 2)
        rec['outcome'] = outcome
        self.get_logger().info(
            f"RECOVERY {rec['id']} -> {outcome} in "
            f"{rec['recovery_time_s']:.2f}s")

    def _track_injection_future(self, future, rec):
        if future is None or rec is None:
            return

        def done(fut):
            if rec['outcome'] != 'pending':
                return
            try:
                res = fut.result()
            except Exception as exc:  # noqa: BLE001
                rec['outcome'] = f'injection_error:{type(exc).__name__}'
                return
            if hasattr(res, 'success') and not res.success:
                rec['outcome'] = 'injection_failed'
        future.add_done_callback(done)

    def inject(self, kind, arg, scheduled_t, actual_t):
        self.get_logger().warn(f'INJECT {kind} {arg}')
        if kind == 'battery':
            rec = self._new_fault('battery', scheduled_t, actual_t)
            if self.batt.service_is_ready():
                req = SetBattery.Request()
                req.level = float(arg)
                self._track_injection_future(self.batt.call_async(req), rec)
            else:
                rec['outcome'] = 'service_unavailable'

        elif kind == 'emergency_on':
            rec = self._new_fault('emergency', scheduled_t, actual_t)
            if self.em.service_is_ready():
                req = SetBool.Request()
                req.data = True
                self._track_injection_future(self.em.call_async(req), rec)
            else:
                rec['outcome'] = 'service_unavailable'

        elif kind == 'emergency_off':
            rec = self._oldest_pending('emergency')
            if rec is not None:
                rec['released_s'] = round(float(actual_t), 2)
            if self.em.service_is_ready():
                req = SetBool.Request()
                req.data = False
                self.em.call_async(req)
            elif rec is not None:
                rec['outcome'] = 'release_service_unavailable'

        elif kind == 'block':
            rec = self._new_fault('block', scheduled_t, actual_t)
            if self.block.service_is_ready():
                x, y, sx, sy = arg
                req = AddObstacle.Request()
                req.x, req.y = float(x), float(y)
                req.size_x, req.size_y = float(sx), float(sy)
                self._track_injection_future(self.block.call_async(req), rec)
            else:
                rec['outcome'] = 'service_unavailable'

    def _update_status_recoveries(self, data, t):
        state = str(data.get('state', '')).upper()

        # Battery fault is recovered when the charging routine has restored
        # resume threshold and control has returned to mission execution.
        for rec in self.fault_records:
            if rec['fault_type'] == 'battery' and rec['outcome'] == 'pending':
                if float(data.get('battery') or 0.0) >= BATTERY_RESUME and \
                        'MISSION' in state and not bool(data.get('charging')):
                    self._mark_recovered(rec, t, 'charged_and_resumed')

        # Emergency recovery is measured from injection until the hold has
        # actually been released into any non-emergency executive state.
        for rec in self.fault_records:
            if rec['fault_type'] == 'emergency' and rec['outcome'] == 'pending':
                if rec.get('released_s') is not None and \
                        not bool(data.get('emergency')) and \
                        'EMERGENCY' not in state:
                    self._mark_recovered(rec, t, 'released')

    # ---------------------------------------------------------------- finish
    def finish(self, success, reason):
        if self.finished:
            return
        self.finished = True

        for rec in self.fault_records:
            if rec['outcome'] == 'pending':
                rec['outcome'] = 'unresolved'

        out_dir = self.get_parameter('out_dir').value
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f'{self.engine}_{self.scenario}.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'state', 'battery', 'charging', 'emergency',
                             'nav', 'mission', 'step', 'n_completed',
                             'n_failed'])
            writer.writerows(self.rows)

        last = self.last or {}
        total_time = 0.0 if self.t0 is None else time.monotonic() - self.t0
        summary = {
            'engine': self.engine,
            'scenario': self.scenario,
            'success': bool(success),
            'termination_reason': reason,
            'controlled_start': True,
            'world_reset_before_run': bool(
                self.get_parameter('reset_world').value),
            'initial_battery_pct': float(
                self.get_parameter('initial_battery_pct').value),
            'total_time_s': round(total_time, 1),
            'missions_completed': len(last.get('completed', [])),
            'missions_failed': len(last.get('failed', [])),
            'state_switches': self.state_switches,
            'fault_recoveries': self.fault_records,
        }
        spath = os.path.join(
            out_dir, f'{self.engine}_{self.scenario}_summary.json')
        with open(spath, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(
            f'RESULTS -> {path}\n{json.dumps(summary, indent=2)}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Benchmark()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:  # executor can raise after self-shutdown
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
