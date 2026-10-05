"""Shared executive infrastructure.

Skills are non-blocking wrappers around the robot's capabilities. All three
executives (BT / FSM / utility) use the same skills and mission driver so the
comparison changes the arbitration architecture, not the robot API.
"""
import json
import time

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from bip_interfaces.action import GoToPose

from bip_core.missions import (MAX_NAV_RETRIES, MissionProgress,  # noqa: F401
                               drive_mission_step, expand_mission,
                               load_missions)

BATTERY_LOW = 25.0
BATTERY_RESUME = 80.0


class BenchmarkStartGate:
    """Optional deterministic benchmark gate.

    Normal demos start immediately. When ``wait_for_start:=true`` the
    executive stays idle until the benchmark runner publishes ``true`` on
    ``/benchmark/start``. Transient-local QoS prevents a startup race.
    """

    def __init__(self, node):
        node.declare_parameter('wait_for_start', False)
        self.node = node
        self.enabled = bool(node.get_parameter('wait_for_start').value)
        self.started = not self.enabled
        cb = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        node.create_subscription(Bool, 'benchmark/start', self._on_start, qos,
                                 callback_group=cb)

    def _on_start(self, msg):
        if not self.enabled:
            return
        was_started = self.started
        self.started = bool(msg.data)
        if self.started and not was_started:
            self.node.get_logger().info('benchmark start gate released')
        elif not self.started and was_started:
            self.node.get_logger().info('benchmark start gate reset')


class Skills:
    """Non-blocking robot skills. Call poll-style methods from a timer."""

    def __init__(self, node, dock_xy):
        self.node = node
        self.dock_x, self.dock_y = dock_xy
        cb = ReentrantCallbackGroup()

        self.battery_pct = 100.0
        self.charging = False
        self.emergency = False

        node.create_subscription(BatteryState, 'battery', self._on_batt, 10,
                                 callback_group=cb)
        node.create_subscription(Bool, 'emergency', self._on_em, 10,
                                 callback_group=cb)
        self.status_pub = node.create_publisher(String, 'executive/status', 10)

        self._nav = ActionClient(node, GoToPose, 'go_to_pose',
                                 callback_group=cb)
        self._start_chg = node.create_client(Trigger, 'battery/start_charging',
                                             callback_group=cb)
        self._stop_chg = node.create_client(Trigger, 'battery/stop_charging',
                                            callback_group=cb)

        self.nav_state = 'idle'   # idle|pending|running|succeeded|failed|canceled
        self.nav_message = ''
        self._goal_handle = None
        self._charge_req_state = 'idle'  # idle|pending|accepted|rejected

    def _on_batt(self, msg):
        self.battery_pct = msg.percentage * 100.0
        self.charging = (msg.power_supply_status ==
                         BatteryState.POWER_SUPPLY_STATUS_CHARGING)

    def _on_em(self, msg):
        self.emergency = msg.data

    # ------------------------------------------------------------ navigation
    def navigate(self, x, y, yaw=0.0):
        """Start navigation (non-blocking). Returns False if not ready."""
        if not self._nav.server_is_ready():
            return False
        self.nav_state = 'pending'
        self.nav_message = ''
        goal = GoToPose.Goal()
        goal.x, goal.y, goal.yaw = float(x), float(y), float(yaw)
        fut = self._nav.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_resp)
        return True

    def _on_goal_resp(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.nav_state = 'failed'
            self.nav_message = 'rejected'
            return
        self._goal_handle = gh
        self.nav_state = 'running'
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        try:
            res = fut.result().result
        except Exception as exc:  # noqa: BLE001
            self.nav_state = 'failed'
            self.nav_message = str(exc)
            return
        if res.success:
            self.nav_state = 'succeeded'
        elif res.message == 'canceled':
            self.nav_state = 'canceled'
        else:
            self.nav_state = 'failed'
        self.nav_message = res.message
        self._goal_handle = None

    def cancel_navigation(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def nav_busy(self):
        return self.nav_state in ('pending', 'running')

    def reset_nav(self):
        if not self.nav_busy():
            self.nav_state = 'idle'

    # -------------------------------------------------------------- charging
    def request_charging(self):
        if self._charge_req_state == 'pending':
            return
        if not self._start_chg.service_is_ready():
            return
        self._charge_req_state = 'pending'
        fut = self._start_chg.call_async(Trigger.Request())

        def done(f):
            try:
                ok = f.result().success
            except Exception:  # noqa: BLE001
                ok = False
            self._charge_req_state = 'accepted' if ok else 'rejected'
        fut.add_done_callback(done)

    def stop_charging(self):
        if self._stop_chg.service_is_ready():
            self._stop_chg.call_async(Trigger.Request())
        self._charge_req_state = 'idle'

    def charge_request_state(self):
        return self._charge_req_state

    # ---------------------------------------------------------------- status
    def publish_status(self, engine, state, extra=None):
        payload = {
            'engine': engine,
            'state': state,
            'battery': round(self.battery_pct, 1),
            'charging': self.charging,
            'emergency': self.emergency,
            'nav': self.nav_state,
            't': time.time(),
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
