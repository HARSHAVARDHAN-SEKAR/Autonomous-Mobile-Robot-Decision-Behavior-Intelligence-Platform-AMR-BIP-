#!/usr/bin/env python3
"""Finite-state-machine executive.

States: MISSION -> (GO_DOCK -> REQUEST_CHARGE -> CHARGING) -> MISSION
        any state -> EMERGENCY -> back to interrupted flow
        MISSION -> DONE

Same skills interface and same mission-driving logic as the BT engine —
only the arbitration mechanism differs.
"""
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from bip_core.decision import StateMachine, schedule
from bip_executive.common import (BATTERY_LOW, BATTERY_RESUME,
                                  BenchmarkStartGate, MissionProgress, Skills,
                                  drive_mission_step, load_missions)


class FSMExecutive(Node):
    def __init__(self):
        super().__init__('fsm_executive')
        self.declare_parameter('missions_file', '')
        self.declare_parameter('dock_x', -5.2)
        self.declare_parameter('dock_y', 3.2)

        dock = (self.get_parameter('dock_x').value,
                self.get_parameter('dock_y').value)
        self.skills = Skills(self, dock)
        self.start_gate = BenchmarkStartGate(self)
        missions, _ = schedule(
            load_missions(self.get_parameter('missions_file').value))
        self.prog = MissionProgress(missions)
        self.get_logger().info(
            f'FSM executive: {len(missions)} missions scheduled')

        self.resume_state = 'MISSION'
        self.sm = self._build()
        self.create_timer(0.1, self.tick)

    def _build(self):
        s = self.skills
        sm = StateMachine('MISSION')

        def check_emergency(current):
            if s.emergency:
                self.resume_state = current
                return 'EMERGENCY'
            return None

        # ------------------------------------------------------------ states
        def mission_tick():
            e = check_emergency('MISSION')
            if e:
                return e
            if s.battery_pct < BATTERY_LOW:
                if s.nav_busy():
                    s.cancel_navigation()
                    return None          # wait for cancel to land
                s.nav_state = 'idle'
                return 'GO_DOCK'
            r = drive_mission_step(s, self.prog, time.monotonic())
            return 'DONE' if r == 'done' else None

        def go_dock_enter():
            s.navigate(self.get_parameter('dock_x').value,
                       self.get_parameter('dock_y').value)

        def go_dock_tick():
            e = check_emergency('GO_DOCK')
            if e:
                return e
            if s.nav_busy():
                return None
            if s.nav_state == 'succeeded':
                s.nav_state = 'idle'
                return 'REQUEST_CHARGE'
            s.nav_state = 'idle'         # failed/canceled -> retry
            go_dock_enter()
            return None

        def request_tick():
            e = check_emergency('REQUEST_CHARGE')
            if e:
                return e
            st = s.charge_request_state()
            if st == 'accepted':
                return 'CHARGING'
            if st == 'rejected':
                s._charge_req_state = 'idle'
                return 'GO_DOCK'
            s.request_charging()
            return None

        def charging_tick():
            e = check_emergency('CHARGING')
            if e:
                return e
            if s.battery_pct >= BATTERY_RESUME:
                s.stop_charging()
                return 'MISSION'
            return None

        def emergency_enter():
            if s.nav_busy():
                s.cancel_navigation()

        def emergency_tick():
            if not s.emergency:
                s.nav_state = 'idle' if not s.nav_busy() else s.nav_state
                # charging request may have been interrupted -> restart flow
                if self.resume_state in ('GO_DOCK', 'REQUEST_CHARGE'):
                    return 'GO_DOCK'
                return self.resume_state
            return None

        sm.add_state('MISSION', mission_tick)
        sm.add_state('GO_DOCK', go_dock_tick, on_enter=go_dock_enter)
        sm.add_state('REQUEST_CHARGE', request_tick)
        sm.add_state('CHARGING', charging_tick)
        sm.add_state('EMERGENCY', emergency_tick, on_enter=emergency_enter)
        sm.add_state('DONE', lambda: None)
        return sm

    def tick(self):
        if not self.start_gate.started:
            self.skills.publish_status('fsm', 'WAITING_START',
                                       self.prog.summary())
            return
        state = self.sm.tick()
        self.skills.publish_status('fsm', state, self.prog.summary())


def main():
    rclpy.init()
    node = FSMExecutive()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
