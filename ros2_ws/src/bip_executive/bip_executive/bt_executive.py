#!/usr/bin/env python3
"""Behavior-tree executive.

Tree (reactive fallback at the root -> priorities are the child order):

    Fallback(root)
    ├── Sequence(emergency)   Cond(emergency?) -> Action(hold_position)
    ├── Sequence(battery)     Cond(low_or_charging?) -> Action(charge_routine)
    └── Action(run_missions)

The reactive fallback re-evaluates emergency/battery every tick, so the
mission branch is preempted (and its navigation goal canceled via on_reset)
the moment a higher-priority branch activates.
"""
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from bip_core.bt import Action, Condition, Fallback, Sequence, Status
from bip_core.decision import schedule
from bip_executive.common import (BATTERY_LOW, BATTERY_RESUME,
                                  BenchmarkStartGate, MissionProgress, Skills,
                                  drive_mission_step, load_missions)


class BTExecutive(Node):
    def __init__(self):
        super().__init__('bt_executive')
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
            f'BT executive: {len(missions)} missions scheduled')

        self.charge_active = False
        self.charge_phase = 'idle'   # idle|navigating|requesting|charging
        self.state_label = 'init'

        self.tree = self._build_tree()
        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------ tree nodes
    def _build_tree(self):
        s = self.skills

        def hold_position():
            self.state_label = 'EMERGENCY_HOLD'
            if s.nav_busy():
                s.cancel_navigation()
            return Status.RUNNING if s.emergency else Status.SUCCESS

        def battery_gate():
            return self.charge_active or s.battery_pct < BATTERY_LOW

        def charge_routine():
            self.charge_active = True
            self.state_label = f'CHARGE_{self.charge_phase.upper()}'
            if self.charge_phase == 'idle':
                if s.nav_busy():           # preempted mission goal still ending
                    s.cancel_navigation()
                    return Status.RUNNING
                s.nav_state = 'idle'
                s.navigate(self.get_parameter('dock_x').value,
                           self.get_parameter('dock_y').value)
                self.charge_phase = 'navigating'
                return Status.RUNNING
            if self.charge_phase == 'navigating':
                if s.nav_busy():
                    return Status.RUNNING
                if s.nav_state == 'succeeded':
                    s.nav_state = 'idle'
                    self.charge_phase = 'requesting'
                else:                       # failed/canceled -> retry
                    s.nav_state = 'idle'
                    self.charge_phase = 'idle'
                return Status.RUNNING
            if self.charge_phase == 'requesting':
                st = s.charge_request_state()
                if st == 'accepted':
                    self.charge_phase = 'charging'
                elif st == 'rejected':
                    s._charge_req_state = 'idle'
                    self.charge_phase = 'idle'   # not at dock -> re-navigate
                else:
                    s.request_charging()
                return Status.RUNNING
            # charging
            if s.battery_pct >= BATTERY_RESUME:
                s.stop_charging()
                self.charge_active = False
                self.charge_phase = 'idle'
                return Status.SUCCESS
            return Status.RUNNING

        def run_missions():
            self.state_label = 'MISSION'
            r = drive_mission_step(s, self.prog, time.monotonic())
            if r == 'done':
                self.state_label = 'DONE'
                return Status.SUCCESS
            return Status.RUNNING

        def preempt_mission():
            if s.nav_busy():
                s.cancel_navigation()

        return Fallback('root', [
            Sequence('emergency', [
                Condition('emergency?', lambda: s.emergency),
                Action('hold', hold_position),
            ]),
            Sequence('battery', [
                Condition('battery_low_or_charging?', battery_gate),
                Action('charge', charge_routine),
            ]),
            Action('missions', run_missions, on_reset=preempt_mission),
        ])

    # ------------------------------------------------------------------ tick
    def tick(self):
        if not self.start_gate.started:
            self.skills.publish_status('bt', 'WAITING_START',
                                       self.prog.summary())
            return
        self.tree.tick()
        self.skills.publish_status('bt', self.state_label,
                                   self.prog.summary())


def main():
    rclpy.init()
    node = BTExecutive()
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
