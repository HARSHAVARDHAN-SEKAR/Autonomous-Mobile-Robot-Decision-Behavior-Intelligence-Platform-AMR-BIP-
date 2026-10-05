#!/usr/bin/env python3
"""Utility-based executive.

Every tick, each candidate activity gets a utility score; the arbiter picks
the max (with hysteresis so the robot doesn't oscillate). Switching activity
cancels the current navigation goal.

    emergency_hold : 1.0 if /emergency else 0
    charge         : battery_urgency(pct) (+ latch bonus while charging flow
                     is active so it runs to completion)
    mission        : 0.55 while missions remain
    idle           : 0.05
"""
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from bip_core.decision import UtilityArbiter, battery_urgency, schedule
from bip_executive.common import (BATTERY_RESUME, BenchmarkStartGate,
                                  MissionProgress, Skills, drive_mission_step,
                                  load_missions)


class UtilityExecutive(Node):
    def __init__(self):
        super().__init__('utility_executive')
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
            f'Utility executive: {len(missions)} missions scheduled')

        self.arbiter = UtilityArbiter(hysteresis=0.08)
        self.activity = None
        self.charge_phase = 'idle'
        self.create_timer(0.1, self.tick)

    # ---------------------------------------------------------------- scores
    def scores(self):
        s = self.skills
        charge_score = battery_urgency(s.battery_pct)
        if self.charge_phase != 'idle':
            charge_score = max(charge_score, 0.75)   # latch until done
        return {
            'emergency_hold': 1.0 if s.emergency else 0.0,
            'charge': charge_score,
            'mission': 0.55 if not self.prog.done() else 0.0,
            'idle': 0.05,
        }

    # ------------------------------------------------------------ activities
    def do_emergency(self):
        if self.skills.nav_busy():
            self.skills.cancel_navigation()

    def do_charge(self):
        s = self.skills
        if self.charge_phase == 'idle':
            if s.nav_busy():
                s.cancel_navigation()
                return
            s.nav_state = 'idle'
            s.navigate(self.get_parameter('dock_x').value,
                       self.get_parameter('dock_y').value)
            self.charge_phase = 'navigating'
        elif self.charge_phase == 'navigating':
            if s.nav_busy():
                return
            if s.nav_state == 'succeeded':
                s.nav_state = 'idle'
                self.charge_phase = 'requesting'
            else:
                s.nav_state = 'idle'
                self.charge_phase = 'idle'
        elif self.charge_phase == 'requesting':
            st = s.charge_request_state()
            if st == 'accepted':
                self.charge_phase = 'charging'
            elif st == 'rejected':
                s._charge_req_state = 'idle'
                self.charge_phase = 'idle'
            else:
                s.request_charging()
        elif self.charge_phase == 'charging':
            if s.battery_pct >= BATTERY_RESUME:
                s.stop_charging()
                self.charge_phase = 'idle'

    def do_mission(self):
        drive_mission_step(self.skills, self.prog, time.monotonic())

    # ------------------------------------------------------------------ tick
    def tick(self):
        if not self.start_gate.started:
            self.skills.publish_status('utility', 'WAITING_START',
                                       self.prog.summary())
            return
        chosen = self.arbiter.choose(self.scores())
        if chosen != self.activity:
            # activity switch -> preempt whatever navigation is running
            if self.skills.nav_busy():
                self.skills.cancel_navigation()
            if self.activity == 'charge' and self.charge_phase == 'navigating':
                self.charge_phase = 'idle'
            self.get_logger().info(f'activity: {self.activity} -> {chosen}')
            self.activity = chosen

        if chosen == 'emergency_hold':
            self.do_emergency()
        elif chosen == 'charge':
            self.do_charge()
        elif chosen == 'mission':
            self.do_mission()

        self.skills.publish_status(
            'utility', f'{chosen}({self.charge_phase})'
            if chosen == 'charge' else (chosen or 'init'),
            self.prog.summary())


def main():
    rclpy.init()
    node = UtilityExecutive()
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
