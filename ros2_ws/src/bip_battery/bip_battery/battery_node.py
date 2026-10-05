#!/usr/bin/env python3
"""Simulated battery.

Drains as a function of speed, charges only when the robot is physically
within the dock radius AND charging has been requested. This is the state
that drives most executive-layer decisions.

Interfaces:
  pub  /battery              sensor_msgs/BatteryState (percentage 0..100)
  srv  /battery/start_charging  std_srvs/Trigger  (fails if not at dock)
  srv  /battery/stop_charging   std_srvs/Trigger
  srv  /battery/set             bip_interfaces/SetBattery (chaos injection)
"""
import math

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger

from bip_interfaces.srv import SetBattery


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.declare_parameter('environment_file', '')
        self.declare_parameter('initial_pct', 100.0)
        self.declare_parameter('idle_drain_pct_s', 0.05)
        self.declare_parameter('move_drain_pct_s', 0.45)
        self.declare_parameter('charge_pct_s', 2.0)

        env_file = self.get_parameter('environment_file').value
        with open(env_file) as f:
            env = yaml.safe_load(f)
        dock = env.get('dock', {'x': 0.0, 'y': 0.0, 'radius': 0.6})
        self.dock_x, self.dock_y = float(dock['x']), float(dock['y'])
        self.dock_r = float(dock['radius'])

        self.pct = float(self.get_parameter('initial_pct').value)
        self.charging = False
        self.speed = 0.0
        self.pos = None

        self.pub = self.create_publisher(BatteryState, 'battery', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_service(Trigger, 'battery/start_charging', self.on_start)
        self.create_service(Trigger, 'battery/stop_charging', self.on_stop)
        self.create_service(SetBattery, 'battery/set', self.on_set)
        self.dt = 0.5
        self.create_timer(self.dt, self.on_timer)
        self.get_logger().info(
            f'battery ready, dock=({self.dock_x},{self.dock_y}) r={self.dock_r}')

    def at_dock(self):
        if self.pos is None:
            return False
        return math.hypot(self.pos[0] - self.dock_x,
                          self.pos[1] - self.dock_y) <= self.dock_r

    def on_odom(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def on_start(self, req, res):
        if not self.at_dock():
            res.success = False
            res.message = 'not_at_dock'
            return res
        self.charging = True
        res.success = True
        res.message = 'charging'
        self.get_logger().info('charging started')
        return res

    def on_stop(self, req, res):
        self.charging = False
        res.success = True
        res.message = 'stopped'
        return res

    def on_set(self, req, res):
        self.pct = max(0.0, min(100.0, float(req.level)))
        self.get_logger().warn(f'battery level forced to {self.pct:.1f}%')
        res.success = True
        return res

    def on_timer(self):
        if self.charging and self.at_dock():
            self.pct += self.get_parameter('charge_pct_s').value * self.dt
        else:
            if self.charging and not self.at_dock():
                self.get_logger().warn('left dock while charging -> stop')
                self.charging = False
            idle = self.get_parameter('idle_drain_pct_s').value
            move = self.get_parameter('move_drain_pct_s').value
            drain = idle + move * min(1.0, self.speed / 0.35)
            self.pct -= drain * self.dt
        self.pct = max(0.0, min(100.0, self.pct))

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = self.pct / 100.0
        msg.present = True
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
