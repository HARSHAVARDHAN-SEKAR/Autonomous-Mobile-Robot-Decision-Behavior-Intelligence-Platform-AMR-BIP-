#!/usr/bin/env python3
"""Fault-injection ("chaos") node.

User-facing services:
  /chaos/set_battery     bip_interfaces/SetBattery
  /chaos/emergency       std_srvs/SetBool
  /chaos/block_path      bip_interfaces/AddObstacle
  /chaos/clear_obstacles std_srvs/Trigger
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from bip_interfaces.srv import AddObstacle, SetBattery


class ChaosNode(Node):
    def __init__(self):
        super().__init__('chaos_node')
        cb = ReentrantCallbackGroup()
        self.em_pub = self.create_publisher(Bool, 'emergency', 10)

        self.batt_cli = self.create_client(SetBattery, 'battery/set',
                                           callback_group=cb)
        self.obs_cli = self.create_client(AddObstacle, 'nav/add_obstacle',
                                          callback_group=cb)
        self.clear_cli = self.create_client(Trigger, 'nav/clear_obstacles',
                                            callback_group=cb)

        self.create_service(SetBattery, 'chaos/set_battery',
                            self.on_battery, callback_group=cb)
        self.create_service(SetBool, 'chaos/emergency',
                            self.on_emergency, callback_group=cb)
        self.create_service(AddObstacle, 'chaos/block_path',
                            self.on_block, callback_group=cb)
        self.create_service(Trigger, 'chaos/clear_obstacles',
                            self.on_clear, callback_group=cb)
        self.get_logger().info('chaos node ready')

    def on_battery(self, req, res):
        if not self.batt_cli.wait_for_service(timeout_sec=2.0):
            res.success = False
            return res
        self.batt_cli.call_async(SetBattery.Request(level=req.level))
        self.get_logger().warn(f'CHAOS: battery -> {req.level:.0f}%')
        res.success = True
        return res

    def on_emergency(self, req, res):
        msg = Bool()
        msg.data = req.data
        self.em_pub.publish(msg)
        self.get_logger().warn(f'CHAOS: emergency = {req.data}')
        res.success = True
        res.message = 'emergency_set'
        return res

    def on_block(self, req, res):
        if not self.obs_cli.wait_for_service(timeout_sec=2.0):
            res.success = False
            return res
        r = AddObstacle.Request()
        r.x, r.y, r.size_x, r.size_y = req.x, req.y, req.size_x, req.size_y
        self.obs_cli.call_async(r)
        self.get_logger().warn(
            f'CHAOS: path blocked at ({req.x:.1f},{req.y:.1f})')
        res.success = True
        return res

    def on_clear(self, req, res):
        del req
        if not self.clear_cli.wait_for_service(timeout_sec=2.0):
            res.success = False
            res.message = 'nav_clear_service_unavailable'
            return res
        self.clear_cli.call_async(Trigger.Request())
        self.get_logger().info('CHAOS: virtual obstacles clear requested')
        res.success = True
        res.message = 'clear_requested'
        return res


def main():
    rclpy.init()
    node = ChaosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
