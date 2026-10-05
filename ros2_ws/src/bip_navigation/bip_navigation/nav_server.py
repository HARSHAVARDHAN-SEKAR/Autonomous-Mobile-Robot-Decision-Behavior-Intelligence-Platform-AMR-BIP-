#!/usr/bin/env python3
"""GoToPose action server: A* global planning + pure-pursuit control.

Deliberately Nav2-free: the planning/control pipeline is explicit and easy to
inspect. Features used by the executive and benchmark layers:
  * cancellation and emergency-stop handling
  * virtual obstacle injection and explicit obstacle reset
  * immediate replanning when the virtual map changes
  * scan-based blocking detection: wait -> replan -> bounded failure
  * navigation event telemetry for benchmark recovery measurements
"""
import json
import math
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from bip_core import astar
from bip_core.grid import grid_from_config
from bip_interfaces.action import GoToPose
from bip_interfaces.srv import AddObstacle


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class NavServer(Node):
    def __init__(self):
        super().__init__('nav_server')
        self.declare_parameter('environment_file', '')
        self.declare_parameter('max_lin', 0.35)
        self.declare_parameter('max_ang', 1.2)
        self.declare_parameter('lookahead', 0.5)
        self.declare_parameter('goal_tol', 0.18)
        self.declare_parameter('yaw_tol', 0.15)
        self.declare_parameter('yaw_align_timeout_s', 4.0)
        self.declare_parameter('obstacle_stop_dist', 0.35)
        self.declare_parameter('block_wait_s', 2.5)
        self.declare_parameter('max_block_retries', 3)

        env_file = self.get_parameter('environment_file').value
        with open(env_file, encoding='utf-8') as f:
            self.env = yaml.safe_load(f)
        self.grid = grid_from_config(self.env)
        self.robot_radius = float(self.env.get('robot_radius', 0.3))

        self.virtual_obstacles = []
        self._map_revision = 0
        self.lock = threading.Lock()

        self.pose = None          # (x, y, yaw)
        self.scan = None
        self.emergency = False

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.event_pub = self.create_publisher(String, 'nav/events', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(Bool, 'emergency', self.on_emergency, 10,
                                 callback_group=cb)
        self.create_service(AddObstacle, 'nav/add_obstacle',
                            self.on_add_obstacle, callback_group=cb)
        self.create_service(Trigger, 'nav/clear_obstacles',
                            self.on_clear_obstacles, callback_group=cb)

        self._active_goal = None
        self.server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info('nav_server ready (A* + pure pursuit, no Nav2)')

    # ------------------------------------------------------------ callbacks
    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_scan(self, msg):
        self.scan = msg

    def on_emergency(self, msg):
        self.emergency = msg.data
        if self.emergency:
            self.get_logger().warn('EMERGENCY STOP received')
            self.stop()

    def on_add_obstacle(self, req, res):
        with self.lock:
            self.virtual_obstacles.append(
                (req.x, req.y, req.size_x, req.size_y))
            self._map_revision += 1
            revision = self._map_revision
        self.get_logger().warn(
            f'virtual obstacle added at ({req.x:.1f},{req.y:.1f}); '
            f'map_revision={revision}')
        self.publish_event('virtual_obstacle_added', map_revision=revision,
                           x=req.x, y=req.y, size_x=req.size_x,
                           size_y=req.size_y)
        res.success = True
        return res

    def on_clear_obstacles(self, req, res):
        del req
        with self.lock:
            count = len(self.virtual_obstacles)
            self.virtual_obstacles.clear()
            self._map_revision += 1
            revision = self._map_revision
        res.success = True
        res.message = f'cleared_{count}_virtual_obstacles'
        self.get_logger().info(
            f'cleared {count} virtual obstacles; map_revision={revision}')
        self.publish_event('virtual_obstacles_cleared', map_revision=revision,
                           count=count)
        return res

    def on_goal(self, goal_request):
        del goal_request
        # Single active goal: reject if busy (executive must cancel first).
        if self._active_goal is not None:
            self.get_logger().warn('goal rejected: server busy')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------ helpers
    def current_map_revision(self):
        with self.lock:
            return self._map_revision

    def occupancy(self):
        with self.lock:
            obstacles = list(self.virtual_obstacles)
        g = grid_from_config(self.env)
        for (x, y, sx, sy) in obstacles:
            g.add_rect(x, y, sx, sy)
        return g, g.inflated(self.robot_radius)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def publish_event(self, event, **details):
        payload = {'event': event}
        payload.update(details)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.event_pub.publish(msg)

    def front_blocked(self):
        s = self.scan
        if s is None:
            return False
        stop_d = self.get_parameter('obstacle_stop_dist').value
        n = len(s.ranges)
        if n == 0:
            return False
        # +-30 deg sector around the front (scan spans -pi..pi).
        sector = int(n * 30.0 / 360.0)
        mid = n // 2
        front = list(s.ranges[mid - sector: mid + sector])
        vals = [r for r in front if s.range_min < r < s.range_max]
        return bool(vals) and min(vals) < stop_d

    def plan_path(self, goal_xy):
        g, occ = self.occupancy()
        start = g.world_to_cell(self.pose[0], self.pose[1])
        goal = g.world_to_cell(*goal_xy)
        cells = astar.plan(occ, start, goal)
        if cells is None:
            return None
        cells = astar.smooth(occ, cells)
        pts = [g.cell_to_world(cx, cy) for (cx, cy) in cells]
        self.publish_path(pts)
        return pts

    def publish_path(self, pts):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _cancel(self, goal_handle, t0):
        goal_handle.canceled()
        self.stop()
        self.publish_event('goal_canceled')
        r = GoToPose.Result()
        r.success = False
        r.message = 'canceled'
        r.total_time = time.monotonic() - t0
        return r

    def _fail(self, goal_handle, msg, t0):
        self.get_logger().warn(f'goal failed: {msg}')
        goal_handle.abort()
        self.publish_event('goal_failed', reason=msg)
        r = GoToPose.Result()
        r.success = False
        r.message = msg
        r.total_time = time.monotonic() - t0
        return r

    def _succeed(self, goal_handle, t0):
        goal_handle.succeed()
        self.publish_event('goal_succeeded')
        r = GoToPose.Result()
        r.success = True
        r.message = 'reached'
        r.total_time = time.monotonic() - t0
        return r

    # ------------------------------------------------------------ execution
    def execute(self, goal_handle):
        self._active_goal = goal_handle
        try:
            return self._execute(goal_handle)
        finally:
            self._active_goal = None
            self.stop()

    def _execute(self, goal_handle):
        goal = goal_handle.request
        t0 = time.monotonic()
        rate_dt = 0.05
        max_lin = self.get_parameter('max_lin').value
        max_ang = self.get_parameter('max_ang').value
        lookahead = self.get_parameter('lookahead').value
        goal_tol = self.get_parameter('goal_tol').value
        yaw_tol = self.get_parameter('yaw_tol').value
        yaw_timeout = self.get_parameter('yaw_align_timeout_s').value
        block_wait = self.get_parameter('block_wait_s').value
        max_retries = int(self.get_parameter('max_block_retries').value)

        while self.pose is None:
            if not rclpy.ok():
                return self._fail(goal_handle, 'shutdown', t0)
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, t0)
            if self.emergency:
                return self._fail(goal_handle, 'emergency_stop', t0)
            time.sleep(0.05)

        path = self.plan_path((goal.x, goal.y))
        if path is None:
            return self._fail(goal_handle, 'no_path', t0)
        plan_revision = self.current_map_revision()
        self.publish_event('goal_started', x=goal.x, y=goal.y,
                           map_revision=plan_revision)

        block_retries = 0
        idx = 0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, t0)
            if self.emergency:
                return self._fail(goal_handle, 'emergency_stop', t0)

            x, y, yaw = self.pose
            gx, gy = goal.x, goal.y

            # Replan immediately when chaos/manual injection changes the
            # virtual map. Without this, an obstacle added after planning would
            # affect only the next goal, invalidating blocked-path benchmarks.
            revision = self.current_map_revision()
            if revision != plan_revision:
                self.stop()
                path = self.plan_path((gx, gy))
                if path is None:
                    return self._fail(goal_handle, 'no_path_after_map_update', t0)
                idx = 0
                plan_revision = self.current_map_revision()
                self.publish_event('replan_succeeded', reason='map_update',
                                   map_revision=plan_revision)

            dist_goal = math.hypot(gx - x, gy - y)
            fb = GoToPose.Feedback()
            fb.distance_remaining = float(dist_goal)
            goal_handle.publish_feedback(fb)

            if dist_goal < goal_tol:
                self.stop()
                target_yaw = goal.yaw
                deadline = time.monotonic() + float(yaw_timeout)
                while time.monotonic() < deadline:
                    if goal_handle.is_cancel_requested:
                        return self._cancel(goal_handle, t0)
                    if self.emergency:
                        return self._fail(goal_handle, 'emergency_stop', t0)
                    err = ang_norm(target_yaw - self.pose[2])
                    if abs(err) < yaw_tol:
                        self.stop()
                        return self._succeed(goal_handle, t0)
                    tw = Twist()
                    tw.angular.z = max(-max_ang, min(max_ang, 2.0 * err))
                    self.cmd_pub.publish(tw)
                    time.sleep(rate_dt)

                self.stop()
                # A pose goal includes orientation; do not silently report
                # success when the positional tolerance was met but yaw was not.
                if goal_handle.is_cancel_requested:
                    return self._cancel(goal_handle, t0)
                if self.emergency:
                    return self._fail(goal_handle, 'emergency_stop', t0)
                if abs(ang_norm(target_yaw - self.pose[2])) >= yaw_tol:
                    return self._fail(goal_handle, 'yaw_timeout', t0)
                return self._succeed(goal_handle, t0)

            # ---- blocking obstacle handling: wait -> replan -> abort
            if self.front_blocked():
                self.stop()
                waited = 0.0
                while waited < block_wait and self.front_blocked():
                    if goal_handle.is_cancel_requested:
                        return self._cancel(goal_handle, t0)
                    if self.emergency:
                        return self._fail(goal_handle, 'emergency_stop', t0)
                    time.sleep(0.1)
                    waited += 0.1

                if self.front_blocked():
                    block_retries += 1
                    if block_retries > max_retries:
                        return self._fail(goal_handle, 'blocked', t0)
                    # Register what we see as a small virtual obstacle and replan.
                    bx = x + 0.5 * math.cos(yaw)
                    by = y + 0.5 * math.sin(yaw)
                    with self.lock:
                        self.virtual_obstacles.append((bx, by, 0.4, 0.4))
                        self._map_revision += 1
                    path = self.plan_path((gx, gy))
                    if path is None:
                        return self._fail(goal_handle, 'no_path_after_block', t0)
                    idx = 0
                    plan_revision = self.current_map_revision()
                    self.publish_event('replan_succeeded', reason='scan_block',
                                       retry=block_retries,
                                       map_revision=plan_revision)
                continue

            # ---- pure pursuit: find lookahead point on remaining path
            while idx < len(path) - 1 and \
                    math.hypot(path[idx][0] - x, path[idx][1] - y) < lookahead:
                idx += 1
            tx, ty = path[idx]
            heading = math.atan2(ty - y, tx - x)
            err = ang_norm(heading - yaw)

            tw = Twist()
            if abs(err) > 0.9:
                tw.linear.x = 0.0                      # rotate in place
            else:
                slow = min(1.0, dist_goal / 0.6)
                tw.linear.x = max_lin * slow * (1.0 - 0.6 * abs(err) / 0.9)
            tw.angular.z = max(-max_ang, min(max_ang, 1.8 * err))
            self.cmd_pub.publish(tw)
            time.sleep(rate_dt)

        return self._fail(goal_handle, 'shutdown', t0)


def main():
    rclpy.init()
    node = NavServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
