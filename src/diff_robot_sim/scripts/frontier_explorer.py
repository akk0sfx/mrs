#!/usr/bin/env python3
from collections import deque
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class FrontierExplorer(Node):
    """Simple frontier-based explorer that sends goals to Nav2."""

    def __init__(self):
        super().__init__('frontier_explorer')
        self._declare_if_missing('use_sim_time', False)
        self._map_topic = self._declare_if_missing('map_topic', '/map')
        self._map_frame = self._declare_if_missing('map_frame', 'map')
        self._base_frame = self._declare_if_missing('base_frame', 'base_footprint')
        self._min_frontier_size = int(self._declare_if_missing('min_frontier_size', 10))
        self._min_goal_distance = float(self._declare_if_missing('min_goal_distance', 0.6))
        self._goal_timeout = float(self._declare_if_missing('goal_timeout', 60.0))
        self._replan_period = float(self._declare_if_missing('replan_period', 2.0))
        self._search_radius = int(self._declare_if_missing('free_cell_search_radius', 12))

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map = None
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._goal_start_time = None
        self._last_goal = None

        self.create_timer(self._replan_period, self._tick)
        self.get_logger().info('Frontier explorer waiting for /map and Nav2.')

    def _declare_if_missing(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _on_map(self, msg):
        self._map = msg

    def _get_robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quaternion(tf.transform.rotation)
        return x, y, yaw

    def _is_frontier(self, data, width, height, x, y):
        idx = y * width + x
        if data[idx] != -1:
            return False
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < width and 0 <= ny < height:
                nidx = ny * width + nx
                if data[nidx] == 0:
                    return True
        return False

    def _cluster_frontiers(self, frontiers):
        frontier_set = set(frontiers)
        clusters = []
        while frontier_set:
            start = frontier_set.pop()
            queue = [start]
            cluster = [start]
            while queue:
                cx, cy = queue.pop()
                for nx in range(cx - 1, cx + 2):
                    for ny in range(cy - 1, cy + 2):
                        candidate = (nx, ny)
                        if candidate in frontier_set:
                            frontier_set.remove(candidate)
                            queue.append(candidate)
                            cluster.append(candidate)
            clusters.append(cluster)
        return clusters

    def _find_nearest_free(self, data, width, height, start_x, start_y):
        visited = set()
        queue = deque()
        queue.append((start_x, start_y, 0))
        visited.add((start_x, start_y))
        while queue:
            x, y, dist = queue.popleft()
            if dist > self._search_radius:
                break
            if 0 <= x < width and 0 <= y < height:
                if data[y * width + x] == 0:
                    return x, y
                for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                    if (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny, dist + 1))
        return None

    def _pick_frontier_goal(self, robot_xy):
        if not self._map:
            return None
        info = self._map.info
        data = self._map.data
        width = info.width
        height = info.height
        if width == 0 or height == 0:
            return None

        frontiers = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if self._is_frontier(data, width, height, x, y):
                    frontiers.append((x, y))

        if not frontiers:
            return None

        clusters = self._cluster_frontiers(frontiers)
        best_goal = None
        best_dist = None
        rx, ry = robot_xy
        for cluster in clusters:
            if len(cluster) < self._min_frontier_size:
                continue
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            goal_cell = self._find_nearest_free(
                data, width, height, int(round(cx)), int(round(cy))
            )
            if not goal_cell:
                continue
            gx, gy = goal_cell
            wx = info.origin.position.x + (gx + 0.5) * info.resolution
            wy = info.origin.position.y + (gy + 0.5) * info.resolution
            dist = math.hypot(wx - rx, wy - ry)
            if dist < self._min_goal_distance:
                continue
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_goal = (wx, wy)
        return best_goal

    def _send_goal(self, goal_xy, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self._map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_xy[0]
        goal_msg.pose.pose.position.y = goal_xy[1]
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._goal_start_time = self.get_clock().now()
        self._last_goal = goal_xy
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected.')
            self._goal_handle = None
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_handle = None
        self._goal_start_time = None
        self.get_logger().info('Nav2 goal finished, searching next frontier.')

    def _tick(self):
        if self._map is None:
            return
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            return

        if self._goal_handle:
            if self._goal_start_time is not None:
                elapsed = (self.get_clock().now() - self._goal_start_time).nanoseconds * 1e-9
                if elapsed > self._goal_timeout:
                    self.get_logger().warn('Goal timeout, canceling.')
                    self._goal_handle.cancel_goal_async()
            return

        pose = self._get_robot_pose()
        if not pose:
            return
        rx, ry, yaw = pose
        goal_xy = self._pick_frontier_goal((rx, ry))
        if goal_xy is None:
            return
        if self._last_goal:
            if math.hypot(goal_xy[0] - self._last_goal[0], goal_xy[1] - self._last_goal[1]) < 0.2:
                return
        self._send_goal(goal_xy, yaw)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
