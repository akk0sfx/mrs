#!/usr/bin/env python3
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class DiagnosticsCheck(Node):
    """Wait for /scan and /odom, then warn if required TF is missing."""

    def __init__(self):
        super().__init__('diagnostics_check')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(1.0, self._on_timer)
        self._topics_ready_logged = False
        self._warned_odom_tf = False
        self._warned_map_tf = False
        self._last_missing_log = 0.0

    def _on_timer(self):
        topics = {name for name, _types in self.get_topic_names_and_types()}
        missing = [name for name in ('/scan', '/odom') if name not in topics]

        if missing:
            now = time.monotonic()
            if now - self._last_missing_log > 5.0:
                self.get_logger().warn(
                    f"Waiting for topics: {', '.join(missing)}"
                )
                self._last_missing_log = now
            return

        if not self._topics_ready_logged:
            self.get_logger().info('Detected /scan and /odom topics.')
            self._topics_ready_logged = True

        self._check_tf('odom', 'base_link', 'odom->base_link', '_warned_odom_tf')
        self._check_tf('map', 'odom', 'map->odom', '_warned_map_tf')

    def _check_tf(self, target, source, label, warned_attr):
        ok = self._tf_buffer.can_transform(
            target, source, Time(), timeout=Duration(seconds=0.2)
        )
        warned = getattr(self, warned_attr)
        if ok and warned:
            self.get_logger().info(f'TF OK: {label}')
            setattr(self, warned_attr, False)
        elif not ok and not warned:
            self.get_logger().warn(
                f'TF missing: {label}. Check odometry or slam_toolbox.'
            )
            setattr(self, warned_attr, True)


def main():
    rclpy.init()
    node = DiagnosticsCheck()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
