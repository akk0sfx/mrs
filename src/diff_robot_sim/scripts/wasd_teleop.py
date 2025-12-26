#!/usr/bin/env python3
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class WASDTeleop(Node):
    """Simple WASD teleop for /cmd_vel with a deadman timeout."""

    def __init__(self):
        super().__init__('wasd_teleop')
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.9)
        self.declare_parameter('deadman_timeout', 0.25)
        self._linear = float(self.get_parameter('linear_speed').value)
        self._angular = float(self.get_parameter('angular_speed').value)
        self._timeout = float(self.get_parameter('deadman_timeout').value)
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._last_twist = Twist()
        self._last_key_time = 0.0
        self._timer = self.create_timer(0.05, self._on_timer)

        self.get_logger().info('WASD teleop: W/S forward/back, A/D rotate, Space stop, X exit.')

    def _get_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def _on_timer(self):
        key = self._get_key()
        if key:
            key = key.lower()
            twist = Twist()
            if key == 'w':
                twist.linear.x = self._linear
            elif key == 's':
                twist.linear.x = -self._linear
            elif key == 'a':
                twist.angular.z = self._angular
            elif key == 'd':
                twist.angular.z = -self._angular
            elif key == ' ':
                twist = Twist()
            elif key in ('x', '\x03'):
                self.get_logger().info('Exiting teleop.')
                rclpy.shutdown()
                return
            else:
                return

            self._last_twist = twist
            self._last_key_time = time.monotonic()
            self._pub.publish(twist)
            return

        if self._last_key_time and (time.monotonic() - self._last_key_time) > self._timeout:
            if self._last_twist.linear.x != 0.0 or self._last_twist.angular.z != 0.0:
                self._last_twist = Twist()
                self._pub.publish(self._last_twist)


def main():
    if not sys.stdin.isatty():
        print('WASD teleop requires a TTY. Run in a terminal window.')
        return

    rclpy.init()
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    node = WASDTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
