#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    """Broadcast odom -> base_link TF from incoming nav_msgs/Odometry."""

    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.declare_parameter('odom_topic', '/odom')
        odom_topic = self.get_parameter('odom_topic').value
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self.get_logger().info(f'Publishing TF from odometry topic: {odom_topic}')

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or 'odom'
        t.child_frame_id = msg.child_frame_id or 'base_link'
        t.transform.translation = msg.pose.pose.position
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTFBroadcaster()
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
