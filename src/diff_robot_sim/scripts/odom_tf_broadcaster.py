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
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        odom_topic = self.get_parameter('odom_topic').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self.get_logger().info(
            f'Publishing TF {self._odom_frame} -> {self._base_frame} from: {odom_topic}'
        )

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            t.header.stamp = self.get_clock().now().to_msg()
        else:
            t.header.stamp = msg.header.stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
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
