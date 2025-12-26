#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()
        cv2.startWindowThread() # macOS

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image/image',
            self.image_callback,
            10
        )

        self.get_logger().info("CameraViewer started — waiting for images...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        cv2.imshow("Diff Robot Camera", cv_image)

        # macOS needs > 1ms delay
        if cv2.waitKey(5) & 0xFF == ord('q'):
            self.get_logger().info("Q pressed — shutting down.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()