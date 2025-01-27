#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        # Match QoS settings of the inference node publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match BEST_EFFORT
            durability=DurabilityPolicy.VOLATILE,       # Match VOLATILE
            depth=10                                    # Match depth
        )
        self.subscription = self.create_subscription(
            Image,
            '/robot1/zed2i/detections',
            self.image_callback,
            qos_profile
        )
        self.bridge = CvBridge()
        self.get_logger().info("Subscribed to /robot1/zed2i/detections with matching QoS")

    def image_callback(self, msg):
        self.get_logger().info("Received an image")
        print(f"Message header: {msg.header}")
        # Convert the ROS2 Image message to an OpenCV frame
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Display the image
        cv2.imshow('Detections', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
