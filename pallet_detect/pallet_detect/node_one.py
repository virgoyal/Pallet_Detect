#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()

        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/robot1/zed2i/left/image_rect_color',
            self.image_callback,
            10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/robot1/zed2i/left/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher
        self.processed_image_pub = self.create_publisher(Image, '/processed_image', 10)

        self.get_logger().info("Camera Subscriber Node Started with YOLO integration.")

    def image_callback(self, msg):
        self.get_logger().info(f"Received image: width={msg.width}, height={msg.height}")
        try:
            # Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Perform object detection
            processed_image = self.perform_yolo_detection(cv_image)

            # Display the image with detections
            cv2.imshow("Processed Image", processed_image)
            cv2.waitKey(1)

            # Publish the processed image
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            self.processed_image_pub.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def perform_yolo_detection(self, image):
        """
        Run YOLO on the input image and return the image with bounding boxes overlaid.
        Replace this dummy implementation with actual YOLO integration.
        """
        # Example: Draw a dummy bounding box
        height, width, _ = image.shape
        start_point = (int(0.3 * width), int(0.3 * height))
        end_point = (int(0.7 * width), int(0.7 * height))
        color = (0, 255, 0)  # Green
        thickness = 2
        image = cv2.rectangle(image, start_point, end_point, color, thickness)
        cv2.putText(image, "Object", (start_point[0], start_point[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return image

    def camera_info_callback(self, msg):
        self.get_logger().info(f"Camera Info: K={msg.k}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
