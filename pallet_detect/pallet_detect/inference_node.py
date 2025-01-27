#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')

        # QoS to match the publisher's settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/robot1/zed2i/left/image_rect_color',
            self.image_callback,
            qos_profile
        )

        # Publisher
        self.publisher = self.create_publisher(Image, '/robot1/zed2i/detections', qos_profile)

        self.bridge = CvBridge()
        self.model_path = '/Users/virgoyal/ros2_ws/src/pallet_detect/models/yolov8n.onnx'
        self.session = ort.InferenceSession(self.model_path)
        self.get_logger().info("Inference node initialized with model: yolov8n.onnx")

    def image_callback(self, msg):
        self.get_logger().info("Received an image")
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detections = self.run_inference(frame)
        self.visualize_and_publish(frame, detections)

    def run_inference(self, frame):
        input_name = self.session.get_inputs()[0].name
        output_name = self.session.get_outputs()[0].name
        frame_resized = cv2.resize(frame, (640, 640))
        frame_preprocessed = frame_resized.transpose(2, 0, 1).astype('float32') / 255.0
        frame_preprocessed = np.expand_dims(frame_preprocessed, axis=0)
        detections = self.session.run([output_name], {input_name: frame_preprocessed})[0]
        return detections

    def visualize_and_publish(self, frame, detections):
        # Iterate over detections and draw bounding boxes
        for detection in detections:
            if detection.ndim == 1 and detection[4] > 0.5:  # Confidence threshold
                x1, y1, x2, y2 = map(int, detection[:4])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Convert the updated frame to an ROS2 Image message
        output_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        output_msg.header.frame_id = "camera_link"  # Use the child frame set in the static transform
        self.publisher.publish(output_msg)
        self.get_logger().info("Published detections")

def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
