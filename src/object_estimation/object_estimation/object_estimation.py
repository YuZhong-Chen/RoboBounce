# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import RGBD

# OpenCV
import cv2
from cv_bridge import CvBridge

# Other
import os
from ultralytics import YOLO


class ObjectEstimation(Node):
    def __init__(self):
        super().__init__("object_estimation_node")

        # Parameters
        self.declare_parameter("input_rgb_image_topic", "/input/rgb_image")
        self.declare_parameter("output_result_image_topic", "/output/result_image")
        self.declare_parameter("yolo_model_path", "/home/user/RoboBounce/models/yolo.pt")

        # Subscribers
        self.rgb_image_sub = self.create_subscription(Image, self.get_parameter("input_rgb_image_topic").value, self.rgb_image_callback, 10)

        # Publishers
        self.result_image_pub = self.create_publisher(Image, self.get_parameter("output_result_image_topic").value, 10)

        # CvBridge
        self.bridge = CvBridge()

        # Variables
        self.yolo_model = None

        self.load_yolo_model()

    def load_yolo_model(self):
        self.yolo_model = YOLO(self.get_parameter("yolo_model_path").value, "v8")
        self.get_logger().info("YOLO model loaded successfully.")

    def rgb_image_callback(self, msg: Image):
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.yolo_model.predict(source=rgb_image, verbose=False, conf=0.5)
        result_image = results[0].plot()

        result_msg = self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
        result_msg.header = msg.header
        self.result_image_pub.publish(result_msg)
