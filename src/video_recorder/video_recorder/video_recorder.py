# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image

# OpenCV
import cv2
from cv_bridge import CvBridge

# Other
import os
import time
from datetime import datetime


class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder_node")

        # Parameters
        self.declare_parameter("input_camera_info_topic", "/input/camera_info")
        self.declare_parameter("input_rgb_image_topic", "/input/rgb_image")
        self.declare_parameter("record_length", 10.0)  # seconds
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("output_dir", "/home/user/RoboBounce/data")

        # Get Parameters
        self.record_length = self.get_parameter("record_length").value
        self.fps = self.get_parameter("fps").value
        self.output_dir = self.get_parameter("output_dir").value

        # Subscribers
        self.camera_info_sub = self.create_subscription(CameraInfo, self.get_parameter("input_camera_info_topic").value, self.CameraInfoCallback, 10)
        self.rgb_image_sub = self.create_subscription(Image, self.get_parameter("input_rgb_image_topic").value, self.RGBImageCallback, 10)

        # CvBridge
        self.bridge = CvBridge()

        # Variables
        self.writer = None
        self.is_recording = True
        self.start_time = None
        self.total_frames = 0

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

    def CameraInfoCallback(self, msg: CameraInfo):
        pass

    def RGBImageCallback(self, msg: Image):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.is_recording:
            if self.writer is None:
                self._init_writer(cv_image.shape)
                self.start_time = time.time()
                self.get_logger().info("Started recording video.")

            # Write frame
            self.writer.write(cv_image)
            self.total_frames += 1

            # Check for timeout
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= self.record_length:
                self._stop_recording()
                raise SystemExit

    def _init_writer(self, shape):
        height, width = shape[:2]

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}.mp4"
        filepath = os.path.join(self.output_dir, filename)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(filepath, fourcc, self.fps, (width, height))

        self.get_logger().info(f"Video: {filepath}, Record length: {self.record_length}s, FPS: {self.fps}")

    def _stop_recording(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
            self.get_logger().info("Stopped recording video.")
            self.get_logger().info(f"Total frames recorded: {self.total_frames}, FPS: {self.total_frames / self.record_length:.2f}")
        self.is_recording = False

    def __del__(self):
        if self.writer is not None:
            self.writer.release()
