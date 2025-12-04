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
import time
from datetime import datetime
import json


class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder_node")

        # Parameters
        self.declare_parameter("input_rgbd_image_topic", "/input/rgbd_image")
        self.declare_parameter("record_length", 10.0)
        self.declare_parameter("output_dir", "/home/user/RoboBounce/data")

        self.record_length = self.get_parameter("record_length").value
        self.output_base_dir = self.get_parameter("output_dir").value

        # Subscribers
        self.rgbd_image_sub = self.create_subscription(RGBD, self.get_parameter("input_rgbd_image_topic").value, self.RGBDImageCallback, 10)

        # CvBridge
        self.bridge = CvBridge()

        # Variables
        self.is_recording = True
        self.start_time = None
        self.total_frames_received = 0
        self.camera_info_saved = False
        self.frame_buffer = []

        # Setup Output Directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join(self.output_base_dir, timestamp)
        self.rgb_dir = os.path.join(self.output_dir, "rgb")
        self.depth_dir = os.path.join(self.output_dir, "depth")
        try:
            os.makedirs(self.rgb_dir, exist_ok=True)
            os.makedirs(self.depth_dir, exist_ok=True)
            self.get_logger().info(f"Created output dir: {self.output_dir}")
        except OSError as e:
            self.get_logger().error(f"Could not create directory: {e}")
            self.is_recording = False

    def RGBDImageCallback(self, msg: RGBD):
        if not self.is_recording:
            return

        if self.start_time is None:
            self.start_time = time.time()
            self._save_camera_info(msg)
            self.get_logger().info("Started recording ...")

        try:
            # Convert ROS messages to OpenCV images
            rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(msg.depth, desired_encoding="16UC1")

            # Store in RAM buffer directly
            self.frame_buffer.append((rgb_image, depth_image))
            self.total_frames_received += 1

            # Check for timeout
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= self.record_length:
                self.get_logger().info("Time limit reached. Stopping subscription...")
                self.is_recording = False  # Stop accepting new images
        except Exception as e:
            self.get_logger().error(f"Callback Error: {e}")

    def save_buffer_to_disk(self):
        self.get_logger().info(f"Starting to write {len(self.frame_buffer)} frames to disk...")

        total_saved = 0
        # Use simple compression for faster writing (Level 1)
        compression_params = [cv2.IMWRITE_PNG_COMPRESSION, 1]
        for i, (rgb_img, depth_img) in enumerate(self.frame_buffer):
            try:
                frame_index_str = str(i).zfill(6)
                rgb_path = os.path.join(self.rgb_dir, f"rgb_{frame_index_str}.png")
                depth_path = os.path.join(self.depth_dir, f"depth_{frame_index_str}.png")

                cv2.imwrite(rgb_path, rgb_img, compression_params)
                cv2.imwrite(depth_path, depth_img, compression_params)
                total_saved += 1

                if total_saved % 100 == 0:
                    self.get_logger().info(f"Saved {total_saved}/{len(self.frame_buffer)} frames...")
            except Exception as e:
                self.get_logger().error(f"Write Error at frame {i}: {e}")

        self.get_logger().info("Finished writing all frames.")
        self.get_logger().info(f"Total frames saved: {total_saved}")
        self.get_logger().info(f"Approximate Recording FPS: {total_saved / self.record_length:.2f}")

    def _save_camera_info(self, msg: RGBD):
        if self.camera_info_saved:
            return

        camera_info_filepath = os.path.join(self.output_dir, "camera_info.json")

        def info_to_dict(info):
            return {
                "width": info.width,
                "height": info.height,
                "distortion_model": info.distortion_model,
                "D": list(info.d),
                "K": list(info.k),
                "R": list(info.r),
                "P": list(info.p),
                "binning_x": info.binning_x,
                "binning_y": info.binning_y,
            }

        data = {
            "rgb": info_to_dict(msg.rgb_camera_info),
            "depth": info_to_dict(msg.depth_camera_info),
        }

        try:
            with open(camera_info_filepath, "w") as f:
                json.dump(data, f, indent=4)
            self.get_logger().info(f"Saved camera info to {camera_info_filepath}")
            self.camera_info_saved = True
        except IOError as e:
            self.get_logger().error(f"Failed to save camera info: {e}")
