# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo

# OpenCV
import cv2
from cv_bridge import CvBridge

# Other
import os
import json


class VideoPlayer(Node):
    def __init__(self):
        super().__init__("video_player_node")

        # Parameters
        self.declare_parameter("output_rgb_image_topic", "/output/rgb_image")
        self.declare_parameter("output_depth_image_topic", "/output/depth_image")
        self.declare_parameter("output_camera_info_topic", "/output/camera_info")
        self.declare_parameter("video_fps", 60.0)
        self.declare_parameter("video_repeat", True)
        self.declare_parameter("video_path", "/home/user/RoboBounce/data")

        # Get Parameters
        self.video_fps = self.get_parameter("video_fps").value
        self.video_repeat = self.get_parameter("video_repeat").value
        self.video_path = self.get_parameter("video_path").value

        # Publishers
        self.rgb_publisher = self.create_publisher(Image, self.get_parameter("output_rgb_image_topic").value, 10)
        self.depth_publisher = self.create_publisher(Image, self.get_parameter("output_depth_image_topic").value, 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, self.get_parameter("output_camera_info_topic").value, 10)

        # Timer
        self.image_timer = self.create_timer(1.0 / self.video_fps, self.publish_image_callback)

        # CvBridge
        self.bridge = CvBridge()

        # Variables
        self.current_frame_index = 0
        self.total_frames = 0
        self.rgb_images = []
        self.depth_images = []
        self.camera_info = None

        # Initialize
        self.load_video()
        self.get_logger().info("Start playing video...")

    def load_video(self) -> None:
        rgb_images_dir = os.path.join(self.video_path, "rgb")
        depth_images_dir = os.path.join(self.video_path, "depth")
        camera_info_path = os.path.join(self.video_path, "camera_info.json")

        self.total_frames = len(os.listdir(rgb_images_dir))

        # Load RGB images
        self.get_logger().info("Loading rgb images...")
        self.rgb_images = []
        for filename in sorted(os.listdir(rgb_images_dir)):
            rgb_image = cv2.imread(os.path.join(rgb_images_dir, filename))
            if rgb_image is not None:
                self.rgb_images.append(rgb_image)

        # Load Depth images
        self.get_logger().info("Loading depth images...")
        self.depth_images = []
        for filename in sorted(os.listdir(depth_images_dir)):
            depth_image = cv2.imread(os.path.join(depth_images_dir, filename), cv2.IMREAD_UNCHANGED)
            if depth_image is not None:
                self.depth_images.append(depth_image)

        # Load Camera Info
        # NOTE: We only load RGB camera info since depth camera is synced with RGB by default
        with open(camera_info_path, "r") as f:
            camera_info_dict = json.load(f)["rgb"]
            self.camera_info = CameraInfo()
            self.camera_info.width = camera_info_dict["width"]
            self.camera_info.height = camera_info_dict["height"]
            self.camera_info.distortion_model = camera_info_dict["distortion_model"]
            self.camera_info.d = camera_info_dict["D"]
            self.camera_info.k = camera_info_dict["K"]
            self.camera_info.r = camera_info_dict["R"]
            self.camera_info.p = camera_info_dict["P"]
            self.camera_info.binning_x = camera_info_dict["binning_x"]
            self.camera_info.binning_y = camera_info_dict["binning_y"]

        self.get_logger().info(f"Loaded {self.total_frames} frames from {self.video_path}")

    def publish_image_callback(self) -> None:
        if self.current_frame_index >= self.total_frames:
            if self.video_repeat:
                self.current_frame_index = 0
            else:
                self.get_logger().info("Finished playing video.")
                raise SystemExit

        # Publish RGB Image
        rgb_image_msg = self.bridge.cv2_to_imgmsg(self.rgb_images[self.current_frame_index], encoding="bgr8")
        rgb_image_msg.header.stamp = self.get_clock().now().to_msg()
        self.rgb_publisher.publish(rgb_image_msg)

        # Publish Depth Image
        depth_image_msg = self.bridge.cv2_to_imgmsg(self.depth_images[self.current_frame_index], encoding="16UC1")
        depth_image_msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_publisher.publish(depth_image_msg)

        # Publish Camera Info
        camera_info_msg = self.camera_info
        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_info_publisher.publish(camera_info_msg)

        self.current_frame_index += 1
