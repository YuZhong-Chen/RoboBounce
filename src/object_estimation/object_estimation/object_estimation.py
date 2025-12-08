# ROS
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import RGBD
from geometry_msgs.msg import TransformStamped, Vector3

# OpenCV
import cv2
from cv_bridge import CvBridge

# Other
import os
import numpy as np
from ultralytics import YOLO
from .filter import OneEuroFilter, KalmanFilterWrapper


class ObjectEstimation(Node):
    def __init__(self):
        super().__init__("object_estimation_node")

        # Parameters
        self.declare_parameter("input_rgbd_image_topic", "/input/rgbd_image")
        self.declare_parameter("output_result_image_topic", "/output/result_image")
        self.declare_parameter("output_ball_velocity_topic", "/output/ball_velocity")
        self.declare_parameter("yolo_model_path", "/home/user/RoboBounce/models/yolo.pt")
        self.declare_parameter("filter_mode", "raw")
        self.declare_parameter("fps", 60.0)

        # Get parameters
        self.filter_mode = self.get_parameter("filter_mode").value

        # Subscribers
        self.rgbd_image_sub = self.create_subscription(RGBD, self.get_parameter("input_rgbd_image_topic").value, self.rgbd_image_callback, 10)

        # Publishers
        self.result_image_pub = self.create_publisher(Image, self.get_parameter("output_result_image_topic").value, 10)
        self.vel_pub = self.create_publisher(Vector3, self.get_parameter("output_ball_velocity_topic").value, 10)

        # Other
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Variables
        self.yolo_model = None
        self.last_detection_time = None
        self.last_detection_center = None
        self.camera_intrinsics = None
        self.camera_frame_id = None
        self.last_time = None
        self.prev_ball_xyz = np.zeros(3)
        self.dt = 1.0 / self.get_parameter("fps").value
        self.DETECTION_TIMEOUT = 0.5  # seconds

        # Filter
        self.kf = KalmanFilterWrapper(dt=self.dt)
        self.one_euro_filters = None  # List [x, y, z]

        self.load_yolo_model()
        self.get_logger().info(f"Initialized with mode: {self.filter_mode}")

    def load_yolo_model(self):
        self.yolo_model = YOLO(self.get_parameter("yolo_model_path").value, "v8")
        self.get_logger().info("YOLO model loaded successfully.")

    def rgbd_image_callback(self, msg: RGBD):
        # Extract intrinsics from message if not yet available
        if self.camera_intrinsics is None:
            fx, fy = msg.rgb_camera_info.k[0], msg.rgb_camera_info.k[4]
            cx, cy = msg.rgb_camera_info.k[2], msg.rgb_camera_info.k[5]
            self.camera_intrinsics = {"fx": fx, "fy": fy, "cx": cx, "cy": cy}
            self.camera_frame_id = msg.rgb_camera_info.header.frame_id

        # Time delta calculation
        current_time = self.get_clock().now()
        dt = self.dt
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        now_sec = current_time.nanoseconds / 1e9

        # Convert images
        rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, desired_encoding="bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(msg.depth, desired_encoding="16UC1")

        # YOLO Inference
        results = self.yolo_model.predict(source=rgb_image, verbose=False, conf=0.5)

        # Process detections
        # NOTE: There should be only one detection, but we handle multiple just in case
        target_box = None
        current_center = None
        raw_xyz = None  # [x, y, z] in meters
        boxes = results[0].boxes
        if len(boxes) > 0:
            xywh = boxes.xywh.cpu().numpy()
            if self.last_detection_center is None:
                # Choose the largest box
                areas = xywh[:, 2] * xywh[:, 3]
                largest_idx = areas.argmax()
                target_box = boxes[largest_idx]
                current_center = (int(xywh[largest_idx, 0]), int(xywh[largest_idx, 1]))
            else:
                # Choose the box closest to the last detection
                distances = (xywh[:, 0] - self.last_detection_center[0]) ** 2 + (xywh[:, 1] - self.last_detection_center[1]) ** 2
                closest_idx = distances.argmin()
                target_box = boxes[closest_idx]
                current_center = (int(xywh[closest_idx, 0]), int(xywh[closest_idx, 1]))
            # Update last detection
            self.last_detection_center = current_center
            self.last_detection_time = self.get_clock().now()
            # Compute Raw 3D Position
            if target_box is not None:
                raw_xyz = self.compute_raw_xyz(target_box, depth_image)
        elif self.last_detection_time is not None:  # Handle detection timeout
            time_diff = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            if time_diff > self.DETECTION_TIMEOUT:
                self.last_detection_center = None
                self.last_detection_time = None
                self.one_euro_filters = None  # Reset filters
                self.kf.reset()

        # Kalman Filter Update
        kf_xyz = self.kf.update(raw_xyz, dt)

        # B. 1 Euro Filter Update
        oe_xyz = None
        if raw_xyz is not None:
            if self.one_euro_filters is None:
                # Initialize 1Euro Filters:
                # X (Left/Right): Standard
                # Y (Height): Needs speed/responsiveness -> High Beta
                # Z (Depth): Needs stability -> Low Beta, Low Cutoff
                config_xy = {"min_cutoff": 3.0, "beta": 0.7, "d_cutoff": 1.0}
                config_z = {"min_cutoff": 0.2, "beta": 0.05, "d_cutoff": 1.0}
                self.one_euro_filters = [
                    OneEuroFilter(now_sec, raw_xyz[0], **config_xy),
                    OneEuroFilter(now_sec, raw_xyz[1], **config_xy),
                    OneEuroFilter(now_sec, raw_xyz[2], **config_z),
                ]
            oe_xyz = np.array(
                [
                    self.one_euro_filters[0](now_sec, raw_xyz[0]),
                    self.one_euro_filters[1](now_sec, raw_xyz[1]),
                    self.one_euro_filters[2](now_sec, raw_xyz[2]),
                ]
            )

        final_xyz = None
        if self.filter_mode == "kalman":
            final_xyz = kf_xyz
        elif self.filter_mode == "one_euro":
            final_xyz = oe_xyz
        elif self.filter_mode == "raw":
            final_xyz = raw_xyz

        # Publish tf and vel Data
        if final_xyz is not None:
            # Publish TF
            if raw_xyz is not None:
                self.publish_tf_xyz(raw_xyz, "raw", msg.header.stamp)
            if kf_xyz is not None:
                self.publish_tf_xyz(kf_xyz, "kalman", msg.header.stamp)
            if oe_xyz is not None:
                self.publish_tf_xyz(oe_xyz, "one_euro", msg.header.stamp)

            # Calculate velocity (Finite difference)
            # This is important for the controller to know if the ball is falling
            vel_xyz = (final_xyz - self.prev_ball_xyz) / dt
            self.prev_ball_xyz = final_xyz

            v_msg = Vector3()
            v_msg.x = float(vel_xyz[0])
            v_msg.y = float(vel_xyz[1])
            v_msg.z = float(vel_xyz[2])
            self.vel_pub.publish(v_msg)

        # Visualization
        result_image = rgb_image.copy()

        # Draw markers
        def draw_marker(xyz, color, marker_type):
            if xyz is None or xyz[2] <= 0.0:
                return
            u = int(xyz[0] * self.camera_intrinsics["fx"] / xyz[2] + self.camera_intrinsics["cx"])
            v = int(xyz[1] * self.camera_intrinsics["fy"] / xyz[2] + self.camera_intrinsics["cy"])
            cv2.drawMarker(result_image, (u, v), color, marker_type, 20, 2)

        draw_marker(raw_xyz, (0, 255, 0), cv2.MARKER_SQUARE)
        draw_marker(kf_xyz, (0, 0, 255), cv2.MARKER_CROSS)
        draw_marker(oe_xyz, (255, 0, 0), cv2.MARKER_DIAMOND)

        # Draw Info at top-left
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 2
        start_x = 5
        start_y = 25
        line_height = 25

        def draw_text_line(label, xyz, color, line_idx):
            y = start_y + (line_idx * line_height)
            if xyz is not None:
                text = f"{label}: {xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}m"
            else:
                text = f"{label}: No Data"
            cv2.putText(result_image, text, (start_x, y), font, font_scale, color, thickness)

        # Display lines (No "Mode" display as requested)
        draw_text_line("Raw", raw_xyz, (0, 255, 0), 0)
        draw_text_line("KF", kf_xyz, (0, 0, 255), 1)
        draw_text_line("1E", oe_xyz, (255, 0, 0), 2)

        # Publish Image
        result_msg = self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
        result_msg.header = msg.header
        self.result_image_pub.publish(result_msg)

    def compute_raw_xyz(self, box, depth_image):
        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

        # Crop 25% margin
        w = x2 - x1
        h = y2 - y1
        crop_w = int(w * 0.25)
        crop_h = int(h * 0.25)

        if crop_w > 0 and crop_h > 0:
            roi_depth = depth_image[y1 + crop_h : y2 - crop_h, x1 + crop_w : x2 - crop_w]
        else:
            roi_depth = depth_image[y1:y2, x1:x2]

        valid_depths = roi_depth[roi_depth > 0]
        if valid_depths.size == 0:
            return None

        # Statistical filtering: Mean of closest 60%
        sorted_depths = np.sort(valid_depths)
        n_samples = int(valid_depths.size * 0.60)

        if n_samples > 0:
            z = np.mean(sorted_depths[:n_samples]) / 1000.0  # mm to meters
        else:
            z = np.mean(valid_depths) / 1000.0

        # 2D -> 3D Back projection
        cx_box = (x1 + x2) / 2
        cy_box = (y1 + y2) / 2
        X = (cx_box - self.camera_intrinsics["cx"]) * z / self.camera_intrinsics["fx"]
        Y = (cy_box - self.camera_intrinsics["cy"]) * z / self.camera_intrinsics["fy"]
        Z = z

        return np.array([X, Y, Z])

    def publish_tf_xyz(self, xyz, frame_id, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.camera_frame_id
        t.child_frame_id = frame_id
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
