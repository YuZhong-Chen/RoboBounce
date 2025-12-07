# ROS
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import RGBD
from geometry_msgs.msg import TransformStamped

# OpenCV
import cv2
from cv_bridge import CvBridge

# Other
import os
import numpy as np
from ultralytics import YOLO


class ObjectEstimation(Node):
    def __init__(self):
        super().__init__("object_estimation_node")

        # Parameters
        self.declare_parameter("input_rgbd_image_topic", "/input/rgbd_image")
        self.declare_parameter("output_result_image_topic", "/output/result_image")
        self.declare_parameter("yolo_model_path", "/home/user/RoboBounce/models/yolo.pt")

        # Subscribers
        self.rgbd_image_sub = self.create_subscription(RGBD, self.get_parameter("input_rgbd_image_topic").value, self.rgbd_image_callback, 10)

        # Publishers
        self.result_image_pub = self.create_publisher(Image, self.get_parameter("output_result_image_topic").value, 10)

        # Other
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Variables
        self.yolo_model = None
        self.last_detection_time = None
        self.last_detection_center = None
        self.camera_intrinsics = None
        self.camera_frame_id = None

        # Constants
        self.DETECTION_TIMEOUT = 0.5  # seconds

        self.load_yolo_model()

    def load_yolo_model(self):
        self.yolo_model = YOLO(self.get_parameter("yolo_model_path").value, "v8")
        self.get_logger().info("YOLO model loaded successfully.")

    def rgbd_image_callback(self, msg: RGBD):
        if self.camera_intrinsics is None:
            fx, fy = msg.rgb_camera_info.k[0], msg.rgb_camera_info.k[4]
            cx, cy = msg.rgb_camera_info.k[2], msg.rgb_camera_info.k[5]
            self.camera_intrinsics = {"fx": fx, "fy": fy, "cx": cx, "cy": cy}
            self.camera_frame_id = msg.header.frame_id

        # Convert images
        rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, desired_encoding="bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(msg.depth, desired_encoding="16UC1")
        result_image = rgb_image.copy()

        # YOLO Inference
        results = self.yolo_model.predict(source=rgb_image, verbose=False, conf=0.5)

        # Process detections
        # NOTE: There should be only one detection, but we handle multiple just in case
        target_box = None
        current_center = None
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
        if current_center is not None:
            self.last_detection_center = current_center
            self.last_detection_time = self.get_clock().now()
        elif self.last_detection_time is not None:
            time_diff = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            # self.get_logger().info(f"No detection. Time since last detection: {time_diff:.2f} seconds.")
            if time_diff > self.DETECTION_TIMEOUT:
                self.last_detection_center = None
                self.last_detection_time = None

        if target_box is not None:
            # Get object depth
            # NOTE: Crop 25% of the bounding box to avoid edges (Keep 50% from the center)
            x1, y1, x2, y2 = map(int, target_box.xyxy[0].cpu().numpy())
            crop_w = int((x2 - x1) * 0.25)
            crop_h = int((y2 - y1) * 0.25)
            if crop_w > 0 and crop_h > 0:
                roi_depth = depth_image[y1 + crop_h : y2 - crop_h, x1 + crop_w : x2 - crop_w]
            else:
                roi_depth = depth_image[y1:y2, x1:x2]  # Fallback if box is too small
            valid_depths = roi_depth[roi_depth > 0]
            if valid_depths.size > 0:
                sorted_depths = np.sort(valid_depths)
                n_samples = int(valid_depths.size * 0.60)  # Use the closest 60% of valid depths
                if n_samples > 0:
                    object_depth = np.mean(sorted_depths[:n_samples]) / 1000.0  # Convert mm to meters
                else:
                    object_depth = np.mean(valid_depths) / 1000.0
                # self.get_logger().info(f"Obj: x={current_center[0]}, y={current_center[1]}, depth={object_depth:.3f} m")

            # Publish TF
            self.publish_object_tf(current_center, object_depth, msg.header.stamp)

            # Draw results
            cv2.rectangle(result_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(result_image, f"Conf: {target_box.conf[0]:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        result_msg = self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
        result_msg.header = msg.header
        self.result_image_pub.publish(result_msg)

    def publish_object_tf(self, object_center, object_depth, timestamp):
        if self.camera_intrinsics is None:
            return

        # Compute 3D coordinates
        u, v = object_center
        X = (u - self.camera_intrinsics["cx"]) * object_depth / self.camera_intrinsics["fx"]
        Y = (v - self.camera_intrinsics["cy"]) * object_depth / self.camera_intrinsics["fy"]
        Z = object_depth

        # Create TransformStamped message
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.camera_frame_id
        t.child_frame_id = "object"
        t.transform.translation.x = float(X)
        t.transform.translation.y = float(Y)
        t.transform.translation.z = float(Z)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
