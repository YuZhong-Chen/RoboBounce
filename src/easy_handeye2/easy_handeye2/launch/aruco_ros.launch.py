from launch import LaunchDescription
from launch_ros.actions import Node

# Reference:
# https://github.com/pal-robotics/aruco_ros/blob/86a0bbb76c935095334ba31682f0246efb44e148/aruco_ros/launch/single.launch.py
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="aruco_ros",
                executable="single",
                name="aruco_single",
                parameters=[
                    {
                        "image_is_rectified": True,
                        "marker_size": 0.07,
                        "marker_id": 582,
                        "reference_frame": "camera_color_optical_frame",
                        "camera_frame": "camera_color_optical_frame",
                        "marker_frame": "aruco_marker_frame",
                    }
                ],
                remappings=[
                    ("/camera_info", "/camera/camera/color/camera_info"),
                    ("/image", "/camera/camera/color/image_raw"),
                ],
            )
        ]
    )
