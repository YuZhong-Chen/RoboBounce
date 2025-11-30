from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from easy_handeye2.common_launch import (
    arg_calibration_type,
    arg_tracking_base_frame,
    arg_tracking_marker_frame,
    arg_robot_base_frame,
    arg_robot_effector_frame,
)

ARGUMENTS = [
    DeclareLaunchArgument(
        "name",
        default_value="eye_on_base_calibration",
        description="Name of the calibration.",
    ),
    DeclareLaunchArgument(
        "calibration_type",
        default_value="eye_on_base",
        choices=["eye_in_hand", "eye_on_base"],
        description="Type of calibration.",
    ),
    DeclareLaunchArgument(
        "tracking_base_frame",
        default_value="camera_link",
        description="TF frame of the tracking system base.",
    ),
    DeclareLaunchArgument(
        "tracking_marker_frame",
        default_value="aruco_marker_frame",
        description="TF frame of the tracking marker.",
    ),
    DeclareLaunchArgument(
        "robot_base_frame",
        default_value="base_link",
        description="TF frame of the robot base.",
    ),
    DeclareLaunchArgument(
        "robot_effector_frame",
        default_value="tool0",
        description="TF frame of the robot end-effector.",
    ),
]


def generate_launch_description():
    arg_name = DeclareLaunchArgument("name")

    node_dummy_calib_eih = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="dummy_publisher",
        condition=LaunchConfigurationEquals("calibration_type", "eye_in_hand"),
        arguments=f"--x 0 --y 0 --z 0.1 --qx 0 --qy 0 --qz 0 --qw 1".split(" ")
        + [
            "--frame-id",
            LaunchConfiguration("robot_effector_frame"),
            "--child-frame-id",
            LaunchConfiguration("tracking_base_frame"),
        ],
    )

    node_dummy_calib_eob = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="dummy_publisher",
        condition=LaunchConfigurationEquals("calibration_type", "eye_on_base"),
        arguments=f"--x 1 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1".split(" ")
        + [
            "--frame-id",
            LaunchConfiguration("robot_base_frame"),
            "--child-frame-id",
            LaunchConfiguration("tracking_base_frame"),
        ],
    )

    handeye_server = Node(
        package="easy_handeye2",
        executable="handeye_server",
        name="handeye_server",
        parameters=[
            {
                "name": LaunchConfiguration("name"),
                "calibration_type": LaunchConfiguration("calibration_type"),
                "tracking_base_frame": LaunchConfiguration("tracking_base_frame"),
                "tracking_marker_frame": LaunchConfiguration("tracking_marker_frame"),
                "robot_base_frame": LaunchConfiguration("robot_base_frame"),
                "robot_effector_frame": LaunchConfiguration("robot_effector_frame"),
            }
        ],
    )

    handeye_rqt_calibrator = Node(
        package="easy_handeye2",
        executable="rqt_calibrator.py",
        name="handeye_rqt_calibrator",
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {
                "name": LaunchConfiguration("name"),
                "calibration_type": LaunchConfiguration("calibration_type"),
                "tracking_base_frame": LaunchConfiguration("tracking_base_frame"),
                "tracking_marker_frame": LaunchConfiguration("tracking_marker_frame"),
                "robot_base_frame": LaunchConfiguration("robot_base_frame"),
                "robot_effector_frame": LaunchConfiguration("robot_effector_frame"),
            }
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(arg_name)
    ld.add_action(arg_calibration_type)
    ld.add_action(arg_tracking_base_frame)
    ld.add_action(arg_tracking_marker_frame)
    ld.add_action(arg_robot_base_frame)
    ld.add_action(arg_robot_effector_frame)
    ld.add_action(node_dummy_calib_eih)
    ld.add_action(node_dummy_calib_eob)
    ld.add_action(handeye_server)
    ld.add_action(handeye_rqt_calibrator)

    return ld
