from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        "initial_reset",
        default_value="false",
        description="Initial reset of the device.",
    ),
    DeclareLaunchArgument(
        "enable_pointcloud",
        default_value="false",
        description="Enable pointcloud.",
    ),
    DeclareLaunchArgument(
        "clip_distance",
        default_value="8.0",
        description="Clip distance.",
    ),
    DeclareLaunchArgument(
        "enable_hole_filling_filter",
        default_value="true",
        description="Enable hole filling filter.",
    ),
    DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch rviz.",
    ),
    DeclareLaunchArgument(
        "rgb_profile",
        default_value="424x240x60",
        choices=["640x480x30", "424x240x60"],
        description="RGB camera profile. Use 'ros2 param describe /camera/camera rgb_camera.color_profile' to see available options.",
    ),
    DeclareLaunchArgument(
        "depth_profile",
        default_value="480x270x60",
        choices=["640x480x30", "480x270x60"],
        description="Depth camera profile. Use 'ros2 param describe /camera/camera depth_module.depth_profile' to see available options.",
    ),
]


def generate_launch_description():
    # Launch realsense
    launch_realsense = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"],
        ),
        launch_arguments={
            "config_file": "''",
            "initial_reset": LaunchConfiguration("initial_reset"),
            "enable_rgbd": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_sync": "true",
            "enable_gyro": "false",
            "enable_accel": "false",
            "align_depth.enable": "true",
            "unite_imu_method": "0",
            "pointcloud.enable": LaunchConfiguration("enable_pointcloud"),
            "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
            "depth_module.infra_profile": LaunchConfiguration("depth_profile"),
            "depth_module.depth_format": "Z16",
            "rgb_camera.color_profile": LaunchConfiguration("rgb_profile"),
            "clip_distance": LaunchConfiguration("clip_distance"),
            "hole_filling_filter.enable": LaunchConfiguration("enable_hole_filling_filter"),
        }.items(),
    )

    # Launch rviz
    rviz_config_file = get_package_share_directory("realsense_launch") + "/rviz/realsense.rviz"
    launch_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_realsense)
    ld.add_action(launch_rviz)

    return ld
