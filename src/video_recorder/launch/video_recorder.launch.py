from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        "config_file",
        default_value="default.yaml",
        choices=["default.yaml"],
        description="The config file to use for the node",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time. (Gazebo)",
    ),
    DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="The ros logging level. Valid levels: debug, info, warn, error, fatal",
    ),
]


def generate_launch_description():
    # Config File
    config_file = PathJoinSubstitution([FindPackageShare("video_recorder"), "config", LaunchConfiguration("config_file")])

    # Set use_sim_time parameter
    use_sim_time = SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time"))

    # Spawn Node
    launch_video_recorder_node = Node(
        package="video_recorder",
        executable="main.py",
        name="video_recorder_node",
        output="screen",
        parameters=[config_file],
        arguments=["--ros-args", "--log-level", ["video_recorder_node:=", LaunchConfiguration("log_level")]],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(use_sim_time)
    ld.add_action(launch_video_recorder_node)

    return ld