from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        "name",
        default_value="eye_on_base_calibration",
        description="Name of the calibration.",
    ),
]


def generate_launch_description():
    arg_name = DeclareLaunchArgument("name")

    handeye_publisher = Node(
        package="easy_handeye2",
        executable="handeye_publisher",
        name="handeye_publisher",
        parameters=[
            {
                "name": LaunchConfiguration("name"),
            }
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(arg_name)
    ld.add_action(handeye_publisher)
    return ld
