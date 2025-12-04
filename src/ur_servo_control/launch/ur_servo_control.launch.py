from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

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
    config_file = PathJoinSubstitution([FindPackageShare("ur_servo_control"), "config", LaunchConfiguration("config_file")])

    # Set use_sim_time parameter
    use_sim_time = SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time"))

    # Launch servo node
    launch_servo_node = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("ur_servo_control"),
                "launch",
                "servo.launch.py",
            ]
        ),
        launch_arguments={
            "ur_type": "ur5",
        }.items(),
    )

    # Switch controllers
    switch_to_servo_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "--activate",
            "forward_velocity_controller",
            "--deactivate",
            "scaled_joint_trajectory_controller",
        ],
        output="screen",
    )
    switch_back_to_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "--activate",
            "scaled_joint_trajectory_controller",
            "--deactivate",
            "forward_velocity_controller",
        ],
        output="screen",
    )

    # Start servo node
    trigger_servo_start = ExecuteProcess(
        cmd=[
            "ros2", "service", "call", 
            "/servo_node/start_servo", 
            "std_srvs/srv/Trigger", 
            "{}"
        ],
        output="screen"
    )

    # Spawn Node
    launch_ur_servo_control_node = Node(
        package="ur_servo_control",
        executable="main.py",
        name="ur_servo_control_node",
        output="screen",
        parameters=[config_file],
        arguments=["--ros-args", "--log-level", ["ur_servo_control_node:=", LaunchConfiguration("log_level")]],
    )

    # Event Handlers
    # NOTE: Since the ros launch will kill all child processes on exit, this handler may not work correctly.
    #       You may need to manually switch back controllers if this fails.
    event_handler_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=launch_ur_servo_control_node,
            on_exit=[
                switch_back_to_trajectory_controller,
            ],
        )
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(use_sim_time)
    ld.add_action(launch_servo_node)
    ld.add_action(switch_to_servo_controller)
    ld.add_action(trigger_servo_start)
    ld.add_action(launch_ur_servo_control_node)
    # ld.add_action(event_handler_on_exit)

    return ld
