# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from ur_msgs.msg import ToolDataMsg
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class OnrobotRG2IOControl(Node):
    def __init__(self):
        super().__init__("onrobot_rg2_io_control_node")

        # Parameters
        self.declare_parameter("input_command_topic", "/rg2/command")
        self.declare_parameter("input_tool_data_topic", "/io_and_status_controller/tool_data")
        self.declare_parameter("output_joint_state_topic", "/rg2_joint_states")
        self.declare_parameter("simulation_mode", False)

        # Get Parameters
        self.simulation_mode = self.get_parameter("simulation_mode").value

        # Publishers
        self.joint_state_publisher = self.create_publisher(JointState, self.get_parameter("output_joint_state_topic").value, 10)

        # Subscribers
        self.command_subscription = self.create_subscription(Float32, self.get_parameter("input_command_topic").value, self.command_callback, 10)
        self.tool_data_subscription = self.create_subscription(ToolDataMsg, self.get_parameter("input_tool_data_topic").value, self.tool_data_callback, 10)

        # Timers
        if self.simulation_mode:
            joint_states_frequency = 125.0  # Hz
            self.joint_states_timer = self.create_timer(1.0 / joint_states_frequency, self.publish_joint_states)
        else:
            raise NotImplementedError("Real hardware mode is not implemented yet.")

        # Variables
        self.MAX_GRIPPER_WIDTH = 11.0  # cm
        self.current_command = 1.0

    def command_callback(self, msg: Float32):
        # TODO: Send the command to the "/io_and_status_controller/set_io" service.
        self.current_command = msg.data
        # self.get_logger().info(f"Command: {self.current_command}")

    def tool_data_callback(self, msg: ToolDataMsg):
        # TODO: Transform the voltage readings to the gripper state.
        # self.get_logger().info(f"analog_input2: {msg.analog_input2}")
        pass

    def cm_to_rad(self, width):
        # NOTE: We map the gripper width to the joint angle linearly, this may not be accurate.
        min_rad = -0.45
        max_rad = 1.0
        width = max(0.0, min(width, self.MAX_GRIPPER_WIDTH))
        return (width / self.MAX_GRIPPER_WIDTH) * (max_rad - min_rad) + min_rad

    def publish_joint_states(self, current_gripper_width: float = 0.0):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["rg2_gripper_joint"]
        joint_state_msg.position = [self.cm_to_rad(self.current_command * self.MAX_GRIPPER_WIDTH if self.simulation_mode else current_gripper_width)]
        self.joint_state_publisher.publish(joint_state_msg)
