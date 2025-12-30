# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from ur_msgs.msg import ToolDataMsg
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# ROS Services
from ur_msgs.srv import SetIO


class OnrobotRG2IOControl(Node):
    def __init__(self):
        super().__init__("onrobot_rg2_io_control_node")

        # Parameters
        self.declare_parameter("input_command_topic", "/rg2/command")
        self.declare_parameter("input_tool_data_topic", "/io_and_status_controller/tool_data")
        self.declare_parameter("output_joint_state_topic", "/rg2_joint_states")
        self.declare_parameter("simulation_mode", False)
        self.declare_parameter("force_mode", "high")  # high or low

        # Get Parameters
        self.simulation_mode = self.get_parameter("simulation_mode").value
        self.force_mode = self.get_parameter("force_mode").value

        # Publishers
        self.joint_state_publisher = self.create_publisher(JointState, self.get_parameter("output_joint_state_topic").value, 10)

        # Subscribers
        self.command_subscription = self.create_subscription(Float32, self.get_parameter("input_command_topic").value, self.command_callback, 10)
        self.tool_data_subscription = self.create_subscription(ToolDataMsg, self.get_parameter("input_tool_data_topic").value, self.tool_data_callback, 10)

        # Services
        self.set_io_client = self.create_client(SetIO, "/io_and_status_controller/set_io")
        while not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/io_and_status_controller/set_io service not available, waiting...")
        self.set_force_mode(self.force_mode)

        # Timers
        joint_states_frequency = 125.0  # Hz
        self.joint_states_timer = self.create_timer(1.0 / joint_states_frequency, self.publish_joint_states)

        # Variables
        self.MAX_GRIPPER_WIDTH = 10.1  # cm
        self.current_command = 1.0
        self.current_gripper_width = self.MAX_GRIPPER_WIDTH  # cm

    def command_callback(self, msg: Float32):
        request = SetIO.Request()
        request.fun = 1  # Set digital output
        request.pin = 16  # Pin 16 controls the RG2 gripper
        request.state = 0.0 if msg.data >= 0.5 else 1.0  # Open if command >= 0.5, else close
        self.set_io_client.call_async(request)

    def set_force_mode(self, mode: str):
        request = SetIO.Request()
        request.fun = 1  # Set digital output
        request.pin = 17  # Pin 17 controls the force mode
        if mode == "high":
            request.state = 0.0  # High force mode
        else:
            request.state = 1.0  # Low force mode
        self.set_io_client.call_async(request)

    def tool_data_callback(self, msg: ToolDataMsg):
        # Convert the voltage reading to gripper width.
        # NOTE: Using a polynomial fit for the gripper data to achieve higher accuracy.
        #       The official linear mapping provided in the manual (shown below) is less precise:
        #           width = (voltage / UR_VOLTAGE_MAX) * 11.0  # units: cm
        #       UR_VOLTAGE_MAX varies by configuration: 3.7V (for 0-5V range) or 3.0V (for 0-10V range).
        voltage = msg.analog_input2
        width = -0.8 * (voltage**2) + 6.6 * voltage - 1.6  # cm
        width = max(0.0, min(width, self.MAX_GRIPPER_WIDTH))
        self.current_gripper_width = width

    def cm_to_rad(self, width):
        # NOTE: We map the gripper width to the joint angle linearly, this may not be accurate.
        min_rad = -0.45
        max_rad = 1.0
        width = max(0.0, min(width, self.MAX_GRIPPER_WIDTH))
        return (width / self.MAX_GRIPPER_WIDTH) * (max_rad - min_rad) + min_rad

    def publish_joint_states(self):
        if self.simulation_mode:
            self.current_gripper_width = self.current_command * self.MAX_GRIPPER_WIDTH
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["rg2_gripper_joint"]
        joint_state_msg.position = [self.cm_to_rad(self.current_gripper_width)]
        self.joint_state_publisher.publish(joint_state_msg)
