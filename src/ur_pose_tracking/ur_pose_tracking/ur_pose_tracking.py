# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from geometry_msgs.msg import TwistStamped


class URPoseTracking(Node):
    def __init__(self):
        super().__init__("ur_pose_tracking_node")

        # Parameters
        self.declare_parameter("output_servo_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("publish_frequency", 50.0)

        # Get parameter
        self.publish_frequency = self.get_parameter("publish_frequency").value

        # Publishers
        self.servo_pub = self.create_publisher(TwistStamped, self.get_parameter("output_servo_topic").value, 10)

        # Timers
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_servo_command_callback)

    def publish_servo_command_callback(self):
        # Create message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"

        # Set velocities
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        # Publish the message
        self.servo_pub.publish(twist_msg)

    def publish_stop_command(self):
        # Create message
        stop_msg = TwistStamped()
        stop_msg.header.frame_id = "world"
        stop_msg.header.stamp = self.get_clock().now().to_msg()

        # Set all velocities to zero
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.linear.y = 0.0
        stop_msg.twist.linear.z = 0.0
        stop_msg.twist.angular.x = 0.0
        stop_msg.twist.angular.y = 0.0
        stop_msg.twist.angular.z = 0.0

        # Publish the stop message
        self.servo_pub.publish(stop_msg)
