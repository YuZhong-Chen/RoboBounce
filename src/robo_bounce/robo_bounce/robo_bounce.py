# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from geometry_msgs.msg import TwistStamped

# TF
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener

# Other
from .pid_controller import PIDController


class RoboBounce(Node):
    def __init__(self):
        super().__init__("robo_bounce_node")

        # Parameters
        self.declare_parameter("output_servo_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("publish_frequency", 50.0)
        self.declare_parameter("racket_frame_id", "racket")
        self.declare_parameter("object_frame_id", "one_euro")
        self.declare_parameter("timer_delay", 0.05)
        self.declare_parameter("tf_timeout", 0.0)

        # PID parameters
        self.declare_parameter("pid_x_kp", 2.0)
        self.declare_parameter("pid_x_ki", 0.0)
        self.declare_parameter("pid_x_kd", 0.0)
        self.declare_parameter("pid_x_max_speed", 0.5)
        self.declare_parameter("pid_y_kp", 13.0)
        self.declare_parameter("pid_y_ki", 0.0)
        self.declare_parameter("pid_y_kd", 3.0)
        self.declare_parameter("pid_y_max_speed", 1.5)
        self.declare_parameter("pid_z_kp", 13.0)
        self.declare_parameter("pid_z_ki", 0.0)
        self.declare_parameter("pid_z_kd", 3.0)
        self.declare_parameter("pid_z_max_speed", 2.0)

        # Racket pose limits
        self.declare_parameter("racket_x_min", 0.45)
        self.declare_parameter("racket_x_max", 0.7)
        self.declare_parameter("racket_y_min", -0.5)
        self.declare_parameter("racket_y_max", 0.1)
        self.declare_parameter("racket_z_min", 0.4)
        self.declare_parameter("racket_z_max", 0.7)

        # Get parameter
        self.publish_frequency = self.get_parameter("publish_frequency").value
        self.racket_frame_id = self.get_parameter("racket_frame_id").value
        self.object_frame_id = self.get_parameter("object_frame_id").value
        self.timer_delay = self.get_parameter("timer_delay").value
        self.tf_timeout = self.get_parameter("tf_timeout").value
        self.racket_x_min = self.get_parameter("racket_x_min").value
        self.racket_x_max = self.get_parameter("racket_x_max").value
        self.racket_y_min = self.get_parameter("racket_y_min").value
        self.racket_y_max = self.get_parameter("racket_y_max").value
        self.racket_z_min = self.get_parameter("racket_z_min").value
        self.racket_z_max = self.get_parameter("racket_z_max").value

        # Publishers
        self.servo_pub = self.create_publisher(TwistStamped, self.get_parameter("output_servo_topic").value, 10)

        # Timers
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_servo_command_callback)

        # tf
        # NOTE: Use MultiThreadedExecutor in main.py rather than spin_thread=True in TransformListener.
        #       Although the TransformListener will create a Thread and run a SingleThreadedExecutor if spin_thread=True,
        #       this method is not suitable for the current architecture, as we include the TransformListener in this node.
        #       DO NOT CHANGE THIS CONFIGURATION. OTHERWISE, THE TF WILL NOT WORK PROPERLY.
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=1.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # PID Controllers
        self.pid_x = PIDController(self.get_parameter("pid_x_kp").value, self.get_parameter("pid_x_ki").value, self.get_parameter("pid_x_kd").value, self.get_parameter("pid_x_max_speed").value)
        self.pid_y = PIDController(self.get_parameter("pid_y_kp").value, self.get_parameter("pid_y_ki").value, self.get_parameter("pid_y_kd").value, self.get_parameter("pid_y_max_speed").value)
        self.pid_z = PIDController(self.get_parameter("pid_z_kp").value, self.get_parameter("pid_z_ki").value, self.get_parameter("pid_z_kd").value, self.get_parameter("pid_z_max_speed").value)

        # Variables
        self.racket_pose = None
        self.object_pose = None
        self.racket_target_x = None

    def publish_servo_command_callback(self):
        # Get current poses
        current_time = self.get_clock().now() - rclpy.duration.Duration(seconds=self.timer_delay)
        self.racket_pose = self.get_racket_pose(timestamp=current_time)
        self.object_pose = self.get_object_pose(timestamp=current_time)
        if self.racket_pose is not None and self.racket_target_x is None:
            self.racket_target_x = self.racket_pose.translation.x
        if self.racket_pose is None or self.object_pose is None:
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.publish_stop_command()
            return

        # Compute errors
        err_x = self.racket_target_x - self.racket_pose.translation.x
        err_y = self.object_pose.translation.y - self.racket_pose.translation.y
        err_z = self.object_pose.translation.z - self.racket_pose.translation.z

        # Apply offsets
        err_y -= 0.2

        # Update PID controllers
        # self.get_logger().info(f"Diff - x: {err_x:.3f}, y: {err_y:.3f}, z: {err_z:.3f}")
        current_time_nanosec = current_time.nanoseconds
        vx = self.pid_x.update(err_x, current_time_nanosec)
        vy = self.pid_y.update(err_y, current_time_nanosec)
        vz = self.pid_z.update(err_z, current_time_nanosec)

        # Create message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"

        # Set velocities
        twist_msg.twist.linear.x = float(vx)
        twist_msg.twist.linear.y = float(vy)
        twist_msg.twist.linear.z = float(vz)
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        # Apply racket pose limits
        if (self.racket_pose.translation.x >= self.racket_x_max and twist_msg.twist.linear.x > 0) or (self.racket_pose.translation.x <= self.racket_x_min and twist_msg.twist.linear.x < 0):
            twist_msg.twist.linear.x = 0.0
        if (self.racket_pose.translation.y >= self.racket_y_max and twist_msg.twist.linear.y > 0) or (self.racket_pose.translation.y <= self.racket_y_min and twist_msg.twist.linear.y < 0):
            twist_msg.twist.linear.y = 0.0
        if (self.racket_pose.translation.z >= self.racket_z_max and twist_msg.twist.linear.z > 0) or (self.racket_pose.translation.z <= self.racket_z_min and twist_msg.twist.linear.z < 0):
            twist_msg.twist.linear.z = 0.0

        # Publish the message
        self.get_logger().info(f"Cmd  - vx: {twist_msg.twist.linear.x:.3f}, vy: {twist_msg.twist.linear.y:.3f}, vz: {twist_msg.twist.linear.z:.3f}")
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

    def get_racket_pose(self, timestamp=None):
        racket_pose = None
        try:
            # Lookup transform from "world" to "racket"
            time = rclpy.time.Time() if timestamp is None else timestamp
            transform = self.tf_buffer.lookup_transform("world", self.racket_frame_id, time, timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            racket_pose = transform.transform
        except Exception as e:
            # self.get_logger().warn(f"Could not get racket pose: {e}")
            racket_pose = None
        return racket_pose

    def get_object_pose(self, timestamp=None):
        object_pose = None
        try:
            # Lookup transform from "world" to "object"
            time = rclpy.time.Time() if timestamp is None else timestamp
            transform = self.tf_buffer.lookup_transform("world", self.object_frame_id, time, timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            object_pose = transform.transform
        except Exception as e:
            # self.get_logger().warn(f"Could not get object pose: {e}")
            object_pose = None
        return object_pose
