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


class State:
    IDLE = 0
    STRIKE = 1
    RECOVER = 2


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
        self.declare_parameter("pid_x_kp", 8.0)
        self.declare_parameter("pid_x_ki", 0.0)
        self.declare_parameter("pid_x_kd", 1.0)
        self.declare_parameter("pid_x_max_speed", 1.0)
        self.declare_parameter("pid_y_kp", 15.0)
        self.declare_parameter("pid_y_ki", 0.0)
        self.declare_parameter("pid_y_kd", 3.0)
        self.declare_parameter("pid_y_max_speed", 2.5)
        self.declare_parameter("pid_z_kp", 13.0)
        self.declare_parameter("pid_z_ki", 0.0)
        self.declare_parameter("pid_z_kd", 3.0)
        self.declare_parameter("pid_z_max_speed", 2.0)
        self.declare_parameter("pid_roll_kp", 20.0)
        self.declare_parameter("pid_roll_ki", 0.0)
        self.declare_parameter("pid_roll_kd", 5.0)
        self.declare_parameter("pid_roll_max_speed", 6.0)
        self.declare_parameter("pid_pitch_kp", 10.0)
        self.declare_parameter("pid_pitch_ki", 0.0)
        self.declare_parameter("pid_pitch_kd", 0.0)
        self.declare_parameter("pid_pitch_max_speed", 2.0)
        self.declare_parameter("pid_yaw_kp", 10.0)
        self.declare_parameter("pid_yaw_ki", 0.0)
        self.declare_parameter("pid_yaw_kd", 0.0)
        self.declare_parameter("pid_yaw_max_speed", 2.0)

        # Racket pose limits
        self.declare_parameter("racket_x_min", 0.45)
        self.declare_parameter("racket_x_max", 0.7)
        self.declare_parameter("racket_y_min", -0.25)
        self.declare_parameter("racket_y_max", 0.05)
        self.declare_parameter("racket_z_min", 0.3)
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
        self.pid_roll = PIDController(self.get_parameter("pid_roll_kp").value, self.get_parameter("pid_roll_ki").value, self.get_parameter("pid_roll_kd").value, self.get_parameter("pid_roll_max_speed").value)
        self.pid_pitch = PIDController(self.get_parameter("pid_pitch_kp").value, self.get_parameter("pid_pitch_ki").value, self.get_parameter("pid_pitch_kd").value, self.get_parameter("pid_pitch_max_speed").value)
        self.pid_yaw = PIDController(self.get_parameter("pid_yaw_kp").value, self.get_parameter("pid_yaw_ki").value, self.get_parameter("pid_yaw_kd").value, self.get_parameter("pid_yaw_max_speed").value)

        # State Machine
        self.state = State.IDLE
        self.idle_z_height = 0.35
        self.strike_z_target_height = 0.90  # Ball striking height for triggering strike
        self.strike_z_target_vel = -0.3  # Ball vertical velocity for triggering strike
        self.strike_z_height = 0.42  # Racket height when striking
        self.strike_z_vel = 1.2  # Racket vertical velocity when striking

        # Variables
        self.racket_pose = None
        self.object_pose = None
        self.racket_target_x = 0.58
        self.racket_target_y = None
        self.racket_target_z = None
        self.racket_target_roll = 0.03
        self.racket_target_pitch = 0.0
        self.racket_target_yaw = 0.0
        self.prev_object_pose = None
        self.prev_time_nanosec = None
        self.prev_racket_pose = None
        self.prev_racket_pose_time_nanosec = None

    def publish_servo_command_callback(self):
        # Get current time with delay
        current_time = self.get_clock().now() - rclpy.duration.Duration(seconds=self.timer_delay)
        current_time_nanosec = current_time.nanoseconds

        # Get racket pose
        self.racket_pose = self.get_racket_pose(timestamp=current_time)
        if self.racket_pose is None:
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_roll.reset()
            self.pid_pitch.reset()
            self.pid_yaw.reset()
            # self.publish_stop_command()
            self.prev_object_pose = None
            self.prev_racket_pose_time_nanosec = None
            return
        racket_vel = [0.0, 0.0, 0.0]
        if self.prev_racket_pose is not None and self.prev_racket_pose_time_nanosec is not None:
            dt = (current_time_nanosec - self.prev_racket_pose_time_nanosec) / 1e9
            if dt > 0.001:
                racket_vel[0] = (self.racket_pose.translation.x - self.prev_racket_pose.translation.x) / dt
                racket_vel[1] = (self.racket_pose.translation.y - self.prev_racket_pose.translation.y) / dt
                racket_vel[2] = (self.racket_pose.translation.z - self.prev_racket_pose.translation.z) / dt
        self.prev_racket_pose = self.racket_pose
        self.prev_racket_pose_time_nanosec = current_time_nanosec

        # Get object pose
        self.object_pose = self.get_object_pose(timestamp=current_time)
        if self.object_pose is not None:
            # Update ball velocity
            ball_vel = [0.0, 0.0, 0.0]
            if self.prev_object_pose is not None and self.prev_time_nanosec is not None:
                dt = (current_time_nanosec - self.prev_time_nanosec) / 1e9
                if dt > 0.001:
                    ball_vel[0] = (self.object_pose.translation.x - self.prev_object_pose.translation.x) / dt
                    ball_vel[1] = (self.object_pose.translation.y - self.prev_object_pose.translation.y) / dt
                    ball_vel[2] = (self.object_pose.translation.z - self.prev_object_pose.translation.z) / dt
            self.prev_object_pose = self.object_pose
            self.prev_time_nanosec = current_time_nanosec
            # self.get_logger().info(f"Ball Velocity - vz: {ball_vel[2]:.2f}")
        else:
            ball_vel = [0.0, 0.0, 0.0]
            self.prev_object_pose = None
            self.prev_time_nanosec = None

        # State Machine
        if self.state == State.IDLE:
            if self.object_pose is None:
                self.racket_target_y = self.racket_pose.translation.y
                self.racket_target_z = self.idle_z_height
            elif ball_vel[2] < self.strike_z_target_vel and self.object_pose.translation.z <= self.strike_z_target_height:
                # self.get_logger().info("STRIKE")
                self.state = State.STRIKE
                self.racket_target_y = self.racket_pose.translation.y  # Keep current y position when striking
                self.racket_target_z = self.strike_z_height
            else:
                self.racket_target_y = self.object_pose.translation.y
                self.racket_target_z = self.idle_z_height
        elif self.state == State.STRIKE:
            if self.racket_pose.translation.z >= self.strike_z_height - 0.03 or (self.object_pose is not None and ball_vel[2] > 0.6):
                # self.get_logger().info("RECOVER")
                self.state = State.RECOVER
                self.racket_target_z = self.idle_z_height
            else:
                # self.racket_target_y = self.racket_pose.translation.y  # Keep current y position when striking
                self.racket_target_y = self.object_pose.translation.y
                self.racket_target_z = self.strike_z_height
        elif self.state == State.RECOVER:
            if ball_vel[2] < self.strike_z_target_vel and self.object_pose.translation.z <= self.strike_z_target_height:
                # self.get_logger().info("STRIKE")
                self.state = State.STRIKE
                self.racket_target_y = self.racket_pose.translation.y  # Keep current y position when striking
                self.racket_target_z = self.strike_z_height
            if self.racket_pose.translation.z <= self.idle_z_height + 0.03:
                # self.get_logger().info("IDLE")
                self.state = State.IDLE
            if self.object_pose is not None:
                self.racket_target_y = self.object_pose.translation.y
            else:
                self.racket_target_y = self.racket_pose.translation.y
            self.racket_target_z = self.idle_z_height
        # self.get_logger().info(f"State: {self.state}")

        # Compute errors
        err_x = self.racket_target_x - self.racket_pose.translation.x
        err_y = self.racket_target_y - self.racket_pose.translation.y
        err_z = self.racket_target_z - self.racket_pose.translation.z
        err_roll = self.racket_target_roll - self.racket_pose.rotation.x
        err_pitch = self.racket_target_pitch - self.racket_pose.rotation.y
        err_yaw = self.racket_target_yaw - self.racket_pose.rotation.z

        # Update PID controllers
        # self.get_logger().info(f"Diff - x: {err_x:.3f}, y: {err_y:.3f}, z: {err_z:.3f}")
        vx = self.pid_x.update(err_x, current_time_nanosec)
        vy = self.pid_y.update(err_y, current_time_nanosec)
        vz = self.pid_z.update(err_z, current_time_nanosec)
        v_roll = self.pid_roll.update(err_roll, current_time_nanosec)
        v_pitch = self.pid_pitch.update(err_pitch, current_time_nanosec)
        v_yaw = self.pid_yaw.update(err_yaw, current_time_nanosec)

        if self.state == State.STRIKE:
            # vx = vy = v_roll = v_pitch = v_yaw = 0.0
            vz = self.strike_z_vel
        elif self.state == State.RECOVER:
            vz = -self.strike_z_vel

        # Create message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"

        # Set velocities
        twist_msg.twist.linear.x = float(vx)
        twist_msg.twist.linear.y = float(vy)
        twist_msg.twist.linear.z = float(vz)
        twist_msg.twist.angular.x = float(v_roll)
        twist_msg.twist.angular.y = float(v_pitch)
        twist_msg.twist.angular.z = float(v_yaw)

        # Apply racket pose limits
        if (self.racket_pose.translation.x >= self.racket_x_max and twist_msg.twist.linear.x > 0) or (self.racket_pose.translation.x <= self.racket_x_min and twist_msg.twist.linear.x < 0):
            twist_msg.twist.linear.x = 0.0
        if (self.racket_pose.translation.y >= self.racket_y_max and twist_msg.twist.linear.y > 0) or (self.racket_pose.translation.y <= self.racket_y_min and twist_msg.twist.linear.y < 0):
            twist_msg.twist.linear.y = 0.0
        if (self.racket_pose.translation.z >= self.racket_z_max and twist_msg.twist.linear.z > 0) or (self.racket_pose.translation.z <= self.racket_z_min and twist_msg.twist.linear.z < 0):
            twist_msg.twist.linear.z = 0.0

        # Publish the message
        # self.get_logger().info(f"cmd vy: {twist_msg.twist.linear.y:+3.2f} racket vy: {racket_vel[1]:+3.2f}")
        # self.get_logger().info(f"cmd vz: {twist_msg.twist.linear.z:+3.2f} racket vz: {racket_vel[2]:+3.2f}")
        # self.get_logger().info(f"Linear  - vx: {twist_msg.twist.linear.x:.3f}, vy: {twist_msg.twist.linear.y:.3f}, vz: {twist_msg.twist.linear.z:.3f}")
        # self.get_logger().info(f"Angular - vx: {twist_msg.twist.angular.x:.3f}, vy: {twist_msg.twist.angular.y:.3f}, vz: {twist_msg.twist.angular.z:.3f}")
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
            # NOTE: Use the latest available transform
            # TODO: Add timeout, check the transform header stamp
            time = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform("world", self.racket_frame_id, time, timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            racket_pose = transform.transform
        except Exception as e:
            self.get_logger().warn(f"Could not get racket pose: {e}")
            racket_pose = None
        return racket_pose

    def get_object_pose(self, timestamp=None):
        object_pose = None
        try:
            # Lookup transform from "world" to "object"
            time = rclpy.time.Time()
            # time = rclpy.time.Time() if timestamp is None else timestamp
            transform = self.tf_buffer.lookup_transform("world", self.object_frame_id, time, timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            object_pose = transform.transform
        except Exception as e:
            self.get_logger().warn(f"Could not get object pose: {e}")
            object_pose = None
        return object_pose
