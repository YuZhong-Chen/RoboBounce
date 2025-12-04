#!/usr/bin/env python3

import rclpy
import time
from ur_servo_control.ur_servo_control import URServoControl


def main(args=None):
    rclpy.init(args=args)

    ur_servo_control = URServoControl()

    try:
        rclpy.spin(ur_servo_control)
    except KeyboardInterrupt:
        # Publish stop command on shutdown
        ur_servo_control.get_logger().info("Shutdown UR servo control node...")
        ur_servo_control.publish_stop_command()
        time.sleep(0.1)
    finally:
        ur_servo_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
