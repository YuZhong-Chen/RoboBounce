#!/usr/bin/env python3

import rclpy
import subprocess
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from robo_bounce.robo_bounce import RoboBounce


def main(args=None):
    rclpy.init(args=args)

    robo_bounce_node = RoboBounce()

    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(robo_bounce_node)
        executor.spin()
    except KeyboardInterrupt:
        # Publish stop command on shutdown
        robo_bounce_node.get_logger().info("Shutdown RoboBounce node...")
        topic_name = "/forward_velocity_controller/commands"
        msg_type = "std_msgs/msg/Float64MultiArray"
        msg_content = "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
        cmd = ["ros2", "topic", "pub", "-t", "3", topic_name, msg_type, msg_content]
        subprocess.run(cmd)
    finally:
        robo_bounce_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
