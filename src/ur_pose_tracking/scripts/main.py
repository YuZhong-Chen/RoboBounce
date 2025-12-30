#!/usr/bin/env python3

import rclpy
import subprocess
from ur_pose_tracking.ur_pose_tracking import URPoseTracking


def main(args=None):
    rclpy.init(args=args)

    ur_pose_tracking = URPoseTracking()

    try:
        rclpy.spin(ur_pose_tracking)
    except KeyboardInterrupt:
        # Publish stop command on shutdown
        ur_pose_tracking.get_logger().info("Shutdown UR pose tracking node...")
        topic_name = "/forward_velocity_controller/commands"
        msg_type = "std_msgs/msg/Float64MultiArray"
        msg_content = "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
        cmd = ["ros2", "topic", "pub", "-t", "3", topic_name, msg_type, msg_content]
        subprocess.run(cmd)
    finally:
        ur_pose_tracking.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
