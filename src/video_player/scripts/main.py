#!/usr/bin/env python3

import rclpy
from video_player.video_player import VideoPlayer


def main(args=None):
    rclpy.init(args=args)

    video_player = VideoPlayer()

    try:
        rclpy.spin(video_player)
    except SystemExit:
        pass
    finally:
        video_player.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
