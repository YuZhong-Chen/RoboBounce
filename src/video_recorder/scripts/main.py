#!/usr/bin/env python3

import rclpy
from video_recorder.video_recorder import VideoRecorder


def main(args=None):
    rclpy.init(args=args)

    video_recorder = VideoRecorder()

    try:
        rclpy.spin(video_recorder)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if video_recorder.writer is not None:
            video_recorder._stop_recording()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
