#!/usr/bin/env python3

import rclpy
from video_recorder.video_recorder import VideoRecorder


def main(args=None):
    rclpy.init(args=args)

    video_recorder = VideoRecorder()

    try:
        while rclpy.ok():
            rclpy.spin_once(video_recorder, timeout_sec=0.001)
            if not video_recorder.is_recording:
                break
    except KeyboardInterrupt:
        video_recorder.get_logger().info("Recording interrupted by user.")
        video_recorder.is_recording = False
        
    finally:
        # Save all data stored in RAM to disk
        video_recorder.save_buffer_to_disk()
        video_recorder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
