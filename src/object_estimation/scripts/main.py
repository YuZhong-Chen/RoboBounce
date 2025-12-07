#!/usr/bin/env python3

import rclpy
from object_estimation.object_estimation import ObjectEstimation


def main(args=None):
    rclpy.init(args=args)

    object_estimation = ObjectEstimation()

    rclpy.spin(object_estimation)

    object_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
