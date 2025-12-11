#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
ORIGINAL_DIR="$(pwd)"

ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=192.168.56.101 \
    target_filename:="/home/user/RoboBounce/src/Universal_Robots_ROS2_Description/config/ur5/default_kinematics.yaml"