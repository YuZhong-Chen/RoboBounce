#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
ORIGINAL_DIR="$(pwd)"

IS_CALIBRATION=false

# Launch the realsense
if [ "$IS_CALIBRATION" = false ] ; then
    ros2 launch realsense_launch realsense.launch.py
else
    ros2 launch realsense_launch realsense.launch.py \
        enable_depth:=false \
        rgb_profile:=1920x1080x8
fi
