#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
ORIGINAL_DIR="$(pwd)"

# Launch the realsense
ros2 launch realsense_launch realsense.launch.py
