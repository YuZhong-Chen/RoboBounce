#!/bin/bash -e

ros2 topic pub --once /rg2/command std_msgs/msg/Float32 "{data: 1.0}"
