# 🏓 RoboBounce: Real-Time Table Tennis Juggling with UR5

> NTU - Robotics Final Project 2025

🌟 An end-to-end pipeline for table tennis juggling that leverages real-time 3D trajectory tracking and precise robotic striking with a UR5 manipulator.

## 🔥 Features

- **Real-Time 3D Perception:** 
    - Integrates object detection with depth estimation to map the ball's 3D position in real-time.
- **Precision Hand-Eye Calibration:** 
    - Utilizes ArUco markers to establish a precise transformation between the RealSense camera and the UR5 robot arm base.
- **Dynamic Trajectory Prediction:** 
    - Tracks the ball’s movement to predict the optimal hitting point for effective juggling.
- **State Machine-Driven Control:** 
    - Employs a robust state machine to guide striking actions at specific heights, helping to overcome system latency and acceleration limits.
- **Workspace Boundary Protection:** 
    - Includes built-in safety constraints to ensure all UR5 movements remain securely over the table area.
- **Integrated Motion Planning:** 
    - Combines MoveIt motion planning with real-time visual feedback for high-precision robot control.

## 🎥 Demo Video

The video has been uploaded to [YouTube](https://youtu.be/l4kwCzyMjKc).  
Click the image below to watch the full demo!

<div align="center">
    <a href="https://youtu.be/l4kwCzyMjKc" target="_blank">
        <img src="https://img.youtube.com/vi/l4kwCzyMjKc/maxresdefault.jpg" alt="RoboBounce: Real-Time Table Tennis Juggling with UR5" width="70%"/>
    </a>
</div>

> Remember to turn on the sound and the CC for the best experience.

## ⚠️ Prerequisites

- Laptop/PC with NVIDIA GPU (for inference acceleration)
- Docker
- UR5 (CB3 version)
- Realsense D435i
- Table tennis / Racket

## 🛠️ Installation

We provide a Dockerfile to facilitate the setup.  
Please follow the steps below to get started:

```bash
git clone https://github.com/YuZhong-Chen/RoboBounce.git
cd RoboBounce/docker
docker compose build
```

## ⚡ Quick Start

Please start the docker container and attach to it:

```bash
docker compose up -d
docker compose exec robo-bounce-ws bash
```

### Hand-eye Calibration

```bash
# Change the `IS_CALIBRATION` variable to true
vim ./scripts/realsense_bringup.sh

# Run the scripts
./scripts/realsense_bringup.sh
./scripts/extract_ur_calibration.sh
./scripts/ur_driver_bringup.sh

# Launch the calibration node
ros2 launch easy_handeye2 aruco_ros.launch.py
ros2 launch easy_handeye2 calibrate.launch.py

# Verify the calibration result
ros2 launch easy_handeye2 publish.launch.py
ros2 launch easy_handeye2 evaluate.launch.py
```

### RoboBounce Juggling

```bash
# Change the `IS_CALIBRATION` variable to false
vim ./scripts/realsense_bringup.sh

# Run the scripts
./scripts/realsense_bringup.sh
./scripts/ur_driver_bringup.sh

# Launch the nodes
ros2 launch easy_handeye2 publish.launch.py
ros2 launch ur_servo_control servo.launch.py
ros2 launch robo_bounce robo_bounce.launch.py
```
