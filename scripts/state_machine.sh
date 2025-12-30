#!/bin/bash

# Define task sequence
# Format for MOVE: "MOVE|TaskName|x|y|z|qx|qy|qz|qw"
# Format for GRIP: "GRIP|TaskName|value"
TASKS=(
    "MOVE|Initial_Pose|0.27|-0.11|0.45|-0.000|0.000|0.000|1.000"
    "GRIP|Open_Gripper|1.0"
    "MOVE|Target_Pose_1|0.58|-0.12|0.249|-0.012|0.451|0.015|0.892"
    "GRIP|Close_Gripper|0.0"
    "MOVE|Initial_Pose|0.27|-0.11|0.45|-0.000|0.000|0.000|1.000"
    "MOVE|Target_Pose_2|0.68|-0.32|0.332|-0.003|0.693|0.020|0.720"
    "GRIP|Open_Gripper|1.0"
    "MOVE|Initial_Pose|0.27|-0.11|0.45|-0.000|0.000|0.000|1.000"
)

echo "=== ROS 2 Task Controller Started ==="
echo "Total tasks in queue: ${#TASKS[@]}"

for i in "${!TASKS[@]}"; do
    # Parse the task type and data
    IFS='|' read -r TYPE NAME DATA1 DATA2 DATA3 DATA4 DATA5 DATA6 DATA7 <<< "${TASKS[$i]}"

    echo "----------------------------------------"
    echo "Progress: [$((i+1))/${#TASKS[@]}] -> [$TYPE] $NAME"

    # Debug Pause
    echo "Press [Enter] to execute this step, or [Ctrl+C] to abort..."
    read -r

    case $TYPE in
        "MOVE")
            ros2 topic pub -1 /target_pose geometry_msgs/msg/PoseStamped \
            "{
              header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
              pose: {
                position: {x: $DATA1, y: $DATA2, z: $DATA3},
                orientation: {x: $DATA4, y: $DATA5, z: $DATA6, w: $DATA7}
              }
            }"
            ;;

        "GRIP")
            ros2 topic pub --once /rg2/command std_msgs/msg/Float32 "{data: $DATA1}"
            ;;

        *)
            echo "!! Unknown Task Type: $TYPE"
            ;;
    esac

    echo "Step completed."
    sleep 0.5
done

echo "----------------------------------------"
echo "All tasks finished successfully."
