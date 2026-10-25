#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

MODEL_DIR="$HOME/ros2_ws/src/my_robot_bringup/models"

for name in A B; do
    if gz model --list 2>/dev/null | grep -q "equipment_${name}"; then
        echo "equipment_${name} already exists. Skipping."
        continue
    fi

    if [ "$name" = "A" ]; then
        X=2.35
        Y=0.0
    else
        X=2.35
        Y=1.5
    fi

    ros2 run ros_gz_sim create \
        -world empty \
        -file "$MODEL_DIR/equipment_${name}.sdf" \
        -name "equipment_${name}" \
        -x "$X" -y "$Y" -z 0.0
done

echo "Equipment spawn commands completed."