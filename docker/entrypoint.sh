#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

cd /workspace/ros2_ws

colcon build --symlink-install --parallel-workers $(( $(nproc) / 2 ))
source /workspace/ros2_ws/install/setup.bash

if ! grep -qxF "#inspire_hand_teleop setup" ~/.bashrc; then
    cat <<'EOF' >> ~/.bashrc

#inspire_hand_teleop setup
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID=0
EOF
fi

exec bash
