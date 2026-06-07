#!/bin/bash
# Note: intentionally NOT 'set -e' — a build failure should drop the user into a
# shell to debug, not kill the container.

source /opt/ros/humble/setup.bash

cd /workspace/ros2_ws

colcon build --symlink-install --parallel-workers $(( $(nproc) / 2 )) \
    || echo "[entrypoint] colcon build failed — opening a shell so you can debug."
source /workspace/ros2_ws/install/setup.bash 2>/dev/null || true

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
