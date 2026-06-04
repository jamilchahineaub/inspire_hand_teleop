# Hand teleop shortcuts — source from ~/.bashrc on the LAPTOP:
#   echo 'source ~/CAIR/teleop_aliases.sh' >> ~/.bashrc
#
# Full reference: ~/CAIR/TELEOP_COMMANDS.md
# In-container actions go through `docker exec`, so they work from any host shell.

export TELEOP_DIR="$HOME/CAIR/inspire_hand_teleop"
export TELEOP_COMPOSE="$TELEOP_DIR/docker/docker-compose.yaml"
export TELEOP_CONTAINER="inspire_hand_teleop"
export MUJOCO_DIR="$HOME/CAIR/hand_mujoco"

# Run a command inside the perception container with ROS sourced.
_teleop_exec() {
  docker exec -it "$TELEOP_CONTAINER" bash -lc \
    "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && $*"
}

# ── Container lifecycle ────────────────────────────────────────────────────
teleop-up()    { xhost +local:docker >/dev/null 2>&1; docker compose -f "$TELEOP_COMPOSE" up -d; }
teleop-down()  { docker compose -f "$TELEOP_COMPOSE" down; }
teleop-shell() { docker exec -it "$TELEOP_CONTAINER" bash; }

# ── Build ──────────────────────────────────────────────────────────────────
teleop-build() {
  _teleop_exec "cd /workspace/ros2_ws && colcon build --symlink-install \
    --packages-select hand_perception hand_perception_msgs && echo BUILD_OK"
}
teleop-rebuild() {                       # clean image rebuild (Dockerfile/dep changes)
  docker compose -f "$TELEOP_COMPOSE" down
  docker compose -f "$TELEOP_COMPOSE" build --no-cache
  docker compose -f "$TELEOP_COMPOSE" up -d
}

# ── Run nodes ──────────────────────────────────────────────────────────────
teleop-perception() {                    # MediaPipe on the compressed ZED stream
  _teleop_exec "ros2 launch hand_perception hand_perception.launch.xml \
    image_topic:=/image/compressed"
}
teleop-real() {                          # send to real RH56 over RS485
  _teleop_exec "ros2 launch hand_perception rs485_hand.launch.xml ${1:+serial_port:=$1}"
}
teleop-sim() {                           # MuJoCo viewer (drops into hand_mujoco container)
  cd "$MUJOCO_DIR" && ./build_and_run.sh
}

# ── Inspect topics ─────────────────────────────────────────────────────────
teleop-check() { _teleop_exec "ros2 topic hz /hand_finger_angles"; }
teleop-echo()  { _teleop_exec "ros2 topic echo /hand_finger_angles --once"; }
teleop-cmd()   { _teleop_exec "ros2 topic echo /hand_command_angles"; }
teleop-act()   { _teleop_exec "ros2 topic echo /hand_actual_angles"; }
teleop-status(){ _teleop_exec "ros2 topic echo /hand_status"; }
teleop-debug() { _teleop_exec "rqt_image_view"; }   # pick /hand_landmarks_debug_image

# ── Standalone hardware diagnostics (no ROS, no container) ─────────────────
teleop-hand-scan()  { python3 "$HOME/Downloads/test.py" --scan-ids; }
teleop-hand-diag()  { python3 "$HOME/Downloads/test.py" --id "${1:-2}" --diagnose-only; }
teleop-hand-clear() { python3 "$HOME/Downloads/test.py" --id "${1:-2}" --clear-error; }

teleop-help() {
  cat <<'EOF'
Hand teleop shortcuts (laptop):
  teleop-up / teleop-down / teleop-shell   container lifecycle
  teleop-build                             colcon build inside container
  teleop-rebuild                           clean --no-cache image rebuild
  teleop-perception                        run MediaPipe on /image/compressed
  teleop-sim                               MuJoCo viewer (hand_mujoco container)
  teleop-real [/dev/ttyXXX]                send to real RH56 over RS485
  teleop-check / teleop-echo               /hand_finger_angles rate / one sample
  teleop-cmd / teleop-act / teleop-status  commanded / actual / status topics
  teleop-debug                             rqt_image_view (skeleton overlay)
  teleop-hand-scan                         find hand ID over serial
  teleop-hand-diag [id]                    ANGLE_ACT / ERROR / STATUS
  teleop-hand-clear [id]                   clear fault latch
Full reference: ~/CAIR/TELEOP_COMMANDS.md
EOF
}

# ── JETSON ONLY — paste into the Jetson's own ~/.bashrc, not the laptop's ──
# teleop-relay() {
#   source /opt/ros/humble/setup.bash
#   python3 ~/compress_relay.py
# }
