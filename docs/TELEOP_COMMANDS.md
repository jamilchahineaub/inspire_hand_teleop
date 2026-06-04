
# Hand Teleop — Command Cheat-Sheet

Quick reference for the full ZED → MediaPipe → MuJoCo / RH56 pipeline.
Shortcuts: `source ~/CAIR/teleop_aliases.sh` then use the `teleop-*` commands.

---

## 0. Architecture

```
ZED (Jetson) ──/image (raw 8MB)──► compress_relay.py (Jetson)
                                          │ /image/compressed (~300KB)
                                          ▼  Ethernet
                         hand_landmark_node (Laptop, MediaPipe)
                                          │ /hand_finger_angles  (cmds 0–1000)
                          ┌───────────────┴───────────────┐
                          ▼                                ▼
            mujoco_hand_ros2_bridge            rs485_hand_node
            (SIM: viewer)                      (REAL: RH56 over RS485)
```
Both consumers read the **same** `/hand_finger_angles`. Run sim, real, or both.

---

## 1. Jetson — camera + compressed relay
'''
source /opt/ros/humble/setup.bash
source ~/Projects/Human_Humanoid_Interaction/g1_real_ws/install/setup.bash
ros2 run zed_skeleton_pub zed_image_pub_node


source /opt/ros/humble/setup.bash
source ~/Projects/Human_Humanoid_Interaction/g1_real_ws/install/setup.bash
ros2 run zed_skeleton_pub zed_image_pub_node
'''
```bash
# (ZED publisher already runs as zed_image_pub_node)
source /opt/ros/humble/setup.bash
python3 ~/compress_relay.py            # raw /image → JPEG /image/compressed

# diagnostics
ros2 topic hz /image                   # ZED raw rate (~30 Hz)
ros2 topic hz /image/compressed        # relay output rate
ros2 topic bw /image                   # confirm raw is ~8 MB/frame
ros2 topic info /image --verbose       # check publisher QoS (RELIABLE)
```

---

## 2. Laptop — perception pipeline

```bash
cd ~/CAIR/inspire_hand_teleop
xhost +local:docker
docker compose -f docker/docker-compose.yaml up -d
docker exec -it inspire_hand_teleop bash
# ── inside container ──
source install/setup.bash
ros2 launch hand_perception hand_perception.launch.xml image_topic:=/image/compressed
```
Startup log should show `subscribed to: /image/compressed`, `compressed_image: True`,
`flip_handedness: True`, and 5 finger calibrations incl. `thumb_bend`.

---

## 3. Verify perception

```bash
# inside the container (source install/setup.bash first)
ros2 topic hz /image/compressed            # images arriving
ros2 topic hz /hand_finger_angles          # processed rate
ros2 topic echo /hand_finger_angles --once # Right hand: angles + cmds (0..1000)
ros2 interface show hand_perception_msgs/msg/HandFingerAngles
rqt_image_view                             # pick /hand_landmarks_debug_image (skeleton)
```
Open hand → cmds near 0. Fist → cmds near 1000.

---

## 4. SIM — MuJoCo viewer

docker exec -it hand_mujoco bash

source /opt/ros/humble/setup.bash
source /workspace/hand_mujoco/ros2_ws/install/setup.bash

```bash
cd ~/CAIR/hand_mujoco
./build_and_run.sh
# ── inside container ──
source /opt/ros/humble/setup.bash
# first time only:
cd /workspace/hand_mujoco/ros2_ws
colcon build --symlink-install --packages-select hand_perception_msgs
source install/setup.bash
# run:
python3 /workspace/hand_mujoco/scripts/mujoco_hand_ros2_bridge.py
# tune the fixed thumb rotation if it looks off:
python3 /workspace/hand_mujoco/scripts/mujoco_hand_ros2_bridge.py --ros-args -p thumb_yaw_rest:=0.4
```

---

## 5. REAL — RH56 hand over RS485

```bash
# inside inspire_hand_teleop container, source install/setup.bash first
ros2 launch hand_perception rs485_hand.launch.xml
# override port if needed:
ros2 launch hand_perception rs485_hand.launch.xml serial_port:=/dev/ttyACM0

# topics it publishes:
ros2 topic echo /hand_command_angles   # what we command (always, even w/o hardware)
ros2 topic echo /hand_actual_angles    # ANGLE_ACT feedback (hardware only)
ros2 topic echo /hand_status           # 0 unclench·1 grasp·2 reached·3 force·5 i-limit·6 lock·7 fault
ros2 topic echo /hand_error            # per-DOF error bitmask
```
DOF order in all three: `[pinky, ring, middle, index, thumb_bend, thumb_rot]`.
With no hand connected the node logs "command-publish-only mode" and keeps
publishing `/hand_command_angles` — sim still works.

---

## 6. Build / rebuild

```bash
# inside container — pick up code/msg changes:
colcon build --symlink-install --packages-select hand_perception hand_perception_msgs
source install/setup.bash

# clean image rebuild (after Dockerfile change / dependency mess):
cd ~/CAIR/inspire_hand_teleop
docker compose -f docker/docker-compose.yaml down
docker compose -f docker/docker-compose.yaml build --no-cache
docker compose -f docker/docker-compose.yaml up -d
```
Note: Python edits are live via `--symlink-install` (just restart the node).
`.msg` changes always require a `colcon build`.

---

## 7. Troubleshooting

```bash
# mediapipe API gone ("module has no attribute 'solutions'"):
pip3 install "mediapipe==0.10.14" --force-reinstall
pip3 install "numpy==1.26.4"          # force-reinstall pulls numpy 2.x → revert

# apt half-configured on Jetson:
sudo dpkg --configure -a

# slow image rate / QoS mismatch:
ros2 topic info /image --verbose      # publisher RELIABLE? subscriber must match
# (hand_landmark_node subscribes RELIABLE; raw 8MB over WiFi is too big → use /image/compressed)
```

---

## 8. Jetson deploy (run pipeline natively, no network bottleneck)

```bash
# from laptop — sync SOURCE (not the x86 image; Jetson is ARM64):
rsync -avz ~/CAIR/inspire_hand_teleop/ user@<jetson_ip>:~/CAIR/inspire_hand_teleop/
# on Jetson — rebuild natively:
cd ~/CAIR/inspire_hand_teleop
docker compose -f docker/docker-compose.yaml build
docker compose -f docker/docker-compose.yaml up -d
# if mediapipe==0.10.14 has no ARM64 wheel: try 0.10.18–0.10.20 (last with mp.solutions)
```

---

## 9. Standalone hardware diagnostics (no ROS) — `~/Downloads/test.py`

Fastest way to confirm the hand is alive before touching ROS:

```bash
python3 ~/Downloads/test.py --scan-ids                  # find the hand's ID (expect ID 2)
python3 ~/Downloads/test.py --scan-baudrates 115200,57600,19200
python3 ~/Downloads/test.py --id 2 --diagnose-only      # print ANGLE_ACT / ERROR / STATUS
python3 ~/Downloads/test.py --id 2 --set-raw 0,0,0,0,0,0          # all open
python3 ~/Downloads/test.py --id 2 --set-raw 1000,1000,1000,1000,1000,1000  # all closed
python3 ~/Downloads/test.py --id 2 --clear-error        # clear locked-rotor / over-current latch
python3 ~/Downloads/test.py --id 2 --debug              # close→open sweep, prints hex frames
```
The ROS `rs485_hand_node` uses the same proven driver internally (hand_id 2).

---

## 10. SIM ↔ REAL switch

| Mode | Terminal 4 command | Result |
|------|--------------------|--------|
| **Sim**  | `python3 /workspace/hand_mujoco/scripts/mujoco_hand_ros2_bridge.py` | MuJoCo viewer mirrors your hand |
| **Real** | `ros2 launch hand_perception rs485_hand.launch.xml` | RH56 moves; feedback on `/hand_actual_angles` |
| **Both** | run both in two terminals | sim shows command, hardware reports actual |

Switching = Ctrl+C one, start the other. No rebuild, no config edits.
