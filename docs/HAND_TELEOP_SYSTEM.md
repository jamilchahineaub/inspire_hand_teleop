# Hand Teleoperation System — Complete Reference

End-to-end teleoperation of an **Inspire RH56 (DFQ) right dexterous hand** from a
**ZED camera**, using **MediaPipe** hand tracking, with two interchangeable
outputs: a **MuJoCo simulation** and the **real hand over RS485**.

This document covers the entire system: architecture, every file and what it
contains, all design decisions, the operating runbook, hardware bring-up, and a
troubleshooting log of every problem we hit and how it was solved.

---

## 1. Overview & Goal

**Goal:** move your real right hand in front of a ZED camera and have a robot hand
(simulated and/or physical) mirror your finger and thumb motion in real time.

**Pieces:**
- **Jetson** (mounted with the ZED): publishes the camera image, and runs a small
  relay that compresses it for the network.
- **Laptop** (the compute): runs MediaPipe perception + the two output drivers,
  all in Docker containers.
- **Inspire RH56 hand**: 6 DOF (4 fingers + thumb bend + thumb rotation), driven
  over RS485 via a USB-RS485 (CH340) adapter.

**Two repos under `~/CAIR/`:**
- `hand_mujoco/` — the MuJoCo simulation of the hand + a ROS 2 bridge.
- `inspire_hand_teleop/` — the ROS 2 perception pipeline + the RS485 hardware node.

---

## 2. End-to-End Architecture

```
┌─────────────── JETSON ───────────────┐        ┌──────────────────────── LAPTOP ────────────────────────┐
│                                       │        │                                                          │
│  ZED camera                           │        │   hand_landmark_node  (MediaPipe)                        │
│    │ /image  (raw, ~8.29 MB/frame)    │        │     │ subscribes /image/compressed                       │
│    ▼                                  │        │     │ publishes  /hand_landmarks                          │
│  compress_relay.py                    │        │     │            /hand_finger_angles  (cmd 0–1000)        │
│    │ /image/compressed (JPEG ~300 KB) │──Eth──▶│     │            /hand_landmarks_debug_image              │
│    ▼                                  │        │     └──────────────┬───────────────────┐                 │
└───────────────────────────────────────┘        │                    ▼                   ▼                 │
                                                  │   mujoco_hand_ros2_bridge      rs485_hand_node           │
                                                  │   (SIM: MuJoCo viewer)         (REAL: RH56 over RS485)   │
                                                  │                                  │ /hand_command_angles  │
                                                  │                                  │ /hand_actual_angles   │
                                                  │                                  │ /hand_status /error   │
                                                  └──────────────────────────────────┴───────────────────────┘
```

Both output nodes subscribe to the **same** `/hand_finger_angles`. Run the sim,
the real hand, or both at once. Switching = start one or the other.

### Topic catalogue

| Topic | Type | Publisher | Subscribers | QoS | Rate |
|-------|------|-----------|-------------|-----|------|
| `/image` | `sensor_msgs/Image` | ZED node (Jetson) | compress_relay | RELIABLE | ~30 Hz |
| `/image/compressed` | `sensor_msgs/CompressedImage` | compress_relay | hand_landmark_node | RELIABLE | ~8–30 Hz |
| `/hand_landmarks` | `hand_perception_msgs/HandLandmarksArray` | hand_landmark_node | (consumers/debug) | BEST_EFFORT | = image rate |
| `/hand_finger_angles` | `hand_perception_msgs/HandFingerAnglesArray` | hand_landmark_node | mujoco bridge, rs485 node | BEST_EFFORT | = image rate |
| `/hand_landmarks_debug_image` | `sensor_msgs/Image` | hand_landmark_node | rqt_image_view | BEST_EFFORT | = image rate |
| `/hand_command_angles` | `std_msgs/Int32MultiArray` | rs485_hand_node | (monitoring) | RELIABLE | = input |
| `/hand_actual_angles` | `std_msgs/Int32MultiArray` | rs485_hand_node | (monitoring) | RELIABLE | 20 Hz (HW only) |
| `/hand_status` | `std_msgs/Int32MultiArray` | rs485_hand_node | (monitoring) | RELIABLE | 2 Hz (HW only) |
| `/hand_error` | `std_msgs/Int32MultiArray` | rs485_hand_node | (monitoring) | RELIABLE | 2 Hz (HW only) |

**DOF order** used in every 6-element command/feedback array:
`[0]=pinky [1]=ring [2]=middle [3]=index [4]=thumb_bend [5]=thumb_rot`
(this is the RH56 ANGLE_SET register order).

---

## 3. Repository Layout

```
~/CAIR/
├── HAND_TELEOP_SYSTEM.md          ← this document
├── TELEOP_COMMANDS.md             ← quick command cheat-sheet
├── teleop_aliases.sh              ← sourceable bash shortcuts (teleop-*)
│
├── hand_mujoco/                   ← MuJoCo SIMULATION
│   ├── Dockerfile                 ← nvidia/opengl + ROS humble + colcon + rosidl build stack
│   ├── build_and_run.sh           ← builds image, runs container (GPU + X11 + host net)
│   ├── model/
│   │   ├── dfq_right_hand.xml      ← the MJCF hand model (joints, mimics, actuators)
│   │   └── scene.xml               ← floor + lighting + <include> of the hand
│   ├── scripts/
│   │   ├── mujoco_hand_ros2_bridge.py  ← subscribes /hand_finger_angles → drives the sim
│   │   ├── view_hand.py            ← standalone interactive viewer (no ROS)
│   │   └── test_joint.py           ← headless single-joint smoke test
│   └── ros2_ws/                    ← holds the shared hand_perception_msgs build
│
└── inspire_hand_teleop/           ← PERCEPTION + REAL HARDWARE
    ├── docker/
    │   ├── Dockerfile              ← ros:humble + mediapipe/numpy/pyserial pins
    │   ├── docker-compose.yaml     ← host net, privileged, DDS env, src mount
    │   └── entrypoint.sh           ← colcon build + source on container start
    └── src/
        ├── hand_perception_msgs/   ← custom messages
        │   ├── msg/HandLandmarks.msg
        │   ├── msg/HandLandmarksArray.msg
        │   ├── msg/HandFingerAngles.msg
        │   ├── msg/HandFingerAnglesArray.msg
        │   ├── CMakeLists.txt
        │   └── package.xml
        └── hand_perception/        ← the perception + hardware package
            ├── hand_perception/    ← importable Python module (symlink-installed)
            │   ├── __init__.py
            │   ├── landmark_detector.py    ← MediaPipe wrapper
            │   ├── finger_angles.py        ← angle math (4 fingers + 2 thumb DOFs)
            │   └── inspire_hand_driver.py  ← RS485 protocol driver
            ├── scripts/            ← node executables (copied-installed → need rebuild)
            │   ├── hand_landmark_node.py   ← perception node
            │   └── rs485_hand_node.py      ← hardware node
            ├── config/
            │   ├── params.yaml             ← perception + calibration params
            │   └── rs485_params.yaml       ← hardware params
            ├── launch/
            │   ├── hand_perception.launch.xml
            │   └── rs485_hand.launch.xml
            ├── CMakeLists.txt
            └── package.xml
```

External (not in the repos):
- **Jetson** `~/compress_relay.py` — image compression relay.
- **Laptop** `~/Downloads/test.py` — standalone RS485 hardware diagnostic CLI.

---

## 4. `hand_mujoco` — File by File

### `model/dfq_right_hand.xml` (the MJCF model)

Translated from the manufacturer URDF
(`~/unitree_ros/robots/g1_description/inspire_hand/DFQ_right_hand.urdf`).

**Structure:** a single fixed palm body `R_hand_base_link` (elevated 0.3 m), with
the thumb chain and four finger chains as children. Meshes resolve from
`meshdir="/workspace/robot_assets/meshes"` (mounted in the container).

**6 independently actuated joints:**
`R_thumb_proximal_yaw_joint`, `R_thumb_proximal_pitch_joint`,
`R_index_proximal_joint`, `R_middle_proximal_joint`, `R_ring_proximal_joint`,
`R_pinky_proximal_joint`.

**6 mimic couplings** (URDF `<mimic>` → MuJoCo `<equality><joint …>`), of the form
`joint1 = polycoef[1] × joint2`:

| Equality | dependent joint | driver joint | multiplier |
|----------|-----------------|--------------|------------|
| thumb_int_eq  | R_thumb_intermediate  | R_thumb_proximal_pitch | 1.6 |
| thumb_dist_eq | R_thumb_distal        | R_thumb_proximal_pitch | 2.4 |
| index_int_eq  | R_index_intermediate  | R_index_proximal       | 1.0 |
| middle_int_eq | R_middle_intermediate | R_middle_proximal      | 1.0 |
| ring_int_eq   | R_ring_intermediate   | R_ring_proximal        | 1.0 |
| pinky_int_eq  | R_pinky_intermediate  | R_pinky_proximal       | 1.0 |

**6 position actuators** (`kp=5`), with ctrl ranges in radians:

| Actuator | Joint | ctrlrange |
|----------|-------|-----------|
| `thumb_yaw`   | R_thumb_proximal_yaw_joint   | −0.1 … 1.3 |
| `thumb_pitch` | R_thumb_proximal_pitch_joint | −0.1 … 0.6 |
| `index`       | R_index_proximal_joint       | 0 … 1.7 |
| `middle`      | R_middle_proximal_joint      | 0 … 1.7 |
| `ring`        | R_ring_proximal_joint        | 0 … 1.7 |
| `pinky`       | R_pinky_proximal_joint       | 0 … 1.7 |

**⚠️ The thumb quaternion fix (important).** The thumb's two base bodies use
`quat=` instead of `euler=`:
```xml
<body name="R_thumb_proximal_base" … quat="0.5 0.5 -0.5 0.5">
  …
  <body name="R_thumb_proximal" … quat="0.099684 0.099685 0.700046 0.700044">
```
Why: **URDF `rpy` is extrinsic XYZ** (`Rz·Ry·Rx`), but **MuJoCo `euler` defaults to
intrinsic XYZ**. For a *single-axis* rotation the two conventions are identical —
and all four fingers have single-axis rpy (`-3.1416 0 0`, etc.), so they translated
fine. The **thumb's two bodies have compound rpy** (`1.5708 -1.5708 0` and
`1.5708 0 2.8587`), where the conventions diverge — so only the thumb came out
wrong (right place, but rolled/twisted). The fix computes the quaternion from the
URDF (extrinsic) convention and writes it directly as `quat`, leaving the (correct)
fingers untouched.

### `model/scene.xml`
Thin wrapper: floor plane, lighting, and `<include file="dfq_right_hand.xml"/>`.
This is the file the bridge loads.

### `scripts/mujoco_hand_ros2_bridge.py` (SIM driver)

Subscribes to `/hand_finger_angles`, takes the first **Right** hand, and maps its
per-DOF `cmd` (0–1000) onto the MuJoCo actuators.

**Module constants:**
- `_ACTUATOR_MAP` — `{field: (actuator_name, ctrl_min, ctrl_max)}`:
  ```
  index→index (0,1.7)   middle→middle (0,1.7)   ring→ring (0,1.7)
  pinky→pinky (0,1.7)   thumb_bend→thumb_pitch (-0.1,0.6)   thumb_rot→thumb_yaw (-0.1,1.3)
  ```
- `_SCENE_XML` — resolves `../model/scene.xml` relative to the script.
- **cmd→ctrl formula:** `ctrl = ctrl_min + (cmd/1000) × (ctrl_max − ctrl_min)`.

**Functions / class:**
- `_find_actuator_ids(model)` → `{name: id}` for all actuators.
- `class MujocoHandBridge(Node)`:
  - `__init__(model, data, actuator_ids)` — declares params:
    - `thumb_yaw_rest` (0.6) — idle thumb yaw when no hand is detected.
    - `thumb_mount_roll_deg` / `thumb_mount_pitch_deg` / `thumb_mount_yaw_deg` (0) —
      live mount-orientation correction (a runtime quaternion edit of the thumb
      base body); used during the thumb-orientation debugging.
    - Captures the thumb base body id + its original `body_quat`.
    - Imports `HandFingerAnglesArray` (fails fast with a build hint if the msgs
      aren't built).
    - Subscribes `/hand_finger_angles` with `qos_profile_sensor_data` (BEST_EFFORT).
  - `_apply_thumb_yaw()` — writes the idle yaw into `data.ctrl`.
  - `_set_thumb_mount(roll, pitch, yaw)` — composes a correction quaternion (local
    frame) onto the original `body_quat`, writes `model.body_quat`, then
    `mujoco.mj_forward()` so the whole thumb subtree re-orients live.
  - `_apply_thumb_mount()` — applies the mount params at startup.
  - `_on_set_params(params)` — lets `ros2 param set` re-tune `thumb_yaw_rest` and
    the three mount angles live (no restart).
  - `_cb(msg)` — first Right hand → for each mapped field, `data.ctrl[id] = lo +
    (cmd/1000)(hi−lo)`.
- `main()` — loads model/data, builds the node, **spins ROS in a background
  thread**, runs `mujoco.viewer.launch_passive` on the **main thread** (required by
  the viewer), stepping + syncing in a loop.

### `scripts/view_hand.py` / `scripts/test_joint.py`
The original standalone tools: an interactive viewer, and a headless test that
sweeps a single joint. Useful for inspecting the model without ROS.

### `Dockerfile`
- Base `nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04` (GPU rendering for the viewer).
- apt: python3 + GL/X libs (glfw, xkbcommon, xinerama, xcursor, xi, xrandr) →
  ROS `ros-humble-ros-base` + `ros-humble-rmw-cyclonedds-cpp` +
  `python3-colcon-common-extensions` + **`python3-ament-package` and the rosidl
  generator/typesupport stack** (needed to *build* the custom messages inside this
  GPU container).
- pip: `mujoco`, `numpy`.

### `build_and_run.sh`
`docker build -t hand_mujoco:v1 .`, `xhost +local:docker`, then `docker run` with:
GPU (`--gpus all --runtime=nvidia --device /dev/dri`), X11 (`DISPLAY`,
`/tmp/.X11-unix`), `--network host`, `ROS_DOMAIN_ID=0`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and mounts for `model/`, `scripts/`,
`~/unitree_ros/.../g1_description` (meshes as `/workspace/robot_assets`), the
`ros2_ws/`, and the inspire `hand_perception_msgs` source overlaid into
`ros2_ws/src`.

### `ros2_ws/`
The colcon workspace where `hand_perception_msgs` is built so the bridge can import
the message type. Built once; `install/` persists on the host between runs.

---

## 5. `inspire_hand_teleop` — File by File

### `src/hand_perception_msgs/` (custom messages)

**`HandLandmarks.msg`**
```
std_msgs/Header   header
string            handedness          # "Left" / "Right" (after flip correction)
float32           score               # detection confidence
geometry_msgs/Point[] points          # 21 MediaPipe landmarks (x,y∈[0,1], z=rel depth)
```
**`HandLandmarksArray.msg`** — `Header header` + `HandLandmarks[] hands`.

**`HandFingerAngles.msg`** — per-DOF raw angle, normalized value, and command:
```
std_msgs/Header header
string  handedness
float32 score
float32 index_angle_deg   middle_angle_deg   ring_angle_deg   pinky_angle_deg
float32 thumb_bend_angle_deg   thumb_rot_angle_deg
float32 index_norm  middle_norm  ring_norm  pinky_norm
float32 thumb_bend_norm   thumb_rot_norm
uint16  index_cmd  middle_cmd  ring_cmd  pinky_cmd     # 0–1000
uint16  thumb_bend_cmd   thumb_rot_cmd                 # 0–1000
```
`cmd` is **uint16** because the RH56 range is **0–1000** (not 0–100).

**`HandFingerAnglesArray.msg`** — `Header header` + `HandFingerAngles[] hands`.
`hands[i]` in this and in `/hand_landmarks` always refer to the same detected hand.

**`CMakeLists.txt` / `package.xml`** — `rosidl_generate_interfaces` over the four
msgs, depending on `std_msgs` + `geometry_msgs`.

### `src/hand_perception/hand_perception/landmark_detector.py`
- `@dataclass HandResult` — `handedness`, `score`, `landmarks`.
- `class LandmarkDetector` — wraps **legacy** `mp.solutions.hands.Hands`
  (context-manager lifecycle). `detect(rgb)` → `List[HandResult]` (caches the raw
  MediaPipe result for drawing); `draw_on(bgr)` → annotated copy with the hand
  skeleton.
- **MediaPipe is pinned to `0.10.14`** — versions ≥0.10.21 removed the
  `mp.solutions` API (the cause of the `module 'mediapipe' has no attribute
  'solutions'` crash).

### `src/hand_perception/hand_perception/finger_angles.py` (the angle math)
- **Landmark indices:** wrist 0; thumb CMC 1, MCP 2, IP 3, TIP 4; index MCP 5/PIP
  6; middle 9/10; ring 13/14; pinky 17/18.
- `FINGER_DEFS` — the **four** fingers only `(name, mcp_idx, pip_idx)`. The thumb is
  handled separately.
- `@dataclass FingerConfig(name, min_deg, max_deg, mcp_idx=-1, pip_idx=-1)`,
  `@dataclass FingerAngles(angle_deg, norm, cmd)`.
- `compute_metacarpal_direction(lms, mcp_idx)` → unit vector wrist→MCP.
- `compute_finger_bend_deg(lms, mcp, pip)` — **the datasheet α**:
  `α = arccos( dot( normalize(PIP−MCP), normalize(MCP−wrist) ) )` — angle between
  the proximal phalanx (MCP→PIP) and the metacarpal axis (wrist→MCP).
- `compute_thumb_bend_deg(lms)` — **thumb flexion ∠θ** (curl): angle at the thumb
  MCP between `CMC→MCP` and `MCP→TIP`. Independent of opposition.
- `compute_thumb_rotation_deg(lms)` — **thumb opposition ∠β** on the palm plane:
  ```
  n   = normalize( (index_MCP−wrist) × (pinky_MCP−wrist) )     # palm normal
  d⊥  = normalize( (TIP−CMC) − ((TIP−CMC)·n) n )               # thumb on plane
  ref = normalize( index_MCP − wrist )
  β   = arccos( dot(d⊥, ref) )
  ```
- `normalize_angle(angle, min, max)` → `clamp((angle−min)/(max−min), 0, 1)`.
- `compute_all_finger_angles(lms, configs)` — dispatches by config name
  (`thumb_bend`/`thumb_rot`/finger), normalizes, and returns
  `cmd = round(1000 × norm)` per DOF.

### `src/hand_perception/hand_perception/inspire_hand_driver.py` (RS485 driver)
Vendored verbatim (CLI stripped) from the hardware-tested `~/Downloads/test.py`.
- **Constants:** `TX_HEADER=EB 90`, `RX_HEADER=90 EB`, `CMD_READ=0x11`,
  `CMD_WRITE=0x12`; registers `CLEAR_ERROR=0x03EC`, `ANGLE_SET=0x05CE`,
  `ANGLE_ACT=0x060A`, `ERROR=0x0646`, `STATUS=0x064C`.
- `class InspireHand(port, baudrate=115200, hand_id=2, …)` — opens serial **8N1**.
  - `checksum(frame) = sum(frame[2:-1]) & 0xFF` (verified against the datasheet:
    ANGLE_SET write → `0x70`, ANGLE_ACT read → `0x32`).
  - `build_frame(cmd, address, payload)` — assembles header/id/len/cmd/addr/data/cksum.
  - `read_response_frame()` — **robust RX**: scans byte-by-byte for the `90 EB`
    header, reads id+len, then `len+1` bytes; validates checksum **and** that the
    responding id matches.
  - `transact(frame)` — flush input, write, read a response.
  - `write_register_words` / `read_register_words` (int16 LE).
  - `set_position_raw(6 values)` → ANGLE_SET; `get_position_raw()` → ANGLE_ACT.
  - `get_status_codes()`, `get_error_codes()`, `clear_error()`.
- **Default `hand_id=2`** — the physical hand ships as ID 2 (confirmed with
  `--scan-ids`).

### `src/hand_perception/scripts/hand_landmark_node.py` (perception node)
- **Params:** `image_topic`, `compressed_image`, `max_num_hands`,
  `min_detection_confidence`, `min_tracking_confidence`, `publish_debug_image`,
  `flip_handedness`, and per-DOF `{index,middle,ring,pinky,thumb_bend,thumb_rot}_
  min/max_angle_deg`.
- **Subscribes** `/image` or `/image/compressed` (RELIABLE QoS to match the ZED;
  decodes JPEG via `cv2.imdecode` when `compressed_image: true`).
- **Publishes** `/hand_landmarks`, `/hand_finger_angles`,
  `/hand_landmarks_debug_image`.
- **Handedness flip** — MediaPipe assumes a mirrored selfie cam; the ZED is not
  mirrored, so `flip_handedness: true` swaps Left↔Right.
- **Single-Right-hand guard** — drops any 2nd Right-hand detection per frame
  (latency reduction).
- `_overlay_angles` draws per-DOF angle/norm/cmd near each MCP; `_MCP_IDX` maps
  DOF→landmark for label placement (thumb_bend→2, thumb_rot→1).

### `src/hand_perception/scripts/rs485_hand_node.py` (hardware node)
- **Params:** `serial_port` (/dev/ttyUSB0), `baud_rate` (115200), `hand_id` (2),
  `thumb_rotation_cmd` (500, fallback), `publish_feedback` (true),
  `publish_diagnostics` (true), `clear_error_on_start` (false),
  `reconnect_period_s` (5.0), **`invert_command` (true)**.
- **QoS split (the fix):** output topics are **RELIABLE**; the
  `/hand_finger_angles` subscription is **`qos_profile_sensor_data` (BEST_EFFORT)**
  to match the perception publisher (a RELIABLE subscriber would receive nothing).
- **Publishers:** `/hand_command_angles` (always), `/hand_actual_angles`,
  `/hand_status`, `/hand_error` (the latter three only when hardware is connected).
- `_cb(msg)` — first Right hand → reorder to `[pinky,ring,middle,index,thumb_bend,
  thumb_rot]`, publish the raw teleop cmd, then if connected write to hardware with
  **`1000 − cmd`** (RH56 ANGLE_SET is inverted: 1000=open, 0=closed).
- `_read_feedback()` (20 Hz) → ANGLE_ACT → `/hand_actual_angles`;
  `_read_diagnostics()` (2 Hz) → STATUS/ERROR → `/hand_status`, `/hand_error`.
- **Defensive serial:** `_try_open` / `_reconnect_if_needed` (retries every 5 s) /
  `_drop_hand` (drop + retry on any serial error). With no hardware it logs once
  ("command-publish-only mode") and keeps publishing `/hand_command_angles`.
- Guarded SIGINT shutdown (`if rclpy.ok(): rclpy.shutdown()`).

### `config/params.yaml` (perception + calibration)
```yaml
hand_landmark_node:
  ros__parameters:
    image_topic: /image/compressed
    compressed_image: true
    max_num_hands: 2
    min_detection_confidence: 0.7
    min_tracking_confidence: 0.5
    publish_debug_image: true
    flip_handedness: true            # true for non-mirrored cameras (ZED)
    # per-DOF calibration (degrees) → mapped to cmd 0–1000:
    index_min_angle_deg / index_max_angle_deg            # 19 / 120
    middle/ring/pinky  (same defaults)
    thumb_bend_min/max_angle_deg     # 10 / 60  (curl: straight→fist)
    thumb_rot_min/max_angle_deg      # 25 / 80  (opposition: tucked→opposed)
```
**Calibration:** with the node running, `ros2 topic echo /hand_finger_angles --once
| grep angle_deg`; read your open/neutral value → `*_min`, your closed/articulated
value → `*_max`; edit the YAML; restart the node (params are data, no rebuild).

### `config/rs485_params.yaml` (hardware)
```yaml
rs485_hand_node:
  ros__parameters:
    serial_port: /dev/ttyUSB0
    baud_rate: 115200
    hand_id: 2
    thumb_rotation_cmd: 500          # fallback only
    publish_feedback: true
    publish_diagnostics: true
    clear_error_on_start: false
    reconnect_period_s: 5.0
    # invert_command (default true in code): 1000-cmd on the serial path
```

### `launch/*.launch.xml`
- `hand_perception.launch.xml` — loads `params.yaml`, exposes `image_topic` and
  `publish_debug_image` as overridable args.
- `rs485_hand.launch.xml` — loads `rs485_params.yaml`, exposes `serial_port`.

### `CMakeLists.txt` / `package.xml`
- The Python **module** (`hand_perception/`) is installed via
  `ament_python_install_package` → **symlinked** (edits live on restart, no build).
- The **scripts** (`hand_landmark_node.py`, `rs485_hand_node.py`) are installed via
  `install(PROGRAMS …)` → **copied** (edits require `colcon build`).
- Depends on rclpy, sensor_msgs, geometry_msgs, std_msgs, cv_bridge,
  hand_perception_msgs, python3-serial.

### `docker/`
- `Dockerfile` — `FROM ros:humble`; apt cv-bridge / image-transport /
  rqt-image-view / cyclonedds; `pip3 install mediapipe==0.10.14 numpy==1.26.4
  pyserial`.
- `docker-compose.yaml` — `network_mode: host`, `privileged: true` (exposes
  `/dev/ttyUSB*`), env `ROS_DOMAIN_ID=0` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` +
  `DISPLAY`, mounts `../src → /workspace/ros2_ws/src` and X11.
- `entrypoint.sh` — sources ROS, `colcon build --symlink-install`, sources the
  workspace, drops to a shell.

---

## 6. External Pieces

### Jetson — `~/compress_relay.py`
Subscribes `/image` (RELIABLE), JPEG-encodes each frame (quality 80), republishes
`/image/compressed` (`CompressedImage`, RELIABLE). Cuts ~8.29 MB/frame down to
~300 KB so the link isn't saturated. Decodes without cv_bridge (pure numpy + cv2)
for portability. Run with `python3 ~/compress_relay.py` after sourcing ROS.

### Laptop — `~/Downloads/test.py`
Standalone RS485 diagnostic (no ROS), the origin of `inspire_hand_driver.py`:
```
--scan-ids                 find which ID the hand answers on (expect 2)
--scan-baudrates B0,B1,..  sweep baud rates
--id N --diagnose-only     read ANGLE_ACT / ERROR / STATUS
--id N --set-raw v0,..,v5  write one 6-DOF target (0..1000, hardware scale)
--id N --clear-error       clear a locked-rotor / over-current latch
--id N --debug             close→open sweep, prints hex TX/RX frames
```

---

## 7. Key Design Decisions & Conventions

- **Command scale.** Teleop convention: `0 = open, 1000 = closed`. The MuJoCo joint
  ranges line up with this, so the sim is correct directly. The **RH56 ANGLE_SET is
  the opposite** (`1000 = open, 0 = closed` — the register tracks actuator stroke,
  largest when extended). So `rs485_hand_node` applies `1000 − cmd` **only on the
  serial path** (`invert_command: true`); `/hand_command_angles` and the sim keep
  the intuitive scale.
- **QoS.** The perception node publishes `/hand_finger_angles` **BEST_EFFORT**
  (sensor data). A **RELIABLE subscriber is incompatible and receives nothing** —
  every subscriber must use `qos_profile_sensor_data`. (This bit us twice: the
  slow image, and the silent RS485 node.)
- **Image bandwidth.** Raw 2K ZED frames are ~8.29 MB each; at 30 Hz that's ~250
  MB/s, far beyond Gigabit. The JPEG relay (~300 KB) fixes it.
- **`--symlink-install` nuance.** The `hand_perception/` module is symlinked (edit
  → just restart). The `scripts/` node executables are **copied** → editing them
  requires `colcon build`.
- **DDS environment.** `ROS_DOMAIN_ID=0` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  are baked into both containers. **Bare-host shells (Jetson, host `ros2` CLI) must
  export them manually** or they won't see the containers' topics.
- **Hand parameters.** ID 2, 115200 8N1; the hand needs its own **24 V** supply
  (USB-RS485 carries data only).

---

## 8. Operating Procedures (Runbook)

### One-time host setup
```bash
sudo apt-get purge -y brltty          # frees the CH340 adapter (see §10)
# after plugging in the USB-RS485 adapter:
ls /dev/ttyUSB*                        # → /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0
```

### Terminal 1 — Jetson (camera relay)
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
python3 ~/compress_relay.py
```

### Terminal 2 — Laptop: perception
```bash
cd ~/CAIR/inspire_hand_teleop
xhost +local:docker
docker compose -f docker/docker-compose.yaml up -d
docker compose -f docker/docker-compose.yaml restart      # so it sees /dev/ttyUSB0
docker exec -it inspire_hand_teleop bash
#  inside:
source install/setup.bash
ros2 launch hand_perception hand_perception.launch.xml image_topic:=/image/compressed
```

### Terminal 3 — Laptop: real hand
```bash
docker exec -it inspire_hand_teleop bash
source install/setup.bash
python3 ~/Downloads/test.py --scan-ids        # confirm "ID 2 responded" first
ros2 launch hand_perception rs485_hand.launch.xml
#  log should say: Opened RH56 hand on /dev/ttyUSB0 (id 2)
```

### Terminal 4 — Laptop: simulation (instead of, or alongside, the real hand)
```bash
cd ~/CAIR/hand_mujoco
./build_and_run.sh
#  inside:
source /opt/ros/humble/setup.bash
source /workspace/hand_mujoco/ros2_ws/install/setup.bash
python3 /workspace/hand_mujoco/scripts/mujoco_hand_ros2_bridge.py
```

### Verify
```bash
ros2 topic hz /hand_finger_angles
ros2 topic echo /hand_finger_angles --once     # handedness: Right, cmds 0–1000
ros2 topic echo /hand_command_angles           # [pinky ring middle index thumb_bend thumb_rot]
ros2 topic echo /hand_status                    # 0=open 1=grasp 2=reached 5=current-limit 6=lockrotor 7=fault
ros2 topic info /hand_finger_angles --verbose   # all subscribers BEST_EFFORT
rqt_image_view                                  # /hand_landmarks_debug_image
```

### Rebuild matrix
| Changed | Action |
|---------|--------|
| module `.py` (`finger_angles.py`, `inspire_hand_driver.py`) or the bridge | restart node, **no build** |
| node script (`hand_landmark_node.py`, `rs485_hand_node.py`) | `colcon build --symlink-install --packages-select hand_perception` |
| a `.msg` | `colcon build … hand_perception_msgs hand_perception` (+ rebuild in hand_mujoco's ros2_ws) |
| Dockerfile / dependency mess | `docker compose … down && build --no-cache && up -d` |

### Shutdown
```bash
# Ctrl+C each node, then:
cd ~/CAIR/inspire_hand_teleop && docker compose -f docker/docker-compose.yaml down
# hand_mujoco container exits when its shell closes (--rm)
```

---

## 9. Hardware Bring-Up (USB / RS485)

1. **Power the hand** with its **24 V** supply (separate from USB).
2. **Plug in the USB-RS485 (CH340) adapter.** `lsusb` shows
   `1a86:7523 QinHeng Electronics CH340 serial converter`.
3. **brltty conflict.** On Ubuntu 22.04, the `brltty` braille service hijacks every
   CH340 and kills `/dev/ttyUSB0` (dmesg: `interface 0 claimed by ch341 while
   'brltty' sets config #1` → `converter now disconnected`). Fix once:
   ```bash
   sudo apt-get purge -y brltty
   # unplug + replug
   ls /dev/ttyUSB*           # → /dev/ttyUSB0, stays
   ```
4. **Permissions:** `sudo chmod 666 /dev/ttyUSB0` (or add yourself to `dialout` and
   re-login).
5. **Container must see the device.** It only inherits devices that existed at
   container start (even with `privileged`). If the port appeared after the
   container started: `docker compose … restart`, then confirm with `ls
   /dev/ttyUSB0` **inside** the container.
6. **Confirm the hand answers:** `python3 ~/Downloads/test.py --scan-ids` → expect
   `ID 2: responded, ANGLE_ACT=[…]`. Then `--set-raw 0,0,0,0,0,0` (closed, hardware
   scale) / `1000,…` (open) to confirm motion.

Quick triage:
- `ls /dev/ttyUSB0` on host works, node says "not available" → **restart the
  container**.
- `ls /dev/ttyUSB0` on host fails → **brltty / driver / power / cable**.

---

## 10. Troubleshooting Log (every issue we actually hit)

| Symptom | Cause | Fix |
|---------|-------|-----|
| `module 'mediapipe' has no attribute 'solutions'` | mediapipe ≥0.10.21 removed legacy API | pin `mediapipe==0.10.14` (in Dockerfile) |
| `numpy 1.x cannot be run in 2.2.6` / cv_bridge `_ARRAY_API not found` | `--force-reinstall` pulled numpy 2.x | `pip3 install numpy==1.26.4`; clean image rebuild |
| Pipeline ~2 Hz, Jetson 30 Hz | RELIABLE/BEST_EFFORT QoS mismatch **and** 8 MB raw frames | match QoS + use `/image/compressed` relay |
| `colcon: command not found` (hand_mujoco) | colcon not in image | add `python3-colcon-common-extensions` |
| `No module named 'ament_package'` / `No 'rosidl_typesupport_c' found` | ROS not sourced in that shell | `source /opt/ros/humble/setup.bash` before colcon |
| Thumb in sim "in right place but rolled/backwards" | URDF extrinsic-xyz rpy ≠ MuJoCo intrinsic-xyz euler (only thumb has compound rpy) | use computed `quat` on the two thumb bodies |
| `/hand_command_angles` silent | rs485 node subscribed RELIABLE to a BEST_EFFORT publisher | subscribe with `qos_profile_sensor_data` |
| Real hand: open↔close inverted (sim fine) | RH56 ANGLE_SET is 1000=open/0=closed, opposite of teleop | `invert_command` → send `1000 − cmd` on serial |
| `/dev/ttyUSB0` keeps vanishing | brltty hijacks the CH340 | `apt-get purge brltty`, replug |
| Node "serial not available" but host has the port | container started before the device existed | `docker compose … restart` |
| Ctrl+C traceback `rcl_shutdown already called` | double shutdown on SIGINT | guard `if rclpy.ok(): rclpy.shutdown()` |
| Left/Right hands swapped | MediaPipe assumes mirrored cam; ZED isn't | `flip_handedness: true` |
| Closed fist read ~50% | calibration `max_angle_deg` too high | recalibrate per-DOF min/max from live values |

---

## 11. Quick Reference

**DOF order (all 6-arrays):** `[pinky, ring, middle, index, thumb_bend, thumb_rot]`

**RH56 registers:** ANGLE_SET `0x05CE` (write 0–1000), ANGLE_ACT `0x060A` (read),
STATUS `0x064C`, ERROR `0x0646`, CLEAR_ERROR `0x03EC`. Serial 115200 8N1, ID 2,
checksum = `sum(frame[2:-1]) & 0xFF`.

**STATUS codes:** 0 unclench · 1 grasp · 2 reached target · 3 reached force · 5
current-protect · 6 locked-rotor · 7 actuator fault.

**MediaPipe landmarks:** 0 wrist; thumb 1 CMC, 2 MCP, 3 IP, 4 TIP; index 5/6/7/8;
middle 9/10/11/12; ring 13/14/15/16; pinky 17/18/19/20.

**Command scale:** teleop & sim & `/hand_command_angles` = 0 open / 1000 closed;
hardware ANGLE_SET = 1000 open / 0 closed (node inverts).

---

## 12. Deferred / Future Work

- **ZED 3D skeleton fusion.** Add `/hand_landmarks_3d` (MediaPipe 2D + ZED depth)
  and `/hand_pose_context` (ZED body tracking for wrist/forearm). Do **not** average
  MediaPipe finger landmarks with ZED's coarse hand keypoints — keep MediaPipe for
  finger articulation, ZED for wrist/body context only.
- **Jetson-native deploy.** Run the whole pipeline on the Jetson (no network image
  transfer). Jetson is ARM64 → rebuild the image natively (`docker compose build`);
  if `mediapipe==0.10.14` has no ARM64 wheel, try 0.10.18–0.10.20 (last versions
  with `mp.solutions`).
- **Thumb-rotation accuracy.** Opposition from a single 2D view is approximate; the
  palm-plane projection is the robust choice but could improve with depth.
```
