# inspire_hand_teleop

Teleoperate an **Inspire RH56 dexterous hand** with your own hand. A ZED camera
feeds **MediaPipe Hands**, which is turned into per-finger commands and sent to the
real hand over RS485 — and/or to a [MuJoCo simulation](https://github.com/jamilchahineaub/inspire_hand_sim).

```
ZED ──image──▶ MediaPipe ──/hand_finger_angles──▶ ┬─▶ MuJoCo sim (hand_mujoco)
                                                   └─▶ real RH56 hand (RS485)
```

The hand has 6 DOF: index, middle, ring, pinky, thumb bend, thumb rotation.

---

## Requirements

- ROS 2 Humble (runs inside Docker — you only need Docker on the host)
- A camera publishing a ROS 2 image (we use a ZED; any `sensor_msgs/Image` works)
- *(optional)* an Inspire RH56 hand + USB-RS485 adapter for real-hand control
- *(optional)* [hand_mujoco](https://github.com/jamilchahineaub/inspire_hand_sim) for the simulation

---

## Quick start

### 1. Build and start the container
```bash
git clone https://github.com/jamilchahineaub/inspire_hand_teleop.git
cd inspire_hand_teleop
docker compose -f docker/docker-compose.yaml up -d
docker exec -it inspire_hand_teleop bash
```

### 2. Run perception
Inside the container:
```bash
source install/setup.bash
ros2 launch hand_perception hand_perception.launch.xml image_topic:=/image
```
Show your hand to the camera and check it is working:
```bash
ros2 topic echo /hand_finger_angles --once
```

### 3a. Drive the simulation
In the [hand_mujoco](https://github.com/jamilchahineaub/inspire_hand_sim) repo, run the bridge —
it subscribes to `/hand_finger_angles` and the simulated hand follows yours.

### 3b. Drive the real hand
Plug in the USB-RS485 adapter, power the hand (24 V), then:
```bash
ros2 launch hand_perception rs485_hand.launch.xml
```
The hand mirrors your motion. Check the live feedback with
`ros2 topic echo /hand_actual_angles`.

> On Ubuntu, the `brltty` service can steal the USB-serial adapter. If
> `/dev/ttyUSB0` keeps disappearing: `sudo apt-get purge -y brltty`, then replug.

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/hand_landmarks` | `HandLandmarksArray` | 21 MediaPipe landmarks per hand |
| `/hand_finger_angles` | `HandFingerAnglesArray` | per-DOF angle, normalised value, and command (0–1000) |
| `/hand_landmarks_debug_image` | `sensor_msgs/Image` | annotated skeleton overlay |
| `/hand_command_angles` | `Int32MultiArray` | 6 commands sent to the hand `[pinky, ring, middle, index, thumb_bend, thumb_rot]` |
| `/hand_actual_angles` | `Int32MultiArray` | actual angles read back from the hand |
| `/hand_status` | `Int32MultiArray` | per-DOF status (grasp / reached / current-limit / fault) |
| `/hand_error` | `Int32MultiArray` | per-DOF error codes |

---

## Configuration

- `src/hand_perception/config/params.yaml` — camera topic, detection settings, and
  per-finger angle calibration (open/closed angle ranges).
- `src/hand_perception/config/rs485_params.yaml` — serial port, hand ID,
  `target_handedness` (which hand drives the robot), and the command settings.

To calibrate a finger, read its angle open vs. closed and set the
`*_min_angle_deg` / `*_max_angle_deg` values, then restart the node.

---

## Documentation

- [`docs/HAND_TELEOP_SYSTEM.md`](docs/HAND_TELEOP_SYSTEM.md) — full system reference
  (every file, the RS485 protocol, design notes, troubleshooting).
- [`docs/TELEOP_COMMANDS.md`](docs/TELEOP_COMMANDS.md) — command cheat-sheet.
- `teleop_aliases.sh` — optional `teleop-*` shell shortcuts (`source` it).

---

## License

MIT — see [LICENSE](LICENSE).
