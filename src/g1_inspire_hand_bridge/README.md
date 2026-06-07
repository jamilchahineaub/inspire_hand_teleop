# g1_inspire_hand_bridge

Bridges the teleop hand command topic onto the **Unitree G1 Inspire hand DDS
interface**.

The G1 only listens on the Unitree DDS topic `rt/inspire/cmd` — it cannot
subscribe to our `/hand_finger_angles` topic. This node sits in between:

```
/hand_finger_angles ──▶ g1_inspire_hand_bridge_node ──▶ rt/inspire/cmd  (robot listens)
(our teleop output)                                  ◀── rt/inspire/state ──▶ /g1_inspire/state
```

It converts `hand_perception_msgs/HandFingerAnglesArray` into
`unitree_go/MotorCmds` (12 entries) and publishes to `rt/inspire/cmd`. It can also
republish `rt/inspire/state` onto a ROS 2 topic for monitoring.

## How it maps the data

- **12-entry layout (Unitree):** `0..5 = right`, `6..11 = left`; per hand
  `[pinky, ring, middle, index, thumb_bend, thumb_rotation]` — same order our
  pipeline already uses.
- **Value convention:** our `*_cmd` is `0 = open … 1000 = closed`; the Unitree
  Inspire interface uses `q ∈ [0,1]`, `0 = close, 1 = open`. So per joint:
  `q = clamp(1.0 - cmd/1000.0, 0, 1)`.
- Only `q` is set on each `MotorCmd` (the Inspire example consumes only `q`).

## Where to run it

Everything runs **inside the `inspire_hand_teleop` container** — the same one that
runs perception. The container ships what's needed to reach the robot's Inspire DDS:

- `unitree_go` messages are **vendored** in this repo at `src/unitree_go/`.
- CycloneDDS is already installed; `docker-compose.yaml` sets
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI` →
  `docker/cyclonedds.xml`.

**You must set the network interface.** Edit `docker/cyclonedds.xml` and set
`<NetworkInterface name="...">` to the NIC on this machine that faces the robot's
network (find it with `ip addr`), then restart the container. The machine must be on
the robot's network so DDS discovery can see `rt/inspire/cmd` / `rt/inspire/state`.

## Build

```bash
# inside the inspire_hand_teleop container:
colcon build --packages-select unitree_go hand_perception_msgs g1_inspire_hand_bridge
source install/setup.bash
```

> `unitree_go` and `hand_perception_msgs` are both in this repo's `src/`, so they
> build locally — no external workspace needed.

## Run

```bash
ros2 launch g1_inspire_hand_bridge bridge.launch.xml
# override defaults as needed:
ros2 launch g1_inspire_hand_bridge bridge.launch.xml \
  ros_command_topic:=/hand_finger_angles \
  command_timeout_sec:=0.5
```

## Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `network_interface` | `""` | DDS interface (logged; DDS is configured by the unitree_ros2 bridge) |
| `ros_command_topic` | `/hand_finger_angles` | input topic (HandFingerAnglesArray) |
| `ros_state_topic` | `/g1_inspire/state` | republished hand state (HandState) |
| `dds_cmd_topic` | `rt/inspire/cmd` | Unitree command topic (output) |
| `dds_state_topic` | `rt/inspire/state` | Unitree state topic (input) |
| `publish_state` | `true` | republish robot hand state |
| `command_timeout_sec` | `0.5` | stop sending stale commands after this idle gap |
| `safe_open_on_idle` | `true` | on idle/startup, command all-open once (`q=1`) |
| `right_handedness_label` / `left_handedness_label` | `Right` / `Left` | which input entry maps to which hand |

## Safety behavior

- On startup, publishes one **all-open** frame so the hands begin at a known pose.
- If no command arrives for `command_timeout_sec`, it **stops republishing stale
  commands**; with `safe_open_on_idle` it sends one safe-open frame first.
- All `q` values are clamped to `[0, 1]`.

## Test without the camera

```bash
# Terminal A — bridge
ros2 launch g1_inspire_hand_bridge bridge.launch.xml

# Terminal B — fake commands (open | close | half | cycle)
ros2 run g1_inspire_hand_bridge test_hand_cmd_publisher --ros-args -p mode:=close
ros2 run g1_inspire_hand_bridge test_hand_cmd_publisher --ros-args -p mode:=cycle

# Terminal C — watch what goes to the robot
ros2 topic echo rt/inspire/cmd          # 12 MotorCmd, q=0 for close / q=1 for open
ros2 topic echo /g1_inspire/state       # republished robot hand state (right_q/left_q)
```

`mode:=cycle` ramps open↔close so you can confirm motion end-to-end. Stop the
publisher and watch the bridge log the idle timeout and send its safe-open frame.

## Topics & types (summary)

| Direction | Topic | Type |
|-----------|-------|------|
| in  | `/hand_finger_angles` | `hand_perception_msgs/HandFingerAnglesArray` |
| out | `rt/inspire/cmd` | `unitree_go/MotorCmds` |
| in  | `rt/inspire/state` | `unitree_go/MotorStates` |
| out | `/g1_inspire/state` | `g1_inspire_hand_bridge/HandState` |
