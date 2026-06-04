#!/usr/bin/env python3
"""RS485 bridge to the Inspire RH56 dexterous hand.

Subscribes to /hand_finger_angles (from the MediaPipe perception node), reorders
the per-finger commands into the RH56 hardware DOF order, and:

  • ALWAYS publishes /hand_command_angles  (the 6 ints we are commanding) — works
    with no hardware, so the command path can be developed/visualised today.
  • If the serial port is open, writes ANGLE_SET to the hand.
  • Publishes /hand_actual_angles (ANGLE_ACT feedback) at 20 Hz when connected.
  • Publishes /hand_status and /hand_error at 2 Hz so stalls/faults are visible.

All hardware I/O goes through the field-tested InspireHand driver
(hand_perception.inspire_hand_driver) — same code proven in the standalone
diagnostic script, so there is no protocol guesswork here.

Hardware is OPTIONAL. If the serial port cannot be opened the node logs one
warning and runs in "command-publish-only" mode, retrying every
reconnect_period_s seconds. Nothing crashes; the MuJoCo sim is unaffected.

RH56 DOF order (ANGLE_SET / ANGLE_ACT register layout):
    [0] pinky  [1] ring  [2] middle  [3] index  [4] thumb_bend  [5] thumb_rot
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray

from hand_perception_msgs.msg import HandFingerAnglesArray
from hand_perception.inspire_hand_driver import InspireHand, InspireHandError


# Our own output topics are RELIABLE so command consumers don't drop frames.
_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# The perception node publishes /hand_finger_angles BEST_EFFORT (sensor data); a
# RELIABLE subscriber is incompatible and receives nothing, so match it here.
_SENSOR_QOS = qos_profile_sensor_data


class RS485HandNode(Node):
    def __init__(self):
        super().__init__('rs485_hand_node')

        self.declare_parameter('serial_port',         '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',           115200)
        self.declare_parameter('hand_id',             2)      # tested hardware ships as ID 2
        self.declare_parameter('target_handedness',  'Right') # which human hand drives the robot
        self.declare_parameter('thumb_rotation_cmd',  500)
        self.declare_parameter('publish_feedback',    True)
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('clear_error_on_start', False)
        self.declare_parameter('reconnect_period_s',  5.0)
        # RH56 ANGLE_SET convention is 1000=open, 0=closed
        self.declare_parameter('invert_command', True)

        self._port_name   = self.get_parameter('serial_port').value
        self._baud        = int(self.get_parameter('baud_rate').value)
        self._hand_id     = int(self.get_parameter('hand_id').value)
        self._target_hand = str(self.get_parameter('target_handedness').value)
        self._thumb_rot   = int(self.get_parameter('thumb_rotation_cmd').value)
        self._pub_fb      = bool(self.get_parameter('publish_feedback').value)
        self._pub_diag    = bool(self.get_parameter('publish_diagnostics').value)
        self._clear_start = bool(self.get_parameter('clear_error_on_start').value)
        self._invert      = bool(self.get_parameter('invert_command').value)
        self._reconnect_s = float(self.get_parameter('reconnect_period_s').value)

        self._hand = None
        self._warned_no_port = False

        # Publishers — command always available; the rest only when hardware talks.
        self._cmd_pub = self.create_publisher(
            Int32MultiArray, '/hand_command_angles', _RELIABLE_QOS)
        self._act_pub = self.create_publisher(
            Int32MultiArray, '/hand_actual_angles', _RELIABLE_QOS)
        self._status_pub = self.create_publisher(
            Int32MultiArray, '/hand_status', _RELIABLE_QOS)
        self._error_pub = self.create_publisher(
            Int32MultiArray, '/hand_error', _RELIABLE_QOS)

        self.create_subscription(
            HandFingerAnglesArray, '/hand_finger_angles', self._cb, _SENSOR_QOS)

        # Open the port now; keep retrying if it isn't there yet.
        self._try_open()
        self.create_timer(self._reconnect_s, self._reconnect_if_needed)

        if self._pub_fb:
            self.create_timer(0.05, self._read_feedback)     # 20 Hz
        if self._pub_diag:
            self.create_timer(0.5, self._read_diagnostics)   # 2 Hz

        self.get_logger().info(
            f'rs485_hand_node ready\n'
            f'  serial_port      : {self._port_name} @ {self._baud}\n'
            f'  hand_id          : {self._hand_id}\n'
            f'  thumb_rotation   : {self._thumb_rot} (fixed)\n'
            f'  publish_feedback : {self._pub_fb}\n'
            f'  publish_diag     : {self._pub_diag}\n'
            f'  DOF order        : [pinky ring middle index thumb_bend thumb_rot]'
        )

    # ── Serial lifecycle ───────────────────────────────────────────────────

    def _try_open(self) -> None:
        try:
            self._hand = InspireHand(
                port=self._port_name,
                baudrate=self._baud,
                hand_id=self._hand_id,
            )
            self.get_logger().info(
                f'Opened RH56 hand on {self._port_name} (id {self._hand_id})')
            self._warned_no_port = False
            if self._clear_start:
                try:
                    self._hand.clear_error()
                    self.get_logger().info('CLEAR_ERROR sent on connect')
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'clear_error on connect failed: {exc}')
        except Exception as exc:  # noqa: BLE001 — pyserial errors vary by platform
            self._hand = None
            if not self._warned_no_port:
                self.get_logger().warn(
                    f'serial port {self._port_name} not available ({exc}) — '
                    f'command-publish-only mode. Retrying every '
                    f'{self._reconnect_s:.0f}s.')
                self._warned_no_port = True

    def _reconnect_if_needed(self) -> None:
        if self._hand is None:
            self._try_open()

    def _drop_hand(self, why: str) -> None:
        self.get_logger().warn(f'{why}; dropping serial, will retry')
        try:
            if self._hand is not None:
                self._hand.close()
        except Exception:  # noqa: BLE001
            pass
        self._hand = None
        self._warned_no_port = False

    # ── Subscription: build command, publish, write to hardware ────────────

    def _cb(self, msg: HandFingerAnglesArray) -> None:
        hand = next((h for h in msg.hands if h.handedness == self._target_hand), None)
        if hand is None:
            return

        # Thumb rotation is now measured; fall back to the fixed param only if the
        # field is absent/zero (e.g. replaying an older bag).
        thumb_rot = int(getattr(hand, 'thumb_rot_cmd', 0)) or int(self._thumb_rot)
        values = [
            int(hand.pinky_cmd),       # ANGLE_SET(0)
            int(hand.ring_cmd),        # ANGLE_SET(1)
            int(hand.middle_cmd),      # ANGLE_SET(2)
            int(hand.index_cmd),       # ANGLE_SET(3)
            int(hand.thumb_bend_cmd),  # ANGLE_SET(4)
            thumb_rot,                 # ANGLE_SET(5) — measured opposition
        ]

        # Publish the raw teleop command (0=open, 1000=closed) — matches the sim.
        self._cmd_pub.publish(Int32MultiArray(data=values))

        if self._hand is not None:
            # RH56 hardware uses the opposite scale (1000=open, 0=closed).
            hw_values = [1000 - v for v in values] if self._invert else values
            try:
                self._hand.set_position_raw(hw_values)
            except (InspireHandError, OSError) as exc:
                self._drop_hand(f'serial write failed ({exc})')

    # ── Timers: read feedback / diagnostics ────────────────────────────────

    def _read_feedback(self) -> None:
        if self._hand is None:
            return
        try:
            actual = self._hand.get_position_raw()
        except (InspireHandError, OSError) as exc:
            self._drop_hand(f'ANGLE_ACT read failed ({exc})')
            return
        self._act_pub.publish(Int32MultiArray(data=[int(v) for v in actual]))

    def _read_diagnostics(self) -> None:
        if self._hand is None:
            return
        try:
            status = self._hand.get_status_codes()
            error = self._hand.get_error_codes()
        except (InspireHandError, OSError) as exc:
            self._drop_hand(f'STATUS/ERROR read failed ({exc})')
            return
        self._status_pub.publish(Int32MultiArray(data=[int(v) for v in status]))
        self._error_pub.publish(Int32MultiArray(data=[int(v) for v in error]))

    def destroy_node(self) -> None:
        if self._hand is not None:
            try:
                self._hand.close()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RS485HandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
