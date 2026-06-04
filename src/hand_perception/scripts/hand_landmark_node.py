#!/usr/bin/env python3
"""ROS 2 hand landmark detection node.

Subscribes to a camera image topic, runs MediaPipe Hands, and publishes:
  /hand_landmarks             — hand_perception_msgs/HandLandmarksArray
  /hand_finger_angles         — hand_perception_msgs/HandFingerAnglesArray
  /hand_landmarks_debug_image — sensor_msgs/Image  (if publish_debug_image=true)

Both array topics are produced from the same per-hand loop, so hands[i] in
/hand_landmarks and hands[i] in /hand_finger_angles always refer to the same
detected hand instance in the same frame.

Landmark coordinates (x, y ∈ [0,1]; z = wrist-relative depth).
An empty hands[] array is published on both topics when no hands are detected.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

_IMAGE_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage

from hand_perception.landmark_detector import LandmarkDetector
from hand_perception.finger_angles import (
    FingerConfig,
    compute_all_finger_angles,
    FINGER_DEFS,
)
from hand_perception_msgs.msg import (
    HandLandmarks,
    HandLandmarksArray,
    HandFingerAngles,
    HandFingerAnglesArray,
)

# MCP landmark indices used for debug-image text placement
_MCP_IDX = {'index': 5, 'middle': 9, 'ring': 13, 'pinky': 17,
            'thumb_bend': 2, 'thumb_rot': 1}

_HANDEDNESS_FLIP = {'Left': 'Right', 'Right': 'Left'}

def _correct_handedness(label: str, flip: bool) -> str:
    """Flip 'Left'↔'Right' when the camera is not mirrored.

    MediaPipe assumes a mirrored (selfie) camera.  Non-mirrored cameras
    (ZED, RealSense, most robotics cameras) produce the correct spatial image
    but MediaPipe labels the hands backwards.  Set flip_handedness=true to
    correct this.
    """
    return _HANDEDNESS_FLIP.get(label, label) if flip else label


def _overlay_angles(annotated, results, per_hand_angles, img_h, img_w):
    """Draw per-finger angle / norm / cmd near each MCP landmark.

    Args:
        annotated:       BGR image (modified in-place).
        results:         List[HandResult] from LandmarkDetector.detect().
        per_hand_angles: List[Dict[str, FingerAngles]] aligned with results.
        img_h, img_w:    Image dimensions for denormalising landmark coords.
    """
    for hand_result, angles in zip(results, per_hand_angles):
        lms = hand_result.landmarks
        for finger, mcp_idx in _MCP_IDX.items():
            fa = angles.get(finger)
            if fa is None:
                continue
            lm = lms[mcp_idx]
            px = int(lm.x * img_w)
            py = int(lm.y * img_h)
            label = f"{fa.angle_deg:.0f}d {fa.norm:.2f} [{fa.cmd}]"
            cv2.putText(
                annotated, label,
                (px, max(py - 8, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 80), 1, cv2.LINE_AA,
            )
    return annotated


class HandLandmarkNode(Node):
    def __init__(self):
        super().__init__('hand_landmark_node')

        # ── Core parameters ───────────────────────────────────────────────────
        self.declare_parameter('image_topic',              '/image')
        self.declare_parameter('max_num_hands',            2)
        self.declare_parameter('min_detection_confidence', 0.7)
        self.declare_parameter('min_tracking_confidence',  0.5)
        self.declare_parameter('publish_debug_image',      True)
        self.declare_parameter('compressed_image',         False)
        # True for non-mirrored cameras (ZED, RealSense, most robotics cameras).
        # MediaPipe assumes a mirrored selfie feed; without this correction the
        # Left and Right hand labels are swapped.
        self.declare_parameter('flip_handedness',          True)

        image_topic        = self.get_parameter('image_topic').value
        max_num_hands      = self.get_parameter('max_num_hands').value
        min_det_conf       = self.get_parameter('min_detection_confidence').value
        min_trk_conf       = self.get_parameter('min_tracking_confidence').value
        self._pub_debug      = self.get_parameter('publish_debug_image').value
        self._flip_hand      = self.get_parameter('flip_handedness').value
        self._compressed     = self.get_parameter('compressed_image').value

        # ── Finger angle calibration parameters ───────────────────────────────
        # One pair (min/max) per DOF.  The four fingers plus the two thumb DOFs
        # (thumb_bend = curl, thumb_rot = opposition).  Defaults are conservative
        # estimates; calibrate by reading /hand_finger_angles open vs articulated.
        for dof in ('index', 'middle', 'ring', 'pinky', 'thumb_bend', 'thumb_rot'):
            self.declare_parameter(f'{dof}_min_angle_deg', 20.0)
            self.declare_parameter(f'{dof}_max_angle_deg', 90.0)

        def _make_config(name, mcp=-1, pip=-1):
            lo = float(self.get_parameter(f'{name}_min_angle_deg').value)
            hi = float(self.get_parameter(f'{name}_max_angle_deg').value)
            if hi <= lo:
                self.get_logger().warn(
                    f'DOF "{name}": max_angle_deg ({hi}) <= min_angle_deg ({lo}); '
                    f'norm/cmd will always be 0'
                )
            return FingerConfig(name=name, min_deg=lo, max_deg=hi, mcp_idx=mcp, pip_idx=pip)

        # Four fingers (landmark-pair based) + the two thumb DOFs (dedicated geometry).
        self._finger_configs: list[FingerConfig] = [
            _make_config(name, mcp, pip) for name, mcp, pip in FINGER_DEFS
        ]
        self._finger_configs.append(_make_config('thumb_bend'))
        self._finger_configs.append(_make_config('thumb_rot'))

        # ── MediaPipe detector ─────────────────────────────────────────────────
        self._detector = LandmarkDetector(
            max_num_hands=max_num_hands,
            min_detection_confidence=min_det_conf,
            min_tracking_confidence=min_trk_conf,
        )

        # ── cv_bridge ─────────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── Subscriber ────────────────────────────────────────────────────────
        # qos_profile_sensor_data = BEST_EFFORT + VOLATILE + depth=5.
        # Accepts both BEST_EFFORT and RELIABLE publishers without QoS warnings.
        if self._compressed:
            self.create_subscription(CompressedImage, image_topic, self._image_cb, _IMAGE_SUB_QOS)
        else:
            self.create_subscription(Image, image_topic, self._image_cb, _IMAGE_SUB_QOS)

        # ── Publishers ────────────────────────────────────────────────────────
        self._landmarks_pub = self.create_publisher(
            HandLandmarksArray, '/hand_landmarks', qos_profile_sensor_data)

        self._angles_pub = self.create_publisher(
            HandFingerAnglesArray, '/hand_finger_angles', qos_profile_sensor_data)

        self._debug_pub = None
        if self._pub_debug:
            self._debug_pub = self.create_publisher(
                Image, '/hand_landmarks_debug_image', qos_profile_sensor_data)

        # ── Startup log ───────────────────────────────────────────────────────
        self.get_logger().info(
            f'hand_landmark_node ready\n'
            f'  subscribed to   : {image_topic}\n'
            f'  max_hands       : {max_num_hands}\n'
            f'  det_conf        : {min_det_conf}  trk_conf: {min_trk_conf}\n'
            f'  debug_image     : {self._pub_debug}\n'
            f'  flip_handedness : {self._flip_hand}  '
            f'(True = non-mirrored camera, e.g. ZED)\n'
            f'  finger angle calibration (open → closed):'
        )
        for cfg in self._finger_configs:
            self.get_logger().info(f'    {cfg.name:6s}: {cfg.min_deg:.1f}° → {cfg.max_deg:.1f}°')

    # ── Callback ──────────────────────────────────────────────────────────────

    def _image_cb(self, msg) -> None:
        try:
            if self._compressed:
                bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    self.get_logger().error('Failed to decode compressed image')
                    return
            else:
                bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        rgb     = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        results = self._detector.detect(rgb)

        # ── Both output arrays are built from the same loop ────────────────
        # Invariant: out_lm.hands[i] and out_angles.hands[i] always refer to
        # the same detected hand in the same frame.
        out_lm              = HandLandmarksArray()
        out_angles          = HandFingerAnglesArray()
        out_lm.header       = msg.header
        out_angles.header   = msg.header
        per_hand_angles     = []  # kept for debug overlay

        right_seen = False
        for r in results:
            handedness = _correct_handedness(r.handedness, self._flip_hand)

            # Keep only the first Right hand per frame. MediaPipe occasionally
            # reports a second spurious Right hand which doubles the per-frame
            # work and adds latency; downstream consumers only use one Right hand.
            if handedness == 'Right':
                if right_seen:
                    continue
                right_seen = True

            # ── HandLandmarks entry ────────────────────────────────────────
            hand_lm             = HandLandmarks()
            hand_lm.header      = msg.header
            hand_lm.handedness  = handedness
            hand_lm.score       = r.score
            for lm in r.landmarks:
                pt = Point()
                pt.x = float(lm.x)
                pt.y = float(lm.y)
                pt.z = float(lm.z)
                hand_lm.points.append(pt)
            out_lm.hands.append(hand_lm)

            # ── HandFingerAngles entry (same r.landmarks) ──────────────────
            angles = compute_all_finger_angles(
                r.landmarks,
                self._finger_configs,
                warn_cb=lambda msg_str: self.get_logger().warn(msg_str),
            )
            per_hand_angles.append(angles)

            ha              = HandFingerAngles()
            ha.header       = msg.header
            ha.handedness   = handedness
            ha.score        = r.score
            for f in ('index', 'middle', 'ring', 'pinky', 'thumb_bend', 'thumb_rot'):
                fa = angles[f]
                setattr(ha, f'{f}_angle_deg', float(fa.angle_deg))
                setattr(ha, f'{f}_norm',      float(fa.norm))
                setattr(ha, f'{f}_cmd',       int(fa.cmd))
            out_angles.hands.append(ha)

        self._landmarks_pub.publish(out_lm)
        self._angles_pub.publish(out_angles)

        # ── Debug image: skeleton + angle overlay ──────────────────────────
        if self._debug_pub is not None:
            annotated = self._detector.draw_on(bgr)
            if per_hand_angles:
                img_h, img_w = annotated.shape[:2]
                annotated = _overlay_angles(
                    annotated, results, per_hand_angles, img_h, img_w)
            try:
                debug_msg        = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                debug_msg.header = msg.header
                self._debug_pub.publish(debug_msg)
            except CvBridgeError as exc:
                self.get_logger().error(f'cv_bridge error (debug image): {exc}')

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandLandmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
