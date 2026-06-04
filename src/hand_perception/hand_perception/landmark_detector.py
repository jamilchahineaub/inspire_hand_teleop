"""MediaPipe Hands wrapper — keeps the ROS 2 node thin."""

from dataclasses import dataclass, field
from typing import List

import cv2
import mediapipe as mp
import numpy as np


@dataclass
class HandResult:
    handedness: str    # "Left" or "Right" as reported by MediaPipe
    score: float       # detection confidence 0.0–1.0
    landmarks: list    # raw mediapipe.framework.formats.landmark_pb2.NormalizedLandmarkList.landmark


class LandmarkDetector:
    """Wraps mp.solutions.hands.Hands with context-manager lifecycle.

    Usage:
        with LandmarkDetector(max_num_hands=2) as detector:
            results = detector.detect(rgb_image)
            annotated = detector.draw_on(bgr_image)
    """

    def __init__(
        self,
        max_num_hands: int = 2,
        min_detection_confidence: float = 0.7,
        min_tracking_confidence: float = 0.5,
    ):
        self._mp_hands = mp.solutions.hands
        self._mp_drawing = mp.solutions.drawing_utils
        self._mp_drawing_styles = mp.solutions.drawing_styles
        self._hands = self._mp_hands.Hands(
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._last_mp_results = None  # cached for draw_on()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def close(self):
        self._hands.close()

    def detect(self, rgb_image: np.ndarray) -> List[HandResult]:
        """Run MediaPipe on an RGB uint8 numpy array.

        Caches raw results internally so draw_on() can annotate without re-running.
        Returns a list of HandResult (empty list when no hands are detected).
        """
        self._last_mp_results = self._hands.process(rgb_image)
        output: List[HandResult] = []

        if (self._last_mp_results.multi_hand_landmarks
                and self._last_mp_results.multi_handedness):
            for hand_lm, hand_class in zip(
                    self._last_mp_results.multi_hand_landmarks,
                    self._last_mp_results.multi_handedness):
                classification = hand_class.classification[0]
                output.append(HandResult(
                    handedness=classification.label,
                    score=float(classification.score),
                    landmarks=hand_lm.landmark,
                ))

        return output

    def draw_on(self, bgr_image: np.ndarray) -> np.ndarray:
        """Draw landmarks from the most recent detect() call onto a BGR image.

        Returns a copy — does not modify the input.
        """
        annotated = bgr_image.copy()
        if (self._last_mp_results is not None
                and self._last_mp_results.multi_hand_landmarks):
            for hand_lm in self._last_mp_results.multi_hand_landmarks:
                self._mp_drawing.draw_landmarks(
                    annotated,
                    hand_lm,
                    self._mp_hands.HAND_CONNECTIONS,
                    self._mp_drawing_styles.get_default_hand_landmarks_style(),
                    self._mp_drawing_styles.get_default_hand_connections_style(),
                )
        return annotated
