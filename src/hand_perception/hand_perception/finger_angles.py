"""Finger bend angle computation from MediaPipe hand landmarks.

Geometry — aligned with Inspire DFQ hardware specification
-----------------------------------------------------------
All MediaPipe landmark coordinates are normalised:
  x ∈ [0, 1]  (left → right in image)
  y ∈ [0, 1]  (top  → bottom in image)
  z           (depth relative to wrist; smaller = closer to camera)

Metacarpal axis (reference direction per finger)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The Inspire DFQ spec defines the finger bend angle α at the MCP joint as
the angle between:
  • the metacarpal axis  = direction from wrist (0) to that finger's MCP
  • the proximal phalanx = direction from MCP to PIP

Each finger uses its OWN metacarpal direction:

  metacarpal_i = normalize( MCP_i − wrist )

This is a single subtraction — no cross-product or plane projection needed.
It is symmetric for left/right hands (no sign dependency).

Per-finger bend angle
~~~~~~~~~~~~~~~~~~~~~
  v_metacarpal = normalize( MCP − wrist )
  v_finger     = normalize( PIP  − MCP  )
  α            = degrees( arccos( clamp( dot(v_finger, v_metacarpal), −1, 1 ) ) )

DFQ hardware range (from spec):
  α = 19°    fully open  (physical rest position)
  α = 176.7° fully closed (robot mechanical limit; not reachable by a human hand)

Typical human hand range with this geometry:
  α ≈ 15°–25°   open
  α ≈ 90°–120°  tight fist at MCP level

Normalisation → Inspire DFQ command
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  x   = clamp( (α − a_min) / (a_max − a_min), 0.0, 1.0 )
  cmd = round( 100 × x )     # integer in [0, 100]

Calibrate a_min with open hand, a_max with the tightest fist you can make.
Default params: a_min = 19°, a_max = 120°.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

import numpy as np


# ---------------------------------------------------------------------------
# MediaPipe landmark index constants
# ---------------------------------------------------------------------------

WRIST_IDX       = 0
THUMB_CMC_IDX   = 1   # carpometacarpal
THUMB_MCP_IDX   = 2   # metacarpophalangeal
THUMB_IP_IDX    = 3   # interphalangeal
THUMB_TIP_IDX   = 4   # tip
INDEX_MCP_IDX   = 5
INDEX_PIP_IDX   = 6
MIDDLE_MCP_IDX  = 9
MIDDLE_PIP_IDX  = 10
RING_MCP_IDX    = 13
RING_PIP_IDX    = 14
PINKY_MCP_IDX   = 17
PINKY_PIP_IDX   = 18

# Canonical definitions for the four non-thumb fingers (name, mcp_idx, pip_idx).
# The thumb has its own two-DOF handling (bend + rotation) below, since neither
# fits the simple metacarpal-axis-vs-proximal-phalanx pattern.
FINGER_DEFS: List[tuple] = [
    ('index',  INDEX_MCP_IDX,  INDEX_PIP_IDX),
    ('middle', MIDDLE_MCP_IDX, MIDDLE_PIP_IDX),
    ('ring',   RING_MCP_IDX,   RING_PIP_IDX),
    ('pinky',  PINKY_MCP_IDX,  PINKY_PIP_IDX),
]


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class FingerConfig:
    """Per-DOF calibration configuration.

    For the four fingers, mcp_idx/pip_idx select the landmarks. The two thumb
    DOFs ('thumb_bend', 'thumb_rot') ignore those indices — they are computed by
    dedicated functions — so the indices default to -1.
    """
    name:    str
    min_deg: float   # a_min: angle at open / neutral pose
    max_deg: float   # a_max: angle at closed / fully-articulated pose
    mcp_idx: int = -1
    pip_idx: int = -1


@dataclass
class FingerAngles:
    """Computed angles for a single DOF."""
    angle_deg: float   # raw angle in degrees
    norm:      float   # normalised value in [0, 1]
    cmd:       int     # Inspire DFQ command in [0, 1000]


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _lm_to_vec(lm: Any) -> np.ndarray:
    """Convert a MediaPipe NormalizedLandmark to a 3-element float64 numpy array."""
    return np.array([lm.x, lm.y, lm.z], dtype=np.float64)


# ---------------------------------------------------------------------------
# Per-finger metacarpal axis
# ---------------------------------------------------------------------------

def compute_metacarpal_direction(lms: Any, mcp_idx: int) -> np.ndarray:
    """Unit vector from wrist (0) to this finger's MCP landmark.

    This is the metacarpal axis used by the Inspire DFQ spec as the reference
    for defining the finger bend angle α.  Each finger uses its own axis, so
    index/middle/ring/pinky are measured independently without averaging.

    Args:
        lms:     MediaPipe hand landmark list (indexable, items have .x, .y, .z).
        mcp_idx: MediaPipe index of this finger's MCP landmark.

    Returns:
        Unit vector (np.ndarray, shape (3,)) pointing from wrist toward MCP.
        Falls back to (0, −1, 0) if the two landmarks coincide.
    """
    W = _lm_to_vec(lms[WRIST_IDX])
    M = _lm_to_vec(lms[mcp_idx])
    v = M - W
    norm = float(np.linalg.norm(v))
    if norm < 1e-9:
        return np.array([0.0, -1.0, 0.0])
    return v / norm


# ---------------------------------------------------------------------------
# Per-finger bend angle
# ---------------------------------------------------------------------------

def compute_finger_bend_deg(lms: Any, mcp_idx: int, pip_idx: int) -> float:
    """Angle α in degrees between the metacarpal axis and the MCP→PIP direction.

    Matches the Inspire DFQ angle definition:
      α = arccos( dot( normalize(PIP − MCP), normalize(MCP − wrist) ) )

    DFQ spec range: 19° (open) → 176.7° (fully closed, robot limit).
    Human hand range: ~15° open, ~90°–120° tight fist.

    Returns 0.0 if MCP and PIP positions are identical (degenerate input).

    Args:
        lms:     MediaPipe landmark list.
        mcp_idx: Index of the MCP landmark.
        pip_idx: Index of the PIP landmark.

    Returns:
        Bend angle α in degrees, in [0°, 180°].
    """
    metacarpal = compute_metacarpal_direction(lms, mcp_idx)

    mcp = _lm_to_vec(lms[mcp_idx])
    pip = _lm_to_vec(lms[pip_idx])
    v = pip - mcp
    norm = float(np.linalg.norm(v))
    if norm < 1e-9:
        return 0.0

    v /= norm
    cos_a = float(np.clip(np.dot(v, metacarpal), -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_a)))


# ---------------------------------------------------------------------------
# Thumb — two independent DOFs (bend ∠θ and rotation/opposition ∠β)
# ---------------------------------------------------------------------------

def _unit(v: np.ndarray) -> Optional[np.ndarray]:
    """Normalise a vector, or return None if it is (near) zero length."""
    n = float(np.linalg.norm(v))
    return v / n if n >= 1e-9 else None


def compute_thumb_bend_deg(lms: Any) -> float:
    """Thumb flexion/curl (datasheet ∠θ), independent of opposition.

    Angle at the thumb MCP between the proximal segment (CMC→MCP) and the rest of
    the thumb (MCP→TIP):
      θ = arccos( dot( normalize(MCP − CMC), normalize(TIP − MCP) ) )

    Straight thumb → small θ; curled thumb → large θ. Uses only thumb landmarks,
    so it does not change as the thumb rotates across the palm.
    """
    cmc = _lm_to_vec(lms[THUMB_CMC_IDX])
    mcp = _lm_to_vec(lms[THUMB_MCP_IDX])
    tip = _lm_to_vec(lms[THUMB_TIP_IDX])
    a = _unit(mcp - cmc)
    b = _unit(tip - mcp)
    if a is None or b is None:
        return 0.0
    return float(np.degrees(np.arccos(float(np.clip(np.dot(a, b), -1.0, 1.0)))))


def compute_thumb_rotation_deg(lms: Any) -> float:
    """Thumb rotation/opposition (datasheet ∠β), measured on the metacarpal plane.

    Build the palm plane from wrist/index_MCP/pinky_MCP, project the thumb
    direction (CMC→TIP) onto it, and measure the in-plane angle against the index
    metacarpal (wrist→index_MCP):
      n   = normalize( (index_MCP − wrist) × (pinky_MCP − wrist) )   # palm normal
      d⊥  = normalize( (TIP − CMC) − ((TIP − CMC)·n) n )             # thumb on plane
      ref = normalize( index_MCP − wrist )
      β   = arccos( dot(d⊥, ref) )

    Thumb tucked alongside the index → small β; thumb opposed/abducted across the
    palm → large β. Independent of how far the thumb is curled.
    """
    wrist     = _lm_to_vec(lms[WRIST_IDX])
    index_mcp = _lm_to_vec(lms[INDEX_MCP_IDX])
    pinky_mcp = _lm_to_vec(lms[PINKY_MCP_IDX])
    cmc       = _lm_to_vec(lms[THUMB_CMC_IDX])
    tip       = _lm_to_vec(lms[THUMB_TIP_IDX])

    n = _unit(np.cross(index_mcp - wrist, pinky_mcp - wrist))   # palm normal
    ref = _unit(index_mcp - wrist)                              # in-plane reference
    if n is None or ref is None:
        return 0.0

    d = tip - cmc
    d_proj = _unit(d - np.dot(d, n) * n)                        # thumb on palm plane
    if d_proj is None:
        return 0.0

    return float(np.degrees(np.arccos(float(np.clip(np.dot(d_proj, ref), -1.0, 1.0)))))


# ---------------------------------------------------------------------------
# Normalisation
# ---------------------------------------------------------------------------

def normalize_angle(
    angle_deg: float,
    min_deg:   float,
    max_deg:   float,
    warn_cb:   Optional[Callable[[str], None]] = None,
) -> float:
    """Normalise a bend angle to [0, 1].

    x = clamp( (angle_deg − min_deg) / (max_deg − min_deg), 0.0, 1.0 )

    Returns 0.0 if max_deg <= min_deg.  If warn_cb is provided it is called
    with a descriptive message when the configuration is degenerate.

    Args:
        angle_deg: Measured bend angle in degrees.
        min_deg:   Calibrated open-hand angle (lower bound).
        max_deg:   Calibrated closed-hand angle (upper bound).
        warn_cb:   Optional callable(str) for logging degenerate config.

    Returns:
        Normalised float in [0.0, 1.0].
    """
    span = max_deg - min_deg
    if span <= 0.0:
        if warn_cb is not None:
            warn_cb(
                f'normalize_angle: max_deg ({max_deg:.1f}) <= min_deg ({min_deg:.1f}); '
                f'returning 0.0'
            )
        return 0.0
    return float(max(0.0, min(1.0, (angle_deg - min_deg) / span)))


# ---------------------------------------------------------------------------
# Batch computation (called once per detected hand)
# ---------------------------------------------------------------------------

def compute_all_finger_angles(
    lms:     Any,
    configs: List[FingerConfig],
    warn_cb: Optional[Callable[[str], None]] = None,
) -> Dict[str, FingerAngles]:
    """Compute angles for all configured DOFs.

    Dispatch by config name:
      * 'thumb_bend' → compute_thumb_bend_deg (flexion/curl)
      * 'thumb_rot'  → compute_thumb_rotation_deg (opposition on palm plane)
      * any other    → compute_finger_bend_deg (metacarpal axis vs MCP→PIP)

    Args:
        lms:     MediaPipe landmark list (21 items with .x, .y, .z).
        configs: List of FingerConfig (one per DOF to compute).
        warn_cb: Optional callable for degenerate-config warnings.

    Returns:
        Dict mapping DOF name → FingerAngles(angle_deg, norm, cmd).
    """
    results: Dict[str, FingerAngles] = {}

    for cfg in configs:
        if cfg.name == 'thumb_bend':
            angle = compute_thumb_bend_deg(lms)
        elif cfg.name == 'thumb_rot':
            angle = compute_thumb_rotation_deg(lms)
        else:
            angle = compute_finger_bend_deg(lms, cfg.mcp_idx, cfg.pip_idx)
        norm = normalize_angle(angle, cfg.min_deg, cfg.max_deg, warn_cb)
        cmd  = int(round(1000.0 * norm))
        results[cfg.name] = FingerAngles(angle_deg=angle, norm=norm, cmd=cmd)

    return results
