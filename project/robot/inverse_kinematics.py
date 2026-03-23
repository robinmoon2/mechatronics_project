"""
inverse_kinematics.py
Computes joint angles from a 3D target position (camera frame → arm frame).
"""
import numpy as np
from dataclasses import dataclass
from dotenv import load_dotenv
import os

load_dotenv()

L1 = float(os.getenv("LINK_SHOULDER_M", 0.105))   # shoulder → elbow
L2 = float(os.getenv("LINK_ELBOW_M",    0.105))   # elbow → wrist
L3 = float(os.getenv("LINK_WRIST_M",    0.05))    # wrist → gripper tip
H  = float(os.getenv("BASE_HEIGHT_M",   0.08))    # base height from ground


@dataclass
class JointAngles:
    base     : float   # degrees
    shoulder : float   # degrees
    elbow    : float   # degrees
    wrist    : float   # degrees
    valid    : bool
    message  : str = ""


def camera_to_arm_frame(camera_pos: np.ndarray) -> np.ndarray:
    """
    Convert camera frame (x right, y down, z forward)
    to arm base frame     (x forward, y left, z up).
    Adjust this rotation to match your physical camera mounting.
    """
    cx, cy, cz = camera_pos
    ax =  cz         # arm forward  = camera depth
    ay = -cx         # arm left     = -camera right
    az = -cy + H     # arm up       = -camera down + base height
    return np.array([ax, ay, az])


def solve_ik(camera_pos: np.ndarray) -> JointAngles:
    """
    Analytical 3-DOF IK (base rotation + 2-link planar arm + wrist).
    """
    pos = camera_to_arm_frame(camera_pos)
    ax, ay, az = pos

    # ── Base rotation ────────────────────────────────────────────
    base_rad = np.arctan2(ay, ax)
    base_deg = np.degrees(base_rad)

    # Reach in the vertical plane
    reach_horizontal = np.sqrt(ax**2 + ay**2) - L3
    reach_vertical   = az

    r = np.sqrt(reach_horizontal**2 + reach_vertical**2)

    # ── Reachability check ───────────────────────────────────────
    if r > (L1 + L2):
        return JointAngles(0, 0, 0, 0, False,
                           f"Target out of reach: r={r:.3f}m max={L1+L2:.3f}m")
    if r < abs(L1 - L2):
        return JointAngles(0, 0, 0, 0, False,
                           f"Target too close: r={r:.3f}m")

    # ── Elbow angle (law of cosines) ─────────────────────────────
    cos_elbow = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_elbow = np.clip(cos_elbow, -1.0, 1.0)
    elbow_rad = np.arccos(cos_elbow)          # elbow-up solution
    elbow_deg = np.degrees(elbow_rad)

    # ── Shoulder angle ───────────────────────────────────────────
    alpha = np.arctan2(reach_vertical, reach_horizontal)
    beta  = np.arctan2(L2 * np.sin(elbow_rad), L1 + L2 * np.cos(elbow_rad))
    shoulder_rad = alpha - beta
    shoulder_deg = np.degrees(shoulder_rad)

    # ── Wrist angle (keep gripper horizontal) ────────────────────
    wrist_deg = 90.0 - shoulder_deg - elbow_deg

    # ── Map to servo range [0, 180] ──────────────────────────────
    base_servo     = np.clip(90 + base_deg,     0, 180)
    shoulder_servo = np.clip(90 - shoulder_deg, 0, 180)
    elbow_servo    = np.clip(elbow_deg,         0, 180)
    wrist_servo    = np.clip(90 + wrist_deg,    0, 180)

    return JointAngles(
        base     = base_servo,
        shoulder = shoulder_servo,
        elbow    = elbow_servo,
        wrist    = wrist_servo,
        valid    = True,
        message  = "OK"
    )
