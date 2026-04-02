from __future__ import annotations
import os
from pathlib import Path
from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")


def _int(env_key: str, default: int) -> int:
    raw = os.getenv(env_key)
    return int(raw) if raw is not None and raw.strip() != "" else default


def _float(env_key: str, default: float) -> float:
    raw = os.getenv(env_key)
    return float(raw) if raw is not None and raw.strip() != "" else default


def _int_list(env_key: str, default: list[int] | None = None) -> list[int]:
    raw = os.getenv(env_key, "")
    if raw.strip() == "":
        return default or []
    return [int(v) for v in raw.split(",") if v.strip()]


class RobotConfig:
    # Networking and UDP
    HOST: str = os.getenv("HOST", "0.0.0.0")
    PORT: int = _int("PORT", 5005)
    BUFFER_SIZE: int = _int("BUFFER_SIZE", 4096)
    GPS_TIMEOUT: float = _float("GPS_TIMEOUT", 0.2)

    # Target navigation
    TARGET_IDS: list[int] = _int_list("TARGET_IDS", [1, 4])
    PAUSE_AT_TARGET: float = _float("PAUSE_AT_TARGET", 2.0)
    ROBOT_ID: int = _int("ROBOT_ID", 5)
    STOP_ID: int = _int("STOP_ID", 1)
    OBJECT_ID: int = _int("OBJECT_ID", 2)

    # Obstacle avoidance
    OBSTACLE_CHECK_HZ: float = _float("OBSTACLE_CHECK_HZ", 10.0)
    OBSTACLE_DIST_CM: float = _float("OBSTACLE_DIST_CM", 25.0)
    CLEAR_DIST_CM: float = _float("CLEAR_DIST_CM", 20.0)
    STOP_PAUSE: float = _float("STOP_PAUSE", 1.0)
    TURN_CHECK_INTERVAL: float = _float("TURN_CHECK_INTERVAL", 0.15)
    MAX_TURN_TIME: float = _float("MAX_TURN_TIME", 20.0)
    MIN_TURN_TIME: float = _float("MIN_TURN_TIME", 1.0)
    AVOID_SPEED: float = _float("AVOID_SPEED", 30.0)
    STEER_CENTER: int = _int("STEER_CENTER", 100)
    STEER_ADJUST: int = _int("STEER_ADJUST", 30)

    # Camera / ArUco
    CAMERA_INDEX: int = _int("CAMERA_INDEX", 0)
    CALIBRATION_FILE: str = os.getenv("CALIBRATION_FILE", "calibration.npz")
    ARUCO_DICT: str = os.getenv("ARUCO_DICT", "DICT_4X4_50")
    MARKER_SIZE_M: float = _float("MARKER_SIZE_M", 0.04)
    TARGET_MARKER_ID: int = _int("TARGET_MARKER_ID", 3)
