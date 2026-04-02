"""
config.py — Central configuration loaded from .env
"""

from __future__ import annotations

import os
import cv2
import numpy as np
from dotenv import load_dotenv

load_dotenv()


def _parse_matrix(env_key: str, shape: tuple[int, int]) -> np.ndarray:
    raw = os.getenv(env_key)
    if raw is None:
        raise ValueError(f"Missing env variable: {env_key}")
    values = [float(v) for v in raw.split(",") if v.strip()]
    matrix = np.array(values, dtype=np.float64)
    if matrix.size != shape[0] * shape[1]:
        raise ValueError(
            f"Invalid matrix size for {env_key}: expected {shape}, got {matrix.shape}"
        )
    return matrix.reshape(shape)


def _parse_int_list(env_key: str) -> list[int]:
    raw = os.getenv(env_key, "")
    return [int(v) for v in raw.split(",") if v.strip()]


class Config:
    # Camera
    CAMERA_SOURCE: str | int = os.getenv("CAMERA_SOURCE", "0")
    MARKER_SIZE: float = float(os.getenv("MARKER_SIZE", "0.10"))

    # Calibration
    CAMERA_MATRIX: np.ndarray = _parse_matrix("CAM_MATRIX", (3, 3))
    DIST_COEFFS: np.ndarray = _parse_matrix("DIST_COEFFS", (1, 5))

    # ArUco
    ARUCO_DICT_NAME: str = os.getenv("ARUCO_DICT", "DICT_4X4_50")
    ORIGIN_ID: int = int(os.getenv("ORIGIN_ID", "0"))
    ROBOT_ID: int = int(os.getenv("ROBOT_ID", "10"))
    TRACKED_IDS: list[int] = _parse_int_list("TRACKED_IDS")

    # Networking
    ROBOT_IP: str = os.getenv("ROBOT_IP", "192.168.1.100")
    ROBOT_PORT: int = int(os.getenv("ROBOT_PORT", "5005"))
    SEND_RATE_HZ: int = int(os.getenv("SEND_RATE_HZ", "10"))

    # Tracking
    STALE_TIMEOUT: float = float(os.getenv("STALE_TIMEOUT", "0.5"))

    # UI and debugging
    DRAW_AXIS_LENGTH: float = float(os.getenv("DRAW_AXIS_LENGTH", "0.05"))

    @property
    def camera_source(self) -> int | str:
        source = str(self.CAMERA_SOURCE).strip()
        return int(source) if source.isdigit() else source

    @property
    def aruco_dict(self) -> int:
        return getattr(cv2.aruco, self.ARUCO_DICT_NAME)

    @property
    def tracked_ids(self) -> list[int]:
        return list(self.TRACKED_IDS)

