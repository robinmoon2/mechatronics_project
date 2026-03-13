import os
import numpy as np
from dotenv import load_dotenv

load_dotenv()

def _parse_matrix(env_key: str, shape: tuple) -> np.ndarray:
    raw = os.getenv(env_key)
    if raw is None:
        raise ValueError(f"Missing env variable: {env_key}")
    values = [float(v) for v in raw.split(",")]
    return np.array(values, dtype=np.float64).reshape(shape)

def _parse_int_list(env_key: str) -> list[int]:
    raw = os.getenv(env_key, "")
    return [int(v) for v in raw.split(",") if v.strip()]

class Config:
    # Camera
    CAMERA_SOURCE: str | int = os.getenv("CAMERA_SOURCE", "2")
    MARKER_SIZE: float       = float(os.getenv("MARKER_SIZE", "0.10"))

    # Calibration
    CAMERA_MATRIX: np.ndarray = _parse_matrix("CAM_MATRIX",   (3, 3))
    DIST_COEFFS:   np.ndarray = _parse_matrix("DIST_COEFFS",  (1, 5))

    # ArUco
    ARUCO_DICT_NAME: str = os.getenv("ARUCO_DICT", "DICT_4X4_50")
    ORIGIN_ID:       int = int(os.getenv("ORIGIN_ID", "0"))
    TRACKED_IDS: list[int] = _parse_int_list("TRACKED_IDS")

    # Network
    ROBOT_IP:      str = os.getenv("ROBOT_IP",   "192.168.1.100")
    ROBOT_PORT:    int = int(os.getenv("ROBOT_PORT",   "5005"))
    SEND_RATE_HZ:  int = int(os.getenv("SEND_RATE_HZ", "10"))

    @property
    def camera_source(self) -> int | str:
        return int(self.CAMERA_SOURCE) if str(self.CAMERA_SOURCE).isdigit() else self.CAMERA_SOURCE

    @property
    def aruco_dict(self):
        return getattr(cv2.aruco, self.ARUCO_DICT_NAME)

import cv2  # noqa: E402 (needed after property definition)
