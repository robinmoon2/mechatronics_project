"""
aruco_target_detector.py
Detects the target object by its ArUco marker ID (TARGET_MARKER_ID from .env)
Returns the 3D position relative to the camera.
"""
import cv2
import numpy as np
from dataclasses import dataclass
from dotenv import load_dotenv
import os

load_dotenv()

TARGET_MARKER_ID = int(os.getenv("TARGET_MARKER_ID", 10))
MARKER_SIZE_M    = float(os.getenv("MARKER_SIZE_M", 0.04))
ARUCO_DICT_NAME  = os.getenv("ARUCO_DICT", "DICT_4X4_50")


@dataclass
class DetectedTarget:
    marker_id : int
    position  : np.ndarray   # (x, y, z) meters in camera frame
    rvec      : np.ndarray
    tvec      : np.ndarray
    distance  : float
    corners   : np.ndarray


class ArucoTargetDetector:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        self.camera_matrix = camera_matrix
        self.dist_coeffs   = dist_coeffs

        aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, ARUCO_DICT_NAME)
        )
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin   = 3
        params.adaptiveThreshWinSizeMax   = 53
        params.adaptiveThreshWinSizeStep  = 4
        params.minMarkerPerimeterRate     = 0.02
        params.cornerRefinementMethod     = cv2.aruco.CORNER_REFINE_SUBPIX

        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    def detect(self, frame: np.ndarray) -> DetectedTarget | None:
        """Returns the target if marker ID matches TARGET_MARKER_ID, else None."""
        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        scale  = 2
        gray_up = cv2.resize(gray, None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_CUBIC)

        corners, ids, _ = self.detector.detectMarkers(gray_up)

        if ids is None:
            return None

        corners = [c / scale for c in corners]

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id != TARGET_MARKER_ID:
                continue

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], MARKER_SIZE_M,
                self.camera_matrix, self.dist_coeffs
            )

            tvec_flat = tvec[0][0]
            distance  = float(np.linalg.norm(tvec_flat))

            return DetectedTarget(
                marker_id = int(marker_id),
                position  = tvec_flat,
                rvec      = rvec[0][0],
                tvec      = tvec_flat,
                distance  = distance,
                corners   = corners[i]
            )

        return None

    def draw(self, frame: np.ndarray, target: DetectedTarget | None) -> np.ndarray:
        out = frame.copy()
        if target is None:
            cv2.putText(out, "Target NOT found", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return out

        cv2.aruco.drawDetectedMarkers(out, [target.corners])
        cv2.drawFrameAxes(out, self.camera_matrix, self.dist_coeffs,
                          target.rvec, target.tvec, MARKER_SIZE_M)

        x, y, z = target.position
        cv2.putText(out,
            f"Target ID={target.marker_id} | "
            f"x={x:.3f} y={y:.3f} z={z:.3f}m | "
            f"dist={target.distance:.3f}m",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
        )
        return out
