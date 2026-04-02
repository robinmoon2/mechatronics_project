"""
ArUco GPS core — detects markers and returns positions
relative to the origin marker (ID 0 by default).
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class MarkerPosition:
    marker_id: int
    x: float
    y: float
    z: float
    rotation: np.ndarray  # 3x3


AXIS_LENGTH = 0.05


def _build_transform(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from rvec/tvec."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = tvec.flatten()
    return T


class ArucoGPS:
    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        marker_size: float,
        aruco_dict_id: int,
        origin_id: int = 0,
        axis_length: float = AXIS_LENGTH,
    ):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_size = marker_size
        self.origin_id = origin_id
        self.axis_length = axis_length

        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    def process_frame(
        self, frame: np.ndarray
    ) -> tuple[dict[int, MarkerPosition], np.ndarray, bool]:
        """
        Detect markers and compute positions relative to origin.

        Returns
        -------
        positions   : dict {marker_id: MarkerPosition}
        annotated   : frame with drawings
        origin_seen : True if origin marker was detected
        """
        annotated = frame.copy()
        gray      = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None:
            return {}, annotated, False

        ids_flat = ids.flatten().tolist()
        transforms: dict[int, np.ndarray] = {}

        for i, marker_id in enumerate(ids_flat):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            transforms[marker_id] = _build_transform(rvec, tvec)
            cv2.drawFrameAxes(
                annotated,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.axis_length,
            )

        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

        if self.origin_id not in transforms:
            return {}, annotated, False

        # ── Origin label ─────────────────────────────────────────
        idx0 = ids_flat.index(self.origin_id)
        tl0  = corners[idx0][0][0].astype(int)
        cv2.putText(
            annotated, "ORIGIN (0,0,0)",
            (tl0[0], tl0[1] - 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2,
        )

        T_to_origin = np.linalg.inv(transforms[self.origin_id])
        positions: dict[int, MarkerPosition] = {}

        for i, marker_id in enumerate(ids_flat):
            if marker_id == self.origin_id:
                continue

            T_rel = T_to_origin @ transforms[marker_id]
            pos   = T_rel[:3, 3]
            rot   = T_rel[:3, :3]

            positions[marker_id] = MarkerPosition(
                marker_id=marker_id,
                x=round(float(pos[0]), 4),
                y=round(float(pos[1]), 4),
                z=round(float(pos[2]), 4),
                rotation=rot,
            )

            tl = corners[i][0][0].astype(int)
            cv2.putText(annotated, f"ID:{marker_id}",
                        (tl[0], tl[1] - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
            cv2.putText(
                annotated,
                f"x={pos[0]:.2f} y={pos[1]:.2f} z={pos[2]:.2f}",
                (tl[0], tl[1] - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1,
            )

        return positions, annotated, True
