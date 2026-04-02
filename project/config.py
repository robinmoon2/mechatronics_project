"""config.py — Loads .env and exposes typed configuration."""

import os
import numpy as np
from dotenv import load_dotenv
import cv2

load_dotenv()

class Config:
    def __init__(self):
        self.camera_source = int(os.getenv("CAMERA_SOURCE", "0"))
        self.MARKER_SIZE = float(os.getenv("MARKER_SIZE", "0.04"))

        # Camera calibration
        cam = list(map(float, os.getenv("CAM_MATRIX").split(",")))
        self.CAMERA_MATRIX = np.array(cam).reshape(3, 3)
        self.DIST_COEFFS = np.array(list(map(float, os.getenv("DIST_COEFFS").split(","))))

        # ArUco
        dict_name = os.getenv("ARUCO_DICT", "DICT_4X4_250")
        self.aruco_dict = getattr(cv2.aruco, dict_name)
        self.ORIGIN_ID = int(os.getenv("ORIGIN_ID", "0"))
        self.ROBOT_ID = int(os.getenv("ROBOT_ID", "10"))
        self.TRACKED_IDS = set(map(int, os.getenv("TRACKED_IDS", "").split(",")))

        # Network
        self.ROBOT_IP = os.getenv("ROBOT_IP", "192.168.137.212")
        self.ROBOT_PORT = int(os.getenv("ROBOT_PORT", "5005"))
        self.CLIENT_LISTEN_PORT = int(os.getenv("CLIENT_LISTEN_PORT", "5006"))
        self.SEND_RATE_HZ = int(os.getenv("SEND_RATE_HZ", "10"))
