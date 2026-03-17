import os
import cv2
import json
import math
import socket
import threading
import time
import logging
import numpy as np
from dotenv import load_dotenv

load_dotenv()
log = logging.getLogger("gps_client")
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")

# ── Load config from .env ───────────────────────────────────────
CAMERA_SOURCE      = int(os.getenv("CAMERA_SOURCE", 0))
MARKER_SIZE        = float(os.getenv("MARKER_SIZE", 0.10))
ARUCO_DICT_NAME    = os.getenv("ARUCO_DICT", "DICT_4X4_250")
ORIGIN_ID          = int(os.getenv("ORIGIN_ID", 0))
ROBOT_ID           = int(os.getenv("ROBOT_ID", 10))
TRACKED_IDS        = list(map(int, os.getenv("TRACKED_IDS","1,2,3").split(",")))
ROBOT_IP           = os.getenv("ROBOT_IP", "192.168.1.1")
ROBOT_PORT         = int(os.getenv("ROBOT_PORT", 5005))
CLIENT_LISTEN_PORT = int(os.getenv("CLIENT_LISTEN_PORT", 5006))
SEND_RATE_HZ       = float(os.getenv("SEND_RATE_HZ", 10))

raw = list(map(float, os.getenv("CAM_MATRIX","600,0,320,0,600,240,0,0,1").split(",")))
CAM_MATRIX  = np.array(raw, dtype=np.float64).reshape(3, 3)
DIST_COEFFS = np.array(list(map(float, os.getenv("DIST_COEFFS","0,0,0,0,0").split(","))),
                        dtype=np.float64)

ARUCO_DICT  = cv2.aruco.getPredefinedDictionary(
                  getattr(cv2.aruco, ARUCO_DICT_NAME))
DETECTOR    = cv2.aruco.ArucoDetector(ARUCO_DICT, cv2.aruco.DetectorParameters())

send_interval = 1.0 / SEND_RATE_HZ

# ── Shared state ────────────────────────────────────────────────
current_target = TRACKED_IDS[0]
target_lock    = threading.Lock()

def listen_for_commands(sock_cmd):
    """Background thread — receives set_target / mission_complete from robot."""
    global current_target
    while True:
        try:
            data, _ = sock_cmd.recvfrom(1024)
            msg = json.loads(data.decode())

            if msg.get("type") == "set_target":
                with target_lock:
                    current_target = int(msg["target_id"])
                log.info("Target updated → ID=%d", current_target)

            elif msg.get("type") == "mission_complete":
                log.info("Mission complete signal received.")

        except Exception:
            pass

def get_transform(rvec, tvec) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = tvec.flatten()
    return T

def run():
    global current_target

    cap      = cv2.VideoCapture(CAMERA_SOURCE)
    sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_cmd.bind(("0.0.0.0", CLIENT_LISTEN_PORT))
    sock_cmd.settimeout(0.01)

    threading.Thread(target=listen_for_commands,
                     args=(sock_cmd,), daemon=True).start()

    last_send = 0.0
    frame_id  = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame_id += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = DETECTOR.detectMarkers(gray)

        if ids is None:
            cv2.imshow("Client", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            continue

        ids_flat = ids.flatten().tolist()

        # ── Estimate pose for every detected marker ─────────────
        transforms = {}
        for i, mid in enumerate(ids_flat):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], MARKER_SIZE, CAM_MATRIX, DIST_COEFFS)
            transforms[mid] = get_transform(rvec, tvec)
            cv2.drawFrameAxes(frame, CAM_MATRIX, DIST_COEFFS, rvec, tvec, 0.05)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # ── Need both robot marker AND current target ────────────
        with target_lock:
            tgt = current_target

        if ROBOT_ID not in transforms or tgt not in transforms:
            cv2.imshow("Client", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            continue

        # ── Compute target position RELATIVE TO ROBOT ───────────
        T_robot_inv = np.linalg.inv(transforms[ROBOT_ID])
        T_rel       = T_robot_inv @ transforms[tgt]
        pos         = T_rel[:3, 3]

        x, y, z = round(float(pos[0]), 4), \
                  round(float(pos[1]), 4), \
                  round(float(pos[2]), 4)

        distance = math.sqrt(x**2 + y**2)
        bearing  = math.degrees(math.atan2(x, y))

        # ── Annotate frame ───────────────────────────────────────
        if tgt in ids_flat:
            idx = ids_flat.index(tgt)
            tl  = corners[idx][0][0].astype(int)
            cv2.putText(frame,
                        f"TARGET ID:{tgt} d={distance:.2f}m b={bearing:.1f}deg",
                        (tl[0], tl[1] - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # ── Send at controlled rate ──────────────────────────────
        now = time.time()
        if now - last_send >= send_interval:
            payload = json.dumps({
                "type":      "position",
                "frame":     frame_id,
                "timestamp": now,
                "target_id": tgt,
                "x":         x,       # + = target is RIGHT of robot
                "y":         y,       # + = target is AHEAD of robot
                "z":         z,
                "distance":  distance,
                "bearing":   bearing, # degrees, 0=ahead +right -left
            }).encode()

            sock_out.sendto(payload, (ROBOT_IP, ROBOT_PORT))
            last_send = now

        cv2.imshow("Client", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run()
