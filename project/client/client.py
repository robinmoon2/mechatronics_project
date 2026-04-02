"""
GPS Client — captures frames, detects ArUco markers,
and streams positions to the robot over UDP.
"""

import cv2
import json
import logging
import socket
import time

from config import Config
from aruco_gps import ArucoGPS, MarkerPosition

# ── Logging ────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("gps_client")

def build_payload(frame_id: int, positions: dict[int, "MarkerPosition"]) -> bytes:
    data = {
        "timestamp": round(time.time(), 4),
        "frame": frame_id,
        "markers": {
            str(mid): {"x": p.x, "y": p.y, "z": p.z}
            for mid, p in positions.items()
        },
    }
    return json.dumps(data).encode("utf-8")

def run():
    cfg = Config()
    gps = ArucoGPS(
        camera_matrix=cfg.CAMERA_MATRIX,
        dist_coeffs=cfg.DIST_COEFFS,
        marker_size=cfg.MARKER_SIZE,
        aruco_dict_id=cfg.aruco_dict,
        origin_id=cfg.ORIGIN_ID,
        axis_length=cfg.DRAW_AXIS_LENGTH,
    )

    cap = cv2.VideoCapture(cfg.camera_source)
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)    # 1 = manual mode
    # cap.set(cv2.CAP_PROP_EXPOSURE, -6)        # lower = darker, try -10 to -13
    # cap.set(cv2.CAP_PROP_BRIGHTNESS, 100)       # default ~128, lower it
    # cap.set(cv2.CAP_PROP_GAIN, 0)             # minimize gain


    sock          = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_interval = 1.0 / cfg.SEND_RATE_HZ
    last_send     = 0.0
    frame_id      = 0

    # ── Last known positions (used when marker lost) ──────────
    last_known = {}  # {marker_id: (MarkerPosition, timestamp)}
    stale_timeout = cfg.STALE_TIMEOUT

    try:
        while True:
            # Drain buffer — always process freshest frame
            for _ in range(3):
                cap.grab()
            ret, frame = cap.retrieve()

            if not ret:
                continue

            frame_id += 1
            positions, annotated, origin_seen = gps.process_frame(frame)

            tracked = {
                mid: pos
                for mid, pos in positions.items()
                if mid in cfg.TRACKED_IDS
            }

            now = time.time()

            # ── Update last known ─────────────────────────────
            for mid, pos in tracked.items():
                last_known[mid] = (pos, now)

            # ── Fill missing with stale data ──────────────────
            effective = dict(tracked)
            for mid in cfg.tracked_ids:
                if mid not in effective and mid in last_known:
                    pos, ts = last_known[mid]
                    if now - ts < stale_timeout:
                        effective[mid] = pos
                        # Visual indicator
                        cv2.putText(
                            annotated,
                            f"ID{mid} STALE",
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 165, 255),
                            2,
                        )

            # ── Send ──────────────────────────────────────────
            if effective and (now - last_send) >= send_interval:
                payload = build_payload(frame_id, effective)
                sock.sendto(payload, (cfg.ROBOT_IP, cfg.ROBOT_PORT))
                last_send = now

            cv2.imshow("ArUco GPS Client", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        log.info("Interrupted.")
    finally:
        cap.release()
        sock.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    run()
