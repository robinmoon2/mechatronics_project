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
from aruco_gps import ArucoGPS

# ── Logging ────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("gps_client")


def build_payload(frame_id: int, positions: dict) -> bytes:
    data = {
        "timestamp": round(time.time(), 4),
        "frame":     frame_id,
        "markers": {
            str(mid): {"x": p.x, "y": p.y, "z": p.z}
            for mid, p in positions.items()
        },
    }
    return json.dumps(data).encode("utf-8")


def run():
    cfg = Config()
    gps = ArucoGPS(
        camera_matrix = cfg.CAMERA_MATRIX,
        dist_coeffs   = cfg.DIST_COEFFS,
        marker_size   = cfg.MARKER_SIZE,
        aruco_dict_id = cfg.aruco_dict,
        origin_id     = cfg.ORIGIN_ID,
    )

    cap = cv2.VideoCapture(cfg.camera_source)
    if not cap.isOpened():
        log.error("Cannot open camera source: %s", cfg.camera_source)
        return

    sock          = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_interval = 1.0 / cfg.SEND_RATE_HZ
    last_send     = 0.0
    frame_id      = 0

    log.info("Streaming to %s:%d at %d Hz", cfg.ROBOT_IP, cfg.ROBOT_PORT, cfg.SEND_RATE_HZ)
    log.info("Tracking IDs %s  |  Origin ID = %d", cfg.TRACKED_IDS, cfg.ORIGIN_ID)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                log.warning("Failed to grab frame — retrying...")
                time.sleep(0.05)
                continue

            frame_id += 1
            positions, annotated, origin_seen = gps.process_frame(frame)

            # ── Filter to tracked IDs only ────────────────────────
            tracked = {
                mid: pos
                for mid, pos in positions.items()
                if mid in cfg.TRACKED_IDS
            }

            # ── HUD ───────────────────────────────────────────────
            if not origin_seen:
                cv2.putText(annotated, "ID 0 (origin) not visible!",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)
            else:
                status = (
                    f"Tracking {list(tracked.keys())} -> "
                    f"{cfg.ROBOT_IP}:{cfg.ROBOT_PORT}"
                )
                cv2.putText(annotated, status,
                            (10, annotated.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

            # ── Send UDP packet ───────────────────────────────────
            now = time.time()
            if tracked and (now - last_send) >= send_interval:
                payload = build_payload(frame_id, tracked)
                sock.sendto(payload, (cfg.ROBOT_IP, cfg.ROBOT_PORT))
                last_send = now
                log.info("frame=%-6d sent=%s", frame_id, {
                    mid: (p.x, p.y, p.z) for mid, p in tracked.items()
                })

            cv2.imshow("ArUco GPS Client", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                log.info("Quit requested.")
                break

    except KeyboardInterrupt:
        log.info("Interrupted.")
    finally:
        cap.release()
        sock.close()
        cv2.destroyAllWindows()
        log.info("Resources released.")


if __name__ == "__main__":
    run()
