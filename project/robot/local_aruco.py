"""
local_approach.py — Use the robot's onboard camera to detect an ArUco marker,
drive toward it using the Navigation class, grab with servo ch2, then return.
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import time
import logging

from navigator import GPS_coordinates, Navigation, STEER_CENTER

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("local_approach")

# ── CONFIG ─────────────────────────────────────────────────────
MARKER_SIZE      = 0.04       # meters — measure your printed marker
CAMERA_INDEX     = 0          # /dev/video0
FRAME_W          = 640
FRAME_H          = 480
STOP_DISTANCE    = 0.10       # 10 cm
SLOW_DISTANCE    = 0.25
SPEED_FAST       = 30
SPEED_SLOW       = 20
KP_STEER         = 0.15       # pixel error → steering degrees
LOST_TIMEOUT     = 3.0        # seconds before giving up
GRAB_ANGLE       = 120
REST_ANGLE       = 90
GRAB_HOLD_TIME   = 2.0

# ── Camera calibration (replace with yours!) ───────────────────
CAMERA_MATRIX = np.array([
    [640,   0, 320],
    [  0, 640, 240],
    [  0,   0,   1],
], dtype=np.float32)
DIST_COEFFS = np.zeros(5, dtype=np.float32)

# ── ArUco ──────────────────────────────────────────────────────
ARUCO_DICT   = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
ARUCO_PARAMS = aruco.DetectorParameters()
DETECTOR     = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)


def detect_marker(frame, target_id):
    """
    Returns (center_x_pixel, distance_meters) or (None, None).
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = DETECTOR.detectMarkers(gray)

    if ids is None:
        return None, None

    for i, mid in enumerate(ids.flatten()):
        if mid == target_id:
            c = corners[i][0]
            cx = float(np.mean(c[:, 0]))

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS
            )
            tx, ty, tz = tvecs[0][0]
            distance = float(np.sqrt(tx**2 +     ty**2 + tz**2))
            return cx, distance

    return None, None


def compute_local_steering(cx, frame_width):
    """Pixel offset from center → steering servo angle."""
    center = frame_width / 2.0
    error = cx - center
    steer = STEER_CENTER + KP_STEER * error
    return max(STEER_CENTER - 20, min(STEER_CENTER + 20, steer))


def approach_and_grab(nav: Navigation, target_id: int, camera_index: int = CAMERA_INDEX) -> bool:
    """
    Use the onboard camera to approach target_id.
    When within 10 cm: stop, ch2 → 120°, wait 2s, ch2 → 90°.
    Returns True if successful, False if marker lost.
    
    Uses the existing Navigation object for motor/servo control.
    """
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        log.error("[LOCAL] Cannot open camera %d", camera_index)
        return False

    log.info("[LOCAL] Camera opened — approaching marker ID %d", target_id)

    last_seen_time = time.time()
    reached = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                log.warning("[LOCAL] Frame grab failed")
                time.sleep(0.05)
                continue

            cx, distance = detect_marker(frame, target_id)

            if cx is not None and distance is not None:
                last_seen_time = time.time()
                log.info("[LOCAL] ID=%d  cx=%.0f  dist=%.3f m", target_id, cx, distance)

                # ── Close enough → STOP + GRAB ──
                if distance <= STOP_DISTANCE:
                    nav.driver.stop()
                    nav.set_steering(STEER_CENTER)
                    log.info("[LOCAL] Reached ID %d at %.1f cm — grabbing!",
                             target_id, distance * 100)

                    log.info("[LOCAL] Servo ch2 → %d°", GRAB_ANGLE)
                    nav.shoulder_servo.angle = GRAB_ANGLE
                    time.sleep(GRAB_HOLD_TIME)

                    log.info("[LOCAL] Servo ch2 → %d°", REST_ANGLE)
                    nav.shoulder_servo.angle = REST_ANGLE
                    time.sleep(0.5)

                    reached = True
                    break

                # ── Steer toward marker ──
                steer = compute_local_steering(cx, FRAME_W)
                nav.set_steering(steer)

                # ── Speed based on distance ──
                if distance < SLOW_DISTANCE:
                    nav.driver.forward(SPEED_SLOW)
                else:
                    nav.driver.forward(SPEED_FAST)

            else:
                # Marker not visible
                elapsed = time.time() - last_seen_time
                if elapsed > LOST_TIMEOUT:
                    nav.driver.stop()
                    nav.set_steering(STEER_CENTER)
                    log.warning("[LOCAL] Lost ID %d for %.1fs — aborting",
                                target_id, elapsed)
                    reached = False
                    break
                else:
                    # Stop and wait
                    nav.driver.stop()
                    log.debug("[LOCAL] Marker not visible — waiting (%.1fs)", elapsed)

            time.sleep(0.05)  # ~20 Hz

    except KeyboardInterrupt:
        log.info("[LOCAL] Interrupted")
        nav.driver.stop()
        reached = False

    finally:
        cap.release()
        nav.driver.stop()
        log.info("[LOCAL] Camera released")

    return reached
