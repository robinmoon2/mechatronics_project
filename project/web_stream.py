import cv2
import math
import numpy as np
import json
import threading
import time
from flask import Flask, Response, render_template, jsonify, request

app = Flask(__name__)

# ====== CONFIG ======
CAMERA_HFOV = 60.0
DICT_NAME = "4X4_250"

ARUCO_DICTS = {
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "4X4_100": cv2.aruco.DICT_4X4_100,
    "4X4_250": cv2.aruco.DICT_4X4_250,
    "4X4_1000": cv2.aruco.DICT_4X4_1000,
    "5X5_50": cv2.aruco.DICT_5X5_50,
    "5X5_100": cv2.aruco.DICT_5X5_100,
    "5X5_250": cv2.aruco.DICT_5X5_250,
    "5X5_1000": cv2.aruco.DICT_5X5_1000,
    "6X6_50": cv2.aruco.DICT_6X6_50,
    "6X6_100": cv2.aruco.DICT_6X6_100,
    "6X6_250": cv2.aruco.DICT_6X6_250,
    "6X6_1000": cv2.aruco.DICT_6X6_1000,
    "7X7_50": cv2.aruco.DICT_7X7_50,
    "7X7_100": cv2.aruco.DICT_7X7_100,
    "7X7_250": cv2.aruco.DICT_7X7_250,
    "7X7_1000": cv2.aruco.DICT_7X7_1000,
    "ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

# Shared state
lock = threading.Lock()
current_frame = None
current_markers = []
current_dict_name = DICT_NAME
current_hfov = CAMERA_HFOV


def compute_marker_info(corners, frame_width, frame_height, hfov_deg):
    results = []
    frame_cx = frame_width / 2.0
    focal_length_px = (frame_width / 2.0) / math.tan(math.radians(hfov_deg / 2.0))

    for corner in corners:
        pts = corner[0]
        cx = float(pts[:, 0].mean())
        cy = float(pts[:, 1].mean())
        offset_x = cx - frame_cx
        angle_deg = math.degrees(math.atan2(offset_x, focal_length_px))

        sides = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
        size_px = float(np.mean(sides))

        if abs(angle_deg) < 2.0:
            direction = "STRAIGHT"
        elif angle_deg < 0:
            direction = "LEFT"
        else:
            direction = "RIGHT"

        results.append({
            "center_px": [int(cx), int(cy)],
            "offset_px": round(offset_x, 1),
            "angle_deg": round(angle_deg, 2),
            "size_px": round(size_px, 1),
            "direction": direction,
        })
    return results


def draw_markers(frame, corners, ids, marker_infos):
    if ids is None:
        return frame

    h, w = frame.shape[:2]
    frame_cx = w // 2

    cv2.line(frame, (frame_cx, 0), (frame_cx, h), (100, 100, 100), 1)
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    for i, (corner, marker_id, info) in enumerate(zip(corners, ids.flatten(), marker_infos)):
        pts = corner[0].astype(int)
        cx, cy = info["center_px"]
        angle = info["angle_deg"]
        direction = info["direction"]

        for pt in pts:
            cv2.circle(frame, tuple(pt), 5, (0, 255, 0), -1)
        cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
        cv2.line(frame, (frame_cx, h // 2), (cx, cy), (255, 255, 0), 2)

        if direction == "STRAIGHT":
            color = (0, 255, 0)
        elif direction == "LEFT":
            color = (255, 165, 0)
        else:
            color = (0, 165, 255)

        y_offset = cy - 60
        cv2.putText(frame, f"ID:{marker_id}", (cx - 40, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"{angle:+.1f} deg", (cx - 50, y_offset + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(frame, f"{direction}", (cx - 40, y_offset + 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    return frame


def camera_loop():
    """Background thread: capture frames, detect markers, annotate."""
    global current_frame, current_markers, current_dict_name, current_hfov

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return

    # Lower resolution for faster streaming on Pi
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera: {frame_width}x{frame_height}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[current_dict_name])
    parameters = cv2.aruco.DetectorParameters()
    last_dict = current_dict_name

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        # Check if dictionary changed
        with lock:
            if current_dict_name != last_dict:
                aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[current_dict_name])
                last_dict = current_dict_name
            hfov = current_hfov

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        markers_data = []
        if ids is not None and len(ids) > 0:
            marker_infos = compute_marker_info(corners, frame_width, frame_height, hfov)
            for marker_id, info in zip(ids.flatten(), marker_infos):
                info["id"] = int(marker_id)
                markers_data.append(info)
            draw_markers(frame, corners, ids, marker_infos)

        # HUD
        n = 0 if ids is None else len(ids)
        cv2.putText(frame, f"Dict: {last_dict} | HFOV: {hfov:.0f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"Markers: {n}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Encode to JPEG
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])

        with lock:
            current_frame = jpeg.tobytes()
            current_markers = markers_data

    cap.release()


def generate_mjpeg():
    """Generator that yields MJPEG frames."""
    while True:
        with lock:
            frame = current_frame
        if frame is None:
            time.sleep(0.05)
            continue
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
        )
        time.sleep(0.03)  # ~30fps max


# ====== ROUTES ======

@app.route('/')
def index():
    return render_template('index.html', dicts=list(ARUCO_DICTS.keys()))


@app.route('/video_feed')
def video_feed():
    return Response(
        generate_mjpeg(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/markers')
def markers():
    """JSON endpoint: current marker data (for robot logic or JS)."""
    with lock:
        return jsonify(current_markers)


@app.route('/set_dict', methods=['POST'])
def set_dict():
    global current_dict_name
    name = request.json.get("dict")
    if name in ARUCO_DICTS:
        with lock:
            current_dict_name = name
        return jsonify({"ok": True, "dict": name})
    return jsonify({"ok": False}), 400


@app.route('/set_hfov', methods=['POST'])
def set_hfov():
    global current_hfov
    val = request.json.get("hfov")
    if val and 10 <= float(val) <= 180:
        with lock:
            current_hfov = float(val)
        return jsonify({"ok": True, "hfov": current_hfov})
    return jsonify({"ok": False}), 400



def main():
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()

    print("=" * 50)
    print("  Open in browser: http://<PI_IP>:5000")
    print("=" * 50)

    # 0.0.0.0 = accessible from other machines on network
    app.run(host='0.0.0.0', port=5000, threaded=True)


if __name__ == '__main__':
    main()
