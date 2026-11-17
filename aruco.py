import cv2
import cv2.aruco as aruco
import numpy as np
import os
import math

os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"

CAMERA_MATRIX = np.array([
    [800,   0, 320],
    [  0, 800, 240],
    [  0,   0,   1]
], dtype=np.float32)

DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)

MARKER_SIZE_CM = 5.0

def rvec_to_euler(rvec):
    """Convert rotation vector to Roll, Pitch, Yaw in degrees"""
    R, _ = cv2.Rodrigues(rvec)

    # Roll (rotation around Z)
    roll  = math.atan2(R[2][1], R[2][2])
    # Pitch (rotation around X)
    pitch = math.atan2(-R[2][0], math.sqrt(R[2][1]**2 + R[2][2]**2))
    # Yaw (rotation around Y)
    yaw   = math.atan2(R[1][0], R[0][0])

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def print_pose(marker_id, tvec, rvec):
    """Print full 6DOF pose of marker relative to camera"""

    # Position (translation vector)
    x = tvec[0][0][0]  # left/right  (+ = right)
    y = tvec[0][0][1]  # up/down     (+ = down)
    z = tvec[0][0][2]  # depth       (+ = forward)

    distance = np.linalg.norm(tvec)

    # Orientation (rotation vector → Euler angles)
    roll, pitch, yaw = rvec_to_euler(rvec)

    print(f"┌─ Marker ID: {marker_id}")
    print(f"│  Position  (cm) : X={x:+7.2f}  Y={y:+7.2f}  Z={z:+7.2f}")
    print(f"│  Distance  (cm) : {distance:.2f}")
    print(f"│  Rotation (deg) : Roll={roll:+7.1f}  Pitch={pitch:+7.1f}  Yaw={yaw:+7.1f}")
    print(f"└{'─'*55}")

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    aruco_dict   = aruco.Dictionary_get(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters_create()

    # Loosen detection
    aruco_params.adaptiveThreshWinSizeMin  = 3
    aruco_params.adaptiveThreshWinSizeMax  = 53
    aruco_params.adaptiveThreshWinSizeStep = 4
    aruco_params.minMarkerPerimeterRate    = 0.01
    aruco_params.errorCorrectionRate       = 1.0

    print("Camera open — press CTRL+C to quit")
    print(f"OpenCV version: {cv2.__version__}")
    print(f"Marker size: {MARKER_SIZE_CM} cm")
    print("=" * 57)
    print("Coordinate system (camera view):")
    print("  X → right      Y → down      Z → forward (depth)")
    print("  Roll=tilt left/right  Pitch=tilt up/down  Yaw=rotate")
    print("=" * 57)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = aruco.detectMarkers(
                gray, aruco_dict, parameters=aruco_params
            )

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):

                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        corners[i], MARKER_SIZE_CM, CAMERA_MATRIX, DIST_COEFFS
                    )

                    print_pose(marker_id, tvec, rvec)

            else:
                pass  # silent when no marker

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        cap.release()
        print("Camera released")

if __name__ == "__main__":
    main()
