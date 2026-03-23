import math

def compute_angle_and_distance(robot_x, robot_y, target_x, target_y):
    """
    Compute angle (degrees) and distance between robot and target.
    Angle is relative to camera frame (Y axis = forward).
    Range: -180 to +180
    Positive = target is to the RIGHT
    Negative = target is to the LEFT
    """
    dx = target_x - robot_x
    dy = target_y - robot_y

    distance = math.sqrt(dx**2 + dy**2)

    # Angle relative to -Y axis (robot faces toward origin = -Y direction)
    angle_rad = math.atan2(dx, -dy)
    angle_deg = math.degrees(angle_rad)

    return angle_deg, distance


# ── Test ──────────────────────────────────────────────────────
if __name__ == "__main__":
    tests = [
        # (robot_x, robot_y, target_x, target_y, expected)
        ((0, 1, 0, 0),   "target straight ahead → 0°"),
        ((0, 1, 1, 0),   "target ahead-right    → ~45°"),
        ((0, 1, -1, 0),  "target ahead-left     → ~-45°"),
        ((0, 1, 1, 2),   "target behind-right   → ~135°"),
        ((0, 1, 0, 2),   "target straight behind→ 180°"),
    ]

    for (rx, ry, tx, ty), desc in tests:
        angle, dist = compute_angle_and_distance(rx, ry, tx, ty)
        print(f"{desc:35s} | angle={angle:7.1f}°  dist={dist:.3f}")
