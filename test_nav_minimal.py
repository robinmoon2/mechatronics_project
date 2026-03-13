"""
Minimal Level-2 path-following controller for Adeept PiCar Pro V2.

What this script does:
- drive the robot toward a target pose (x, y, theta)
- use the Adeept motor and steering modules
- continuously monitor the ultrasonic sensor
- stop immediately when an obstacle is detected closer than a safety threshold

Important:
- this is the minimal version requested for Level 2
- it does NOT avoid the obstacle yet, it only stops
- localization is expected to come from an external process (for example an ArUco pipeline)
- the external process must keep writing the current robot pose to a JSON file

Expected JSON file format:
{
    "x": 0.35,
    "y": 1.10,
    "theta": 1.57,
    "stamp": 1710000000.25
}

Units:
- x, y in meters
- theta in radians
- stamp in Unix seconds
"""

import argparse
import json
import math
import signal
import sys
import time
from dataclasses import dataclass
from statistics import median
from typing import Optional


@dataclass
class Pose:
    x: float
    y: float
    theta: float
    stamp: float


@dataclass
class TargetPose:
    x: float
    y: float
    theta: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class JsonPoseSource:
    """Read the current robot pose from a JSON file written by another process."""

    def __init__(self, path: str, max_pose_age_s: float = 0.5) -> None:
        self.path = path
        self.max_pose_age_s = max_pose_age_s

    def read(self) -> Optional[Pose]:
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return None

        try:
            pose = Pose(
                x=float(data["x"]),
                y=float(data["y"]),
                theta=float(data["theta"]),
                stamp=float(data.get("stamp", time.time())),
            )
        except (KeyError, TypeError, ValueError):
            return None

        if (time.time() - pose.stamp) > self.max_pose_age_s:
            return None

        return pose


class MinimalPathFollower:
    def __init__(
        self,
        pose_source: JsonPoseSource,
        target: TargetPose,
        stop_distance_cm: float = 25.0,
        position_tolerance_m: float = 0.06,
        theta_tolerance_deg: float = 8.0,
        control_period_s: float = 0.05,
        verbose: bool = True,
    ) -> None:
        self.pose_source = pose_source
        self.target = target
        self.stop_distance_cm = stop_distance_cm
        self.position_tolerance_m = position_tolerance_m
        self.theta_tolerance_rad = math.radians(theta_tolerance_deg)
        self.control_period_s = control_period_s
        self.verbose = verbose

        # Vehicle / control tuning.
        self.max_steering_deg = 28.0
        self.min_speed = 14
        self.max_speed = 28
        self.align_speed = 12
        self.final_heading_speed = 10
        self.heading_only_threshold_rad = math.radians(35.0)

        # Proportional gains.
        self.k_alpha = 1.7   # heading to target
        self.k_theta = 1.8   # final heading correction
        self.k_rho = 18.0    # distance to target

        # Ultrasonic filtering.
        self.ultrasonic_samples = 3
        self.obstacle_confirmations = 2
        self.obstacle_counter = 0

        self.running = True
        self.last_status_print = 0.0

        # Adeept hardware setup.
        self.steering = RPIservo.ServoCtrl()
        self.steering.start()
        move.setup()
        self.center_steering()
        move.motorStop()

    def center_steering(self) -> None:
        self.steering.moveAngle(0, 0)

    def set_steering(self, steering_deg: float) -> None:
        steering_deg = clamp(steering_deg, -self.max_steering_deg, self.max_steering_deg)
        self.steering.moveAngle(0, steering_deg)

    def drive_forward(self, speed: int) -> None:
        speed = int(clamp(speed, 0, 100))
        if speed <= 0:
            move.motorStop()
            return
        move.move(speed, 1, "no")

    def stop_robot(self) -> None:
        move.motorStop()
        self.center_steering()

    def cleanup(self) -> None:
        self.stop_robot()

    def read_filtered_distance_cm(self) -> Optional[float]:
        samples = []
        for _ in range(self.ultrasonic_samples):
            distance = ultra.checkdist()
            if 2.0 <= distance <= 400.0:
                samples.append(distance)
            time.sleep(0.01)

        if not samples:
            return None

        return float(median(samples))

    def obstacle_detected(self) -> tuple[bool, Optional[float]]:
        distance_cm = self.read_filtered_distance_cm()
        if distance_cm is None:
            self.obstacle_counter = 0
            return False, None

        if distance_cm < self.stop_distance_cm:
            self.obstacle_counter += 1
        else:
            self.obstacle_counter = 0

        is_blocked = self.obstacle_counter >= self.obstacle_confirmations
        return is_blocked, distance_cm

    def compute_drive_command(self, pose: Pose) -> tuple[int, float, float, float]:
        dx = self.target.x - pose.x
        dy = self.target.y - pose.y
        rho = math.hypot(dx, dy)

        heading_to_goal = math.atan2(dy, dx)
        alpha = wrap_to_pi(heading_to_goal - pose.theta)
        theta_error = wrap_to_pi(self.target.theta - pose.theta)

        if rho > self.position_tolerance_m:
            steering_rad = clamp(
                self.k_alpha * alpha,
                math.radians(-self.max_steering_deg),
                math.radians(self.max_steering_deg),
            )
            steering_deg = math.degrees(steering_rad)

            if abs(alpha) > self.heading_only_threshold_rad:
                speed = self.align_speed
            else:
                raw_speed = self.min_speed + self.k_rho * rho
                speed = int(clamp(raw_speed, self.min_speed, self.max_speed))

            # Reduce speed if the robot is badly oriented.
            speed_scale = 1.0 - 0.5 * min(abs(alpha) / math.pi, 1.0)
            speed = int(clamp(speed * speed_scale, self.align_speed, self.max_speed))
            return speed, steering_deg, rho, theta_error

        # Final heading adjustment.
        steering_rad = clamp(
            self.k_theta * theta_error,
            math.radians(-self.max_steering_deg),
            math.radians(self.max_steering_deg),
        )
        steering_deg = math.degrees(steering_rad)
        return self.final_heading_speed, steering_deg, rho, theta_error

    def goal_reached(self, pose: Pose) -> bool:
        dx = self.target.x - pose.x
        dy = self.target.y - pose.y
        rho = math.hypot(dx, dy)
        theta_error = wrap_to_pi(self.target.theta - pose.theta)
        return rho <= self.position_tolerance_m and abs(theta_error) <= self.theta_tolerance_rad

    def print_status(
        self,
        pose: Pose,
        rho: float,
        theta_error: float,
        steering_deg: float,
        speed: int,
        distance_cm: Optional[float],
    ) -> None:
        now = time.time()
        if not self.verbose or (now - self.last_status_print) < 0.25:
            return

        self.last_status_print = now
        distance_txt = "N/A" if distance_cm is None else f"{distance_cm:.1f}"
        print(
            f"pose=({pose.x:.3f}, {pose.y:.3f}, {math.degrees(pose.theta):.1f} deg) | "
            f"goal_error=(rho={rho:.3f} m, theta={math.degrees(theta_error):.1f} deg) | "
            f"cmd=(speed={speed}, steering={steering_deg:.1f} deg) | "
            f"ultra={distance_txt} cm"
        )

    def run(self) -> int:
        print("Starting minimal Level-2 path follower...")
        print(
            f"Target pose: x={self.target.x:.3f} m, y={self.target.y:.3f} m, "
            f"theta={math.degrees(self.target.theta):.1f} deg"
        )
        print(f"Obstacle stop threshold: {self.stop_distance_cm:.1f} cm")

        while self.running:
            blocked, distance_cm = self.obstacle_detected()
            if blocked:
                self.stop_robot()
                print(f"Obstacle detected at {distance_cm:.1f} cm -> robot stopped.")
                return 1

            pose = self.pose_source.read()
            if pose is None:
                self.stop_robot()
                print("No valid pose available (missing or stale pose file). Waiting...")
                time.sleep(self.control_period_s)
                continue

            if self.goal_reached(pose):
                self.stop_robot()
                print("Target pose reached.")
                return 0

            speed, steering_deg, rho, theta_error = self.compute_drive_command(pose)
            self.set_steering(steering_deg)
            self.drive_forward(speed)
            self.print_status(pose, rho, theta_error, steering_deg, speed, distance_cm)

            time.sleep(self.control_period_s)

        self.stop_robot()
        return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Minimal Level-2 path follower for Adeept PiCar Pro V2")
    parser.add_argument("--goal-x", type=float, required=True, help="Target x in meters")
    parser.add_argument("--goal-y", type=float, required=True, help="Target y in meters")
    parser.add_argument("--goal-theta-deg", type=float, required=True, help="Target theta in degrees")
    parser.add_argument(
        "--pose-file",
        type=str,
        default="/tmp/robot_pose.json",
        help="Path to the JSON file containing the current robot pose",
    )
    parser.add_argument(
        "--max-pose-age",
        type=float,
        default=0.5,
        help="Maximum accepted pose age in seconds",
    )
    parser.add_argument(
        "--stop-distance-cm",
        type=float,
        default=25.0,
        help="Ultrasonic stop threshold in cm",
    )
    parser.add_argument(
        "--pos-tol",
        type=float,
        default=0.06,
        help="Position tolerance in meters",
    )
    parser.add_argument(
        "--theta-tol-deg",
        type=float,
        default=8.0,
        help="Heading tolerance in degrees",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Reduce console output",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    pose_source = JsonPoseSource(args.pose_file, max_pose_age_s=args.max_pose_age)
    target = TargetPose(
        x=args.goal_x,
        y=args.goal_y,
        theta=math.radians(args.goal_theta_deg),
    )

    controller = MinimalPathFollower(
        pose_source=pose_source,
        target=target,
        stop_distance_cm=args.stop_distance_cm,
        position_tolerance_m=args.pos_tol,
        theta_tolerance_deg=args.theta_tol_deg,
        verbose=not args.quiet,
    )

    def handle_signal(signum, frame):  # noqa: ANN001, ARG001
        controller.running = False
        controller.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        return controller.run()
    finally:
        controller.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
