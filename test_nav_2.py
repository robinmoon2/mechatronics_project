#!/usr/bin/env python3
"""
Quadrant test version of the minimal Level-2 path follower for Adeept PiCar Pro V2.

Purpose:
- keep almost the same structure as the original minimal controller
- temporarily replace the ArUco pose source with an internal simulated pose source
- let the user choose one of 4 target poses around the robot:
    * front_right
    * front_left
    * rear_right
    * rear_left
- optionally command the real robot at low speed while still using simulated pose
- still monitor the ultrasonic sensor and stop if an obstacle is too close

Important:
- the simulated pose is only for controller testing before ArUco localization is ready
- if you enable real motion, the robot motion and the simulated pose can drift apart
- start with low speed in a clear area

Frame convention used here:
- x forward
- y left
- theta positive counterclockwise
"""

import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass
from statistics import median
from typing import Optional

# Hardware imports are optional so the script can also run in pure simulation.
HARDWARE_AVAILABLE = True
try:
    import Move as move
    import Ultra as ultra
    import RPIservo
except Exception:
    HARDWARE_AVAILABLE = True
    move = True
    ultra = True
    RPIservo = True


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


class SimulatedPoseSource:
    """Provide a simulated robot pose and update it from the commanded motion."""

    def __init__(
        self,
        start_x: float = 0.0,
        start_y: float = 0.0,
        start_theta_deg: float = 0.0,
        wheelbase_m: float = 0.145,
        speed_to_mps: float = 0.008,
    ) -> None:
        self.pose = Pose(
            x=start_x,
            y=start_y,
            theta=math.radians(start_theta_deg),
            stamp=time.time(),
        )
        self.wheelbase_m = wheelbase_m
        self.speed_to_mps = speed_to_mps

    def read(self) -> Pose:
        self.pose.stamp = time.time()
        return Pose(self.pose.x, self.pose.y, self.pose.theta, self.pose.stamp)

    def apply_motion(self, speed: int, direction: int, steering_deg: float, dt: float) -> None:
        if speed <= 0:
            self.pose.stamp = time.time()
            return

        v = float(speed) * self.speed_to_mps * float(direction)
        delta = math.radians(clamp(steering_deg, -35.0, 35.0))

        theta_dot = 0.0
        if abs(math.cos(delta)) > 1e-3:
            theta_dot = (v / self.wheelbase_m) * math.tan(delta)

        self.pose.x += v * math.cos(self.pose.theta) * dt
        self.pose.y += v * math.sin(self.pose.theta) * dt
        self.pose.theta = wrap_to_pi(self.pose.theta + theta_dot * dt)
        self.pose.stamp = time.time()


class MinimalPathFollower:
    def __init__(
        self,
        pose_source: SimulatedPoseSource,
        target: TargetPose,
        stop_distance_cm: float = 25.0,
        position_tolerance_m: float = 0.06,
        theta_tolerance_deg: float = 8.0,
        control_period_s: float = 0.05,
        verbose: bool = True,
        drive_real_robot: bool = True,
        use_ultrasonic: bool = True,
    ) -> None:
        self.pose_source = pose_source
        self.target = target
        self.stop_distance_cm = stop_distance_cm
        self.position_tolerance_m = position_tolerance_m
        self.theta_tolerance_rad = math.radians(theta_tolerance_deg)
        self.control_period_s = control_period_s
        self.verbose = verbose
        self.drive_real_robot = drive_real_robot and HARDWARE_AVAILABLE
        self.use_ultrasonic = use_ultrasonic and HARDWARE_AVAILABLE

        # Vehicle / control tuning.
        self.max_steering_deg = 28.0
        self.min_speed = 14
        self.max_speed = 24
        self.align_speed = 12
        self.final_heading_speed = 10
        self.heading_only_threshold_rad = math.radians(35.0)
        self.reverse_selection_threshold_rad = math.radians(100.0)

        # Proportional gains.
        self.k_alpha = 1.7
        self.k_theta = 1.6
        self.k_rho = 18.0

        # Ultrasonic filtering.
        self.ultrasonic_samples = 3
        self.obstacle_confirmations = 2
        self.obstacle_counter = 0

        self.running = True
        self.last_status_print = 0.0

        self.steering = None
        if self.drive_real_robot:
            self.steering = RPIservo.ServoCtrl()
            self.steering.start()
            move.setup()
            self.center_steering()
            move.motorStop()

    def center_steering(self) -> None:
        if self.steering is not None:
            self.steering.moveAngle(0, 0)

    def set_steering(self, steering_deg: float) -> None:
        steering_deg = clamp(steering_deg, -self.max_steering_deg, self.max_steering_deg)
        if self.steering is not None:
            self.steering.moveAngle(0, steering_deg)

    def drive(self, speed: int, direction: int) -> None:
        speed = int(clamp(speed, 0, 100))
        direction = 1 if direction >= 0 else -1

        if self.drive_real_robot:
            if speed <= 0:
                move.motorStop()
            else:
                move.move(speed, direction, "no")

    def stop_robot(self) -> None:
        if self.drive_real_robot:
            move.motorStop()
            self.center_steering()

    def cleanup(self) -> None:
        self.stop_robot()

    def read_filtered_distance_cm(self) -> Optional[float]:
        if not self.use_ultrasonic:
            return None

        samples = []
        for _ in range(self.ultrasonic_samples):
            try:
                distance = ultra.checkdist()
            except Exception:
                return None
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

    def compute_drive_command(self, pose: Pose) -> tuple[int, float, float, float, int]:
        dx = self.target.x - pose.x
        dy = self.target.y - pose.y
        rho = math.hypot(dx, dy)

        heading_to_goal = math.atan2(dy, dx)
        alpha_forward = wrap_to_pi(heading_to_goal - pose.theta)
        alpha_reverse = wrap_to_pi(heading_to_goal - wrap_to_pi(pose.theta + math.pi))
        theta_error = wrap_to_pi(self.target.theta - pose.theta)

        if rho > self.position_tolerance_m:
            use_reverse = abs(alpha_forward) > self.reverse_selection_threshold_rad
            direction = -1 if use_reverse else 1
            alpha = alpha_reverse if use_reverse else alpha_forward

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

            speed_scale = 1.0 - 0.5 * min(abs(alpha) / math.pi, 1.0)
            speed = int(clamp(speed * speed_scale, self.align_speed, self.max_speed))
            return speed, steering_deg, rho, theta_error, direction

        # Final heading correction.
        theta_error_forward = wrap_to_pi(self.target.theta - pose.theta)
        theta_error_reverse = wrap_to_pi(self.target.theta - wrap_to_pi(pose.theta + math.pi))

        if abs(theta_error_forward) <= abs(theta_error_reverse):
            direction = 1
            theta_error_for_control = theta_error_forward
        else:
            direction = -1
            theta_error_for_control = theta_error_reverse

        steering_rad = clamp(
            self.k_theta * theta_error_for_control,
            math.radians(-self.max_steering_deg),
            math.radians(self.max_steering_deg),
        )
        steering_deg = math.degrees(steering_rad)
        return self.final_heading_speed, steering_deg, rho, theta_error, direction

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
        direction: int,
        distance_cm: Optional[float],
    ) -> None:
        now = time.time()
        if not self.verbose or (now - self.last_status_print) < 0.25:
            return

        self.last_status_print = now
        distance_txt = "N/A" if distance_cm is None else f"{distance_cm:.1f}"
        direction_txt = "FWD" if direction == 1 else "REV"
        print(
            f"pose=({pose.x:.3f}, {pose.y:.3f}, {math.degrees(pose.theta):.1f} deg) | "
            f"goal_error=(rho={rho:.3f} m, theta={math.degrees(theta_error):.1f} deg) | "
            f"cmd=({direction_txt}, speed={speed}, steering={steering_deg:.1f} deg) | "
            f"ultra={distance_txt} cm"
        )

    def run(self) -> int:
        print("Starting quadrant test path follower...")
        print(
            f"Target pose: x={self.target.x:.3f} m, y={self.target.y:.3f} m, "
            f"theta={math.degrees(self.target.theta):.1f} deg"
        )
        print(f"Obstacle stop threshold: {self.stop_distance_cm:.1f} cm")
        print(f"Drive real robot: {self.drive_real_robot}")
        print(f"Use ultrasonic: {self.use_ultrasonic}")

        while self.running:
            blocked, distance_cm = self.obstacle_detected()
            if blocked:
                self.stop_robot()
                print(f"Obstacle detected at {distance_cm:.1f} cm -> robot stopped.")
                return 1

            pose = self.pose_source.read()

            if self.goal_reached(pose):
                self.stop_robot()
                print("Target pose reached.")
                return 0

            speed, steering_deg, rho, theta_error, direction = self.compute_drive_command(pose)
            self.set_steering(steering_deg)
            self.drive(speed, direction)

            # Update the internal simulated pose from the same commanded motion.
            self.pose_source.apply_motion(
                speed=speed,
                direction=direction,
                steering_deg=steering_deg,
                dt=self.control_period_s,
            )

            self.print_status(pose, rho, theta_error, steering_deg, speed, direction, distance_cm)
            time.sleep(self.control_period_s)

        self.stop_robot()
        return 0


def build_target_from_option(option: str, forward_distance: float, lateral_distance: float) -> TargetPose:
    option = option.strip().lower()

    if option == "front_right":
        x = forward_distance
        y = -lateral_distance
    elif option == "front_left":
        x = forward_distance
        y = lateral_distance
    elif option == "rear_right":
        x = -forward_distance
        y = -lateral_distance
    elif option == "rear_left":
        x = -forward_distance
        y = lateral_distance
    else:
        raise ValueError(f"Unknown target option: {option}")

    theta = math.atan2(y, x)
    return TargetPose(x=x, y=y, theta=theta)


def choose_option_interactively() -> str:
    options = {
        "1": "front_right",
        "2": "front_left",
        "3": "rear_right",
        "4": "rear_left",
    }

    print("Choose the target option:")
    print("  1 -> front_right")
    print("  2 -> front_left")
    print("  3 -> rear_right")
    print("  4 -> rear_left")

    while True:
        choice = input("Selection [1-4]: ").strip()
        if choice in options:
            return options[choice]
        print("Invalid choice. Please enter 1, 2, 3 or 4.")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quadrant test path follower for Adeept PiCar Pro V2")
    parser.add_argument(
        "--option",
        type=str,
        choices=["front_right", "front_left", "rear_right", "rear_left"],
        help="Target option to test",
    )
    parser.add_argument(
        "--forward-distance",
        type=float,
        default=0.60,
        help="Forward offset used to build the target, in meters",
    )
    parser.add_argument(
        "--lateral-distance",
        type=float,
        default=0.35,
        help="Lateral offset used to build the target, in meters",
    )
    parser.add_argument(
        "--start-x",
        type=float,
        default=0.0,
        help="Initial simulated x in meters",
    )
    parser.add_argument(
        "--start-y",
        type=float,
        default=0.0,
        help="Initial simulated y in meters",
    )
    parser.add_argument(
        "--start-theta-deg",
        type=float,
        default=0.0,
        help="Initial simulated theta in degrees",
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
        "--drive-real-robot",
        action="store_true",
        help="Actually send commands to the motors and steering servo",
    )
    parser.add_argument(
        "--ignore-ultra",
        action="store_true",
        help="Disable ultrasonic obstacle monitoring",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Reduce console output",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    option = args.option or choose_option_interactively()
    target = build_target_from_option(option, args.forward_distance, args.lateral_distance)

    pose_source = SimulatedPoseSource(
        start_x=args.start_x,
        start_y=args.start_y,
        start_theta_deg=args.start_theta_deg,
    )

    controller = MinimalPathFollower(
        pose_source=pose_source,
        target=target,
        stop_distance_cm=args.stop_distance_cm,
        position_tolerance_m=args.pos_tol,
        theta_tolerance_deg=args.theta_tol_deg,
        verbose=not args.quiet,
        drive_real_robot=args.drive_real_robot,
        use_ultrasonic=not args.ignore_ultra,
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
