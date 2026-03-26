"""
navigation.py — ArUco-based navigation with odometry fallback.
Uses MotorDriver for DC motors and directly controls steering servo.
"""

import math
from dataclasses import dataclass
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from motor_driver import MotorDriver,  WHEEL_BASE_M, METRES_PER_PULSE


STEER_CENTER = 90
SPEED_MAX = 30
SPEED_MIN = 20
SLOW_ZONE = 0.45
STOP_DIST = 0.20
KV = 0.5
MAX_STEER_ANGLE = 90
WHEEL_RADIUS = 5

# ── Data ───────────────────────────────────────────────────────
@dataclass
class GPS_coordinates:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Navigation:
    KH = 1

    def __init__(self, robot: GPS_coordinates, target: GPS_coordinates):
        self.robot = robot
        self.target = target
        self.finished = False
        self.prev_robot = GPS_coordinates(x=robot.x, y=robot.y)
        self.robot_heading = 0.0
        self.current_steering = STEER_CENTER
        self.prev_steer_angle = STEER_CENTER

        # DC motors + encoders
        self.driver = MotorDriver(enable_encoders=True)
        self.driver.stop()
        
        self.prev_left = 0
        self.prev_right = 0

        # Steering servo (shares the same PCA9685 as motors)
        self.steering_servo = servo.Servo(
            self.driver._pca.channels[0],
            min_pulse=500, max_pulse=2400, actuation_range=180
        )
        self.set_steering(STEER_CENTER)
        self.tourelle_servo=servo.Servo(
            self.driver._pca.channels[1],
            min_pulse=500, max_pulse=2400, actuation_range=180
        )
        self.tourelle_servo.angle=90

    # --- Steering GPS
    def set_steering(self, angle: float):
        angle = max(0, min(180, angle))
        # Smooth: limit change to 10° per cycle
        max_delta = 60
        delta = angle - self.prev_steer_angle
        delta = max(-max_delta, min(max_delta, delta))
        angle = self.prev_steer_angle + delta
        self.prev_steer_angle = angle
        
        self.steering_servo.angle = angle
        self.current_steering = math.radians(angle - STEER_CENTER)
        
    # --- Utility functions GPS

    def distance(self) -> float:
        dx = self.target.x - self.robot.x
        dy = self.target.y - self.robot.y
        return math.sqrt(dx**2 + dy**2)

    def desired_heading(self) -> float:
        dx = self.target.x - self.robot.x
        dy = self.target.y - self.robot.y
        return math.atan2(dy, dx)

    def heading_error(self) -> float:
        err = self.desired_heading() - self.robot_heading
        err = (err + math.pi) % (2 * math.pi) - math.pi
        return err

    # -- navigation functions GPS

    def update_heading(self):
        dx = self.robot.x - self.prev_robot.x
        dy = self.robot.y - self.prev_robot.y
        if math.sqrt(dx**2 + dy**2) > 0.01:
            self.robot_heading = math.atan2(dy, dx) 
        self.prev_robot = GPS_coordinates(x=self.robot.x, y=self.robot.y)

    def compute_steering(self) -> float:
        err = self.heading_error()
        steer_offset = math.degrees(err) * self.KH
        steer_offset = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_offset))
        return STEER_CENTER - steer_offset

    def compute_speed(self) -> float:
        dist = self.distance()
        if dist < SLOW_ZONE:
            speed = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * (dist / SLOW_ZONE)
        else:
            speed = SPEED_MAX
        return speed

    # --- hardware functions
 

    def destroy(self):
        self.driver.shutdown()
    
    def activate_motors(self, gps_alive: bool = True) -> float:
        dist = self.distance()
        dx = self.target.x - self.robot.x
        dy = self.target.y - self.robot.y
        desired = math.degrees(self.desired_heading())
        heading = math.degrees(self.robot_heading)
        err = math.degrees(self.heading_error())
        steer_val = self.compute_steering()
        print(f"[NAV] dist={dist:.4f}  robot=({self.robot.x:.3f},{self.robot.y:.3f})"
          f"  target=({self.target.x:.3f},{self.target.y:.3f})"
          f"  heading={math.degrees(self.robot_heading):.1f}°"
          f"  err={math.degrees(self.heading_error()):.1f}°")

        if dist < STOP_DIST:
            self.driver.stop()
            self.set_steering(STEER_CENTER)
            self.finished = True
            print("DESTINATION REACHED")
            return 0.0

        if gps_alive:
            self.update_heading()

        steer = self.compute_steering()
        speed = self.compute_speed()
        self.set_steering(steer)
        self.driver.forward(speed)
        return speed

    # def activate_motors(self, gps_alive: bool = True) -> float:
    #     dist = self.distance()
    #     print(f"[NAV] dist={dist:.4f}  target=({self.target.x:.3f},{self.target.y:.3f})"
    #         f"  robot=({self.robot.x:.3f},{self.robot.y:.3f})  finished={self.finished}")

    #     if dist < STOP_DIST:
    #         self.driver.stop()
    #         self.set_steering(STEER_CENTER)
    #         self.finished = True
    #         print("  DESTINATION REACHED")
    #         return 0.0

    #     # Only update heading from GPS delta when GPS is alive
    #     # Odometry already updated heading in compute_odometry()
    #     if gps_alive:
    #         self.update_heading()

    #     steer = self.compute_steering()
    #     speed = self.compute_speed()
    #     self.set_steering(steer)
    #     self.driver.forward(speed)
    #     return speed

    def navigation_choice(self, gps_alive: bool = False):
        if gps_alive:
            self.driver.reset_odometry()
            self.update_heading()          # heading from GPS delta
        else:
            self.compute_odometry()        # heading from odometry

        if not self.finished:
            self.activate_motors(gps_alive=gps_alive)   # pass flag
        else:
            self.driver.stop()

    # Temporarily boost odometry aggressiveness
    def compute_odometry(self):
        results = self.driver.get_odometry()
        tl = results["total_left_pulses"]
        tr = results["total_right_pulses"]

        delta_l = tl - self.prev_left
        delta_r = tr - self.prev_right
        self.prev_left = tl
        self.prev_right = tr

        dl = delta_l * METRES_PER_PULSE
        dr = delta_r * METRES_PER_PULSE
        d_distance = (dl + dr) / 2.0

        if abs(d_distance) < 1e-6:
            print("[ODO] No movement detected")
            d_distance = (SPEED_MIN / 100.0) * 0.1  # rough fallback
