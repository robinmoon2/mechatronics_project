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
SPEED_MAX = 50
SPEED_MIN = 10
SLOW_ZONE = 0.45
STOP_DIST = 0.20
KV = 0.5
MAX_STEER_ANGLE = 40
WHEEL_RADIUS = 5

# ── Data ───────────────────────────────────────────────────────
@dataclass
class GPS_coordinates:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Navigation:
    KH = 0.5

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

    # --- Steering GPS
    def set_steering(self, angle: float):
        angle = max(0, min(180, angle))
        # Smooth: limit change to 10° per cycle
        max_delta = 10
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

    def activate_motors(self) -> float:
        self.update_heading()
        dist = self.distance()

        if dist < STOP_DIST:
            self.driver.stop()
            self.set_steering(STEER_CENTER)
            self.finished = True
            print("DESTINATION REACHED")
            return 0.0

        steer = self.compute_steering()
        speed = self.compute_speed()
        
        self.set_steering(steer)
        self.driver.forward(speed)
        return speed

    def destroy(self):
        self.driver.shutdown()
    
    
    def navigation_choice(self,gps_alive=False):
        if gps_alive:
            self.driver.reset_odometry()
        else:
            self.compute_odometry()

        if not self.finished:
            self.activate_motors()
        else:
            self.driver.stop()

    
    def compute_odometry(self):
        results = self.driver.get_odometry()
        tl = results["total_left_pulses"]
        tr = results["total_right_pulses"]
        
        delta_l = tl - self.prev_left
        delta_r = tr - self.prev_right
        
        # self.prev_left = tl
        # self.prev_right = tr

        dl = delta_l * METRES_PER_PULSE
        dr = delta_r * METRES_PER_PULSE

        d_distance = (dl + dr) / 2.0

        # Bicycle model
        d_theta = d_distance * math.tan(self.current_steering) / WHEEL_BASE_M

        self.robot_heading += d_theta
        self.robot.x += d_distance * math.cos(self.robot_heading)
        self.robot.y += d_distance * math.sin(self.robot_heading)
        
        print(f"[ODO] pulses L={tl} R={tr} dist={d_distance:.4f} "
          f"steer={math.degrees(self.current_steering):.1f}° "
          f"d_theta={math.degrees(d_theta):.2f}° "
          f"heading={math.degrees(self.robot_heading):.1f}° "
          f"pos=({self.robot.x:.3f}, {self.robot.y:.3f}) "
          f"target=({self.target.x:.3f}, {self.target.y:.3f})")
        self.driver.reset_odometry()

