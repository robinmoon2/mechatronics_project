"""
navigation.py — Reads ArUco GPS coordinates from the same UDP stream
and computes distance + bearing from ROBOT marker to TARGET marker.
IDs are configured via a .env file.
"""

import json
import math
import os
import socket
from dataclasses import dataclass, field
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

MOTOR_M1_IN1 =  15      #Define the positive pole of M1
MOTOR_M1_IN2 =  14      #Define the negative pole of M1
MOTOR_M2_IN1 =  12      #Define the positive pole of M2
MOTOR_M2_IN2 =  13      #Define the negative pole of M2
MOTOR_M3_IN1 =  11      #Define the positive pole of M3
MOTOR_M3_IN2 =  10      #Define the negative pole of M3
MOTOR_M4_IN1 =  8       #Define the positive pole of M4
MOTOR_M4_IN2 =  9       #Define the negative pole of M4

STEER_CENTER = 90
SPEED_MAX = 50
SPEED_MIN = 30        # never go below this when far
SLOW_ZONE = 0.30       # meters — start slowing down within 0.5m
STOP_DIST = 0.15      # meters — stop here
KV = 0.5
MAX_STEER_ANGLE = 30   # max degrees left/right from center
KH = 0.5               # proportional gain (tune this)
def map(x,in_min,in_max,out_min,out_max):
  return (x - in_min)/(in_max - in_min) *(out_max - out_min) +out_min

i2c = busio.I2C(SCL, SDA)
pwm_motor = PCA9685(i2c, address=0x5f)
pwm_motor.frequency = 50

motor1 = motor.DCMotor(pwm_motor.channels[MOTOR_M1_IN1],pwm_motor.channels[MOTOR_M1_IN2] )
motor1.decay_mode = (motor.SLOW_DECAY)
motor2 = motor.DCMotor(pwm_motor.channels[MOTOR_M2_IN1],pwm_motor.channels[MOTOR_M2_IN2] )
motor2.decay_mode = (motor.SLOW_DECAY)
motor3 = motor.DCMotor(pwm_motor.channels[MOTOR_M3_IN1],pwm_motor.channels[MOTOR_M3_IN2] )
motor3.decay_mode = (motor.SLOW_DECAY)
motor4 = motor.DCMotor(pwm_motor.channels[MOTOR_M4_IN1],pwm_motor.channels[MOTOR_M4_IN2] )
motor4.decay_mode = (motor.SLOW_DECAY)

def Motor(channel,direction,motor_speed):
  if motor_speed > 100:
    motor_speed = 100
  elif motor_speed < 0:
    motor_speed = 0
  speed = map(motor_speed, 0, 100, 0, 1.0)
  if direction == -1:
    speed = -speed

  if channel == 1:
    motor1.throttle = speed
  elif channel == 2:
    motor2.throttle = speed
  elif channel == 3:
    motor3.throttle = speed
  elif channel == 4:
    motor4.throttle = speed

def motorStop():#Motor stops
    motor1.throttle = 0
    motor2.throttle = 0
    motor3.throttle = 0
    motor4.throttle = 0

def destroy():
  motorStop()
  pwm_motor.deinit()



## SERVO MOTEUR 

import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c, address=0x5f)  # default 0x40

pca.frequency = 50


def set_angle(ID, angle):
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    servo_angle.angle = angle


def test(channel):
    for i in range(180):  # The servo turns from 0 to 180 degrees.
        set_angle(channel, i)
        time.sleep(0.01)
    time.sleep(0.5)
    for i in range(180):  # The servo turns from 180 to 0 degrees.
        set_angle(channel, 180 - i)
        time.sleep(0.01)
    time.sleep(0.5)


# ── Data ───────────────────────────────────────────────────────
@dataclass
class GPS_coordinates:
    x: float= 0.0
    y: float= 0.0
    z: float= 0.0


class Navigation:
    """
    Proportional controller — drives to a target point.
    Based on 'Driving to a point' (course p.42)

    δ = Kh * (ψ* - ψ)   steering proportional control
    v = Kv * dist        speed proportional control (clamped)
    """

    KH = 0.2   # steering gain — tune this

    def __init__(self, robot: GPS_coordinates, target: GPS_coordinates):
        self.robot    = robot
        self.target   = target
        self.finished = False

        # Initial state — stopped, wheels centered
        motorStop()
        set_angle(0, STEER_CENTER)

    # ── Pose ───────────────────────────────────────────────────

    def distance(self) -> float:
        """Euclidean distance to target (XY plane only)"""
        dx = self.target.x - self.robot.x
        dy = self.target.y - self.robot.y
        return math.sqrt(dx**2 + dy**2)

    def desired_heading(self) -> float:
        """
        ψ* = atan2(dy, dx)  — desired heading in radians [-π, π]
        Course p.42
        """
        dx = self.target.x - self.robot.x
        dy = self.target.y - self.robot.y
        return math.atan2(dy, dx)   # radians

    @staticmethod
    def normalize_angle(angle_rad: float) -> float:
        """
        Clamp angle to [-π, π]
        Uses atan2 trick — always valid
        """
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    # ── Control law ────────────────────────────────────────────

    def compute_steering(self, robot_heading_rad: float = 0.0) -> float:
        psi_star = self.desired_heading()
        error    = self.normalize_angle(psi_star - robot_heading_rad + STEER_CENTER)
        delta_deg = self.KH * math.degrees(error)

        # Find correct sign empirically:
        steer_value = STEER_CENTER - delta_deg  # try + first, swap to - if reversed

        print(f"dx:{self.target.x-self.robot.x:.2f} dy:{self.target.y-self.robot.y:.2f} "
            f"ψ*:{math.degrees(psi_star):.1f}° err:{math.degrees(error):.1f}° "
            f"steer:{steer_value:.1f}°")

        return max(60, min(120, steer_value))
    
    def compute_speed(self, dist: float) -> float:
        """
        v = Kv * dist   proportional speed — clamped between SPEED_MIN and SPEED_MAX
        Course p.42
        """
        speed = KV * self.distance()
        return max(SPEED_MIN, min(SPEED_MAX, speed))

    # ── Main loop ──────────────────────────────────────────────

    def activate_motors():
        """
        steer_value    : servo angle (60-120°, center=90)
        distance       : meters to target
        base_speed     : motor speed 0-100
        stop_threshold : meters, stop when closer than this
        """
        distance = self.distance()
        
        base_speed = self.compute_speed()
        steer_value = self.compute_steering()
        
        if distance < STOP_DIST:
            motorStop()
            set_angle(0, STEER_CENTER)
            print("DESTINATION REACHED")
            return True  # finished

        # Apply steering
        set_angle(0, steer_value)

        # Drive forward (motors 1 & 2)
        Motor(1, 1, base_speed)
        Motor(2, 1, base_speed)
        Motor(3, 1, 0)
        Motor(4, 1, 0)

        return False  # not finished

