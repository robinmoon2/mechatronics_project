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
SPEED_MAX = 80
SPEED_MIN = 20        # never go below this when far
SLOW_ZONE = 0.30       # meters — start slowing down within 0.5m
STOP_DIST = 0.15      # meters — stop here
KV = 100
MAX_STEER_ANGLE = 30   # max degrees left/right from center
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
    KH = 0.5
    
    def __init__(self, robot: GPS_coordinates, target: GPS_coordinates):
        self.robot = robot
        self.target = target
        self.finished = False
        self.prev_robot = GPS_coordinates(x=robot.x, y=robot.y)
        self.robot_heading = 0.0  # radians, estimated from motion
        
        motorStop()
        set_angle(0, STEER_CENTER)

    def distance(self) -> float:
        dx = self.target.x - self.robot.x
        dy = self.target.y - self.robot.y
        return math.sqrt(dx**2 + dy**2)

    def desired_heading(self) -> float:
        dx = self.target.x - self.robot.x
        dy = self.target.y - self.robot.y
        return math.atan2(dy, dx)

    def update_heading(self):
        """Estimate robot heading from movement between frames."""
        dx = self.robot.x - self.prev_robot.x
        dy = self.robot.y - self.prev_robot.y
        if math.sqrt(dx**2 + dy**2) > 0.01:  # moved enough
            self.robot_heading = math.atan2(dy, dx)
        self.prev_robot.x = self.robot.x
        self.prev_robot.y = self.robot.y

    @staticmethod
    def normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def compute_steering(self) -> float:
        psi_star = self.desired_heading()
        error = self.normalize_angle(psi_star - self.robot_heading)
        delta_deg = self.KH * math.degrees(error)

        steer_value = STEER_CENTER + delta_deg  # + or - : flip if robot turns wrong way

        steer_value = max(STEER_CENTER - MAX_STEER_ANGLE,
                          min(STEER_CENTER + MAX_STEER_ANGLE, steer_value))

        print(f"heading:{math.degrees(self.robot_heading):.1f}° "
              f"desired:{math.degrees(psi_star):.1f}° "
              f"err:{math.degrees(error):.1f}° steer:{steer_value:.1f}°")
        return steer_value

    def compute_speed(self) -> float:
        return KV*self.distance()

    def activate_motors(self) -> float:
        self.update_heading()
        dist = self.distance()

        if dist < STOP_DIST:
            motorStop()
            set_angle(0, STEER_CENTER)
            self.finished = True
            print("DESTINATION REACHED")
            return 0.0

        steer = self.compute_steering()
        speed = self.compute_speed()

        set_angle(0, steer)
        Motor(1, 1, speed)
        Motor(2, 1, speed)

        return speed
