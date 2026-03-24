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
from adafruit_motor import motor, servo
MOTOR_M1_IN1 =  15      #Define the positive pole of M1
MOTOR_M1_IN2 =  14      #Define the negative pole of M1
MOTOR_M2_IN1 =  12      #Define the positive pole of M2
MOTOR_M2_IN2 =  13      #Define the negative pole of M2
MOTOR_M3_IN1 =  11      #Define the positive pole of M3
MOTOR_M3_IN2 =  10      #Define the negative pole of M3
MOTOR_M4_IN1 =  8       #Define the positive pole of M4
MOTOR_M4_IN2 =  9       #Define the negative pole of M4

STEER_CENTER = 90
SPEED_MAX = 40
SPEED_MIN = 20        # never go below this when far
SLOW_ZONE = 0.40       # meters — start slowing down within 0.5m
STOP_DIST = 0.15      # meters — stop here
KV = 1
MAX_STEER_ANGLE = 60   # max degrees left/right from 
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
steering_servo = servo.Servo(pwm_motor.channels[0], min_pulse=500, max_pulse=2400, actuation_range=180)

def set_angle(angle):
    steering_servo.angle = angle
    
# def set_angle(ID, angle):
#     servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
#     servo_angle.angle = angle


def test(channel):
    for i in range(180):  # The servo turns from 0 to 180 degrees.
        set_angle( i)
        time.sleep(0.01)
    time.sleep(0.5)
    for i in range(180):  # The servo turns from 180 to 0 degrees.
        set_angle( 180 - i)
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
        set_angle(STEER_CENTER)
        
        
    #--- Utility functions

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
        # Normalize to [-pi, pi]
        err = (err + math.pi) % (2 * math.pi) - math.pi
        return err
    
    # -- navigation functions 
    def update_heading(self):
        """Estimate heading from successive position updates."""
        dx = self.robot.x - self.prev_robot.x
        dy = self.robot.y - self.prev_robot.y
        if math.sqrt(dx**2 + dy**2) > 0.01:  # only update if moved enough
            self.robot_heading = math.atan2(dy, dx)
        self.prev_robot = GPS_coordinates(x=self.robot.x, y=self.robot.y)

    def compute_steering(self) -> float:
        """Convert heading error to a servo angle."""
        err = self.heading_error()
        # err in radians → map to degrees offset
        steer_offset = math.degrees(err) * self.KH
        steer_offset = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_offset))
        return STEER_CENTER - steer_offset
    
    def compute_speed(self) -> float:
        """Proportional speed: slow down near target."""
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
            motorStop()
            set_angle( STEER_CENTER)
            self.finished = True
            print("DESTINATION REACHED")
            return 0.0

        steer = self.compute_steering()
        speed = self.compute_speed()

        set_angle( steer)
        Motor(1, 1, speed)
        Motor(2, 1, speed)

        return speed
