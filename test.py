#!/usr/bin/env/python
# File name   : LineTracking.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date        : 2025/03/7

import time
from gpiozero import InputDevice
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from gpiozero import DistanceSensor
from adafruit_motor import motor , servo

line_pin_left = 22
line_pin_middle = 27
line_pin_right = 17

Tr = 23
Ec = 24
sensor = DistanceSensor(echo=Ec, trigger=Tr,max_distance=2) # Maximum detection distance 2m.

MOTOR_M1_IN1 =  15      #Define the positive pole of M1
MOTOR_M1_IN2 =  14      #Define the negative pole of M1
MOTOR_M2_IN1 =  12      #Define the positive pole of M2
MOTOR_M2_IN2 =  13      #Define the negative pole of M2
MOTOR_M3_IN1 =  11      #Define the positive pole of M3
MOTOR_M3_IN2 =  10      #Define the negative pole of M3
MOTOR_M4_IN1 =  8       #Define the positive pole of M4
MOTOR_M4_IN2 =  9       #Define the negative pole of M4

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

left = InputDevice(pin=line_pin_left)
middle = InputDevice(pin=line_pin_middle)
right = InputDevice(pin=line_pin_right)

def grabobject():
  while checkdist() <= 5:
    Motor(1, 1, 0)
    Motor(2, 1, 0)

    set_angle(1, 0)
    set_angle(2, 180)
    set_angle(3, 180)
    time.sleep(1)

    set_angle(1, 180)
    time.sleep(1)

    set_angle(1, 90)
    set_angle(2, 90)
    set_angle(3, 90)
    print("Object grabbed! Stopping the car.")

    print("Object released! Resuming the car's movement.")
def run():
    if checkdist() < 10:
      print("Object detected within 5 cm! Stopping the car.")
      grabobject()
      Motor(1, 1, 15)
      Motor(2, 1, 15) 
    status_right = right.value
    status_middle = middle.value
    status_left = left.value
   # print('left: %d   middle: %d   right: %d' %(status_left,status_middle,status_right))
    
    if (status_left == 0) :
        set_angle(0, 120)
        set_angle(1, 70)
    elif(status_left == 0 and status_middle == 0):
        set_angle(0, 140)
        set_angle(1, 50)
    elif(status_middle == 1)and (status_left == 1) and (status_right == 1):
        Motor(1, 1, 15)
        Motor(2, 1, 15)
        set_angle(0, 90)
        set_angle(1, 90)
    elif (status_right == 0):
        set_angle(0, 60)
        set_angle(1, 110)
    elif(status_right==0 and status_middle==0):
        set_angle(0,40)
        set_angle(1,130)
    elif (status_middle == 0) and (status_left == 0) and (status_right == 0):
        Motor(1, 1, 0)
        Motor(2, 1, 0)

def checkdist():
    return (sensor.distance) *100

def map(x,in_min,in_max,out_min,out_max):
  return (x - in_min)/(in_max - in_min) *(out_max - out_min) +out_min

def Motor(channel,direction,motor_speed):
  if motor_speed > 100:
    motor_speed = 100
  elif motor_speed < 0:
    motor_speed = 0
  speed = map(motor_speed, 0, 100, 0, 1.0)
  if direction == 1:
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

def set_angle(ID, angle):
    servo_angle = servo.Servo(pwm_motor.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    servo_angle.angle = angle


if __name__ == '__main__':
    try:
        Motor(1, 1, 10)
        Motor(2, 1, 10)
        set_angle(1, 90)
        set_angle(2, 90)
        set_angle(3, 90)
        while 1:
            run()
            #set_angle(4, 180)

    except KeyboardInterrupt:
        destroy()
        pass
