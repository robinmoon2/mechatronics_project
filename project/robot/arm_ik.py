# arm_ik.py
import math
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO

# PCA9685 setup
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50

# Servo channels
BASE_CH = 1
SHOULDER_CH = 2
ELBOW_CH = 3
GRIPPER_CH = 4

# Ultrasonic sensor pins
TRIG = 23
ECHO = 24

# Link lengths in cm
L0 = 9.0
L1 = 8.5
L2 = 10.0
L3 = 18.0
SHOULDER_HEIGHT = L0 + L1  # 19.5 cm

OBJECT_HEIGHT = 17.0  # cm from ground

# GPIO setup for ultrasonic
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    """Read distance from ultrasonic sensor in cm."""
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    timeout = time.time() + 0.04
    pulse_start = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return -1

    timeout = time.time() + 0.04
    pulse_end = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return -1

    distance = (pulse_end - pulse_start) * 17150
    return round(distance, 1)

def set_angle(ID, angle):
    angle = max(0, min(180, angle))
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    servo_angle.angle = angle

WRIST_CH = 5  # adjust to your actual wrist channel

def compute_arm_angles(x, y, z_target=0):
    theta1 = math.degrees(math.atan2(y, x))
    r = math.sqrt(x**2 + y**2)
    z_eff = z_target - SHOULDER_HEIGHT
    d_sq = r**2 + z_eff**2
    d = math.sqrt(d_sq)

    if d > (L2 + L3) or d < abs(L2 - L3):
        print(f"Target ({x}, {y}, {z_target}) unreachable! d={d:.1f}, range=[{abs(L2-L3):.1f}, {L2+L3:.1f}]")
        return None

    cos_theta3 = (d_sq - L2**2 - L3**2) / (2 * L2 * L3)
    cos_theta3 = max(-1, min(1, cos_theta3))
    theta3 = math.acos(cos_theta3)

    theta2 = math.atan2(z_eff, r) - math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))

    theta2_deg = math.degrees(theta2)
    theta3_deg = math.degrees(theta3)

    servo1 = 90 + theta1
    servo2 = 90 - theta2_deg
    servo3 = 90 + theta3_deg

    print(f"  IK: theta1={theta1:.1f}° theta2={theta2_deg:.1f}° theta3={theta3_deg:.1f}°")
    return servo1, servo2, servo3, theta2_deg, theta3_deg

def move_to(x, y, z=0):
    result = compute_arm_angles(x, y, z)
    if result is None:
        return None
    s1, s2, s3, t2, t3 = result
    print(f"  Servos: base={s1:.1f}, shoulder={s2:.1f}, elbow={s3:.1f}")
    set_angle(BASE_CH, s1)
    time.sleep(0.3)
    set_angle(SHOULDER_CH, s2)
    time.sleep(0.3)
    set_angle(ELBOW_CH, s3)
    time.sleep(0.3)
    return t2, t3

def grab_at(x, y, z=0):
    APPROACH_OFFSET = 10

    print("--- Opening gripper ---")
    set_angle(GRIPPER_CH, 120)
    time.sleep(2)

    print("--- Approaching ---")
    result = move_to(x - APPROACH_OFFSET, y, z)
    time.sleep(2)

 

    print("--- Closing gripper ---")
    set_angle(GRIPPER_CH, 0)
    set_angle(GRIPPER_CH, 0)
    time.sleep(2)

    print("--- Lifting ---")
    move_to(x, y, z + 8)
    time.sleep(0.5)
    return True

def home():
    print("--- Homing ---")
    for ch in [BASE_CH, SHOULDER_CH, ELBOW_CH, GRIPPER_CH]:
        set_angle(ch, 90)
        time.sleep(0.3)

# ============ MAIN ============
if __name__ == "__main__":
    print("=== ULTRASONIC GRAB ===\n")
    try:
        home()
        time.sleep(1)
        # Take multiple readings for accuracy
        print("--- Measuring distance ---")
        readings = []
        for i in range(5):
            d = get_distance()
            if d > 0:
                readings.append(d)
                print(f"  Reading {i+1}: {d} cm")
            time.sleep(0.1)

        if not readings:
            print("ERROR: No valid ultrasonic readings!")
        else:
            x_distance = sum(readings) / len(readings) +5
            print(f"\n  Average distance (X): {x_distance:.1f} cm")
            print(f"  Object height (Z): {OBJECT_HEIGHT} cm")

            # Object is straight ahead (y=0), at measured X, at 17cm height
            print(f"\n--- Grabbing object at ({x_distance:.1f}, 0, {OBJECT_HEIGHT}) ---")
            grab_at(x_distance, 0, 15)

            time.sleep(2)
            home()
            print("\nDone!")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Homing.")
        home()
    finally:
        pca.deinit()
        GPIO.cleanup()
