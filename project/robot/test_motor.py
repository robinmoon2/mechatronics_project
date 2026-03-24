#!/usr/bin/env python3
"""
test.py — Interactive motor & encoder test for Adeept robot.
Run with:  sudo python3 test.py
"""

import time
import threading
try:
    import RPi.GPIO as GPIO
except ImportError:
    import gpiod  # For newer Raspberry Pi OS without RPi.GPIO

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo

# ══════════════════════════════════════════════════════════════
# PCA9685 / Motor Setup (from MotorCtrl.py)
# ══════════════════════════════════════════════════════════════
MOTOR_M1_IN1 = 15
MOTOR_M1_IN2 = 14
MOTOR_M2_IN1 = 12
MOTOR_M2_IN2 = 13
MOTOR_M3_IN1 = 11
MOTOR_M3_IN2 = 10
MOTOR_M4_IN1 = 8
MOTOR_M4_IN2 = 9

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50

motor1 = motor.DCMotor(pca.channels[MOTOR_M1_IN1], pca.channels[MOTOR_M1_IN2])
motor1.decay_mode = motor.SLOW_DECAY
motor2 = motor.DCMotor(pca.channels[MOTOR_M2_IN1], pca.channels[MOTOR_M2_IN2])
motor2.decay_mode = motor.SLOW_DECAY
motor3 = motor.DCMotor(pca.channels[MOTOR_M3_IN1], pca.channels[MOTOR_M3_IN2])
motor3.decay_mode = motor.SLOW_DECAY
motor4 = motor.DCMotor(pca.channels[MOTOR_M4_IN1], pca.channels[MOTOR_M4_IN2])
motor4.decay_mode = motor.SLOW_DECAY

steering_servo = servo.Servo(
    pca.channels[0], min_pulse=500, max_pulse=2400, actuation_range=180
)

motors = {1: motor1, 2: motor2, 3: motor3, 4: motor4}

def map_val(x, in_min, in_max, out_min, out_max):
    return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

def Motor(channel, direction, speed_pct):
    speed_pct = max(0, min(100, speed_pct))
    throttle = map_val(speed_pct, 0, 100, 0, 1.0)
    if direction == -1:
        throttle = -throttle
    motors[channel].throttle = throttle

def motorStop():
    for m in motors.values():
        m.throttle = 0

def set_angle(angle):
    steering_servo.angle = max(0, min(180, angle))

def destroy():
    motorStop()
    set_angle(90)
    pca.deinit()

# ══════════════════════════════════════════════════════════════
# Encoder Setup
# ══════════════════════════════════════════════════════════════
# Adjust these GPIO pins to match YOUR encoder wiring!
# Typical: two encoders on rear wheels (left = M1, right = M2)
ENCODER_LEFT_PIN  = 17   # BCM pin for left encoder signal
ENCODER_RIGHT_PIN = 27   # BCM pin for right encoder signal
TICKS_PER_REV = 20       # depends on your encoder disc

encoder_left_count  = 0
encoder_right_count = 0
encoder_lock = threading.Lock()

USE_ENCODERS = True

try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENCODER_LEFT_PIN,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_RIGHT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def _left_tick(channel):
        global encoder_left_count
        with encoder_lock:
            encoder_left_count += 1

    def _right_tick(channel):
        global encoder_right_count
        with encoder_lock:
            encoder_right_count += 1

    GPIO.add_event_detect(ENCODER_LEFT_PIN,  GPIO.RISING, callback=_left_tick)
    GPIO.add_event_detect(ENCODER_RIGHT_PIN, GPIO.RISING, callback=_right_tick)
    print("✅ Encoders initialised on GPIO %d (L) and %d (R)" %
          (ENCODER_LEFT_PIN, ENCODER_RIGHT_PIN))
except Exception as e:
    USE_ENCODERS = False
    print(f"⚠️  Encoder init failed ({e}) — running without encoders")

def reset_encoders():
    global encoder_left_count, encoder_right_count
    with encoder_lock:
        encoder_left_count = 0
        encoder_right_count = 0

def read_encoders():
    with encoder_lock:
        return encoder_left_count, encoder_right_count

# ══════════════════════════════════════════════════════════════
# Test Routines
# ══════════════════════════════════════════════════════════════

def test_individual_motors():
    """Spin each motor one at a time to verify wiring."""
    print("\n═══ TEST 1: Individual Motor Check ═══")
    for ch in range(1, 5):
        print(f"  Motor {ch} FORWARD @ 40%...", end="", flush=True)
        Motor(ch, 1, 40)
        time.sleep(1.5)
        motorStop()
        time.sleep(0.3)

        print(f" REVERSE @ 40%...", end="", flush=True)
        Motor(ch, -1, 40)
        time.sleep(1.5)
        motorStop()
        time.sleep(0.5)
        print(" ✓")
    print("  Done.\n")


def test_all_forward_backward():
    """Drive all motors forward then backward."""
    print("\n═══ TEST 2: All Motors Forward / Backward ═══")
    speed = 50
    reset_encoders()

    print(f"  ALL FORWARD @ {speed}%  (2 s)")
    for ch in range(1, 5):
        Motor(ch, 1, speed)
    time.sleep(2)
    motorStop()
    l, r = read_encoders()
    if USE_ENCODERS:
        print(f"  Encoder ticks — L: {l}  R: {r}")

    time.sleep(0.5)
    reset_encoders()

    print(f"  ALL REVERSE  @ {speed}%  (2 s)")
    for ch in range(1, 5):
        Motor(ch, -1, speed)
    time.sleep(2)
    motorStop()
    l, r = read_encoders()
    if USE_ENCODERS:
        print(f"  Encoder ticks — L: {l}  R: {r}")

    print("  Done.\n")


def test_speed_ramp():
    """Ramp speed from 0 → 100 → 0 and log encoder rates."""
    print("\n═══ TEST 3: Speed Ramp (Motor 1 & 2) ═══")
    print("  Speed | Ticks/0.5s (L) | Ticks/0.5s (R)")
    print("  ------+----------------+----------------")

    for speed_pct in list(range(0, 101, 10)) + list(range(90, -1, -10)):
        reset_encoders()
        Motor(1, 1, speed_pct)
        Motor(2, 1, speed_pct)
        time.sleep(0.5)
        l, r = read_encoders()
        if USE_ENCODERS:
            print(f"  {speed_pct:>5} | {l:>14} | {r:>14}")
        else:
            print(f"  {speed_pct:>5} | (no encoders)  | (no encoders)")

    motorStop()
    print("  Done.\n")


def test_steering():
    """Sweep the steering servo left → center → right."""
    print("\n═══ TEST 4: Steering Servo Sweep ═══")
    for angle in [90, 60, 45, 30, 90, 120, 135, 150, 90]:
        print(f"  Steering → {angle}°")
        set_angle(angle)
        time.sleep(0.6)
    print("  Done.\n")


def test_drive_with_steering():
    """Drive forward while steering left, center, right."""
    print("\n═══ TEST 5: Drive + Steer ═══")
    speed = 40

    for label, angle in [("LEFT 60°", 60), ("CENTER", 90), ("RIGHT 120°", 120)]:
        print(f"  {label} @ {speed}%  (1.5 s)")
        reset_encoders()
        set_angle(angle)
        time.sleep(0.2)
        Motor(1, 1, speed)
        Motor(2, 1, speed)
        time.sleep(1.5)
        motorStop()
        l, r = read_encoders()
        if USE_ENCODERS:
            print(f"    Ticks — L: {l}  R: {r}")
        time.sleep(0.5)

    set_angle(90)
    print("  Done.\n")


def test_encoder_sanity():
    """Quick check: spin motors and verify encoders are counting."""
    if not USE_ENCODERS:
        print("\n═══ TEST 6: SKIPPED (no encoders) ═══\n")
        return

    print("\n═══ TEST 6: Encoder Sanity Check ═══")
    reset_encoders()

    Motor(1, 1, 60)
    Motor(2, 1, 60)
    time.sleep(2)
    motorStop()

    l, r = read_encoders()
    print(f"  Left ticks:  {l}  {'✅' if l > 0 else '❌ NOT COUNTING'}")
    print(f"  Right ticks: {r}  {'✅' if r > 0 else '❌ NOT COUNTING'}")

    if l > 0 and r > 0:
        ratio = max(l, r) / min(l, r)
        print(f"  L/R ratio:   {ratio:.2f}  {'✅ balanced' if ratio < 1.3 else '⚠️  imbalanced'}")
    print("  Done.\n")


# ══════════════════════════════════════════════════════════════
# Interactive Menu
# ══════════════════════════════════════════════════════════════

TESTS = {
    "1": ("Individual motor check",    test_individual_motors),
    "2": ("All forward / backward",    test_all_forward_backward),
    "3": ("Speed ramp + encoders",     test_speed_ramp),
    "4": ("Steering servo sweep",      test_steering),
    "5": ("Drive + steer combo",       test_drive_with_steering),
    "6": ("Encoder sanity check",      test_encoder_sanity),
    "a": ("Run ALL tests",             None),
}

def main():
    print("╔══════════════════════════════════════════╗")
    print("║     Adeept Motor & Encoder Test Suite     ║")
    print("╠══════════════════════════════════════════╣")
    if USE_ENCODERS:
        print("║  Encoders: ✅ active                      ║")
    else:
        print("║  Encoders: ❌ not available                ║")
    print("╚══════════════════════════════════════════╝")

    try:
        while True:
            print("\nSelect a test:")
            for key, (desc, _) in TESTS.items():
                print(f"  [{key}] {desc}")
            print("  [q] Quit\n")

            choice = input(">>> ").strip().lower()

            if choice == "q":
                break
            elif choice == "a":
                for key, (_, fn) in TESTS.items():
                    if fn is not None:
                        fn()
            elif choice in TESTS and TESTS[choice][1] is not None:
                TESTS[choice][1]()
            else:
                print("Invalid choice.")

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
    finally:
        print("Cleaning up...")
        destroy()
        if USE_ENCODERS:
            GPIO.cleanup()
        print("Done. Goodbye!")

if __name__ == "__main__":
    main()
