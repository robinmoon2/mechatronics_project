#!/usr/bin/env python3
# File name   : picar_direction_controller.py
# Description : Real motion test controller for Adeept PiCar Pro V2
# Notes       : Based on the user's working motor/encoder code.
#               - Rear motors are driven with encoders.
#               - Steering servo is on PCA9685 channel 1.
#               - Ultrasonic stop is enabled by default.
#               - This script is meant for REAL robot tests.

import math
import time
import threading
import smbus
from gpiozero import DistanceSensor
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo

# ============================================================
# PCA9685 / Motors / Steering Servo
# ============================================================
MOTOR_M1_IN1 = 15
MOTOR_M1_IN2 = 14
MOTOR_M2_IN1 = 12
MOTOR_M2_IN2 = 13
MOTOR_M3_IN1 = 11
MOTOR_M3_IN2 = 10
MOTOR_M4_IN1 = 8
MOTOR_M4_IN2 = 9

STEERING_SERVO_CHANNEL = 0
STEERING_CENTER_DEG = 90.0
STEERING_MAX_OFFSET_DEG = 30.0   # tune on your robot

# Your mapping from the working code
LEFT_CHANNELS = [1, 3]
RIGHT_CHANNELS = [2, 4]

# ============================================================
# Ultrasonic
# ============================================================
ULTRA_TRIG = 23
ULTRA_ECHO = 24

# ============================================================
# Physical constants - tune these on the real robot
# ============================================================
WHEEL_DIAMETER_M = 0.07
TRACK_WIDTH_M = 0.20         # left-right distance for pivot turning
PULSES_PER_REV = 11
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M
METRES_PER_PULSE = WHEEL_CIRCUMFERENCE_M / PULSES_PER_REV

# ADS7830 channels from your working code
ENC_LEFT_CH_A = 6
ENC_LEFT_CH_B = 7
ENC_RIGHT_CH_A = 4
ENC_RIGHT_CH_B = 5
ADC_THRESHOLD = 127

# Default motion parameters
DEFAULT_DRIVE_SPEED = 65.0
DEFAULT_TURN_SPEED = 48.0
DEFAULT_UPDATE_DT = 0.03
STOP_DISTANCE_CM = 25.0


# ============================================================
# Low-level hardware setup
# ============================================================
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def pct_to_throttle(speed_pct: float) -> float:
    speed_pct = clamp(speed_pct, -100.0, 100.0)
    return speed_pct / 100.0


class ADS7830:
    """8-channel 8-bit ADC over I2C."""

    I2C_ADDRESS = 0x48
    CMD_BASE = 0x84

    def __init__(self):
        self.bus = smbus.SMBus(1)

    def analog_read(self, chn: int) -> int:
        cmd = self.CMD_BASE | (((chn << 2 | chn >> 1) & 0x07) << 4)
        return self.bus.read_byte_data(self.I2C_ADDRESS, cmd)


class EncoderReader(threading.Thread):
    """
    Count encoder pulses from ADS7830.
    Rising edge on channel A increments/decrements pulse count.
    """

    def __init__(self, adc: ADS7830,
                 ch_a_left: int, ch_b_left: int,
                 ch_a_right: int, ch_b_right: int,
                 poll_interval: float = 0.002):
        super().__init__(daemon=True)
        self.adc = adc
        self.ch_a_left = ch_a_left
        self.ch_b_left = ch_b_left
        self.ch_a_right = ch_a_right
        self.ch_b_right = ch_b_right
        self.poll_interval = poll_interval

        self._lock = threading.Lock()
        self._pulses_left = 0
        self._pulses_right = 0
        self._prev_a_left = False
        self._prev_a_right = False
        self._running = True

    def get_and_reset(self):
        with self._lock:
            pl = self._pulses_left
            pr = self._pulses_right
            self._pulses_left = 0
            self._pulses_right = 0
        return pl, pr

    def stop(self):
        self._running = False

    def run(self):
        while self._running:
            raw_al = self.adc.analog_read(self.ch_a_left)
            raw_bl = self.adc.analog_read(self.ch_b_left)
            raw_ar = self.adc.analog_read(self.ch_a_right)
            raw_br = self.adc.analog_read(self.ch_b_right)

            a_left = raw_al > ADC_THRESHOLD
            b_left = raw_bl > ADC_THRESHOLD
            a_right = raw_ar > ADC_THRESHOLD
            b_right = raw_br > ADC_THRESHOLD

            with self._lock:
                if a_left and not self._prev_a_left:
                    self._pulses_left += 1 if not b_left else -1
                if a_right and not self._prev_a_right:
                    self._pulses_right += 1 if not b_right else -1

            self._prev_a_left = a_left
            self._prev_a_right = a_right
            time.sleep(self.poll_interval)


class Odometry:
    """Differential-style odometry for straight runs and pivot turns."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, left_pulses: int, right_pulses: int):
        d_left = left_pulses * METRES_PER_PULSE
        d_right = right_pulses * METRES_PER_PULSE
        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / TRACK_WIDTH_M

        self.x += d_center * math.cos(self.theta + 0.5 * d_theta)
        self.y += d_center * math.sin(self.theta + 0.5 * d_theta)
        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    @property
    def pose(self):
        return self.x, self.y, self.theta


class PiCarDirectionController:
    def __init__(self, stop_distance_cm: float = STOP_DISTANCE_CM):
        self.i2c = busio.I2C(SCL, SDA)
        self.pwm = PCA9685(self.i2c, address=0x5F)
        self.pwm.frequency = 50

        self.motor1 = motor.DCMotor(self.pwm.channels[MOTOR_M1_IN1], self.pwm.channels[MOTOR_M1_IN2])
        self.motor2 = motor.DCMotor(self.pwm.channels[MOTOR_M2_IN1], self.pwm.channels[MOTOR_M2_IN2])
        self.motor3 = motor.DCMotor(self.pwm.channels[MOTOR_M3_IN1], self.pwm.channels[MOTOR_M3_IN2])
        self.motor4 = motor.DCMotor(self.pwm.channels[MOTOR_M4_IN1], self.pwm.channels[MOTOR_M4_IN2])
        for m in (self.motor1, self.motor2, self.motor3, self.motor4):
            m.decay_mode = motor.SLOW_DECAY

        self.steering = servo.Servo(
            self.pwm.channels[STEERING_SERVO_CHANNEL],
            min_pulse=500,
            max_pulse=2400,
            actuation_range=180,
        )

        self.ultra = DistanceSensor(echo=ULTRA_ECHO, trigger=ULTRA_TRIG, max_distance=2)
        self.stop_distance_cm = stop_distance_cm

        self.adc = ADS7830()
        self.encoder = EncoderReader(
            adc=self.adc,
            ch_a_left=ENC_LEFT_CH_A,
            ch_b_left=ENC_LEFT_CH_B,
            ch_a_right=ENC_RIGHT_CH_A,
            ch_b_right=ENC_RIGHT_CH_B,
        )
        self.encoder.start()

        self.odometry = Odometry()
        self._center_steering()
        self.stop()

    # ------------------------
    # Low-level motor control
    # ------------------------
    def _motors(self):
        return {
            1: self.motor1,
            2: self.motor2,
            3: self.motor3,
            4: self.motor4,
        }

    def set_motor(self, channel: int, speed_pct: float):
        motors = self._motors()
        if channel not in motors:
            return
        motors[channel].throttle = pct_to_throttle(speed_pct)

    def _set_left(self, speed_pct: float):
        for ch in LEFT_CHANNELS:
            self.set_motor(ch, speed_pct)

    def _set_right(self, speed_pct: float):
        for ch in RIGHT_CHANNELS:
            self.set_motor(ch, speed_pct)

    def set_drive(self, left_speed_pct: float, right_speed_pct: float):
        self._set_left(left_speed_pct)
        self._set_right(right_speed_pct)

    def stop(self):
        self.set_drive(0.0, 0.0)

    # ------------------------
    # Steering control
    # ------------------------
    def _center_steering(self):
        self.set_steering(0.0)

    def set_steering(self, offset_deg: float):
        offset_deg = clamp(offset_deg, -STEERING_MAX_OFFSET_DEG, STEERING_MAX_OFFSET_DEG)
        target = clamp(STEERING_CENTER_DEG + offset_deg, 0.0, 180.0)
        self.steering.angle = target

    # ------------------------
    # Sensors / safety
    # ------------------------
    def read_distance_cm(self) -> float:
        return round(self.ultra.distance * 100.0, 2)

    def obstacle_detected(self) -> bool:
        return self.read_distance_cm() < self.stop_distance_cm

    def _safety_check(self):
        if self.obstacle_detected():
            self.stop()
            self._center_steering()
            raise RuntimeError(f"Obstacle detected at {self.read_distance_cm():.1f} cm")

    # ------------------------
    # Encoder / odometry helpers
    # ------------------------
    def reset_motion_counters(self):
        self.encoder.get_and_reset()

    def reset_pose(self):
        self.odometry.reset()
        self.reset_motion_counters()

    # ------------------------
    # Motion primitives
    # ------------------------
    def drive_distance(self,
                       distance_m: float,
                       speed_pct: float = DEFAULT_DRIVE_SPEED,
                       steering_deg: float = 0.0,
                       update_dt: float = DEFAULT_UPDATE_DT,
                       verbose: bool = True):
        """
        Drive forward/backward for a measured distance.
        steering_deg controls the front wheels during the motion.
        Positive distance = forward, negative distance = backward.
        """
        if distance_m == 0:
            return

        direction = 1.0 if distance_m > 0 else -1.0
        target = abs(distance_m)
        travelled = 0.0

        self.reset_motion_counters()
        self.set_steering(steering_deg)

        if verbose:
            print(f"[Drive] target={distance_m:.3f} m | steering={steering_deg:.1f} deg")

        while travelled < target:
            self._safety_check()
            self.set_drive(direction * speed_pct, direction * speed_pct)
            time.sleep(update_dt)

            pl, pr = self.encoder.get_and_reset()
            self.odometry.update(pl, pr)

            d_left = abs(pl) * METRES_PER_PULSE
            d_right = abs(pr) * METRES_PER_PULSE
            ds = 0.5 * (d_left + d_right)
            travelled += ds

            if verbose:
                print(
                    f"  travelled={travelled:.3f}/{target:.3f} m | "
                    f"ultra={self.read_distance_cm():.1f} cm | "
                    f"pose=({self.odometry.x:.3f}, {self.odometry.y:.3f}, {math.degrees(self.odometry.theta):.1f} deg)"
                )

        self.stop()
        self._center_steering()

    def pivot_turn(self,
                   angle_deg: float,
                   speed_pct: float = DEFAULT_TURN_SPEED,
                   update_dt: float = DEFAULT_UPDATE_DT,
                   verbose: bool = True):
        """
        Turn on the spot using opposite wheel speeds.
        Positive = CCW (left), negative = CW (right).
        Front wheels are centered during the pivot turn.
        """
        if angle_deg == 0:
            return

        target_rad = abs(math.radians(angle_deg))
        turned_rad = 0.0
        sign = 1.0 if angle_deg > 0 else -1.0

        self.reset_motion_counters()
        self._center_steering()

        if verbose:
            print(f"[Turn] target={angle_deg:.1f} deg")

        while turned_rad < target_rad:
            self._safety_check()
            left_speed = -sign * speed_pct
            right_speed = sign * speed_pct
            self.set_drive(left_speed, right_speed)
            time.sleep(update_dt)

            pl, pr = self.encoder.get_and_reset()
            self.odometry.update(pl, pr)

            d_left = pl * METRES_PER_PULSE
            d_right = pr * METRES_PER_PULSE
            d_theta = abs((d_right - d_left) / TRACK_WIDTH_M)
            turned_rad += d_theta

            if verbose:
                print(
                    f"  turned={math.degrees(turned_rad):.1f}/{abs(angle_deg):.1f} deg | "
                    f"ultra={self.read_distance_cm():.1f} cm | "
                    f"theta={math.degrees(self.odometry.theta):.1f} deg"
                )

        self.stop()
        self._center_steering()

    def move_direction(self,
                       name: str,
                       distance_m: float = 1,
                       steering_deg: float = 45.0,
                       speed_pct: float = DEFAULT_DRIVE_SPEED,
                       verbose: bool = True):
        """
        Execute one of the 4 simple real-world test directions.

        front_right, front_left, rear_right, rear_left
        """
        name = name.strip().lower()
        presets = {
            "front_right": (+distance_m, +abs(steering_deg)),
            "front_left": (+distance_m, -abs(steering_deg)),
            "rear_right": (-distance_m, +abs(steering_deg)),
            "rear_left": (-distance_m, -abs(steering_deg)),
        }
        if name not in presets:
            raise ValueError(f"Unknown direction '{name}'")

        dist, steer = presets[name]
        if verbose:
            print(f"[Preset] {name} -> distance={dist:.3f} m, steering={steer:.1f} deg")
        self.drive_distance(dist, speed_pct=speed_pct, steering_deg=steer, verbose=verbose)

    def shutdown(self):
        try:
            self.stop()
            self._center_steering()
        finally:
            self.encoder.stop()
            self.pwm.deinit()


# ============================================================
# Demo / CLI menu
# ============================================================
def print_menu():
    print("\nChoose a motion:")
    print("  1 -> forward straight")
    print("  2 -> backward straight")
    print("  3 -> pivot left")
    print("  4 -> pivot right")
    print("  5 -> front_right")
    print("  6 -> front_left")
    print("  7 -> rear_right")
    print("  8 -> rear_left")
    print("  q -> quit")


def main():
    robot = PiCarDirectionController(stop_distance_cm=STOP_DISTANCE_CM)
    try:
        print("PiCar real motion controller started.")
        print("Keep the robot on a clear floor and be ready to cut power if needed.")

        while True:
            print_menu()
            cmd = input("Your choice: ").strip().lower()

            if cmd == "q":
                break
            elif cmd == "1":
                robot.drive_distance(0.40)
            elif cmd == "2":
                robot.drive_distance(-0.40)
            elif cmd == "3":
                robot.pivot_turn(+90.0)
            elif cmd == "4":
                robot.pivot_turn(-90.0)
            elif cmd == "5":
                robot.move_direction("front_right")
            elif cmd == "6":
                robot.move_direction("front_left")
            elif cmd == "7":
                robot.move_direction("rear_right")
            elif cmd == "8":
                robot.move_direction("rear_left")
            else:
                print("Unknown command.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except RuntimeError as exc:
        print(f"\nEmergency stop: {exc}")
    finally:
        robot.shutdown()
        print("Robot shutdown complete.")


if __name__ == "__main__":
    main()
