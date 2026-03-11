#!/usr/bin/env python3
# File name   : OdometryMotorCtrl.py
# Description : Encoder-based odometry motor control for Adeept robot
# Encoders    : Motor1 -> ADS7830 CH6 (A), CH7 (B)
#               Motor2 -> ADS7830 CH6 (A), CH7 (B)  <- adjust channels as needed

import time
import math
import threading
import smbus
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

# ─────────────────────────────────────────────
#  PCA9685 / Motor setup  (from MotorCtrl.py)
# ─────────────────────────────────────────────
MOTOR_M1_IN1 = 15
MOTOR_M1_IN2 = 14
MOTOR_M2_IN1 = 12
MOTOR_M2_IN2 = 13
MOTOR_M3_IN1 = 11
MOTOR_M3_IN2 = 10
MOTOR_M4_IN1 = 8
MOTOR_M4_IN2 = 9

i2c = busio.I2C(SCL, SDA)
pwm_motor = PCA9685(i2c, address=0x5F)
pwm_motor.frequency = 50

motor1 = motor.DCMotor(pwm_motor.channels[MOTOR_M1_IN1], pwm_motor.channels[MOTOR_M1_IN2])
motor1.decay_mode = motor.SLOW_DECAY
motor2 = motor.DCMotor(pwm_motor.channels[MOTOR_M2_IN1], pwm_motor.channels[MOTOR_M2_IN2])
motor2.decay_mode = motor.SLOW_DECAY
motor3 = motor.DCMotor(pwm_motor.channels[MOTOR_M3_IN1], pwm_motor.channels[MOTOR_M3_IN2])
motor3.decay_mode = motor.SLOW_DECAY
motor4 = motor.DCMotor(pwm_motor.channels[MOTOR_M4_IN1], pwm_motor.channels[MOTOR_M4_IN2])
motor4.decay_mode = motor.SLOW_DECAY

# ─────────────────────────────────────────────
#  ADS7830 ADC  (from Voltage.py)
# ─────────────────────────────────────────────
class ADS7830:
    """8-channel 8-bit ADC over I2C."""
    I2C_ADDRESS = 0x48
    CMD_BASE    = 0x84

    def __init__(self):
        self.bus = smbus.SMBus(1)

    def analog_read(self, chn: int) -> int:
        """Return raw 0-255 value for ADC channel *chn* (0-7)."""
        cmd = self.CMD_BASE | (((chn << 2 | chn >> 1) & 0x07) << 4)
        return self.bus.read_byte_data(self.I2C_ADDRESS, cmd)


# ─────────────────────────────────────────────
#  Robot physical constants  – TUNE THESE
# ─────────────────────────────────────────────
WHEEL_DIAMETER_M   = 0.07          # metres
WHEEL_BASE_M       = 0.20          # distance between left & right wheels (metres)
PULSES_PER_REV     = 11             # encoder rising-edges per full wheel revolution
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER_M
METRES_PER_PULSE   = WHEEL_CIRCUMFERENCE / PULSES_PER_REV

# ADC channels for the two encoders (channel A only – used for counting)
ENC_LEFT_CH_A  = 6   # ADS7830 channel A6
ENC_LEFT_CH_B  = 7   # ADS7830 channel A7  (used for direction)
ENC_RIGHT_CH_A = 4     # adjust if second encoder uses different channels
ENC_RIGHT_CH_B = 5   # adjust accordingly

# Threshold: ADC value above this  = logic HIGH (0-255 range)
ADC_THRESHOLD = 127

# PID gains  – TUNE THESE for your motors
KP = 2.0
KI = 0.5
KD = 0.1

# Base motor speed (0-100 %)
BASE_SPEED = 79


# ─────────────────────────────────────────────
#  Low-level motor helpers
# ─────────────────────────────────────────────
def _map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min


def set_motor(channel: int, direction: int, speed_pct: float):
    """
    channel   : 1-4
    direction : 1 = forward, -1 = backward
    speed_pct : 0-100
    """
    speed_pct = max(0.0, min(100.0, speed_pct))
    throttle  = _map(speed_pct, 0, 100, 0.0, 1.0)
    if direction == -1:
        throttle = -throttle

    motors = {1: motor1, 2: motor2, 3: motor3, 4: motor4}
    if channel in motors:
        motors[channel].throttle = throttle


def motor_stop():
    for m in (motor1, motor2, motor3, motor4):
        m.throttle = 0


def destroy():
    motor_stop()
    pwm_motor.deinit()


# ─────────────────────────────────────────────
#  Encoder reader  (runs in its own thread)
# ─────────────────────────────────────────────
class EncoderReader(threading.Thread):
    """
    Polls ADS7830 channels at high frequency to count encoder pulses.
    Channel A rising-edge  → increment pulse count.
    Channel B level at that moment → determine direction.
    """

    def __init__(self, adc: ADS7830,
                 ch_a_left:  int, ch_b_left:  int,
                 ch_a_right: int, ch_b_right: int,
                 poll_interval: float = 0.002):
        super().__init__(daemon=True)
        self.adc           = adc
        self.ch_a_left     = ch_a_left
        self.ch_b_left     = ch_b_left
        self.ch_a_right    = ch_a_right
        self.ch_b_right    = ch_b_right
        self.poll_interval = poll_interval

        self._lock          = threading.Lock()
        self._pulses_left   = 0
        self._pulses_right  = 0

        # Previous A-channel logic levels (for edge detection)
        self._prev_a_left  = False
        self._prev_a_right = False

        self._running = True

    # ── public API ──────────────────────────────
    def get_and_reset(self):
        """Return (left_pulses, right_pulses) since last call, then reset."""
        with self._lock:
            pl = self._pulses_left
            pr = self._pulses_right
            self._pulses_left  = 0
            self._pulses_right = 0
        return pl, pr

    def stop(self):
        self._running = False

    # ── thread body ─────────────────────────────
    def run(self):
        while self._running:
            raw_al = self.adc.analog_read(self.ch_a_left)
            raw_bl = self.adc.analog_read(self.ch_b_left)
            raw_ar = self.adc.analog_read(self.ch_a_right)
            raw_br = self.adc.analog_read(self.ch_b_right)

            a_left  = raw_al > ADC_THRESHOLD
            b_left  = raw_bl > ADC_THRESHOLD
            a_right = raw_ar > ADC_THRESHOLD
            b_right = raw_br > ADC_THRESHOLD

            # Detect rising edge on channel A
            with self._lock:
                # Left encoder
                if a_left and not self._prev_a_left:
                    # B LOW when A rises → forward; B HIGH → backward
                    self._pulses_left += 1 if not b_left else -1

                # Right encoder
                if a_right and not self._prev_a_right:
                    self._pulses_right += 1 if not b_right else -1

            self._prev_a_left  = a_left
            self._prev_a_right = a_right

            time.sleep(self.poll_interval)


# ─────────────────────────────────────────────
#  Odometry tracker
# ─────────────────────────────────────────────
class Odometry:
    """Integrates wheel pulses into (x, y, theta)."""

    def __init__(self):
        self.x     = 0.0   # metres
        self.y     = 0.0   # metres
        self.theta = 0.0   # radians

    def update(self, left_pulses: int, right_pulses: int):
        """Call after each encoder sample period."""
        d_left  = left_pulses  * METRES_PER_PULSE
        d_right = right_pulses * METRES_PER_PULSE

        d_centre  = (d_left + d_right) / 2.0
        d_theta   = (d_right - d_left) / WHEEL_BASE_M

        self.x     += d_centre * math.cos(self.theta + d_theta / 2.0)
        self.y     += d_centre * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta
        # keep theta in (-π, π]
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

    @property
    def pose(self):
        return self.x, self.y, self.theta

    def reset(self):
        self.x = self.y = self.theta = 0.0


# ─────────────────────────────────────────────
#  Odometry-based motion controller
# ─────────────────────────────────────────────
class OdometryMotorController:
    """
    High-level controller that uses encoder odometry to drive the robot
    a precise distance or turn a precise angle.

    Motors 1 & 3 = LEFT  side  (channels 1 and 3)
    Motors 2 & 4 = RIGHT side  (channels 2 and 4)
    Adjust LEFT_CHANNELS / RIGHT_CHANNELS if your wiring differs.
    """

    LEFT_CHANNELS  = [1, 3]
    RIGHT_CHANNELS = [2, 4]

    def __init__(self):
        self.adc      = ADS7830()
        self.encoder  = EncoderReader(
            self.adc,
            ch_a_left  = ENC_LEFT_CH_A,
            ch_b_left  = ENC_LEFT_CH_B,
            ch_a_right = ENC_RIGHT_CH_A,
            ch_b_right = ENC_RIGHT_CH_B,
        )
        self.odometry = Odometry()
        self.encoder.start()

    # ── internal helpers ────────────────────────
    def _set_left(self, speed_pct: float):
        direction = 1 if speed_pct >= 0 else -1
        for ch in self.LEFT_CHANNELS:
            set_motor(ch, direction, abs(speed_pct))

    def _set_right(self, speed_pct: float):
        direction = 1 if speed_pct >= 0 else -1
        for ch in self.RIGHT_CHANNELS:
            set_motor(ch, direction, abs(speed_pct))

    def _apply_speeds(self, left_pct: float, right_pct: float):
        self._set_left(left_pct)
        self._set_right(right_pct)

    # ── public motion primitives ────────────────
    def drive_distance(self, distance_m: float,
                       base_speed: float = BASE_SPEED,
                       update_interval: float = 0.05):
        """
        Drive straight for *distance_m* metres (negative = backward).
        Uses a PID loop on the left/right pulse difference to keep straight.
        """
        sign       = 1 if distance_m >= 0 else -1
        target_m   = abs(distance_m)
        travelled  = 0.0

        pid_integral  = 0.0
        pid_prev_err  = 0.0

        self.odometry.reset()

        print(f"[OdoCtrl] drive_distance: {distance_m:.3f} m")

        while travelled < target_m:
            pl, pr = self.encoder.get_and_reset()
            self.odometry.update(pl * sign, pr * sign)

            d_left  = pl * METRES_PER_PULSE
            d_right = pr * METRES_PER_PULSE
            travelled += (d_left + d_right) / 2.0

            # PID: error = difference between wheels (keep straight)
            error         = d_left - d_right
            pid_integral += error * update_interval
            pid_derivative = (error - pid_prev_err) / update_interval
            correction    = KP * error + KI * pid_integral + KD * pid_derivative
            pid_prev_err  = error

            left_speed  = sign * (base_speed - correction)
            right_speed = sign * (base_speed + correction)

            # Clamp to ±100
            left_speed  = max(-100.0, min(100.0, left_speed))
            right_speed = max(-100.0, min(100.0, right_speed))

            self._apply_speeds(left_speed, right_speed)

            remaining = target_m - travelled
            print(f"  travelled={travelled:.3f} m  remaining={remaining:.3f} m  "
                  f"L={left_speed:.1f}%  R={right_speed:.1f}%")

            time.sleep(update_interval)

        motor_stop()
        print(f"[OdoCtrl] drive_distance done. Pose: {self.odometry.pose}")

    def turn_angle(self, angle_deg: float,
                   base_speed: float = BASE_SPEED * 0.6,
                   update_interval: float = 0.05):
        """
        Turn in place by *angle_deg* degrees.
        Positive = counter-clockwise, negative = clockwise.
        """
        target_rad  = math.radians(angle_deg)
        arc_target  = abs(target_rad) * (WHEEL_BASE_M / 2.0)   # arc each wheel travels
        arc_done    = 0.0

        sign_left  = -1 if angle_deg >= 0 else  1   # CCW: left back, right forward
        sign_right =  1 if angle_deg >= 0 else -1

        self.odometry.reset()

        print(f"[OdoCtrl] turn_angle: {angle_deg:.1f}°")

        while arc_done < arc_target:
            pl, pr = self.encoder.get_and_reset()
            self.odometry.update(pl * sign_left, pr * sign_right)

            d_left  = pl * METRES_PER_PULSE
            d_right = pr * METRES_PER_PULSE
            arc_done += (d_left + d_right) / 2.0

            left_speed  = sign_left  * base_speed
            right_speed = sign_right * base_speed

            self._apply_speeds(left_speed, right_speed)

            remaining_deg = math.degrees(target_rad) - math.degrees(arc_done / (WHEEL_BASE_M / 2.0))
            print(f"  arc_done={arc_done:.4f} m  remaining≈{remaining_deg:.1f}°  "
                  f"θ={math.degrees(self.odometry.theta):.1f}°")

            time.sleep(update_interval)

        motor_stop()
        print(f"[OdoCtrl] turn_angle done. Pose: {self.odometry.pose}")

    def go_to_pose(self, target_x: float, target_y: float,
                   target_theta_deg: float = None,
                   position_tolerance: float = 0.02,
                   angle_tolerance_deg: float = 3.0):
        """
        Navigate to (target_x, target_y) in the odometry frame, then
        optionally rotate to *target_theta_deg*.

        Strategy: turn-then-drive  (simple but effective for open spaces).
        """
        print(f"[OdoCtrl] go_to_pose: ({target_x:.3f}, {target_y:.3f}), "
              f"θ={target_theta_deg}°")

        # ── Step 1: align heading toward target ──
        dx = target_x - self.odometry.x
        dy = target_y - self.odometry.y
        desired_heading = math.atan2(dy, dx)
        heading_error   = math.degrees(
            math.atan2(math.sin(desired_heading - self.odometry.theta),
                       math.cos(desired_heading - self.odometry.theta))
        )

        if abs(heading_error) > angle_tolerance_deg:
            self.turn_angle(heading_error)

        # ── Step 2: drive to target ───────────────
        distance = math.hypot(target_x - self.odometry.x,
                              target_y - self.odometry.y)
        if distance > position_tolerance:
            self.drive_distance(distance)

        # ── Step 3: optional final heading ───────
        if target_theta_deg is not None:
            current_deg = math.degrees(self.odometry.theta)
            final_error = target_theta_deg - current_deg
            # Wrap to (-180, 180]
            final_error = (final_error + 180) % 360 - 180
            if abs(final_error) > angle_tolerance_deg:
                self.turn_angle(final_error)

        print(f"[OdoCtrl] go_to_pose done. Final pose: {self.odometry.pose}")

    def shutdown(self):
        self.encoder.stop()
        destroy()


# ─────────────────────────────────────────────
#  Demo / entry-point
# ─────────────────────────────────────────────
if __name__ == "__main__":
    ctrl = OdometryMotorController()
    try:
        # Drive forward 0.5 m
        ctrl.drive_distance(0.5)
        time.sleep(0.5)

        # Turn 90° counter-clockwise
        ctrl.turn_angle(90)
        time.sleep(0.5)

        # Drive forward another 0.3 m
        ctrl.drive_distance(0.3)
        time.sleep(0.5)

        # Go to an absolute pose (1 m, 1 m) facing 0°
        ctrl.go_to_pose(1.0, 1.0, target_theta_deg=0.0)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        ctrl.shutdown()
