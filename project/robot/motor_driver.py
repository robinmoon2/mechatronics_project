"""
motor_driver.py — DC motor control with optional ADC-based encoder odometry.

Usage:
    from motor_driver import MotorDriver

    driver = MotorDriver()
    driver.forward(50)          # 50% speed
    time.sleep(2)
    odom = driver.get_odometry()  # {'left_pulses': ..., 'right_pulses': ..., 'x': ..., ...}
    driver.stop()
    driver.shutdown()
"""

import time
import math
import threading
import smbus
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor


# ═══════════════════════════════════════════════════════════════
#  Constants — ADJUST TO YOUR ROBOT

# PCA9685 channel assignments
MOTOR_M1_IN1 = 15
MOTOR_M1_IN2 = 14
MOTOR_M2_IN1 = 12
MOTOR_M2_IN2 = 13
MOTOR_M3_IN1 = 11
MOTOR_M3_IN2 = 10
MOTOR_M4_IN1 = 8
MOTOR_M4_IN2 = 9

# Motor grouping: which channels are left / right side
LEFT_CHANNELS  = [1, 3]   # motor1, motor3
RIGHT_CHANNELS = [2, 4]   # motor2, motor4

# Wheel geometry
WHEEL_DIAMETER_M    = 0.07
WHEEL_BASE_M        = 0.20
PULSES_PER_REV      = 11
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER_M
METRES_PER_PULSE    = WHEEL_CIRCUMFERENCE / PULSES_PER_REV

# ADS7830 encoder ADC channels
ENC_LEFT_CH_A  = 6
ENC_LEFT_CH_B  = 7
ENC_RIGHT_CH_A = 4
ENC_RIGHT_CH_B = 5
ADC_THRESHOLD  = 127

# PCA9685 I2C address
PCA_ADDRESS = 0x5F


# ═══════════════════════════════════════════════════════════════
#  ADS7830 ADC

class ADS7830:
    """8-channel, 8-bit ADC over I2C (address 0x48)."""

    def __init__(self, bus_num=1, address=0x48):
        self.bus = smbus.SMBus(bus_num)
        self.address = address

    def read(self, channel: int) -> int:
        """Read raw 0–255 from ADC *channel* (0–7)."""
        cmd = 0x84 | (((channel << 2 | channel >> 1) & 0x07) << 4)
        return self.bus.read_byte_data(self.address, cmd)


# ═══════════════════════════════════════════════════════════════
#  Encoder Reader Thread

class _EncoderReader(threading.Thread):
    """
    Polls ADC channels to count encoder pulses via edge detection.
    Runs as a daemon thread.
    """
    def __init__(self, adc: ADS7830, poll_interval: float = 0.002):
        super().__init__(daemon=True, name="EncoderReader")
        self.adc = adc
        self.poll_interval = poll_interval

        self._lock = threading.Lock()
        self._left_pulses = 0
        self._right_pulses = 0
        self._total_left = 0
        self._total_right = 0

        self._prev_a_left = False
        self._prev_a_right = False
        self._running = True

    def get_and_reset(self) -> tuple:
        """Return (left_pulses, right_pulses) since last call, then reset."""
        with self._lock:
            lp = self._left_pulses
            rp = self._right_pulses
            self._left_pulses = 0
            self._right_pulses = 0
        return lp, rp

    @property
    def totals(self) -> tuple:
        """Cumulative (left, right) pulse counts since start."""
        with self._lock:
            return self._total_left, self._total_right

    def stop(self):
        self._running = False

    def run(self):
        adc = self.adc
        while self._running:
            try:
                al = adc.read(ENC_LEFT_CH_A) > ADC_THRESHOLD
                bl = adc.read(ENC_LEFT_CH_B) > ADC_THRESHOLD
                ar = adc.read(ENC_RIGHT_CH_A) > ADC_THRESHOLD
                br = adc.read(ENC_RIGHT_CH_B) > ADC_THRESHOLD
            except OSError:
                # I2C glitch — skip this cycle
                time.sleep(self.poll_interval)
                continue

            with self._lock:
                # Left encoder — rising edge on A
                if al and not self._prev_a_left:
                    delta = 1 if not bl else -1
                    self._left_pulses += delta
                    self._total_left += delta

                # Right encoder — rising edge on A
                if ar and not self._prev_a_right:
                    delta = 1 if not br else -1
                    self._right_pulses += delta
                    self._total_right += delta

            self._prev_a_left = al
            self._prev_a_right = ar

            time.sleep(self.poll_interval)


# ═══════════════════════════════════════════════════════════════
#  Odometry State
# ═══════════════════════════════════════════════════════════════

class _Odometry:
    """Integrates encoder pulses into (x, y, theta) pose."""

    def __init__(self):
        self.x = 0.0       # metres
        self.y = 0.0       # metres
        self.theta = 0.0   # radians

    def update(self, left_pulses: int, right_pulses: int):
        dl = left_pulses * METRES_PER_PULSE
        dr = right_pulses * METRES_PER_PULSE

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / WHEEL_BASE_M

        self.x += dc * math.cos(self.theta + dtheta / 2.0)
        self.y += dc * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def reset(self):
        self.x = self.y = self.theta = 0.0

    @property
    def pose(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "theta_rad": self.theta,
            "theta_deg": math.degrees(self.theta),
        }


# ═══════════════════════════════════════════════════════════════
#  MotorDriver — the public API
# ═══════════════════════════════════════════════════════════════

class MotorDriver:
    """
    Controls 4 DC motors via PCA9685 and reads 2 quadrature
    encoders via ADS7830 ADC.

    Motors are grouped into LEFT (M1, M3) and RIGHT (M2, M4).

    Usage:
        driver = MotorDriver()
        driver.forward(60)
        driver.set_speeds(left=50, right=70)   # differential
        driver.stop()
        print(driver.get_odometry())
        driver.shutdown()
    """

    def __init__(self, enable_encoders: bool = True):
        # ── PCA9685 ───────────────────────────────────────────
        self._i2c = busio.I2C(SCL, SDA)
        self._pca = PCA9685(self._i2c, address=PCA_ADDRESS)
        self._pca.frequency = 50

        # Silence all channels to prevent gripper surprises
        for ch in range(16):
            self._pca.channels[ch].duty_cycle = 0

        # ── DC motors ─────────────────────────────────────────
        self._motors = {
            1: motor.DCMotor(self._pca.channels[MOTOR_M1_IN1],
                             self._pca.channels[MOTOR_M1_IN2]),
            2: motor.DCMotor(self._pca.channels[MOTOR_M2_IN1],
                             self._pca.channels[MOTOR_M2_IN2]),
            3: motor.DCMotor(self._pca.channels[MOTOR_M3_IN1],
                             self._pca.channels[MOTOR_M3_IN2]),
            4: motor.DCMotor(self._pca.channels[MOTOR_M4_IN1],
                             self._pca.channels[MOTOR_M4_IN2]),
        }
        for m in self._motors.values():
            m.decay_mode = motor.SLOW_DECAY
            m.throttle = 0

        # ── Encoders + odometry ───────────────────────────────
        self._encoders_enabled = enable_encoders
        self._odometry = _Odometry()
        self._encoder_reader = None

        if enable_encoders:
            try:
                adc = ADS7830()
                adc.read(0)  # quick test
                self._encoder_reader = _EncoderReader(adc)
                self._encoder_reader.start()
            except OSError:
                print("[MotorDriver] WARNING: ADS7830 not found — "
                      "encoders disabled")
                self._encoders_enabled = False

    # ── Private helpers ───────────────────────────────────────

    @staticmethod
    def _pct_to_throttle(pct: float) -> float:
        """Convert signed percentage (-100..+100) to throttle (-1..+1)."""
        pct = max(-100.0, min(100.0, pct))
        return pct / 100.0

    def _set_group(self, channels: list, speed_pct: float):
        throttle = self._pct_to_throttle(speed_pct)
        for ch in channels:
            self._motors[ch].throttle = throttle

    def _update_odometry(self):
        if self._encoder_reader is not None:
            lp, rp = self._encoder_reader.get_and_reset()
            self._odometry.update(lp, rp)

    @property
    def pca(self):
        return self._pca
    # ── Public motor control ──────────────────────────────────

    def forward(self, speed_pct: float = 50):
        """Both sides forward at *speed_pct* %."""
        self.set_speeds(speed_pct, speed_pct)

    def backward(self, speed_pct: float = 50):
        """Both sides backward at *speed_pct* %."""
        self.set_speeds(-speed_pct, -speed_pct)

    def set_speeds(self, left_pct: float, right_pct: float):
        """
        Set left and right side speeds independently.
        Positive = forward, negative = backward.
        Range: -100 to +100.
        """
        self._update_odometry()
        self._set_group(LEFT_CHANNELS, left_pct)
        self._set_group(RIGHT_CHANNELS, right_pct)

    def set_motor(self, channel: int, speed_pct: float):
        """Set a single motor (1–4). Positive = forward."""
        if channel not in self._motors:
            raise ValueError(f"Invalid motor channel {channel}. Use 1–4.")
        self._motors[channel].throttle = self._pct_to_throttle(speed_pct)

    def stop(self):
        """Stop all motors immediately."""
        for m in self._motors.values():
            m.throttle = 0
        self._update_odometry()

    # ── Odometry ──────────────────────────────────────────────

    def get_odometry(self) -> dict:
        """
        Return current odometry estimate.
        Also returns raw encoder totals for debugging.
        """
        self._update_odometry()
        result = self._odometry.pose
        if self._encoder_reader is not None:
            tl, tr = self._encoder_reader.totals
            result["total_left_pulses"] = tl
            result["total_right_pulses"] = tr
        return result

    def reset_odometry(self):
        self._odometry.reset()

    def is_moving(self) -> bool:
        """
        Check if any encoder has counted pulses recently.
        Useful as a 'am I stuck?' detector.
        """
        if self._encoder_reader is None:
            return False
        lp, rp = self._encoder_reader.get_and_reset()
        self._odometry.update(lp, rp)
        return abs(lp) > 0 or abs(rp) > 0

    # ── Lifecycle ─────────────────────────────────────────────

    def shutdown(self):
        """Stop motors, stop encoder thread, release PCA9685."""
        self.stop()
        if self._encoder_reader is not None:
            self._encoder_reader.stop()
            self._encoder_reader.join(timeout=1.0)
        # Silence all channels
        for ch in range(16):
            self._pca.channels[ch].duty_cycle = 0
        self._pca.deinit()

    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════
#  Quick self-test

if __name__ == "__main__":
  driver = MotorDriver(enable_encoders=True)

  try:
      # ── Test each motor one at a time ──
      for ch in range(1, 5):
          print(f"\n--- Motor {ch} spinning forward at 40% for 1.5s ---")
          driver.set_motor(ch, 40)
          time.sleep(1.5)
          driver.stop()
          odom = driver.get_odometry()
          print(f"    Encoder totals:  L={odom.get('total_left_pulses','?')}  "
                f"R={odom.get('total_right_pulses','?')}")
          driver.reset_odometry()
          time.sleep(1)

      # ── All forward ──
      print("\n--- All motors forward 50% for 2s ---")
      driver.reset_odometry()
      driver.forward(50)
      time.sleep(2)
      driver.stop()
      print(f"    Odometry: {driver.get_odometry()}")
      time.sleep(1)

      # ── All backward ──
      print("\n--- All motors backward 50% for 2s ---")
      driver.reset_odometry()
      driver.backward(50)
      time.sleep(2)
      driver.stop()
      print(f"    Odometry: {driver.get_odometry()}")

  except KeyboardInterrupt:
      print("\nInterrupted.")
  finally:
      driver.shutdown()
      print("Done.")
