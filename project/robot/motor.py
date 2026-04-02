#!/usr/bin/env python3
# File: dc_motor_encoder.py

import time
import math
import threading
import smbus
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor as adafruit_motor

# ── PCA9685 setup ────────────────────────────────────────────────────────────
i2c       = busio.I2C(SCL, SDA)
pwm_motor = PCA9685(i2c, address=0x5F)
pwm_motor.frequency = 50

# ── ADS7830 ADC ──────────────────────────────────────────────────────────────
class ADS7830:
    I2C_ADDRESS = 0x48
    CMD_BASE    = 0x84

    def __init__(self):
        self.bus = smbus.SMBus(1)

    def read(self, channel: int) -> int:
        cmd = self.CMD_BASE | (((channel << 2 | channel >> 1) & 0x07) << 4)
        return self.bus.read_byte_data(self.I2C_ADDRESS, cmd)

# Shared ADC instance
_adc = ADS7830()

# ── Physical constants ───────────────────────────────────────────────────────
WHEEL_DIAMETER_M    = 0.07
PULSES_PER_REV      = 11
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER_M
METRES_PER_PULSE    = WHEEL_CIRCUMFERENCE / PULSES_PER_REV
ADC_THRESHOLD       = 127

# ── DCMotorWithEncoder ───────────────────────────────────────────────────────
class DCMotorWithEncoder:
    """
    One DC motor driven through PCA9685 + one quadrature encoder read via ADS7830.

    Parameters
    ----------
    pin_in1, pin_in2 : PCA9685 channel indices for the motor H-bridge
    enc_ch_a, enc_ch_b : ADS7830 channel indices for encoder A / B signals
    name : optional label for logging
    poll_interval : encoder polling period in seconds
    """

    def __init__(self,
                 pin_in1:      int,
                 pin_in2:      int,
                 enc_ch_a:     int,
                 enc_ch_b:     int,
                 name:         str   = "motor",
                 poll_interval: float = 0.002):

        self.name = name

        # ── Motor ────────────────────────────────────────────────
        self._motor = adafruit_motor.DCMotor(
            pwm_motor.channels[pin_in1],
            pwm_motor.channels[pin_in2],
        )
        self._motor.decay_mode = adafruit_motor.SLOW_DECAY

        # ── Encoder ──────────────────────────────────────────────
        self._enc_ch_a     = enc_ch_a
        self._enc_ch_b     = enc_ch_b
        self._poll_interval = poll_interval

        self._lock         = threading.Lock()
        self._pulses       = 0          # signed pulse counter
        self._prev_a       = False

        # Speed estimation
        self._last_time    = time.monotonic()
        self._speed_mps    = 0.0        # metres per second (updated each edge)

        # ── Start polling thread ──────────────────────────────────
        self._running = True
        self._thread  = threading.Thread(target=self._poll, daemon=True)
        self._thread.start()

    # ── Speed control ────────────────────────────────────────────────────────
    def set_speed(self, speed_pct: float):
        """
        Set motor speed.
        speed_pct : -100 (full reverse) … +100 (full forward)
        """
        speed_pct = max(-100.0, min(100.0, speed_pct))
        self._motor.throttle = speed_pct / 100.0

    def stop(self):
        self._motor.throttle = 0

    # ── Encoder / odometry API ───────────────────────────────────────────────
    @property
    def pulses(self) -> int:
        """Cumulative signed pulse count since last reset."""
        with self._lock:
            return self._pulses

    @property
    def distance_m(self) -> float:
        """Total signed distance travelled since last reset (metres)."""
        return self.pulses * METRES_PER_PULSE

    @property
    def speed_mps(self) -> float:
        """Instantaneous speed in metres per second (updated on each pulse edge)."""
        with self._lock:
            return self._speed_mps
        
    
    def reset(self):
        """Reset pulse counter and distance to zero."""
        with self._lock:
            self._pulses   = 0
            self._speed_mps = 0.0
            self._last_time = time.monotonic()

    def get_and_reset(self) -> int:
        """Return current pulse count then reset it (useful for odometry loops)."""
        with self._lock:
            p = self._pulses
            self._pulses = 0
        return p

    # ── Background polling ───────────────────────────────────────────────────
    def _poll(self):
        while self._running:
            raw_a = _adc.read(self._enc_ch_a)
            raw_b = _adc.read(self._enc_ch_b)

            a = raw_a > ADC_THRESHOLD
            b = raw_b > ADC_THRESHOLD

            # Rising edge on channel A → count pulse, use B for direction
            if a and not self._prev_a:
                direction = 1 if not b else -1
                now       = time.monotonic()

                with self._lock:
                    self._pulses += direction
                    dt = now - self._last_time
                    if dt > 0:
                        self._speed_mps = (METRES_PER_PULSE / dt) * direction
                    self._last_time = now

            self._prev_a = a
            time.sleep(self._poll_interval)

    # ── Lifecycle ────────────────────────────────────────────────────────────
    def shutdown(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.stop()

    def __repr__(self):
        return (f"DCMotorWithEncoder(name={self.name!r}, "
                f"pulses={self.pulses}, "
                f"distance={self.distance_m:.4f} m, "
                f"speed={self.speed_mps:.3f} m/s)")
