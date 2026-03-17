"""
Navigator — reads GPSReceiver, computes control signals,
drives FrontWheels / BackWheels.
No UDP code here.
"""

import logging
import math
import time

from adeept_picarpro.front_wheels import FrontWheels
from adeept_picarpro.back_wheels  import BackWheels

from gps_receiver import GPSReceiver, TargetReading

log = logging.getLogger("navigator")

# ── Tuning constants ───────────────────────────────────────────
STEER_CENTER      = 90       # servo neutral (degrees)
STEER_MAX_OFFSET  = 30       # max ± offset from center
BASE_SPEED        = 60       # PWM when far from target
MIN_SPEED         = 35       # minimum PWM to keep moving
ARRIVAL_DIST      = 0.05     # 5 cm → consider arrived
SLOW_DIST         = 0.30     # 30 cm → start slowing down

# P-controller gains
KP_STEER  = 1.8     # steering gain  (bearing_deg → servo offset)
KP_SPEED  = 200.0   # speed gain     (distance → PWM reduction)


class Navigator:
    def __init__(self, gps: GPSReceiver):
        self.gps          = gps
        self.front        = FrontWheels()
        self.back         = BackWheels()
        self._is_moving   = False

        # Reset hardware
        self.front.turn(STEER_CENTER)
        self.back.stop()

    # ── High-level API ────────────────────────────────────────
    def navigate_to_target(
        self,
        timeout: float = 60.0,
        update_hz: float = 20.0,
    ) -> bool:
        """
        Drive toward the current GPS target until arrival or timeout.
        Returns True  if arrived,
                False if timeout or GPS lost.
        """
        log.info("Starting navigation to current target...")
        deadline = time.time() + timeout
        dt       = 1.0 / update_hz

        try:
            while time.time() < deadline:
                r = self.gps.reading

                # ── Safety: stop if GPS is stale ──────────────
                if not r.valid:
                    log.warning("GPS stale — stopping motors")
                    self._stop()
                    time.sleep(0.1)
                    continue

                # ── Arrival check ──────────────────────────────
                if r.distance <= ARRIVAL_DIST:
                    log.info(
                        "✅ Arrived at target ID %d  (dist=%.3f m)",
                        r.target_id, r.distance
                    )
                    self._stop()
                    return True

                # ── Compute control signals ────────────────────
                steer_cmd, speed_cmd = self._compute_control(r)

                # ── Apply to hardware ──────────────────────────
                self._apply(steer_cmd, speed_cmd)

                log.debug(
                    "dist=%.3f  bearing=%.1f°  steer=%d  speed=%d",
                    r.distance, r.bearing_deg, steer_cmd, speed_cmd
                )

                time.sleep(dt)

        except KeyboardInterrupt:
            pass
        finally:
            self._stop()

        log.warning("Navigation timeout or interrupted.")
        return False

    def stop(self):
        self._stop()

    # ── Control law ───────────────────────────────────────────
    def _compute_control(
        self, r: TargetReading
    ) -> tuple[int, int]:
        """
        Returns (steer_angle_deg, speed_pwm)

        Steering  : proportional to bearing error
                    bearing=0  → straight ahead → STEER_CENTER
                    bearing>0  → target right  → turn right (> CENTER)
                    bearing<0  → target left   → turn left  (< CENTER)

        Speed     : full speed when far, ramp down near target
        """
        # ── Steering ──────────────────────────────────────────
        raw_offset = KP_STEER * r.bearing_deg
        offset     = max(-STEER_MAX_OFFSET,
                         min( STEER_MAX_OFFSET, raw_offset))
        steer      = int(STEER_CENTER + offset)

        # ── Speed (distance-based ramp) ────────────────────────
        if r.distance >= SLOW_DIST:
            speed = BASE_SPEED
        else:
            # linear ramp: BASE_SPEED → MIN_SPEED over [SLOW_DIST, ARRIVAL_DIST]
            ratio = (r.distance - ARRIVAL_DIST) / (SLOW_DIST - ARRIVAL_DIST)
            ratio = max(0.0, min(1.0, ratio))
            speed = int(MIN_SPEED + ratio * (BASE_SPEED - MIN_SPEED))

        return steer, speed

    # ── Hardware helpers ──────────────────────────────────────
    def _apply(self, steer: int, speed: int):
        self.front.turn(steer)
        self.back.speed = speed
        if not self._is_moving:
            self.back.forward()
            self._is_moving = True

    def _stop(self):
        self.back.stop()
        self.front.turn(STEER_CENTER)
        self._is_moving = False
