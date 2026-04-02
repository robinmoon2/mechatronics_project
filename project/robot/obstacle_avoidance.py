"""
obstacle_avoidance.py — simple functions to dodge an obstacle and resume.

Strategy:
  1. Stop
  2. Steer right + forward until path is clear
  3. Resume navigation

Usage from server.py:
    from obstacle_avoidance import check_obstacle, avoid_obstacle

    dist = check_obstacle()
    if dist < OBSTACLE_DIST_CM:
        avoid_obstacle(nav)
    else:
        nav.navigation_choice(gps_alive=...)
"""

import time
import RPi.GPIO as GPIO

from config import RobotConfig

cfg = RobotConfig()

# ── Ultrasonic pins ───────────────────────────────────────────
TRIG = 23
ECHO_PIN = 24

# ── Thresholds ─────────────────────────────────────────────────
OBSTACLE_DIST_CM = cfg.OBSTACLE_DIST_CM
CLEAR_DIST_CM = cfg.CLEAR_DIST_CM

# ── Avoidance tuning ──────────────────────────────────────────
STOP_PAUSE = cfg.STOP_PAUSE
TURN_CHECK_INTERVAL = cfg.TURN_CHECK_INTERVAL
MAX_TURN_TIME = cfg.MAX_TURN_TIME
MIN_TURN_TIME = cfg.MIN_TURN_TIME
AVOID_SPEED = cfg.AVOID_SPEED
STEER_CENTER = cfg.STEER_CENTER
STEER_RIGHT = cfg.STEER_CENTER + cfg.STEER_ADJUST

# ── GPIO setup ─────────────────────────────────────────────────
_gpio_ready = False

def _ensure_gpio():
    global _gpio_ready
    if not _gpio_ready:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO_PIN, GPIO.IN)
        GPIO.output(TRIG, False)
        time.sleep(0.05)
        _gpio_ready = True

def check_obstacle() -> float:
    """Return front distance in cm (999 on timeout)."""
    _ensure_gpio()
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    timeout = time.time() + 0.04
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return 999.0

    pulse_end = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return 999.0

    return round((pulse_end - pulse_start) * 17150, 2)

def check_obstacle_avg(n: int = 3) -> float:
    """Median of n readings."""
    readings = sorted(check_obstacle() for _ in range(n))
    return readings[n // 2]

def avoid_obstacle(nav):
    """
    Blocking avoidance: stop, then steer right + drive forward
    until the ultrasonic sensor says the path is clear.
    """
    print("[AVOID]  Obstacle detected — turning right")

    # ── Stop ───────────────────────────────────────────────────
    nav.driver.stop()
    nav.set_steering(STEER_CENTER)
    nav.driver.backward(AVOID_SPEED)
    
    time.sleep(STOP_PAUSE)
    nav.driver.stop()

    # ── Steer right + forward until clear ──────────────────────
    nav.set_steering(STEER_RIGHT)
    nav.driver.forward(AVOID_SPEED)

    t_start = time.time()
    while (time.time() - t_start) < MAX_TURN_TIME:
        dist = check_obstacle_avg()
        print(f"[AVOID] turning right — sensor: {dist:.1f} cm")

        if dist >= CLEAR_DIST_CM and (time.time() - t_start) > MIN_TURN_TIME:
            print("[AVOID] Path is clear!")
            break

        time.sleep(TURN_CHECK_INTERVAL)
    else:
        print("[AVOID] Max turn time reached — proceeding anyway")

    # ── Straighten and go ──────────────────────────────────────
    nav.set_steering(STEER_CENTER)
    nav.driver.forward(AVOID_SPEED)
    print("[AVOID]  Avoidance complete — resuming ")


# ══════════════════════════════════════════════════════════════
#  Standalone test
# ══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    from navigator import Navigation, GPS_coordinates

    nav = Navigation(
        robot=GPS_coordinates(0, 0, 0),
        target=GPS_coordinates(1, 0, 0),
    )
    nav.set_steering(STEER_CENTER)

    try:
        print("[TEST] Driving forward — will avoid obstacles")
        nav.driver.forward(AVOID_SPEED)

        while True:
            dist = check_obstacle_avg()
            print(f"  distance: {dist:.1f} cm", end="\r")

            if dist < OBSTACLE_DIST_CM:
                avoid_obstacle(nav)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[TEST] Stopped")
    finally:
        nav.set_steering(STEER_CENTER)
        nav.destroy()
        GPIO.cleanup()
