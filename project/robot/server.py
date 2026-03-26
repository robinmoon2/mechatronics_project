"""
GPS Server — receives ArUco position packets from the client
and displays them in a clean terminal dashboard.
"""

import json
import logging
import socket
import time
import os
import threading
import math
from dataclasses import dataclass, field
from enum import Enum, auto

from navigator import GPS_coordinates, Navigation, STEER_CENTER
from gpiozero import DistanceSensor

# ── Ultrasonic ─────────────────────────────────────────────────
TRIG            = 23
ECHO_PIN        = 24
OBSTACLE_DIST_CM = 25

sensor = DistanceSensor(echo=ECHO_PIN, trigger=TRIG, max_distance=2)

def checkdist() -> float:
    return sensor.distance * 100  # cm

# ── State machine enums ────────────────────────────────────────
class RobotState(Enum):
    NAVIGATING = auto()
    OBSTACLE   = auto()
    AVOIDING   = auto()
    RESUME     = auto()
    FINISHED   = auto()

class AvoidStep(Enum):
    TURN_RIGHT     = auto()   # rotate 90° right
    FORWARD_CLEAR  = auto()   # advance until front is clear
    TURN_LEFT_1    = auto()   # rotate 90° left  (restore heading)
    FORWARD_PASS   = auto()   # advance to clear the obstacle width
    TURN_LEFT_2    = auto()   # rotate 90° left
    FORWARD_ALIGN  = auto()   # advance to realign laterally
    TURN_RIGHT_2   = auto()   # rotate 90° right (restore heading)
    DONE           = auto()

# ── Avoidance timing constants (tune on your robot!) ──────────
TURN_90_S      = 0.8   # seconds for a 90° turn
FORWARD_PASS_S = 1.5   # seconds to clear obstacle side
FORWARD_ALIGN_S= 1.5   # seconds to realign laterally

# ── Logging ────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("gps_server")

# ── Config ─────────────────────────────────────────────────────
HOST      = "0.0.0.0"
PORT      = 5005
BUFFER_SIZE = 4096
ROBOT_ID  = 5
TARGET_ID = 1
GPS_TIMEOUT = 0.2

@dataclass
class MarkerState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    last_seen: float = field(default_factory=time.time)
    update_count: int = 0

# ── Obstacle monitor thread ────────────────────────────────────
obstacle_detected = threading.Event()
nav_lock          = threading.Lock()

def obstacle_monitor(nav_ref: dict):
    """Only reads sensor and sets the event — never touches motors."""
    while not nav_ref.get("stop"):
        dist_cm = checkdist()
        if dist_cm < OBSTACLE_DIST_CM:
            obstacle_detected.set()
        else:
            obstacle_detected.clear()
        time.sleep(0.05)   # 20 Hz

# ── Dashboard ──────────────────────────────────────────────────
def clear():
    os.system("cls" if os.name == "nt" else "clear")

def render_dashboard(states, client_ip, frame_id, pkt_count,
                     fps, distance, speed, angle, nav_mode, robot_state):
    clear()
    print("╔══════════════════════════════════════════════════════╗")
    print("║              ArUco GPS — Robot Receiver              ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  Client : {client_ip:<43}║")
    print(f"║  Frame  : {frame_id:<6}   Packets: {pkt_count:<6}   FPS: {fps:<6.1f} ║")
    print(f"║  Mode   : {nav_mode:<43}║")
    print(f"║  State  : {robot_state.name:<43}║")
    print("╠════════╦══════════════╦══════════════╦══════════════╣")
    print("║  ID    ║     X (m)    ║     Y (m)    ║     Z (m)    ║")
    print("╠════════╬══════════════╬══════════════╬══════════════╣")
    if not states:
        print("║  --    ║   No markers visible                        ║")
    else:
        for mid, s in sorted(states.items(), key=lambda kv: int(kv[0])):
            age   = time.time() - s.last_seen
            stale = "  ⚠ stale" if age > 1.0 else ""
            print(f"║  {mid:<5} ║ {s.x:>+12.4f} ║ {s.y:>+12.4f} ║ {s.z:>+12.4f} ║{stale}")
    print("╚════════╩══════════════╩══════════════╩══════════════╝")
    print(f" DISTANCE : {distance:.4f} m" if distance is not None else " DISTANCE : N/A")
    print(f" SPEED    : {speed:.4f}"      if speed    is not None else " SPEED    : N/A")
    print(f" ANGLE    : {angle:.4f}"      if angle    is not None else " ANGLE    : N/A")
    print(f" OBSTACLE : {checkdist():.1f} cm")
    print("  Press Ctrl+C to stop.")

# ── State machine ──────────────────────────────────────────────
class RobotController:
    """
    Single place that controls the motors.
    Call update() every loop iteration.
    """

    def __init__(self):
        self.nav:        Navigation | None = None
        self.state:      RobotState  = RobotState.NAVIGATING
        self.avoid_step: AvoidStep   = AvoidStep.TURN_RIGHT
        self.step_start: float       = 0.0
        self.saved_heading: float    = 0.0   # heading before avoidance

    # ── public ────────────────────────────────────────────────
    def set_nav(self, nav: Navigation):
        self.nav = nav

    def update(self, robot_fresh: bool, target_fresh: bool) -> str:
        """
        Call once per loop.
        Returns a human-readable nav_mode string.
        """
        if self.nav is None:
            return "⏳ Waiting for markers"

        if self.state == RobotState.FINISHED:
            self.nav.driver.stop()
            return "🏁 Finished"

        obstacle = obstacle_detected.is_set()

        # ── Transitions ──────────────────────────────────────
        if self.state == RobotState.NAVIGATING:
            if self.nav.finished:
                self.state = RobotState.FINISHED
                self.nav.driver.stop()
                return "🏁 Finished"

            if obstacle:
                print("[STATE] NAVIGATING → OBSTACLE")
                self.nav.driver.stop()
                self.state = RobotState.OBSTACLE
                return "🛑 OBSTACLE"

            # Normal navigation
            if robot_fresh and target_fresh:
                self.nav.navigation_choice(gps_alive=True)
                return "🛰️  GPS"
            else:
                self.nav.navigation_choice(gps_alive=False)
                return "📏 Odometry"

        elif self.state == RobotState.OBSTACLE:
            # Save heading then start avoidance
            self.saved_heading = self.nav.robot_heading
            self.avoid_step    = AvoidStep.TURN_RIGHT
            self.step_start    = time.time()
            self.state         = RobotState.AVOIDING
            print(f"[STATE] OBSTACLE → AVOIDING  (saved heading={math.degrees(self.saved_heading):.1f}°)")
            return "⚠️  AVOIDING"

        elif self.state == RobotState.AVOIDING:
            return self._run_avoidance()

        elif self.state == RobotState.RESUME:
            # GPS will naturally correct — hand back to NAVIGATING
            print("[STATE] RESUME → NAVIGATING")
            self.state = RobotState.NAVIGATING
            return "🔄 Resume"

        return "❓ Unknown"

    # ── avoidance steps ───────────────────────────────────────
    def _run_avoidance(self) -> str:
        elapsed = time.time() - self.step_start
        step    = self.avoid_step

        if step == AvoidStep.TURN_RIGHT:
            # Steer hard right, drive forward to pivot
            self.nav.set_steering(STEER_CENTER + 40)
            self.nav.driver.forward(30)
            if elapsed >= TURN_90_S:
                self._next_step()

        elif step == AvoidStep.FORWARD_CLEAR:
            # Go straight until sensor sees no obstacle
            self.nav.set_steering(STEER_CENTER)
            self.nav.driver.forward(30)
            if not obstacle_detected.is_set():
                self._next_step()

        elif step == AvoidStep.TURN_LEFT_1:
            # Steer hard left to restore original heading
            self.nav.set_steering(STEER_CENTER - 40)
            self.nav.driver.forward(30)
            if elapsed >= TURN_90_S:
                self._next_step()

        elif step == AvoidStep.FORWARD_PASS:
            # Go straight to pass the obstacle width
            self.nav.set_steering(STEER_CENTER)
            self.nav.driver.forward(30)
            if elapsed >= FORWARD_PASS_S:
                self._next_step()

        elif step == AvoidStep.TURN_LEFT_2:
            # Steer left to move back toward original line
            self.nav.set_steering(STEER_CENTER - 40)
            self.nav.driver.forward(30)
            if elapsed >= TURN_90_S:
                self._next_step()

        elif step == AvoidStep.FORWARD_ALIGN:
            # Go straight to realign laterally
            self.nav.set_steering(STEER_CENTER)
            self.nav.driver.forward(30)
            if elapsed >= FORWARD_ALIGN_S:
                self._next_step()

        elif step == AvoidStep.TURN_RIGHT_2:
            # Turn right to restore original heading
            self.nav.set_steering(STEER_CENTER + 40)
            self.nav.driver.forward(30)
            if elapsed >= TURN_90_S:
                self._next_step()

        elif step == AvoidStep.DONE:
            self.nav.driver.stop()
            self.nav.set_steering(STEER_CENTER)
            self.state = RobotState.RESUME
            print("[STATE] AVOIDING → RESUME")

        return f"⚠️  AVOIDING [{step.name}]"

    def _next_step(self):
        steps = list(AvoidStep)
        idx   = steps.index(self.avoid_step)
        self.avoid_step = steps[idx + 1]
        self.step_start = time.time()
        print(f"[AVOID] → {self.avoid_step.name}")


# ── Main server loop ───────────────────────────────────────────
def run():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(1.0)

    states     = {}
    client_ip  = "waiting..."
    frame_id   = 0
    pkt_count  = 0
    distance   = None
    speed      = None
    angle      = None
    fps_window = []
    fps        = 0.0

    controller = RobotController()
    nav_ref    = {"nav": None, "stop": False}

    # Start obstacle monitor thread
    monitor_thread = threading.Thread(
        target=obstacle_monitor,
        args=(nav_ref,),
        daemon=True
    )
    monitor_thread.start()

    try:
        while True:
            # ── Receive UDP packet ─────────────────────────────
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
            except socket.timeout:
                # Still run the state machine on timeout
                with nav_lock:
                    nav_mode = controller.update(
                        robot_fresh=False,
                        target_fresh=False
                    )
                render_dashboard(
                    states, client_ip, frame_id, pkt_count,
                    fps, distance, speed, angle, nav_mode, controller.state
                )
                continue

            # ── Parse ──────────────────────────────────────────
            try:
                msg = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                continue

            client_ip = f"{addr[0]}:{addr[1]}"
            frame_id  = msg.get("frame", 0)
            markers   = msg.get("markers", {})

            now = time.time()
            for mid, pos in markers.items():
                if mid not in states:
                    states[mid] = MarkerState()
                s = states[mid]
                s.x, s.y, s.z = pos["x"], pos["y"], pos["z"]
                s.last_seen    = now
                s.update_count += 1

            fps_window.append(now)
            fps_window = [t for t in fps_window if now - t <= 1.0]
            fps       = float(len(fps_window))
            pkt_count += 1

            # ── Update nav object ──────────────────────────────
            robot_id_str  = str(ROBOT_ID)
            target_id_str = str(TARGET_ID)

            robot_fresh  = (robot_id_str  in states and
                            now - states[robot_id_str].last_seen  < GPS_TIMEOUT)
            target_fresh = (target_id_str in states and
                            now - states[target_id_str].last_seen < GPS_TIMEOUT)

            with nav_lock:
                if robot_fresh and target_fresh:
                    robot_s  = states[robot_id_str]
                    target_s = states[target_id_str]

                    if controller.nav is None:
                        # First time: create Navigation
                        nav = Navigation(
                            robot=GPS_coordinates(
                                x=robot_s.x, y=robot_s.y, z=robot_s.z),
                            target=GPS_coordinates(
                                x=target_s.x, y=target_s.y, z=target_s.z)
                        )
                        controller.set_nav(nav)
                        nav_ref["nav"] = nav
                        print("[NAV] Navigation object created")
                    elif controller.state not in (RobotState.AVOIDING, RobotState.OBSTACLE):
                        nav = controller.nav
                        
                        if not robot_fresh: # was in odom and GPS again
                            nav.prev_robot.x = robot_s.x
                            nav.prev_robot.y = robot_s.y

                        nav.robot.x  = robot_s.x
                        nav.robot.y  = robot_s.y
                        nav.robot.z  = robot_s.z

                        # ── Target: always update when visible
                        nav.target.x = target_s.x
                        nav.target.y = target_s.y
                        nav.target.z = target_s.z
                    else:
                        # Only update GPS positions when NOT avoiding
                        if controller.state not in (
                            RobotState.AVOIDING, RobotState.OBSTACLE
                        ):
                            controller.nav.robot.x  = robot_s.x
                            controller.nav.robot.y  = robot_s.y
                            controller.nav.robot.z  = robot_s.z
                            controller.nav.target.x = target_s.x
                            controller.nav.target.y = target_s.y
                            controller.nav.target.z = target_s.z

                # ── Run state machine (ONE place that controls motors)
                nav_mode = controller.update(robot_fresh, target_fresh)

            nav = controller.nav
            distance = nav.distance()         if nav else None
            angle    = nav.compute_steering() if nav else None
            speed    = None if nav is None else (0.0 if nav.finished else None)

            render_dashboard(
                states, client_ip, frame_id, pkt_count,
                fps, distance, speed, angle, nav_mode, controller.state
            )

    except KeyboardInterrupt:
        log.info("Server stopped.")
    finally:
        nav_ref["stop"] = True
        nav = controller.nav
        if nav is not None:
            nav.set_steering(STEER_CENTER)
            nav.destroy()
        sock.close()

if __name__ == "__main__":
    run()
