"""
GPS Server — receives ArUco position packets from the client,
navigates toward target, avoids obstacles, then resumes.
Cycles through a list of target marker IDs.
"""

import json
import logging
import socket
import time
import os
import threading
from dataclasses import dataclass, field

from navigator import GPS_coordinates, Navigation, STEER_CENTER, SPEED_MIN
from obstacle_avoidance import check_obstacle_avg, avoid_obstacle, OBSTACLE_DIST_CM
from config import RobotConfig

#  Logging 
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("gps_server")

cfg = RobotConfig()

# --- Config 
HOST = cfg.HOST
PORT = cfg.PORT
BUFFER_SIZE = cfg.BUFFER_SIZE
ROBOT_ID = cfg.ROBOT_ID
GPS_TIMEOUT = cfg.GPS_TIMEOUT

# --- Target waypoint list (ArUco IDs in order) 
TARGET_IDS = cfg.TARGET_IDS
PAUSE_AT_TARGET = cfg.PAUSE_AT_TARGET

# -- Obstacle polling 
OBSTACLE_CHECK_HZ = cfg.OBSTACLE_CHECK_HZ

# --- Stop for object 
STOP_ID = cfg.STOP_ID
OBJECT_ID = cfg.OBJECT_ID

@dataclass
class MarkerState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    last_seen: float = field(default_factory=time.time)
    update_count: int = 0

# - Shared state between threads --
nav_lock          = threading.Lock()
obstacle_flag     = threading.Event()
stop_event        = threading.Event()

def obstacle_monitor():
    while not stop_event.is_set():
        dist = check_obstacle_avg(n=3)
        if dist < OBSTACLE_DIST_CM:
            obstacle_flag.set()
        else:
            obstacle_flag.clear()
        time.sleep(1.0 / OBSTACLE_CHECK_HZ)

#  Dashboard 
def clear():
    os.system("cls" if os.name == "nt" else "clear")

def render_dashboard(states, client_ip, frame_id, pkt_count,
                     fps, distance, angle, nav_mode, is_avoiding,
                     current_target_id, waypoint_index, total_waypoints):
    clear()
    print("╔══════════════════════════════════════════════════════╗")
    print("║              ArUco GPS — Robot Receiver              ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  Client : {client_ip:<43}║")
    print(f"║  Frame  : {frame_id:<6}   Packets: {pkt_count:<6}   FPS: {fps:<6.1f} ║")
    print(f"║  Mode   : {nav_mode:<43}║")
    wp_str = f"Target ID={current_target_id}  ({waypoint_index+1}/{total_waypoints})"
    print(f"║  Waypnt : {wp_str:<43}║")
    avoid_str = "AVOIDING" if is_avoiding else "NAVIGATING"
    print(f"║  State  : {avoid_str:<43}║")
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
    print(f" ANGLE    : {angle:.4f}"      if angle    is not None else " ANGLE    : N/A")
    obs_dist = check_obstacle_avg(n=1)
    print(f" OBSTACLE : {obs_dist:.1f} cm")
    print("  Press Ctrl+C to stop.")

#  Main server loop 
def run():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(1.0)

    states     = {}
    client_ip  = "waiting..."
    frame_id   = 0
    pkt_count  = 0
    fps_window = []
    fps        = 0.0

    nav: Navigation | None = None
    is_avoiding = False

    # Waypoint tracking
    waypoint_index = 0
    all_done = False

    monitor = threading.Thread(target=obstacle_monitor, daemon=True)
    monitor.start()

    try:
        while True:
            if all_done:
                log.info("[NAV] All waypoints reached — stopping.")
                break

            current_target_id = TARGET_IDS[waypoint_index]
            target_id_str = str(current_target_id)
            robot_id_str  = str(ROBOT_ID)

            #  Receive UDP packet 
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
            except socket.timeout:
                with nav_lock:
                    if nav is not None and not nav.finished and not is_avoiding:
                        if obstacle_flag.is_set():
                            log.info("[MAIN] Obstacle detected — avoiding")
                            is_avoiding = True
                            avoid_obstacle(nav)
                            is_avoiding = False
                            log.info("[MAIN] Avoidance done — resuming")
                        else:
                            nav.navigation_choice(gps_alive=False)
                continue

            #  Parse 
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

            #  Check marker freshness 
            robot_fresh  = (robot_id_str in states and
                            now - states[robot_id_str].last_seen < GPS_TIMEOUT)
            target_fresh = (target_id_str in states and
                            now - states[target_id_str].last_seen < GPS_TIMEOUT)

            with nav_lock:
                #  Create or update Navigation 
                if robot_fresh and target_fresh:
                    robot_s  = states[robot_id_str]
                    target_s = states[target_id_str]

                    if nav is None:
                        nav = Navigation(
                            robot=GPS_coordinates(
                                x=robot_s.x, y=robot_s.y, z=robot_s.z),
                            target=GPS_coordinates(
                                x=target_s.x, y=target_s.y, z=target_s.z)
                        )
                        log.info("[NAV] Navigation created : target ID %d (%d/%d)",
                                 current_target_id, waypoint_index+1, len(TARGET_IDS))
                    elif not is_avoiding:
                        nav.prev_robot.x = nav.robot.x
                        nav.prev_robot.y = nav.robot.y
                        nav.robot.x  = robot_s.x
                        nav.robot.y  = robot_s.y
                        nav.robot.z  = robot_s.z
                        nav.target.x = target_s.x
                        nav.target.y = target_s.y
                        nav.target.z = target_s.z

                #  Navigate or avoid 
                nav_mode = "Waiting"
                if nav is not None and not nav.finished:
                    if obstacle_flag.is_set() and not is_avoiding:
                        log.info("[MAIN] Obstacle detected — avoiding")
                        is_avoiding = True
                        avoid_obstacle(nav)
                        is_avoiding = False
                        nav_mode = "Avoidance done"
                        log.info("[MAIN] Avoidance done — resuming")
                    elif not is_avoiding:
                        gps_alive = robot_fresh and target_fresh
                        nav.navigation_choice(gps_alive=gps_alive)
                        nav_mode = "GPS" if gps_alive else "Odometry"

                #  Waypoint reached — pause & advance 
                elif nav is not None and nav.finished:
                    nav.driver.stop()
                    log.info("[NAV] Reached target ID %d (%d/%d) — pausing %.1fs",
                             current_target_id, waypoint_index+1, len(TARGET_IDS),
                             PAUSE_AT_TARGET)
                    nav_mode = f"Paused at ID {current_target_id}"

                    # Release lock during sleep so other threads aren't blocked
                    time.sleep(PAUSE_AT_TARGET)
                    waypoint_index += 1
                    
                    # if current_target_id == STOP_ID:
                    #     # we grab the object
                    #     tourelle_angle, cx, dist = nav.localize_object()
                    # if tourelle_angle is not None:
                    #     log.info("[NAV] Found at tourelle=%d° dist=%.3fm — approaching", tourelle_angle, dist)
                        
                    #     steer_correction = STEER_CENTER - (tourelle_angle - 90) * 0.8
                    #     nav.set_steering(steer_correction)
                    #     # Now approach using camera feedback
                    #     grabbed = nav.approach_and_grab(min_dist=0.10)

                    #     if grabbed:
                    #         log.info("[NAV] Object grabbed!")
                    #     else:
                    #         log.warning("[NAV] Approach failed")
                    # else:
                    #     log.warning("[NAV] Object not found in sweep")                        
                        
                    if waypoint_index >= len(TARGET_IDS):
                        log.info("[NAV] All %d waypoints completed!", len(TARGET_IDS))
                        all_done = True
                        nav_mode = "All done"
                    else:
                        # Reset nav for next target
                        next_target_id = TARGET_IDS[waypoint_index]
                        next_target_str = str(next_target_id)
                        log.info("[NAV] Next target: ID %d (%d/%d)",
                                 next_target_id, waypoint_index+1, len(TARGET_IDS))

                        # If we already see the next target, update immediately
                        if next_target_str in states:
                            ts = states[next_target_str]
                            nav.target.x = ts.x
                            nav.target.y = ts.y
                            nav.target.z = ts.z

                        # Reset finished flag so navigation resumes
                        nav.finished = False
                        nav.driver.reset_odometry()
                        nav_mode = f"Heading to ID {next_target_id}"

            #  Dashboard 
            distance = nav.distance()         if nav else None
            angle    = nav.compute_steering() if nav else None
            # render_dashboard(
            #     states, client_ip, frame_id, pkt_count,
            #     fps, distance, angle, nav_mode, is_avoiding,
            #     current_target_id, waypoint_index, len(TARGET_IDS)
            # )

    except KeyboardInterrupt:
        log.info("Server stopped.")
    finally:
        stop_event.set()
        if nav is not None:
            nav.set_steering(STEER_CENTER)
            nav.destroy()
        sock.close()
        log.info("[SHUTDOWN] Done")

if __name__ == "__main__":
    run()
