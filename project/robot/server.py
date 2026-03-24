"""
GPS Server — receives ArUco position packets from the client
and displays them in a clean terminal dashboard.
"""

import json
import logging
import socket
import time
from dataclasses import dataclass, field
from navigator import GPS_coordinates, Navigation
import os

# ── Logging ────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("gps_server")

@dataclass
class MarkerState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    last_seen: float = field(default_factory=time.time)
    update_count: int = 0

# ── Config ─────────────────────────────────────────────────────
HOST = "0.0.0.0"
PORT = 5005
BUFFER_SIZE = 4096
ROBOT_ID = 5
TARGET_ID = 1

def clear():
    os.system("cls" if os.name == "nt" else "clear")

def render_dashboard(
    states:     dict,
    client_ip:  str,
    frame_id:   int,
    pkt_count:  int,
    fps:        float,
    distance:   float,
    speed:      float,
    angle:      float,
    nav_mode:   str = "N/A"
):
    clear()
    print("╔══════════════════════════════════════════════════════╗")
    print("║              ArUco GPS — Robot Receiver              ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  Client : {client_ip:<43}║")
    print(f"║  Frame  : {frame_id:<6}   Packets: {pkt_count:<6}   FPS: {fps:<6.1f} ║")
    print(f"║  Mode   : {nav_mode:<43}║")
    print("╠════════╦══════════════╦══════════════╦══════════════╣")
    print("║  ID    ║     X (m)    ║     Y (m)    ║     Z (m)    ║")
    print("╠════════╬══════════════╬══════════════╬══════════════╣")
    if not states:
        print("║  --    ║   No markers visible                        ║")
    else:
        for mid, s in sorted(states.items(), key=lambda kv: int(kv[0])):
            age = time.time() - s.last_seen
            stale = "  ⚠ stale" if age > 1.0 else ""
            print(
                f"║  {mid:<5} ║ {s.x:>+12.4f} ║ {s.y:>+12.4f} ║ {s.z:>+12.4f} ║{stale}"
            )

    print("╚════════╩══════════════╩══════════════╩══════════════╝")
    print(f" DISTANCE : {distance:.4f} m" if distance is not None else " DISTANCE : N/A")
    print(f" SPEED    : {speed:.4f}"      if speed    is not None else " SPEED    : N/A")
    print(f" ANGLE    : {angle:.4f}"      if angle    is not None else " ANGLE    : N/A")
    print("  Press Ctrl+C to stop.")


# RPM = (delta_ticks / ticks_per_revolution) × (60 / delta_time)


def run():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(1.0)

    log.info("Listening on UDP %s:%d ...", HOST, PORT)

    states:    dict = {}
    client_ip  = "waiting..."
    frame_id   = 0
    pkt_count  = 0
    distance   = None
    speed      = None
    nav        = None
    angle      = None
    nav_mode   = None

    fps_window: list = []
    fps = 0.0

    try:
        while True:
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
            except socket.timeout:
                # No packet for 1s — safety stop
                if nav is not None:
                    nav.driver.stop()
                    nav.set_steering(90)
                render_dashboard(states, client_ip, frame_id,
                                 pkt_count, fps, distance, speed, angle)
                continue

            # ── Parse ─────────────────────────────────────────
            try:
                msg = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                log.warning("Bad packet from %s — skipped", addr)
                continue

            client_ip = f"{addr[0]}:{addr[1]}"
            frame_id  = msg.get("frame", 0)
            markers   = msg.get("markers", {})

            # ── Update states ─────────────────────────────────
            now = time.time()
            for mid, pos in markers.items():
                if mid not in states:
                    states[mid] = MarkerState()
                s = states[mid]
                s.x, s.y, s.z = pos["x"], pos["y"], pos["z"]
                s.last_seen    = now
                s.update_count += 1

            # ── FPS ───────────────────────────────────────────
            fps_window.append(now)
            fps_window = [t for t in fps_window if now - t <= 1.0]
            fps = float(len(fps_window))
            pkt_count += 1

            # ── Navigation ────────────────────────────────────
            robot_id_str  = str(ROBOT_ID)
            target_id_str = str(TARGET_ID)

            GPS_TIMEOUT = 0.2  # seconds

            robot_fresh  = (robot_id_str in states and 
                           (now - states[robot_id_str].last_seen) < GPS_TIMEOUT)
            target_fresh = (target_id_str in states and 
                           (now - states[target_id_str].last_seen) < GPS_TIMEOUT)

            if robot_fresh and target_fresh:
                robot_s  = states[robot_id_str]
                target_s = states[target_id_str]

                if nav is None:
                    nav = Navigation(
                        robot=GPS_coordinates(x=robot_s.x, y=robot_s.y, z=robot_s.z),
                        target=GPS_coordinates(x=target_s.x, y=target_s.y, z=target_s.z)
                    )
                else:
                    nav.robot.x, nav.robot.y, nav.robot.z = robot_s.x, robot_s.y, robot_s.z
                    nav.target.x, nav.target.y, nav.target.z = target_s.x, target_s.y, target_s.z

                nav.navigation_choice(gps_alive=True)
                nav_mode = "🛰️  GPS"

            else:
                if nav is not None:
                    nav.navigation_choice(gps_alive=False)
                    nav_mode = "📏 Odometry"
                else:
                    nav_mode = "⏳ Waiting for markers"

            distance = nav.distance() if nav else None
            angle    = nav.compute_steering() if nav else None
            speed    = 0.0 if (nav and nav.finished) else None
            
            # ── Render ────────────────────────────────────────
            #render_dashboard(states, client_ip, frame_id,
            #                 pkt_count, fps, distance, speed, angle, nav_mode)

    except KeyboardInterrupt:
        log.info("Server stopped.")
    finally:
        if nav is not None:
            nav.set_steering(90)
            nav.destroy()
        sock.close()


if __name__ == "__main__":
    run()
