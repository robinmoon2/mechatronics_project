"""
Main entry point — wires GPS receiver → Navigator → mission loop.
"""

import logging
import time

from gps_receiver import GPSReceiver
from navigator    import Navigator

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("main")

# ── Mission definition ─────────────────────────────────────────
WAYPOINTS = [1, 4, 2]       # ArUco IDs to visit in order

def run_mission():
    # ── Start GPS receiver thread ──────────────────────────────
    gps = GPSReceiver()
    gps.start()

    nav = Navigator(gps)

    log.info("Waiting for first GPS packet...")
    while not gps.reading.valid:
        time.sleep(0.1)
    log.info("GPS ready ✓")

    # ── Mission loop ───────────────────────────────────────────
    for i, waypoint_id in enumerate(WAYPOINTS):
        log.info("── Waypoint %d/%d  (ID=%d) ──",
                 i + 1, len(WAYPOINTS), waypoint_id)

        # Tell the client PC which marker to track
        gps.send_command({
            "type":      "set_target",
            "target_id": waypoint_id,
        })

        # Wait briefly so client switches target
        time.sleep(0.3)

        # Drive to it
        arrived = nav.navigate_to_target(timeout=60.0)

        if not arrived:
            log.error("Failed to reach waypoint ID=%d — aborting", waypoint_id)
            break

        log.info("Reached waypoint %d ✓  pausing 1 s...", waypoint_id)
        time.sleep(1.0)

    else:
        log.info("🏁 All waypoints completed!")

    gps.send_command({"type": "mission_complete"})
    gps.stop()


if __name__ == "__main__":
    try:
        run_mission()
    except KeyboardInterrupt:
        log.info("Mission aborted by user.")
