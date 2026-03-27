"""
dashboard.py — Non-blocking terminal dashboard running in its own thread.
"""

import threading
import time
import math
import os
from dataclasses import dataclass
from typing import Optional, Dict


class Dashboard:
    """
    Renders robot state to terminal in a background thread.
    Main loop writes to shared state; dashboard reads it independently.
    """

    def __init__(self, refresh_hz: float = 5.0):
        self._refresh_interval = 1.0 / refresh_hz
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # ── Shared state (written by main loop, read by dashboard) ──
        self._states: Dict         = {}
        self._robot_state: str     = "INIT"
        self._nav_data: dict       = {}
        self._packet_count: int    = 0
        self._last_packet_time: float = 0.0
        self._obstacle_dist: float = 0.0

    # ── Public API (called from main loop) ─────────────────────

    def update(
        self,
        states: dict,
        robot_state,
        nav=None,
        packet_count: int = 0,
        obstacle_dist: float = 0.0,
    ):
        """
        Thread-safe update of dashboard data.
        Returns immediately — never blocks the caller.
        """
        nav_data = {}
        if nav is not None:
            try:
                nav_data = {
                    "robot_x":   nav.robot.x,
                    "robot_y":   nav.robot.y,
                    "target_x":  nav.target.x,
                    "target_y":  nav.target.y,
                    "heading":   math.degrees(nav.robot_heading),
                    "distance":  nav.distance(),
                    "steering":  nav.current_steering,
                    "finished":  nav.finished,
                }
            except Exception:
                pass  # nav object mid-update, skip this frame

        with self._lock:
            self._states          = dict(states)
            self._robot_state     = str(robot_state)
            self._nav_data        = nav_data
            self._packet_count    = packet_count
            self._obstacle_dist   = obstacle_dist
            self._last_packet_time = time.time()

    def start(self):
        """Start background render thread."""
        self._running = True
        self._thread = threading.Thread(
            target=self._render_loop,
            name="dashboard",
            daemon=True,          # dies automatically when main exits
        )
        self._thread.start()

    def stop(self):
        """Stop background render thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    # ── Internal render loop (runs in background thread) ───────

    def _render_loop(self):
        while self._running:
            start = time.monotonic()
            try:
                self._render()
            except Exception:
                pass  # Never crash the render thread
            elapsed = time.monotonic() - start
            sleep_time = self._refresh_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _render(self):
        # Take a snapshot under lock — minimise lock hold time
        with self._lock:
            states          = dict(self._states)
            robot_state     = self._robot_state
            nav_data        = dict(self._nav_data)
            packet_count    = self._packet_count
            obstacle_dist   = self._obstacle_dist
            last_pkt        = self._last_packet_time

        now     = time.time()
        age     = now - last_pkt if last_pkt else 0.0
        lines   = []

        lines.append("╔══════════════════════════════════════════╗")
        lines.append("║           GPS NAVIGATION DASHBOARD        ║")
        lines.append("╠══════════════════════════════════════════╣")

        # ── Connection status ─────────────────────────────────
        conn_ok  = age < 1.0
        conn_sym = "🟢" if conn_ok else "🔴"
        lines.append(f"║  {conn_sym} Packets: {packet_count:<6d}  "
                     f"Last: {age:>5.2f}s ago        ║")

        # ── Robot state ───────────────────────────────────────
        lines.append(f"║  State   : {robot_state:<30s} ║")

        # ── Obstacle ──────────────────────────────────────────
        obs_sym = "⚠️ " if obstacle_dist < 15 else "  "
        lines.append(f"║  Obstacle: {obs_sym}{obstacle_dist:>6.1f} cm                  ║")

        lines.append("╠══════════════════════════════════════════╣")

        # ── Marker states ─────────────────────────────────────
        if states:
            lines.append("║  MARKERS                                 ║")
            for mid, s in states.items():
                age_m = now - s.get("last_seen", now)
                fresh = "●" if age_m < 0.5 else "○"
                lines.append(
                    f"║  {fresh} ID{mid:>2s}  "
                    f"x={s.get('x', 0):>7.3f}  "
                    f"y={s.get('y', 0):>7.3f}  "
                    f"z={s.get('z', 0):>6.3f}  ║"
                )
        else:
            lines.append("║  No markers visible                      ║")

        lines.append("╠══════════════════════════════════════════╣")

        # ── Navigation data ───────────────────────────────────
        if nav_data:
            lines.append("║  NAVIGATION                              ║")
            lines.append(
                f"║  Robot  : ({nav_data['robot_x']:>7.3f}, "
                f"{nav_data['robot_y']:>7.3f})              ║"
            )
            lines.append(
                f"║  Target : ({nav_data['target_x']:>7.3f}, "
                f"{nav_data['target_y']:>7.3f})              ║"
            )
            lines.append(
                f"║  Heading: {nav_data['heading']:>7.1f}°  "
                f"Dist: {nav_data['distance']:>7.3f} m       ║"
            )
            steer_deg = math.degrees(nav_data['steering'])
            lines.append(
                f"║  Steer  : {steer_deg:>7.1f}°                        ║"
            )
            fin = "YES ✓" if nav_data["finished"] else "no"
            lines.append(f"║  Finished: {fin:<30s} ║")
        else:
            lines.append("║  Navigation not initialised              ║")

        lines.append("╚══════════════════════════════════════════╝")

        # ── Single atomic write ───────────────────────────────
        output = "\033[H\033[J" + "\n".join(lines)  # clear + render
        print(output, end="", flush=True)
