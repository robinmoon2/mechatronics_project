"""
GPS Receiver — listens for ArUco position packets,
exposes clean data to the rest of the system.
No motor logic here.
"""

import json
import logging
import socket
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger("gps_receiver")

# ── Config ─────────────────────────────────────────────────────
HOST        = "0.0.0.0"
PORT        = 5005
CLIENT_PORT = 5006        # port we reply to on the PC
BUFFER_SIZE = 4096

# ── Parsed position ────────────────────────────────────────────
@dataclass
class TargetReading:
    """
    Position of the TARGET marker relative to the ORIGIN marker.
    x : right  (m)
    y : forward(m)   ← main axis for distance
    z : up     (m)   (not used for 2D nav)
    age  : seconds since last valid packet
    valid: False if no packet received recently
    """
    x:        float = 0.0
    y:        float = 0.0
    z:        float = 0.0
    target_id: int  = -1
    frame:     int  = 0
    timestamp: float = field(default_factory=time.time)

    @property
    def age(self) -> float:
        return time.time() - self.timestamp

    @property
    def valid(self) -> bool:
        return self.age < 1.0      # stale after 1 s

    @property
    def distance(self) -> float:
        """Euclidean distance in the XY plane (m)."""
        return (self.x ** 2 + self.y ** 2) ** 0.5

    @property
    def bearing_deg(self) -> float:
        """
        Angle to target in DEGREES relative to robot forward axis.
        - Negative = target is to the LEFT
        - Positive = target is to the RIGHT
        0° means straight ahead.
        """
        import math
        return math.degrees(math.atan2(self.x, self.y))


class GPSReceiver:
    """
    Threaded UDP receiver.
    Call .start() once, then read .reading from anywhere.
    """

    def __init__(self, host: str = HOST, port: int = PORT):
        self._host    = host
        self._port    = port
        self._lock    = threading.Lock()
        self._reading = TargetReading()
        self._client_addr: tuple | None = None

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self._host, self._port))
        self._sock.settimeout(0.5)

        self._running = False
        self._thread  = threading.Thread(target=self._loop, daemon=True)

    # ── Public API ────────────────────────────────────────────
    def start(self):
        self._running = True
        self._thread.start()
        log.info("GPSReceiver listening on %s:%d", self._host, self._port)

    def stop(self):
        self._running = False
        self._sock.close()

    @property
    def reading(self) -> TargetReading:
        with self._lock:
            # return a copy so caller can't mutate internal state
            r = self._reading
            return TargetReading(
                x=r.x, y=r.y, z=r.z,
                target_id=r.target_id,
                frame=r.frame,
                timestamp=r.timestamp,
            )

    def send_command(self, cmd: dict):
        """Send a JSON command back to the client PC."""
        if self._client_addr is None:
            return
        payload = json.dumps(cmd).encode("utf-8")
        self._sock.sendto(payload, self._client_addr)

    # ── Internal ──────────────────────────────────────────────
    def _loop(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(BUFFER_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break

            # store client address for replies
            self._client_addr = (addr[0], CLIENT_PORT)

            try:
                msg = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                log.warning("Bad packet — skipped")
                continue

            if msg.get("type") != "position":
                continue

            markers = msg.get("markers", {})
            tid     = msg.get("target_id", -1)
            tid_str = str(tid)

            if tid_str not in markers:
                continue                         # target not visible

            pos = markers[tid_str]

            with self._lock:
                self._reading = TargetReading(
                    x=pos["x"],
                    y=pos["y"],
                    z=pos["z"],
                    target_id=tid,
                    frame=msg.get("frame", 0),
                    timestamp=time.time(),
                )

            # ack
            self.send_command({"type": "ack", "frame": msg.get("frame", 0)})
