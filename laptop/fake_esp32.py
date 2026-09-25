"""
fake_esp32.py — simulated ESP32 tracker device.

Behaves enough like the real firmware that link.py's parsing, reconnect, and
heartbeat logic run completely unmodified against it. This lets the whole
software team build and test app.py / weather.py / link.py before the
physical tracker exists, and lets the hardware team swap in the real ESP32
later by changing only TRACKER_TRANSPORT (see README.md).

Transport: a plain TCP socket server on 127.0.0.1:9091 by default. link.py
connects to it with pyserial's `serial_for_url("socket://host:port")`, which
pyserial treats exactly like a real Serial object (same .read()/.write()/
.timeout/.dtr/.rts interface) — link.py does not know or care that it isn't
talking to a real USB device. This is the "configurable simulator transport"
approach: no PTYs, no platform-specific device files, works the same on
macOS/Linux/Windows.

The simulated tracker keeps ticking in a background thread regardless of
whether a laptop is connected — exactly like the real ESP32 keeps tracking
(or keeps stowing) when the laptop disappears. A reconnecting laptop just
starts receiving the live telemetry stream again; nothing resets.

Restarting THIS PROCESS simulates a real ESP32 power-cycle: the SAFE latch is
persisted to logs/fake_esp32_state.json (standing in for the ESP32's NVS
flash), so a restart behaves like a real reboot — SAFE survives — but the
millis() counter restarts from ~0, which link.py detects as a reboot event,
exactly as it would from a real device's `B,` boot line or a millis() reset.

Run:
    python fake_esp32.py
    FAKE_ESP32_PORT=9500 python fake_esp32.py   # non-default port
"""
from __future__ import annotations

import json
import math
import os
import random
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link import (  # noqa: E402
    DEFAULT_SIM_HOST, DEFAULT_SIM_PORT, format_telemetry, format_boot,
    parse_command, FLAG_LINK, FLAG_DARK, FLAG_LIMIT, FLAG_MOVING,
)

STATE_FILE = Path(__file__).resolve().parent.parent / "logs" / "fake_esp32_state.json"

# --- tracker tunables (mirrors the real firmware's documented starting values) ---
ANGLE_MIN, ANGLE_MAX = -55.0, 55.0
STOW_DEG = 0.0
E_START, E_STOP = 0.06, 0.02
START_TICKS = 3
KP, STEP_MIN, STEP_MAX = 25.0, 0.2, 2.0
DARK_MV = 300.0
RATE_STOW, RATE_REOPEN, RATE_HOLD, RATE_TRACK = 30.0, 5.0, 20.0, 20.0
LINK_TIMEOUT_S = 5.0
REOPEN_MAX_S = 20.0
TICK_S = 0.1

# --- elevation (top servo): mirrors firmware/solar_tracker/solar_tracker.ino's
#     TRACK_ELEVATION_DEG/STOW_ELEVATION_DEG fallback + clamp behavior, for
#     protocol-level testing. Unlike azimuth, this is NOT part of the frozen
#     telemetry -- it exists here only so authority-hierarchy tests (SAFE
#     overrides a commanded elevation, etc.) can inspect tracker.elev_angle
#     directly. The simulator has no physical servo to calibrate against, so
#     (unlike the firmware's solarElevationToServoTarget()) it applies the
#     commanded elevation directly, clamped to ELEVATION_MIN/MAX_DEG -- no
#     offset/inversion mapping, since there's no real actuator here to need one.
ELEVATION_MIN_DEG, ELEVATION_MAX_DEG = 0.0, 70.0
TRACK_ELEVATION_DEG = 45.0
STOW_ELEVATION_DEG = 0.0
RATE_ELEVATION = 20.0

# --- simulated sun: slowly sweeps back and forth so the dashboard graph
#     visibly shows balance falling and recovering without any manual input ---
SUN_PERIOD_S = 45.0
SUN_AMPLITUDE_DEG = 45.0
SUN_HALF_WIDTH_DEG = 18.0   # matches the real LDR divider-wall geometry
BASE_SUM_MV = 4200.0
AMBIENT_FLOOR = 0.15
NOISE_MV = 40.0


def load_latch() -> bool:
    try:
        return bool(json.loads(STATE_FILE.read_text())["latch"])
    except Exception:
        return False


def save_latch(latch: bool) -> None:
    try:
        STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        STATE_FILE.write_text(json.dumps({"latch": bool(latch)}))
    except OSError:
        pass


class SimTracker:
    """All simulated state. One instance per process, independent of clients."""

    def __init__(self):
        self.boot_time = time.monotonic()
        self.latched = load_latch()
        self.state = "STW" if self.latched else "TRK"
        self.mode = "AUTO"
        self.hold_deg = 0.0
        self.angle = STOW_DEG if self.latched else 0.0
        self.target = self.angle
        self.moving = False
        self.over_count = 0
        self.settled_ticks = 0
        self.rpn_start = 0.0
        self.last_cmd_time = 0.0
        self.link_up = False
        self.dark = False
        self.err = 0.0
        self.L = self.R = BASE_SUM_MV / 2
        self.commanded_elevation: float | None = None
        self.elev_angle = STOW_ELEVATION_DEG if self.latched else TRACK_ELEVATION_DEG
        self._lock = threading.Lock()

    def apply_command(self, verdict: str, mode: str, hold: float,
                       elevation: float | None = None) -> None:
        with self._lock:
            if verdict == "SAFE":
                self._set_latch(True)
            elif verdict == "OK":
                self._set_latch(False)
            # UNKNOWN: latch untouched — this is the whole point of the latch.
            self.mode = mode
            self.hold_deg = max(ANGLE_MIN, min(ANGLE_MAX, hold))
            self.commanded_elevation = elevation
            self.last_cmd_time = time.monotonic()

    def _set_latch(self, value: bool) -> None:
        if value != self.latched:
            self.latched = value
            save_latch(value)

    # --- simulated light model ---

    def _sun_angle(self, t: float) -> float:
        return SUN_AMPLITUDE_DEG * math.sin(2 * math.pi * t / SUN_PERIOD_S)

    def _sample_ldrs(self, t: float):
        diff = self._sun_angle(t) - self.angle
        falloff = max(AMBIENT_FLOOR, math.cos(math.radians(diff)))
        total = BASE_SUM_MV * falloff
        e_model = max(-1.0, min(1.0, diff / SUN_HALF_WIDTH_DEG)) * 0.9
        L = total * (1 + e_model) / 2 + random.uniform(-NOISE_MV, NOISE_MV)
        R = total * (1 - e_model) / 2 + random.uniform(-NOISE_MV, NOISE_MV)
        return max(0.0, L), max(0.0, R)

    def _track_step(self) -> float:
        if self.dark:
            self.moving = False
            self.over_count = 0
            return 0.0
        if self.moving:
            if abs(self.err) < E_STOP:
                self.moving = False
        else:
            self.over_count = self.over_count + 1 if abs(self.err) > E_START else 0
            if self.over_count >= START_TICKS:
                self.moving = True
                self.over_count = 0
        if not self.moving:
            return 0.0
        step = max(STEP_MIN, min(STEP_MAX, KP * abs(self.err)))
        return step if self.err > 0 else -step

    def _reopen_done(self, now: float) -> bool:
        if now - self.rpn_start > REOPEN_MAX_S:
            return True
        if self.mode == "HOLD":
            return abs(self.angle - self.hold_deg) < 1.0
        return self.dark or self.settled_ticks >= 10

    def tick(self) -> str:
        """Advance the simulation by one TICK_S and return the T, telemetry line."""
        now = time.monotonic()
        with self._lock:
            self.link_up = (now - self.last_cmd_time) < LINK_TIMEOUT_S
            if not self.link_up:
                self.mode = "AUTO"

            L, R = self._sample_ldrs(now - self.boot_time)
            self.L, self.R = L, R
            total = L + R
            self.dark = total < DARK_MV
            self.err = 0.0 if self.dark else (L - R) / total

            # 1) state selection — the only place the state changes
            if self.latched:
                self.state = "STW"
            elif self.state == "STW":
                self.state = "RPN"
                self.rpn_start = now
                self.settled_ticks = 0
            elif self.state == "RPN":
                if self._reopen_done(now):
                    self.state = "HLD" if self.mode == "HOLD" else "TRK"
            else:
                self.state = "HLD" if self.mode == "HOLD" else "TRK"

            # 2) target + rate — the only place the target is set
            if self.state == "STW":
                self.target, rate = STOW_DEG, RATE_STOW
            elif self.state == "HLD":
                self.target, rate = self.hold_deg, RATE_HOLD
            elif self.state == "RPN":
                self.target = self.hold_deg if self.mode == "HOLD" else self.angle + self._track_step()
                rate = RATE_REOPEN
            else:
                self.target = self.angle + self._track_step()
                rate = RATE_TRACK

            self.target = max(ANGLE_MIN, min(ANGLE_MAX, self.target))

            # 3) rate limit + drive — the only place the angle changes
            max_step = rate * TICK_S
            delta = max(-max_step, min(max_step, self.target - self.angle))
            self.angle += delta
            self.settled_ticks = 0 if self.moving else self.settled_ticks + 1

            # 4) elevation (top servo) — STOW always wins, same authority
            #    order as azimuth above; otherwise use the last commanded
            #    solar elevation if we have one, else the fixed fallback.
            if self.state == "STW":
                elev_target = STOW_ELEVATION_DEG
            elif self.commanded_elevation is not None:
                elev_target = self.commanded_elevation
            else:
                elev_target = TRACK_ELEVATION_DEG
            elev_target = max(ELEVATION_MIN_DEG, min(ELEVATION_MAX_DEG, elev_target))
            elev_max_step = RATE_ELEVATION * TICK_S
            elev_delta = max(-elev_max_step, min(elev_max_step, elev_target - self.elev_angle))
            self.elev_angle += elev_delta

            ms = int((now - self.boot_time) * 1000)
            at_limit = self.angle <= ANGLE_MIN + 0.1 or self.angle >= ANGLE_MAX - 0.1
            flags = ((FLAG_LINK if self.link_up else 0) |
                     (FLAG_DARK if self.dark else 0) |
                     (FLAG_LIMIT if at_limit else 0) |
                     (FLAG_MOVING if self.moving else 0))
            return format_telemetry(ms, self.state, self.angle, self.target, self.L, self.R, self.err, flags)


clients: set = set()
clients_lock = threading.Lock()


def run_control_loop(tracker: SimTracker, stop_event: threading.Event) -> None:
    next_tick = time.monotonic()
    while not stop_event.is_set():
        line = (tracker.tick() + "\n").encode("ascii")
        with clients_lock:
            dead = []
            for c in clients:
                try:
                    c.sendall(line)
                except OSError:
                    dead.append(c)
            for c in dead:
                clients.discard(c)
        next_tick += TICK_S
        time.sleep(max(0.0, next_tick - time.monotonic()))


def handle_client(conn: socket.socket, tracker: SimTracker, boot_line: str, process_boot_time: float) -> None:
    conn.settimeout(1.0)
    buf = b""
    with clients_lock:
        clients.add(conn)
    peer = conn.getpeername()
    print(f"[fake_esp32] client connected: {peer}")
    # If a client connects shortly after process start, replay the boot banner
    # (mirrors a real device still printing its boot message when you plug in).
    if time.monotonic() - process_boot_time < 2.0:
        try:
            conn.sendall((boot_line + "\n").encode("ascii"))
        except OSError:
            pass
    try:
        while True:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "ignore").strip()
                if not line:
                    continue
                cmd = parse_command(line)
                if cmd:
                    tracker.apply_command(*cmd)
                # malformed commands are silently ignored, same as the firmware
    except OSError:
        pass
    finally:
        with clients_lock:
            clients.discard(conn)
        conn.close()
        print(f"[fake_esp32] client disconnected: {peer}")


def main() -> None:
    host = os.environ.get("FAKE_ESP32_HOST", DEFAULT_SIM_HOST)
    port = int(os.environ.get("FAKE_ESP32_PORT", str(DEFAULT_SIM_PORT)))

    tracker = SimTracker()
    boot_line = format_boot(reset=0, latch=tracker.latched)
    process_boot_time = time.monotonic()
    print(f"[fake_esp32] {boot_line}")
    print(f"[fake_esp32] simulated tracker starting, latch={tracker.latched} "
          f"(state file: {STATE_FILE})")

    stop_event = threading.Event()
    threading.Thread(target=run_control_loop, args=(tracker, stop_event), daemon=True).start()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind((host, port))
    except OSError as exc:
        print(f"[fake_esp32] FAILED to bind {host}:{port} — {exc}")
        print("[fake_esp32] is another fake_esp32.py already running? "
              "Set FAKE_ESP32_PORT to use a different port.")
        sys.exit(1)
    srv.listen(4)
    print(f"[fake_esp32] listening on {host}:{port}")
    print("[fake_esp32] link.py will find this automatically with TRACKER_TRANSPORT=sim (the default)")
    try:
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=handle_client, args=(conn, tracker, boot_line, process_boot_time),
                              daemon=True).start()
    except KeyboardInterrupt:
        print("\n[fake_esp32] shutting down")
    finally:
        stop_event.set()
        srv.close()


if __name__ == "__main__":
    main()
