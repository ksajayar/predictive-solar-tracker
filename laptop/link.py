"""
link.py — owns the connection to the ESP32 (real or simulated).

No other module talks to the serial/socket transport directly. app.py and
weather.py only ever call Link.snapshot() / Link.set_verdict() / Link.set_mode().

This file also holds the FROZEN wire protocol as small, pure, side-effect-free
functions (parse_telemetry, format_telemetry, parse_command, format_command,
parse_boot, format_boot). They have no dependency on the Link class or on any
transport, so tests/test_protocol.py imports them directly, and fake_esp32.py
imports them too — both sides of the wire share one implementation of the
framing, so they cannot drift apart.

Protocol (frozen — see CLAUDE.md before changing it):

  ESP32 -> laptop, ~10 Hz:
      T,<ms>,<state>,<angle>,<target>,<L>,<R>,<err>,<flags>
      e.g. T,183420,TRK,31.5,33.0,1820,2410,-0.139,9
      state:  TRK | STW | RPN | HLD
      flags:  1=laptop link active  2=dark/no light  4=angle limit  8=correcting

      B,tracker,v1,reset=<n>,latch=<0|1>      (sent once, at boot)

  laptop -> ESP32, 1 Hz heartbeat + immediately on change:
      C,<verdict>,<mode>,<hold_angle>[,<elevation_target>]
      e.g. C,OK,AUTO,0   C,SAFE,AUTO,0   C,UNKNOWN,AUTO,0   C,OK,HOLD,20
           C,OK,AUTO,0,57.4   (calculated solar elevation, degrees)
      verdict: OK | SAFE | UNKNOWN
      mode:    AUTO | HOLD
      elevation_target: OPTIONAL 5th field, added for calculated solar
        elevation (see laptop/solar_position.py). Omitting it is still
        valid -- older firmware/tooling that only knows the 4-field form
        keeps working unmodified; a device that understands the 5th field
        falls back to its own fixed elevation set-point when it's absent.

Transport:
  TRACKER_TRANSPORT=sim (default)   -> connect to the fake_esp32.py TCP server
                                        at socket://FAKE_ESP32_HOST:FAKE_ESP32_PORT
  TRACKER_TRANSPORT=serial          -> auto-detect a real ESP32 by USB VID, or
                                        use TRACKER_PORT if set explicitly
  TRACKER_PORT                      -> overrides discovery entirely (device path
                                        like /dev/cu.usbserial-XXXX, or a full
                                        "socket://host:port" URL)

Both transports are opened through pyserial's serial_for_url(), so the rest of
this file (and all of app.py/weather.py) is completely transport-agnostic.
"""
from __future__ import annotations

import collections
import csv
import os
import threading
import time
from pathlib import Path
from typing import Optional

import serial
from serial.tools import list_ports

# --------------------------------------------------------------------------
# Protocol constants
# --------------------------------------------------------------------------

TELEMETRY_STATES = {"TRK", "STW", "RPN", "HLD"}
VERDICTS = {"OK", "SAFE", "UNKNOWN"}
MODES = {"AUTO", "HOLD"}

TELEMETRY_FIELDS = ("ms", "state", "angle", "target", "L", "R", "err", "flags")

FLAG_LINK = 1     # ESP32's own view of whether the laptop heartbeat is current
FLAG_DARK = 2      # not enough light to compute a useful error
FLAG_LIMIT = 4     # commanded angle is at (or within 0.1 deg of) a travel limit
FLAG_MOVING = 8    # tracker is actively correcting this tick

BAUD = 115200

# Real hardware USB-serial chip VIDs commonly used on ESP32 dev boards.
ESP32_USB_VIDS = {0x10C4, 0x1A86, 0x0403, 0x303A}  # CP210x, CH340, FTDI, Espressif native USB

# Simulator transport defaults (also imported by fake_esp32.py, single source of truth)
DEFAULT_SIM_HOST = "127.0.0.1"
DEFAULT_SIM_PORT = 9091

# --------------------------------------------------------------------------
# Protocol: pure parse / format functions
# --------------------------------------------------------------------------


def parse_telemetry(line: str) -> Optional[dict]:
    """Parse one 'T,...' line. Returns None (never raises) for anything malformed."""
    fields = line.strip().split(",")
    if len(fields) != 9 or fields[0] != "T":
        return None
    if fields[2] not in TELEMETRY_STATES:
        return None
    try:
        return {
            "ms": int(fields[1]),
            "state": fields[2],
            "angle": float(fields[3]),
            "target": float(fields[4]),
            "L": float(fields[5]),
            "R": float(fields[6]),
            "err": float(fields[7]),
            "flags": int(fields[8]),
        }
    except ValueError:
        return None


def format_telemetry(ms: int, state: str, angle: float, target: float,
                      L: float, R: float, err: float, flags: int) -> str:
    if state not in TELEMETRY_STATES:
        raise ValueError(f"unknown telemetry state: {state!r}")
    return f"T,{int(ms)},{state},{angle:.1f},{target:.1f},{L:.0f},{R:.0f},{err:.3f},{int(flags)}"


def parse_boot(line: str) -> Optional[dict]:
    """Parse a 'B,tracker,v1,reset=<n>,latch=<0|1>' line. None if not a boot line."""
    line = line.strip()
    if not line.startswith("B,"):
        return None
    out: dict = {"raw": line}
    for part in line.split(",")[1:]:
        if "=" in part:
            k, v = part.split("=", 1)
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
    return out


def format_boot(reset: int = 0, latch: bool = False) -> str:
    return f"B,tracker,v1,reset={int(reset)},latch={int(bool(latch))}"


def parse_command(line: str) -> Optional[tuple]:
    """Parse a 'C,<verdict>,<mode>,<hold>[,<elevation>]' line ->
    (verdict, mode, hold_deg, elevation_deg_or_None), or None if malformed.

    Accepts exactly 4 fields (old form, elevation=None) or exactly 5 fields
    (new form, with a calculated solar elevation target) -- anything else
    (3 fields, 6 fields, ...) is rejected, same as before this field existed.
    """
    fields = line.strip().split(",")
    if len(fields) not in (4, 5) or fields[0] != "C":
        return None
    verdict, mode, hold_s = fields[1], fields[2], fields[3]
    if verdict not in VERDICTS or mode not in MODES:
        return None
    try:
        hold = float(hold_s)
    except ValueError:
        return None
    elevation = None
    if len(fields) == 5:
        try:
            elevation = float(fields[4])
        except ValueError:
            return None
    return (verdict, mode, hold, elevation)


def format_command(verdict: str, mode: str, hold: float = 0.0,
                    elevation: Optional[float] = None) -> str:
    """`elevation=None` (the default) emits the old 4-field line, unchanged
    byte-for-byte from before this field existed -- a receiver that has
    never heard of elevation targets sees exactly what it always has."""
    if verdict not in VERDICTS:
        raise ValueError(f"unknown verdict: {verdict!r}")
    if mode not in MODES:
        raise ValueError(f"unknown mode: {mode!r}")
    line = f"C,{verdict},{mode},{float(hold):.1f}"
    if elevation is not None:
        line += f",{float(elevation):.1f}"
    return line


# --------------------------------------------------------------------------
# Link: owns the transport, runs a background thread, exposes thread-safe
# snapshots. This is the ONLY place that touches `serial`/the socket.
# --------------------------------------------------------------------------

HEARTBEAT_INTERVAL_S = 1.0     # resend the command at least this often
RECONNECT_RETRY_S = 1.0        # how often to retry opening the transport
READ_TIMEOUT_S = 0.2           # pyserial read timeout, keeps the loop responsive
STALE_TELEMETRY_S = 2.5        # no T line for this long -> reported as disconnected
FORCE_RECONNECT_S = 5.0        # no T line for this long -> proactively reopen


class Link:
    """Background serial/socket link to the ESP32 (or fake_esp32.py)."""

    def __init__(self, port: Optional[str] = None, log_dir: Optional[str] = None,
                 log_telemetry: bool = True):
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self.latest: Optional[dict] = None
        self.history = collections.deque(maxlen=900)  # ~90s at 10 Hz
        self.events = collections.deque(maxlen=50)
        self.bad_lines = 0
        self.boots = 0

        self.port_spec = port          # explicit override passed by caller
        self.port: Optional[str] = None  # the port actually in use once connected
        self._last_rx = 0.0
        self._last_ms: Optional[int] = None

        # Safe default: never claim OK until weather.py explicitly decides.
        # 4th slot (elevation) defaults to None: no calculated solar
        # elevation known yet -> format_command() omits the 5th wire field
        # entirely, so an ESP32 falls back to its own fixed set-point,
        # exactly as if talking to code from before this field existed.
        self._cmd = ["UNKNOWN", "AUTO", 0.0, None]
        self._dirty = True
        self._last_tx = 0.0
        self._ser = None

        self._log_file = None
        self._csv = None
        self.log_path = None
        if log_telemetry:
            log_dir_path = Path(log_dir) if log_dir else Path(__file__).resolve().parent.parent / "logs"
            log_dir_path.mkdir(parents=True, exist_ok=True)
            self.log_path = log_dir_path / time.strftime("telemetry_%Y%m%d_%H%M%S.csv")
            self._log_file = open(self.log_path, "w", newline="")
            self._csv = csv.writer(self._log_file)
            self._csv.writerow(["rx_time"] + list(TELEMETRY_FIELDS))

    # ---- lifecycle ----

    def start(self) -> "Link":
        threading.Thread(target=self._run, daemon=True, name="link-rx").start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._close()
        if self._log_file is not None:
            try:
                self._log_file.close()
            except Exception:
                pass

    # ---- command API (called from weather.py / app.py) ----

    def set_verdict(self, verdict: str) -> None:
        if verdict not in VERDICTS:
            raise ValueError(f"unknown verdict: {verdict!r}")
        self._set(0, verdict)

    def set_mode(self, mode: str, hold: float = 0.0) -> None:
        if mode not in MODES:
            raise ValueError(f"unknown mode: {mode!r}")
        self._set(1, mode)
        self._set(2, float(hold))

    def set_elevation(self, elevation: Optional[float]) -> None:
        """Calculated solar elevation target, degrees, or None to omit the
        field entirely (falls back to the device's own fixed set-point)."""
        self._set(3, None if elevation is None else float(elevation))

    def _set(self, idx: int, value) -> None:
        with self._lock:
            if self._cmd[idx] != value:
                self._cmd[idx] = value
                self._dirty = True

    # ---- read-side API (called from app.py) ----

    def snapshot(self) -> dict:
        with self._lock:
            latest = dict(self.latest) if self.latest else None
            history = list(self.history)
            cmd = list(self._cmd)
            events = list(self.events)
        now = time.time()
        connected = self._ser is not None and latest is not None and (now - self._last_rx) < STALE_TELEMETRY_S
        return {
            "connected": connected,
            "port": self.port,
            "latest": latest,
            "history": history,
            "cmd": cmd,
            "bad_lines": self.bad_lines,
            "boots": self.boots,
            "events": events,
            "log_path": str(self.log_path) if self.log_path else None,
        }

    # ---- internals ----

    def _event(self, msg: str) -> None:
        self.events.append(f"{time.strftime('%H:%M:%S')}  {msg}")
        print(f"[link] {msg}")

    def _resolve_port(self) -> Optional[str]:
        if self.port_spec:
            return self.port_spec
        env_port = os.environ.get("TRACKER_PORT")
        if env_port:
            return env_port

        transport = os.environ.get("TRACKER_TRANSPORT", "sim").strip().lower()
        if transport == "sim":
            host = os.environ.get("FAKE_ESP32_HOST", DEFAULT_SIM_HOST)
            port = os.environ.get("FAKE_ESP32_PORT", str(DEFAULT_SIM_PORT))
            return f"socket://{host}:{port}"

        # transport == "serial": auto-detect real hardware by USB VID
        for p in list_ports.comports():
            if p.vid in ESP32_USB_VIDS:
                return p.device
        return None

    def _connect(self) -> bool:
        target = self._resolve_port()
        if not target:
            return False
        try:
            ser = serial.serial_for_url(target, do_not_open=True, baudrate=BAUD, timeout=READ_TIMEOUT_S)
            try:
                # Prevents the classic pyserial-opens-port-toggles-DTR-resets-ESP32
                # problem on real hardware. No-op (or harmless) on socket:// transport.
                ser.dtr = False
                ser.rts = False
            except Exception:
                pass
            ser.open()
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            self._ser = ser
            self.port = target
            self._last_rx = time.time()
            self._last_ms = None
            self._dirty = True  # resend the full desired state right away
            self._event(f"connected to {target}")
            return True
        except Exception as exc:  # never let a bad URL/permission error crash the thread
            self._event(f"connect failed ({target}): {exc!r}")
            return False

    def _close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.port = None

    def _send(self) -> None:
        with self._lock:
            verdict, mode, hold, elevation = self._cmd
            self._dirty = False
        try:
            line = format_command(verdict, mode, hold, elevation)
            self._ser.write((line + "\n").encode("ascii"))
            self._last_tx = time.time()
        except Exception as exc:
            self._event(f"send failed: {exc!r}")
            self._close()

    def _handle_line(self, line: str) -> None:
        if not line:
            return
        if line.startswith("B,"):
            boot = parse_boot(line)
            self.boots += 1
            self._last_ms = None
            self._dirty = True
            with self._lock:
                self.history.clear()
            self._event(f"ESP32 boot message: {boot}")
            return

        t = parse_telemetry(line)
        if t is None:
            self.bad_lines += 1
            return

        if self._last_ms is not None and t["ms"] < self._last_ms - 5:
            # millis() went backwards without a B, line -> the device rebooted
            # and we simply didn't catch the boot message in time.
            self.boots += 1
            self._dirty = True
            with self._lock:
                self.history.clear()
            self._event("ESP32 reboot inferred (millis reset)")

        self._last_ms = t["ms"]
        t["rx"] = time.time()
        self._last_rx = t["rx"]
        with self._lock:
            self.latest = t
            self.history.append(t)
        if self._csv is not None:
            self._csv.writerow([t["rx"]] + [t[k] for k in TELEMETRY_FIELDS])
            self._log_file.flush()

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._ser is None:
                if not self._connect():
                    time.sleep(RECONNECT_RETRY_S)
                    continue
            try:
                raw = self._ser.readline()
                if raw:
                    self._handle_line(raw.decode("ascii", "ignore").strip())

                now = time.time()
                if self._dirty or (now - self._last_tx) >= HEARTBEAT_INTERVAL_S:
                    self._send()

                if self.latest is not None and (now - self._last_rx) > FORCE_RECONNECT_S:
                    self._event("telemetry stale for too long — reconnecting")
                    self._close()
            except Exception as exc:
                self._event(f"link error: {exc!r} — reconnecting")
                self._close()
                time.sleep(RECONNECT_RETRY_S)


# --------------------------------------------------------------------------
# Manual terminal test: `python link.py`
# Type: OK | SAFE | UNKNOWN | HOLD <deg> | AUTO
# --------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    link = Link().start()

    def repl():
        for raw in sys.stdin:
            parts = raw.split()
            if not parts:
                continue
            cmd = parts[0].upper()
            if cmd in ("OK", "SAFE", "UNKNOWN"):
                link.set_verdict(cmd)
            elif cmd == "AUTO":
                link.set_mode("AUTO")
            elif cmd == "HOLD" and len(parts) > 1:
                link.set_mode("HOLD", float(parts[1]))
            else:
                print("commands: OK | SAFE | UNKNOWN | AUTO | HOLD <deg>")

    threading.Thread(target=repl, daemon=True).start()

    try:
        while True:
            s = link.snapshot()
            t = s["latest"]
            status = "LINK" if s["connected"] else "----"
            if t:
                body = (f"{t['state']:>3} angle={t['angle']:+6.1f} target={t['target']:+6.1f} "
                        f"L={t['L']:5.0f} R={t['R']:5.0f} err={t['err']:+.3f} flags={t['flags']}")
            else:
                body = "no telemetry yet"
            print(f"{status}  {body}  | sent={s['cmd']} bad={s['bad_lines']} boots={s['boots']} port={s['port']}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        link.stop()
