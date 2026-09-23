"""
weather.py — weather fetch, DEMO decision rules, and simulated scenarios.

evaluate_rules() is a pure function: given a weather reading (or None), a
"did the fetch succeed" flag, the current time, and the previous decision
memory, it returns (verdict, reason, new_memory) with NO side effects and NO
internal clock reads. That's what makes it trivially unit-testable (see
tests/test_weather.py) and what makes "the same function handles live and
simulated weather" a literal, checkable fact rather than a slogan.

All thresholds here are DEMO thresholds, picked so a judge can see the state
change happen — NOT structural/engineering wind ratings. See CLAUDE.md.

WeatherService wraps evaluate_rules() with the parts that DO have side
effects: polling Open-Meteo on a timer, tracking simulated-scenario
selection, and pushing the resulting verdict into a Link.
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

import requests

# --------------------------------------------------------------------------
# Data types
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class WeatherData:
    code: int             # WMO weather code
    cloud_pct: float       # cloud cover %, informational only — never a SAFE trigger
    wind_kmh: float        # current sustained wind, informational only
    gust_kmh: float        # current wind gust — a SAFE trigger
    gust_next_kmh: float   # max forecast gust in the next hour — a pre-emptive SAFE trigger
    t: float               # unix timestamp this reading is valid as-of
    source: str             # "live" or "sim:<SCENARIO NAME>"


@dataclass(frozen=True)
class Mem:
    """Decision memory carried between evaluate_rules() calls."""
    latched: bool = False
    clear_since: Optional[float] = None


@dataclass(frozen=True)
class RulesConfig:
    # --- DEMO thresholds: chosen so judges can see the state change, NOT
    #     an engineering wind rating. Real stow thresholds come from a
    #     tracker manufacturer's structural/wind-tunnel qualification. ---
    safe_gust_kmh: float = 40.0
    clear_gust_kmh: float = 30.0       # hysteresis: must drop below this before clearing starts
    dwell_live_s: float = 600.0        # live-weather clearing dwell (10 min)
    dwell_sim_s: float = 8.0           # simulated-scenario clearing dwell (fast, for the demo)
    stale_s: float = 900.0             # data older than this counts as UNKNOWN (15 min)


DEFAULT_CFG = RulesConfig()

THUNDERSTORM_CODES = frozenset({95, 96, 99})  # WMO codes: thunderstorm, +slight/heavy hail

WMO_DESCRIPTIONS = {
    0: "Clear sky", 1: "Mainly clear", 2: "Partly cloudy", 3: "Overcast",
    45: "Fog", 48: "Depositing rime fog",
    51: "Light drizzle", 53: "Drizzle", 55: "Dense drizzle",
    61: "Slight rain", 63: "Rain", 65: "Heavy rain",
    71: "Slight snow", 73: "Snow", 75: "Heavy snow",
    80: "Slight rain showers", 81: "Rain showers", 82: "Violent rain showers",
    95: "Thunderstorm", 96: "Thunderstorm, slight hail", 99: "Thunderstorm, heavy hail",
}


def describe_code(code: int) -> str:
    return WMO_DESCRIPTIONS.get(code, f"code {code}")


# --------------------------------------------------------------------------
# The decision engine — pure, deterministic, unit-testable
# --------------------------------------------------------------------------


def evaluate_rules(wx: Optional[WeatherData], fetch_ok: bool, now: float, mem: Mem,
                    cfg: RulesConfig = DEFAULT_CFG):
    """(weather, memory) -> (verdict, reason, new_memory). Pure. No I/O, no clock reads.

    Safety property (see CLAUDE.md / tests/test_weather.py): missing or stale
    data returns UNKNOWN and NEVER changes mem.latched. Only an explicit SAFE
    or OK reading may set or clear the latch.
    """
    if wx is None or not fetch_ok or (now - wx.t) > cfg.stale_s:
        return "UNKNOWN", "No current weather data", Mem(mem.latched, None)

    danger = []
    if wx.code in THUNDERSTORM_CODES:
        danger.append(f"thunderstorm conditions ({describe_code(wx.code)})")
    if wx.gust_kmh >= cfg.safe_gust_kmh:
        danger.append(f"gust {wx.gust_kmh:.0f} km/h >= {cfg.safe_gust_kmh:.0f} km/h (demo threshold)")
    elif wx.gust_next_kmh >= cfg.safe_gust_kmh:
        danger.append(f"forecast gust {wx.gust_next_kmh:.0f} km/h within 1h (pre-emptive, demo threshold)")

    if danger:
        return "SAFE", "; ".join(danger), Mem(True, None)

    if not mem.latched:
        return "OK", "Normal conditions", mem

    # Currently latched SAFE. Hysteresis: must drop below the (lower) clear
    # threshold before the clearing dwell timer is even allowed to start.
    if max(wx.gust_kmh, wx.gust_next_kmh) >= cfg.clear_gust_kmh:
        return "SAFE", f"Holding SAFE until gust < {cfg.clear_gust_kmh:.0f} km/h (hysteresis)", Mem(True, None)

    dwell = cfg.dwell_sim_s if wx.source.startswith("sim:") else cfg.dwell_live_s
    since = mem.clear_since if mem.clear_since is not None else now
    remaining = dwell - (now - since)
    if remaining > 0:
        return "SAFE", f"Conditions calm - reopening in {remaining:.0f}s", Mem(True, since)
    return "OK", "Calm for the full dwell period - clearing to reopen", Mem(False, None)


# --------------------------------------------------------------------------
# Simulated scenarios — MUST pass through evaluate_rules() exactly like live data
# --------------------------------------------------------------------------

SCENARIOS = {
    "NORMAL":    dict(code=1,  cloud_pct=15,  wind_kmh=10, gust_kmh=18, gust_next_kmh=20),
    "CLOUDY":    dict(code=3,  cloud_pct=95,  wind_kmh=12, gust_kmh=20, gust_next_kmh=22),
    "HIGH WIND": dict(code=2,  cloud_pct=40,  wind_kmh=30, gust_kmh=48, gust_next_kmh=52),
    "STORM":     dict(code=95, cloud_pct=100, wind_kmh=45, gust_kmh=70, gust_next_kmh=75),
}


def make_sim_weather(name: str, now: float) -> WeatherData:
    d = SCENARIOS[name]
    return WeatherData(code=d["code"], cloud_pct=d["cloud_pct"], wind_kmh=d["wind_kmh"],
                        gust_kmh=d["gust_kmh"], gust_next_kmh=d["gust_next_kmh"],
                        t=now, source=f"sim:{name}")


# --------------------------------------------------------------------------
# Live fetch — Open-Meteo (free, no API key)
# --------------------------------------------------------------------------

OPEN_METEO_URL = "https://api.open-meteo.com/v1/forecast"
DEFAULT_LAT = 0.0   # deliberately a placeholder (middle of the ocean) so nobody
DEFAULT_LON = 0.0   # mistakes it for real data — set TRACKER_LAT/TRACKER_LON


def fetch_live(lat: float, lon: float, timeout: float = 5.0) -> WeatherData:
    resp = requests.get(OPEN_METEO_URL, timeout=timeout, params={
        "latitude": lat, "longitude": lon, "timezone": "auto", "forecast_hours": 2,
        "current": "weather_code,cloud_cover,wind_speed_10m,wind_gusts_10m",
        "hourly": "wind_gusts_10m",
    })
    resp.raise_for_status()
    data = resp.json()
    cur = data["current"]
    hourly_gusts = [g for g in data.get("hourly", {}).get("wind_gusts_10m", []) if g is not None]
    gust_next = max(hourly_gusts) if hourly_gusts else float(cur["wind_gusts_10m"])
    return WeatherData(
        code=int(cur["weather_code"]),
        cloud_pct=float(cur["cloud_cover"]),
        wind_kmh=float(cur["wind_speed_10m"]),
        gust_kmh=float(cur["wind_gusts_10m"]),
        gust_next_kmh=float(gust_next),
        t=time.time(),
        source="live",
    )


# --------------------------------------------------------------------------
# WeatherService — the side-effecting wrapper around evaluate_rules()
# --------------------------------------------------------------------------


class WeatherService:
    """Runs two background threads: one polls Open-Meteo, one makes decisions
    every 0.5s and pushes the verdict into `link`. Nothing here blocks the
    Streamlit main thread."""

    def __init__(self, link, lat: Optional[float] = None, lon: Optional[float] = None,
                 cfg: RulesConfig = DEFAULT_CFG, poll_s: float = 300.0, retry_s: float = 30.0):
        self.link = link
        self.lat = lat if lat is not None else float(os.environ.get("TRACKER_LAT", DEFAULT_LAT))
        self.lon = lon if lon is not None else float(os.environ.get("TRACKER_LON", DEFAULT_LON))
        self.cfg = cfg
        self.poll_s, self.retry_s = poll_s, retry_s

        self._lock = threading.Lock()
        self._stop = threading.Event()

        self.scenario = "LIVE"
        self.force_offline = False   # debug toggle: simulate a weather API failure
        self._refresh_now = True

        self.live_wx: Optional[WeatherData] = None
        self.fetch_ok = False
        self.last_fetch_attempt = 0.0
        self.last_fetch_ok_at: Optional[float] = None
        self.last_error = ""

        self.mem = Mem()
        self.seeded = False   # becomes True once we've copied the latch from the ESP32

        self.verdict, self.reason = "UNKNOWN", "Waiting for ESP32 telemetry before evaluating weather"
        self.current_wx: Optional[WeatherData] = None

    # ---- controls (called from app.py) ----

    def set_scenario(self, name: str) -> None:
        with self._lock:
            self.scenario = name

    def request_refresh(self) -> None:
        self._refresh_now = True

    def snapshot(self) -> dict:
        with self._lock:
            return dict(verdict=self.verdict, reason=self.reason, scenario=self.scenario,
                        wx=self.current_wx, fetch_ok=self.fetch_ok,
                        last_fetch_ok_at=self.last_fetch_ok_at, last_error=self.last_error,
                        seeded=self.seeded, force_offline=self.force_offline)

    # ---- lifecycle ----

    def start(self) -> "WeatherService":
        threading.Thread(target=self._fetch_loop, daemon=True, name="weather-fetch").start()
        threading.Thread(target=self._decide_loop, daemon=True, name="weather-decide").start()
        return self

    def stop(self) -> None:
        self._stop.set()

    # ---- worker threads ----

    def _fetch_loop(self) -> None:
        while not self._stop.is_set():
            due = self.retry_s if not self.fetch_ok else self.poll_s
            if self._refresh_now or (time.time() - self.last_fetch_attempt) >= due:
                self._refresh_now = False
                self.last_fetch_attempt = time.time()
                try:
                    if self.force_offline:
                        raise RuntimeError("simulated weather API failure (force_offline)")
                    wx = fetch_live(self.lat, self.lon)
                    with self._lock:
                        self.live_wx, self.fetch_ok = wx, True
                        self.last_fetch_ok_at, self.last_error = wx.t, ""
                except Exception as exc:  # network error, timeout, bad JSON, HTTP error...
                    with self._lock:
                        self.fetch_ok = False
                        self.last_error = str(exc)[:200]
            time.sleep(0.5)

    def _decide_loop(self) -> None:
        while not self._stop.is_set():
            if not self.seeded:
                # Copy the ESP32's CURRENT latch state before sending anything
                # but UNKNOWN. This is what stops a Python restart from ever
                # accidentally releasing a SAFE the hardware is still holding.
                snap = self.link.snapshot()
                t = snap["latest"]
                if t is None or not snap["connected"]:
                    self.link.set_verdict("UNKNOWN")
                    time.sleep(0.3)
                    continue
                self.mem = Mem(latched=(t["state"] == "STW"))
                self.seeded = True

            now = time.time()
            with self._lock:
                scenario = self.scenario
                live_wx, fetch_ok = self.live_wx, self.fetch_ok

            if scenario == "LIVE":
                wx, ok = live_wx, fetch_ok
            else:
                wx, ok = make_sim_weather(scenario, now), True

            verdict, reason, self.mem = evaluate_rules(wx, ok, now, self.mem, self.cfg)
            self.link.set_verdict(verdict)
            with self._lock:
                self.verdict, self.reason, self.current_wx = verdict, reason, wx
            time.sleep(0.5)


# --------------------------------------------------------------------------
# Manual terminal test: `python weather.py`
# Type: LIVE | NORMAL | CLOUDY | HIGH WIND | STORM | OFFLINE (toggles force_offline)
# --------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    if "--selftest" in sys.argv:
        # quick sanity pass without needing a Link/ESP32 at all
        calm = WeatherData(1, 10, 10, 15, 15, 0, "sim:NORMAL")
        storm = WeatherData(95, 100, 45, 70, 70, 1, "sim:STORM")
        v, _, m = evaluate_rules(calm, True, 0, Mem())
        assert v == "OK", v
        v, _, m = evaluate_rules(storm, True, 1, m)
        assert v == "SAFE" and m.latched, (v, m)
        v, _, m = evaluate_rules(None, False, 2, m)
        assert v == "UNKNOWN" and m.latched, (v, m)
        print("weather.py --selftest: OK")
        sys.exit(0)

    from link import Link  # local import: only needed for the manual demo below

    link = Link().start()
    svc = WeatherService(link).start()

    def repl():
        for raw in sys.stdin:
            cmd = raw.strip().upper()
            if cmd == "OFFLINE":
                svc.force_offline = not svc.force_offline
                svc.request_refresh()
                print(f"force_offline = {svc.force_offline}")
            elif cmd == "LIVE" or cmd in SCENARIOS:
                svc.set_scenario(cmd)
                print(f"scenario = {cmd}")
            else:
                print("commands: LIVE | NORMAL | CLOUDY | HIGH WIND | STORM | OFFLINE")

    threading.Thread(target=repl, daemon=True).start()

    try:
        while True:
            ls, ws = link.snapshot(), svc.snapshot()
            t = ls["latest"]
            print(f"[{ws['scenario']:9}] verdict={ws['verdict']:7} esp={t['state'] if t else '---':3} "
                  f"angle={t['angle'] if t else 0:+.1f} | {ws['reason']}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        svc.stop()
        link.stop()
