# Solar Tracker

Weather-aware solar-tracker prototype:

```
                    time + location (TRACKER_LAT/TRACKER_LON)
                                 |
                                 v
                     laptop/solar_position.py
                        (calculated elevation)
                                 |
Weather API (Open-Meteo) ---+    |
                             v   v
                     Python edge controller   (laptop/, this repo's "backend")
                                 |
                             USB serial          (frozen protocol — see below)
                                 |
                                 v
                              ESP32-S3
                                 |
                       +---------+---------+
                       |                   |
                    2 LDRs             elevation target
                       |                   |
                       v                   v
                AZIMUTH CONTROL       TOP SERVO
                       |
                       v
                  BASE SERVO
```

The 2 LDRs give exactly **one** optical light-balance measurement, and that
one error drives the **base servo (azimuth)** tracking — real-time, closed
around the LDR reading. The **top servo (elevation)** has no optical
feedback of its own: its target is *calculated* from date, time, and the
configured latitude/longitude (`laptop/solar_position.py`, NOAA solar
position algorithm), sent over the same serial link as an optional field,
and falls back to a fixed set-point if Python hasn't sent one yet. Accurate
terminology: **"two-LDR azimuth feedback with calculated solar-elevation
positioning."** This is **not** independent dual-axis optical sensing —
don't describe it that way, and don't describe the elevation target as
"measured" — it's commanded/open-loop, same as azimuth already is, just
computed instead of light-driven.

Weather/safety verdicts from the Python side always override BOTH tracking
behaviours: a `SAFE` verdict stops azimuth tracking and drives elevation to
stow immediately, regardless of what the LDRs are reporting or what solar
elevation was last calculated.

This repo has two ESP32-side implementations. **Real hardware is the
primary system** — `fake_esp32.py` is development/testing tooling, not a
second production path:
- [`firmware/solar_tracker/solar_tracker.ino`](firmware/solar_tracker/solar_tracker.ino)
  — the real ESP32-S3 C++ firmware, compiled against the `esp32:esp32:esp32s3`
  Arduino core. **DEVELOPMENT/TESTING ONLY note doesn't apply here** — this
  is what the physical tracker runs. See that file's header comment for
  hardware mapping, calibration constants, and known limitations (base servo
  type positional-vs-continuous is unconfirmed against real hardware and is
  a configurable constant, not a guess baked into the logic).
- [`laptop/fake_esp32.py`](laptop/fake_esp32.py) — **DEVELOPMENT / TESTING
  ONLY.** A Python simulator, run as its own process, standing in for the
  ESP32 so `app.py`/`link.py` can be developed and the protocol can be
  exercised without hardware attached. Nothing in `tests/` currently imports
  it directly (only `link.py`'s parse/format functions are unit-tested), but
  it stays: it's the executable reference the firmware was ported from, the
  fastest way to develop the dashboard without hardware, and the "Run it
  (simulator)" instructions below still use it.

Both talk the identical frozen wire protocol, so `laptop/link.py` and the
Streamlit dashboard work unmodified against either one — switching between
them is one environment variable (`TRACKER_TRANSPORT`, see below), never a
silent runtime fallback. If the real ESP32 disconnects, the dashboard shows
"ESP32 DISCONNECTED"; it never substitutes simulator data to paper over a
lost hardware link.

**The laptop advises. The ESP32 controls.** See [CLAUDE.md](CLAUDE.md) for
the full architecture, the frozen serial protocol, and the safety invariants.

## Folder structure

```
solar-tracker/
├── laptop/
│   ├── app.py              Streamlit dashboard — display + operator input only
│   ├── link.py              owns the transport; protocol parse/format + Link class
│   ├── weather.py           Open-Meteo fetch, evaluate_rules(), WeatherService
│   ├── solar_position.py    calculated solar elevation (pure, unit-tested)
│   ├── fake_esp32.py        simulated ESP32 device — DEVELOPMENT/TESTING ONLY
│   └── requirements.txt
├── firmware/
│   └── solar_tracker/
│       └── solar_tracker.ino   real ESP32-S3 firmware (2 LDRs, base+elevation servos)
├── tests/
│   ├── test_protocol.py       wire-format parsing/formatting
│   ├── test_solar_position.py solar elevation calculation (deterministic)
│   └── test_weather.py        evaluate_rules() safety properties
├── logs/                    telemetry CSVs + fake_esp32's persisted latch (gitignored)
├── CLAUDE.md                 architecture notes for Claude Code sessions in this repo
└── README.md
```

## Setup

Requires Python 3.10+ (a system Python 3.9 will NOT work — `pip install`
streamlit needs 3.10+). On macOS with Homebrew Python already installed:

```bash
cd solar-tracker
python3.12 -m venv .venv        # or python3.11 / python3.10, whichever you have
source .venv/bin/activate       # Windows: .venv\Scripts\activate
pip install -r laptop/requirements.txt
```

## Run it (simulator — no hardware needed)

Two terminals, both with the venv activated:

```bash
# terminal 1 — the simulated ESP32
cd laptop
python fake_esp32.py
```

```bash
# terminal 2 — the dashboard
cd laptop
streamlit run app.py
```

Open the URL Streamlit prints (usually http://localhost:8501). You should
see live LDR values changing, Light Balance rising and falling as the
simulated sun sweeps back and forth, and the panel angle tracking it. Click
through the weather scenario buttons (NORMAL / CLOUDY / HIGH WIND / STORM) to
see the SAFE-mode / stow / reopening behavior.

`fake_esp32.py` listens on `127.0.0.1:9091` by default. Change it with
`FAKE_ESP32_PORT=9500 python fake_esp32.py` (and set the same value on the
dashboard side — see env vars below — if you do).

### Try the resilience behavior

- **Kill `fake_esp32.py` (Ctrl-C) while Streamlit is running.** The banner
  goes to "ESP32 DISCONNECTED". Nothing crashes.
- **Restart `fake_esp32.py`.** The dashboard reconnects automatically within
  a couple of seconds — no action needed on the Streamlit side. If it was in
  SAFE MODE before the kill, it's still in SAFE MODE after the restart (the
  latch is persisted to `logs/fake_esp32_state.json`, standing in for the
  ESP32's real NVS flash).
- **Ctrl-C the Streamlit process, then `streamlit run app.py` again**, while
  `fake_esp32.py` keeps running the whole time. This is the "Python crashed"
  scenario: the simulated tracker never stopped, and the new dashboard
  process reconnects and picks up exactly where the device is, without ever
  sending a premature `OK` that could release an existing SAFE latch (see
  the seeding note in [CLAUDE.md](CLAUDE.md)).
- **Toggle "Simulate weather API failure"** in the sidebar, then select
  `LIVE`. The weather card shows "WEATHER DATA UNAVAILABLE" and — if a SAFE
  latch is currently held — the banner explicitly says the latch holds
  regardless.

## Run the tests

```bash
cd solar-tracker            # repo root, not laptop/
source .venv/bin/activate
python -m pytest tests/ -v
```

69 tests, covering the wire protocol (valid/invalid/malformed/garbage
packets, round-trips, old/new command backward compatibility), the
calculated solar elevation (sunrise/midday/sunset/night, date/latitude
sensitivity, determinism, tz-aware input), and the weather decision engine
(each scenario's verdict, the "missing data never clears SAFE" property,
hysteresis, the clearing dwell, and pre-emptive forecast-gust SAFE).

## Simulator ↔ real ESP32

Everything goes through one environment variable. Nothing in `app.py`,
`weather.py`, or the decision logic changes.

| Variable | Default | Meaning |
|---|---|---|
| `TRACKER_TRANSPORT` | `sim` | `sim` connects to `fake_esp32.py`. `serial` auto-detects a real ESP32 by USB VID (CP210x / CH340 / FTDI / Espressif native USB). |
| `TRACKER_PORT` | unset | Overrides discovery entirely. A device path (`/dev/cu.usbserial-XXXX` on macOS, `COM5` on Windows) for real hardware, or a full `socket://host:port` URL. |
| `FAKE_ESP32_HOST` / `FAKE_ESP32_PORT` | `127.0.0.1` / `9091` | Where `link.py` looks for the simulator when `TRACKER_TRANSPORT=sim`. Must match what you passed to `fake_esp32.py`. |
| `TRACKER_LAT` / `TRACKER_LON` | `0.0` / `0.0` (placeholder — the middle of the ocean, on purpose) | Set these to the venue's coordinates before the demo. Used for BOTH the Open-Meteo weather fetch AND the calculated solar elevation (`laptop/solar_position.py`) — one place configures both, deliberately. Left unset, LIVE weather is meaningless AND the elevation target will be wrong for your location. |

Hardware team, on demo day:

```bash
export TRACKER_TRANSPORT=serial
# usually not needed (auto-detected by VID), but if two boards are plugged in:
export TRACKER_PORT=/dev/cu.usbserial-0001
streamlit run app.py
```

Stop running `fake_esp32.py` at that point — with `TRACKER_TRANSPORT=serial`
it's simply not used.

## Common failures

| Symptom | Likely cause / fix |
|---|---|
| Dashboard stuck on "Waiting for ESP32 telemetry..." | Is `fake_esp32.py` running? Check `TRACKER_TRANSPORT`/`FAKE_ESP32_PORT` match on both sides. |
| `fake_esp32.py` prints "FAILED to bind" | Another instance is already running on that port — kill it, or set `FAKE_ESP32_PORT` to something else. |
| Real ESP32 not found with `TRACKER_TRANSPORT=serial` | Set `TRACKER_PORT` explicitly. Check the USB cable is a data cable, not charge-only. On macOS use the `/dev/cu.*` path, not `/dev/tty.*`. |
| Weather card always shows the ocean/placeholder-looking numbers | Set `TRACKER_LAT` / `TRACKER_LON` to your venue. |
| "WEATHER DATA UNAVAILABLE" even though you expect LIVE to work | Check `laptop/logs` — actually check the Diagnostics expander in the UI for the fetch error text. Likely no internet, or the "Simulate weather API failure" toggle is still on. |
| Streamlit shows stale numbers after `git pull` / editing `link.py` or `weather.py` | Fully restart `streamlit run app.py` (Ctrl-C, rerun). `@st.cache_resource` keeps the background threads alive across a hot-reload from `runOnSave`, which can pin you to stale module code — a full process restart is the reliable fix. |
| `pip install` fails on `streamlit` | You're on system Python 3.9. Use Python 3.10+ (see Setup). |

## What the real ESP32 firmware implements

`firmware/solar_tracker/solar_tracker.ino` implements the identical protocol
documented at the top of `laptop/link.py` and in [CLAUDE.md](CLAUDE.md):

- Telemetry line format, field order, and the 4 state names (`TRK`/`STW`/`RPN`/`HLD`)
- The flags bitmask (1/2/4/8)
- The boot line format (`B,tracker,v1,reset=<n>,latch=<0|1>`)
- Accepting `C,<OK|SAFE|UNKNOWN>,<AUTO|HOLD>,<hold_deg>[,<elevation_target>]`
  and treating `UNKNOWN` as "leave the SAFE latch untouched"
- ~10 Hz telemetry, and re-sending its own state promptly after a reboot

The command's 5th field (calculated solar elevation, degrees) is **optional
and fully backward compatible**:

```
OLD (still valid):  C,OK,AUTO,0.0
NEW:                 C,OK,AUTO,0.0,57.4
```

A 4-field command means "no calculated elevation available this tick" — the
firmware falls back to its fixed `TRACK_ELEVATION_DEG` set-point, exactly as
if this feature didn't exist. A 5-field command carries the elevation target
computed by `laptop/solar_position.py`. Either way, `SAFE` still overrides
elevation to the stow position, unconditionally — a commanded elevation can
never move the top servo away from stow while the safety latch is held.

The telemetry `angle`/`target` fields carry the base/azimuth axis only — the
one axis the 2 LDRs actually drive, and the only one with any kind of
position estimate at all. The elevation servo has no wire representation in
telemetry (deliberately, to avoid changing the frozen format) and no
feedback sensor — its target is commanded open-loop, whether it came from
the fixed fallback or from `solar_position.py`.

`laptop/fake_esp32.py` is the executable reference this firmware was ported
from (including the elevation authority logic), and remains
**development/testing tooling** — it's what the "Run it (simulator)"
instructions above use, and it's still the fastest way to iterate on
`app.py`/`weather.py` without hardware. It is NOT part of the hardware demo
path. When in doubt about exact framing or timing on either side, `link.py`'s
parse/format functions are the single source of truth.

Compiled and verified against `esp32:esp32:esp32s3` (Arduino ESP32 core
3.3.12, ESP32Servo 3.2.1) — see the header comment in `solar_tracker.ino` for
the full pin/calibration map and known limitations before flashing real
hardware.

## Data provenance

For the hardware demo, know where every number on the dashboard actually
comes from:

| Data | Source | Notes |
|---|---|---|
| Left/right LDR readings, azimuth tracking error | **REAL / MEASURED** | `analogReadMilliVolts()` on the physical ESP32-S3, when running real firmware |
| ESP32 tracker state, azimuth angle/target | **REAL / MEASURED** (state) or **commanded/open-loop** (angle) | The state machine is real; the angle itself has no position feedback sensor — see `solar_tracker.ino`'s known limitations |
| Elevation target | **CALCULATED** | `laptop/solar_position.py`, from date/time/latitude/longitude — never optically sensed |
| Weather (wind, gusts, conditions) | **EXTERNAL** | Open-Meteo, or an operator-selected simulated scenario (clearly labeled "SIMULATED" in the UI when active) |
| Everything from `fake_esp32.py` | **SIMULATED** | Development/testing only — never used for the real demo (see above) |
