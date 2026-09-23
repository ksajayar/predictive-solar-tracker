# Solar Tracker — laptop / backend

Weather-aware single-axis solar-tracker prototype. This is the laptop side:
serial link, weather decision logic, and the Streamlit dashboard. It runs
today against a simulated ESP32 (`fake_esp32.py`) so software work doesn't
block on hardware, and switches to the real board later by changing one
environment variable — see [Simulator ↔ real ESP32](#simulator--real-esp32).

**The laptop advises. The ESP32 controls.** See [CLAUDE.md](CLAUDE.md) for
the full architecture, the frozen serial protocol, and the safety invariants.

## Folder structure

```
solar-tracker/
├── laptop/
│   ├── app.py              Streamlit dashboard — display + operator input only
│   ├── link.py              owns the transport; protocol parse/format + Link class
│   ├── weather.py           Open-Meteo fetch, evaluate_rules(), WeatherService
│   ├── fake_esp32.py        simulated ESP32 device (run as its own process)
│   └── requirements.txt
├── tests/
│   ├── test_protocol.py     wire-format parsing/formatting
│   └── test_weather.py      evaluate_rules() safety properties
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

57 tests, covering the wire protocol (valid/invalid/malformed/garbage
packets, round-trips) and the weather decision engine (each scenario's
verdict, the "missing data never clears SAFE" property, hysteresis, the
clearing dwell, and pre-emptive forecast-gust SAFE).

## Simulator ↔ real ESP32

Everything goes through one environment variable. Nothing in `app.py`,
`weather.py`, or the decision logic changes.

| Variable | Default | Meaning |
|---|---|---|
| `TRACKER_TRANSPORT` | `sim` | `sim` connects to `fake_esp32.py`. `serial` auto-detects a real ESP32 by USB VID (CP210x / CH340 / FTDI / Espressif native USB). |
| `TRACKER_PORT` | unset | Overrides discovery entirely. A device path (`/dev/cu.usbserial-XXXX` on macOS, `COM5` on Windows) for real hardware, or a full `socket://host:port` URL. |
| `FAKE_ESP32_HOST` / `FAKE_ESP32_PORT` | `127.0.0.1` / `9091` | Where `link.py` looks for the simulator when `TRACKER_TRANSPORT=sim`. Must match what you passed to `fake_esp32.py`. |
| `TRACKER_LAT` / `TRACKER_LON` | `0.0` / `0.0` (placeholder — the middle of the ocean, on purpose) | Set these to the venue's coordinates before the demo, or LIVE weather will be meaningless. |

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

## What the hardware team must match exactly

The real ESP32 firmware must implement the identical protocol documented at
the top of `laptop/link.py` and in [CLAUDE.md](CLAUDE.md):

- Telemetry line format, field order, and the 4 state names (`TRK`/`STW`/`RPN`/`HLD`)
- The flags bitmask (1/2/4/8)
- The boot line format (`B,tracker,v1,reset=<n>,latch=<0|1>`)
- Accepting `C,<OK|SAFE|UNKNOWN>,<AUTO|HOLD>,<hold_deg>` and treating `UNKNOWN`
  as "leave the SAFE latch untouched"
- ~10 Hz telemetry, and re-sending its own state promptly after a reboot

`fake_esp32.py` (`laptop/fake_esp32.py`) is the executable reference for all
of the above — when in doubt about exact framing or timing, that's the
source of truth alongside `link.py`'s parse/format functions.
