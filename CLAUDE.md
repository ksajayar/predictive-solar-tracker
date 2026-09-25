# CLAUDE.md — project memory for this repo

Weather-aware solar-tracker hackathon prototype: 2 LDRs give exactly one
optical error dimension (azimuth only), driving a base servo; a second, top
servo handles elevation, driven by a CALCULATED solar elevation
(`laptop/solar_position.py`, date+time+location) when Python has sent one,
else a fixed configurable set-point. Neither axis has any position feedback
sensor — both are commanded/open-loop. This file is context for any Claude
Code session working in this repo. See README.md for setup/run
instructions.

**Real hardware is the primary system.** `laptop/fake_esp32.py` is
development/testing tooling, not a second production path — see "Where
things live" below.

## Architecture (frozen — do not redesign without a genuine blocking flaw)

```
Weather API (Open-Meteo) ──┐
                            ▼
Simulated scenarios ──► evaluate_rules() ──► OK | SAFE | UNKNOWN ──┐
                                                                    ▼
                                                                 link.py
                                                          (owns the transport)
                                                                    │
                                                     USB serial / socket (sim)
                                                                    ▼
                                                                  ESP32
                                        (2 LDRs, 2 servos: base azimuth + top elevation)
```

**The laptop advises. The ESP32 controls.** The ESP32 (real or `fake_esp32.py`)
owns all real-time behavior: reading the LDRs, the tracking control loop,
servo movement/limits, the state machine, and the SAFE latch. It keeps
running if the laptop disappears. The laptop only ever sends a high-level
verdict (`OK`/`SAFE`/`UNKNOWN`) and mode (`AUTO`/`HOLD`) — never raw PWM,
never per-tick servo commands.

## Hardware constraints (do not change)

- Exactly 2 LDRs, giving exactly one optical error dimension (azimuth only —
  no independent elevation sensing, ever, regardless of the elevation
  feature below). Exactly 2 servos: base (azimuth, LDR-driven) + top
  (elevation, driven by calculated solar position with a fixed-set-point
  fallback — never LDR-driven). No 4-LDR sensor head, no third servo, no
  stepper motor.
- No real photovoltaic panel. No INA219, no voltage/current/power
  measurement, no MPPT, no energy/efficiency claims anywhere in the UI.
- No ESP32 Wi-Fi — USB serial only (or the socket-based simulator transport
  that stands in for it during development).
- No database, no Flask/FastAPI, no Docker, no MQTT, no cloud backend, no ML.

## The frozen serial protocol

See the docstring at the top of `laptop/link.py` — that is the single source
of truth (parsing/formatting functions live there, imported by both `link.py`
itself and `fake_esp32.py`, so the two ends of the wire cannot drift apart).

```
ESP32 -> laptop, ~10 Hz:  T,<ms>,<state>,<angle>,<target>,<L>,<R>,<err>,<flags>
                          state: TRK|STW|RPN|HLD   flags: 1=link 2=dark 4=limit 8=moving
                          boot:  B,tracker,v1,reset=<n>,latch=<0|1>
laptop -> ESP32, 1 Hz:    C,<OK|SAFE|UNKNOWN>,<AUTO|HOLD>,<hold_deg>[,<elevation_target>]
```

The 5th command field (calculated solar elevation, degrees) is OPTIONAL —
omitting it is a fully valid 4-field command, byte-for-byte what this
protocol looked like before the field existed. A receiver that gets a
4-field command (old sender, or a new sender with no elevation to report
yet) falls back to its own fixed elevation set-point; this is re-evaluated
on every command, not sticky (see `solar_tracker.ino`'s `processCommandLine`
and `fake_esp32.py`'s `SimTracker.apply_command`).

Treat this as frozen. If you must change it, change it in exactly one place
(`link.py`'s parse/format functions), then update `fake_esp32.py`,
`tests/test_protocol.py`, and `firmware/solar_tracker/solar_tracker.ino`
together. The real firmware now lives in this repo, ported field-for-field
from `fake_esp32.py`'s tracking state machine. Its `angle`/`target` telemetry
fields carry the base/azimuth axis only — elevation has no wire
representation, by design (see that file's header comment); the Python side
knows what elevation it last commanded (it just sent it), it just isn't told
back what the ESP32 did with it.

## The one safety property everything else depends on

**Missing or stale weather data must never clear an existing SAFE latch.**
`evaluate_rules()` in `laptop/weather.py` returns `UNKNOWN` (never `OK`) when
data is missing, a fetch failed, or the reading is stale — and `UNKNOWN`
never changes `mem.latched`. Only an explicit `SAFE` sets the latch and only
an explicit `OK` clears it. The same rule is mirrored on the device side (see
`SimTracker.apply_command` in `fake_esp32.py`): `UNKNOWN` leaves the latch
untouched.

This is also why `Link` defaults its outgoing command to `UNKNOWN` (never
`OK`) until `WeatherService` has explicitly decided something, and why
`WeatherService._decide_loop` seeds its internal `Mem` from the ESP32's
*actual current telemetry state* (`latched = (state == "STW")`) before it
ever sends a real verdict. That seeding step is what makes a Python/Streamlit
restart safe: it can never accidentally send `OK` to a device it hasn't
even confirmed is still in SAFE.

## Demo thresholds vs. real thresholds

Everything in `weather.py`'s `RulesConfig` (gust ≥ 40 km/h, etc.) is a DEMO
threshold picked so a judge can see the state change happen within a few
seconds. It is explicitly NOT a structural/engineering wind rating — those
come from a real tracker manufacturer's wind-tunnel qualification. Don't
"improve" these numbers to look more official; don't remove the "(demo
threshold)" label from reasons shown in the UI.

## Transport: simulator now, real hardware later, same code

`TRACKER_TRANSPORT=sim` (default) connects `link.py` to `fake_esp32.py` over
`socket://127.0.0.1:9091`. `TRACKER_TRANSPORT=serial` auto-detects a real
ESP32 by USB VID (or use `TRACKER_PORT` explicitly). Both paths go through
pyserial's `serial_for_url()`, so `link.py`, `weather.py`, and `app.py` do
not change at all when the hardware team connects the real board — only the
environment variable changes. Do not special-case the simulator anywhere
outside of `link.py`'s `_resolve_port()`.

## Where things live

- `laptop/link.py` — the ONLY module that touches the transport. Protocol
  parse/format functions + the `Link` class (background thread, reconnect,
  CSV logging, thread-safe snapshots).
- `laptop/solar_position.py` — `calculate_solar_elevation(lat, lon, dt)`,
  pure and unit-tested (NOAA solar position algorithm, stdlib only, no
  dependency added). The ONLY place solar geometry is computed. Never
  touches weather, never touches azimuth.
- `laptop/fake_esp32.py` — **DEVELOPMENT/TESTING ONLY**, not part of the real
  demo path (real hardware is primary — see intro). Simulated device, run as
  a separate process. Talks the real protocol over a TCP socket so it
  exercises `link.py` unmodified. Persists the SAFE latch to
  `logs/fake_esp32_state.json` to simulate NVS surviving a reboot. Also
  simulates the elevation authority logic (STOW overrides, fallback) so
  protocol-level tests can exercise it without hardware — this state is
  internal only, never telemetered (matches the real firmware).
- `laptop/weather.py` — `evaluate_rules()` (pure, unit-tested), Open-Meteo
  fetch, simulated scenarios, `WeatherService` (background threads). Reads
  `TRACKER_LAT`/`TRACKER_LON` — the SAME two env vars `app.py` reuses for
  solar elevation; location is configured in exactly one place for both.
- `laptop/app.py` — Streamlit UI. Display + operator input only; never
  touches the transport or runs decision logic itself. Also starts one small
  background thread (`_solar_elevation_loop`) that recomputes elevation
  every 30s and pushes it via `Link.set_elevation()` — the only place that
  wiring lives.
- `firmware/solar_tracker/solar_tracker.ino` — the real ESP32-S3 C++
  firmware. Ported from `fake_esp32.py`'s tracking state machine; compiled
  against `esp32:esp32:esp32s3` (Arduino ESP32 core). Base servo type
  (continuous-rotation vs. positional) is unconfirmed against real hardware —
  see the file's header comment and its `BASE_SERVO_MODE` constant.
  `solarElevationToServoTarget()` is the ONLY place solar-elevation
  calibration (offset/inversion/clamp) happens.
- `tests/` — `test_protocol.py` (wire format, old/new command compat),
  `test_solar_position.py` (deterministic, explicit datetimes, no clock
  reads), `test_weather.py` (`evaluate_rules()` safety properties). Run with
  `pytest tests/`.

## Style / process notes

- Keep `evaluate_rules()` pure: no `time.time()` inside it, no I/O, `now` is
  always a parameter. That's what makes it unit-testable without mocking a
  clock.
- Prefer editing an existing `laptop/` file over adding a new one. This is a
  laptop-local hackathon app — resist the urge to add a database, a web
  framework, or a message broker "for later." `solar_position.py` earned a
  new file because it's a genuinely separate, independently-testable concern
  (pure math, no I/O) — that's the bar for adding another one, not "it felt
  cleaner here."
