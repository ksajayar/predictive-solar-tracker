/*
  solar_tracker.ino — real ESP32-S3 firmware for the weather-aware solar tracker.

  Implements the ESP32 side of the FROZEN protocol documented in
  ../../laptop/link.py and ../../laptop/CLAUDE.md. laptop/fake_esp32.py is the
  executable reference this firmware was ported from — the tracking state
  machine (TRK/STW/RPN/HLD), the hysteresis+debounce LDR control loop, and the
  telemetry/command framing all mirror it field-for-field so link.py needs
  zero changes to talk to real hardware.

  Hardware (see "FINAL HARDWARE MAPPING" below and the CALIBRATE comments):
    - Exactly 2 LDRs (left/right), read as millivolts via the ESP32's
      self-calibrated ADC.
    - 2 servos: BASE (azimuth, physically rotates the whole tracker) and
      ELEVATION (tilt).

  IMPORTANT — physical limitation of 2 LDRs (do not "fix" this, it's real):
    L and R give exactly ONE scalar light-balance error. That error drives
    azimuth (the base) only. Elevation has no optical feedback at all in this
    hardware; it is held at a fixed configurable TRACK_ELEVATION_DEG. See
    "KNOWN LIMITATIONS" at the bottom of this file.

  IMPORTANT — base servo type is UNKNOWN from this repository:
    Nothing in solar-tracker documents whether the "360° servo" driving the
    base is a true positional actuator or a continuous-rotation servo (which
    has no absolute position at all — it only takes a speed/direction
    command). BASE_SERVO_MODE below defaults to BASE_CONTINUOUS because
    that's what "360° servo" conventionally means in hobbyist hardware, but
    this is an assumption, not a fact proven by this repo. Flip it to
    BASE_POSITIONAL if the real part turns out to be a positional actuator.
    Both code paths are compiled and maintained, not stubbed.
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// ============================================================================
// CALIBRATION CONSTANTS — every hardware-specific value lives here.
// Anything marked "CALIBRATE ON REAL HARDWARE" is a placeholder / best-guess
// starting point, not a measured value. Verify all of them against the real
// tracker before trusting the demo.
// ============================================================================

// ---- Pins -------------------------------------------------------------
// ESP32-S3 ADC1 channels (safe with no Wi-Fi in use). CALIBRATE ON REAL
// HARDWARE — confirm against the exact dev-board silkscreen; avoid strapping
// pins (0, 3, 45, 46), the native-USB pins (19, 20), and any pins your board
// wires to PSRAM/octal flash.
const int LDR_LEFT_PIN = 1;   // ADC1_CH0
const int LDR_RIGHT_PIN = 2;  // ADC1_CH1

const int BASE_SERVO_PIN = 4;       // CALIBRATE ON REAL HARDWARE
const int ELEVATION_SERVO_PIN = 5;  // CALIBRATE ON REAL HARDWARE

// ---- Base servo type (see file header) --------------------------------
enum BaseServoMode { BASE_CONTINUOUS, BASE_POSITIONAL };
// CALIBRATE ON REAL HARDWARE: UNKNOWN which this really is — see header.
const BaseServoMode BASE_SERVO_MODE = BASE_CONTINUOUS;

// ---- Base servo — continuous-rotation tuning (used if BASE_CONTINUOUS) --
// Continuous-rotation servos are commanded as a pulse width around a
// "neutral" (stop) point; there is no absolute-angle command for them.
const int BASE_SERVO_NEUTRAL_US = 1500;   // CALIBRATE — stop pulse, varies per servo unit
const int BASE_SERVO_MIN_SPEED_US = 60;   // CALIBRATE — smallest offset that reliably moves it
const int BASE_SERVO_MAX_SPEED_US = 400;  // CALIBRATE — offset at full commanded speed

// ---- Base servo — positional tuning (used if BASE_POSITIONAL) ---------
const int BASE_SERVO_MIN_US = 500;
const int BASE_SERVO_MAX_US = 2500;

// ---- Base axis software range ------------------------------------------
// In BASE_POSITIONAL mode this is a real, enforced mechanical limit.
// In BASE_CONTINUOUS mode this bounds a DEAD-RECKONED estimate only (there is
// no feedback sensor) — see updateBaseServo() and KNOWN LIMITATIONS.
// Carried over from the single-axis simulator's default; almost certainly
// needs re-tuning for the new base/azimuth role. CALIBRATE ON REAL HARDWARE.
const float BASE_ANGLE_MIN_DEG = -55.0f;
const float BASE_ANGLE_MAX_DEG = 55.0f;

// ---- Elevation servo tuning ---------------------------------------------
const int ELEVATION_SERVO_MIN_US = 500;   // CALIBRATE ON REAL HARDWARE
const int ELEVATION_SERVO_MAX_US = 2500;  // CALIBRATE ON REAL HARDWARE
const float ELEVATION_MIN_DEG = 0.0f;      // CALIBRATE — flattest physical position
const float ELEVATION_MAX_DEG = 70.0f;     // CALIBRATE — steepest physical position
const float TRACK_ELEVATION_DEG = 45.0f;   // CALIBRATE — fixed elevation used whenever not stowed
const float STOW_ELEVATION_DEG = 0.0f;     // CALIBRATE — flat, low-wind-exposure stow position
const float RATE_ELEVATION_DEG_S = 20.0f;  // CALIBRATE — max elevation slew rate; avoids mechanical shock

// ---- LDR tracking control (ported from laptop/fake_esp32.py SimTracker) --
// Starting values are exactly what fake_esp32.py already uses. Treat them as
// tunable prototype parameters, not final constants.
const float E_START = 0.06f;     // CALIBRATE — |err| above this starts a correction
const float E_STOP = 0.02f;      // CALIBRATE — |err| below this stops a correction (hysteresis)
const int START_TICKS = 3;       // CALIBRATE — consecutive over-threshold samples before moving
const float KP = 25.0f;          // CALIBRATE — proportional gain (deg-equivalent per unit error)
const float STEP_MIN_DEG = 0.2f; // CALIBRATE
const float STEP_MAX_DEG = 2.0f; // CALIBRATE
const float RATE_TRACK_DEG_S = 20.0f;   // CALIBRATE — base slew rate while actively tracking
const float RATE_HOLD_DEG_S = 20.0f;    // CALIBRATE — base slew rate while holding (positional mode only)
const float RATE_REOPEN_DEG_S = 5.0f;   // CALIBRATE — base slew rate while leaving stow
// NOTE: there is deliberately no "RATE_STOW" for the base axis. A
// continuous-rotation servo cannot be driven to a known absolute azimuth
// without position feedback, so STOW freezes the base in place instead of
// steering it anywhere — see updateStateMachine() and KNOWN LIMITATIONS.

const float DARK_THRESHOLD_MV = 300.0f;  // CALIBRATE — from fake_esp32.py's DARK_MV

// ---- Timing --------------------------------------------------------------
const unsigned long SENSOR_INTERVAL_MS = 100;     // ~10 Hz, matches fake_esp32.py's TICK_S
const unsigned long TELEMETRY_INTERVAL_MS = 100;  // ~10 Hz, matches the frozen protocol
const unsigned long LINK_TIMEOUT_MS = 5000;       // matches fake_esp32.py's LINK_TIMEOUT_S
const unsigned long REOPEN_MAX_MS = 20000;        // matches fake_esp32.py's REOPEN_MAX_S
const unsigned long SETTLED_TICKS_REQUIRED = 10;  // matches fake_esp32.py

const unsigned long SERIAL_BAUD = 115200;  // matches laptop/link.py's BAUD

// ---- Frozen wire-protocol flag bits (laptop/link.py) ----------------------
const uint8_t FLAG_LINK = 1;
const uint8_t FLAG_DARK = 2;
const uint8_t FLAG_LIMIT = 4;
const uint8_t FLAG_MOVING = 8;

// ============================================================================
// State
// ============================================================================

Preferences prefs;
Servo baseServo;
Servo elevServo;

enum Mode { MODE_AUTO, MODE_HOLD };
enum TrackerState { ST_TRK, ST_STW, ST_RPN, ST_HLD };

bool safetyLatched = false;
Mode mode = MODE_AUTO;
TrackerState state = ST_TRK;
float holdDegRequested = 0.0f;

// Base (azimuth) axis. In BASE_CONTINUOUS mode this is a dead-reckoned
// estimate, not a measured position — see KNOWN LIMITATIONS.
float baseAngle = 0.0f;
float baseTarget = 0.0f;
bool moving = false;
int overCount = 0;
unsigned long settledTicks = 0;
unsigned long rpnStartMs = 0;
float trackStepDeg = 0.0f;

// Elevation axis (no optical feedback; fixed set-point strategy)
float elevAngle = STOW_ELEVATION_DEG;

// Sensors
float L_mV = 0.0f, R_mV = 0.0f;
float err = 0.0f;
bool dark = false;

// Link watchdog
unsigned long lastCommandMs = 0;
bool currentLinkUp = false;

// Boot bookkeeping
unsigned long resetCount = 0;

// ============================================================================
// Small helpers
// ============================================================================

int angleToMicroseconds(float angleDeg, float angleMin, float angleMax, int usMin, int usMax) {
  float t = (angleDeg - angleMin) / (angleMax - angleMin);
  t = constrain(t, 0.0f, 1.0f);
  return usMin + (int)(t * (usMax - usMin));
}

void stopBaseServo() {
  if (BASE_SERVO_MODE == BASE_CONTINUOUS) {
    baseServo.writeMicroseconds(BASE_SERVO_NEUTRAL_US);
  }
  // In BASE_POSITIONAL mode "stop" just means "don't change the target",
  // which updateStateMachine() already arranges for STW/HOLD-freeze cases.
}

// ============================================================================
// Setup
// ============================================================================

void setupBaseServo() {
  ESP32PWM::allocateTimer(0);
  baseServo.setPeriodHertz(50);
  if (BASE_SERVO_MODE == BASE_CONTINUOUS) {
    baseServo.attach(BASE_SERVO_PIN, 500, 2500);
    baseServo.writeMicroseconds(BASE_SERVO_NEUTRAL_US);
  } else {
    baseServo.attach(BASE_SERVO_PIN, BASE_SERVO_MIN_US, BASE_SERVO_MAX_US);
    baseServo.writeMicroseconds(
        angleToMicroseconds(baseAngle, BASE_ANGLE_MIN_DEG, BASE_ANGLE_MAX_DEG,
                             BASE_SERVO_MIN_US, BASE_SERVO_MAX_US));
  }
}

void setupElevationServo() {
  ESP32PWM::allocateTimer(1);
  elevServo.setPeriodHertz(50);
  elevServo.attach(ELEVATION_SERVO_PIN, ELEVATION_SERVO_MIN_US, ELEVATION_SERVO_MAX_US);
  elevServo.writeMicroseconds(
      angleToMicroseconds(elevAngle, ELEVATION_MIN_DEG, ELEVATION_MAX_DEG,
                           ELEVATION_SERVO_MIN_US, ELEVATION_SERVO_MAX_US));
}

void sendBootMessage() {
  Serial.printf("B,tracker,v1,reset=%lu,latch=%d\n", resetCount, safetyLatched ? 1 : 0);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);  // one-time USB-CDC/servo settle time, NOT part of the control loop

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  prefs.begin("tracker", false);
  safetyLatched = prefs.getBool("latch", false);
  resetCount = prefs.getULong("resetCount", 0) + 1;
  prefs.putULong("resetCount", resetCount);

  state = safetyLatched ? ST_STW : ST_TRK;
  baseAngle = 0.0f;
  baseTarget = baseAngle;
  elevAngle = safetyLatched ? STOW_ELEVATION_DEG : TRACK_ELEVATION_DEG;

  setupBaseServo();
  setupElevationServo();

  // Start with the link watchdog already "timed out" so we never report
  // FLAG_LINK before a laptop has actually sent a command. millis() is
  // zero-based at boot, so naively seeding lastCommandMs=0 would make
  // (millis() - lastCommandMs) look small — and therefore "linked" — for the
  // first LINK_TIMEOUT_MS after power-on. Seed it the other way instead.
  lastCommandMs = millis() - (LINK_TIMEOUT_MS + 1);
  rpnStartMs = millis();

  sendBootMessage();
}

// ============================================================================
// Sensors + tracking decision (ported from fake_esp32.py SimTracker)
// ============================================================================

void readLDRs() {
  L_mV = (float)analogReadMilliVolts(LDR_LEFT_PIN);
  R_mV = (float)analogReadMilliVolts(LDR_RIGHT_PIN);
}

void calculateTrackingError() {
  float total = L_mV + R_mV;
  dark = total < DARK_THRESHOLD_MV;
  err = dark ? 0.0f : (L_mV - R_mV) / total;
  // err > 0 => more light on LEFT  => positive base movement ("toward left")
  // err < 0 => more light on RIGHT => negative base movement ("toward right")
}

void updateTrackingDecision() {
  if (dark) {
    moving = false;
    overCount = 0;
    trackStepDeg = 0.0f;
    return;
  }
  if (moving) {
    if (fabsf(err) < E_STOP) moving = false;
  } else {
    overCount = (fabsf(err) > E_START) ? overCount + 1 : 0;
    if (overCount >= START_TICKS) {
      moving = true;
      overCount = 0;
    }
  }
  if (!moving) {
    trackStepDeg = 0.0f;
    return;
  }
  float step = constrain(KP * fabsf(err), STEP_MIN_DEG, STEP_MAX_DEG);
  trackStepDeg = (err > 0) ? step : -step;
}

// ============================================================================
// State machine — the ONLY place state/target/angle change. Mirrors
// SimTracker.tick() in fake_esp32.py, split into base (LDR-driven) and
// elevation (fixed set-point) halves.
// ============================================================================

bool reopenDone(unsigned long now) {
  if (now - rpnStartMs > REOPEN_MAX_MS) return true;
  if (mode == MODE_HOLD && BASE_SERVO_MODE == BASE_POSITIONAL) {
    // Only meaningful with a real positional base — a continuous-rotation
    // base has nothing to converge toward (see KNOWN LIMITATIONS).
    return fabsf(baseAngle - holdDegRequested) < 1.0f;
  }
  return dark || settledTicks >= SETTLED_TICKS_REQUIRED;
}

void updateStateMachine(unsigned long now) {
  // 1) state selection — safety latch always wins, checked first.
  if (safetyLatched) {
    state = ST_STW;
  } else if (state == ST_STW) {
    state = ST_RPN;
    rpnStartMs = now;
    settledTicks = 0;
  } else if (state == ST_RPN) {
    if (reopenDone(now)) {
      state = (mode == MODE_HOLD) ? ST_HLD : ST_TRK;
    }
  } else {
    state = (mode == MODE_HOLD) ? ST_HLD : ST_TRK;
  }

  // 2) base target + rate
  float baseRate;
  switch (state) {
    case ST_STW:
      // Freeze in place. Do NOT drive a continuous-rotation base toward a
      // fixed "stow azimuth" — it has no absolute position to aim for.
      baseTarget = baseAngle;
      baseRate = 0.0f;
      break;
    case ST_HLD:
      if (BASE_SERVO_MODE == BASE_POSITIONAL) {
        baseTarget = holdDegRequested;
        baseRate = RATE_HOLD_DEG_S;
      } else {
        // Continuous-rotation base has no absolute position feedback and
        // cannot honor an absolute hold_deg — freeze instead.
        baseTarget = baseAngle;
        baseRate = 0.0f;
      }
      break;
    case ST_RPN:
      if (mode == MODE_HOLD && BASE_SERVO_MODE == BASE_POSITIONAL) {
        baseTarget = holdDegRequested;
      } else {
        baseTarget = baseAngle + trackStepDeg;
      }
      baseRate = RATE_REOPEN_DEG_S;
      break;
    default:  // ST_TRK
      baseTarget = baseAngle + trackStepDeg;
      baseRate = RATE_TRACK_DEG_S;
      break;
  }
  baseTarget = constrain(baseTarget, BASE_ANGLE_MIN_DEG, BASE_ANGLE_MAX_DEG);

  float baseMaxStep = baseRate * (SENSOR_INTERVAL_MS / 1000.0f);
  float baseDelta = constrain(baseTarget - baseAngle, -baseMaxStep, baseMaxStep);
  baseAngle += baseDelta;

  settledTicks = moving ? 0 : settledTicks + 1;

  // 3) elevation target — independent of LDR error (see KNOWN LIMITATIONS):
  // flat/low in STW, fixed TRACK_ELEVATION_DEG in every other state.
  float elevTarget = (state == ST_STW) ? STOW_ELEVATION_DEG : TRACK_ELEVATION_DEG;
  elevTarget = constrain(elevTarget, ELEVATION_MIN_DEG, ELEVATION_MAX_DEG);
  float elevMaxStep = RATE_ELEVATION_DEG_S * (SENSOR_INTERVAL_MS / 1000.0f);
  float elevDelta = constrain(elevTarget - elevAngle, -elevMaxStep, elevMaxStep);
  elevAngle += elevDelta;
}

// ============================================================================
// Servo output — translates current baseAngle/elevAngle into hardware
// commands every loop() iteration. Never decides state; only flushes it.
// ============================================================================

void updateBaseServo() {
  if (BASE_SERVO_MODE == BASE_POSITIONAL) {
    baseServo.writeMicroseconds(
        angleToMicroseconds(baseAngle, BASE_ANGLE_MIN_DEG, BASE_ANGLE_MAX_DEG,
                             BASE_SERVO_MIN_US, BASE_SERVO_MAX_US));
    return;
  }

  // Continuous-rotation: never claim an absolute angle, only spin toward
  // baseTarget. STW and continuous-mode HOLD set baseTarget == baseAngle
  // every tick (see updateStateMachine), which naturally commands neutral
  // here with no special-casing needed.
  float remaining = baseTarget - baseAngle;
  if (fabsf(remaining) < 0.05f) {
    baseServo.writeMicroseconds(BASE_SERVO_NEUTRAL_US);
    return;
  }
  float speedFrac = constrain(fabsf(trackStepDeg) / STEP_MAX_DEG, 0.0f, 1.0f);
  int offset = BASE_SERVO_MIN_SPEED_US +
               (int)(speedFrac * (BASE_SERVO_MAX_SPEED_US - BASE_SERVO_MIN_SPEED_US));
  // CALIBRATE ON REAL HARDWARE: verify this sign convention drives the base
  // the correct physical direction; flip if left/right come out reversed.
  int us = (remaining > 0) ? (BASE_SERVO_NEUTRAL_US + offset) : (BASE_SERVO_NEUTRAL_US - offset);
  baseServo.writeMicroseconds(us);
}

void updateElevationServo() {
  elevServo.writeMicroseconds(
      angleToMicroseconds(elevAngle, ELEVATION_MIN_DEG, ELEVATION_MAX_DEG,
                           ELEVATION_SERVO_MIN_US, ELEVATION_SERVO_MAX_US));
}

// ============================================================================
// Safety / command handling
// ============================================================================

void setSafetyLatch(bool value) {
  if (value != safetyLatched) {
    safetyLatched = value;
    prefs.putBool("latch", value);
  }
}

void updateSafetyState() {
  currentLinkUp = (millis() - lastCommandMs) < LINK_TIMEOUT_MS;
  if (!currentLinkUp) {
    // Mirrors fake_esp32.py: losing the laptop link reverts HOLD to AUTO but
    // never touches the safety latch — UNKNOWN/no-link must never clear SAFE.
    mode = MODE_AUTO;
  }
}

// Parses and applies one "C,<verdict>,<mode>,<hold>" line. All-or-nothing,
// matching laptop/link.py's parse_command() exactly (including rejecting a
// line with more than 4 comma-separated fields). Malformed lines are
// silently ignored, same as fake_esp32.py.
void processCommandLine(char* line) {
  char* fields[4];
  int n = 0;
  char* tok = strtok(line, ",");
  while (tok != NULL && n < 4) {
    fields[n++] = tok;
    tok = strtok(NULL, ",");
  }
  if (n != 4 || tok != NULL || strcmp(fields[0], "C") != 0) return;

  const char* verdict = fields[1];
  const char* modeStr = fields[2];
  bool verdictOk = !strcmp(verdict, "OK") || !strcmp(verdict, "SAFE") || !strcmp(verdict, "UNKNOWN");
  bool modeOk = !strcmp(modeStr, "AUTO") || !strcmp(modeStr, "HOLD");
  if (!verdictOk || !modeOk) return;

  char* endptr;
  float hold = strtof(fields[3], &endptr);
  if (endptr == fields[3]) return;  // not a valid float

  // All fields valid — apply atomically.
  if (!strcmp(verdict, "SAFE")) setSafetyLatch(true);
  else if (!strcmp(verdict, "OK")) setSafetyLatch(false);
  // UNKNOWN: latch left untouched — this is the whole point of the latch.

  mode = !strcmp(modeStr, "HOLD") ? MODE_HOLD : MODE_AUTO;
  holdDegRequested = constrain(hold, BASE_ANGLE_MIN_DEG, BASE_ANGLE_MAX_DEG);
  lastCommandMs = millis();
}

void processSerialInput() {
  static char lineBuf[80];
  static uint8_t lineLen = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (lineLen > 0 && lineBuf[lineLen - 1] == '\r') lineLen--;
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) processCommandLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // overflow — drop the line, same spirit as "silently ignored"
    }
  }
}

// ============================================================================
// Telemetry
// ============================================================================

void sendTelemetry(unsigned long now) {
  const char* stateStr =
      (state == ST_TRK) ? "TRK" : (state == ST_STW) ? "STW" : (state == ST_RPN) ? "RPN" : "HLD";

  bool atLimit = (baseAngle <= BASE_ANGLE_MIN_DEG + 0.1f) || (baseAngle >= BASE_ANGLE_MAX_DEG - 0.1f);
  uint8_t flags = 0;
  if (currentLinkUp) flags |= FLAG_LINK;
  if (dark) flags |= FLAG_DARK;
  if (atLimit) flags |= FLAG_LIMIT;
  if (moving) flags |= FLAG_MOVING;

  Serial.printf("T,%lu,%s,%.1f,%.1f,%.0f,%.0f,%.3f,%d\n", now, stateStr, baseAngle, baseTarget,
                L_mV, R_mV, err, flags);
}

// ============================================================================
// Main loop — non-blocking, millis()-paced. No delay() here, ever.
// ============================================================================

void loop() {
  processSerialInput();
  updateSafetyState();

  unsigned long now = millis();

  static unsigned long lastSensorMs = 0;
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    readLDRs();
    calculateTrackingError();
    updateTrackingDecision();
    updateStateMachine(now);
  }

  updateBaseServo();
  updateElevationServo();

  static unsigned long lastTelemetryMs = 0;
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    sendTelemetry(now);
  }
}

// ============================================================================
// KNOWN LIMITATIONS (read before the demo)
// ============================================================================
// 1. Two LDRs give exactly ONE optical error dimension (light-left-vs-right).
//    That drives azimuth (the base) only. There is no elevation sensing at
//    all — elevation is an open-loop fixed set-point (TRACK_ELEVATION_DEG),
//    not "two-axis LDR tracking." Do not describe this as independent
//    dual-axis optical sensing.
// 2. If BASE_SERVO_MODE is BASE_CONTINUOUS (the default guess), baseAngle is
//    a DEAD-RECKONED SOFTWARE ESTIMATE, not a measured position — there is
//    no feedback sensor on a continuous-rotation servo. It can drift from
//    the real physical azimuth over time, especially after direction
//    reversals or stalls. BASE_ANGLE_MIN/MAX_DEG and FLAG_LIMIT are
//    therefore soft, estimated limits, not verified mechanical ones.
// 3. Because of (2), STOW and continuous-mode HOLD freeze the base in place
//    rather than driving it to a known absolute azimuth — there is nothing
//    to home to without a feedback mechanism (limit switch, encoder,
//    potentiometer, etc.), none of which are implemented here.
// 4. The frozen telemetry protocol carries one angle/target pair. It is
//    mapped to the BASE/azimuth axis, since that's what the 2-LDR error
//    actually drives. Elevation position is not telemetered at all — the
//    Python backend has no visibility into it. This is intentional
//    backward-compatibility, not an oversight: adding an elevation field
//    would require changing the frozen protocol.
// 5. FLAG_MOVING reflects base-axis LDR correction only, matching its
//    original single-axis meaning. Elevation motion between TRACK_ELEVATION
//    and STOW_ELEVATION does not set this flag.
