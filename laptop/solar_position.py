"""
solar_position.py — deterministic solar elevation angle calculation.

Pure, side-effect-free, unit-tested (see tests/test_solar_position.py). Used
ONLY to compute the top/elevation servo's set-point. Azimuth is NOT computed
here and never will be — it comes exclusively from the 2 LDRs, on the ESP32
(see firmware/solar_tracker/solar_tracker.ino). Two LDRs give one optical
error dimension; this module supplies the separate, calculated elevation
dimension. Do not describe the combination as "two-axis LDR sensing."

Algorithm: the NOAA Solar Position Algorithm, as published in NOAA's Solar
Calculator spreadsheets (https://gml.noaa.gov/grad/solcalc/), itself derived
from Jean Meeus, "Astronomical Algorithms", 2nd ed. (1998), chapter 25 "Solar
Coordinates". This is the truncated low-order series (a handful of
correction terms) — accurate to a small fraction of a degree, far more
precise than this demo's servo/mechanical tolerance requires, and small
enough to implement and unit-test directly without a third-party dependency
(the repo's existing "resist adding dependencies" preference — see
CLAUDE.md — plus this repo already requires Python 3.10+, so the stdlib
`zoneinfo` module is available for any caller that needs a local timezone).
"""
from __future__ import annotations

import math
from datetime import datetime, timezone


def calculate_solar_elevation(latitude: float, longitude: float, dt: datetime) -> float:
    """Solar elevation angle in degrees. >0 = above horizon, <=0 = at/below.

    `dt` MUST be timezone-aware (`dt.tzinfo is not None`) — naive datetimes
    are ambiguous about which timezone they represent and are rejected
    rather than silently assumed to be UTC or local. `dt` is converted to
    UTC internally; the result depends only on the instant in time `dt`
    names, not which timezone it was expressed in. Passing an explicit `dt`
    (rather than reading the clock inside this function) is what makes this
    deterministic and unit-testable — see solar_elevation_now() below for
    the wall-clock convenience wrapper.

    latitude: degrees, north positive.
    longitude: degrees, EAST positive (west negative) — matches the
    existing TRACKER_LON convention in laptop/weather.py (Open-Meteo's
    `longitude` query parameter uses the same sign convention).
    """
    if dt.tzinfo is None:
        raise ValueError(
            "calculate_solar_elevation requires a timezone-aware datetime "
            "(got a naive one). Pass dt.replace(tzinfo=timezone.utc) for "
            "UTC, or an aware local time (e.g. via zoneinfo) — never a bare "
            "datetime.now()."
        )
    dt_utc = dt.astimezone(timezone.utc)

    jd = _julian_day(dt_utc)
    t = (jd - 2451545.0) / 36525.0  # Julian centuries since J2000.0

    l0 = _geom_mean_long_sun(t)
    m = _geom_mean_anomaly_sun(t)
    e = _eccentricity_earth_orbit(t)
    c = _sun_eq_of_center(t, m)

    true_long = l0 + c
    omega = 125.04 - 1934.136 * t
    app_long = true_long - 0.00569 - 0.00478 * math.sin(math.radians(omega))

    eps0 = _mean_obliquity_ecliptic(t)
    eps = eps0 + 0.00256 * math.cos(math.radians(omega))

    decl = math.degrees(math.asin(math.sin(math.radians(eps)) * math.sin(math.radians(app_long))))

    eq_time = _equation_of_time(t, l0, e, m, eps)

    minutes_utc = dt_utc.hour * 60 + dt_utc.minute + dt_utc.second / 60.0
    true_solar_time = (minutes_utc + eq_time + 4.0 * longitude) % 1440.0

    hour_angle = true_solar_time / 4.0 - 180.0  # degrees; 0 at local solar noon

    lat_r, decl_r, ha_r = math.radians(latitude), math.radians(decl), math.radians(hour_angle)
    cos_zenith = (math.sin(lat_r) * math.sin(decl_r)
                  + math.cos(lat_r) * math.cos(decl_r) * math.cos(ha_r))
    cos_zenith = max(-1.0, min(1.0, cos_zenith))  # guard float drift at the poles/horizon
    zenith = math.degrees(math.acos(cos_zenith))

    return 90.0 - zenith


def solar_elevation_now(latitude: float, longitude: float) -> float:
    """Convenience wrapper: solar elevation right now, using the laptop's
    system clock (timezone-aware). Not itself unit-tested beyond a smoke
    test — all the real logic (and all the determinism) is in
    calculate_solar_elevation() above, which this just calls with `dt`
    filled in."""
    return calculate_solar_elevation(latitude, longitude, datetime.now().astimezone())


# --------------------------------------------------------------------------
# NOAA solar position algorithm — internal steps, kept as small named
# functions so each one is independently readable/checkable against the
# published spreadsheet formulas.
# --------------------------------------------------------------------------


def _julian_day(dt_utc: datetime) -> float:
    """Julian Day Number for a UTC datetime, via the Unix epoch (exact —
    avoids manual Gregorian-calendar Julian Day arithmetic)."""
    return 2440587.5 + dt_utc.timestamp() / 86400.0


def _geom_mean_long_sun(t: float) -> float:
    l0 = 280.46646 + t * (36000.76983 + t * 0.0003032)
    return l0 % 360.0


def _geom_mean_anomaly_sun(t: float) -> float:
    return 357.52911 + t * (35999.05029 - 0.0001537 * t)


def _eccentricity_earth_orbit(t: float) -> float:
    return 0.016708634 - t * (0.000042037 + 0.0000001267 * t)


def _sun_eq_of_center(t: float, m: float) -> float:
    m_r = math.radians(m)
    return (math.sin(m_r) * (1.914602 - t * (0.004817 + 0.000014 * t))
            + math.sin(2 * m_r) * (0.019993 - 0.000101 * t)
            + math.sin(3 * m_r) * 0.000289)


def _mean_obliquity_ecliptic(t: float) -> float:
    seconds = 21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))
    return 23.0 + (26.0 + seconds / 60.0) / 60.0


def _equation_of_time(t: float, l0: float, e: float, m: float, eps: float) -> float:
    """Minutes — how far apparent solar time runs ahead of mean solar time."""
    y = math.tan(math.radians(eps) / 2.0) ** 2
    l0_r, m_r = math.radians(l0), math.radians(m)
    result = (y * math.sin(2 * l0_r)
              - 2 * e * math.sin(m_r)
              + 4 * e * y * math.sin(m_r) * math.cos(2 * l0_r)
              - 0.5 * y * y * math.sin(4 * l0_r)
              - 1.25 * e * e * math.sin(2 * m_r))
    return 4.0 * math.degrees(result)
