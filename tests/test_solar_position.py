"""Tests for laptop/solar_position.py — deterministic, no wall-clock reads.

Every test passes an explicit tz-aware datetime; none of them depend on
`datetime.now()`. Reference expectations are basic, well-known solar
geometry (declination ~= 0 at equinox, ~= +/-23.44 deg at solstices), not a
copy of the implementation, so these tests would catch a genuinely wrong
formula, not just confirm the code does what the code does.
"""
import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))

from solar_position import calculate_solar_elevation  # noqa: E402

GREENWICH = (51.4769, -0.0005)  # lat, lon -- lon ~ 0 so solar noon ~= 12:00 UTC
EQUATOR_LON0 = (0.0, 0.0)


# ---- A. sunrise / low sun ----

def test_low_sun_shortly_after_sunrise():
    # Equinox sunrise at the equator is ~06:00 local solar time (lon 0 here).
    dt = datetime(2026, 3, 20, 6, 15, 0, tzinfo=timezone.utc)
    e = calculate_solar_elevation(*EQUATOR_LON0, dt)
    assert 0 < e < 15, f"expected a low positive elevation shortly after sunrise, got {e}"


# ---- B. midday ----

def test_midday_elevation_is_substantially_higher_than_sunrise():
    sunrise = calculate_solar_elevation(*EQUATOR_LON0, datetime(2026, 3, 20, 6, 15, 0, tzinfo=timezone.utc))
    noon = calculate_solar_elevation(*EQUATOR_LON0, datetime(2026, 3, 20, 12, 0, 0, tzinfo=timezone.utc))
    assert noon > sunrise + 40, f"noon ({noon}) should be far higher than sunrise ({sunrise})"
    assert noon > 80, f"equinox noon at the equator should be near-zenith, got {noon}"


# ---- C. sunset ----

def test_sun_approaches_horizon_near_sunset():
    # Equinox sunset at the equator is ~18:00 local solar time.
    dt = datetime(2026, 3, 20, 17, 45, 0, tzinfo=timezone.utc)
    e = calculate_solar_elevation(*EQUATOR_LON0, dt)
    assert 0 < e < 15, f"expected a low positive elevation shortly before sunset, got {e}"


# ---- D. night ----

def test_night_elevation_is_at_or_below_horizon():
    dt = datetime(2026, 3, 20, 0, 0, 0, tzinfo=timezone.utc)  # midnight at lon 0
    e = calculate_solar_elevation(*EQUATOR_LON0, dt)
    assert e <= 0, f"expected sun below horizon at midnight, got {e}"


# ---- E. different date ----

def test_elevation_changes_with_date_matching_solstice_geometry():
    noon = lambda month, day: calculate_solar_elevation(  # noqa: E731
        *GREENWICH, datetime(2026, month, day, 12, 0, 0, tzinfo=timezone.utc))
    summer = noon(6, 21)
    winter = noon(12, 21)
    equinox = noon(3, 20)
    # Northern-hemisphere mid-latitude: summer solstice noon is highest,
    # winter solstice noon is lowest, equinox sits in between.
    assert summer > equinox > winter, (summer, equinox, winter)
    # Solstice separation should be close to 2x the axial tilt (~46.9 deg).
    assert abs((summer - winter) - 46.9) < 2.0, (summer, winter)


# ---- F. different latitude ----

def test_elevation_changes_with_latitude():
    dt = datetime(2026, 3, 20, 12, 0, 0, tzinfo=timezone.utc)
    equator = calculate_solar_elevation(0.0, 0.0, dt)
    mid_lat = calculate_solar_elevation(51.4769, 0.0, dt)
    near_pole = calculate_solar_elevation(80.0, 0.0, dt)
    assert equator > mid_lat > near_pole, (equator, mid_lat, near_pole)


# ---- determinism / input validation ----

def test_same_input_gives_identical_output():
    dt = datetime(2026, 6, 1, 15, 30, 0, tzinfo=timezone.utc)
    a = calculate_solar_elevation(40.0, -74.0, dt)
    b = calculate_solar_elevation(40.0, -74.0, dt)
    assert a == b


def test_naive_datetime_is_rejected():
    with pytest.raises(ValueError):
        calculate_solar_elevation(0.0, 0.0, datetime(2026, 1, 1))


def test_timezone_aware_local_time_matches_equivalent_utc_instant():
    from zoneinfo import ZoneInfo
    utc_dt = datetime(2026, 6, 1, 12, 0, 0, tzinfo=timezone.utc)
    local_dt = utc_dt.astimezone(ZoneInfo("America/New_York"))
    assert calculate_solar_elevation(40.0, -74.0, utc_dt) == calculate_solar_elevation(40.0, -74.0, local_dt)
