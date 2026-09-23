"""Tests for the pure decision engine in laptop/weather.py.

evaluate_rules() takes no clock reads and has no side effects, so every test
here controls `now` explicitly instead of relying on wall-clock time.
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))

from weather import (  # noqa: E402
    WeatherData, Mem, RulesConfig, evaluate_rules,
    SCENARIOS, make_sim_weather, THUNDERSTORM_CODES,
)

CFG = RulesConfig(safe_gust_kmh=40.0, clear_gust_kmh=30.0,
                   dwell_live_s=600.0, dwell_sim_s=8.0, stale_s=900.0)


def wx(**kw) -> WeatherData:
    base = dict(code=1, cloud_pct=10, wind_kmh=10, gust_kmh=10, gust_next_kmh=10, t=0.0, source="live")
    base.update(kw)
    return WeatherData(**base)


# ---- scenario -> verdict, through the SAME function as live data ----

def test_normal_scenario_is_ok():
    v, reason, mem = evaluate_rules(make_sim_weather("NORMAL", 0), True, 0, Mem(), CFG)
    assert v == "OK"
    assert not mem.latched


def test_cloudy_scenario_is_ok_despite_high_cloud_cover():
    assert SCENARIOS["CLOUDY"]["cloud_pct"] >= 90  # sanity-check the fixture itself
    v, reason, mem = evaluate_rules(make_sim_weather("CLOUDY", 0), True, 0, Mem(), CFG)
    assert v == "OK", "cloud cover must never be a SAFE trigger on its own"


def test_high_wind_scenario_triggers_safe():
    v, reason, mem = evaluate_rules(make_sim_weather("HIGH WIND", 0), True, 0, Mem(), CFG)
    assert v == "SAFE"
    assert mem.latched
    assert "gust" in reason.lower()


def test_storm_scenario_triggers_safe_via_thunderstorm_code():
    assert SCENARIOS["STORM"]["code"] in THUNDERSTORM_CODES  # sanity-check the fixture
    v, reason, mem = evaluate_rules(make_sim_weather("STORM", 0), True, 0, Mem(), CFG)
    assert v == "SAFE"
    assert mem.latched


def test_all_scenarios_produce_the_expected_verdict():
    expected = {"NORMAL": "OK", "CLOUDY": "OK", "HIGH WIND": "SAFE", "STORM": "SAFE"}
    for name, want in expected.items():
        v, _, _ = evaluate_rules(make_sim_weather(name, 0), True, 0, Mem(), CFG)
        assert v == want, f"scenario {name}: expected {want}, got {v}"


# ---- the core safety property: missing/failed data never clears an existing latch ----

def test_missing_weather_while_latched_returns_unknown_and_keeps_latch():
    mem = Mem(latched=True, clear_since=None)
    v, reason, mem2 = evaluate_rules(None, True, 100, mem, CFG)
    assert v == "UNKNOWN"
    assert mem2.latched is True


def test_fetch_failure_while_latched_returns_unknown_and_keeps_latch():
    mem = Mem(latched=True)
    v, reason, mem2 = evaluate_rules(wx(t=100), False, 100, mem, CFG)
    assert v == "UNKNOWN"
    assert mem2.latched is True


def test_stale_weather_is_treated_as_unknown():
    mem = Mem(latched=False)
    old_reading = wx(t=0, gust_kmh=5)
    v, reason, mem2 = evaluate_rules(old_reading, True, CFG.stale_s + 1, mem, CFG)
    assert v == "UNKNOWN"
    assert mem2.latched is False  # was never latched; still isn't


def test_unknown_never_clears_an_existing_latch_even_after_a_long_time():
    mem = Mem(latched=True, clear_since=50)
    v, reason, mem2 = evaluate_rules(None, False, 200, mem, CFG)
    assert v == "UNKNOWN"
    assert mem2.latched is True
    # clear_since resets under UNKNOWN so a later real "OK" has to complete a full dwell
    assert mem2.clear_since is None


def test_unknown_while_unlatched_stays_unlatched():
    mem = Mem(latched=False)
    v, reason, mem2 = evaluate_rules(None, False, 10, mem, CFG)
    assert v == "UNKNOWN"
    assert mem2.latched is False


# ---- recovery dwell ----

def test_recovery_dwell_live_gates_clearing():
    mem = Mem(latched=True)

    v, reason, mem = evaluate_rules(wx(t=0, gust_kmh=5, gust_next_kmh=5, source="live"), True, 0, mem, CFG)
    assert v == "SAFE" and mem.clear_since == 0

    v, reason, mem = evaluate_rules(
        wx(t=CFG.dwell_live_s - 1, gust_kmh=5, gust_next_kmh=5, source="live"),
        True, CFG.dwell_live_s - 1, mem, CFG)
    assert v == "SAFE", f"should still be waiting out the dwell: {reason}"

    v, reason, mem = evaluate_rules(
        wx(t=CFG.dwell_live_s + 1, gust_kmh=5, gust_next_kmh=5, source="live"),
        True, CFG.dwell_live_s + 1, mem, CFG)
    assert v == "OK" and not mem.latched


def test_recovery_dwell_simulated_is_much_faster_than_live():
    mem = Mem(latched=True)
    v, reason, mem = evaluate_rules(
        wx(t=0, gust_kmh=5, gust_next_kmh=5, source="sim:NORMAL"), True, 0, mem, CFG)
    assert v == "SAFE" and mem.clear_since == 0

    v, reason, mem = evaluate_rules(
        wx(t=CFG.dwell_sim_s + 1, gust_kmh=5, gust_next_kmh=5, source="sim:NORMAL"),
        True, CFG.dwell_sim_s + 1, mem, CFG)
    assert v == "OK" and not mem.latched


def test_a_gust_spike_during_the_dwell_resets_the_clearing_timer():
    mem = Mem(latched=True)
    v, _, mem = evaluate_rules(wx(t=0, gust_kmh=5, gust_next_kmh=5), True, 0, mem, CFG)
    assert mem.clear_since == 0

    # a brief spike above the clear threshold mid-dwell
    v, _, mem = evaluate_rules(wx(t=5, gust_kmh=35, gust_next_kmh=35), True, 5, mem, CFG)
    assert v == "SAFE"
    assert mem.clear_since is None  # timer reset

    # calm again — the dwell must restart from here, not from t=0
    v, _, mem = evaluate_rules(wx(t=6, gust_kmh=5, gust_next_kmh=5), True, 6, mem, CFG)
    assert mem.clear_since == 6
    v, _, mem = evaluate_rules(wx(t=6 + CFG.dwell_sim_s - 1, gust_kmh=5, gust_next_kmh=5),
                                True, 6 + CFG.dwell_sim_s - 1, mem, CFG)
    assert v == "SAFE", "must not clear early using the original t=0 start time"


# ---- hysteresis ----

def test_gust_hysteresis_holds_safe_between_clear_and_safe_thresholds():
    mem = Mem(latched=True)
    borderline = wx(t=0, gust_kmh=35, gust_next_kmh=35)  # between clear(30) and safe(40)
    v, reason, mem2 = evaluate_rules(borderline, True, 0, mem, CFG)
    assert v == "SAFE"
    assert mem2.clear_since is None  # hasn't started clearing yet
    assert "hysteresis" in reason.lower()


def test_hysteresis_only_matters_once_latched_not_when_already_clear():
    # 35 km/h is below the 40 SAFE trigger, so if we were never latched, it's fine
    v, reason, mem = evaluate_rules(wx(t=0, gust_kmh=35, gust_next_kmh=35), True, 0, Mem(latched=False), CFG)
    assert v == "OK"


# ---- pre-emptive forecast gust ----

def test_forecast_gust_triggers_preemptive_safe_even_if_calm_right_now():
    mem = Mem()
    calm_now_windy_soon = wx(t=0, gust_kmh=15, gust_next_kmh=45)
    v, reason, mem = evaluate_rules(calm_now_windy_soon, True, 0, mem, CFG)
    assert v == "SAFE"
    assert "forecast" in reason.lower() or "pre-emptive" in reason.lower()


# ---- invariant / fuzz check across the whole input space ----

def test_safe_iff_thunderstorm_or_gust_over_threshold_when_starting_unlatched():
    random.seed(0)
    for _ in range(300):
        code = random.choice([0, 1, 2, 3, 45, 61, 80, 95, 96, 99])
        gust = random.uniform(0, 90)
        gust_next = random.uniform(0, 90)
        sample = wx(t=0, code=code, gust_kmh=gust, gust_next_kmh=gust_next)
        v, _, mem = evaluate_rules(sample, True, 0, Mem(), CFG)
        expect_safe = (code in THUNDERSTORM_CODES) or (gust >= CFG.safe_gust_kmh) or (gust_next >= CFG.safe_gust_kmh)
        assert (v == "SAFE") == expect_safe, (code, gust, gust_next, v)
