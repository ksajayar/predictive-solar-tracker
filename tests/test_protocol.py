"""Tests for the frozen wire protocol implemented in laptop/link.py."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))

from link import (  # noqa: E402
    parse_telemetry, format_telemetry, parse_boot, format_boot,
    parse_command, format_command, TELEMETRY_STATES, VERDICTS, MODES,
    FLAG_LINK, FLAG_DARK, FLAG_MOVING,
)


# ---- telemetry: parse_telemetry ----

def test_parse_valid_telemetry():
    t = parse_telemetry("T,183420,TRK,31.5,33.0,1820,2410,-0.139,9")
    assert t is not None
    assert t["ms"] == 183420
    assert t["state"] == "TRK"
    assert t["angle"] == 31.5
    assert t["target"] == 33.0
    assert t["L"] == 1820
    assert t["R"] == 2410
    assert abs(t["err"] - (-0.139)) < 1e-9
    assert t["flags"] == 9
    assert t["flags"] & FLAG_LINK
    assert t["flags"] & FLAG_MOVING
    assert not (t["flags"] & FLAG_DARK)


def test_parse_strips_whitespace_and_crlf():
    t = parse_telemetry("T,1,TRK,0,0,0,0,0,0\r\n")
    assert t is not None
    assert t["ms"] == 1


@pytest.mark.parametrize("line", [
    "T,1,TRK,1,1,1,1,1",        # 8 fields, missing one
    "T,1,TRK,1,1,1,1,1,1,1",     # 10 fields, one too many
    "1,TRK,1,1,1,1,1,1",         # missing prefix entirely
])
def test_invalid_field_count(line):
    assert parse_telemetry(line) is None


@pytest.mark.parametrize("line", [
    "T,abc,TRK,1,1,1,1,1,1",       # ms not an int
    "T,1,TRK,xx,1,1,1,1,1",        # angle not a float
    "T,1,TRK,1,yy,1,1,1,1",        # target not a float
    "T,1,TRK,1,1,zz,1,1,1",        # L not a float
    "T,1,TRK,1,1,1,ww,1,1",        # R not a float
    "T,1,TRK,1,1,1,1,vv,1",        # err not a float
    "T,1,TRK,1,1,1,1,1,notanint",  # flags not an int
    "T,1.5,TRK,1,1,1,1,1,1",       # ms must be an int, not a float string
])
def test_malformed_numbers_rejected(line):
    assert parse_telemetry(line) is None


@pytest.mark.parametrize("line", [
    "T,1,XXX,1,1,1,1,1,1",   # not a recognized state
    "T,1,trk,1,1,1,1,1,1",   # case-sensitive
    "T,1,,1,1,1,1,1,1",      # empty state
])
def test_unknown_state_rejected(line):
    assert parse_telemetry(line) is None


@pytest.mark.parametrize("line", [
    "",
    "\x00\x01\x02garbage",
    "hello world",
    ",,,,,,,,",
    "C,OK,AUTO,0",          # a command line, not telemetry
    "B,tracker,v1,reset=0,latch=0",  # a boot line, not telemetry
    "X,1,TRK,1,1,1,1,1,1",  # wrong prefix
])
def test_garbage_and_wrong_prefix_rejected(line):
    assert parse_telemetry(line) is None


def test_telemetry_round_trip():
    line = format_telemetry(ms=5000, state="RPN", angle=-12.34, target=-10.0,
                             L=1000.4, R=999.6, err=0.0004, flags=5)
    t = parse_telemetry(line)
    assert t is not None
    assert t["ms"] == 5000
    assert t["state"] == "RPN"
    assert abs(t["angle"] - (-12.3)) < 0.05  # formatted to 1 decimal place
    assert t["flags"] == 5


def test_format_telemetry_rejects_unknown_state():
    with pytest.raises(ValueError):
        format_telemetry(ms=0, state="ZZZ", angle=0, target=0, L=0, R=0, err=0, flags=0)


# ---- boot packet ----

def test_parse_boot_packet():
    b = parse_boot("B,tracker,v1,reset=9,latch=1")
    assert b is not None
    assert b["reset"] == 9
    assert b["latch"] == 1


def test_parse_boot_rejects_non_boot_lines():
    assert parse_boot("T,1,TRK,1,1,1,1,1,1") is None
    assert parse_boot("garbage") is None
    assert parse_boot("") is None


def test_format_boot_round_trip():
    line = format_boot(reset=3, latch=True)
    b = parse_boot(line)
    assert b["reset"] == 3
    assert b["latch"] == 1
    line2 = format_boot(reset=0, latch=False)
    b2 = parse_boot(line2)
    assert b2["latch"] == 0


# ---- command: parse_command / format_command ----

def test_parse_command_valid():
    # 4-field form (no elevation target): elevation parses as None.
    assert parse_command("C,OK,AUTO,0") == ("OK", "AUTO", 0.0, None)
    assert parse_command("C,SAFE,AUTO,0") == ("SAFE", "AUTO", 0.0, None)
    assert parse_command("C,UNKNOWN,AUTO,0") == ("UNKNOWN", "AUTO", 0.0, None)
    assert parse_command("C,OK,HOLD,20") == ("OK", "HOLD", 20.0, None)
    assert parse_command("C,OK,HOLD,-15.5") == ("OK", "HOLD", -15.5, None)


def test_parse_command_valid_with_elevation():
    # 5-field form (calculated solar elevation target).
    assert parse_command("C,OK,AUTO,0,57.4") == ("OK", "AUTO", 0.0, 57.4)
    assert parse_command("C,OK,AUTO,0,-10.0") == ("OK", "AUTO", 0.0, -10.0)  # sun below horizon
    assert parse_command("C,SAFE,AUTO,0,57.4") == ("SAFE", "AUTO", 0.0, 57.4)  # still parses -- SAFE authority is the firmware's job, not the parser's


@pytest.mark.parametrize("line", [
    "C,BANANA,AUTO,0",           # unknown verdict
    "C,OK,SIDEWAYS,0",           # unknown mode
    "C,OK,AUTO,notanumber",      # unparseable hold angle
    "C,OK,AUTO",                 # too few fields
    "C,OK,AUTO,0,notanumber",    # unparseable elevation
    "C,OK,AUTO,0,57.4,extra",    # too many fields (6)
    "T,1,TRK,1,1,1,1,1,1",       # wrong prefix (a telemetry line)
    "",
    "garbage",
])
def test_parse_command_invalid(line):
    assert parse_command(line) is None


def test_command_round_trip():
    line = format_command("SAFE", "HOLD", 12.5)
    assert line == "C,SAFE,HOLD,12.5"  # old 4-field form, byte-for-byte unchanged
    assert parse_command(line) == ("SAFE", "HOLD", 12.5, None)


def test_command_round_trip_with_elevation():
    line = format_command("OK", "AUTO", 0.0, elevation=57.4)
    assert line == "C,OK,AUTO,0.0,57.4"
    assert parse_command(line) == ("OK", "AUTO", 0.0, 57.4)


def test_format_command_rejects_invalid_values():
    with pytest.raises(ValueError):
        format_command("MAYBE", "AUTO", 0)
    with pytest.raises(ValueError):
        format_command("OK", "SIDEWAYS", 0)


def test_protocol_vocabularies_frozen():
    assert TELEMETRY_STATES == {"TRK", "STW", "RPN", "HLD"}
    assert VERDICTS == {"OK", "SAFE", "UNKNOWN"}
    assert MODES == {"AUTO", "HOLD"}
