"""Interlock logic: latch, no automatic restart, first-out, and agreement with the .st file."""
import os
import re

import pytest

from rh_cell_ic.interlock import FIRST_OUT_NAMES, INPUTS, Inputs, Interlock

ALL_OK = dict(estop_ok=True, door_closed=True, guard_ok=True, ft_alive=True, heartbeat_ok=True)


def ok(**override):
    d = dict(ALL_OK)
    d.update(override)
    return Inputs(**d)


def reset(il, **override):
    il.scan(ok(reset=True, **override))
    return il.scan(ok(reset=False, **override))


def test_starts_without_permit_even_when_healthy():
    il = Interlock()
    assert il.scan(ok()) is False


def test_reset_rising_edge_gives_permit():
    il = Interlock()
    assert il.scan(ok(reset=True)) is True
    assert il.first_out == 0


def test_holding_reset_does_not_rearm_after_trip():
    il = Interlock()
    il.scan(ok(reset=True))
    il.scan(ok(estop_ok=False, reset=True))
    # fault cleared, reset button still held: no rising edge -> no permit
    assert il.scan(ok(reset=True)) is False


def test_reset_ignored_while_unhealthy():
    il = Interlock()
    assert il.scan(ok(door_closed=False, reset=True)) is False


@pytest.mark.parametrize("name", INPUTS)
def test_any_input_drops_permit_and_no_auto_restart(name):
    il = Interlock()
    assert reset(il) is True
    assert il.scan(ok(**{name: False})) is False
    assert il.first_out == INPUTS.index(name) + 1
    assert il.scan(ok()) is False          # fault gone, still no permit
    assert reset(il) is True               # operator reset needed


def test_first_out_is_the_first_cause_not_the_cascade():
    il = Interlock()
    reset(il)
    il.scan(ok(guard_ok=False))                          # force guard trips first
    il.scan(ok(guard_ok=False, estop_ok=False))          # then someone hits the e-stop
    assert il.first_out_name == "FORCE GUARD"


def test_simultaneous_faults_use_priority_order():
    il = Interlock()
    reset(il)
    il.scan(ok(heartbeat_ok=False, estop_ok=False))
    assert il.first_out_name == "E-STOP"


def test_st_file_uses_the_same_codes():
    here = os.path.dirname(__file__)
    st = open(os.path.join(here, "..", "plc", "rh_cell_interlock.st")).read()
    # every "first_out := N" in the .st file must name a code that exists in the Python table
    codes = {int(c) for c in re.findall(r"first_out\s*:=\s*(\d+)", st)}
    assert codes - {0} == set(range(1, 7))
    assert set(FIRST_OUT_NAMES) == set(range(0, 7))
    # the priority order in the ELSIF chain matches INPUTS
    chain = re.findall(r"in_(\w+?)\s*<>\s*1", st)
    assert chain == ["estop_ok", "door_closed", "guard_ok", "ft_alive"]
    assert list(INPUTS[:4]) == chain
