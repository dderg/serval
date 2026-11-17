"""The hunt driver sizes each batch to the slot time it targets and stops,
under the slot's name for it, when slots.py ends a batch from outside."""

import importlib

import pytest

from tools.sim import hunt

pytestmark = pytest.mark.sim_unit

TEN_MINUTES_S = 600.0
SEEDS = range(100, 110)


def _slots(monkeypatch):
    monkeypatch.syspath_prepend(str(hunt.SLOTS.parent))
    return importlib.import_module("slots")


def test_slot_ends_are_the_exits_slots_py_names(monkeypatch):
    slots = _slots(monkeypatch)
    assert hunt.slot_end(slots.HOLD_EXPIRED_EXIT) is hunt.SlotEnd.HOLD_CAP
    assert hunt.slot_end(slots.MEMORY_CAP_EXIT) is hunt.SlotEnd.MEMORY_CAP
    assert hunt.slot_end(0) is None
    assert hunt.slot_end(1) is None


@pytest.mark.parametrize("end", list(hunt.SlotEnd))
def test_a_batch_ended_by_its_slot_stops_the_hunt_under_that_name(
    monkeypatch, capsys, end
):
    before_and_after = iter([set(), set(SEEDS[:3])])
    monkeypatch.setattr(
        hunt, "recorded_seeds", lambda run_name: next(before_and_after)
    )
    monkeypatch.setattr(
        hunt, "_run_batch", lambda seeds, run_name, extra: (end.value, 2400.0)
    )
    argv = ["--seeds", f"{SEEDS[0]}..{SEEDS[-1]}", "--run-name", "t"]
    assert hunt.main(argv) == end.value
    message = capsys.readouterr().err
    assert end.name in message
    assert "3 of its 10 seeds recorded, the rest stay pending" in message


def test_a_batch_that_records_nothing_stops_the_hunt(monkeypatch):
    monkeypatch.setattr(hunt, "recorded_seeds", lambda run_name: set())
    monkeypatch.setattr(
        hunt, "_run_batch", lambda seeds, run_name, extra: (1, 10.0)
    )
    with pytest.raises(SystemExit):
        hunt.main(["--seeds", "1..2", "--run-name", "t"])


def test_a_batch_fills_the_target_at_the_last_rate():
    assert hunt.next_size(40, 300.0, TEN_MINUTES_S) == 80
    assert hunt.next_size(40, 1200.0, TEN_MINUTES_S) == 20


def test_a_batch_stays_inside_its_bounds():
    assert hunt.next_size(1, 3600.0, TEN_MINUTES_S) == hunt.MIN_BATCH
    assert hunt.next_size(1000, 10.0, TEN_MINUTES_S) == hunt.MAX_BATCH
    assert hunt.next_size(0, 100.0, TEN_MINUTES_S) == hunt.MIN_BATCH
