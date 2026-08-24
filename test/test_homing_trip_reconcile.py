import types

import pytest
from fakes import FakeGcmd, FakeMcu, FakePrinter, FakeReactor

from klippy import motion as motion_mod
from klippy import motion_kinematics
from klippy.extras import homing as homing_mod
from klippy.extras.homing import Homing


class StepcompressEndstop:
    endstop_id = 4

    def engine_mcu_handle(self):
        return 1

    def remote_freeze(self):
        return None

    def arm(self, poll_period):
        pass

    def disarm(self):
        pass

    def query_trip_state(self):
        return {"tripped": True, "trip_clock": 19}


class StepcompressLaneEngine:
    def __init__(self, stop_z, overshoot):
        self.stop_z = stop_z
        self.overshoot = overshoot
        self.unreconciled = 0.0
        self.reconciliations = []

    def motion_drained(self):
        return True

    def motion_drain_poll(self):
        return True

    def motion_drain_finalize(self):
        return None

    def wait_moves(self):
        return None

    def frontier_print_time(self, mcu_handle):
        return 0.0

    def home_axis_start(self, axis, direction, speed, max_travel, endstops):
        self.unreconciled += self.overshoot

    def home_axis_poll(self):
        final_z = self.stop_z + self.unreconciled
        return ([0.0, 0.0, self.stop_z], [0.0, 0.0, final_z], 19)

    def set_position(self, x, y, z):
        self.reconciliations.append((x, y, z))
        self.unreconciled = 0.0


def production_toolhead(engine, position):
    """The real reconciliation bridge: Motion.set_position ->
    kinematics.set_position -> engine.set_position. Only the engine and the
    mcu clock are faked, so dropping any host link breaks these tests."""
    printer = FakePrinter(reactor=FakeReactor())
    toolhead = motion_mod.Motion.__new__(motion_mod.Motion)
    kin = motion_kinematics._LinearKinematics.__new__(
        motion_kinematics._LinearKinematics
    )
    kin._motion = toolhead
    kin.rails = []
    kin.limits = [(1.0, -1.0)] * 3
    kin._parked_dirty = [False] * 3
    toolhead.printer = printer
    toolhead.reactor = printer.get_reactor()
    toolhead.mcu = FakeMcu(printer=printer, handle=1, est_print_time=1.0)
    toolhead.motion_lead = 0.25
    toolhead.engine = engine
    toolhead.kin = kin
    toolhead.commanded_pos = list(position)
    return toolhead


@pytest.fixture
def homing(monkeypatch):
    monkeypatch.setattr(
        homing_mod,
        "get_danger_options",
        lambda: types.SimpleNamespace(homing_trip_deadline_margin=5.0),
    )
    instance = Homing.__new__(Homing)
    instance.printer = FakePrinter(reactor=FakeReactor())
    return instance


def test_repeated_stepcompress_trips_reconcile_without_frame_drift(homing):
    engine = StepcompressLaneEngine(stop_z=5.0, overshoot=-0.03125)
    toolhead = production_toolhead(engine, [0.0, 0.0, 10.0, 0.0])
    entry = {"endstops": [StepcompressEndstop()], "provider": None}

    final_z = []
    for _ in range(12):
        _, final_pos = homing.trip_move(
            FakeGcmd(error=RuntimeError),
            toolhead,
            engine,
            2,
            -1.0,
            5.0,
            10.0,
            entry,
        )
        final_z.append(final_pos[2])

    assert final_z == pytest.approx([5.0 - 0.03125] * 12)
    assert [
        position[2] for position in engine.reconciliations
    ] == pytest.approx(final_z)


def test_a_trip_reconciles_the_host_position_through_the_kinematics(homing):
    engine = StepcompressLaneEngine(stop_z=5.0, overshoot=-0.03125)
    toolhead = production_toolhead(engine, [1.5, 2.5, 10.0, 7.0])
    entry = {"endstops": [StepcompressEndstop()], "provider": None}

    homing.trip_move(
        FakeGcmd(error=RuntimeError),
        toolhead,
        engine,
        2,
        -1.0,
        5.0,
        10.0,
        entry,
    )

    assert toolhead.get_position() == pytest.approx(
        [0.0, 0.0, 5.0 - 0.03125, 7.0]
    )
    assert engine.reconciliations == [pytest.approx((0.0, 0.0, 5.0 - 0.03125))]
    assert "toolhead:set_position" in toolhead.printer.events
