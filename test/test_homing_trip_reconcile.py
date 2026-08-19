import types

import pytest
from fakes import FakeGcmd, FakePrinter, FakeReactor, FakeToolhead

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

    def home_axis_start(self, axis, direction, speed, max_travel, endstops):
        self.unreconciled += self.overshoot

    def home_axis_poll(self):
        final_z = self.stop_z + self.unreconciled
        return ([0.0, 0.0, self.stop_z], [0.0, 0.0, final_z], 19)

    def reconcile_position(self, position):
        self.reconciliations.append(list(position))
        self.unreconciled = 0.0


class ReconciledKinematics:
    def __init__(self, engine):
        self.engine = engine

    def set_position(self, position, homing_axes):
        self.engine.reconcile_position(position)


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
    toolhead = FakeToolhead(
        kin=ReconciledKinematics(engine), position=[0.0, 0.0, 10.0, 0.0]
    )
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
