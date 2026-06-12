import types

import pytest

from klippy.extras import homing as homing_mod
from klippy.rail import HomingInfo

TOLERANCE = 0.5


class CommandError(Exception):
    pass


class FakeGcmd:
    error = CommandError


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class FakeStepperEnable:
    def __init__(self):
        self.calls = []

    def motor_debug_enable(self, name, enable):
        self.calls.append((name, enable))


class FakeToolhead:
    def __init__(self, position):
        self._pos = list(position)
        self.moves = []
        self.set_positions = []

    def get_position(self):
        return list(self._pos)

    def set_position(self, newpos, homing_axes=None):
        self._pos = list(newpos)
        self.set_positions.append(
            (list(newpos), list(homing_axes) if homing_axes else None)
        )

    def move(self, pos, speed):
        self._pos = list(pos)
        self.moves.append((list(pos), speed))

    def wait_moves(self):
        pass

    def get_last_move_time(self):
        return 0.0


class FakeHomingRail:
    def __init__(self, hi, position_range):
        self._hi = hi
        self._range = position_range

    def get_homing_info(self):
        return self._hi

    def get_range(self):
        return self._range

    def get_steppers(self):
        return [FakeStepper("stepper_x")]

    def get_tmc_current_helpers(self):
        return []

    def get_name(self, short=False):
        return "stepper_x"


class FakeKin:
    def __init__(self, rail):
        self._rail = rail

    def _axis_rails(self):
        return {0: self._rail}

    def active_rails(self, *deltas):
        return [self._rail]


class FakePrinter:
    def __init__(self, stepper_enable):
        self._stepper_enable = stepper_enable
        self.events = []

    def lookup_object(self, name):
        if name == "stepper_enable":
            return self._stepper_enable
        raise KeyError(name)

    def send_event(self, event, *args):
        self.events.append((event, args))


def make_homing_info(**overrides):
    base = dict(
        speed=10.0,
        position_endstop=0.0,
        retract_speed=10.0,
        retract_dist=5.0,
        positive_dir=False,
        second_homing_speed=8.0,
        use_sensorless_homing=True,
        min_home_dist=40.0,
        accel=None,
    )
    base.update(overrides)
    return HomingInfo(**base)


def scripted_trip(travels, overshoot=0.0):
    seq = iter(travels)
    calls = []

    def trip_move(
        gcmd, toolhead, bridge, axis, direction, speed, max_travel, entry
    ):
        travel = next(seq)
        calls.append({"speed": speed, "max_travel": max_travel})
        no_trigger_within_travel = travel > max_travel + 1e-9
        if no_trigger_within_travel:
            raise gcmd.error(
                "%s endstop did not trigger within %.1fmm of travel"
                % ("XYZ"[axis], max_travel)
            )
        start = toolhead.get_position()[axis]
        trip = list(toolhead.get_position())
        trip[axis] = start + direction * travel
        final = list(trip)
        final[axis] = trip[axis] + direction * overshoot
        return trip, final

    trip_move.calls = calls
    return trip_move


@pytest.fixture(autouse=True)
def _fixed_tolerance(monkeypatch):
    monkeypatch.setattr(
        homing_mod,
        "get_danger_options",
        lambda: types.SimpleNamespace(
            homing_elapsed_distance_tolerance=TOLERANCE
        ),
    )


def run_home_axis(hi, travels, overshoot=0.0, initial=(0.0, 0.0, 0.0, 0.0)):
    rail = FakeHomingRail(hi, (0.0, 200.0))
    kin = FakeKin(rail)
    toolhead = FakeToolhead(initial)
    stepper_enable = FakeStepperEnable()
    printer = FakePrinter(stepper_enable)
    entry = {"endstop": object(), "provider": None, "trigger_height": None}

    homing = homing_mod.Homing.__new__(homing_mod.Homing)
    homing.printer = printer
    homing._homing_axes = []

    trip = scripted_trip(travels, overshoot)
    homing.trip_move = trip

    gcmd = FakeGcmd()
    bridge = object()
    homing._home_axis(gcmd, toolhead, bridge, kin, 0, entry)
    return homing, toolhead, trip


def test_adequate_first_home_does_not_rehome():
    homing, toolhead, trip = run_home_axis(make_homing_info(), [50.0])
    assert len(trip.calls) == 1
    assert toolhead.moves == [([5.0, 0.0, 0.0, 0.0], 10.0)]
    assert homing._homing_axes == [0]
    assert [e[0] for e in homing.printer.events] == ["homing:home_rails_end"]


def test_early_first_home_retracts_and_rehomes():
    homing, toolhead, trip = run_home_axis(make_homing_info(), [10.0, 40.0])
    assert len(trip.calls) == 2
    second_homing_speed = 8.0
    min_home_dist_plus_tolerance = 40.5
    assert trip.calls[1]["speed"] == second_homing_speed
    assert trip.calls[1]["max_travel"] == pytest.approx(
        min_home_dist_plus_tolerance
    )
    rehome_setup_retract = ([40.0, 0.0, 0.0, 0.0], 10.0)
    final_resting_retract = ([5.0, 0.0, 0.0, 0.0], 10.0)
    assert toolhead.moves[0] == rehome_setup_retract
    assert toolhead.moves[-1] == final_resting_retract
    assert homing._homing_axes == [0]


def test_early_trigger_on_rehome_aborts():
    with pytest.raises(CommandError, match="early trigger"):
        run_home_axis(make_homing_info(), [10.0, 5.0])


def test_rehome_without_trigger_aborts():
    with pytest.raises(CommandError, match="did not trigger"):
        run_home_axis(make_homing_info(), [10.0, 50.0])


def test_non_sensorless_short_home_does_not_rehome():
    hi = make_homing_info(use_sensorless_homing=False)
    homing, toolhead, trip = run_home_axis(hi, [10.0])
    assert len(trip.calls) == 1
    assert homing._homing_axes == [0]


def test_zero_min_home_dist_disables_rehome():
    hi = make_homing_info(min_home_dist=0.0)
    homing, toolhead, trip = run_home_axis(hi, [1.0])
    assert len(trip.calls) == 1


def test_final_retract_absorbs_overshoot():
    homing, toolhead, trip = run_home_axis(
        make_homing_info(), [50.0], overshoot=0.12
    )
    clean_coordinate_retract = ([5.0, 0.0, 0.0, 0.0], 10.0)
    assert toolhead.moves[-1] == clean_coordinate_retract
    assert toolhead.get_position()[0] == 5.0
