import pytest

from klippy.extras.resonance_buzz import (
    ResonanceBuzz,
    buzz_axis_to_motor_mask,
)


def test_corexy_x_drives_both_motors_in_phase():
    axis_mask, sign_mask = buzz_axis_to_motor_mask("x", coupled=True)
    assert axis_mask == 0b011
    assert sign_mask == 0b000


def test_corexy_y_drives_both_motors_anti_phase():
    axis_mask, sign_mask = buzz_axis_to_motor_mask("y", coupled=True)
    assert axis_mask == 0b011
    assert sign_mask == 0b010


def test_corexy_z_is_single_slot():
    assert buzz_axis_to_motor_mask("z", coupled=True) == (0b100, 0b000)


def test_cartesian_axes_map_one_to_one():
    assert buzz_axis_to_motor_mask("x", coupled=False) == (0b001, 0b000)
    assert buzz_axis_to_motor_mask("y", coupled=False) == (0b010, 0b000)
    assert buzz_axis_to_motor_mask("z", coupled=False) == (0b100, 0b000)


def test_case_insensitive():
    assert buzz_axis_to_motor_mask(
        "X", coupled=True
    ) == buzz_axis_to_motor_mask("x", coupled=True)


def test_unsupported_axis_raises():
    with pytest.raises(ValueError, match="unsupported buzz axis"):
        buzz_axis_to_motor_mask("e", coupled=False)


class FakeBuzzToolhead:
    def get_kinematics(self):
        return object()

    def wait_moves(self):
        pass


class FakeBuzzMotion:
    def __init__(self):
        self.calls = []

    def submit_resonance_buzz(self, *args):
        self.calls.append(args)


class FakeBuzzReactor:
    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        pass


class FakeBuzzPrinter:
    def __init__(self):
        self.motion = FakeBuzzMotion()
        self._objs = {"toolhead": FakeBuzzToolhead(), "motion": self.motion}

    def lookup_object(self, name, default=None):
        return self._objs.get(name, default)

    def get_reactor(self):
        return FakeBuzzReactor()


class FakeBuzzGcmd:
    error = RuntimeError

    def __init__(self):
        self.infos = []

    def respond_info(self, msg):
        self.infos.append(msg)


def _resonance_buzz():
    buzz = ResonanceBuzz.__new__(ResonanceBuzz)
    buzz.printer = FakeBuzzPrinter()
    return buzz


def test_over_ceiling_accel_per_hz_fails_loud_instead_of_clamping():
    buzz = _resonance_buzz()
    with pytest.raises(RuntimeError, match="largest ACCEL_PER_HZ"):
        buzz.run_sweep(
            FakeBuzzGcmd(), "x", 100.0, 400.0, 300.0, 0.1, 600.0, 0.0
        )
    assert buzz.printer.motion.calls == []


def test_accel_per_hz_at_ceiling_runs():
    buzz = _resonance_buzz()
    amplitude = buzz.run_sweep(
        FakeBuzzGcmd(), "x", 100.0, 400.0, 300.0, 0.1, 500.0, 0.0
    )
    assert buzz.printer.motion.calls
    assert amplitude > 0.0
