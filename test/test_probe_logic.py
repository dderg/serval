import pytest

from klippy import pins
from klippy.extras.probe import (
    calc_probe_z_result,
    validate_virtual_endstop_request,
)


def _pin_params(pin="z_virtual_endstop", invert=0, pullup=0):
    return {
        "chip": object(),
        "chip_name": "probe",
        "pin": pin,
        "invert": invert,
        "pullup": pullup,
    }


def test_average():
    assert calc_probe_z_result([1.0, 2.0, 6.0], "average") == pytest.approx(3.0)


def test_median_odd():
    assert calc_probe_z_result([5.0, 1.0, 2.0], "median") == 2.0


def test_median_even_averages_middle_pair():
    assert calc_probe_z_result([4.0, 1.0, 2.0, 3.0], "median") == pytest.approx(
        2.5
    )


def test_unknown_method_raises():
    with pytest.raises(ValueError):
        calc_probe_z_result([1.0], "mode")


def test_valid_virtual_endstop_request_passes():
    validate_virtual_endstop_request(_pin_params(), 2)


def test_wrong_pin_name_rejected():
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(pin="virtual_endstop"), 2)


def test_modifiers_rejected():
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(pullup=1), 2)
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(invert=1), 2)


def test_non_z_axis_rejected():
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(), 0)


class _FakeGCmd:
    def __init__(self, params=None):
        self._params = params or {}

    def get_float(self, name, default=None, above=None, minval=None):
        return float(self._params.get(name, default))


def test_get_lift_speed_returns_config_value_without_gcmd():
    from klippy.extras.probe import PrinterProbe

    probe = PrinterProbe.__new__(PrinterProbe)
    probe.lift_speed = 7.5
    assert probe.get_lift_speed() == 7.5


def test_get_lift_speed_honors_gcmd_override():
    from klippy.extras.probe import PrinterProbe

    probe = PrinterProbe.__new__(PrinterProbe)
    probe.lift_speed = 7.5
    assert probe.get_lift_speed(_FakeGCmd({"LIFT_SPEED": 3.0})) == 3.0
    assert probe.get_lift_speed(_FakeGCmd()) == 7.5


def test_multi_probe_lifecycle_is_noop():
    from klippy.extras.probe import PrinterProbe

    probe = PrinterProbe.__new__(PrinterProbe)
    assert probe.multi_probe_begin() is None
    assert probe.multi_probe_end() is None


class _ProbeError(RuntimeError):
    pass


class _ProbeGCmd:
    def error(self, msg):
        return _ProbeError(msg)


class _FakeRail:
    def get_range(self):
        return (-2.0, 200.0)


class _FakeKin:
    def _axis_rails(self):
        return {2: _FakeRail()}


class _FakeToolhead:
    def __init__(self, z):
        self._z = z

    def get_kinematics(self):
        return _FakeKin()

    def get_position(self):
        return [0.0, 0.0, self._z, 0.0]

    def set_position(self, newpos):
        self._z = newpos[2]


def _probe_with_trip(trip_z, final_z):
    from klippy.extras.probe import PrinterProbe

    probe = PrinterProbe.__new__(PrinterProbe)
    probe._endstop = object()

    class _Homing:
        def trip_move(self, gcmd, toolhead, engine, axis, *_args):
            return [0.0, 0.0, trip_z], [0.0, 0.0, final_z]

    toolhead = _FakeToolhead(z=10.0)
    return probe._probe_once(_ProbeGCmd(), toolhead, _Homing(), object(), 5.0)


def test_probe_rejects_trigger_prior_to_movement():
    with pytest.raises(_ProbeError, match="prior to movement"):
        _probe_with_trip(trip_z=10.0, final_z=10.0)


def test_probe_returns_trip_height_after_real_movement():
    assert _probe_with_trip(trip_z=3.5, final_z=3.4) == pytest.approx(3.5)
