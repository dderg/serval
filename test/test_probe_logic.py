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
