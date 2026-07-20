import pytest
from fakes import FakeConfig, FakeConfigError, FakeMcu, FakePrinter

from klippy import stepper


class FakePinParams:
    def __init__(self, pin, chip):
        self.pin = pin
        self.chip = chip

    def __getitem__(self, key):
        return {"pin": self.pin, "invert": False, "chip": self.chip}[key]


class FakePins:
    def __init__(self, chip):
        self.chip = chip

    def lookup_pin(self, pin, can_invert=False, can_pullup=False):
        return FakePinParams(pin, self.chip)


class FakeRegistrar:
    def register_stepper(self, config, mcu_stepper):
        pass


class LocalFakePrinter(FakePrinter):
    def __init__(self):
        super().__init__()
        self.mcu = FakeMcu(printer=self)
        self.pins = FakePins(self.mcu)
        self.objects["pins"] = self.pins

    def load_object(self, config, name, default=None):
        return self.objects.setdefault(name, FakeRegistrar())


def make_axis_rail(axis_values, motor_values):
    printer = LocalFakePrinter()
    axis_name = axis_values.pop("__name__")
    axis_config = FakeConfig(printer, axis_name, axis_values)
    motor_specs = []
    for mv in motor_values:
        name = mv.pop("__name__")
        short = name.split(None, 1)[1]
        motor_specs.append((FakeConfig(printer, name, mv), short))
    return stepper.AxisRail(axis_config, motor_specs)


def motor_section(name, endstop_pin=None, **extra):
    values = {
        "__name__": name,
        "step_pin": "PF0",
        "dir_pin": "PF1",
        "rotation_distance": 40.0,
        "microsteps": 16,
    }
    if endstop_pin is not None:
        values["endstop_pin"] = endstop_pin
    values.update(extra)
    return values


def test_axis_rail_reads_range_from_axis_section():
    rail = make_axis_rail(
        {
            "__name__": "axis x",
            "position_min": 0.0,
            "position_max": 300.0,
            "position_endstop": 0.0,
            "endstop_pin": "^PE5",
            "homing_speed": 50.0,
        },
        [motor_section("motor a")],
    )
    assert rail.get_range() == (0.0, 300.0)
    assert rail.position_endstop == 0.0
    assert len(rail.get_steppers()) == 1
    assert rail.get_steppers()[0].get_name() == "a"
    hi = rail.get_homing_info()
    assert hi.speed == 50.0
    assert hi.positive_dir is False


def test_axis_rail_multiple_motors_lockstep():
    rail = make_axis_rail(
        {
            "__name__": "axis z",
            "position_min": 0.0,
            "position_max": 200.0,
            "position_endstop": 0.5,
            "endstop_pin": "^PD3",
        },
        [
            motor_section("motor z0"),
            motor_section("motor z1"),
            motor_section("motor z2"),
        ],
    )
    assert len(rail.get_steppers()) == 3


def test_axis_rail_defers_endstops_to_central_homing():
    rail = make_axis_rail(
        {
            "__name__": "axis x",
            "position_min": 0.0,
            "position_max": 300.0,
            "position_endstop": 0.0,
            "endstop_pin": "^PE5",
        },
        [motor_section("motor a")],
    )
    assert rail.get_endstops() == []
    assert rail.endstop_pin == "^PE5"


def test_homing_keys_on_motor_section_rejected():
    with pytest.raises(FakeConfigError):
        make_axis_rail(
            {
                "__name__": "axis x",
                "position_min": 0.0,
                "position_max": 300.0,
                "position_endstop": 0.0,
                "endstop_pin": "^PE5",
            },
            [motor_section("motor a", position_min=0.0)],
        )
