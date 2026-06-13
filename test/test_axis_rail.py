import pytest

from klippy import stepper


class FakeError(Exception):
    pass


class FakePinParams:
    def __init__(self, pin, chip):
        self.pin = pin
        self.chip = chip

    def __getitem__(self, key):
        return {"pin": self.pin, "invert": False, "chip": self.chip}[key]


class FakeMCUEndstop:
    def __init__(self, pin):
        self.pin = pin
        self.steppers = []

    def add_stepper(self, stepper):
        self.steppers.append(stepper)


class FakePins:
    def __init__(self, chip):
        self.chip = chip

    def lookup_pin(self, pin, can_invert=False, can_pullup=False):
        return FakePinParams(pin, self.chip)

    def setup_pin(self, pin_type, pin_desc):
        assert pin_type == "endstop"
        return FakeMCUEndstop(pin_desc)


class FakeMCU:
    def __init__(self, printer):
        self._printer = printer
        self._oid = 0

    def create_oid(self):
        self._oid += 1
        return self._oid

    def register_config_callback(self, cb):
        pass

    def get_printer(self):
        return self._printer


class FakeRegistrar:
    def register_stepper(self, config, mcu_stepper):
        pass


class FakePrinter:
    def __init__(self):
        self.mcu = FakeMCU(self)
        self.pins = FakePins(self.mcu)
        self._objects = {"pins": self.pins}

    def lookup_object(self, name):
        return self._objects[name]

    def load_object(self, config, name):
        return self._objects.setdefault(name, FakeRegistrar())

    config_error = FakeError


_UNSET = object()


class FakeConfig:
    def __init__(self, printer, name, values):
        self._printer = printer
        self._name = name
        self._values = values
        self.error = FakeError

    def get_name(self):
        return self._name

    def get_printer(self):
        return self._printer

    def _raw(self, option, default):
        if option in self._values:
            return self._values[option]
        if default is _UNSET:
            raise FakeError(
                "Option '%s' missing in [%s]" % (option, self._name)
            )
        return default

    def get(self, option, default=_UNSET, note_valid=True):
        return self._raw(option, default)

    def getfloat(
        self,
        option,
        default=_UNSET,
        minval=None,
        maxval=None,
        above=None,
        below=None,
        note_valid=True,
    ):
        val = self._raw(option, default)
        if val is None:
            return None
        return float(val)

    def getint(self, option, default=_UNSET, minval=None, note_valid=True):
        val = self._raw(option, default)
        if val is None:
            return None
        return int(val)

    def getboolean(self, option, default=_UNSET, note_valid=True):
        val = self._raw(option, default)
        if val is None or isinstance(val, bool):
            return val
        return str(val).strip().lower() in ("1", "true", "yes", "on")

    def getlists(
        self,
        option,
        default=_UNSET,
        seps=(",",),
        count=None,
        parser=str,
        note_valid=True,
    ):
        return self._raw(option, default)


def make_axis_rail(axis_values, motor_values):
    printer = FakePrinter()
    axis_name = axis_values.pop("__name__")
    axis_config = FakeConfig(printer, axis_name, axis_values)
    motor_configs = []
    for mv in motor_values:
        name = mv.pop("__name__")
        motor_configs.append(FakeConfig(printer, name, mv))
    return stepper.AxisRail(axis_config, motor_configs)


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
        [motor_section("motor_a")],
    )
    assert rail.get_range() == (0.0, 300.0)
    assert rail.position_endstop == 0.0
    assert len(rail.get_steppers()) == 1
    assert rail.get_steppers()[0].get_name() == "motor_a"
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
            motor_section("motor_z0"),
            motor_section("motor_z1"),
            motor_section("motor_z2"),
        ],
    )
    assert len(rail.get_steppers()) == 3
    assert len(rail.get_endstops()) == 1


def test_motor_section_per_motor_endstop_override():
    rail = make_axis_rail(
        {
            "__name__": "axis z",
            "position_min": 0.0,
            "position_max": 200.0,
            "position_endstop": 0.5,
            "endstop_pin": "^PD3",
        },
        [
            motor_section("motor_z0"),
            motor_section("motor_z1", endstop_pin="^PD4"),
        ],
    )
    assert len(rail.get_steppers()) == 2
    assert len(rail.get_endstops()) == 2

    motor_z0, motor_z1 = rail.get_steppers()
    endstops_by_name = {name: mcu for mcu, name in rail.get_endstops()}
    assert set(endstops_by_name) == {"z", "motor_z1"}

    assert endstops_by_name["motor_z1"].steppers == [motor_z1]
    assert endstops_by_name["z"].steppers == [motor_z0]


def test_homing_keys_on_motor_section_rejected():
    with pytest.raises(FakeError):
        make_axis_rail(
            {
                "__name__": "axis x",
                "position_min": 0.0,
                "position_max": 300.0,
                "position_endstop": 0.0,
                "endstop_pin": "^PE5",
            },
            [motor_section("motor_a", position_min=0.0)],
        )
