import pytest

from klippy.extras import extruder_stepper
from klippy.kinematics.extruder import PrinterExtruder


class ConfigError(Exception):
    pass


_UNSET = object()


class StubSection:
    error = ConfigError

    def __init__(self, name, options):
        self.name = name
        self.options = options

    def get_name(self):
        return self.name

    def get(self, key, default=_UNSET, note_valid=True):
        if key in self.options:
            return self.options[key]
        if default is _UNSET:
            raise ConfigError("missing option '%s' in [%s]" % (key, self.name))
        return default

    def getfloat(self, key, default=_UNSET, **kwargs):
        val = self.get(key, default)
        return None if val is None else float(val)

    def get_printer(self):
        return self.printer


class FakeHeater:
    can_extrude = True

    def get_status(self, eventtime):
        return {}


class FakeHeaters:
    def setup_heater(self, config, gcode_id=None):
        return FakeHeater()


class FakeGcode:
    def register_command(self, *a, **k):
        pass

    def register_mux_command(self, *a, **k):
        pass


class FakeToolhead:
    def __init__(self):
        self.extruder = None

    def get_max_velocity(self):
        return 300.0, 3000.0

    def set_extruder(self, extruder, pos):
        self.extruder = extruder


class FakePrinter:
    command_error = ConfigError

    def __init__(self):
        self.objects = {
            "heaters": FakeHeaters(),
            "gcode": FakeGcode(),
            "toolhead": FakeToolhead(),
        }

    def load_object(self, config, name):
        return self.objects[name]

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


def make_extruder_section(name="extruder", **options):
    printer = FakePrinter()
    base = {
        "nozzle_diameter": 0.4,
        "filament_diameter": 1.75,
    }
    base.update(options)
    section = StubSection(name, base)
    section.printer = printer
    return section


def test_extruder_section_with_step_pin_rejected():
    section = make_extruder_section(step_pin="PE2")
    with pytest.raises(ConfigError, match=r"\[<motor>\] section"):
        PrinterExtruder(section, 0)


def test_extruder_section_with_rotation_distance_rejected():
    section = make_extruder_section(rotation_distance=22.0)
    with pytest.raises(ConfigError, match=r"\[axis e\] motors"):
        PrinterExtruder(section, 0)


def test_extruder_heater_only_section_loads():
    section = make_extruder_section()
    pe = PrinterExtruder(section, 0)
    assert pe.extruder_stepper is None
    assert pe.get_heater() is not None
    # def_max_cross_section = 4 * 0.4**2 = 0.64; filament_area = pi*0.875**2.
    expected_area = 3.141592653589793 * (1.75 * 0.5) ** 2
    assert pe.max_extrude_ratio == pytest.approx((4.0 * 0.4**2) / expected_area)


class FakeMove:
    def __init__(self, axes_d, axes_r):
        self.axes_d = axes_d
        self.axes_r = axes_r
        self.move_d = (axes_d[0] ** 2 + axes_d[1] ** 2) ** 0.5


def test_extruder_check_move_overextrusion_fires():
    section = make_extruder_section()
    pe = PrinterExtruder(section, 0)
    # XY move of 1mm with extrusion ratio far above max_extrude_ratio.
    bad_ratio = pe.max_extrude_ratio * 10.0
    move = FakeMove(
        axes_d=[1.0, 0.0, 0.0, 1.0], axes_r=[1.0, 0.0, 0.0, bad_ratio]
    )
    with pytest.raises(ConfigError, match="maximum extrusion"):
        pe.check_move(move)


def test_extruder_stepper_extra_rejected():
    config = StubSection("extruder_stepper foo", {})
    with pytest.raises(ConfigError, match=r"\[<motor>\] section"):
        extruder_stepper.load_config_prefix(config)
