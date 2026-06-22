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
    def __init__(self, can_extrude=True):
        self.can_extrude = can_extrude

    def get_status(self, eventtime):
        return {}


class FakeHeaters:
    def __init__(self, can_extrude=True):
        self._can_extrude = can_extrude

    def setup_heater(self, config, gcode_id=None):
        return FakeHeater(self._can_extrude)


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


class FakeMotion:
    def __init__(self, axis_sections):
        self.axis_sections = axis_sections


class FakePrinter:
    command_error = ConfigError

    def __init__(self, axis_sections=None, can_extrude=True):
        if axis_sections is None:
            axis_sections = [
                ("x", [], ["x"], []),
                ("y", [], ["y"], []),
                ("z", [], ["z"], []),
                ("e", ["x", "y", "z"], ["e"], []),
            ]
        self.objects = {
            "heaters": FakeHeaters(can_extrude),
            "gcode": FakeGcode(),
            "toolhead": FakeToolhead(),
            "motion": FakeMotion(axis_sections),
        }

    def load_object(self, config, name):
        return self.objects[name]

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


def make_extruder_section(
    name="extruder", axis="e", axis_sections=None, can_extrude=True, **options
):
    printer = FakePrinter(axis_sections, can_extrude)
    base = {
        "nozzle_diameter": 0.4,
        "filament_diameter": 1.75,
    }
    if axis is not None:
        base["axis"] = axis
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
    assert pe.get_heater() is not None
    assert pe.axis_name == "e"


def test_extruder_missing_axis_rejected():
    section = make_extruder_section(axis=None)
    with pytest.raises(ConfigError, match="axis"):
        PrinterExtruder(section, 0)


def test_extruder_axis_not_declared_rejected():
    section = make_extruder_section(axis="q")
    with pytest.raises(ConfigError, match="not a declared"):
        PrinterExtruder(section, 0)


def test_extruder_non_follower_axis_rejected():
    section = make_extruder_section(axis="x")
    with pytest.raises(ConfigError, match="must be a follower axis"):
        PrinterExtruder(section, 0)


def test_extruder_valid_follower_axis_loads():
    section = make_extruder_section(axis="e")
    pe = PrinterExtruder(section, 0)
    assert pe.axis_name == "e"


class FakeMove:
    def __init__(self, axes_d, axes_r):
        self.axes_d = axes_d
        self.axes_r = axes_r
        self.move_d = (axes_d[0] ** 2 + axes_d[1] ** 2) ** 0.5


def test_extruder_check_move_allows_heavy_extrusion():
    section = make_extruder_section()
    pe = PrinterExtruder(section, 0)
    # A move that the deleted over-extrusion guard would have rejected:
    # planner-side limits, not the extruder, govern motion now.
    move = FakeMove(axes_d=[1.0, 0.0, 0.0, 50.0], axes_r=[1.0, 0.0, 0.0, 50.0])
    pe.check_move(move)


def test_extruder_check_move_cold_extrude_rejected():
    section = make_extruder_section(can_extrude=False)
    pe = PrinterExtruder(section, 0)
    move = FakeMove(axes_d=[0.0, 0.0, 0.0, 1.0], axes_r=[0.0, 0.0, 0.0, 1.0])
    with pytest.raises(ConfigError, match="minimum temp"):
        pe.check_move(move)


@pytest.mark.parametrize(
    "key, value",
    [
        ("max_extrude_cross_section", 1.0),
        ("instantaneous_corner_velocity", 1.0),
        ("max_extrude_only_distance", 500.0),
    ],
)
def test_extruder_removed_option_rejected(key, value):
    section = make_extruder_section(**{key: value})
    with pytest.raises(ConfigError, match="no longer supported"):
        PrinterExtruder(section, 0)


@pytest.mark.parametrize(
    "key", ["max_extrude_only_velocity", "max_extrude_only_accel"]
)
def test_extruder_extrude_only_limit_is_read_into_attribute(key):
    section = make_extruder_section(**{key: 100.0})
    pe = PrinterExtruder(section, 0)
    assert getattr(pe, key) == 100.0


@pytest.mark.parametrize(
    "key", ["max_extrude_only_velocity", "max_extrude_only_accel"]
)
def test_extruder_extrude_only_limit_defaults_to_none(key):
    pe = PrinterExtruder(make_extruder_section(), 0)
    assert getattr(pe, key) is None


def test_extruder_stepper_extra_rejected():
    config = StubSection("extruder_stepper foo", {})
    with pytest.raises(ConfigError, match=r"\[<motor>\] section"):
        extruder_stepper.load_config_prefix(config)
