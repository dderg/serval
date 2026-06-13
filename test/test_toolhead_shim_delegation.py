import pytest

from klippy import gcode
from klippy.kinematics import extruder as extruder_mod
from klippy.motion import BridgeKinematics, Motion, ToolheadShim

EVENTTIME = 100.0


class FakeRail:
    def __init__(self, name, position_min, position_max):
        self._name = name
        self.position_min = position_min
        self.position_max = position_max

    def get_name(self, short=False):
        return self._name


class FakeMcu:
    def estimated_print_time(self, eventtime):
        return eventtime + 1.0


class FakePrinter:
    def __init__(self):
        self.objects = {}

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


@pytest.fixture
def toolhead_fixture():
    printer = FakePrinter()

    kin = BridgeKinematics.__new__(BridgeKinematics)
    kin.rails = [
        FakeRail("stepper_x", 0.0, 200.0),
        FakeRail("stepper_y", 0.0, 200.0),
        FakeRail("stepper_z", 0.0, 250.0),
    ]
    kin.limits = [(1.0, -1.0)] * 3

    motion = Motion.__new__(Motion)
    motion.printer = printer
    motion.kin = kin
    motion.mcu = FakeMcu()
    motion.Coord = gcode.Coord
    motion.commanded_pos = [0.0, 0.0, 0.0, 0.0]
    motion.print_time = 0.0
    motion.print_stall = 0
    motion.extruder = extruder_mod.DummyExtruder(printer)
    motion.max_velocity = 300.0
    motion.max_accel = 3000.0
    motion.min_cruise_ratio = 0.0
    motion.square_corner_velocity = 5.0

    printer.add_object("motion", motion)
    printer.add_object("toolhead", ToolheadShim(motion))
    return printer


def test_toolhead_is_shim_motion_is_real(toolhead_fixture):
    printer = toolhead_fixture
    shim = printer.lookup_object("toolhead")
    motion = printer.lookup_object("motion")
    assert shim is not motion
    assert shim.motion is motion


def test_fossil_methods_only_on_shim(toolhead_fixture):
    printer = toolhead_fixture
    motion = printer.lookup_object("motion")
    shim = printer.lookup_object("toolhead")
    for fossil in (
        "register_lookahead_callback",
        "note_step_generation_scan_time",
        "get_trapq",
        "note_mcu_movequeue_activity",
        "limit_next_junction_speed",
    ):
        assert not hasattr(motion, fossil)
        assert callable(getattr(shim, fossil))


def test_shim_delegates_state(toolhead_fixture):
    printer = toolhead_fixture
    shim = printer.lookup_object("toolhead")
    motion = printer.lookup_object("motion")
    assert shim.get_position() == motion.get_position()
    assert shim.get_status(EVENTTIME) == motion.get_status(EVENTTIME)
