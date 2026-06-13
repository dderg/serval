import pytest

from klippy import gcode
from klippy.kinematics import extruder as extruder_mod
from klippy.motion import BridgeKinematics, Motion, ToolheadShim

EXPECTED_STATUS_KEYS = {
    "homed_axes",
    "axis_minimum",
    "axis_maximum",
    "print_time",
    "stalls",
    "estimated_print_time",
    "extruder",
    "position",
    "max_velocity",
    "max_accel",
    "minimum_cruise_ratio",
    "square_corner_velocity",
}

LEGACY_METHODS = [
    "move",
    "manual_move",
    "dwell",
    "wait_moves",
    "wait_moves_and_mcu",
    "get_last_move_time",
    "get_position",
    "set_position",
    "flush_step_generation",
    "get_status",
    "check_busy",
    "stats",
    "get_kinematics",
    "get_max_velocity",
    "get_extruder",
    "set_extruder",
    "register_step_generator",
    "register_lookahead_callback",
    "note_step_generation_scan_time",
    "note_mcu_movequeue_activity",
    "limit_next_junction_speed",
    "get_trapq",
]

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

    toolhead = Motion.__new__(Motion)
    toolhead.printer = printer
    toolhead.kin = kin
    toolhead.mcu = FakeMcu()
    toolhead.Coord = gcode.Coord
    toolhead.commanded_pos = [0.0, 0.0, 0.0, 0.0]
    toolhead.print_time = 0.0
    toolhead.print_stall = 0
    toolhead.extruder = extruder_mod.DummyExtruder(printer)
    toolhead.max_velocity = 300.0
    toolhead.max_accel = 3000.0
    toolhead.min_cruise_ratio = 0.0
    toolhead.square_corner_velocity = 5.0

    printer.add_object("toolhead", ToolheadShim(toolhead))
    return printer


def test_toolhead_status_keys_exact(toolhead_fixture):
    toolhead = toolhead_fixture.lookup_object("toolhead")
    status = toolhead.get_status(EVENTTIME)
    assert set(status.keys()) == EXPECTED_STATUS_KEYS


def test_toolhead_method_surface_complete(toolhead_fixture):
    toolhead = toolhead_fixture.lookup_object("toolhead")
    missing = [
        m for m in LEGACY_METHODS if not callable(getattr(toolhead, m, None))
    ]
    assert missing == []
