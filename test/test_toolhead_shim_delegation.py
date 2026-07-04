import pytest

from klippy import gcode
from klippy.kinematics import extruder as extruder_mod
from klippy.motion import Motion, ToolheadShim

EVENTTIME = 100.0


class FakeKin:
    def __init__(self, ranges):
        self._ranges = ranges
        self.limits = [(1.0, -1.0)] * 3

    def get_status(self, eventtime):
        from klippy import gcode as gcode_mod

        (x_min, x_max), (y_min, y_max), (z_min, z_max) = self._ranges
        homed = "".join(
            a
            for i, a in enumerate("xyz")
            if self.limits[i][0] <= self.limits[i][1]
        )
        return {
            "homed_axes": homed,
            "axis_minimum": gcode_mod.Coord(x_min, y_min, z_min, 0.0),
            "axis_maximum": gcode_mod.Coord(x_max, y_max, z_max, 0.0),
        }


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

    kin = FakeKin([(0.0, 200.0), (0.0, 200.0), (0.0, 250.0)])

    motion = Motion.__new__(Motion)
    motion.printer = printer
    motion.kin = kin
    motion.mcu = FakeMcu()
    motion.Coord = gcode.Coord
    motion.commanded_pos = [0.0, 0.0, 0.0, 0.0]
    motion.print_time = 0.0
    motion._mcu_pending_end_time = 0.0
    motion.print_stall = 0
    motion.extruder = extruder_mod.DummyExtruder(printer)
    motion._max_velocity = 300.0
    motion._max_accel = 3000.0
    motion.min_cruise_ratio = 0.0
    motion._square_corner_velocity = 5.0
    motion._planner_ready = False

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
        "note_step_generation_scan_time",
        "get_trapq",
        "note_mcu_movequeue_activity",
        "limit_next_junction_speed",
    ):
        assert not hasattr(motion, fossil)
        assert callable(getattr(shim, fossil))
    # register_lookahead_callback graduated from fossil to a real Motion
    # method (fence-backed); the shim only delegates it.
    assert callable(motion.register_lookahead_callback)
    assert callable(shim.register_lookahead_callback)


def test_shim_delegates_state(toolhead_fixture):
    printer = toolhead_fixture
    shim = printer.lookup_object("toolhead")
    motion = printer.lookup_object("motion")
    assert shim.get_position() == motion.get_position()
    assert shim.get_status(EVENTTIME) == motion.get_status(EVENTTIME)
