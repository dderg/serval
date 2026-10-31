import pytest
from fakes import FakeEngine as _FakeEngine
from fakes import FakeKin, FakeMcu, FakePrinter, FakeReactor

from klippy import gcode
from klippy.kinematics import extruder as extruder_mod
from klippy.motion import Motion, ToolheadShim

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
    "corner_deviation",
}

LEGACY_METHODS = [
    "move",
    "manual_move",
    "dwell",
    "wait_moves",
    "get_last_move_time",
    "get_position",
    "set_position",
    "flush_step_generation",
    "get_status",
    "check_busy",
    "stats",
    "get_kinematics",
    "get_extruder",
    "set_extruder",
    "register_lookahead_callback",
    "note_mcu_movequeue_activity",
]

EVENTTIME = 100.0


@pytest.fixture
def toolhead_fixture():
    printer = FakePrinter()

    kin = FakeKin(get_status_ranges=[(0.0, 200.0), (0.0, 200.0), (0.0, 250.0)])

    toolhead = Motion.__new__(Motion)
    toolhead.printer = printer
    toolhead.kin = kin
    toolhead.mcu = FakeMcu(print_time_offset=1.0)
    toolhead.Coord = gcode.Coord
    toolhead.commanded_pos = [0.0, 0.0, 0.0, 0.0]
    toolhead.print_time = 0.0
    toolhead.print_stall = 0
    toolhead.extruder = extruder_mod.DummyExtruder(printer)
    toolhead._max_velocity = 300.0
    toolhead._max_accel = 3000.0
    toolhead._corner_deviation = 0.0034517796864424596
    toolhead._planner_ready = False

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


def _make_correction_toolhead():
    th = Motion.__new__(Motion)
    th.mcu = FakeMcu(print_time_offset=1.0)
    th.all_mcus = [th.mcu]
    th.kin = None
    th.reactor = FakeReactor(now=100.0)
    th.engine = _FakeEngine(
        motion_lead_secs=0.25,
        fence_start=1,
        fence_print_time_poll=0.0,
    )
    th.printer = FakePrinter(reactor=th.reactor)
    th.motion_lead = 0.25
    th._engine_wakeup = None
    return th


def test_get_last_move_time_uses_motion_lead():
    th = _make_correction_toolhead()
    th.motion_lead = 0.5
    assert th.get_last_move_time() == pytest.approx(101.5)
