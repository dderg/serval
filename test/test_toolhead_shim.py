import pytest

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
    "register_lookahead_callback",
    "note_step_generation_scan_time",
    "note_mcu_movequeue_activity",
    "limit_next_junction_speed",
    "get_trapq",
]

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


class _RecordingBridge:
    def __init__(self, duration):
        self._duration = duration
        self.last_call = None

    def motion_lead_secs(self):
        return 0.25

    def submit_correction_sequence(
        self,
        mcu_id,
        axis_idx,
        motor_idx,
        segments,
        speed,
        accel,
        start_host_secs,
    ):
        self.last_call = dict(
            kind="correction",
            mcu_id=mcu_id,
            axis_idx=axis_idx,
            motor_idx=motor_idx,
            segments=list(segments),
            speed=speed,
            accel=accel,
            start_host_secs=start_host_secs,
        )
        return self._duration

    def adjust_motor(
        self,
        mcu_id,
        axis_idx,
        motor_idx,
        delta_mm,
        speed,
        accel,
        start_host_secs,
    ):
        self.last_call = dict(
            kind="adjust",
            mcu_id=mcu_id,
            axis_idx=axis_idx,
            motor_idx=motor_idx,
            delta_mm=delta_mm,
            speed=speed,
            accel=accel,
            start_host_secs=start_host_secs,
        )
        return self._duration


class _FixedReactor:
    def monotonic(self):
        return 100.0


def _make_correction_toolhead(duration):
    th = Motion.__new__(Motion)
    th.mcu = FakeMcu()  # estimated_print_time(t) = t + 1.0
    th.reactor = _FixedReactor()
    th.bridge = _RecordingBridge(duration)
    th.motion_lead = 0.25
    th._mcu_pending_end_time = 0.0
    return th


def test_get_last_move_time_uses_motion_lead():
    th = _make_correction_toolhead(0.0)
    th.motion_lead = 0.5
    # est = 100 + 1 = 101; floor = est + 0.5 = 101.5; pending 0 < est -> floor
    assert th.get_last_move_time() == pytest.approx(101.5)


def test_submit_correction_anchors_on_timeline_and_advances_pending():
    th = _make_correction_toolhead(0.6)
    # idle: glmt = est(101.0) + lead(0.25) = 101.25
    # start_host = now + (glmt - est) = 100 + 0.25 = 100.25
    wait_s = th.submit_correction(7, 1, 0, [0.3, -0.3], 80.0, 5000.0)
    call = th.bridge.last_call
    assert call["kind"] == "correction"
    assert (
        call["mcu_id"] == 7 and call["axis_idx"] == 1 and call["motor_idx"] == 0
    )
    assert call["start_host_secs"] == pytest.approx(100.25)
    # pending advanced past the buzz: glmt + duration = 101.25 + 0.6 = 101.85
    assert th._mcu_pending_end_time == pytest.approx(101.85)
    # caller wait = (start_host - now) + duration = 0.25 + 0.6 = 0.85
    assert wait_s == pytest.approx(0.85)


def test_submit_motor_adjust_anchors_on_timeline_and_advances_pending():
    th = _make_correction_toolhead(0.6)
    wait_s = th.submit_motor_adjust(2, 0, 1, 0.05, 5.0, 100.0)
    call = th.bridge.last_call
    assert call["kind"] == "adjust"
    assert (
        call["mcu_id"] == 2 and call["axis_idx"] == 0 and call["motor_idx"] == 1
    )
    assert call["delta_mm"] == pytest.approx(0.05)
    assert call["start_host_secs"] == pytest.approx(100.25)
    assert th._mcu_pending_end_time == pytest.approx(101.85)
    assert wait_s == pytest.approx(0.85)
