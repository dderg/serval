import pytest

from klippy.extras import servo_axis, servo_sync


class FakeReactor:
    def __init__(self):
        self._t = 0.0

    def monotonic(self):
        self._t += 0.001
        return self._t

    def pause(self, until):
        self._t = until


class FakeGcode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, func, desc=None):
        self.commands[name] = func


class FakeToolhead:
    def __init__(self, kin):
        self._kin = kin
        self.wait_moves_calls = 0

    def get_kinematics(self):
        return self._kin

    def wait_moves(self):
        self.wait_moves_calls += 1


class FakeKin:
    def __init__(self, rails, lane_names):
        self.rails = rails
        self._lane_names = lane_names

    def lanes(self):
        return [(i, name, []) for i, name in enumerate(self._lane_names)]


class FakeNode:
    def __init__(self, name, handle, slots):
        self.name = name
        self._handle = handle
        self._slots = slots

    def get_engine_handle(self):
        return self._handle

    def get_slot_for_motor(self, motor_name):
        return self._slots[motor_name]


OK_REPORT = (0, 0, 1, 80, -78, 40, 3, 2, -1, 512)


class FakeEngine:
    def __init__(self, scripted=None):
        self.calls = []
        self._scripted = list(scripted or [])

    def motion_drained(self):
        return True

    def sync_servo_pair(self, handle, axis, *tuning):
        self.calls.append((handle, axis) + tuning)
        if self._scripted:
            return self._scripted.pop(0)
        return OK_REPORT


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs
        self._reactor = FakeReactor()

    def lookup_object(self, name):
        return self._objs[name]

    def get_reactor(self):
        return self._reactor

    def is_shutdown(self):
        return False


class FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer

    def getfloat(self, name, default, **kw):
        return default

    def getint(self, name, default, **kw):
        return default


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def get_float(self, name, default=None):
        return float(self._params.get(name, default))

    def get_int(self, name, default=None, minval=None, maxval=None):
        return int(self._params.get(name, default))

    def respond_info(self, msg):
        self.responses.append(msg)


def make_motor(name, node_name, slot_ignored=None):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = name
    m.node_name = node_name
    m.encoder_counts_per_rev = 131072
    m.rotation_distance = 40.0
    return m


def make_rail(axis, motor_names, node_name="xy_drives"):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis " + axis
    rail.axis = axis
    rail.motors = [make_motor(n, node_name) for n in motor_names]
    return rail


def make_sync(engine=None, rails=None, lane_names=("x", "y")):
    engine = engine or FakeEngine()
    if rails is None:
        rails = [
            make_rail("x", ["motor_a", "motor_a1"]),
            make_rail("y", ["motor_b", "motor_b1"]),
        ]
    node = FakeNode(
        "xy_drives",
        7,
        {"motor_a": 0, "motor_a1": 1, "motor_b": 2, "motor_b1": 3},
    )
    kin = FakeKin(rails, lane_names)
    toolhead = FakeToolhead(kin)
    printer = FakePrinter(
        {
            "toolhead": toolhead,
            "motion_engine": engine,
            "ethercat_node xy_drives": node,
            "gcode": FakeGcode(),
        }
    )
    ss = servo_sync.ServoSync(FakeConfig(printer))
    return ss, engine, toolhead


def slot_reports(primary, secondary, result=0):
    return (result, primary, secondary, 80, -78, 40, 3, 2, -1, 512)


def test_syncs_every_belt_pair_twice_with_converted_units():
    ss, engine, toolhead = make_sync(
        engine=FakeEngine(
            [
                slot_reports(0, 1),
                slot_reports(1, 0),
                slot_reports(2, 3),
                slot_reports(3, 2),
            ]
        )
    )
    gcmd = FakeGcmd()
    ss.cmd_SERVO_SYNC(gcmd)
    assert toolhead.wait_moves_calls == 1
    assert engine.calls == [
        (7, 0, 30, 2000, 250_000, 4000, 1500, False),
        (7, 0, 30, 2000, 250_000, 4000, 1500, True),
        (7, 1, 30, 2000, 250_000, 4000, 1500, False),
        (7, 1, 30, 2000, 250_000, 4000, 1500, True),
    ]
    assert len(gcmd.responses) == 4
    assert "motor_a holds" in gcmd.responses[0]
    assert "motor_a1 re-seeded" in gcmd.responses[0]
    assert "released 512 counts" in gcmd.responses[0]
    assert "motor_a1 holds" in gcmd.responses[1]
    assert "motor_a re-seeded" in gcmd.responses[1]
    assert "motor_b1 re-seeded" in gcmd.responses[2]


def test_both_zero_keeps_the_single_pass_behavior():
    ss, engine, _ = make_sync(
        engine=FakeEngine([slot_reports(0, 1), slot_reports(2, 3)])
    )
    ss.cmd_SERVO_SYNC(FakeGcmd(BOTH="0"))
    assert [c[-1] for c in engine.calls] == [False, False]
    assert [c[1] for c in engine.calls] == [0, 1]


def test_axis_filter_limits_to_one_lane():
    ss, engine, _ = make_sync(
        engine=FakeEngine([slot_reports(2, 3), slot_reports(3, 2)])
    )
    ss.cmd_SERVO_SYNC(FakeGcmd(AXIS="Y"))
    assert [c[1] for c in engine.calls] == [1, 1]


def test_gcode_overrides_reach_the_engine():
    ss, engine, _ = make_sync(
        engine=FakeEngine(
            [
                slot_reports(0, 1),
                slot_reports(1, 0),
                slot_reports(2, 3),
                slot_reports(3, 2),
            ]
        )
    )
    ss.cmd_SERVO_SYNC(
        FakeGcmd(TORQUE_OK="5.0", AMPLITUDE="0.05", FREQ="8", DURATION="0.25")
    )
    assert engine.calls[0][2:] == (50, 2000, 50_000, 8000, 250, False)


def test_residual_final_fight_earns_one_more_round():
    engine = FakeEngine(
        [
            slot_reports(0, 1, result=-846),
            slot_reports(1, 0),
            slot_reports(2, 3),
            slot_reports(3, 2),
            slot_reports(0, 1),
            slot_reports(1, 0),
            slot_reports(2, 3),
            slot_reports(3, 2),
        ]
    )
    ss, engine, _ = make_sync(engine=engine)
    gcmd = FakeGcmd()
    ss.cmd_SERVO_SYNC(gcmd)
    assert len(engine.calls) == 8
    assert any("running another" in r for r in gcmd.responses)


def test_still_fighting_after_max_rounds_errors():
    engine = FakeEngine(
        [
            slot_reports(0, 1, result=-846),
            slot_reports(1, 0),
            slot_reports(2, 3, result=-846),
            slot_reports(3, 2),
        ]
        * 4
    )
    ss, engine, _ = make_sync(engine=engine)
    with pytest.raises(RuntimeError, match="still fighting"):
        ss.cmd_SERVO_SYNC(FakeGcmd())
    assert len(engine.calls) == 8


def test_hard_failure_stops_immediately_with_measurements():
    engine = FakeEngine(
        [
            slot_reports(0, 1, result=-844),
            slot_reports(1, 0),
            slot_reports(2, 3),
            slot_reports(3, 2),
        ]
    )
    ss, engine, _ = make_sync(engine=engine)
    gcmd = FakeGcmd()
    with pytest.raises(RuntimeError, match="see measurements"):
        ss.cmd_SERVO_SYNC(gcmd)
    assert len(engine.calls) == 4
    assert any("settle timeout" in r for r in gcmd.responses)


def test_z_axis_is_rejected_loudly():
    rails = [
        make_rail("x", ["motor_a", "motor_a1"]),
        make_rail("z", ["motor_z", "motor_z1"]),
    ]
    ss, _, _ = make_sync(rails=rails, lane_names=("x", "z"))
    with pytest.raises(RuntimeError, match="racking"):
        ss.cmd_SERVO_SYNC(FakeGcmd(AXIS="Z"))


def test_single_drive_axes_are_not_syncable():
    rails = [make_rail("x", ["motor_a"]), make_rail("y", ["motor_b"])]
    ss, _, _ = make_sync(rails=rails)
    with pytest.raises(RuntimeError, match="no belt axis"):
        ss.cmd_SERVO_SYNC(FakeGcmd())
