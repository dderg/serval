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


OK_REPORT = (0, 0x0F, [80, -78, 40, -38], [3, -2, 1, -1], [512, -498, 60, -55])


class FakeEngine:
    def __init__(self, scripted=None):
        self.calls = []
        self._scripted = list(scripted or [])

    def motion_drained(self):
        return True

    def sync_servo_release(self, handle, slot_mask, *tuning):
        self.calls.append((handle, slot_mask) + tuning)
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


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def get_float(self, name, default=None):
        return float(self._params.get(name, default))

    def respond_info(self, msg):
        self.responses.append(msg)


def make_motor(name, node_name):
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


def failed_report(result):
    return (result,) + OK_REPORT[1:]


def test_releases_every_belt_drive_in_one_call_with_converted_units():
    ss, engine, toolhead = make_sync()
    gcmd = FakeGcmd()
    ss.cmd_SERVO_SYNC(gcmd)
    assert toolhead.wait_moves_calls == 1
    assert engine.calls == [(7, 0x0F, 30, 2000)]
    assert len(gcmd.responses) == 2
    assert "axis x released" in gcmd.responses[0]
    assert "motor_a +8.0% -> +0.3%" in gcmd.responses[0]
    assert "motor_a1 -7.8% -> -0.2%" in gcmd.responses[0]
    assert "rotor moved +0.1562 mm" in gcmd.responses[0]
    assert "axis y released" in gcmd.responses[1]
    assert "motor_b +4.0% -> +0.1%" in gcmd.responses[1]


def test_axis_filter_releases_only_that_pair():
    ss, engine, _ = make_sync()
    ss.cmd_SERVO_SYNC(FakeGcmd(AXIS="Y"))
    assert engine.calls == [(7, 0x0C, 30, 2000)]


def test_gcode_torque_ok_override_reaches_the_engine():
    ss, engine, _ = make_sync()
    ss.cmd_SERVO_SYNC(FakeGcmd(TORQUE_OK="5.0"))
    assert engine.calls == [(7, 0x0F, 50, 2000)]


def test_failed_release_reports_measurements_and_raises():
    ss, engine, _ = make_sync(engine=FakeEngine([failed_report(-846)]))
    gcmd = FakeGcmd()
    with pytest.raises(RuntimeError, match="see measurements"):
        ss.cmd_SERVO_SYNC(gcmd)
    assert len(gcmd.responses) == 2
    assert "still fighting" in gcmd.responses[0]
    assert "code -846" in gcmd.responses[0]


def test_settle_timeout_names_the_failure():
    ss, engine, _ = make_sync(engine=FakeEngine([failed_report(-844)]))
    gcmd = FakeGcmd()
    with pytest.raises(RuntimeError):
        ss.cmd_SERVO_SYNC(gcmd)
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
