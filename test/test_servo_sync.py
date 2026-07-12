import pytest

from klippy.extras import servo_axis, servo_sync


class FakeReactor:
    def __init__(self):
        self._t = 0.0
        self.pauses = []

    def monotonic(self):
        self._t += 0.001
        return self._t

    def pause(self, until):
        self.pauses.append(until - self._t)
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
        self.dwells = []

    def get_kinematics(self):
        return self._kin

    def wait_moves(self):
        self.wait_moves_calls += 1

    def dwell(self, delay):
        self.dwells.append(delay)

    def get_last_move_time(self):
        return 12.5


class FakeKin:
    def __init__(self, rails, lane_names):
        self.rails = rails
        self._lane_names = lane_names
        self.parked = []

    def lanes(self):
        return [(i, name, []) for i, name in enumerate(self._lane_names)]

    def mark_servo_parked(self, axes):
        self.parked.append(tuple(axes))


class FakeNode:
    def __init__(self, name, handle, slots):
        self.name = name
        self._handle = handle
        self._slots = slots
        self._torque_motors = set()
        self.torque_calls = []
        self.waiter_calls = 0

    def get_engine_handle(self):
        return self._handle

    def get_slot_for_motor(self, motor_name):
        return self._slots[motor_name]

    def set_motor_torque(self, motor_name, value, print_time):
        self.torque_calls.append((motor_name, value, print_time))
        if value:
            first = not self._torque_motors
            self._torque_motors.add(motor_name)
            if first:

                def waiter():
                    self.waiter_calls += 1

                return waiter
        else:
            self._torque_motors.discard(motor_name)
        return None


class FakeEngine:
    def __init__(self, torques=None):
        self.sdo_reads = []
        self._torques = list(
            torques if torques is not None else [80, -78, 40, -38, 3, -2, 1, -1]
        )

    def motion_drained(self):
        return True

    def sdo_read(self, handle, slot, index, subindex):
        self.sdo_reads.append((handle, slot, index, subindex))
        raw = self._torques.pop(0)
        return (2, raw & 0xFFFF)


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

    def get_float(self, name, default=None, **kw):
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
    return ss, engine, node, printer


def test_torque_cycles_off_then_on_for_every_belt_rail():
    ss, engine, node, printer = make_sync()
    gcmd = FakeGcmd()
    ss.cmd_SERVO_SYNC(gcmd)
    assert node.torque_calls == [
        ("axis x", False, 12.5),
        ("axis y", False, 12.5),
        ("axis x", True, 12.5),
        ("axis y", True, 12.5),
    ]
    assert node.waiter_calls == 1
    toolhead = printer.lookup_object("toolhead")
    assert toolhead.dwells == [pytest.approx(1.0)]
    assert toolhead.wait_moves_calls == 2, (
        "the settle must be waited out in the print-time domain, or the "
        "re-enable cancels the still-pending disable"
    )
    assert toolhead.get_kinematics().parked == [(0, 1)]


def test_reads_torque_before_and_after_and_reports_per_axis():
    ss, engine, node, printer = make_sync()
    gcmd = FakeGcmd()
    ss.cmd_SERVO_SYNC(gcmd)
    assert engine.sdo_reads == [(7, s, 0x6077, 0) for s in (0, 1, 2, 3)] * 2
    assert len(gcmd.responses) == 2
    assert "axis x released: motor_a +8.0% -> +0.3%" in gcmd.responses[0]
    assert "motor_a1 -7.8% -> -0.2%" in gcmd.responses[0]
    assert "axis y released: motor_b +4.0% -> +0.1%" in gcmd.responses[1]


def test_axis_filter_releases_only_that_pair():
    ss, engine, node, _ = make_sync(engine=FakeEngine(torques=[40, -38, 1, -1]))
    ss.cmd_SERVO_SYNC(FakeGcmd(AXIS="Y"))
    assert [c[0] for c in node.torque_calls] == ["axis y", "axis y"]
    assert [r[1] for r in engine.sdo_reads] == [2, 3, 2, 3]


def test_settle_override_stretches_the_relax_dwell():
    ss, _, _, printer = make_sync()
    ss.cmd_SERVO_SYNC(FakeGcmd(SETTLE="2.5"))
    assert printer.lookup_object("toolhead").dwells == [pytest.approx(2.5)]


def test_residual_fight_after_release_errors_loudly():
    ss, _, _, _ = make_sync(
        engine=FakeEngine(torques=[80, -78, 40, -38, 60, -55, 1, -1])
    )
    gcmd = FakeGcmd()
    with pytest.raises(RuntimeError, match="still fighting"):
        ss.cmd_SERVO_SYNC(gcmd)
    assert any("motor_a +8.0% -> +6.0%" in r for r in gcmd.responses)


def test_torque_ok_override_loosens_the_threshold():
    ss, _, _, _ = make_sync(
        engine=FakeEngine(torques=[80, -78, 40, -38, 60, -55, 1, -1])
    )
    ss.cmd_SERVO_SYNC(FakeGcmd(TORQUE_OK="7.0"))


def test_z_axis_is_rejected_loudly():
    rails = [
        make_rail("x", ["motor_a", "motor_a1"]),
        make_rail("z", ["motor_z", "motor_z1"]),
    ]
    ss, _, _, _ = make_sync(rails=rails, lane_names=("x", "z"))
    with pytest.raises(RuntimeError, match="racking"):
        ss.cmd_SERVO_SYNC(FakeGcmd(AXIS="Z"))


def test_single_drive_axes_are_not_syncable():
    rails = [make_rail("x", ["motor_a"]), make_rail("y", ["motor_b"])]
    ss, _, _, _ = make_sync(rails=rails)
    with pytest.raises(RuntimeError, match="no belt axis"):
        ss.cmd_SERVO_SYNC(FakeGcmd())
