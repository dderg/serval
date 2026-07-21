import pytest
from fakes import (
    FakeConfig,
    FakeGcode,
    FakeKin,
    FakeNode,
    FakePrinter,
)
from fakes import (
    FakeEngine as FakeEngineBase,
)
from fakes import (
    FakeGcmd as FakeGcmdBase,
)
from fakes import (
    FakeToolhead as FakeToolheadBase,
)

from klippy.extras import servo_axis, servo_sync

DEFAULT_TORQUES = [80, -78, 40, -38, 3, -2, 1, -1]


class FakeGcmd(FakeGcmdBase):
    error = RuntimeError


class FakeEngine(FakeEngineBase):
    def __init__(self, torques=None):
        torques = (
            list(torques) if torques is not None else list(DEFAULT_TORQUES)
        )
        super().__init__(sdo_read=[(2, t & 0xFFFF) for t in torques])

    @property
    def sdo_reads(self):
        return [c[1:] for c in self.calls if c[0] == "sdo_read"]


class FakeToolhead(FakeToolheadBase):
    def __init__(self, kin):
        super().__init__(kin=kin, last_move_time=12.5)

    @property
    def wait_moves_calls(self):
        return sum(1 for c in self.calls if c[0] == "wait_moves")

    @property
    def print_time_waits(self):
        return [c[1] for c in self.calls if c[0] == "wait_until_print_time"]


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
        handle=7,
        slots={"motor_a": 0, "motor_a1": 1, "motor_b": 2, "motor_b1": 3},
        name="xy_drives",
    )
    kin = FakeKin(
        rails=rails,
        lanes=[(i, name, []) for i, name in enumerate(lane_names)],
    )
    toolhead = FakeToolhead(kin)
    printer = FakePrinter(
        objects={
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
    assert toolhead.print_time_waits == [pytest.approx(13.5)], (
        "the settle must be waited out past the scheduled disable time on "
        "the MCU clock, or the re-enable cancels the still-pending disable"
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


def test_settle_override_stretches_the_relax_window():
    ss, _, _, printer = make_sync()
    ss.cmd_SERVO_SYNC(FakeGcmd(SETTLE="2.5"))
    toolhead = printer.lookup_object("toolhead")
    assert toolhead.print_time_waits == [pytest.approx(15.0)]


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


def test_retry_recycles_only_the_fighting_axis_then_succeeds():
    ss, engine, node, _ = make_sync(
        engine=FakeEngine(
            torques=[1, -1, 80, -78, 2, -2, 80, -78, 60, -55, 1, -1]
        )
    )
    gcmd = FakeGcmd(RETRIES="1")
    ss.cmd_SERVO_SYNC(gcmd)
    assert [r[1] for r in engine.sdo_reads] == [
        0, 1, 2, 3,
        0, 1, 2, 3,
        2, 3,
        2, 3,
    ]  # fmt: skip
    assert [(c[0], c[1]) for c in node.torque_calls] == [
        ("axis x", False),
        ("axis y", False),
        ("axis x", True),
        ("axis y", True),
        ("axis y", False),
        ("axis y", True),
    ]
    assert any("retrying release (1/1)" in r for r in gcmd.responses)


def test_retries_exhausted_still_errors_loudly():
    ss, _, _, _ = make_sync(
        engine=FakeEngine(
            torques=[0, 0, 0, 0, 80, -78, 90, -88, 0, 0, 0, 0, 60, -55, 88, -86]
        )
    )
    gcmd = FakeGcmd(RETRIES="1")
    with pytest.raises(RuntimeError, match="and 1 retries"):
        ss.cmd_SERVO_SYNC(gcmd)


def test_retries_config_default_used_without_param():
    ss, engine, _, _ = make_sync(
        engine=FakeEngine(
            torques=[1, -1, 80, -78, 2, -2, 80, -78, 60, -55, 1, -1]
        )
    )
    ss.retries = 1
    ss.cmd_SERVO_SYNC(FakeGcmd())
    assert len(engine.sdo_reads) == 12
