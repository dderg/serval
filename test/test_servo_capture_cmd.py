import pytest
from fakes import (
    FakeConfig,
    FakeGcode,
    FakeKin,
    FakeNode,
    FakeToolhead,
)
from fakes import (
    FakeEngine as _FakeEngine,
)
from fakes import (
    FakeGcmd as _FakeGcmd,
)
from fakes import (
    FakePrinter as _FakePrinter,
)

from klippy.extras import servo_axis, servo_capture


class FakeEngine(_FakeEngine):
    def __init__(self, stop_result=(0, 1234, None)):
        super().__init__(stop_servo_capture=stop_result)

    @property
    def start_calls(self):
        return [c[1:] for c in self.calls if c[0] == "start_servo_capture"]

    @property
    def stop_calls(self):
        return [c[1] for c in self.calls if c[0] == "stop_servo_capture"]


class FakePrinter(_FakePrinter):
    command_error = RuntimeError


class FakeGcmd(_FakeGcmd):
    error = RuntimeError


def make_servo_motor(motor_name, node_name):
    motor = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    motor.motor_name = motor_name
    motor.node_name = node_name
    return motor


def make_servo_rail(motor_name, node_name, slot=0, axis=None):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "servo " + motor_name
    rail.axis = axis if axis is not None else motor_name
    rail.motors = [make_servo_motor(motor_name, node_name)]
    return rail


def make_capture(motors=None, engine=None):
    gcode = FakeGcode()
    engine = engine or FakeEngine()
    if motors is None:
        motors = {"x": ("node_x", 7, 0)}
    rails = []
    objs = {"gcode": gcode, "motion_engine": engine}
    node_slots = {}
    node_handles = {}
    for motor_name, (node_name, handle, slot) in motors.items():
        rails.append(make_servo_rail(motor_name, node_name))
        node_slots.setdefault(node_name, {})[motor_name] = slot
        node_handles[node_name] = handle
    for node_name, slots in node_slots.items():
        objs["ethercat_node " + node_name] = FakeNode(
            node_handles[node_name], slots
        )
    objs["toolhead"] = FakeToolhead(FakeKin(rails))
    printer = FakePrinter(objs)
    sc = servo_capture.ServoCapture(FakeConfig(printer))
    return sc, gcode, engine


def node_of(sc, node_name="node_x"):
    return sc.printer.lookup_object("ethercat_node " + node_name)


def test_registers_both_commands():
    _, gcode, _ = make_capture()
    assert "SERVO_CAPTURE_START" in gcode.commands
    assert "SERVO_CAPTURE_STOP" in gcode.commands


def test_start_defaults_to_sole_servo_and_builds_path():
    sc, gcode, engine = make_capture()
    gcmd = FakeGcmd(NAME="xtune")
    gcode.commands["SERVO_CAPTURE_START"](gcmd)
    assert len(engine.start_calls) == 1
    handle, path, started_utc, drives = engine.start_calls[0]
    assert handle == 7
    assert drives == [(0, "x")]
    assert "/servo_captures/" in path
    assert path.endswith(".scap")
    assert "xtune_" in path
    assert started_utc.endswith("Z")
    assert any("started" in r for r in gcmd.responses)


def test_start_rejects_bad_name():
    sc, gcode, engine = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(NAME="../evil"))
    assert engine.start_calls == []


def test_start_rejects_unknown_servo_and_comma_list():
    sc, gcode, engine = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(SERVO="nope"))
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(SERVO="a,b"))
    assert engine.start_calls == []


def test_double_start_rejected_in_klippy():
    sc, gcode, _ = make_capture()
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())


def test_start_on_multi_drive_node_resolves_each_motor_to_its_slot():
    motors = {
        "x": ("node_xy", 9, 0),
        "y": ("node_xy", 9, 1),
    }
    for servo, expected in (("y", (1, "y")), ("x", (0, "x"))):
        _, gcode, engine = make_capture(motors=motors)
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(SERVO=servo))
        assert len(engine.start_calls) == 1
        handle, _path, _utc, drives = engine.start_calls[0]
        assert handle == 9
        assert drives == [expected]


def test_start_resolves_by_axis_regardless_of_motor_name():
    rail_x = make_servo_rail("baz", "node_xy", axis="x")
    rail_y = make_servo_rail("foobar", "node_xy", axis="y")
    gcode = FakeGcode()
    engine = FakeEngine()
    objs = {
        "gcode": gcode,
        "motion_engine": engine,
        "toolhead": FakeToolhead(FakeKin([rail_x, rail_y])),
        "ethercat_node node_xy": FakeNode(9, {"baz": 0, "foobar": 1}),
    }
    servo_capture.ServoCapture(FakeConfig(FakePrinter(objs)))
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(AXIS="Y"))
    handle, _path, _utc, drives = engine.start_calls[0]
    assert handle == 9
    assert drives == [(1, "foobar")]


def test_start_rejects_axis_and_servo_together():
    _, gcode, engine = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(AXIS="x", SERVO="x"))
    assert engine.start_calls == []


def assert_fresh_start_possible(gcode):
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())


def test_stop_without_start_rejected():
    sc, gcode, engine = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    assert engine.stop_calls == []


def test_stop_reports_samples():
    sc, gcode, engine = make_capture()
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    gcmd = FakeGcmd()
    gcode.commands["SERVO_CAPTURE_STOP"](gcmd)
    assert engine.stop_calls == [7]
    assert any("1234" in r for r in gcmd.responses)


def test_stop_overflow_raises_with_failed_filename():
    engine = FakeEngine(stop_result=(-323, 999, 4096))
    sc, gcode, _ = make_capture(engine=engine)
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    with pytest.raises(RuntimeError, match="failed.scap"):
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    assert_fresh_start_possible(gcode)


def test_start_without_engine_handle_fails_loudly():
    sc, gcode, engine = make_capture(motors={"x": ("node_x", None, 0)})
    with pytest.raises(RuntimeError, match="no engine handle"):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    assert engine.start_calls == []


def test_stop_after_node_vanished_clears_state_and_skips_engine():
    sc, gcode, engine = make_capture()
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    fake_node = node_of(sc)
    fake_node._handle = None
    with pytest.raises(RuntimeError, match="vanished"):
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    assert engine.stop_calls == []
    fake_node._handle = 7
    assert_fresh_start_possible(gcode)
    assert len(engine.start_calls) == 2


def test_multiple_servos_require_servo_param():
    motors = {"a": ("node_a", 1, 0), "b": ("node_b", 2, 0)}
    sc, gcode, engine = make_capture(motors=motors)
    with pytest.raises(RuntimeError, match="SERVO= is required"):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    assert engine.start_calls == []
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(SERVO="b"))
    assert len(engine.start_calls) == 1
    assert engine.start_calls[0][0] == 2
    assert engine.start_calls[0][3] == [(0, "b")]


def test_no_servos_configured_errors():
    sc, gcode, engine = make_capture(motors={})
    with pytest.raises(RuntimeError, match="no servo motors configured"):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    assert engine.start_calls == []


def test_stop_failure_message_includes_code_and_cycle():
    engine = FakeEngine(stop_result=(-323, 999, 4096))
    sc, gcode, _ = make_capture(engine=engine)
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    expected_path = engine.start_calls[0][1]
    failed_path = expected_path[: expected_path.rfind(".scap")] + ".failed.scap"
    with pytest.raises(RuntimeError) as exc_info:
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    msg = str(exc_info.value)
    assert "-323" in msg
    assert "4096" in msg
    assert failed_path in msg


def test_start_rejects_name_with_trailing_newline():
    sc, gcode, engine = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(NAME="evil\n"))
    assert engine.start_calls == []


def test_start_engine_failure_is_command_error_not_shutdown():
    class FailingEngine(FakeEngine):
        def start_servo_capture(self, handle, path, started_utc, drives):
            raise RuntimeError("endpoint result -322")

    sc, gcode, _ = make_capture(engine=FailingEngine())
    with pytest.raises(RuntimeError, match="start failed.*-322"):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    assert sc.active is None
