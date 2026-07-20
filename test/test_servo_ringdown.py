import json
import os
import sys
import tempfile

import pytest
from fakes import (
    FakeConfig,
    FakeGcode,
    FakeKin,
    FakeNode,
    FakeServoCapture,
    FakeToolhead,
)
from fakes import FakeEngine as _FakeEngine
from fakes import FakeGcmd as _FakeGcmd
from fakes import FakePrinter as _FakePrinter

from klippy.extras import servo_axis, servo_calibration


class FakeGcmd(_FakeGcmd):
    error = RuntimeError


class FakeEngine(_FakeEngine):
    def sdo_read(self, handle, slot, index, subindex):
        return 2, 7

    def effective_limits(self):
        return 3000.0, 25000.0, 0.05

    def set_post_processor_bypass(self, enabled):
        return self._call("bypass", enabled)

    def set_jerk_override(self, jerk):
        return self._call("jerk", jerk)


class FakeAccelClient:
    def __init__(self, valid=True):
        self.finished = False
        self.valid = valid

    def finish_measurements(self):
        self.finished = True

    def has_valid_samples(self):
        return self.valid

    def get_samples(self):
        return [(100.0 + 0.001 * k, 1.0, 2.0, 3.0) for k in range(100)]


class FakeAccelChip:
    def __init__(self, valid=True):
        self.clients = []
        self.valid = valid

    def start_internal_client(self):
        client = FakeAccelClient(self.valid)
        self.clients.append(client)
        return client


class FakePrinter(_FakePrinter):
    command_error = RuntimeError


def _make_rail(motor, node_name, axis, invert=False):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = motor
    m.node_name = node_name
    m.invert_direction = invert
    m.chain_index = 0
    m.rotation_distance = 40.0
    m.encoder_counts_per_rev = 131072
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "servo " + motor
    rail.axis = axis
    rail.motors = [m]
    return rail


def make_calibration(coupled=True):
    gcode = FakeGcode()
    rails = [
        _make_rail("motor_a", "drive_a", "x"),
        _make_rail("motor_b", "drive_b", "y", invert=True),
    ]
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails=rails, coupled_xy=coupled)),
        "servo_capture": FakeServoCapture(),
        "motion_engine": FakeEngine(),
        "adxl345 tool": FakeAccelChip(),
        "ethercat_node drive_a": FakeNode(
            handle=1, slots={"motor_a": 0}, name="ethercat_node drive_a"
        ),
        "ethercat_node drive_b": FakeNode(
            handle=1, slots={"motor_b": 0}, name="ethercat_node drive_b"
        ),
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.dynamics_dir = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if len(argv) >= 3 and argv[1] == "analyze":
            with open(os.path.join(argv[2], "results.json"), "w") as f:
                json.dump(
                    {
                        "verdict": {
                            "recommended_step": None,
                            "reason": "ring after stop: 40.0 Hz",
                            "flags": [],
                        }
                    },
                    f,
                )

    sc._run = fake_run
    return sc, gcode


def _cap(sc):
    return sc.printer.lookup_object("servo_capture")


def _manifest_for(sc):
    run_dir = os.path.dirname(_cap(sc).starts[0][0])
    with open(os.path.join(run_dir, "manifest.json")) as f:
        return json.load(f)


def _g1_lines(scripts):
    return [
        line
        for s in scripts
        if isinstance(s, str)
        for line in s.splitlines()
        if line.startswith("G1 ")
    ]


def test_one_step_per_speed_with_recorded_stops():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="X", SPEEDS="400,100,100"))
    manifest = _manifest_for(sc)
    assert manifest["experiment"] == "ringdown"
    names = [s["name"] for s in manifest["steps"]]
    assert names == ["ringdown_v100", "ringdown_v400"]
    for step in manifest["steps"]:
        assert len(step["stops"]) == 6, "3 iterations = 6 stops per step"
        assert step["stops"] == sorted(step["stops"])
        assert step["accel"] is None
    plan = manifest["stroke_plan"]
    assert plan["speeds"] == [100, 400]
    assert plan["accel"] == 25000.0, (
        "defaults to the printer's effective max accel, not the "
        "[servo_calibration] accels list"
    )
    assert plan["dwell_ms"] == 1500
    assert plan["iterations"] == 3
    assert plan["center"] == 110.0, "bounds (20,200) center on 110"
    assert plan["cruise_ms"] == 200


def _step_extents(gcode, coord_letter="X"):
    extents = set()
    for line in _g1_lines(gcode.scripts):
        for tok in line.split():
            if tok.startswith(coord_letter):
                extents.add(float(tok[1:]))
    return extents


def test_strokes_are_short_and_centered():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="X", SPEEDS="100,400"))
    # accel 25000, cruise 200 ms: v=100 -> 0.4 + 20 = 20.4 mm centered on
    # 110 -> 99.8..120.2; v=400 -> 6.4 + 80 = 86.4 mm -> 66.8..153.2.
    assert _step_extents(gcode) == {99.8, 120.2, 66.8, 153.2}


def test_cruise_ms_scales_the_stroke():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="X", SPEEDS="100", CRUISE_MS=0))
    # No cruise: the stroke is exactly the accel+decel reach v^2/a = 0.4 mm.
    assert _step_extents(gcode) == {109.8, 110.2}


def test_stroke_exceeding_bounds_fails_loud():
    sc, _ = make_calibration()
    with pytest.raises(RuntimeError, match="lower SPEEDS or CRUISE_MS"):
        sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="X", SPEEDS="2000"))


def test_accel_above_printer_limit_fails_loud():
    sc, _ = make_calibration()
    with pytest.raises(RuntimeError, match="max accel"):
        sc.cmd_SERVO_MEASURE_RINGDOWN(
            FakeGcmd(AXIS="X", SPEEDS="250", ACCEL=30000)
        )


def test_speed_above_printer_limit_fails_loud():
    sc, _ = make_calibration()
    with pytest.raises(RuntimeError, match="max velocity"):
        sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="X", SPEEDS="5000"))


def test_bypass_and_jerk_wrap_the_strokes_and_restore():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="X", SPEEDS="250"))
    engine = sc.printer.lookup_object("motion_engine")
    assert engine.calls == [
        ("bypass", True),
        ("jerk", float("inf")),
        ("jerk", None),
        ("bypass", False),
    ]


def test_bypass_and_jerk_restore_on_failure():
    sc, gcode = make_calibration()
    sc.printer.add_object("adxl345 tool", FakeAccelChip(valid=False))
    with pytest.raises(RuntimeError, match="no data"):
        sc.cmd_SERVO_MEASURE_RINGDOWN(
            FakeGcmd(AXIS="X", SPEEDS="250", ACCEL_CHIP="adxl345 tool")
        )
    engine = sc.printer.lookup_object("motion_engine")
    assert engine.calls[-2:] == [("jerk", None), ("bypass", False)]
    assert sc._active_run is None


def test_strokes_are_submitted_one_move_at_a_time():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_RINGDOWN(
        FakeGcmd(AXIS="X", SPEEDS="250", ITERATIONS=2)
    )
    strokes = [
        s for s in gcode.scripts if isinstance(s, str) and s.startswith("G1 ")
    ]
    assert len(strokes) == 4, "2 iterations = 4 single-move stroke scripts"
    for s in strokes:
        assert "\n" not in s, (
            "each stroke must be its own script so the stop fence "
            "lands between the move and the dwell: %r" % (s,)
        )
    assert any(
        isinstance(s, str) and s.startswith("SET_VELOCITY_LIMIT ACCEL=25000")
        for s in gcode.scripts
    )


def test_dwell_below_minimum_fails_loud():
    sc, _ = make_calibration()
    with pytest.raises(RuntimeError, match="DWELL_MS"):
        sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="X", DWELL_MS=200))


def test_accel_chip_capture_recorded_in_manifest():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_RINGDOWN(
        FakeGcmd(AXIS="X", SPEEDS="250", ACCEL_CHIP="adxl345 tool")
    )
    chip = sc.printer.lookup_object("adxl345 tool")
    assert len(chip.clients) == 1
    assert chip.clients[0].finished
    manifest = _manifest_for(sc)
    step = manifest["steps"][0]
    assert step["accel"] == "step_ringdown_v250_accel.csv"
    run_dir = os.path.dirname(_cap(sc).starts[0][0])
    assert os.path.exists(os.path.join(run_dir, step["accel"]))


def test_diagonal_axis_captures_one_belt():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="A", SPEEDS="250"))
    assert _cap(sc).starts[0][1] == ["motor_a"]
    manifest = _manifest_for(sc)
    assert manifest["belts"] is None, "diagonal runs must not combine belts"


def test_corexy_x_records_belts_for_the_combined_source():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="X", SPEEDS="250"))
    manifest = _manifest_for(sc)
    assert manifest["belts"] is not None
    servos = _cap(sc).starts[0][1]
    assert "motor_a" in servos and "motor_b" in servos


def test_analyze_invoked_on_the_run_dir():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_RINGDOWN(FakeGcmd(AXIS="X", SPEEDS="250"))
    run_dir = os.path.dirname(_cap(sc).starts[0][0])
    analyze = [
        argv
        for s in gcode.scripts
        if isinstance(s, tuple) and s[0] == "RUN"
        for argv in [s[1]]
        if argv[1] == "analyze"
    ]
    assert analyze and analyze[-1][2] == run_dir
    assert sc._active_run is None, "run must be closed even on success"
