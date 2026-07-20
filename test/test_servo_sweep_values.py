import json
import os
import sys
import tempfile

import pytest
from fakes import (
    FakeConfig,
    FakeKin,
    FakeNode,
    FakeToolhead,
)
from fakes import FakeEngine as _FakeEngine
from fakes import FakeGcmd as _FakeGcmd
from fakes import FakeGcode as _FakeGcode
from fakes import FakePrinter as _FakePrinter
from fakes import FakeServoCapture as _FakeServoCapture

from klippy.extras import servo_axis, servo_calibration


class FakeServoCapture(_FakeServoCapture):
    def __init__(self):
        super().__init__()
        self.captures = []

    def start_capture_to(self, path, servos):
        super().start_capture_to(path, servos)
        self.captures.append((path, list(servos)))

    def stop_capture(self):
        super().stop_capture()
        return self.captures[-1][0], 1000, 250


def test_validate_gain_values_ranges():
    servo_calibration.validate_gain_values([1, 20000], "position")
    servo_calibration.validate_gain_values([1, 20000], "speed")
    servo_calibration.validate_gain_values([15, 51200], "integral")
    servo_calibration.validate_gain_values([5, 16000], "torque_filter")


def test_validate_gain_values_rejects_out_of_range():
    with pytest.raises(ValueError, match="outside drive range"):
        servo_calibration.validate_gain_values([20001], "position")
    with pytest.raises(ValueError, match="outside drive range"):
        servo_calibration.validate_gain_values([14], "integral")
    with pytest.raises(ValueError, match="outside drive range"):
        servo_calibration.validate_gain_values([4], "torque_filter")
    with pytest.raises(ValueError, match="outside drive range"):
        servo_calibration.validate_gain_values([16001], "torque_filter")


def test_validate_gain_values_rejects_nonpositive():
    with pytest.raises(ValueError, match="positive integer"):
        servo_calibration.validate_gain_values([0], "position")


def test_validate_gain_values_rejects_bad_param():
    with pytest.raises(ValueError, match="PARAM must be"):
        servo_calibration.validate_gain_values([100], "torque")


class FakeGcode(_FakeGcode):
    error = RuntimeError


class FakeGcmd(_FakeGcmd):
    error = RuntimeError


class FakeEngine(_FakeEngine):
    def __init__(self, reads):
        super().__init__()
        self._reads = reads

    def sdo_read(self, handle, slot, index, subindex):
        self.calls.append(("sdo_read", handle, slot, index, subindex))
        return 2, self._reads.get((index, subindex), 7)


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


def make_calibration(reads):
    gcode = FakeGcode()
    rails = [
        _make_rail("motor_a", "drive_a", "x"),
        _make_rail("motor_b", "drive_b", "y"),
    ]
    engine = FakeEngine(reads)
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails, coupled_xy=True)),
        "motion_engine": engine,
        "servo_capture": FakeServoCapture(),
        "ethercat_node drive_a": FakeNode(
            handle=1, slots={"motor_a": 0}, name="drive_a"
        ),
        "ethercat_node drive_b": FakeNode(
            handle=2, slots={"motor_b": 0}, name="drive_b"
        ),
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._strokes = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if argv[1] == "analyze":
            with open(os.path.join(argv[2], "results.json"), "w") as f:
                json.dump({"verdict": {"reason": "ok", "flags": []}}, f)

    sc._run = fake_run
    return sc, gcode


CURRENT_GAINS = {
    (0x2001, 0x01): 400,
    (0x2001, 0x02): 2500,
    (0x2001, 0x03): 3184,
    (0x2001, 0x19): 318,
    (0x2000, 0x07): 100,
}


def _param_writes(scripts, addr):
    out = []
    for s in scripts:
        if not isinstance(s, str):
            continue
        for line in s.splitlines():
            if "SET=" + addr in line:
                out.append(line)
    return out


def test_sweep_inertia_axis_writes_both_drives_and_restores():
    sc, gcode = make_calibration(dict(CURRENT_GAINS))
    gcmd = FakeGcmd(RATIOS="150,160", AXIS="X")
    sc.cmd_SERVO_SWEEP_INERTIA(gcmd)
    writes = _param_writes(gcode.scripts, "0x2000.0x07")
    values = [int(w.split("VALUE=")[1].split()[0]) for w in writes]
    for servo in ("motor_a", "motor_b"):
        assert any("SERVO=%s " % servo in w for w in writes)
    assert values[-2:] == [100, 100]
    assert 150 in values and 160 in values


def test_sweep_inertia_restores_on_failure():
    sc, gcode = make_calibration(dict(CURRENT_GAINS))

    def boom(*a, **k):
        raise RuntimeError("stroke exploded")

    sc._strokes = boom
    gcmd = FakeGcmd(RATIOS="150,160", AXIS="X")
    with pytest.raises(RuntimeError, match="stroke exploded"):
        sc.cmd_SERVO_SWEEP_INERTIA(gcmd)
    writes = _param_writes(gcode.scripts, "0x2000.0x07")
    assert int(writes[-1].split("VALUE=")[1].split()[0]) == 100
