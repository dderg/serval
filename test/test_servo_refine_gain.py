import json
import os
import sys
import tempfile

import pytest

from klippy.extras import servo_axis, servo_calibration


class FakeServoCapture:
    def __init__(self):
        self.captures = []

    def start_capture_to(self, path, servos):
        self.captures.append((path, list(servos)))

    def stop_capture(self):
        return self.captures[-1][0], 1000, 250


def test_refine_values_default_span_includes_current():
    vals = servo_calibration.refine_values(2500, None, 0.3, 5)
    assert 2500 in vals
    assert vals == sorted(vals)
    assert vals[0] == 1750 and vals[-1] == 3250
    assert len(vals) == 5


def test_refine_values_odd_steps_center_is_current():
    vals = servo_calibration.refine_values(1000, None, 0.2, 3)
    assert vals == [800, 1000, 1200]


def test_refine_values_even_steps_still_include_current():
    vals = servo_calibration.refine_values(1000, None, 0.2, 4)
    assert 1000 in vals
    assert vals == sorted(set(vals))


def test_refine_values_explicit_list_dedupes_and_sorts():
    vals = servo_calibration.refine_values(0, "300,250,300,400", None, 0)
    assert vals == [250, 300, 400]


def test_refine_values_empty_explicit_list_fails():
    with pytest.raises(ValueError, match="no usable numbers"):
        servo_calibration.refine_values(0, " , ", None, 0)


def test_refine_values_bad_span_or_steps_fail():
    with pytest.raises(ValueError, match="STEPS"):
        servo_calibration.refine_values(1000, None, 0.3, 1)
    with pytest.raises(ValueError, match="SPAN"):
        servo_calibration.refine_values(1000, None, 1.5, 5)


def test_validate_gain_values_ranges():
    servo_calibration.validate_gain_values([1, 30000], "position")
    servo_calibration.validate_gain_values([1, 20000], "speed")
    servo_calibration.validate_gain_values([15, 51200], "integral")


def test_validate_gain_values_rejects_out_of_range():
    with pytest.raises(ValueError, match="outside drive range"):
        servo_calibration.validate_gain_values([30001], "position")
    with pytest.raises(ValueError, match="outside drive range"):
        servo_calibration.validate_gain_values([14], "integral")


def test_validate_gain_values_rejects_nonpositive():
    with pytest.raises(ValueError, match="positive integer"):
        servo_calibration.validate_gain_values([0], "position")


def test_validate_gain_values_rejects_bad_param():
    with pytest.raises(ValueError, match="PARAM must be"):
        servo_calibration.validate_gain_values([100], "torque")


class FakeGcode:
    def __init__(self):
        self.commands = {}
        self.scripts = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def run_script_from_command(self, script):
        self.scripts.append(script)

    error = RuntimeError


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def get_int(self, name, default=None, minval=None, maxval=None):
        return int(self._params.get(name, default))

    def get_float(
        self,
        name,
        default=None,
        minval=None,
        maxval=None,
        above=None,
        below=None,
    ):
        return float(self._params.get(name, default))

    def respond_info(self, msg):
        self.responses.append(msg)


class FakeKin:
    def __init__(self, rails):
        self.rails = rails

    def coupled_xy(self):
        return True

    def get_kinematics(self):
        return self

    def get_status(self, eventtime):
        return {"homed_axes": "xyz"}


class FakeToolhead:
    def __init__(self, kin):
        self.kin = kin

    def get_kinematics(self):
        return self.kin


class FakeNode:
    def __init__(self, name, handle, slots):
        self.name = name
        self._handle = handle
        self._slots = slots

    def get_engine_handle(self):
        return self._handle

    def get_slot_for_motor(self, motor):
        return self._slots[motor]


class FakeEngine:
    def __init__(self, reads):
        self.reads = reads

    def sdo_read(self, handle, slot, index, subindex):
        return 2, self.reads[(index, subindex)]


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name):
        return self._objs[name]


class FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer

    def get(self, name, default=None):
        return default

    def getlist(self, name, default=None):
        return default

    def getfloat(self, name, default=None, **kw):
        return default

    def getfloatlist(self, name, default=None):
        return default

    def getint(self, name, default=None, **kw):
        return default


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
        "toolhead": FakeToolhead(FakeKin(rails)),
        "motion_engine": engine,
        "servo_capture": FakeServoCapture(),
        "ethercat_node drive_a": FakeNode("drive_a", 1, {"motor_a": 0}),
        "ethercat_node drive_b": FakeNode("drive_b", 2, {"motor_b": 0}),
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


def test_refine_gain_writes_both_drives_and_restores():
    sc, gcode = make_calibration(dict(CURRENT_GAINS))
    gcmd = FakeGcmd(PARAM="speed", AXIS="X", VALUES="1750,2500,3250")
    sc.cmd_SERVO_REFINE_GAIN(gcmd)
    writes = _param_writes(gcode.scripts, "0x2001.0x02")
    values = [int(w.split("VALUE=")[1].split()[0]) for w in writes]
    for servo in ("motor_a", "motor_b"):
        assert any("SERVO=%s " % servo in w for w in writes)
    assert values[-1] == 2500
    assert 1750 in values and 3250 in values


def test_refine_gain_reads_current_as_center():
    sc, gcode = make_calibration(dict(CURRENT_GAINS))
    gcmd = FakeGcmd(PARAM="speed", AXIS="X", SPAN=0.2, STEPS=3)
    sc.cmd_SERVO_REFINE_GAIN(gcmd)
    writes = _param_writes(gcode.scripts, "0x2001.0x02")
    values = sorted({int(w.split("VALUE=")[1].split()[0]) for w in writes})
    assert values == [2000, 2500, 3000]


def test_refine_gain_restores_on_failure():
    sc, gcode = make_calibration(dict(CURRENT_GAINS))

    def boom(*a, **k):
        raise RuntimeError("stroke exploded")

    sc._strokes = boom
    gcmd = FakeGcmd(PARAM="speed", AXIS="X", VALUES="1750,2500,3250")
    with pytest.raises(RuntimeError, match="stroke exploded"):
        sc.cmd_SERVO_REFINE_GAIN(gcmd)
    writes = _param_writes(gcode.scripts, "0x2001.0x02")
    last = int(writes[-1].split("VALUE=")[1].split()[0])
    assert last == 2500


def test_refine_gain_rejects_bad_param():
    sc, _ = make_calibration(dict(CURRENT_GAINS))
    with pytest.raises(RuntimeError, match="PARAM must be"):
        sc.cmd_SERVO_REFINE_GAIN(FakeGcmd(PARAM="torque", AXIS="X"))


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
