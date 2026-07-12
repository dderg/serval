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
    def __init__(self, rails, coupled=True):
        self.rails = rails
        self._coupled = coupled

    def coupled_xy(self):
        return self._coupled

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
    def __init__(self, name, slots):
        self.name = name
        self._slots = slots

    def get_engine_handle(self):
        return 1

    def get_slot_for_motor(self, motor_name):
        return self._slots[motor_name]


class FakeEngine:
    def sdo_read(self, handle, slot, index, subindex):
        return 2, 7


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


def make_calibration(coupled=True):
    gcode = FakeGcode()
    rails = [
        _make_rail("motor_a", "drive_a", "x"),
        _make_rail("motor_b", "drive_b", "y", invert=True),
    ]
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails, coupled)),
        "servo_capture": FakeServoCapture(),
        "motion_engine": FakeEngine(),
        "ethercat_node drive_a": FakeNode(
            "ethercat_node drive_a", {"motor_a": 0}
        ),
        "ethercat_node drive_b": FakeNode(
            "ethercat_node drive_b", {"motor_b": 0}
        ),
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.dynamics_dir = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if len(argv) >= 3 and argv[1] == "analyze":
            with open(os.path.join(argv[2], "results.json"), "w") as f:
                json.dump(
                    {
                        "verdict": {
                            "recommended_step": "s1",
                            "reason": "ok",
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
    run_dir = os.path.dirname(_cap(sc).captures[0][0])
    with open(os.path.join(run_dir, "manifest.json")) as f:
        return json.load(f)


def _analyze_argv(gcode):
    for tag, argv, _t in reversed(
        [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    ):
        if argv[1] == "analyze":
            return argv
    raise AssertionError("no analyze invocation recorded")


def _g1_lines(scripts):
    lines = []
    for s in scripts:
        if not isinstance(s, str):
            continue
        for line in s.splitlines():
            if line.startswith("G1 X") and "Y" in line:
                lines.append(line)
    return lines


def _coord(line, letter):
    for tok in line.split():
        if tok.startswith(letter):
            return float(tok[1:])
    raise AssertionError("no %s word in %r" % (letter, line))


def test_diagonal_a_moves_x_and_y_together_centered():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="A"))
    lines = _g1_lines(gcode.scripts)
    assert lines, "diagonal stroke emitted no XY G1 moves"
    # bounds default (20,200)x(20,200): center 110, half 90 -> 20..200
    ends = [(_coord(ln, "X"), _coord(ln, "Y")) for ln in lines]
    for x, y in ends:
        assert x == pytest.approx(y), "AXIS=A must move x and y together (+45)"
    xs = [x for x, _ in ends]
    assert min(xs) == pytest.approx(20.0)
    assert max(xs) == pytest.approx(200.0)


def test_diagonal_b_moves_x_up_y_down():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="B"))
    lines = _g1_lines(gcode.scripts)
    ends = [(_coord(ln, "X"), _coord(ln, "Y")) for ln in lines]
    for x, y in ends:
        # -45 diagonal about center (110,110): x-110 == -(y-110)
        assert (x - 110.0) == pytest.approx(-(y - 110.0))


def test_diagonal_a_captures_motor_a_only():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="A"))
    captures = _cap(sc).captures
    assert len(captures) == 1
    assert captures[0][1] == ["motor_a"]


def test_diagonal_b_captures_motor_b_only():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="B"))
    assert _cap(sc).captures[0][1] == ["motor_b"]


def test_diagonal_report_has_no_combine_corexy():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="A"))
    assert _manifest_for(sc)["belts"] is None
    assert _analyze_argv(gcode)[1] == "analyze"


def test_diagonal_requires_corexy():
    sc, _ = make_calibration(coupled=False)
    with pytest.raises(RuntimeError, match="not coupled_xy"):
        sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="A"))


def test_diagonal_reach_validation_uses_toolhead_length():
    sc, _ = make_calibration()
    # length = (END-START)*sqrt(2); reach = SPEED^2/ACCEL. Make reach exceed it.
    gcmd = FakeGcmd(AXIS="A", START=0, END=1, SPEED=100, ACCEL=100)
    with pytest.raises(RuntimeError, match="too short"):
        sc.cmd_SERVO_MEASURE_TRACKING(gcmd)


def test_sweep_accel_step_naming_and_report_invocation():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_SWEEP_ACCEL(FakeGcmd(AXIS="A", ACCELS="20000,10000,10000"))
    names = [os.path.basename(p) for p, _s in _cap(sc).captures]
    # dedup + sorted ascending
    assert names == ["step_accel_a10000.scap", "step_accel_a20000.scap"]
    manifest = _manifest_for(sc)
    assert manifest["experiment"] == "accel_sweep"
    assert [s["name"] for s in manifest["steps"]] == [
        "accel_a10000",
        "accel_a20000",
    ]
    argv = _analyze_argv(gcode)
    assert argv[1] == "analyze"
    assert argv[2] == os.path.dirname(_cap(sc).captures[0][0])


def test_sweep_accel_requires_accels():
    sc, _ = make_calibration()
    with pytest.raises(RuntimeError, match="ACCELS"):
        sc.cmd_SERVO_SWEEP_ACCEL(FakeGcmd(AXIS="X"))


def test_sweep_accel_single_axis_x():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_SWEEP_ACCEL(FakeGcmd(AXIS="X", ACCELS="5000"))
    servos = _cap(sc).captures[0][1]
    # X lane is CoreXY-coupled: both motors drive it
    assert "motor_a" in servos and "motor_b" in servos
    lines = [
        ln
        for s in gcode.scripts
        if isinstance(s, str)
        for ln in s.splitlines()
        if ln.startswith("G1 X")
    ]
    assert lines and all("Y" not in ln for ln in lines)
