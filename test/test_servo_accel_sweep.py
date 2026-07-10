import pytest

from klippy.extras import servo_axis, servo_calibration


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
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc._prep = lambda *a, **k: None
    sc._restore = lambda *a, **k: None
    sc._run = lambda *a, **k: gcode.scripts.append(("RUN",) + tuple(a[1:]))
    return sc, gcode


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
    starts = [
        s for s in gcode.scripts if isinstance(s, str) and "CAPTURE_START" in s
    ]
    assert len(starts) == 1
    assert "SERVO=motor_a" in starts[0]
    assert "motor_b" not in starts[0]


def test_diagonal_b_captures_motor_b_only():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="B"))
    starts = [
        s for s in gcode.scripts if isinstance(s, str) and "CAPTURE_START" in s
    ]
    assert "SERVO=motor_b" in starts[0]
    assert "motor_a" not in starts[0]


def test_diagonal_report_has_no_combine_corexy():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="A"))
    runs = [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    assert runs
    args = runs[-1][2]
    assert "--combine-corexy" not in args


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
    starts = [
        s for s in gcode.scripts if isinstance(s, str) and "CAPTURE_START" in s
    ]
    names = [s.split("NAME=")[1].split()[0] for s in starts]
    # dedup + sorted ascending
    assert names == ["accel_a10000", "accel_a20000"]
    runs = [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    _tag, script, args, _timeout = runs[-1]
    assert script == "servo_accel_report.py"
    assert "--steps" in args
    assert args[args.index("--steps") + 1] == "accel_a10000,accel_a20000"


def test_sweep_accel_requires_accels():
    sc, _ = make_calibration()
    with pytest.raises(RuntimeError, match="ACCELS"):
        sc.cmd_SERVO_SWEEP_ACCEL(FakeGcmd(AXIS="X"))


def test_sweep_accel_single_axis_x():
    sc, gcode = make_calibration()
    sc.cmd_SERVO_SWEEP_ACCEL(FakeGcmd(AXIS="X", ACCELS="5000"))
    starts = [
        s for s in gcode.scripts if isinstance(s, str) and "CAPTURE_START" in s
    ]
    # X lane is CoreXY-coupled: both motors drive it
    assert "motor_a" in starts[0] and "motor_b" in starts[0]
    lines = [
        ln
        for s in gcode.scripts
        if isinstance(s, str)
        for ln in s.splitlines()
        if ln.startswith("G1 X")
    ]
    assert lines and all("Y" not in ln for ln in lines)
