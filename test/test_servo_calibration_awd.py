import json
import os
import sys
import tempfile

import pytest

from klippy.extras import servo_axis, servo_calibration, servo_strokes


class FakeServoCapture:
    def __init__(self):
        self.captures = []

    def start_capture_to(self, path, servos):
        self.captures.append((path, list(servos)))

    def stop_capture(self):
        return self.captures[-1][0], 1000, 250


class FakeGcode:
    error = RuntimeError

    def __init__(self):
        self.commands = {}
        self.scripts = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def run_script_from_command(self, script):
        self.scripts.append(script)


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
        value = self._params.get(name, default)
        return None if value is None else float(value)

    def respond_info(self, msg):
        self.responses.append(msg)


class FakeKin:
    def __init__(self, rails, coupled=True):
        self.rails = rails
        self._coupled = coupled

    def coupled_xy(self):
        return self._coupled


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


def _motor(name, node_name, chain_index, invert=False):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = name
    m.node_name = node_name
    m.chain_index = chain_index
    m.invert_direction = invert
    m.rotation_distance = 40.0
    m.encoder_counts_per_rev = 131072
    return m


def _rail(axis, motors):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis " + axis
    rail.axis = axis
    rail.motors = motors
    return rail


def make_calibration(rails, coupled=True):
    gcode = FakeGcode()
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails, coupled)),
        "servo_capture": FakeServoCapture(),
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.dynamics_dir = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._goto_xy = lambda *a, **k: None
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


def _capture_servos(sc):
    return [servos for _p, servos in _cap(sc).captures]


def _manifest_for(sc):
    run_dir = os.path.dirname(_cap(sc).captures[0][0])
    with open(os.path.join(run_dir, "manifest.json")) as f:
        return json.load(f)


def _fit_argv(gcode):
    for _tag, argv, _t in reversed(
        [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    ):
        if argv[1] == "fit":
            return argv
    raise AssertionError("no fit invocation recorded")


def _flag(argv, key):
    return argv[argv.index(key) + 1]


def awd_rails(node="xy_drives", node_b=None):
    return [
        _rail(
            "x",
            [
                _motor("motor_a1", node, 1),
                _motor("motor_a", node, 0),
            ],
        ),
        _rail(
            "y",
            [
                _motor("motor_b", node_b or node, 2, invert=True),
                _motor("motor_b1", node_b or node, 3),
            ],
        ),
    ]


def single_drive_rails():
    return [
        _rail("x", [_motor("motor_a", "xy_drives", 0)]),
        _rail("y", [_motor("motor_b", "xy_drives", 1)]),
    ]


def cartesian_awd_rails():
    return [
        _rail(
            "x",
            [
                _motor("motor_a", "xy_drives", 0),
                _motor("motor_a1", "xy_drives", 1),
            ],
        ),
        _rail("y", [_motor("motor_b", "xy_drives", 2)]),
    ]


def test_layout_single_drive_pairs_is_none():
    sc, _ = make_calibration(single_drive_rails())
    layout = servo_strokes.corexy_fit_layout(FakeGcmd(), sc._kin())
    assert layout == {"servos": ["motor_a", "motor_b"], "pairs": None}


def test_layout_awd_orders_pairs_by_chain_index():
    sc, _ = make_calibration(awd_rails())
    layout = servo_strokes.corexy_fit_layout(FakeGcmd(), sc._kin())
    assert layout["servos"] == [
        "motor_a",
        "motor_a1",
        "motor_b",
        "motor_b1",
    ]
    assert layout["pairs"] == "motor_a,motor_a1;motor_b,motor_b1"


def test_layout_awd_rejects_split_nodes():
    sc, _ = make_calibration(awd_rails(node_b="other_node"))
    with pytest.raises(RuntimeError, match="one ethercat node"):
        servo_strokes.corexy_fit_layout(FakeGcmd(), sc._kin())


def test_layout_rejects_mixed_drive_counts():
    sc, _ = make_calibration(cartesian_awd_rails())
    with pytest.raises(RuntimeError, match="one or two drives per belt"):
        servo_strokes.corexy_fit_layout(FakeGcmd(), sc._kin())


def test_layout_requires_coupled_xy():
    sc, _ = make_calibration(single_drive_rails(), coupled=False)
    with pytest.raises(RuntimeError, match="coupled_xy"):
        servo_strokes.corexy_fit_layout(FakeGcmd(), sc._kin())


def test_servos_override_must_match_kinematics():
    sc, _ = make_calibration(awd_rails())
    layout = servo_strokes.corexy_fit_layout(FakeGcmd(), sc._kin())
    with pytest.raises(RuntimeError, match="does not match"):
        servo_strokes.check_servos_override(
            FakeGcmd(SERVOS="motor_a,motor_b"), layout
        )
    servo_strokes.check_servos_override(
        FakeGcmd(SERVOS="motor_b1, motor_a, motor_b, motor_a1"), layout
    )


def test_fit_dynamics_corexy_captures_all_four_and_passes_axes():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd())
    assert _capture_servos(sc)[0] == [
        "motor_a",
        "motor_a1",
        "motor_b",
        "motor_b1",
    ]
    argv = _fit_argv(gcode)
    assert _flag(argv, "--structure") == "corexy-awd"
    assert _flag(argv, "--axes") == "motor_a,motor_a1,motor_b,motor_b1"


def test_fit_dynamics_corexy_two_drives_is_plain_corexy():
    sc, gcode = make_calibration(single_drive_rails())
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd())
    argv = _fit_argv(gcode)
    assert "--pairs" not in argv
    assert _flag(argv, "--structure") == "corexy"
    assert _flag(argv, "--axes") == "motor_a,motor_b"


def test_scalar_fit_requires_drive_on_multi_drive_axis():
    sc, _ = make_calibration(cartesian_awd_rails(), coupled=False)
    kin = sc._kin()
    with pytest.raises(RuntimeError, match="pass DRIVE="):
        servo_strokes.scalar_fit_drive(FakeGcmd(AXIS="X"), kin)
    assert servo_strokes.scalar_fit_drive(
        FakeGcmd(AXIS="X", DRIVE="motor_a1"), kin
    ) == ("motor_a1")
    with pytest.raises(RuntimeError, match="not among"):
        servo_strokes.scalar_fit_drive(FakeGcmd(AXIS="X", DRIVE="motor_z"), kin)


def test_fit_dynamics_scalar_fit_selects_drive_via_axes():
    sc, gcode = make_calibration(cartesian_awd_rails(), coupled=False)
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd(AXIS="X", DRIVE="motor_a"))
    argv = _fit_argv(gcode)
    assert _flag(argv, "--structure") == "scalar"
    assert _flag(argv, "--axes") == "motor_a"


def test_tracking_combined_view_lists_every_motor_per_belt():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    assert _capture_servos(sc)[0] == [
        "motor_a1",
        "motor_a",
        "motor_b",
        "motor_b1",
    ]
    assert _manifest_for(sc)["belts"] == (
        "motor_a:1+motor_a1:1,motor_b:-1+motor_b1:1"
    )


def test_tracking_single_drive_belts_keep_plain_terms():
    sc, gcode = make_calibration(single_drive_rails())
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    assert _manifest_for(sc)["belts"] == "motor_a:1,motor_b:1"


def test_measure_inertia_captures_every_motor_moving_the_axis():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(AXIS="X"))
    assert _capture_servos(sc)[0] == [
        "motor_a1",
        "motor_a",
        "motor_b",
        "motor_b1",
    ]


def test_measure_inertia_cartesian_captures_only_its_rail():
    rails = [
        _rail(
            "x",
            [_motor("motor_x", "node", 0), _motor("motor_x1", "node", 1)],
        ),
        _rail("y", [_motor("motor_y", "node", 2)]),
    ]
    sc, gcode = make_calibration(rails, coupled=False)
    sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(AXIS="X"))
    assert _capture_servos(sc)[0] == ["motor_x", "motor_x1"]


def test_measure_inertia_cartesian_rejects_corexy_only_params():
    rails = [
        _rail(
            "x",
            [_motor("motor_x", "node", 0), _motor("motor_x1", "node", 1)],
        ),
        _rail("y", [_motor("motor_y", "node", 2)]),
    ]
    sc, _ = make_calibration(rails, coupled=False)
    with pytest.raises(RuntimeError, match="coupled_xy kinematics"):
        sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(AXIS="X", SERVOS="motor_x"))
    with pytest.raises(RuntimeError, match="coupled_xy kinematics"):
        sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(AXIS="X", X_START="10"))


def test_measure_inertia_defaults_to_kinematics_servos_when_corexy():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd())
    assert _capture_servos(sc)[0] == [
        "motor_a1",
        "motor_a",
        "motor_b",
        "motor_b1",
    ]


def test_measure_inertia_corexy_servos_override():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(SERVOS="motor_a,motor_b"))
    assert _capture_servos(sc)[0] == ["motor_a", "motor_b"]


def test_tracking_single_rail_dual_motor_axis_gets_no_combine():
    rails = [
        _rail(
            "x",
            [_motor("motor_x", "x_node", 0), _motor("motor_x1", "x_node", 1)],
        ),
    ]
    sc, gcode = make_calibration(rails, coupled=False)
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    assert _manifest_for(sc)["belts"] is None


def test_calibrate_inertia_ratio_corexy_uses_coupled_grid():
    sc, gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_CALIBRATE_INERTIA_RATIO(
        FakeGcmd(TORQUE_NM=0.3, INERTIA_KGM2=1e-5)
    )
    assert _capture_servos(sc)[0] == [
        "motor_a",
        "motor_a1",
        "motor_b",
        "motor_b1",
    ]
    argv = _fit_argv(gcode)
    assert _flag(argv, "--structure") == "corexy-awd"
    assert _flag(argv, "--rated-torque-nm") == "0.3"


def test_calibrate_inertia_ratio_cartesian_rejects_corexy_only_params():
    sc, _ = make_calibration(single_drive_rails(), coupled=False)
    with pytest.raises(RuntimeError, match="coupled_xy kinematics"):
        sc.cmd_SERVO_CALIBRATE_INERTIA_RATIO(
            FakeGcmd(TORQUE_NM=0.3, INERTIA_KGM2=1e-5, Y_START="10")
        )
