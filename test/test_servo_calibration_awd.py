import json
import os
import sys
import tempfile

import pytest

from klippy.extras import servo_axis, servo_calibration, servo_strokes

try:
    import tomllib
except ImportError:
    tomllib = None

requires_tomllib = pytest.mark.skipif(
    tomllib is None,
    reason="the iterative SERVO_FIT_DYNAMICS requires tomllib (3.11+)",
)


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

    def get_commandline(self):
        return "FAKE_CMD " + " ".join(
            "%s=%s" % kv for kv in self._params.items()
        )

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

    def __init__(self, objs, reactor=None):
        self._objs = objs
        self._reactor = reactor

    def lookup_object(self, name):
        return self._objs[name]

    def get_reactor(self):
        return self._reactor


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


class FakeNode:
    def __init__(self, name, slots, handle=7):
        self.name = name
        self._slots = slots
        self._handle = handle
        self.dynamics_profile = None

    def get_engine_handle(self):
        return self._handle

    def get_slot_for_motor(self, motor_name):
        return self._slots[motor_name]

    def get_dynamics_profile(self):
        return self.dynamics_profile

    def get_drive_count(self):
        return len(self._slots)


class FakeEngine:
    def __init__(self):
        self.buzzes = []
        self.dampers = []
        self.trims = []
        self.dynamics_calls = []

    def sdo_read(self, handle, slot, index, subindex):
        return 2, 7

    def resonance_buzz(self, *args):
        self.buzzes.append(args)

    def set_diff_damper(self, *args):
        self.dampers.append(args)

    def set_diff_trim(self, *args):
        self.trims.append(args)

    def set_dynamics_model(self, *args):
        self.dynamics_calls.append(args)


class FakeReactor:
    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        pass


def make_calibration(rails, coupled=True, extra_objs=None, reactor=None):
    gcode = FakeGcode()
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails, coupled)),
        "servo_capture": FakeServoCapture(),
        "motion_engine": FakeEngine(),
    }
    node_slots = {}
    for rail in rails:
        for m in rail.motors:
            node_slots.setdefault(m.node_name, {})[m.motor_name] = m.chain_index
    for node_name, slots in node_slots.items():
        objs["ethercat_node " + node_name] = FakeNode(node_name, slots)
    objs.update(extra_objs or {})
    sc = servo_calibration.ServoCalibration(
        FakeConfig(FakePrinter(objs, reactor))
    )
    sc.captures_root = tempfile.mkdtemp()
    sc.dynamics_dir = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._goto_xy = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    sc.fake_fit_params = []

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
        if len(argv) >= 2 and argv[1] == "fit":
            params = (
                sc.fake_fit_params.pop(0)
                if sc.fake_fit_params
                else (0.02, 0.004, 1.0)
            )
            _write_fit_profile(argv, *params)
        return ""

    sc._run = fake_run
    return sc, gcode


def _write_fit_profile(argv, mass, viscous, coulomb):
    def flag(key):
        return argv[argv.index(key) + 1]

    axes = flag("--axes").split(",")
    modes = flag("--modes").split(",")
    frame = [
        [float(v) for v in row.split(",")] for row in flag("--frame").split(";")
    ]
    n = len(modes)
    lines = [
        "version = 6",
        "axes = [%s]" % (", ".join('"%s"' % a for a in axes),),
        "modes = [%s]" % (", ".join('"%s"' % m for m in modes),),
        "frame = [%s]"
        % (
            ", ".join(
                "[%s]" % (", ".join("%r" % v for v in row),) for row in frame
            ),
        ),
        "mass = [%s]" % (", ".join("%r" % mass for _ in range(n)),),
        "viscous = [%s]" % (", ".join("%r" % viscous for _ in range(n)),),
        "coulomb = [%s]" % (", ".join("%r" % coulomb for _ in range(n)),),
    ]
    with open(flag("--out"), "w") as f:
        f.write("\n".join(lines) + "\n")


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


def trident_awd_rails(node="xy_drives"):
    return [
        _rail(
            "x",
            [
                _motor("motor_a1", node, 1, invert=True),
                _motor("motor_a", node, 0),
            ],
        ),
        _rail(
            "y",
            [
                _motor("motor_b", node, 2, invert=True),
                _motor("motor_b1", node, 3, invert=True),
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


@requires_tomllib
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
    assert "--structure" not in argv
    assert _flag(argv, "--modes") == "x,y"
    assert _flag(argv, "--axes") == "motor_a,motor_a1,motor_b,motor_b1"
    assert _flag(argv, "--frame") == "0.25,0.25,-0.25,0.25;0.25,0.25,0.25,-0.25"
    assert "--signs" not in argv


@requires_tomllib
def test_fit_dynamics_trident_awd_inverted_drives_share_axes():
    sc, gcode = make_calibration(trident_awd_rails())
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd())
    argv = _fit_argv(gcode)
    assert _flag(argv, "--axes") == "motor_a,motor_a1,motor_b,motor_b1"
    assert _flag(argv, "--frame") == (
        "0.25,-0.25,-0.25,-0.25;0.25,-0.25,0.25,0.25"
    )
    assert "--signs" not in argv


@requires_tomllib
def test_fit_dynamics_corexy_two_drives_is_plain_corexy():
    sc, gcode = make_calibration(single_drive_rails())
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd())
    argv = _fit_argv(gcode)
    assert "--pairs" not in argv
    assert "--structure" not in argv
    assert _flag(argv, "--modes") == "x,y"
    assert _flag(argv, "--axes") == "motor_a,motor_b"
    assert _flag(argv, "--frame") == "0.5,0.5;0.5,-0.5"


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
    assert "--structure" not in argv
    assert _flag(argv, "--modes") == "motor_a"
    assert _flag(argv, "--axes") == "motor_a"
    assert _flag(argv, "--frame") == "1"


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


def make_differential_calibration():
    slots = {"motor_a": 0, "motor_a1": 1, "motor_b": 2, "motor_b1": 3}
    engine = FakeEngine()
    node = FakeNode("xy_drives", slots)
    sc, gcode = make_calibration(
        awd_rails(),
        extra_objs={
            "ethercat_node xy_drives": node,
            "motion_engine": engine,
        },
        reactor=FakeReactor(),
    )
    return sc, gcode, engine


def test_differential_buzzes_belt_pair_anti_phase():
    sc, gcode, engine = make_differential_calibration()
    sc.cmd_SERVO_MEASURE_DIFFERENTIAL(FakeGcmd(BELT="A"))
    assert _capture_servos(sc) == [["motor_a", "motor_a1"]]
    assert len(engine.buzzes) == 1
    handle, slot_mask, sign_mask, fs, fe, amp, dur, _ramp = engine.buzzes[0]
    assert handle == 7
    assert slot_mask == 0b0011
    assert sign_mask == 0b0010
    assert (fs, fe) == (20000, 250000)
    assert amp == 50000
    assert dur == int(round((250.0 - 20.0) / 5.0 * 1000.0))
    manifest = _manifest_for(sc)
    assert manifest["experiment"] == "differential"
    assert manifest["axis"] == "A"
    assert manifest["stroke_plan"]["belt"] == "A"
    assert manifest["stroke_plan"]["freq_start"] == 20.0
    assert manifest["stroke_plan"]["freq_end"] == 250.0
    assert manifest["stroke_plan"]["amplitude"] == 0.05
    assert [m["name"] for m in manifest["motors"]] == [
        "motor_a",
        "motor_a1",
    ]
    assert manifest["steps"][0]["name"] == "diff"
    assert sc._active_run is None


def test_differential_belt_b_records_inverts_and_analyzes_run_dir():
    sc, gcode, engine = make_differential_calibration()
    sc.cmd_SERVO_MEASURE_DIFFERENTIAL(FakeGcmd(BELT="B", NAME="dtest"))
    assert _capture_servos(sc) == [["motor_b", "motor_b1"]]
    _handle, slot_mask, sign_mask = engine.buzzes[0][:3]
    assert slot_mask == 0b1100
    assert sign_mask == 0b1000
    manifest = _manifest_for(sc)
    assert manifest["tag"] == "dtest"
    assert [(m["name"], m["invert"]) for m in manifest["motors"]] == [
        ("motor_b", True),
        ("motor_b1", False),
    ]
    runs = [s for s in gcode.scripts if isinstance(s, tuple) and s[0] == "RUN"]
    argv = runs[-1][1]
    assert argv[1] == "analyze"
    assert argv[2] == os.path.dirname(_cap(sc).captures[0][0])


def test_differential_rejects_single_drive_belts():
    sc, _gcode, engine = make_differential_calibration()
    sc.printer = FakePrinter(
        {
            "gcode": sc.gcode,
            "toolhead": FakeToolhead(FakeKin(single_drive_rails())),
        }
    )
    with pytest.raises(RuntimeError, match="two drives per belt"):
        sc.cmd_SERVO_MEASURE_DIFFERENTIAL(FakeGcmd(BELT="A"))
    assert not engine.buzzes


def test_differential_rejects_oversized_amplitude():
    sc, _gcode, engine = make_differential_calibration()
    with pytest.raises(RuntimeError, match="differential ceiling"):
        sc.cmd_SERVO_MEASURE_DIFFERENTIAL(FakeGcmd(BELT="A", AMPLITUDE="0.6"))
    assert not engine.buzzes


def test_differential_stops_capture_when_buzz_fails():
    sc, _gcode, engine = make_differential_calibration()
    stopped = []
    sc._stop_capture = lambda: stopped.append(True)

    def explode(*args):
        raise RuntimeError("endpoint rejected")

    engine.resonance_buzz = explode
    with pytest.raises(RuntimeError, match="endpoint rejected"):
        sc.cmd_SERVO_MEASURE_DIFFERENTIAL(FakeGcmd(BELT="A"))
    assert stopped == [True]
    assert sc._active_run is None


def test_diff_damper_arms_both_belts_by_default():
    sc, _gcode, engine = make_differential_calibration()
    sc.cmd_SERVO_DIFF_DAMPER(FakeGcmd(GAIN="2.5"))
    assert engine.dampers == [
        (7, 0, 1, 2500, 50, 300000, 0),
        (7, 2, 3, 2500, 50, 300000, 0),
    ]


def test_diff_damper_single_belt_with_explicit_knobs():
    sc, _gcode, engine = make_differential_calibration()
    sc.cmd_SERVO_DIFF_DAMPER(
        FakeGcmd(BELT="B", GAIN="0.5", CLAMP="120", LPF_HZ="250", LEAD_US="900")
    )
    assert engine.dampers == [(7, 2, 3, 500, 120, 250000, 900)]


def test_diff_damper_zero_gain_disarms():
    sc, _gcode, engine = make_differential_calibration()
    sc.cmd_SERVO_DIFF_DAMPER(FakeGcmd(BELT="A", GAIN="0"))
    assert engine.dampers == [(7, 0, 1, 0, 50, 300000, 0)]


def test_diff_damper_rejects_oversized_clamp():
    sc, _gcode, engine = make_differential_calibration()
    with pytest.raises(RuntimeError, match="ceiling"):
        sc.cmd_SERVO_DIFF_DAMPER(FakeGcmd(GAIN="1", CLAMP="400"))
    assert not engine.dampers


def test_diff_damper_rejects_single_drive_belts():
    sc, _gcode, engine = make_differential_calibration()
    sc.printer = FakePrinter(
        {
            "gcode": sc.gcode,
            "toolhead": FakeToolhead(FakeKin(single_drive_rails())),
            "motion_engine": engine,
        }
    )
    with pytest.raises(RuntimeError, match="two drives per belt"):
        sc.cmd_SERVO_DIFF_DAMPER(FakeGcmd(GAIN="1"))
    assert not engine.dampers


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


@requires_tomllib
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
    assert "--structure" not in argv
    assert _flag(argv, "--frame") == "0.25,0.25,-0.25,0.25;0.25,0.25,0.25,-0.25"
    assert _flag(argv, "--rated-torque-nm") == "0.3"


def test_calibrate_inertia_ratio_cartesian_rejects_corexy_only_params():
    sc, _ = make_calibration(single_drive_rails(), coupled=False)
    with pytest.raises(RuntimeError, match="coupled_xy kinematics"):
        sc.cmd_SERVO_CALIBRATE_INERTIA_RATIO(
            FakeGcmd(TORQUE_NM=0.3, INERTIA_KGM2=1e-5, Y_START="10")
        )


def _capture_paths(sc):
    return [
        os.path.basename(p)
        for p, _s in sc.printer.lookup_object("servo_capture").captures
    ]


@requires_tomllib
def test_fit_dynamics_iterates_pattern_captures_until_convergence():
    sc, gcode = make_calibration(awd_rails())
    sc.bounds = {"X": (20.0, 280.0), "Y": (20.0, 280.0)}
    strokes = []
    sc._strokes = lambda axis, start, end, *a: strokes.append(axis)
    gcmd = FakeGcmd()
    sc.cmd_SERVO_FIT_DYNAMICS(gcmd)
    assert _capture_paths(sc) == [
        "step_fit_r0.scap",
        "step_fit_r1.scap",
        "step_fit_verify.scap",
    ]
    assert strokes == []
    argv = _fit_argv(gcode)
    caps = [argv[i + 1] for i, a in enumerate(argv) if a == "--capture"]
    assert [os.path.basename(c) for c in caps] == ["step_fit_verify.scap"]
    engine = sc.printer.lookup_object("motion_engine")
    assert len(engine.dynamics_calls) == 2
    assert any("stays live until RESTART" in r for r in gcmd.responses)
    assert any("converged in 2 rounds" in r for r in gcmd.responses)


def _pattern_scripts(gcode):
    return [
        s
        for s in gcode.scripts
        if isinstance(s, str) and "SET_VELOCITY_LIMIT VELOCITY=" in s
    ]


def test_measure_inertia_corexy_pattern_runs_every_grid_cell():
    sc, gcode = make_calibration(awd_rails())
    gcmd = FakeGcmd(PATTERN=1, ACCELS="5000,10000", SPEEDS="100,400")
    sc.cmd_SERVO_MEASURE_INERTIA(gcmd)
    plan = _manifest_for(sc)["stroke_plan"]
    assert plan["pattern"]["x_bounds"] == [20.0, 200.0]
    assert plan["pattern"]["y_bounds"] == [20.0, 200.0]
    assert plan["pattern"]["inset"] == 20.0
    assert plan["pattern"]["small_size"] == 20.0
    assert plan["pattern"]["segments"] == 21
    assert plan["speeds"] == [100.0, 400.0]
    assert plan["accels"] == [5000.0, 10000.0]
    assert _capture_servos(sc)[0] == [
        "motor_a1",
        "motor_a",
        "motor_b",
        "motor_b1",
    ]
    scripts = _pattern_scripts(gcode)
    assert len(scripts) == 4
    for s in scripts:
        assert s.count("G0 X") == 21 * plan["iterations"]
    reach = [
        r
        for r in gcmd.responses
        if r.startswith("accel ") and "pattern segments" in r
    ]
    assert len(reach) == 4


def test_measure_inertia_pattern_ignores_minimum_stroke_distance():
    sc, _ = make_calibration(awd_rails())
    sc.bounds = {"X": (20.0, 60.0), "Y": (20.0, 60.0)}
    with pytest.raises(RuntimeError, match="too short to reach"):
        sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(SPEEDS="400", ACCELS="3000"))
    sc2, _ = make_calibration(awd_rails())
    sc2.bounds = {"X": (20.0, 60.0), "Y": (20.0, 60.0)}
    gcmd = FakeGcmd(PATTERN=1, BOUND=0, SPEEDS="400", ACCELS="3000")
    sc2.cmd_SERVO_MEASURE_INERTIA(gcmd)
    assert any("triangular" in r for r in gcmd.responses), gcmd.responses


def test_measure_inertia_corexy_pattern_rejects_stroke_bounds():
    sc, _ = make_calibration(awd_rails())
    with pytest.raises(RuntimeError, match="single-axis stroke bounds"):
        sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(PATTERN=1, X_START="10"))


def _cartesian_dual_x_rails():
    return [
        _rail(
            "x",
            [_motor("motor_x", "node", 0), _motor("motor_x1", "node", 1)],
        ),
        _rail("y", [_motor("motor_y", "node", 2)]),
    ]


def test_measure_inertia_cartesian_pattern_rejects_stroke_bounds():
    sc, _ = make_calibration(_cartesian_dual_x_rails(), coupled=False)
    with pytest.raises(RuntimeError, match="single-axis stroke bounds"):
        sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(PATTERN=1, START="10"))


def test_measure_inertia_cartesian_pattern_captures_axis_servos():
    sc, gcode = make_calibration(_cartesian_dual_x_rails(), coupled=False)
    sc.cmd_SERVO_MEASURE_INERTIA(FakeGcmd(PATTERN=1, AXIS="X"))
    assert _capture_servos(sc)[0] == ["motor_x", "motor_x1"]
    assert _pattern_scripts(gcode)


def _metric_profile(mass, viscous, coulomb):
    return {
        "modes": ["x", "y"],
        "mass": [mass, mass],
        "viscous": [viscous, viscous],
        "coulomb": [coulomb, coulomb],
    }


def test_torque_changes_zero_for_identical_fits():
    p = _metric_profile(0.02, 0.004, 1.0)
    assert servo_calibration.dynamics_torque_changes(p, p, 10000, 400) == [
        0.0,
        0.0,
    ]


def test_torque_changes_weight_terms_by_operating_point():
    prev = _metric_profile(0.02, 0.004, 1.0)
    new = _metric_profile(0.02, 0.004, 2.0)
    ref = 0.02 * 10000 + 0.004 * 400 + 1.0
    changes = servo_calibration.dynamics_torque_changes(prev, new, 10000, 400)
    assert changes == pytest.approx([1.0 / ref, 1.0 / ref])
    near_zero_viscous_flap = servo_calibration.dynamics_torque_changes(
        _metric_profile(0.02, 1e-6, 1.0),
        _metric_profile(0.02, 3e-6, 1.0),
        10000,
        400,
    )
    assert max(near_zero_viscous_flap) < 0.001


def test_torque_changes_reject_degenerate_and_mismatched_fits():
    zero = _metric_profile(0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="degenerate"):
        servo_calibration.dynamics_torque_changes(
            zero, _metric_profile(0.02, 0.004, 1.0), 10000, 400
        )
    swapped = dict(_metric_profile(0.02, 0.004, 1.0), modes=["y", "x"])
    with pytest.raises(ValueError, match="disagree on modes"):
        servo_calibration.dynamics_torque_changes(
            _metric_profile(0.02, 0.004, 1.0), swapped, 10000, 400
        )


@requires_tomllib
def test_fit_dynamics_rejects_the_excitation_matrix_params():
    sc, _ = make_calibration(awd_rails())
    for params in (
        {"ACCELS": "5000"},
        {"SPEEDS": "100"},
        {"PATTERN": "1"},
    ):
        with pytest.raises(RuntimeError, match="MAX_ACCEL/MAX_SPEED"):
            sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd(**params))


@requires_tomllib
def test_fit_dynamics_records_the_envelope_and_pattern_plan():
    sc, _gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd(MAX_ACCEL="8000", MAX_SPEED="300"))
    plan = _manifest_for(sc)["stroke_plan"]
    assert plan["pattern"]["segments"] == 21
    assert plan["pattern"]["inset"] == 20.0
    assert plan["max_accel"] == 8000.0
    assert plan["converge_accel"] == 4000.0
    assert plan["speeds"] == [150.0, 300.0]
    fit = _manifest_for(sc)["dynamics_fit"]
    assert fit["rounds"] == 2
    assert fit["converged_change"] == 0.0
    assert fit["verify_shift"] == 0.0


@requires_tomllib
def test_fit_dynamics_fails_loudly_when_rounds_never_converge():
    sc, _ = make_calibration(awd_rails())
    sc.fake_fit_params = [
        (0.02, 0.004, 1.0),
        (0.04, 0.004, 1.0),
        (0.08, 0.004, 1.0),
        (0.16, 0.004, 1.0),
    ]
    gcmd = FakeGcmd(MAX_ROUNDS="4")
    with pytest.raises(RuntimeError, match="did not converge in 4 rounds"):
        sc.cmd_SERVO_FIT_DYNAMICS(gcmd)
    assert any("stays live until RESTART" in r for r in gcmd.responses)


@requires_tomllib
def test_fit_dynamics_applies_mass_only_by_default():
    sc, _gcode = make_calibration(awd_rails())
    gcmd = FakeGcmd()
    sc.cmd_SERVO_FIT_DYNAMICS(gcmd)
    engine = sc.printer.lookup_object("motion_engine")
    for call in engine.dynamics_calls:
        assert call[2] == [0.02, 0.02]
        assert call[3] == [0.0, 0.0]
        assert call[4] == [0.0, 0.0]
    profile_path = _manifest_for(sc)["dynamics_fit"]["profile"]
    with open(profile_path) as f:
        text = f.read()
    written = servo_calibration.parse_dynamics_profile(text)
    assert written["mass"] == [0.02, 0.02]
    assert written["viscous"] == [0.0, 0.0]
    assert written["coulomb"] == [0.0, 0.0]
    assert 'applied_terms = ["mass"]' in text
    assert "fitted_viscous = [0.004, 0.004]" in text
    assert "fitted_coulomb = [1.0, 1.0]" in text
    assert "fitted_mass" not in text
    assert any("fitted but not applied" in r for r in gcmd.responses)


@requires_tomllib
def test_fit_dynamics_terms_can_apply_the_full_model():
    sc, _gcode = make_calibration(awd_rails())
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd(TERMS="MASS,VISCOUS,COULOMB"))
    engine = sc.printer.lookup_object("motion_engine")
    assert engine.dynamics_calls[-1][3] == [0.004, 0.004]
    assert engine.dynamics_calls[-1][4] == [1.0, 1.0]
    profile_path = _manifest_for(sc)["dynamics_fit"]["profile"]
    with open(profile_path) as f:
        text = f.read()
    assert "fitted_viscous" not in text
    assert 'applied_terms = ["mass", "viscous", "coulomb"]' in text


@requires_tomllib
def test_fit_dynamics_rejects_massless_or_unknown_terms():
    sc, _ = make_calibration(awd_rails())
    for terms in ("COULOMB", "MASS,STICTION", ""):
        with pytest.raises(RuntimeError, match="include MASS"):
            sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd(TERMS=terms))


@requires_tomllib
def test_fit_dynamics_mass_only_converges_despite_wandering_friction():
    sc, _ = make_calibration(awd_rails())
    sc.fake_fit_params = [
        (0.02, 0.004, 1.0),
        (0.02, 0.008, 3.0),
        (0.02, 0.001, 0.2),
    ]
    gcmd = FakeGcmd()
    sc.cmd_SERVO_FIT_DYNAMICS(gcmd)
    assert any("converged in 2 rounds" in r for r in gcmd.responses)


@requires_tomllib
def test_fit_dynamics_aborts_when_the_model_drifts_at_max_accel():
    sc, _ = make_calibration(awd_rails())
    sc.fake_fit_params = [
        (0.02, 0.004, 1.0),
        (0.02, 0.004, 1.0),
        (0.04, 0.004, 1.0),
    ]
    with pytest.raises(RuntimeError, match="does not hold at MAX_ACCEL"):
        sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd())


@requires_tomllib
def test_fit_dynamics_restores_the_configured_baseline_model():
    sc, _ = make_calibration(awd_rails())
    node = sc.printer.lookup_object("ethercat_node xy_drives")
    baseline = os.path.join(tempfile.mkdtemp(), "baseline.toml")
    with open(baseline, "w") as f:
        f.write(
            "\n".join(
                [
                    "version = 6",
                    'axes = ["motor_a", "motor_a1", "motor_b", "motor_b1"]',
                    'modes = ["x", "y"]',
                    "frame = [[0.25, 0.25, -0.25, 0.25],"
                    " [0.25, 0.25, 0.25, -0.25]]",
                    "mass = [0.5, 0.5]",
                    "viscous = [0.01, 0.01]",
                    "coulomb = [9.0, 9.0]",
                ]
            )
            + "\n"
        )
    node.dynamics_profile = baseline
    gcmd = FakeGcmd()
    sc.cmd_SERVO_FIT_DYNAMICS(gcmd)
    engine = sc.printer.lookup_object("motion_engine")
    assert engine.dynamics_calls[-1][2] == [0.5, 0.5]
    assert engine.dynamics_calls[-1][4] == [9.0, 9.0]
    assert any("restored to configured baseline" in r for r in gcmd.responses)
