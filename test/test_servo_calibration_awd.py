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
    def __init__(self, slots, handle=7):
        self._slots = slots
        self._handle = handle

    def get_engine_handle(self):
        return self._handle

    def get_slot_for_motor(self, motor_name):
        return self._slots[motor_name]


class FakeEngine:
    def __init__(self):
        self.buzzes = []
        self.dampers = []
        self.trims = []

    def sdo_read(self, handle, slot, index, subindex):
        return 2, 7

    def resonance_buzz(self, *args):
        self.buzzes.append(args)

    def set_diff_damper(self, *args):
        self.dampers.append(args)

    def set_diff_trim(self, *args):
        self.trims.append(args)


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
        objs["ethercat_node " + node_name] = FakeNode(slots)
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


def test_fit_dynamics_trident_awd_inverted_drives_share_axes():
    sc, gcode = make_calibration(trident_awd_rails())
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd())
    argv = _fit_argv(gcode)
    assert _flag(argv, "--axes") == "motor_a,motor_a1,motor_b,motor_b1"
    assert _flag(argv, "--frame") == (
        "0.25,-0.25,-0.25,-0.25;0.25,-0.25,0.25,0.25"
    )
    assert "--signs" not in argv


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
    node = FakeNode(slots)
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


def test_fit_dynamics_coupled_runs_one_full_range_capture():
    sc, gcode = make_calibration(awd_rails())
    strokes = []
    sc._strokes = lambda axis, start, end, *a: strokes.append(
        (axis, start, end)
    )
    sc.bounds = {"X": (20.0, 280.0), "Y": (20.0, 280.0)}
    sc.cmd_SERVO_FIT_DYNAMICS(FakeGcmd())
    assert _capture_paths(sc) == ["step_ident.scap"]
    argv = _fit_argv(gcode)
    caps = [argv[i + 1] for i, a in enumerate(argv) if a == "--capture"]
    assert [os.path.basename(c) for c in caps] == ["step_ident.scap"]
    assert {(s, e) for ax, s, e in strokes if ax == "X"} == {(20.0, 280.0)}
    assert {(s, e) for ax, s, e in strokes if ax == "Y"} == {(20.0, 280.0)}
