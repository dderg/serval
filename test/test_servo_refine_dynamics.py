import json
import os
import sys
import tempfile

import pytest

from klippy.extras import servo_axis, servo_calibration

try:
    import tomllib
except ImportError:
    tomllib = None

pytestmark = pytest.mark.skipif(
    tomllib is None, reason="SERVO_REFINE_DYNAMICS requires tomllib (3.11+)"
)

BASELINE_TOML = """\
version = 4
axes = ["motor_a", "motor_b"]
modes = ["x", "y"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.020, 0.030]
viscous = [0.004, 0.005]
coulomb = [1.0, 1.5]
fit_rms_residual = [0.5, 0.5]
"""

ONE_AXIS_TOML = """\
version = 4
axes = ["motor_a"]
modes = ["x"]
frame = [[1.0]]
mass = [0.020]
viscous = [0.004]
coulomb = [1.0]
"""

NON_XY_TOML = """\
version = 4
axes = ["motor_a", "motor_b"]
modes = ["a", "b"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.020, 0.030]
viscous = [0.004, 0.005]
coulomb = [1.0, 1.5]
"""

AWD_TOML = """\
version = 4
axes = ["motor_a", "motor_a1", "motor_b", "motor_b1"]
modes = ["x", "y"]
frame = [[0.25, 0.25, -0.25, 0.25], [0.25, 0.25, 0.25, -0.25]]
mass = [0.020, 0.030]
viscous = [0.004, 0.005]
coulomb = [1.0, 1.5]

[[pair]]
slots = ["motor_a", "motor_a1"]
belt_position_split = [0.02, -0.0003]

[[pair]]
slots = ["motor_b", "motor_b1"]
belt_position_split = [0.03, -0.0002]
"""

AWD_PAIRS = [
    {
        "slots": ["motor_a", "motor_a1"],
        "split": [0.02, -0.0003],
    },
    {
        "slots": ["motor_b", "motor_b1"],
        "split": [0.03, -0.0002],
    },
]

BASELINE_FRAME_FLAT = [0.5, 0.5, 0.5, -0.5]
BASELINE_MASS = [0.020, 0.030]
BASELINE_VISCOUS = [0.004, 0.005]
BASELINE_COULOMB = [1.0, 1.5]


def test_gss_converges_on_a_quadratic():
    best, score, probes = servo_calibration.golden_section_search(
        lambda x: (x - 0.9) ** 2, 0.7, 1.3, 0.02, 30
    )
    assert abs(best - 0.9) < 0.03
    assert score == min(s for _x, s in probes)


def test_gss_respects_max_evals():
    calls = []

    def f(x):
        calls.append(x)
        return (x - 0.9) ** 2

    servo_calibration.golden_section_search(f, 0.7, 1.3, 1e-9, 5)
    assert len(calls) == 5


def test_gss_never_reevaluates_a_probe():
    calls = []

    def f(x):
        calls.append(x)
        return (x - 1.0) ** 2

    _best, _score, probes = servo_calibration.golden_section_search(
        f, 0.7, 1.3, 0.005, 30
    )
    assert len(calls) == len(probes)
    assert len(set(calls)) == len(calls)


def test_gss_returns_measured_best_probe_under_noise():
    scores = iter([5.0, 3.0, 4.0, 1.0, 2.0, 6.0, 7.0, 8.0])
    seen = {}

    def noisy(x):
        seen[x] = next(scores)
        return seen[x]

    best, score, probes = servo_calibration.golden_section_search(
        noisy, 0.7, 1.3, 0.02, 8
    )
    assert score == min(seen.values())
    assert seen[best] == score
    assert probes == sorted(seen.items())


def test_gss_rejects_bad_inputs():
    f = lambda x: x  # noqa: E731
    with pytest.raises(ValueError, match="LO < HI"):
        servo_calibration.golden_section_search(f, 1.3, 0.7, 0.02, 10)
    with pytest.raises(ValueError, match="LO < HI"):
        servo_calibration.golden_section_search(f, -1.0, 1.0, 0.02, 10)
    with pytest.raises(ValueError, match="TOL"):
        servo_calibration.golden_section_search(f, 0.7, 1.3, 0.0, 10)
    with pytest.raises(ValueError, match="MAX_EVALS"):
        servo_calibration.golden_section_search(f, 0.7, 1.3, 0.02, 2)


def test_parse_dynamics_profile_roundtrip():
    p = servo_calibration.parse_dynamics_profile(BASELINE_TOML)
    assert p["axes"] == ["motor_a", "motor_b"]
    assert p["modes"] == ["x", "y"]
    assert p["frame"] == [[0.5, 0.5], [0.5, -0.5]]
    assert p["mass"] == BASELINE_MASS
    assert p["viscous"] == BASELINE_VISCOUS
    assert p["coulomb"] == BASELINE_COULOMB
    assert p["pairs"] == []


def test_parse_dynamics_profile_parses_pairs():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    assert p["axes"] == ["motor_a", "motor_a1", "motor_b", "motor_b1"]
    assert p["pairs"] == AWD_PAIRS


def test_parse_dynamics_profile_rejects_violations():
    with pytest.raises(ValueError, match="refit with SERVO_FIT_DYNAMICS"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("version = 4", "version = 1")
        )
    with pytest.raises(ValueError, match="frame"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace(
                "frame = [[0.5, 0.5], [0.5, -0.5]]", "frame = [[0.5, 0.5]]"
            )
        )
    with pytest.raises(ValueError, match="mass"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("mass = [0.020, 0.030]", "mass = [0.020]")
        )
    with pytest.raises(ValueError, match="viscous"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace(
                "viscous = [0.004, 0.005]", "viscous = [0.004]"
            )
        )
    with pytest.raises(ValueError, match="non-finite"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace(
                "viscous = [0.004, 0.005]", "viscous = [0.004, nan]"
            )
        )


def test_parse_dynamics_profile_rejects_pair_violations():
    with pytest.raises(ValueError, match="not among profile axes"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                'slots = ["motor_a", "motor_a1"]',
                'slots = ["motor_a", "motor_zz"]',
            )
        )
    with pytest.raises(ValueError, match="more than one pair"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                'slots = ["motor_b", "motor_b1"]',
                'slots = ["motor_a", "motor_b1"]',
            )
        )
    with pytest.raises(ValueError, match="two distinct motors"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                'slots = ["motor_a", "motor_a1"]',
                'slots = ["motor_a", "motor_a"]',
            )
        )
    with pytest.raises(ValueError, match="split must list exactly 2"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                "belt_position_split = [0.02, -0.0003]",
                "belt_position_split = [0.02]",
            )
        )
    with pytest.raises(ValueError, match="non-finite"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                "belt_position_split = [0.02, -0.0003]",
                "belt_position_split = [0.02, inf]",
            )
        )


def test_scale_dynamics_copies_pair_split_verbatim():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    for term, scale in (("MASS", 2.0), ("VISCOUS", 0.5), ("COULOMB", 3.0)):
        scaled = servo_calibration.scale_dynamics(p, term, scale)
        assert scaled["pairs"] == AWD_PAIRS
        assert scaled["pairs"] is not p["pairs"]
    mode = servo_calibration.scale_dynamics_mode(p, "MASS", 0, 1.5)
    assert mode["pairs"] == AWD_PAIRS


def test_render_dynamics_toml_roundtrips_pairs():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    text = servo_calibration.render_dynamics_toml(
        p, "/cfg/awd.toml", "MASS", {"": 1.0}, "/logs/run_awd"
    )
    again = servo_calibration.parse_dynamics_profile(text)
    assert again["axes"] == p["axes"]
    assert again["frame"] == p["frame"]
    assert again["pairs"] == AWD_PAIRS
    raw = tomllib.loads(text)
    assert raw["version"] == 4


def test_adapter_streams_pair_indices_and_split():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    engine = FakeEngine()
    adapter = servo_calibration.DynamicsModelAdapter(
        engine,
        5,
        p,
        lambda prof, s: servo_calibration.scale_dynamics(prof, "MASS", s),
        "mass",
        "t",
    )
    adapter.apply(1.1)
    adapter.revert()
    assert len(engine.dynamics_calls) == 2
    expected_split = AWD_PAIRS[0]["split"] + AWD_PAIRS[1]["split"]
    for call in engine.dynamics_calls:
        assert call[0] == 5
        assert call[5] == [0, 1, 2, 3]
        assert call[6] == expected_split


def test_scale_dynamics_touches_only_the_chosen_term():
    p = servo_calibration.parse_dynamics_profile(BASELINE_TOML)
    m = servo_calibration.scale_dynamics(p, "MASS", 1.1)
    assert m["mass"][0] == pytest.approx(0.022)
    assert m["mass"][1] == pytest.approx(0.033)
    assert m["viscous"] == p["viscous"]
    assert m["frame"] == p["frame"]
    v = servo_calibration.scale_dynamics(p, "VISCOUS", 0.5)
    assert v["viscous"] == [0.002, 0.0025]
    assert v["mass"] == p["mass"]
    c = servo_calibration.scale_dynamics(p, "COULOMB", 2.0)
    assert c["coulomb"] == [2.0, 3.0]
    assert c["mass"] == p["mass"]
    assert c["viscous"] == p["viscous"]
    with pytest.raises(ValueError, match="unknown dynamics term"):
        servo_calibration.scale_dynamics(p, "STICTION", 1.0)


def test_scale_dynamics_mode_scales_one_entry():
    p = servo_calibration.parse_dynamics_profile(BASELINE_TOML)
    mx = servo_calibration.scale_dynamics_mode(p, "MASS", 0, 1.25)
    assert mx["mass"][0] == pytest.approx(0.025)
    assert mx["mass"][1] == pytest.approx(0.030)
    my = servo_calibration.scale_dynamics_mode(p, "MASS", 1, 2.0)
    assert my["mass"][0] == pytest.approx(0.020)
    assert my["mass"][1] == pytest.approx(0.060)
    assert my["viscous"] == p["viscous"]
    with pytest.raises(ValueError, match="unknown dynamics term"):
        servo_calibration.scale_dynamics_mode(p, "STICTION", 0, 1.0)


def test_render_dynamics_toml_reparses_with_provenance():
    p = servo_calibration.parse_dynamics_profile(BASELINE_TOML)
    scaled = servo_calibration.scale_dynamics(p, "MASS", 0.93)
    text = servo_calibration.render_dynamics_toml(
        scaled, "/cfg/dynamics_ident.toml", "MASS", {"": 0.93}, "/logs/run_1"
    )
    again = servo_calibration.parse_dynamics_profile(text)
    assert again["mass"][0] == pytest.approx(0.020 * 0.93)
    assert again["mass"][1] == pytest.approx(0.030 * 0.93)
    assert again["frame"] == [[0.5, 0.5], [0.5, -0.5]]
    raw = tomllib.loads(text)
    assert raw["refined_source"] == "/cfg/dynamics_ident.toml"
    assert raw["refined_term"] == "mass"
    assert raw["refined_scale"] == pytest.approx(0.93)
    assert raw["refined_run"] == "/logs/run_1"
    assert "fit_rms_residual" not in raw


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
    def __init__(self, name, handle, slots, profile=None):
        self.name = name
        self._handle = handle
        self._slots = slots
        self._profile = profile

    def get_engine_handle(self):
        return self._handle

    def get_slot_for_motor(self, motor):
        return self._slots[motor]

    def get_drive_count(self):
        return len(self._slots)

    def get_dynamics_profile(self):
        return self._profile


class FakeEngine:
    def __init__(self):
        self.dynamics_calls = []

    def sdo_read(self, handle, slot, index, subindex):
        return 2, 7

    def set_dynamics_model(
        self, handle, frame, mass, viscous, coulomb, pairs, pair_split
    ):
        self.dynamics_calls.append(
            (
                handle,
                list(frame),
                list(mass),
                list(viscous),
                list(coulomb),
                list(pairs),
                list(pair_split),
            )
        )


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


def _make_rail(motor, node_name, axis):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = motor
    m.node_name = node_name
    m.invert_direction = False
    m.chain_index = 0
    m.rotation_distance = 40.0
    m.encoder_counts_per_rev = 131072
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "servo " + motor
    rail.axis = axis
    rail.motors = [m]
    return rail


def make_calibration(
    profile_text=BASELINE_TOML,
    overshoot_min=0.9,
    ferr_rms_min=1.1,
    single_node=True,
    configure_profile=True,
):
    gcode = FakeGcode()
    node_a = "drive_ab" if single_node else "drive_a"
    node_b = "drive_ab" if single_node else "drive_b"
    rails = [
        _make_rail("motor_a", node_a, "x"),
        _make_rail("motor_b", node_b, "y"),
    ]
    engine = FakeEngine()
    profile_path = None
    if profile_text is not None:
        fd, profile_path = tempfile.mkstemp(suffix=".toml")
        with os.fdopen(fd, "w") as f:
            f.write(profile_text)
    node_profile = profile_path if configure_profile else None
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails)),
        "motion_engine": engine,
        "servo_capture": FakeServoCapture(),
    }
    if single_node:
        objs["ethercat_node drive_ab"] = FakeNode(
            "drive_ab", 1, {"motor_a": 0, "motor_b": 1}, node_profile
        )
    else:
        objs["ethercat_node drive_a"] = FakeNode(
            "drive_a", 1, {"motor_a": 0}, node_profile
        )
        objs["ethercat_node drive_b"] = FakeNode(
            "drive_b", 2, {"motor_b": 0}, node_profile
        )
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.dynamics_dir = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._strokes = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if argv[1] != "analyze":
            return
        run_dir = argv[2]
        with open(os.path.join(run_dir, "manifest.json")) as f:
            manifest = json.load(f)
        steps = []
        for step in manifest["steps"]:
            scale = step["swept"].get("scale", 1.0)
            move = {
                "move": 0,
                "ferr_peak": 500.0 + 3000.0 * (scale - overshoot_min) ** 2,
                "ferr_rms": 100.0 + 2000.0 * (scale - ferr_rms_min) ** 2,
                "overshoot": 40.0 + 5000.0 * (scale - overshoot_min) ** 2,
                "settle_ms": 10.0,
                "settle_window_truncated": False,
            }
            steps.append(
                {
                    "name": step["name"],
                    "flags": [],
                    "drives": {
                        "motor_a": {"metrics": {"moves": [move]}},
                        "motor_b": {"metrics": {"moves": [move]}},
                    },
                }
            )
        results = {
            "steps": steps,
            "verdict": {"reason": "host-side", "flags": []},
        }
        with open(os.path.join(run_dir, "results.json"), "w") as f:
            json.dump(results, f)

    sc._run = fake_run
    return sc, gcode, engine, profile_path


def _baseline_call(engine, profile_path):
    return (
        1,
        BASELINE_FRAME_FLAT,
        BASELINE_MASS,
        BASELINE_VISCOUS,
        BASELINE_COULOMB,
        [],
        [],
    )


def _written_profiles(sc):
    return [
        os.path.join(sc.dynamics_dir, f)
        for f in sorted(os.listdir(sc.dynamics_dir))
    ]


def test_refine_dynamics_mass_converges_and_restores():
    sc, gcode, engine, profile_path = make_calibration(overshoot_min=0.9)
    gcmd = FakeGcmd(AXIS="X")
    sc.cmd_SERVO_REFINE_DYNAMICS(gcmd)
    assert engine.dynamics_calls, "no models were streamed"
    assert engine.dynamics_calls[-1] == _baseline_call(engine, profile_path)
    for call in engine.dynamics_calls:
        assert call[0] == 1
        assert len(call[1]) == 4 and len(call[2]) == 2
    profiles = _written_profiles(sc)
    assert len(profiles) == 1
    with open(profiles[0], "rb") as f:
        raw = tomllib.load(f)
    sx, sy = raw["refined_scale_x"], raw["refined_scale_y"]
    assert abs(sx - 0.9) < 0.03
    assert abs(sy - 0.9) < 0.03
    assert raw["refined_term"] == "mass"
    assert raw["refined_source"] == profile_path
    assert raw["frame"] == [[0.5, 0.5], [0.5, -0.5]]
    assert raw["mass"][0] == pytest.approx(0.020 * sx)
    assert raw["mass"][1] == pytest.approx(0.030 * sy)
    assert raw["viscous"] == BASELINE_VISCOUS


def test_refine_dynamics_viscous_refines_each_mode():
    sc, gcode, engine, _path = make_calibration(ferr_rms_min=1.1)
    gcmd = FakeGcmd(AXIS="X", TERM="VISCOUS")
    sc.cmd_SERVO_REFINE_DYNAMICS(gcmd)
    candidate = engine.dynamics_calls[-2]
    assert candidate[1] == BASELINE_FRAME_FLAT
    assert candidate[2] == BASELINE_MASS
    profiles = _written_profiles(sc)
    assert len(profiles) == 1
    with open(profiles[0], "rb") as f:
        raw = tomllib.load(f)
    assert raw["refined_term"] == "viscous"
    sx, sy = raw["refined_scale_x"], raw["refined_scale_y"]
    assert abs(sx - 1.1) < 0.03
    assert abs(sy - 1.1) < 0.03
    assert raw["mass"] == BASELINE_MASS
    assert raw["viscous"][0] == pytest.approx(0.004 * sx)
    assert raw["viscous"][1] == pytest.approx(0.005 * sy)


def test_refine_dynamics_mass_runs_full_grid_one_axis_per_phase():
    sc, gcode, engine, _path = make_calibration()
    strokes = []
    sc._strokes = lambda axis, start, end, speed, accel, *a: strokes.append(
        (axis, speed, accel)
    )
    sc.cmd_SERVO_REFINE_DYNAMICS(
        FakeGcmd(ACCELS="5000,10000", SPEEDS="100,400")
    )
    candidates = len(engine.dynamics_calls) - 1
    grid = {(s, a) for a in (5000.0, 10000.0) for s in (100.0, 400.0)}
    assert len(strokes) == candidates * len(grid)
    axes_order = [ax for ax, _s, _a in strokes]
    assert set(axes_order) == {"X", "Y"}
    first_y = axes_order.index("Y")
    assert all(ax == "X" for ax in axes_order[:first_y])
    assert all(ax == "Y" for ax in axes_order[first_y:])
    for axis in ("X", "Y"):
        assert {(s, a) for ax, s, a in strokes if ax == axis} == grid


def test_refine_dynamics_reports_all_metrics_per_scale():
    sc, gcode, engine, _path = make_calibration(overshoot_min=0.9)
    gcmd = FakeGcmd(AXIS="X")
    sc.cmd_SERVO_REFINE_DYNAMICS(gcmd)
    summary = [r for r in gcmd.responses if r.startswith("  mass_")]
    assert summary, "no per-scale summary lines"
    assert any(r.startswith("  mass_x scale ") for r in summary)
    assert any(r.startswith("  mass_y scale ") for r in summary)
    for line in summary:
        assert "overshoot" in line
        assert "ferr_rms" in line
        assert "ferr_peak" in line


def test_refine_dynamics_coulomb_refines_each_mode():
    sc, gcode, engine, _path = make_calibration(overshoot_min=1.05)
    gcmd = FakeGcmd(AXIS="X", TERM="COULOMB")
    sc.cmd_SERVO_REFINE_DYNAMICS(gcmd)
    profiles = _written_profiles(sc)
    assert len(profiles) == 1
    with open(profiles[0], "rb") as f:
        raw = tomllib.load(f)
    assert raw["refined_term"] == "coulomb"
    sx, sy = raw["refined_scale_x"], raw["refined_scale_y"]
    assert abs(sx - 1.05) < 0.03
    assert abs(sy - 1.05) < 0.03
    assert raw["coulomb"][0] == pytest.approx(1.0 * sx)
    assert raw["coulomb"][1] == pytest.approx(1.5 * sy)
    assert raw["mass"] == BASELINE_MASS
    assert raw["viscous"] == BASELINE_VISCOUS


def test_refine_dynamics_skips_write_when_baseline_wins():
    sc, gcode, engine, _path = make_calibration(overshoot_min=1.0)
    gcmd = FakeGcmd(AXIS="X")
    sc.cmd_SERVO_REFINE_DYNAMICS(gcmd)
    assert _written_profiles(sc) == []
    assert any("baseline already optimal" in r for r in gcmd.responses)
    assert engine.dynamics_calls[-1] == _baseline_call(engine, _path)


def test_refine_dynamics_restores_baseline_on_stroke_failure():
    sc, gcode, engine, profile_path = make_calibration()

    def boom(*a, **k):
        raise RuntimeError("stroke exploded")

    sc._strokes = boom
    with pytest.raises(RuntimeError, match="stroke exploded"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X"))
    assert engine.dynamics_calls[-1] == _baseline_call(engine, profile_path)
    assert _written_profiles(sc) == []


def _flagging_run(sc, flag):
    real_run = sc._run

    def flagged_run(gcmd, argv, timeout):
        real_run(gcmd, argv, timeout)
        if argv[1] != "analyze":
            return
        path = os.path.join(argv[2], "results.json")
        with open(path) as f:
            results = json.load(f)
        for step in results["steps"]:
            step["flags"] = [flag]
        with open(path, "w") as f:
            json.dump(results, f)

    return flagged_run


def test_refine_dynamics_aborts_on_torque_rail():
    sc, gcode, engine, _path = make_calibration()
    sc._run = _flagging_run(sc, "torque_saturated")
    with pytest.raises(RuntimeError, match="torque rail"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X"))
    assert len(engine.dynamics_calls) == 2
    assert engine.dynamics_calls[-1] == _baseline_call(engine, _path)


def test_refine_dynamics_ignores_the_resonance_flag():
    sc, gcode, engine, _path = make_calibration(overshoot_min=0.9)
    sc._run = _flagging_run(sc, "resonance_detected")
    gcmd = FakeGcmd(AXIS="X")
    sc.cmd_SERVO_REFINE_DYNAMICS(gcmd)
    profiles = _written_profiles(sc)
    assert len(profiles) == 1
    with open(profiles[0], "rb") as f:
        raw = tomllib.load(f)
    assert abs(raw["refined_scale_x"] - 0.9) < 0.03
    assert engine.dynamics_calls[-1] == _baseline_call(engine, _path)


def test_refine_dynamics_requires_a_profile():
    sc, _gcode, engine, _path = make_calibration(configure_profile=False)
    with pytest.raises(RuntimeError, match="no baseline dynamics profile"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X"))
    assert engine.dynamics_calls == []


def test_refine_dynamics_profile_override_param():
    sc, _gcode, engine, path = make_calibration(configure_profile=False)
    sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X", PROFILE=path))
    assert engine.dynamics_calls[-1] == _baseline_call(engine, path)


def test_refine_dynamics_rejects_multi_node_axes():
    sc, _gcode, engine, _path = make_calibration(single_node=False)
    with pytest.raises(RuntimeError, match="span multiple ethercat nodes"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X"))
    assert engine.dynamics_calls == []


def test_refine_dynamics_rejects_axis_count_mismatch():
    sc, _gcode, engine, _path = make_calibration(profile_text=ONE_AXIS_TOML)
    with pytest.raises(RuntimeError, match="describes 1 axes"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X"))
    assert engine.dynamics_calls == []


def test_refine_dynamics_rejects_non_xy_modes():
    sc, _gcode, engine, _path = make_calibration(profile_text=NON_XY_TOML)
    with pytest.raises(RuntimeError, match="x and y modes"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X"))
    assert engine.dynamics_calls == []


def test_refine_dynamics_rejects_bad_term_and_bracket():
    sc, _gcode, engine, _path = make_calibration()
    with pytest.raises(RuntimeError, match="TERM must be"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X", TERM="STICTION"))
    with pytest.raises(RuntimeError, match="must contain 1.0"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X", LO=1.05, HI=1.3))
    assert engine.dynamics_calls == []


def test_scale_dynamics_pair_scales_only_that_pair():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    s = servo_calibration.scale_dynamics_pair(p, 0, 2.0)
    assert s["pairs"][0]["split"] == [w * 2.0 for w in p["pairs"][0]["split"]]
    assert s["pairs"][1]["split"] == p["pairs"][1]["split"]
    assert s["mass"] == p["mass"]
    assert s["viscous"] == p["viscous"]
    assert s["coulomb"] == p["coulomb"]


def _make_awd_rail(names, node_name, axis):
    motors = []
    for name in names:
        m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
        m.motor_name = name
        m.node_name = node_name
        m.invert_direction = False
        m.chain_index = 0
        m.rotation_distance = 40.0
        m.encoder_counts_per_rev = 131072
        motors.append(m)
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "servo " + axis
    rail.axis = axis
    rail.motors = motors
    return rail


def make_calibration_awd(minima=(0.9, 1.15), off_pair_min=1.3, lean_gain=200.0):
    """The fake analyzer leans each split phase's own pair mates apart in
    ferr_rms proportionally to (scale - minima entry), so the imbalance
    objective crosses zero exactly at the minima entry, and leans the
    OTHER pair around an adversarial off_pair_min - if the per-pair drive
    filter broke and the score read the wrong pair's mates, the found
    best would drift to off_pair_min."""
    gcode = FakeGcode()
    rails = [
        _make_awd_rail(["motor_a", "motor_a1"], "drive_all", "x"),
        _make_awd_rail(["motor_b", "motor_b1"], "drive_all", "y"),
    ]
    engine = FakeEngine()
    fd, profile_path = tempfile.mkstemp(suffix=".toml")
    with os.fdopen(fd, "w") as f:
        f.write(AWD_TOML)
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails)),
        "motion_engine": engine,
        "servo_capture": FakeServoCapture(),
        "ethercat_node drive_all": FakeNode(
            "drive_all",
            1,
            {"motor_a": 0, "motor_a1": 1, "motor_b": 2, "motor_b1": 3},
            profile_path,
        ),
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.dynamics_dir = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._strokes = lambda *a, **k: None
    sc._restore = lambda *a, **k: None
    sc._goto_xy = lambda *a, **k: None

    def move_for(mini, scale, lean):
        return {
            "move": 0,
            "ferr_peak": 500.0 + 3000.0 * (scale - mini) ** 2,
            "ferr_rms": 300.0 + 2000.0 * (scale - mini) ** 2 + lean,
            "overshoot": 40.0 + 5000.0 * (scale - mini) ** 2,
            "settle_ms": 10.0,
            "settle_window_truncated": False,
        }

    def mate_moves(mini, scale):
        lean = lean_gain * (scale - mini)
        return (
            [move_for(mini, scale, lean)],
            [move_for(mini, scale, -lean)],
        )

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if argv[1] != "analyze":
            return
        run_dir = argv[2]
        with open(os.path.join(run_dir, "manifest.json")) as f:
            manifest = json.load(f)
        steps = []
        for step in manifest["steps"]:
            scale = step["swept"].get("scale", 1.0)
            if "split_motor_a" in step["name"]:
                pair_a, pair_b = minima[0], off_pair_min
            elif "split_motor_b" in step["name"]:
                pair_a, pair_b = off_pair_min, minima[1]
            else:
                pair_a = pair_b = 1.0
            a0, a1 = mate_moves(pair_a, scale)
            b0, b1 = mate_moves(pair_b, scale)
            steps.append(
                {
                    "name": step["name"],
                    "flags": [],
                    "drives": {
                        "motor_a": {"metrics": {"moves": a0}},
                        "motor_a1": {"metrics": {"moves": a1}},
                        "motor_b": {"metrics": {"moves": b0}},
                        "motor_b1": {"metrics": {"moves": b1}},
                    },
                }
            )
        results = {
            "steps": steps,
            "verdict": {"reason": "host-side", "flags": []},
        }
        with open(os.path.join(run_dir, "results.json"), "w") as f:
            json.dump(results, f)

    sc._run = fake_run
    return sc, gcode, engine, profile_path


def test_refine_dynamics_split_requires_pairs():
    sc, _gcode, engine, _path = make_calibration()
    with pytest.raises(RuntimeError, match="no \\[\\[pair\\]\\] tables"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X", TERM="SPLIT"))
    assert engine.dynamics_calls == []


def test_refine_dynamics_split_even_mates_keep_baseline():
    sc, _gcode, _engine, _path = make_calibration_awd(
        minima=(0.8, 0.8), off_pair_min=0.8, lean_gain=0.0
    )
    sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(TERM="SPLIT"))
    assert _written_profiles(sc) == []


def test_refine_dynamics_split_reports_each_mate_per_scale():
    sc, _gcode, _engine, _path = make_calibration_awd()
    gcmd = FakeGcmd(TERM="SPLIT")
    sc.cmd_SERVO_REFINE_DYNAMICS(gcmd)
    for pair, mates in (
        ("split_motor_a", ("motor_a", "motor_a1")),
        ("split_motor_b", ("motor_b", "motor_b1")),
    ):
        summary = [
            r for r in gcmd.responses if r.startswith("  %s scale " % pair)
        ]
        assert summary, "no per-scale summary lines for %s" % pair
        for line in summary:
            for mate in mates:
                assert "ferr_rms[%s]" % mate in line
            assert "ferr_rms_imbalance" in line


def test_refine_dynamics_split_refines_each_pair_on_its_own_drives():
    sc, gcode, engine, profile_path = make_calibration_awd()
    sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(TERM="SPLIT"))
    profiles = _written_profiles(sc)
    assert len(profiles) == 1
    with open(profiles[0], "rb") as f:
        raw = tomllib.load(f)
    assert raw["refined_term"] == "split"
    sa = raw["refined_scale_motor_a"]
    sb = raw["refined_scale_motor_b"]
    assert abs(sa - 0.9) < 0.03, sa
    assert abs(sb - 1.15) < 0.03, sb
    base = servo_calibration.parse_dynamics_profile(AWD_TOML)
    for written, pair, scale in zip(raw["pair"], base["pairs"], (sa, sb)):
        assert written["belt_position_split"] == pytest.approx(
            [w * scale for w in pair["split"]]
        )
    assert raw["mass"] == pytest.approx(base["mass"])
    assert raw["viscous"] == pytest.approx(base["viscous"])
    assert raw["coulomb"] == pytest.approx(base["coulomb"])
    last = engine.dynamics_calls[-1]
    assert last[5] == [0, 1, 2, 3]
    assert last[6] == pytest.approx(
        base["pairs"][0]["split"] + base["pairs"][1]["split"]
    )
