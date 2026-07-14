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
version = 1
axes = ["a", "b"]
mass = [[0.030, -0.010], [-0.010, 0.030]]
viscous = [0.004, 0.004]
coulomb_fwd = [1.0, 1.0]
coulomb_rev = [-1.0, -1.0]
coulomb_deadband_mm_s = 0.5
fit_rms_residual = [0.5, 0.5]
"""

BASELINE_MASS_FLAT = [0.030, -0.010, -0.010, 0.030]
BASELINE_VISCOUS = [0.004, 0.004]


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
    assert p["axes"] == ["a", "b"]
    assert p["mass"] == [[0.030, -0.010], [-0.010, 0.030]]
    assert p["viscous"] == BASELINE_VISCOUS
    assert p["coulomb_deadband_mm_s"] == 0.5


def test_parse_dynamics_profile_rejects_violations():
    with pytest.raises(ValueError, match="version"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("version = 1", "version = 2")
        )
    with pytest.raises(ValueError, match="mass"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace(
                "mass = [[0.030, -0.010], [-0.010, 0.030]]",
                "mass = [[0.030, -0.010]]",
            )
        )
    with pytest.raises(ValueError, match="viscous"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace(
                "viscous = [0.004, 0.004]", "viscous = [0.004]"
            )
        )
    with pytest.raises(ValueError, match="non-finite"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace(
                "viscous = [0.004, 0.004]", "viscous = [0.004, nan]"
            )
        )


def test_scale_dynamics_touches_only_the_chosen_term():
    p = servo_calibration.parse_dynamics_profile(BASELINE_TOML)
    m = servo_calibration.scale_dynamics(p, "MASS", 1.1)
    assert m["mass"][0][0] == pytest.approx(0.033)
    assert m["mass"][0][1] == pytest.approx(-0.011)
    assert m["viscous"] == p["viscous"]
    v = servo_calibration.scale_dynamics(p, "VISCOUS", 0.5)
    assert v["viscous"] == [0.002, 0.002]
    assert v["mass"] == p["mass"]
    with pytest.raises(ValueError, match="unknown dynamics term"):
        servo_calibration.scale_dynamics(p, "COULOMB", 1.0)


def test_render_dynamics_toml_reparses_with_provenance():
    p = servo_calibration.parse_dynamics_profile(BASELINE_TOML)
    scaled = servo_calibration.scale_dynamics(p, "MASS", 0.93)
    text = servo_calibration.render_dynamics_toml(
        scaled, "/cfg/dynamics_ident.toml", "MASS", 0.93, "/logs/run_1"
    )
    again = servo_calibration.parse_dynamics_profile(text)
    assert again["mass"][0][0] == pytest.approx(0.030 * 0.93)
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
        self, handle, mass, viscous, coulomb_fwd, coulomb_rev, deadband_mm_s
    ):
        self.dynamics_calls.append(
            (
                handle,
                list(mass),
                list(viscous),
                list(coulomb_fwd),
                list(coulomb_rev),
                deadband_mm_s,
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
                "ferr_peak": 500.0 + 3000.0 * (scale - ferr_rms_min) ** 2,
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
        BASELINE_MASS_FLAT,
        BASELINE_VISCOUS,
        [1.0, 1.0],
        [-1.0, -1.0],
        0.5,
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
    scale = raw["refined_scale"]
    assert abs(scale - 0.9) < 0.03
    assert raw["refined_term"] == "mass"
    assert raw["refined_source"] == profile_path
    assert raw["mass"][0][0] == pytest.approx(0.030 * scale)
    assert raw["viscous"] == BASELINE_VISCOUS


def test_refine_dynamics_viscous_scales_only_viscous():
    sc, gcode, engine, _path = make_calibration(ferr_rms_min=1.1)
    gcmd = FakeGcmd(AXIS="X", TERM="VISCOUS")
    sc.cmd_SERVO_REFINE_DYNAMICS(gcmd)
    candidate = engine.dynamics_calls[-2]
    assert candidate[1] == BASELINE_MASS_FLAT
    profiles = _written_profiles(sc)
    assert len(profiles) == 1
    with open(profiles[0], "rb") as f:
        raw = tomllib.load(f)
    assert raw["refined_term"] == "viscous"
    assert abs(raw["refined_scale"] - 1.1) < 0.03
    assert raw["mass"][0][0] == pytest.approx(0.030)
    assert raw["viscous"][0] == pytest.approx(0.004 * raw["refined_scale"])


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


def test_refine_dynamics_aborts_on_flagged_step():
    sc, gcode, engine, _path = make_calibration()
    real_run = sc._run

    def flagged_run(gcmd, argv, timeout):
        real_run(gcmd, argv, timeout)
        if argv[1] != "analyze":
            return
        path = os.path.join(argv[2], "results.json")
        with open(path) as f:
            results = json.load(f)
        results["steps"][-1]["flags"] = ["torque_saturated"]
        with open(path, "w") as f:
            json.dump(results, f)

    sc._run = flagged_run
    with pytest.raises(RuntimeError, match="torque_saturated"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X"))
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
    one_axis = BASELINE_TOML.replace(
        'axes = ["a", "b"]', 'axes = ["a"]'
    ).replace("mass = [[0.030, -0.010], [-0.010, 0.030]]", "mass = [[0.030]]")
    for key in ("viscous = [0.004, 0.004]",):
        one_axis = one_axis.replace(key, "viscous = [0.004]")
    one_axis = one_axis.replace(
        "coulomb_fwd = [1.0, 1.0]", "coulomb_fwd = [1.0]"
    )
    one_axis = one_axis.replace(
        "coulomb_rev = [-1.0, -1.0]", "coulomb_rev = [-1.0]"
    )
    sc, _gcode, engine, _path = make_calibration(profile_text=one_axis)
    with pytest.raises(RuntimeError, match="describes 1 axes"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X"))
    assert engine.dynamics_calls == []


def test_refine_dynamics_rejects_bad_term_and_bracket():
    sc, _gcode, engine, _path = make_calibration()
    with pytest.raises(RuntimeError, match="TERM must be"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X", TERM="COULOMB"))
    with pytest.raises(RuntimeError, match="must contain 1.0"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X", LO=1.05, HI=1.3))
    assert engine.dynamics_calls == []
