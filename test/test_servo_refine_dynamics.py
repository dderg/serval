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

pytestmark = pytest.mark.skipif(
    tomllib is None, reason="SERVO_REFINE_DYNAMICS requires tomllib (3.11+)"
)

BASELINE_TOML = """\
version = 6
axes = ["motor_a", "motor_b"]
modes = ["x", "y"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.020, 0.030]
viscous = [0.004, 0.005]
coulomb = [1.0, 1.5]
fit_rms_residual = [0.5, 0.5]
"""

ONE_AXIS_TOML = """\
version = 6
axes = ["motor_a"]
modes = ["x"]
frame = [[1.0]]
mass = [0.020]
viscous = [0.004]
coulomb = [1.0]
"""

NON_XY_TOML = """\
version = 6
axes = ["motor_a", "motor_b"]
modes = ["a", "b"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.020, 0.030]
viscous = [0.004, 0.005]
coulomb = [1.0, 1.5]
"""

AWD_TOML = """\
version = 6
axes = ["motor_a", "motor_a1", "motor_b", "motor_b1"]
modes = ["x", "y"]
frame = [[0.25, 0.25, -0.25, 0.25], [0.25, 0.25, 0.25, -0.25]]
mass = [0.020, 0.030]
viscous = [0.004, 0.005]
coulomb = [1.0, 1.5]

[[pair]]
slots = ["motor_a", "motor_a1"]
direction_split = 0.05

[[pair]]
slots = ["motor_b", "motor_b1"]
direction_split = -0.1
"""

OLD_AWD_TOML = AWD_TOML.split("\n[[pair]]", 1)[0] + "\n"

AWD_PAIRS = [
    {"slots": ["motor_a", "motor_a1"], "direction_split": 0.05},
    {"slots": ["motor_b", "motor_b1"], "direction_split": -0.1},
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
    best, _score, _probes = servo_calibration.golden_section_search(
        lambda x: (x + 0.2) ** 2, -1.0, 1.0, 0.02, 10
    )
    assert abs(best + 0.2) < 0.03
    with pytest.raises(ValueError, match="finite LO < HI"):
        servo_calibration.golden_section_search(f, float("nan"), 1.0, 0.02, 10)
    with pytest.raises(ValueError, match="TOL"):
        servo_calibration.golden_section_search(f, 0.7, 1.3, 0.0, 10)
    with pytest.raises(ValueError, match="MAX_EVALS"):
        servo_calibration.golden_section_search(f, 0.7, 1.3, 0.02, 2)


def test_parse_old_v6_dynamics_profile_without_pairs():
    p = servo_calibration.parse_dynamics_profile(BASELINE_TOML)
    assert p["axes"] == ["motor_a", "motor_b"]
    assert p["modes"] == ["x", "y"]
    assert p["frame"] == [[0.5, 0.5], [0.5, -0.5]]
    assert p["mass"] == BASELINE_MASS
    assert p["viscous"] == BASELINE_VISCOUS
    assert p["coulomb"] == BASELINE_COULOMB
    assert p["pairs"] == []


@pytest.mark.parametrize(
    "axes, message",
    [
        ('["motor_a", ""]', "non-empty strings"),
        ('["motor_a", "   "]', "non-empty strings"),
        ('["motor_a", 7]', "non-empty strings"),
        ('["motor_a", "motor_a"]', "unique"),
    ],
)
def test_parse_dynamics_profile_requires_unique_nonempty_axis_names(
    axes, message
):
    text = BASELINE_TOML.replace(
        'axes = ["motor_a", "motor_b"]', "axes = " + axes
    )
    with pytest.raises(ValueError, match=message):
        servo_calibration.parse_dynamics_profile(text)


def test_axis_uniqueness_is_checked_before_pair_mapping():
    text = AWD_TOML.replace(
        'axes = ["motor_a", "motor_a1", "motor_b", "motor_b1"]',
        'axes = ["motor_a", "motor_a", "motor_b", "motor_b1"]',
    )
    with pytest.raises(ValueError, match="axes must be unique"):
        servo_calibration.parse_dynamics_profile(text)


def test_parse_dynamics_profile_parses_signed_pairs():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    assert p["axes"] == ["motor_a", "motor_a1", "motor_b", "motor_b1"]
    assert p["pairs"] == AWD_PAIRS


def test_pair_slot_order_transforms_coefficient_by_frame_lambda():
    swapped = AWD_TOML.replace(
        'slots = ["motor_a", "motor_a1"]\ndirection_split = 0.05',
        'slots = ["motor_a1", "motor_a"]\ndirection_split = -0.05',
    ).replace(
        'slots = ["motor_b", "motor_b1"]\ndirection_split = -0.1',
        'slots = ["motor_b1", "motor_b"]\ndirection_split = -0.1',
    )
    pairs = servo_calibration.parse_dynamics_profile(swapped)["pairs"]
    assert pairs == [
        {"slots": ["motor_a1", "motor_a"], "direction_split": -0.05},
        {"slots": ["motor_b1", "motor_b"], "direction_split": -0.1},
    ]


def test_parse_dynamics_profile_rejects_violations():
    with pytest.raises(ValueError, match="refit with SERVO_FIT_DYNAMICS"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("version = 6", "version = 1")
        )
    with pytest.raises(ValueError, match="refit with SERVO_FIT_DYNAMICS"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("version = 6", "version = 5")
        )
    with pytest.raises(ValueError, match="direction_split"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML
            + '\n[[pair]]\nslots = ["motor_a", "motor_b"]\n'
            + "belt_position_split = [0.02, -0.0003]\n"
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


@pytest.mark.parametrize("value", ["nan", "0.5", "-0.5", "true"])
def test_parse_dynamics_profile_rejects_bad_direction_split(value):
    with pytest.raises(ValueError, match="direction_split"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                "direction_split = 0.05", "direction_split = " + value
            )
        )


def test_parse_dynamics_profile_rejects_pair_slot_violations():
    with pytest.raises(ValueError, match="not among profile axes"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace('motor_a1"]', 'motor_z"]', 1)
        )
    with pytest.raises(ValueError, match="more than one pair"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                'slots = ["motor_b", "motor_b1"]',
                'slots = ["motor_a", "motor_b1"]',
            )
        )
    with pytest.raises(ValueError, match="exact equal or opposite"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                "frame = [[0.25, 0.25, -0.25, 0.25]",
                "frame = [[0.25, 0.2, -0.25, 0.25]",
            )
        )


def test_parse_dynamics_profile_rejects_global_split_and_orientation():
    with pytest.raises(ValueError, match="not a global field"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("mass =", "direction_split = 0.1\nmass =")
        )
    with pytest.raises(ValueError, match="orientation is not supported"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                "direction_split = 0.05",
                "direction_split = 0.05\norientation = -1",
            )
        )


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


def test_dynamics_scaling_preserves_pairs_and_direction_adds():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    for term in ("MASS", "VISCOUS", "COULOMB"):
        scaled = servo_calibration.scale_dynamics(p, term, 1.1)
        assert scaled["pairs"] == AWD_PAIRS
        assert scaled["pairs"] is not p["pairs"]
    mode = servo_calibration.scale_dynamics_mode(p, "MASS", 0, 1.1)
    assert mode["pairs"] == AWD_PAIRS
    added = servo_calibration.add_dynamics_direction_split(p, 0, -0.2)
    assert added["pairs"][0]["direction_split"] == pytest.approx(-0.15)
    assert added["pairs"][1] == AWD_PAIRS[1]
    assert added["mass"] == p["mass"]
    with pytest.raises(ValueError, match=r"abs\(value\) < 0.5"):
        servo_calibration.add_dynamics_direction_split(p, 0, 0.45)


def test_discover_dynamics_pairs_uses_equal_or_opposite_columns():
    p = servo_calibration.parse_dynamics_profile(OLD_AWD_TOML)
    assert servo_calibration.discover_dynamics_pairs(p) == [
        {"slots": ["motor_a", "motor_a1"], "direction_split": 0.0},
        {"slots": ["motor_b", "motor_b1"], "direction_split": 0.0},
    ]
    mixed = dict(p)
    mixed["axes"] = ["a", "a1", "zero", "u", "u1"]
    mixed["frame"] = [[1.0, 1.0, 0.0, 2.0, 4.0], [0.0, 0.0, 0.0, 1.0, 2.0]]
    assert servo_calibration.discover_dynamics_pairs(mixed) == [
        {"slots": ["a", "a1"], "direction_split": 0.0}
    ]
    ambiguous = dict(p)
    ambiguous["axes"] = ["a", "b", "c"]
    ambiguous["frame"] = [[0.5, 0.5, -0.5]]
    with pytest.raises(ValueError, match="ambiguous equal/opposite"):
        servo_calibration.discover_dynamics_pairs(ambiguous)


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


def test_render_dynamics_toml_preserves_pairs():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    text = servo_calibration.render_dynamics_toml(
        p,
        "/cfg/awd.toml",
        "DIRECTION_SPLIT",
        {"motor_a": -0.1, "motor_b": 0.2},
        "/logs/awd",
    )
    assert servo_calibration.parse_dynamics_profile(text)["pairs"] == AWD_PAIRS
    raw = tomllib.loads(text)
    assert raw["refined_delta_motor_a"] == pytest.approx(-0.1)
    assert raw["refined_delta_motor_b"] == pytest.approx(0.2)


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
        self,
        handle,
        frame,
        mass,
        viscous,
        coulomb,
        pair_slots,
        direction_split,
    ):
        self.dynamics_calls.append(
            (
                handle,
                list(frame),
                list(mass),
                list(viscous),
                list(coulomb),
                list(pair_slots),
                list(direction_split),
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


def test_adapter_streams_flat_pair_slots_and_coefficients():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    engine = FakeEngine()
    adapter = servo_calibration.DynamicsModelAdapter(
        engine, 5, p, lambda profile, _value: profile, "pair", "test"
    )
    adapter.apply(0.0)
    assert engine.dynamics_calls[0][0] == 5
    assert engine.dynamics_calls[0][5] == [0, 1, 2, 3]
    assert engine.dynamics_calls[0][6] == pytest.approx([0.05, -0.1])


def test_adapter_uploads_old_v6_profile_with_empty_pairs():
    p = servo_calibration.parse_dynamics_profile(BASELINE_TOML)
    engine = FakeEngine()
    adapter = servo_calibration.DynamicsModelAdapter(
        engine, 5, p, lambda profile, _value: profile, "plain", "test"
    )
    adapter.apply(0.0)
    assert engine.dynamics_calls[0][5:] == ([], [])


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


def test_refine_dynamics_rejects_profile_axis_order_mismatch():
    reordered = BASELINE_TOML.replace(
        'axes = ["motor_a", "motor_b"]',
        'axes = ["motor_b", "motor_a"]',
    )
    sc, _gcode, engine, _path = make_calibration(profile_text=reordered)
    with pytest.raises(RuntimeError, match="maps it to slot"):
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
    with pytest.raises(RuntimeError, match="TERM must be"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X", TERM="SPLIT"))
    with pytest.raises(RuntimeError, match="must contain 1"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(AXIS="X", LO=1.05, HI=1.3))
    assert engine.dynamics_calls == []


def _make_awd_rail(names, node_name, axis, first_slot):
    motors = []
    for offset, name in enumerate(names):
        motor = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
        motor.motor_name = name
        motor.node_name = node_name
        motor.invert_direction = False
        motor.chain_index = first_slot + offset
        motor.rotation_distance = 40.0
        motor.encoder_counts_per_rev = 131072
        motors.append(motor)
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "servo " + axis
    rail.axis = axis
    rail.motors = motors
    return rail


def make_calibration_awd(
    profile_text=OLD_AWD_TOML,
    minima=(-0.12, 0.15),
    motor_biases=None,
    malformed=None,
):
    if motor_biases is None:
        motor_biases = {}
    gcode = FakeGcode()
    rails = [
        _make_awd_rail(["motor_a", "motor_a1"], "drive_all", "x", 0),
        _make_awd_rail(["motor_b", "motor_b1"], "drive_all", "y", 2),
    ]
    engine = FakeEngine()
    fd, profile_path = tempfile.mkstemp(suffix=".toml")
    with os.fdopen(fd, "w") as f:
        f.write(profile_text)
    node = FakeNode(
        "drive_all",
        1,
        {"motor_a": 0, "motor_a1": 1, "motor_b": 2, "motor_b1": 3},
        profile_path,
    )
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails)),
        "motion_engine": engine,
        "servo_capture": FakeServoCapture(),
        "ethercat_node drive_all": node,
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.dynamics_dir = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._strokes = lambda *a, **k: None
    sc._goto_xy = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    def move(move_id, direction, ferr_mean_moving):
        return {
            "move": move_id,
            "direction": direction,
            "start_ms": 1000.0 + 2000.0 * move_id,
            "end_ms": 2000.0 + 2000.0 * move_id,
            "ferr_mean_moving": ferr_mean_moving,
            "ferr_peak": 500.0,
            "ferr_rms": 300.0 + abs(ferr_mean_moving),
            "overshoot": 40.0,
            "settle_ms": 10.0,
            "settle_window_truncated": False,
        }

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if argv[1] != "analyze":
            return
        run_dir = argv[2]
        with open(os.path.join(run_dir, "manifest.json")) as f:
            manifest = json.load(f)
        steps = []
        for step in manifest["steps"]:
            delta = step["swept"].get("delta", 0.0)
            if "direction_split_motor_a" in step["name"]:
                pair_minima = (minima[0], -0.24)
            else:
                pair_minima = (0.24, minima[1])
            values = {}
            for pair_names, optimum in zip(
                (("motor_a", "motor_a1"), ("motor_b", "motor_b1")),
                pair_minima,
            ):
                pair_lambda = 1 if pair_names[0] == "motor_a" else -1
                split_error = 200.0 * (delta - optimum)
                first_bias = motor_biases.get(pair_names[0], 0.0)
                second_bias = motor_biases.get(pair_names[1], 0.0)
                first_moves = []
                second_moves = []
                for move_id, first_direction in enumerate((1, -1)):
                    second_direction = pair_lambda * first_direction
                    first_error = (
                        split_error / 2.0 + first_direction * first_bias
                    )
                    second_error = (
                        -pair_lambda * split_error / 2.0
                        + second_direction * second_bias
                    )
                    first_moves.append(
                        move(move_id, first_direction, first_error)
                    )
                    second_moves.append(
                        move(move_id, second_direction, second_error)
                    )
                values[pair_names[0]] = {"metrics": {"moves": first_moves}}
                values[pair_names[1]] = {"metrics": {"moves": second_moves}}
            first_moves = values["motor_a"]["metrics"]["moves"]
            second_moves = values["motor_a1"]["metrics"]["moves"]
            if malformed == "move_set":
                second_moves[0]["move"] = 9
            elif malformed == "window":
                second_moves[0]["start_ms"] += 1.0
            elif malformed == "zero_direction":
                first_moves[0]["direction"] = 0
            elif malformed == "lambda_direction":
                second_moves[0]["direction"] *= -1
            elif malformed == "missing_bin":
                first_moves.pop()
                second_moves.pop()
            elif malformed == "missing_mean":
                del first_moves[0]["ferr_mean_moving"]
            steps.append({"name": step["name"], "flags": [], "drives": values})
        with open(os.path.join(run_dir, "results.json"), "w") as f:
            json.dump(
                {
                    "steps": steps,
                    "verdict": {"reason": "host-side", "flags": []},
                },
                f,
            )

    sc._run = fake_run
    return sc, gcode, engine, profile_path


@pytest.mark.parametrize(
    "pairs, message",
    [
        ("motor_a,motor_a;motor_b,motor_b1", "must be distinct"),
        ("motor_a,motor_a1;motor_a,motor_b1", "overlap at slots"),
    ],
)
def test_kinematically_derived_pairs_reject_repeated_slots(
    monkeypatch, pairs, message
):
    sc, _gcode, _engine, _path = make_calibration_awd()
    monkeypatch.setattr(
        servo_strokes,
        "corexy_fit_layout",
        lambda _gcmd, _kin: {"servos": [], "pairs": pairs},
    )
    baseline = servo_calibration.parse_dynamics_profile(OLD_AWD_TOML)
    with pytest.raises(RuntimeError, match=message):
        sc._direction_split_baseline(FakeGcmd(), sc._kin(), baseline)


def test_kinematic_pair_rejects_unequal_parallel_columns():
    unequal = OLD_AWD_TOML.replace(
        "frame = [[0.25, 0.25, -0.25, 0.25], [0.25, 0.25, 0.25, -0.25]]",
        "frame = [[0.25, 0.2, -0.25, 0.25], [0.25, 0.2, 0.25, -0.25]]",
    )
    baseline = servo_calibration.parse_dynamics_profile(unequal)
    assert baseline["pairs"] == []
    sc, _gcode, _engine, _path = make_calibration_awd(unequal)
    with pytest.raises(RuntimeError, match="does not have equal parallel"):
        sc._direction_split_baseline(FakeGcmd(), sc._kin(), baseline)


def test_direction_split_refine_recovers_both_lambda_signs_on_old_v6_profile():
    sc, _gcode, engine, _path = make_calibration_awd()
    gcmd = FakeGcmd(TERM="DIRECTION_SPLIT")
    sc.cmd_SERVO_REFINE_DYNAMICS(gcmd)
    profiles = _written_profiles(sc)
    assert len(profiles) == 1
    with open(profiles[0], "rb") as f:
        raw = tomllib.load(f)
    da = raw["refined_delta_motor_a"]
    db = raw["refined_delta_motor_b"]
    assert da == pytest.approx(-0.12, abs=0.03)
    assert db == pytest.approx(0.15, abs=0.03)
    assert [pair["direction_split"] for pair in raw["pair"]] == pytest.approx(
        [da, db]
    )
    assert raw["mass"] == BASELINE_MASS
    assert raw["viscous"] == BASELINE_VISCOUS
    assert raw["coulomb"] == BASELINE_COULOMB
    assert engine.dynamics_calls[-1][5] == [0, 1, 2, 3]
    assert engine.dynamics_calls[-1][6] == [0.0, 0.0]
    candidates = [
        response
        for response in gcmd.responses
        if response.endswith("(counts, mean per move)")
    ]
    assert candidates
    assert all("q_plus" in line for line in candidates)
    assert all("q_minus" in line for line in candidates)
    assert all("ferr_mean_direction_imbalance" in line for line in candidates)


def test_direction_split_refine_cancels_bidirectional_per_motor_bias():
    sc, _gcode, _engine, _path = make_calibration_awd(
        motor_biases={
            "motor_a": 80.0,
            "motor_a1": -35.0,
            "motor_b": -60.0,
            "motor_b1": 25.0,
        }
    )
    sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(TERM="DIRECTION_SPLIT"))
    with open(_written_profiles(sc)[0], "rb") as f:
        raw = tomllib.load(f)
    assert raw["refined_delta_motor_a"] == pytest.approx(-0.12, abs=0.03)
    assert raw["refined_delta_motor_b"] == pytest.approx(0.15, abs=0.03)


def test_direction_split_refine_adds_to_existing_signed_coefficients():
    sc, _gcode, _engine, _path = make_calibration_awd(AWD_TOML)
    sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(TERM="DIRECTION_SPLIT"))
    with open(_written_profiles(sc)[0], "rb") as f:
        raw = tomllib.load(f)
    assert raw["pair"][0]["direction_split"] == pytest.approx(
        0.05 + raw["refined_delta_motor_a"]
    )
    assert raw["pair"][1]["direction_split"] == pytest.approx(
        -0.1 + raw["refined_delta_motor_b"]
    )


@pytest.mark.parametrize(
    "malformed, message",
    [
        ("move_set", "different move sets"),
        ("window", "mismatched start_ms"),
        ("zero_direction", "nonmoving direction"),
        ("lambda_direction", "directions do not match lambda"),
        ("missing_bin", "needs moves in both"),
        ("missing_mean", "invalid ferr_mean_moving"),
    ],
)
def test_direction_split_refine_rejects_malformed_directional_moves(
    malformed, message
):
    sc, _gcode, engine, _profile_path = make_calibration_awd(
        malformed=malformed
    )
    with pytest.raises(RuntimeError, match=message):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(TERM="DIRECTION_SPLIT"))
    assert engine.dynamics_calls[-1] == (
        1,
        [0.25, 0.25, -0.25, 0.25, 0.25, 0.25, 0.25, -0.25],
        BASELINE_MASS,
        BASELINE_VISCOUS,
        BASELINE_COULOMB,
        [0, 1, 2, 3],
        [0.0, 0.0],
    )


def test_direction_split_refine_rejects_missing_or_unsafe_pairs():
    sc, _gcode, engine, _path = make_calibration()
    with pytest.raises(RuntimeError, match="found no explicit"):
        sc.cmd_SERVO_REFINE_DYNAMICS(FakeGcmd(TERM="DIRECTION_SPLIT"))
    assert engine.dynamics_calls == []
    sc, _gcode, engine, _path = make_calibration_awd(AWD_TOML)
    with pytest.raises(RuntimeError, match=r"outside abs\(w\) < 0.5"):
        sc.cmd_SERVO_REFINE_DYNAMICS(
            FakeGcmd(TERM="DIRECTION_SPLIT", LO=-0.45, HI=0.45)
        )
    assert engine.dynamics_calls == []
