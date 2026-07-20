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

requires_tomllib = pytest.mark.skipif(
    tomllib is None, reason="SERVO_TUNE_DYNAMICS requires tomllib (3.11+)"
)

BASELINE_TOML = """\
version = 6
axes = ["motor_a", "motor_b"]
modes = ["x", "y"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.020, 0.030]
viscous = [0.004, 0.005]
coulomb = [1.0, 1.5]
"""

BASELINE_MASS = [0.020, 0.030]
BASELINE_VISCOUS = [0.004, 0.005]
BASELINE_COULOMB = [1.0, 1.5]


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
        value = None if value is None else float(value)
        if value is not None:
            if minval is not None and value < minval:
                raise self.error(
                    "%s must be at least %s (got %s)" % (name, minval, value)
                )
            if maxval is not None and value > maxval:
                raise self.error(
                    "%s must be at most %s (got %s)" % (name, maxval, value)
                )
            if above is not None and value <= above:
                raise self.error(
                    "%s must be above %s (got %s)" % (name, above, value)
                )
        return value

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


class FakeNode:
    def __init__(self, name, slots, handle=1, profile=None):
        self.name = name
        self._slots = slots
        self._handle = handle
        self.dynamics_profile = profile

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
        self.dynamics_calls = []

    def sdo_read(self, handle, slot, index, subindex):
        return 2, 7

    def set_dynamics_model(self, *args):
        self.dynamics_calls.append(args)


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


def single_drive_rails(node="xy_drives"):
    return [
        _rail("x", [_motor("motor_a", node, 0)]),
        _rail("y", [_motor("motor_b", node, 1)]),
    ]


def _ferr_json(
    modes=("x", "y"),
    mass=(0.0, 0.0),
    viscous=(0.0, 0.0),
    coulomb=(0.0, 0.0),
    mass_se=(1e-6, 1e-6),
    viscous_se=(1e-6, 1e-6),
    coulomb_se=(1e-6, 1e-6),
    ferr_rms=(0.01, 0.01),
    samples=500,
):
    return {
        "version": 1,
        "modes": list(modes),
        "coef": {
            "mass": list(mass),
            "viscous": list(viscous),
            "coulomb": list(coulomb),
        },
        "stderr": {
            "mass": list(mass_se),
            "viscous": list(viscous_se),
            "coulomb": list(coulomb_se),
        },
        "ferr_rms": list(ferr_rms),
        "samples": samples,
    }


def make_calibration(
    rails=None,
    coupled=True,
    profile_text=BASELINE_TOML,
    configure_profile=True,
):
    rails = rails if rails is not None else single_drive_rails()
    gcode = FakeGcode()
    profile_path = None
    if profile_text is not None:
        fd, profile_path = tempfile.mkstemp(suffix=".toml")
        with os.fdopen(fd, "w") as f:
            f.write(profile_text)
    node_profile = profile_path if configure_profile else None
    node_slots = {}
    for rail in rails:
        for m in rail.motors:
            node_slots.setdefault(m.node_name, {})[m.motor_name] = m.chain_index
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails, coupled)),
        "servo_capture": FakeServoCapture(),
        "motion_engine": FakeEngine(),
    }
    for node_name, slots in node_slots.items():
        objs["ethercat_node " + node_name] = FakeNode(
            node_name, slots, profile=node_profile
        )
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.dynamics_dir = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._goto_xy = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    sc.fake_ferr_queue = []
    sc.fake_flags_by_step = {}

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if len(argv) >= 3 and argv[1] == "analyze":
            run_dir = argv[2]
            with open(os.path.join(run_dir, "manifest.json")) as f:
                manifest = json.load(f)
            steps = [
                {
                    "name": s["name"],
                    "flags": sc.fake_flags_by_step.get(s["name"], []),
                }
                for s in manifest["steps"]
            ]
            with open(os.path.join(run_dir, "results.json"), "w") as f:
                json.dump(
                    {"steps": steps, "verdict": {"reason": "ok", "flags": []}},
                    f,
                )
            return ""
        if len(argv) >= 2 and argv[1] == "fit":
            out_path = argv[argv.index("--out") + 1]
            payload = (
                sc.fake_ferr_queue.pop(0)
                if sc.fake_ferr_queue
                else _ferr_json()
            )
            with open(out_path, "w") as f:
                json.dump(payload, f)
            return ""
        return ""

    sc._run = fake_run
    return sc, gcode, profile_path


def _manifest_for(sc):
    run_dir = os.path.dirname(
        sc.printer.lookup_object("servo_capture").captures[0][0]
    )
    with open(os.path.join(run_dir, "manifest.json")) as f:
        return json.load(f)


def test_tune_dynamics_converges_immediately_leaves_model_live_writes_profile():
    sc, gcode, _path = make_calibration()
    engine = sc.printer.lookup_object("motion_engine")
    gcmd = FakeGcmd()
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    assert any("converged in 1 rounds" in r for r in gcmd.responses)
    assert len(engine.dynamics_calls) == 1
    assert engine.dynamics_calls[0][2] == BASELINE_MASS
    assert engine.dynamics_calls[0][3] == BASELINE_VISCOUS
    assert engine.dynamics_calls[0][4] == BASELINE_COULOMB
    assert not any("restored to baseline" in r for r in gcmd.responses)
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["converged"] is True
    assert len(tune["rounds"]) == 1
    with open(tune["profile"]) as f:
        written = servo_calibration.parse_dynamics_profile(f.read())
    assert written["mass"] == BASELINE_MASS
    assert written["viscous"] == BASELINE_VISCOUS
    assert written["coulomb"] == BASELINE_COULOMB


def test_tune_dynamics_positive_mass_coefficient_increases_mass():
    sc, _gcode, _path = make_calibration()
    sc.fake_ferr_queue = [
        _ferr_json(
            mass=(5e-6, 0.0),
            mass_se=(1e-9, 1e-9),
        ),
        _ferr_json(),
    ]
    gcmd = FakeGcmd(TERMS="MASS", ROUNDS=3)
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    round0 = tune["rounds"][0]
    assert round0["values"]["mass"][0] > BASELINE_MASS[0]
    assert round0["values"]["mass"][1] == BASELINE_MASS[1]


def test_tune_dynamics_negative_mass_coefficient_decreases_mass():
    sc, _gcode, _path = make_calibration()
    sc.fake_ferr_queue = [
        _ferr_json(
            mass=(-5e-6, 0.0),
            mass_se=(1e-9, 1e-9),
        ),
        _ferr_json(),
    ]
    gcmd = FakeGcmd(TERMS="MASS", ROUNDS=3)
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    round0 = tune["rounds"][0]
    assert round0["values"]["mass"][0] < BASELINE_MASS[0]


def test_tune_dynamics_secant_uses_the_two_probes():
    sc, _gcode, _path = make_calibration()
    sc.fake_ferr_queue = [
        _ferr_json(mass=(5e-6, 0.0), mass_se=(1e-9, 1e-9)),
        _ferr_json(mass=(-2e-6, 0.0), mass_se=(1e-9, 1e-9)),
        _ferr_json(),
    ]
    gcmd = FakeGcmd(TERMS="MASS", ROUNDS=4)
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    v0 = BASELINE_MASS[0]
    v1 = tune["rounds"][0]["values"]["mass"][0]
    g0 = 5e-6
    g1 = -2e-6
    expected_v2 = v1 - g1 * (v1 - v0) / (g1 - g0)
    v2 = tune["rounds"][1]["values"]["mass"][0]
    assert v2 == pytest.approx(expected_v2)
    assert len(tune["rounds"]) == 3


ZERO_FRICTION_TOML = """\
version = 6
axes = ["motor_a", "motor_b"]
modes = ["x", "y"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.020, 0.030]
viscous = [0.0, 0.0]
coulomb = [0.0, 0.0]
"""


def test_tune_dynamics_negative_coulomb_on_zero_baseline_bounds_at_zero():
    sc, _gcode, _path = make_calibration(profile_text=ZERO_FRICTION_TOML)
    engine = sc.printer.lookup_object("motion_engine")
    sc.fake_ferr_queue = [
        _ferr_json(coulomb=(-2e-3, -5e-4), coulomb_se=(1e-9, 1e-9)),
    ]
    gcmd = FakeGcmd()
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    assert any("converged in 1 rounds" in r for r in gcmd.responses)
    assert any("bounded at 0" in r for r in gcmd.responses)
    for call in engine.dynamics_calls:
        assert all(c >= 0.0 for c in call[4])
        assert all(v >= 0.0 for v in call[3])
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["rounds"][0]["values"]["coulomb"] == [0.0, 0.0]


def test_tune_dynamics_positive_viscous_on_zero_baseline_probes_up():
    sc, _gcode, _path = make_calibration(profile_text=ZERO_FRICTION_TOML)
    sc.fake_ferr_queue = [
        _ferr_json(viscous=(5e-6, 0.0), viscous_se=(1e-9, 1e-9)),
        _ferr_json(),
    ]
    gcmd = FakeGcmd(TERMS="VISCOUS", ROUNDS=3, MAX_SPEED=1000)
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["rounds"][0]["values"]["viscous"][0] == pytest.approx(
        servo_calibration.TUNE_ZERO_FLOOR_STEPS["VISCOUS"]
    )
    assert tune["rounds"][0]["values"]["viscous"][1] == 0.0


def test_tune_dynamics_sub_encoder_count_coefficients_converge():
    sc, _gcode, _path = make_calibration()
    sc.fake_ferr_queue = [
        _ferr_json(
            mass=(7.5e-9, 1.76e-8),
            mass_se=(6.4e-10, 6.7e-10),
            viscous=(6.8e-8, 1.1e-7),
            viscous_se=(7.8e-9, 7.7e-9),
            coulomb=(-1.5e-5, -2.5e-6),
            coulomb_se=(3.7e-6, 3.7e-6),
        ),
        _ferr_json(mass=(7.5e-9, 9e-9), mass_se=(6.4e-10, 6.7e-10)),
    ]
    gcmd = FakeGcmd(MAX_ACCEL=25000, MAX_SPEED=1000)
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    assert any("converged in 2 rounds" in r for r in gcmd.responses)
    assert any("within tolerance" in r for r in gcmd.responses)
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["converged"] is True
    round0 = tune["rounds"][0]["values"]
    assert round0["mass"][0] == BASELINE_MASS[0]
    assert round0["mass"][1] > BASELINE_MASS[1]
    assert round0["viscous"] == BASELINE_VISCOUS
    assert round0["coulomb"] == BASELINE_COULOMB


def test_tune_dynamics_tolerance_scales_with_the_envelope():
    sc, _gcode, _path = make_calibration()
    sc.fake_ferr_queue = [
        _ferr_json(mass=(7.5e-9, 0.0), mass_se=(6.4e-10, 6.7e-10)),
        _ferr_json(),
    ]
    gcmd = FakeGcmd(TERMS="MASS", ROUNDS=3, MAX_ACCEL=100000)
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["rounds"][0]["values"]["mass"][0] > BASELINE_MASS[0]


def test_tune_dynamics_secant_never_streams_negative_friction():
    sc, _gcode, _path = make_calibration()
    engine = sc.printer.lookup_object("motion_engine")
    sc.fake_ferr_queue = [
        _ferr_json(coulomb=(5e-6, 0.0), coulomb_se=(1e-9, 1e-9)),
        _ferr_json(coulomb=(4.9e-6, 0.0), coulomb_se=(1e-9, 1e-9)),
        _ferr_json(),
    ]
    gcmd = FakeGcmd(TERMS="COULOMB", ROUNDS=4)
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    for call in engine.dynamics_calls:
        assert all(c >= 0.0 for c in call[4])


def test_tune_dynamics_exhausted_rounds_restores_baseline_and_raises():
    sc, _gcode, _path = make_calibration()
    engine = sc.printer.lookup_object("motion_engine")
    sc.fake_ferr_queue = [
        _ferr_json(mass=(5e-6, 0.0), mass_se=(1e-9, 1e-9)),
        _ferr_json(mass=(4e-6, 0.0), mass_se=(1e-9, 1e-9)),
    ]
    gcmd = FakeGcmd(TERMS="MASS", ROUNDS=2)
    with pytest.raises(RuntimeError, match="did not converge in 2 rounds"):
        sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    assert engine.dynamics_calls[-1][2] == BASELINE_MASS
    assert any("restored to baseline" in r for r in gcmd.responses)
    manifest = _manifest_for(sc)
    assert "dynamics_tune" not in manifest


def test_tune_dynamics_torque_saturated_restores_baseline_and_raises():
    sc, _gcode, _path = make_calibration()
    engine = sc.printer.lookup_object("motion_engine")
    sc.fake_flags_by_step = {"tune_r0": ["torque_saturated"]}
    gcmd = FakeGcmd()
    with pytest.raises(RuntimeError, match="torque rail"):
        sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    assert engine.dynamics_calls[-1][2] == BASELINE_MASS
    assert engine.dynamics_calls[-1][3] == BASELINE_VISCOUS
    assert engine.dynamics_calls[-1][4] == BASELINE_COULOMB
    manifest = _manifest_for(sc)
    assert "dynamics_tune" not in manifest


def test_tune_dynamics_resonance_detected_warns_and_continues():
    sc, _gcode, _path = make_calibration()
    sc.fake_flags_by_step = {"tune_r0": ["resonance_detected"]}
    gcmd = FakeGcmd()
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    assert any("resonance_detected" in r for r in gcmd.responses)
    assert any("converged in 1 rounds" in r for r in gcmd.responses)


def test_tune_dynamics_rejects_excitation_matrix_params():
    sc, _gcode, _path = make_calibration()
    for params in (
        {"ACCELS": "8000,16000"},
        {"SPEEDS": "100"},
        {"PATTERN": "1"},
    ):
        with pytest.raises(RuntimeError, match="excitation"):
            sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd(**params))


def test_tune_dynamics_rejects_non_coupled_kinematics():
    sc, _gcode, _path = make_calibration(coupled=False)
    with pytest.raises(RuntimeError, match="coupled_xy"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


def test_tune_dynamics_requires_a_baseline_profile():
    sc, _gcode, _path = make_calibration(configure_profile=False)
    with pytest.raises(RuntimeError, match="no baseline dynamics profile"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


def test_tune_dynamics_terms_mass_only_touches_mass_vectors():
    sc, _gcode, _path = make_calibration()
    sc.fake_ferr_queue = [
        _ferr_json(
            mass=(5e-6, 0.0),
            viscous=(5e-6, 0.0),
            coulomb=(5e-6, 0.0),
            mass_se=(1e-9, 1e-9),
            viscous_se=(1e-9, 1e-9),
            coulomb_se=(1e-9, 1e-9),
        ),
        _ferr_json(mass=(0.0, 0.0), mass_se=(1e-9, 1e-9)),
    ]
    engine = sc.printer.lookup_object("motion_engine")
    gcmd = FakeGcmd(TERMS="MASS", ROUNDS=3)
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    assert engine.dynamics_calls[-1][2] != BASELINE_MASS
    assert engine.dynamics_calls[-1][3] == BASELINE_VISCOUS
    assert engine.dynamics_calls[-1][4] == BASELINE_COULOMB
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["terms"] == ["mass"]
    assert "viscous" not in tune["rounds"][0]["coef"]


def test_tune_dynamics_step_dynamics_step_first_probe_and_secant():
    assert servo_calibration.dynamics_tune_step(
        0.02, 5e-6, 0.15, None
    ) == pytest.approx(0.02 + 0.15 * 0.02)
    assert servo_calibration.dynamics_tune_step(
        0.02, -5e-6, 0.15, None
    ) == pytest.approx(0.02 - 0.15 * 0.02)
    assert servo_calibration.dynamics_tune_step(
        1.0, 3e-6, 0.15, None, zero_floor_step=5.0
    ) == pytest.approx(1.15)
    assert servo_calibration.dynamics_tune_step(
        0.0, 3e-6, 0.15, None, zero_floor_step=5.0
    ) == pytest.approx(5.0)
    assert servo_calibration.dynamics_tune_step(
        0.0, -3e-6, 0.15, None, zero_floor_step=5.0
    ) == pytest.approx(-5.0)
    v1, g1 = 0.023, -2e-6
    v0, g0 = 0.02, 5e-6
    expected = v1 - g1 * (v1 - v0) / (g1 - g0)
    assert servo_calibration.dynamics_tune_step(
        v1, g1, 0.15, (v0, g0)
    ) == pytest.approx(expected)
    with pytest.raises(ValueError, match="degenerate"):
        servo_calibration.dynamics_tune_step(0.023, 5e-6, 0.15, (0.02, 5e-6))
