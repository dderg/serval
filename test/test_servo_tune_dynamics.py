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
    ferr_rms_raw=(0.002, 0.002),
    onset_bias=(0.0, 0.0),
    onset_windows=8,
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
        "ferr_rms_raw": list(ferr_rms_raw),
        "onset_bias": list(onset_bias),
        "onset_windows": onset_windows,
        "samples": samples,
    }


def quadratic_rms(
    mass_opt=BASELINE_MASS,
    viscous_opt=BASELINE_VISCOUS,
    coulomb_opt=BASELINE_COULOMB,
    floor=(0.0015, 0.0015),
    curvature=(0.05, 0.05),
):
    """Per-mode rms objective for the fake bench: a parabola over the
    normalized distance of every term from its optimum, so the tuner has
    a real minimum to find and cross-term probes move the score."""

    def rms(mass, viscous, coulomb):
        out = []
        for k in range(2):
            d2 = 0.0
            for vals, opts in (
                (mass, mass_opt),
                (viscous, viscous_opt),
                (coulomb, coulomb_opt),
            ):
                scale = max(abs(opts[k]), 1e-9)
                d2 += ((vals[k] - opts[k]) / scale) ** 2
            out.append(floor[k] + curvature[k] * d2)
        return out

    return rms


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
    sc.fake_rms_fn = quadratic_rms()
    sc.fake_coef_hints = {
        "mass": (0.0, 0.0),
        "viscous": (0.0, 0.0),
        "coulomb": (0.0, 0.0),
    }
    sc.fake_onset_fn = lambda mass, viscous, coulomb: (0.0, 0.0)

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
            if sc.fake_ferr_queue:
                payload = sc.fake_ferr_queue.pop(0)
            else:
                engine = sc.printer.lookup_object("motion_engine")
                _h, _frame, mass, viscous, coulomb, _ps, _ds = (
                    engine.dynamics_calls[-1]
                )
                payload = _ferr_json(
                    mass=sc.fake_coef_hints["mass"],
                    viscous=sc.fake_coef_hints["viscous"],
                    coulomb=sc.fake_coef_hints["coulomb"],
                    ferr_rms_raw=sc.fake_rms_fn(mass, viscous, coulomb),
                    onset_bias=sc.fake_onset_fn(mass, viscous, coulomb),
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


RLS = servo_calibration.RmsLineSearch


def drive(search, rms_fn, budget=50):
    for _ in range(budget):
        if search.done:
            return
        search.feed(rms_fn(search.trial))
    raise AssertionError("search did not finish in %d probes" % budget)


def test_line_search_marches_to_a_higher_minimum():
    def obj(v):
        return 1.0 + (v - 0.040) ** 2

    s = RLS(0.020, obj(0.020), step=0.003, tol=1e-9, hint=1.0)
    drive(s, obj)
    assert s.improved
    assert s.best == pytest.approx(0.040, rel=0.05)
    assert s.best_rms <= obj(0.020)


def test_line_search_flips_a_wrong_hint():
    def obj(v):
        return 1.0 + (v - 0.010) ** 2

    s = RLS(0.020, obj(0.020), step=0.002, tol=1e-9, hint=1.0)
    drive(s, obj)
    assert s.improved
    assert s.best == pytest.approx(0.010, rel=0.1)


def test_line_search_converges_at_start_when_already_optimal():
    def obj(v):
        return 1.0 + (v - 0.020) ** 2

    s = RLS(0.020, obj(0.020), step=0.003, tol=1e-9)
    drive(s, obj)
    assert s.best == pytest.approx(0.020, rel=0.06)
    # both first probes fail, the parabola may still polish the start
    assert len(s.history) <= 5


def test_line_search_ignores_improvement_below_tol():
    def obj(v):
        return 1.0 - 0.001 * v

    s = RLS(0.020, obj(0.020), step=0.003, tol=1.0)
    drive(s, obj)
    assert not s.improved
    assert s.best == 0.020


def test_line_search_at_floor_probes_up_despite_negative_hint():
    s = RLS(0.0, 1.0, step=5.0, tol=1e-9, lo=0.0, hint=-1.0)
    assert not s.done, "a floor start must earn at least one probe"
    assert s.trial == pytest.approx(5.0)
    s.feed(2.0)
    assert s.done
    assert s.best == 0.0
    assert not s.improved


def test_line_search_zero_start_probes_up_and_escapes():
    def obj(v):
        return 1.0 + (v - 10.0) ** 2 / 100.0

    s = RLS(0.0, obj(0.0), step=5.0, tol=1e-9, lo=0.0, hint=1.0)
    drive(s, obj)
    assert s.best == pytest.approx(10.0, rel=0.2)


def test_line_search_respects_lower_bound_mid_march():
    def obj(v):
        return 1.0 + v

    s = RLS(0.020, obj(0.020), step=0.05, tol=1e-9, lo=0.002, hint=-1.0)
    drive(s, obj)
    assert s.best == 0.002
    assert "bounded" in s.note or "no further" in s.note


def test_line_search_rejects_feed_without_trial():
    s = RLS(0.0, 1.0, step=5.0, tol=1e-9, lo=0.0, hint=-1.0)
    s.feed(2.0)
    assert s.done
    with pytest.raises(ValueError, match="without an outstanding trial"):
        s.feed(1.0)


@requires_tomllib
def test_tune_dynamics_already_optimal_converges_and_writes_baseline():
    sc, gcode, _path = make_calibration()
    engine = sc.printer.lookup_object("motion_engine")
    gcmd = FakeGcmd()
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    assert any("converged in" in r for r in gcmd.responses)
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["converged"] is True
    with open(tune["profile"], "rb") as f:
        prof = tomllib.load(f)
    assert prof["mass"] == pytest.approx(BASELINE_MASS, rel=0.06)
    assert prof["viscous"] == pytest.approx(BASELINE_VISCOUS, rel=0.06)
    assert prof["coulomb"] == pytest.approx(BASELINE_COULOMB, rel=0.06)
    # winner is streamed and left live
    _h, _f, mass, viscous, coulomb, _ps, _ds = engine.dynamics_calls[-1]
    assert mass == pytest.approx(prof["mass"])
    assert viscous == pytest.approx(prof["viscous"])
    assert coulomb == pytest.approx(prof["coulomb"])


@requires_tomllib
def test_tune_dynamics_finds_a_higher_mass_minimum():
    sc, _gcode, _path = make_calibration()
    sc.fake_rms_fn = quadratic_rms(mass_opt=[0.030, 0.045])
    gcmd = FakeGcmd(TERMS="MASS")
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    with open(tune["profile"], "rb") as f:
        prof = tomllib.load(f)
    assert prof["mass"][0] == pytest.approx(0.030, rel=0.15)
    assert prof["mass"][1] == pytest.approx(0.045, rel=0.15)
    assert prof["viscous"] == pytest.approx(BASELINE_VISCOUS)
    assert prof["coulomb"] == pytest.approx(BASELINE_COULOMB)


@requires_tomllib
def test_tune_dynamics_walks_mass_down_when_rms_says_lower():
    sc, _gcode, _path = make_calibration()
    sc.fake_rms_fn = quadratic_rms(mass_opt=[0.014, 0.022])
    # correlation hint says UP - exactly the bench pathology; rms must win
    sc.fake_coef_hints["mass"] = (2.5e-8, 2.7e-8)
    gcmd = FakeGcmd(TERMS="MASS")
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    with open(tune["profile"], "rb") as f:
        prof = tomllib.load(f)
    assert prof["mass"][0] == pytest.approx(0.014, rel=0.2)
    assert prof["mass"][1] == pytest.approx(0.022, rel=0.2)


@requires_tomllib
def test_tune_dynamics_torque_rail_aborts_and_restores_baseline():
    sc, _gcode, _path = make_calibration()
    engine = sc.printer.lookup_object("motion_engine")
    sc.fake_flags_by_step["tune_r0"] = ["torque_saturated"]
    with pytest.raises(RuntimeError, match="torque rail"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())
    _h, _f, mass, _v, _c, _ps, _ds = engine.dynamics_calls[-1]
    assert mass == pytest.approx(BASELINE_MASS)


@requires_tomllib
def test_tune_dynamics_resonance_flag_warns_and_continues():
    sc, _gcode, _path = make_calibration()
    sc.fake_flags_by_step["tune_r0"] = ["resonance_detected"]
    gcmd = FakeGcmd()
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    assert any("resonance_detected" in r for r in gcmd.responses)
    assert any("converged in" in r for r in gcmd.responses)


@requires_tomllib
def test_tune_dynamics_zero_coulomb_stays_bounded_when_worse():
    zero_coulomb = BASELINE_TOML.replace(
        "coulomb = [1.0, 1.5]", "coulomb = [0.0, 0.0]"
    )
    sc, _gcode, _path = make_calibration(profile_text=zero_coulomb)
    sc.fake_rms_fn = quadratic_rms(coulomb_opt=[0.0, 0.0])
    gcmd = FakeGcmd(TERMS="COULOMB")
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    with open(tune["profile"], "rb") as f:
        prof = tomllib.load(f)
    assert prof["coulomb"] == pytest.approx([0.0, 0.0], abs=1e-9)


@requires_tomllib
def test_tune_dynamics_zero_viscous_probes_off_the_floor():
    zero_viscous = BASELINE_TOML.replace(
        "viscous = [0.004, 0.005]", "viscous = [0.0, 0.0]"
    )
    sc, _gcode, _path = make_calibration(profile_text=zero_viscous)
    sc.fake_rms_fn = quadratic_rms(viscous_opt=[0.08, 0.10])
    gcmd = FakeGcmd(TERMS="VISCOUS")
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    with open(tune["profile"], "rb") as f:
        prof = tomllib.load(f)
    assert prof["viscous"][0] > 0.0
    assert prof["viscous"][1] > 0.0


@requires_tomllib
def test_tune_dynamics_reuses_measurements_instead_of_recapturing():
    sc, _gcode, _path = make_calibration()
    gcmd = FakeGcmd()
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    keys = [json.dumps(r["values"], sort_keys=True) for r in tune["rounds"]]
    assert len(keys) == len(set(keys)), "same model captured twice"


@requires_tomllib
def test_tune_dynamics_requires_ferr_rms_raw_from_the_binary():
    sc, _gcode, _path = make_calibration()
    stale = _ferr_json()
    del stale["ferr_rms_raw"]
    sc.fake_ferr_queue = [stale]
    with pytest.raises(RuntimeError, match="ferr_rms_raw"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


def test_tune_dynamics_rejects_non_coupled_kinematics():
    sc, _gcode, _path = make_calibration(
        rails=single_drive_rails(), coupled=False
    )
    with pytest.raises(RuntimeError, match="coupled_xy"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


def test_tune_dynamics_requires_a_baseline_profile():
    sc, _gcode, _path = make_calibration(configure_profile=False)
    with pytest.raises(RuntimeError, match="dynamics_profile|PROFILE"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


def test_tune_dynamics_rejects_excitation_matrix_params():
    sc, _gcode, _path = make_calibration()
    for params in ({"ACCELS": "5000"}, {"SPEEDS": "100"}, {"PATTERN": "1"}):
        with pytest.raises(RuntimeError):
            sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd(**params))


@requires_tomllib
def test_tune_dynamics_onset_bias_steers_the_first_mass_probe():
    sc, _gcode, _path = make_calibration()
    sc.fake_rms_fn = quadratic_rms(mass_opt=[0.014, 0.022])
    # regression coefficient claims under-fed (the bench pathology) but the
    # onset excursion says over-fed - the onset must win the direction call
    sc.fake_coef_hints["mass"] = (2.5e-8, 2.7e-8)
    sc.fake_onset_fn = lambda mass, viscous, coulomb: (
        -0.001 if mass[0] > 0.014 else 0.001,
        -0.001 if mass[1] > 0.022 else 0.001,
    )
    gcmd = FakeGcmd(TERMS="MASS")
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    first_probe = tune["rounds"][1]["values"]["mass"]
    assert first_probe[0] < BASELINE_MASS[0], "onset says down, probe went up"
    assert first_probe[1] < BASELINE_MASS[1]
    with open(tune["profile"], "rb") as f:
        prof = tomllib.load(f)
    assert prof["mass"][0] == pytest.approx(0.014, rel=0.2)
    assert prof["mass"][1] == pytest.approx(0.022, rel=0.2)


@requires_tomllib
def test_tune_dynamics_requires_onset_bias_from_the_binary():
    sc, _gcode, _path = make_calibration()
    stale = _ferr_json()
    del stale["onset_bias"]
    sc.fake_ferr_queue = [stale]
    with pytest.raises(RuntimeError, match="onset_bias"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())
