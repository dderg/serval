import json
import os
import sys
import tempfile

import pytest
from fakes import (
    FakeConfig,
    FakeGcode,
    FakeKin,
    FakeNode,
    FakeServoCapture,
    FakeToolhead,
)
from fakes import FakeEngine as _FakeEngine
from fakes import FakeGcmd as _FakeGcmd
from fakes import FakePrinter as _FakePrinter

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
ff_lead_us = 250.0
"""

NO_LEAD_BASELINE_TOML = """\
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


class FakeGcmd(_FakeGcmd):
    error = RuntimeError


class FakePrinter(_FakePrinter):
    command_error = RuntimeError


class FakeEngine(_FakeEngine):
    def __init__(self):
        super().__init__(sdo_read=(2, 7))
        self.dynamics_calls = []
        self.ff_lead_calls = []

    def set_dynamics_model(self, *args):
        self.dynamics_calls.append(args)

    def set_ff_lead(self, handle, slot, lead_ns):
        self.ff_lead_calls.append((handle, slot, lead_ns))


def _motor(name, node_name, chain_index, invert=False):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = name
    m.node_name = node_name
    m.chain_index = chain_index
    m.invert_direction = invert
    m.rotation_distance = 40.0
    m.encoder_counts_per_rev = 131072
    m.velocity_ff = True
    m.ff_max_torque = 30.0
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
    ff_sigma=(1e-9, 1e-9),
    ff_windows=(8, 8),
):
    ff = {
        term: {
            "rms": list(ferr_rms_raw),
            "sigma": list(ff_sigma),
            "windows": list(ff_windows),
        }
        for term in ("mass", "viscous", "coulomb", "lead")
    }
    return {
        "version": 3,
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
        "ferr_rms_ff": ff,
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
        "toolhead": FakeToolhead(FakeKin(rails=rails, coupled_xy=coupled)),
        "servo_capture": FakeServoCapture(),
        "motion_engine": FakeEngine(),
    }
    for node_name, slots in node_slots.items():
        objs["ethercat_node " + node_name] = FakeNode(
            name=node_name, slots=slots, handle=1, dynamics_profile=node_profile
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
    sc.fake_lead_penalty_fn = lambda lead_s: (0.0, 0.0)
    sc.fake_lead_onset_fn = lambda lead_s: (0.0, 0.0)

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
                lead_s = (
                    engine.ff_lead_calls[-1][2] * 1e-9
                    if engine.ff_lead_calls
                    else 250e-6
                )
                lead_pen = sc.fake_lead_penalty_fn(lead_s)
                lead_onset = sc.fake_lead_onset_fn(lead_s)
                payload = _ferr_json(
                    mass=sc.fake_coef_hints["mass"],
                    viscous=sc.fake_coef_hints["viscous"],
                    coulomb=sc.fake_coef_hints["coulomb"],
                    ferr_rms_raw=[
                        b + p
                        for b, p in zip(
                            sc.fake_rms_fn(mass, viscous, coulomb), lead_pen
                        )
                    ],
                    onset_bias=[
                        o + lo
                        for o, lo in zip(
                            sc.fake_onset_fn(mass, viscous, coulomb),
                            lead_onset,
                        )
                    ],
                )
            with open(out_path, "w") as f:
                json.dump(payload, f)
            return ""
        return ""

    sc._run = fake_run
    return sc, gcode, profile_path


def _manifest_for(sc):
    run_dir = os.path.dirname(
        sc.printer.lookup_object("servo_capture").starts[0][0]
    )
    with open(os.path.join(run_dir, "manifest.json")) as f:
        return json.load(f)


RLS = servo_calibration.RmsLineSearch


def drive(search, rms_fn, sigma=0.0, budget=50):
    for _ in range(budget):
        if search.done:
            return
        search.feed(rms_fn(search.trial), sigma)
    raise AssertionError("search did not finish in %d probes" % budget)


def test_line_search_marches_to_a_higher_minimum():
    def obj(v):
        return 1.0 + (v - 0.040) ** 2

    s = RLS(0.020, obj(0.020), sigma=0.0, step=0.003, hint=1.0)
    drive(s, obj)
    assert s.improved
    assert s.best == pytest.approx(0.040, rel=0.05)
    assert s.best_rms <= obj(0.020)


def test_line_search_flips_a_wrong_hint():
    def obj(v):
        return 1.0 + (v - 0.010) ** 2

    s = RLS(0.020, obj(0.020), sigma=0.0, step=0.002, hint=1.0)
    drive(s, obj)
    assert s.improved
    assert s.best == pytest.approx(0.010, rel=0.1)


def test_line_search_converges_at_start_when_already_optimal():
    def obj(v):
        return 1.0 + (v - 0.020) ** 2

    s = RLS(0.020, obj(0.020), sigma=0.0, step=0.003)
    drive(s, obj)
    assert s.best == pytest.approx(0.020, rel=0.06)
    # both first probes fail, the parabola may still polish the start
    assert len(s.history) <= 5


def test_line_search_deadband_rejects_sub_2sigma_but_accepts_beyond():
    # identical absolute improvement (0.010), different measured scatter:
    # the noisy probe sits inside the 2-sigma deadband and is rejected;
    # the clean probe clears it and wins - the whole point of dropping the
    # fixed TOL_UM knob for a measured-noise gate.
    noisy = RLS(1.0, 1.000, sigma=0.010, step=1.0, lo=0.0, hint=1.0)
    assert noisy.trial == pytest.approx(2.0)
    noisy.feed(0.990, 0.010)
    assert not noisy.improved
    assert noisy.best == 1.0

    clean = RLS(1.0, 1.000, sigma=0.0005, step=1.0, lo=0.0, hint=1.0)
    assert clean.trial == pytest.approx(2.0)
    clean.feed(0.990, 0.0005)
    assert clean.improved
    assert clean.best == pytest.approx(2.0)


def test_line_search_refine_polishes_between_one_and_two_sigma():
    # the incumbent trap: the true optimum sits between the march probes
    # and its improvement clears 1 sigma but not 2. The march gate (2
    # sigma) must reject the flanking probes, yet the bracket refine (1
    # sigma) must still claim the vertex instead of pinning the start
    # value forever. But sub-2-sigma polish must NOT count as `improved`
    # - the tune's outer loop repeats passes while anything improved, and
    # treating noise-level polish as improvement made the bench orbit a
    # single lead value for 50 captures.
    sigma = 0.012

    def obj(v):
        return 1.0 + 1e4 * (v - 0.0215) ** 2

    s = RLS(0.020, obj(0.020), sigma=sigma, step=0.003, hint=1.0)
    drive(s, obj, sigma=sigma)
    assert s.best == pytest.approx(0.0215, rel=0.01)
    assert s.note == "refined to the bracket minimum"
    assert not s.improved, "sub-2-sigma polish must not drive another pass"


def test_line_search_refine_probe_budget_is_bounded():
    # a refine probe that keeps producing fresh vertices must stop after
    # MAX_REFINE_PROBES captures instead of iterating forever.
    sigma = 0.012

    def obj(v):
        return 1.0 + 1e4 * (v - 0.02151234) ** 2 + 1e-7 * v

    s = RLS(0.020, obj(0.020), sigma=sigma, step=0.003, hint=1.0)
    drive(s, obj, sigma=sigma)
    assert s.done
    refine_probes = len(s.history) - 4
    assert refine_probes <= servo_calibration.search.MAX_REFINE_PROBES


def test_line_search_rejects_null_or_nan_sigma():
    with pytest.raises(ValueError, match="sigma"):
        RLS(0.0, 1.0, sigma=None, step=5.0)
    s = RLS(0.0, 1.0, sigma=0.0, step=5.0, hint=1.0)
    with pytest.raises(ValueError, match="sigma"):
        s.feed(0.5, float("nan"))


def test_line_search_at_floor_probes_up_despite_negative_hint():
    s = RLS(0.0, 1.0, sigma=0.0, step=5.0, lo=0.0, hint=-1.0)
    assert not s.done, "a floor start must earn at least one probe"
    assert s.trial == pytest.approx(5.0)
    s.feed(2.0, 0.0)
    assert s.done
    assert s.best == 0.0
    assert not s.improved


def test_line_search_zero_start_probes_up_and_escapes():
    def obj(v):
        return 1.0 + (v - 10.0) ** 2 / 100.0

    s = RLS(0.0, obj(0.0), sigma=0.0, step=5.0, lo=0.0, hint=1.0)
    drive(s, obj)
    assert s.best == pytest.approx(10.0, rel=0.2)


def test_line_search_respects_lower_bound_mid_march():
    def obj(v):
        return 1.0 + v

    s = RLS(0.020, obj(0.020), sigma=0.0, step=0.05, lo=0.002, hint=-1.0)
    drive(s, obj)
    assert s.best == 0.002
    assert "bounded" in s.note or "no further" in s.note


def test_line_search_rejects_feed_without_trial():
    s = RLS(0.0, 1.0, sigma=0.0, step=5.0, lo=0.0, hint=-1.0)
    s.feed(2.0, 0.0)
    assert s.done
    with pytest.raises(ValueError, match="without an outstanding trial"):
        s.feed(1.0, 0.0)


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
def test_tune_dynamics_chains_from_the_previous_tune_not_the_config():
    sc, _gcode, config_path = make_calibration()
    sc.fake_rms_fn = quadratic_rms(mass_opt=[0.030, 0.045])
    node = sc.printer.lookup_object("ethercat_node xy_drives")
    sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd(TERMS="MASS"))
    first = _manifest_for(sc)["dynamics_tune"]
    assert node.get_live_dynamics_profile() == first["profile"]
    assert first["profile"] != config_path
    sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd(TERMS="MASS", NAME="tune2"))
    run_dir = os.path.dirname(
        sc.printer.lookup_object("servo_capture").starts[-1][0]
    )
    with open(os.path.join(run_dir, "manifest.json")) as f:
        second = json.load(f)["dynamics_tune"]
    with open(first["profile"], "rb") as f:
        first_prof = tomllib.load(f)
    assert second["rounds"][0]["values"]["mass"] == pytest.approx(
        first_prof["mass"]
    ), "second tune must start from the first tune's live result"


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
    keys = [
        json.dumps((r["values"], r["lead_us"]), sort_keys=True)
        for r in tune["rounds"]
    ]
    assert len(keys) == len(set(keys)), "same model captured twice"


@requires_tomllib
def test_tune_dynamics_requires_ferr_rms_raw_from_the_binary():
    sc, _gcode, _path = make_calibration()
    stale = _ferr_json()
    del stale["ferr_rms_raw"]
    sc.fake_ferr_queue = [stale]
    with pytest.raises(RuntimeError, match="ferr_rms_raw"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


@requires_tomllib
def test_tune_dynamics_rejects_version_1_ferr_json():
    sc, _gcode, _path = make_calibration()
    stale = _ferr_json()
    stale["version"] = 1
    del stale["ferr_rms_ff"]
    sc.fake_ferr_queue = [stale]
    with pytest.raises(RuntimeError, match="version"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


@requires_tomllib
def test_tune_dynamics_requires_ferr_rms_ff_from_the_binary():
    sc, _gcode, _path = make_calibration()
    stale = _ferr_json()
    del stale["ferr_rms_ff"]
    sc.fake_ferr_queue = [stale]
    with pytest.raises(RuntimeError, match="ferr_rms_ff"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


@requires_tomllib
def test_tune_dynamics_fails_when_tuned_term_has_no_transient_windows():
    sc, _gcode, _path = make_calibration()
    stale = _ferr_json(ff_windows=(0, 0), ff_sigma=(None, None))
    for term in ("mass", "viscous", "coulomb"):
        stale["ferr_rms_ff"][term]["rms"] = [None, None]
    sc.fake_ferr_queue = [stale]
    with pytest.raises(RuntimeError, match="transient windows"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


@requires_tomllib
def test_tune_dynamics_fails_when_tuned_term_sigma_is_null():
    sc, _gcode, _path = make_calibration()
    stale = _ferr_json(ff_windows=(1, 1), ff_sigma=(None, None))
    sc.fake_ferr_queue = [stale]
    with pytest.raises(RuntimeError, match="sigma|scatter"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


@requires_tomllib
def test_tune_dynamics_ignores_removed_tol_um_param():
    sc, _gcode, _path = make_calibration()
    sc.fake_rms_fn = quadratic_rms(mass_opt=[0.030, 0.045])
    # TOL_UM used to gate acceptance and default to 0.05um; it is gone now.
    # A huge value must be silently ignored, not stop the search early.
    gcmd = FakeGcmd(TERMS="MASS", TOL_UM="999")
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    assert "tol_um" not in tune
    assert tune["objective"] == "transient_rms"
    assert tune["accept_z"] == pytest.approx(2.0)
    with open(tune["profile"], "rb") as f:
        prof = tomllib.load(f)
    assert prof["mass"][0] == pytest.approx(0.030, rel=0.15)
    assert prof["mass"][1] == pytest.approx(0.045, rel=0.15)


@requires_tomllib
def test_tune_dynamics_rejects_non_coupled_kinematics():
    sc, _gcode, _path = make_calibration(
        rails=single_drive_rails(), coupled=False
    )
    with pytest.raises(RuntimeError, match="coupled_xy"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd())


@requires_tomllib
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


def _lead_quadratic(opt_s, curvature=0.05, scale=250e-6):
    def penalty(lead_s):
        d2 = ((lead_s - opt_s) / scale) ** 2
        return (curvature * d2, curvature * d2)

    return penalty


@requires_tomllib
def test_tune_dynamics_lead_converges_to_the_rms_optimum():
    sc, _gcode, _path = make_calibration()
    engine = sc.printer.lookup_object("motion_engine")
    sc.fake_lead_penalty_fn = _lead_quadratic(375e-6)
    gcmd = FakeGcmd(TERMS="LEAD")
    sc.cmd_SERVO_TUNE_DYNAMICS(gcmd)
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["lead_us"] == pytest.approx(375.0, abs=40.0)
    # the winner is streamed to every slot and left live
    final = [c for c in engine.ff_lead_calls[-2:]]
    assert {slot for _h, slot, _ns in final} == {0, 1}
    assert all(
        ns == pytest.approx(tune["lead_us"] * 1e3, abs=1.0)
        for _h, _s, ns in final
    )
    assert any("carried in the tuned profile" in r for r in gcmd.responses)


@requires_tomllib
def test_tune_dynamics_lead_onset_steers_the_first_probe():
    sc, _gcode, _path = make_calibration()
    # negative onset = FF lands early = the first lead probe must go DOWN
    sc.fake_lead_penalty_fn = _lead_quadratic(150e-6)
    sc.fake_lead_onset_fn = lambda lead_s: (
        (-0.001, -0.001) if lead_s > 150e-6 else (0.001, 0.001)
    )
    sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd(TERMS="LEAD"))
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["rounds"][1]["lead_us"] < 250.0, (
        "onset says early, probe went up"
    )
    assert tune["lead_us"] == pytest.approx(150.0, abs=40.0)


@requires_tomllib
def test_tune_dynamics_abort_restores_the_configured_lead():
    sc, _gcode, _path = make_calibration()
    engine = sc.printer.lookup_object("motion_engine")
    sc.fake_flags_by_step["tune_r1"] = ["torque_saturated"]
    with pytest.raises(RuntimeError, match="torque rail"):
        sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd(TERMS="LEAD"))
    restored = engine.ff_lead_calls[-2:]
    assert {slot for _h, slot, _ns in restored} == {0, 1}
    assert all(ns == 250_000 for _h, _s, ns in restored)


@requires_tomllib
def test_tune_dynamics_lead_toml_matches_manifest():
    sc, _gcode, _path = make_calibration()
    sc.fake_lead_penalty_fn = _lead_quadratic(375e-6)
    sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd(TERMS="LEAD"))
    tune = _manifest_for(sc)["dynamics_tune"]
    with open(tune["profile"], "rb") as f:
        prof = tomllib.load(f)
    assert prof["ff_lead_us"] == pytest.approx(tune["lead_us"])


@requires_tomllib
def test_tune_dynamics_mass_only_passes_baseline_lead_through():
    sc, _gcode, _path = make_calibration()
    sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd(TERMS="MASS"))
    tune = _manifest_for(sc)["dynamics_tune"]
    with open(tune["profile"], "rb") as f:
        prof = tomllib.load(f)
    assert prof["ff_lead_us"] == pytest.approx(250.0)


@requires_tomllib
def test_tune_dynamics_lead_defaults_to_zero_when_profile_omits_it():
    sc, _gcode, _path = make_calibration(profile_text=NO_LEAD_BASELINE_TOML)
    sc.cmd_SERVO_TUNE_DYNAMICS(FakeGcmd(TERMS="LEAD"))
    tune = _manifest_for(sc)["dynamics_tune"]
    assert tune["rounds"][0]["lead_us"] == pytest.approx(0.0)
    assert tune["rounds"][1]["lead_us"] > 0.0
