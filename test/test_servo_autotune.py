import json
import os
import sys
import tempfile

import pytest

from klippy.extras import servo_axis, servo_calibration


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
    kind = "corexy"

    def __init__(self, rails):
        self.rails = rails

    def coupled_xy(self):
        return True


class FakeToolhead:
    def __init__(self, kin):
        self.kin = kin

    def get_kinematics(self):
        return self.kin


class FakeNode:
    def __init__(self, name, handle, slots):
        self.name = name
        self._handle = handle
        self._slots = slots

    def get_engine_handle(self):
        return self._handle

    def get_slot_for_motor(self, motor_name):
        return self._slots[motor_name]


class FakeEngine:
    def __init__(self, values=None):
        self._values = values or {}

    def sdo_read(self, handle, slot, index, subindex):
        return 2, self._values.get((index, subindex), 7)


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name):
        return self._objs[name]


class FakeConfig:
    def __init__(self, printer, values=None):
        self._printer = printer
        self._values = values or {}

    def get_printer(self):
        return self._printer

    def get(self, name, default=None):
        return self._values.get(name, default)

    def getlist(self, name, default=None):
        return self._values.get(name, default)

    def getfloat(self, name, default=None, **kw):
        return self._values.get(name, default)

    def getfloatlist(self, name, default=None):
        return self._values.get(name, default)

    def getint(self, name, default=None, **kw):
        return self._values.get(name, default)


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


def _applied(servo, addr, value):
    return {"servo": servo, "addr": addr, "type": "u16", "value": value}


INERTIA_RATIO_ADDR = servo_calibration.INERTIA_RATIO_ADDR
POS_ADDR, SPEED_ADDR, INTEGRAL_ADDR = (
    servo_calibration.GAIN_PARAMS["position"][0],
    servo_calibration.GAIN_PARAMS["speed"][0],
    servo_calibration.GAIN_PARAMS["integral"][0],
)

# The bench state a successful sequence settles on: C00.06=85%, gain-sweep
# picks speed=1000 (pos=1600, integral=1250) - one static readback map
# serves the whole chain.
GOOD_ENGINE_VALUES = {
    (0x2000, 0x07): 85,
    (0x2001, 0x01): 1600,
    (0x2001, 0x02): 1000,
    (0x2001, 0x03): 1250,
}


def _synth_results(run_dir, verdict, ferr_peak, overshoot, flagged_step=None):
    with open(os.path.join(run_dir, "manifest.json")) as f:
        manifest = json.load(f)
    motors = [m["name"] for m in manifest["motors"]]
    steps = []
    for s in manifest["steps"]:
        flags = ["torque_saturated"] if s["name"] == flagged_step else []
        steps.append(
            {
                "name": s["name"],
                "drives": {
                    m: {
                        "metrics": {
                            "moves": [
                                {"ferr_peak": ferr_peak, "overshoot": overshoot}
                            ]
                        }
                    }
                    for m in motors
                },
                "flags": flags,
            }
        )
    return {"verdict": verdict, "steps": steps}


def make_autotune(
    engine_values=None,
    gain_recommend_speed=1000,
    flagged_stage=None,
    baseline_ferr=100.0,
    verify_ferr=90.0,
    torque_nm=0.3,
    inertia_kgm2=1e-5,
):
    """One ServoCalibration wired for SERVO_AUTOTUNE: a single-drive-per-belt
    corexy layout (motor_a on X, motor_b on Y), a stubbed subprocess runner
    that fabricates `fit` stdout and `analyze` results.json content per
    stage, and a static SDO readback map consistent across every stage's
    write+verify (see GOOD_ENGINE_VALUES)."""
    gcode = FakeGcode()
    rails = [
        _rail("x", [_motor("motor_a", "n", 0)]),
        _rail("y", [_motor("motor_b", "n", 1, invert=True)]),
    ]
    node = FakeNode("ethercat_node n", 1, {"motor_a": 0, "motor_b": 1})
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails)),
        "servo_capture": FakeServoCapture(),
        "motion_engine": FakeEngine(engine_values or dict(GOOD_ENGINE_VALUES)),
        "ethercat_node n": node,
    }
    config = FakeConfig(
        FakePrinter(objs),
        values={
            "rated_torque_nm": torque_nm,
            "rotor_inertia_kgm2": inertia_kgm2,
        },
    )
    sc = servo_calibration.ServoCalibration(config)
    sc.captures_root = tempfile.mkdtemp()
    sc.dynamics_dir = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    pos = GOOD_ENGINE_VALUES[(0x2001, 0x01)]
    integral = GOOD_ENGINE_VALUES[(0x2001, 0x03)]
    gain_step_name = "autotune_gain_speed_v%d" % (gain_recommend_speed,)
    gain_verdict = {
        "recommended_step": gain_step_name,
        "reason": "highest gain step without resonance or torque rail",
        "flags": [],
        "apply": [
            _applied(servo, addr, value)
            for servo in ("motor_a", "motor_b")
            for addr, value in (
                (POS_ADDR, pos),
                (SPEED_ADDR, gain_recommend_speed),
                (INTEGRAL_ADDR, integral),
            )
        ],
    }
    no_pick_verdict = {
        "recommended_step": None,
        "reason": "not a sweep",
        "flags": [],
    }

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if argv[1] == "fit":
            return "recommended C00.06 (light direction): 85%\n"
        assert argv[1] == "analyze"
        run_dir = argv[2]
        base = os.path.basename(run_dir)
        if base.startswith("autotune_baseline_"):
            results = _synth_results(
                run_dir, no_pick_verdict, baseline_ferr, 5.0
            )
        elif base.startswith("autotune_verify_"):
            results = _synth_results(run_dir, no_pick_verdict, verify_ferr, 5.0)
        elif base.startswith("autotune_gain_"):
            flagged = gain_step_name if flagged_stage == "gain_sweep" else None
            results = _synth_results(
                run_dir, gain_verdict, 100.0, 10.0, flagged_step=flagged
            )
        elif base.startswith("verify_"):
            # SERVO_CALIBRATE_GAINS's own APPLY=1 nested
            # verification stroke - irrelevant to the autotune-level
            # before/after check, just needs to analyze cleanly.
            results = _synth_results(run_dir, no_pick_verdict, 40.0, 5.0)
        else:
            raise AssertionError("unexpected run_dir %r" % (run_dir,))
        with open(os.path.join(run_dir, "results.json"), "w") as f:
            json.dump(results, f)
        return ""

    sc._run = fake_run
    return sc, gcode


def _stage_names():
    return [s.name for s in servo_calibration.AUTOTUNE_STAGES]


def test_stage_order_matches_the_documented_sequence():
    assert _stage_names() == [
        "baseline",
        "inertia_ratio",
        "apply_inertia_ratio",
        "coarse_gains",
        "gain_sweep",
        "fit_dynamics",
        "verify",
    ]


def test_dry_run_walks_every_stage_without_persistent_writes():
    sc, gcode = make_autotune()
    gcmd = FakeGcmd(AXIS="X")
    outcomes = sc.cmd_SERVO_AUTOTUNE(gcmd)
    assert [o["stage"] for o in outcomes] == _stage_names()
    assert outcomes[0]["outcome"] == "ran"  # baseline
    assert outcomes[1]["outcome"] == "ran"  # inertia_ratio (identify only)
    assert outcomes[2]["outcome"] == "would_run"  # apply_inertia_ratio
    assert outcomes[3]["outcome"] == "would_run"  # coarse_gains
    assert outcomes[4]["outcome"] == "ran"  # gain_sweep (report-only)
    assert outcomes[5]["outcome"] == "would_run"  # fit_dynamics
    assert outcomes[6]["outcome"] == "skipped"  # verify

    scripts = [s for s in gcode.scripts if isinstance(s, str)]
    assert not any(
        "SET=%s VALUE=85" % (INERTIA_RATIO_ADDR,) in s for s in scripts
    )
    assert not any("SET=%s VALUE=400" % (POS_ADDR,) in s for s in scripts)
    assert any("SERVO_AUTOTUNE summary" in r for r in gcmd.responses)


def test_apply_run_succeeds_end_to_end_and_reminds_to_save():
    sc, gcode = make_autotune()
    gcmd = FakeGcmd(AXIS="X", APPLY=1)
    outcomes = sc.cmd_SERVO_AUTOTUNE(gcmd)
    assert [o["stage"] for o in outcomes] == _stage_names()
    assert all(o["outcome"] == "ran" for o in outcomes)
    assert outcomes[1]["recommended_ratio"] == 85
    assert outcomes[4]["recommended_step"].startswith("autotune_gain_")
    assert outcomes[5]["profile"]

    scripts = [s for s in gcode.scripts if isinstance(s, str)]
    assert any("SET=%s VALUE=85" % (INERTIA_RATIO_ADDR,) in s for s in scripts)
    assert any("SET=%s VALUE=400" % (POS_ADDR,) in s for s in scripts)
    assert any(
        "nothing persisted" in r and "SERVO_SAVE_TUNING" in r
        for r in gcmd.responses
    )


def test_apply_requires_torque_and_inertia_up_front():
    sc, _ = make_autotune(torque_nm=None, inertia_kgm2=None)
    with pytest.raises(RuntimeError, match="requires rated_torque_nm"):
        sc.cmd_SERVO_AUTOTUNE(FakeGcmd(AXIS="X", APPLY=1))


def test_flagged_gain_sweep_verdict_aborts_the_sequence():
    sc, _ = make_autotune(flagged_stage="gain_sweep")
    with pytest.raises(RuntimeError) as exc:
        sc.cmd_SERVO_AUTOTUNE(FakeGcmd(AXIS="X", APPLY=1))
    msg = str(exc.value)
    assert "aborting at stage 'gain_sweep'" in msg
    assert "torque_saturated" in msg
    assert "autotune_gain_" in msg  # names the run directory


def test_verification_regression_aborts():
    sc, _ = make_autotune(baseline_ferr=100.0, verify_ferr=200.0)
    with pytest.raises(RuntimeError) as exc:
        sc.cmd_SERVO_AUTOTUNE(FakeGcmd(AXIS="X", APPLY=1))
    msg = str(exc.value)
    assert "aborting at stage 'verify'" in msg
    assert "regressed" in msg


def test_structured_events_emitted_per_stage(monkeypatch):
    events = []

    def fake_event(subsystem, event, **fields):
        events.append((subsystem, event, fields))

    monkeypatch.setattr(servo_calibration.structured_log, "event", fake_event)
    sc, _ = make_autotune()
    sc.cmd_SERVO_AUTOTUNE(FakeGcmd(AXIS="X"))
    stage_events = [
        f
        for (sub, ev, f) in events
        if sub == "calibration" and ev == "autotune_stage"
    ]
    assert [f["stage"] for f in stage_events] == _stage_names()
    for f in stage_events:
        assert "outcome" in f and "run_dir" in f
