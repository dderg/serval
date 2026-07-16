import json
import os
import sys
import tempfile

import pytest

from klippy.extras import servo_axis, servo_calibration, servo_param


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
        self.responses = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def run_script_from_command(self, script):
        self.scripts.append(script)

    def respond_info(self, msg):
        self.responses.append(msg)


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


class FakeReactor:
    def __init__(self):
        self._t = 0.0

    def monotonic(self):
        self._t += 0.001
        return self._t

    def pause(self, until):
        self._t = until


class FakePrinter:
    command_error = RuntimeError
    _sentinel = object()

    def __init__(self, objs):
        self._objs = objs
        self._reactor = FakeReactor()

    def get_reactor(self):
        return self._reactor

    def lookup_object(self, name, default=_sentinel):
        if name in self._objs:
            return self._objs[name]
        if default is not self._sentinel:
            return default
        raise KeyError(name)


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


def make_sc(handle=1, engine_values=None, verdict=None):
    gcode = FakeGcode()
    rails = [
        _rail("x", [_motor("motor_a", "n", 0)]),
        _rail("y", [_motor("motor_b", "n", 1, invert=True)]),
    ]
    node = FakeNode("ethercat_node n", handle, {"motor_a": 0, "motor_b": 1})
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(FakeKin(rails)),
        "servo_capture": FakeServoCapture(),
        "motion_engine": FakeEngine(engine_values),
        "ethercat_node n": node,
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    payload = (
        verdict
        if verdict is not None
        else {
            "recommended_step": "track",
            "reason": "highest clean step",
            "flags": [],
        }
    )

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if argv[1] == "analyze":
            with open(os.path.join(argv[2], "results.json"), "w") as f:
                json.dump({"verdict": payload}, f)

    sc._run = fake_run
    return sc, gcode


def _manifest(sc):
    run_dir = os.path.dirname(
        sc.printer.lookup_object("servo_capture").captures[0][0]
    )
    with open(os.path.join(run_dir, "manifest.json")) as f:
        return json.load(f)


def test_manifest_records_experiment_motors_belts_and_step():
    servo_param.drain_param_writes()
    sc, _ = make_sc()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    m = _manifest(sc)
    assert m["version"] == 1
    assert m["experiment"] == "tracking"
    assert m["command"] == "FAKE_CMD AXIS=X"
    assert m["axis"] == "X"
    assert m["kinematics"] == "corexy"
    assert m["git_rev"]
    assert m["session_id"]
    assert m["belts"] == "motor_a:1,motor_b:-1"
    assert m["motors"] == [
        {
            "name": "motor_a",
            "invert": False,
            "rotation_distance": 40.0,
            "counts_per_mm": 131072 / 40.0,
        },
        {
            "name": "motor_b",
            "invert": True,
            "rotation_distance": 40.0,
            "counts_per_mm": 131072 / 40.0,
        },
    ]
    assert [s["name"] for s in m["steps"]] == ["track"]
    assert m["steps"][0]["capture"] == "step_track.scap"
    assert m["steps"][0]["applied"] == []
    assert m["stroke_plan"]["speed"] == 100.0


def test_manifest_rewritten_after_each_step():
    run = servo_calibration.ExperimentRun(
        tempfile.mkdtemp(), "stamp", {"steps": []}
    )
    run.write()
    run.record_step(servo_calibration.SweepStep("a", {"accel": 1}, []))
    with open(run.manifest_path) as f:
        assert len(json.load(f)["steps"]) == 1
    run.record_step(servo_calibration.SweepStep("b", {"accel": 2}, []))
    with open(run.manifest_path) as f:
        after = json.load(f)
    assert [s["name"] for s in after["steps"]] == ["a", "b"]


def test_journal_params_are_read_back_into_ambient():
    servo_param.drain_param_writes()
    sc, _ = make_sc(engine_values={(0x2001, 0x31): 2})
    sc.journal_params = [("0x2001.0x31", "u16")]
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    journal = _manifest(sc)["ambient"]["journal_params"]
    assert journal["motor_a"]["0x2001.0x31"] == 2
    assert journal["motor_b"]["0x2001.0x31"] == 2


def test_journal_readback_failure_is_command_error():
    sc, _ = make_sc(handle=None)
    sc.journal_params = [("0x2001.0x31", "u16")]
    with pytest.raises(RuntimeError, match="journal_params readback failed"):
        sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))


def test_param_writes_since_last_run_are_drained():
    servo_param.drain_param_writes()
    servo_param.record_param_write("motor_a", "0x2001.0x31", 1)
    sc, _ = make_sc()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    writes = _manifest(sc)["ambient"]["param_writes_since_last_run"]
    assert len(writes) == 1
    assert writes[0]["servo"] == "motor_a"
    assert writes[0]["addr"] == "0x2001.0x31"
    assert writes[0]["value"] == 1
    # The drain emptied the log, so a second run records none.
    sc2, _ = make_sc()
    sc2.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    assert _manifest(sc2)["ambient"]["param_writes_since_last_run"] == []


def test_machinery_writes_are_suppressed_from_the_journal():
    servo_param.drain_param_writes()
    with servo_param.suppress_write_log():
        servo_param.record_param_write("motor_a", "0x2001.0x01", 880)
    servo_param.record_param_write("motor_a", "0x2001.0x31", 1)
    writes = servo_param.drain_param_writes()
    assert [(w["addr"], w["value"]) for w in writes] == [("0x2001.0x31", 1)]


def test_verdict_one_liner_names_step_and_run_dir():
    servo_param.drain_param_writes()
    sc, _ = make_sc(
        verdict={
            "recommended_step": "cal_p1280_s800_i1562",
            "reason": "highest clean gain",
            "flags": [],
        }
    )
    gcmd = FakeGcmd(AXIS="X")
    sc.cmd_SERVO_MEASURE_TRACKING(gcmd)
    run_dir = os.path.dirname(
        sc.printer.lookup_object("servo_capture").captures[0][0]
    )
    assert gcmd.responses == [
        "verdict: cal_p1280_s800_i1562 (highest clean gain) | run %s"
        % (run_dir,)
    ]


def test_verdict_one_liner_reports_no_step():
    servo_param.drain_param_writes()
    sc, _ = make_sc(
        verdict={"recommended_step": None, "reason": "not a sweep", "flags": []}
    )
    gcmd = FakeGcmd(AXIS="X")
    sc.cmd_SERVO_MEASURE_TRACKING(gcmd)
    assert len(gcmd.responses) == 1
    assert gcmd.responses[0].startswith("verdict: no step (not a sweep) | run ")


def test_missing_binary_is_command_error():
    servo_param.drain_param_writes()
    sc, _ = make_sc()
    sc.servo_cal_binary = "/nonexistent/servo-cal"
    with pytest.raises(
        RuntimeError, match="cargo build --release -p servo-ident"
    ):
        sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))


def _synth_results(run_dir, verdict, ferr_peak, overshoot, flags=()):
    with open(os.path.join(run_dir, "manifest.json")) as f:
        manifest = json.load(f)
    motors = [m["name"] for m in manifest["motors"]]
    steps = [
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
            "flags": list(flags)
            if s["name"] == verdict.get("recommended_step")
            else [],
        }
        for s in manifest["steps"]
    ]
    return {"verdict": verdict, "steps": steps}


def _applied(servo, addr, value):
    return {"servo": servo, "addr": addr, "type": "u16", "value": value}


def make_sc_apply(engine_values=None, verdict=None, verdict_flags=()):
    """Like make_sc, but the analyze stub writes a full results.json (steps
    with per-drive move metrics) so APPLY=1's before/after headline and
    readback verification have something real to read."""
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
        "motion_engine": FakeEngine(engine_values),
        "ethercat_node n": node,
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if argv[1] != "analyze":
            return
        run_dir = argv[2]
        is_verify = "verify_" in os.path.basename(run_dir)
        if is_verify:
            v = {"recommended_step": None, "reason": "not a sweep", "flags": []}
            results = _synth_results(run_dir, v, 40.0, 5.0)
        else:
            results = _synth_results(
                run_dir, verdict, 100.0, 10.0, flags=verdict_flags
            )
        with open(os.path.join(run_dir, "results.json"), "w") as f:
            json.dump(results, f)

    sc._run = fake_run
    return sc, gcode


def _script_indices(gcode, marker):
    return [
        i
        for i, s in enumerate(gcode.scripts)
        if isinstance(s, str) and marker in s
    ]


def test_apply_writes_after_revert_and_verifies():
    servo_param.drain_param_writes()
    apply_writes = [
        _applied(servo, addr, value)
        for servo in ("motor_a", "motor_b")
        for addr, value in (
            ("0x2001.0x01", 1040),
            ("0x2001.0x02", 650),
            ("0x2001.0x03", 1923),
        )
    ]
    sc, gcode = make_sc_apply(
        engine_values={
            (0x2001, 0x01): 1040,
            (0x2001, 0x02): 650,
            (0x2001, 0x03): 1923,
        },
        verdict={
            "recommended_step": "cal_p1040_s650_i1923",
            "reason": "highest clean gain",
            "flags": [],
            "apply": apply_writes,
        },
    )
    gcmd = FakeGcmd(AXIS="X", SPEED_GAINS="500,650", APPLY=1)
    sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)

    # sg=500 reverts to pos=800/speed=500/integral=2500 - the revert write
    # is the only place VALUE=2500 (integral) is ever written.
    revert_idx = _script_indices(gcode, "VALUE=2500")
    # sg=650 -> integral=1923 is written once mid-sweep and again by APPLY;
    # ordering only holds if APPLY runs after the revert, so it must be the
    # *last* occurrence that matters.
    win_idx = _script_indices(gcode, "VALUE=1923")
    assert revert_idx and win_idx
    assert max(win_idx) > max(revert_idx)
    assert any("APPLY verified" in r for r in gcmd.responses)


def test_apply_readback_mismatch_is_command_error():
    servo_param.drain_param_writes()
    apply_writes = [
        _applied("motor_a", "0x2001.0x02", 650),
        _applied("motor_b", "0x2001.0x02", 650),
    ]
    sc, gcode = make_sc_apply(
        engine_values={(0x2001, 0x02): 999},
        verdict={
            "recommended_step": "cal_p1040_s650_i1923",
            "reason": "highest clean gain",
            "flags": [],
            "apply": apply_writes,
        },
    )
    gcmd = FakeGcmd(AXIS="X", SPEED_GAINS="500,650", APPLY=1)
    with pytest.raises(RuntimeError, match="APPLY readback mismatch"):
        sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)


def test_apply_null_verdict_is_command_error_and_applies_nothing():
    servo_param.drain_param_writes()
    sc, gcode = make_sc_apply(
        verdict={
            "recommended_step": None,
            "reason": "every step flags resonance or a torque rail",
            "flags": [],
            "apply": None,
        }
    )
    gcmd = FakeGcmd(AXIS="X", SPEED_GAINS="500,650", APPLY=1)
    with pytest.raises(RuntimeError, match="nothing to apply"):
        sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)
    assert not any(
        isinstance(s, str) and "APPLY verified" in s for s in gcmd.responses
    )


def test_apply_default_off_does_not_apply():
    servo_param.drain_param_writes()
    sc, gcode = make_sc_apply(
        verdict={
            "recommended_step": "cal_p1040_s650_i1923",
            "reason": "highest clean gain",
            "flags": [],
            "apply": [_applied("motor_a", "0x2001.0x02", 650)],
        }
    )
    gcmd = FakeGcmd(AXIS="X", SPEED_GAINS="500,650")
    sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)
    assert not any(
        isinstance(r, str) and "APPLY verified" in r for r in gcmd.responses
    )


def test_base_speed_gain_pins_non_swept_servos():
    servo_param.drain_param_writes()
    sc, gcode = make_sc_apply(
        verdict={
            "recommended_step": "cal_p1040_s650_i1923",
            "reason": "highest clean gain",
            "flags": [],
            "apply": [_applied("motor_a", "0x2001.0x02", 650)],
        }
    )
    gcmd = FakeGcmd(
        AXIS="X", SERVO="motor_a", SPEED_GAINS="500,650", BASE_SPEED_GAIN="400"
    )
    sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)

    base_writes = [
        s
        for s in gcode.scripts
        if isinstance(s, str) and "SERVO=motor_b" in s and "0x2001" in s
    ]
    assert any("VALUE=640" in s for s in base_writes)
    assert any("VALUE=400" in s for s in base_writes)
    assert any("VALUE=3125" in s for s in base_writes)
    for s in gcode.scripts:
        if isinstance(s, str) and ("VALUE=650" in s or "VALUE=500" in s):
            assert "motor_b" not in s, "sweep must not touch the pinned servo"
    assert _manifest(sc)["base_gains"] == {
        "servos": ["motor_b"],
        "position": 640,
        "speed": 400,
        "integral": 3125,
    }


def test_base_speed_gain_without_servo_subset_errors():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc_apply(
        verdict={
            "recommended_step": "cal_p1040_s650_i1923",
            "reason": "highest clean gain",
            "flags": [],
            "apply": None,
        }
    )
    gcmd = FakeGcmd(AXIS="X", SPEED_GAINS="500,650", BASE_SPEED_GAIN="400")
    with pytest.raises(RuntimeError, match="subset"):
        sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)


def test_calibrate_gains_restores_prior_gains():
    servo_param.drain_param_writes()
    sc, gcode = make_sc()
    gcmd = FakeGcmd(AXIS="Y", SPEED_GAINS="450")
    sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)
    # derive(450) writes integral 2778 mid-sweep; the fake drive reads every
    # gain as 7, so the restore writes VALUE=7 to all three addresses after
    # the last sweep write.
    sweep_idx = _script_indices(gcode, "VALUE=2778")
    restore_idx = [
        i
        for i, s in enumerate(_str_scripts(gcode))
        if "SET=0x2001.0x03 VALUE=7" in s
    ]
    assert sweep_idx and restore_idx
    assert min(restore_idx) > max(sweep_idx)
    assert any("restoring the pre-sweep gains" in r for r in gcmd.responses)


def test_calibrate_gains_restores_on_failure():
    servo_param.drain_param_writes()
    sc, gcode = make_sc()

    def boom(*a, **k):
        raise RuntimeError("stroke exploded")

    sc._strokes = boom
    gcmd = FakeGcmd(AXIS="Y", SPEED_GAINS="450")
    with pytest.raises(RuntimeError, match="stroke exploded"):
        sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)
    scripts = _str_scripts(gcode)
    sweep_idx = [
        i for i, s in enumerate(scripts) if "SET=0x2001.0x03 VALUE=2778" in s
    ]
    restore_idx = [
        i for i, s in enumerate(scripts) if "SET=0x2001.0x03 VALUE=7" in s
    ]
    assert sweep_idx and restore_idx
    assert min(restore_idx) > max(sweep_idx)


def test_revert_gain_param_is_rejected():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    gcmd = FakeGcmd(AXIS="Y", SPEED_GAINS="450", REVERT_GAIN="300")
    with pytest.raises(RuntimeError, match="REVERT_GAIN was removed"):
        sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)


class FakeServoSync:
    def __init__(self):
        self.runs = []

    def run(self, gcmd, axis_filter=None, torque_ok_pct=None, settle=None):
        self.runs.append(axis_filter)


def test_strain_map_raster_records_one_capture_per_line():
    servo_param.drain_param_writes()
    sc, gcode = make_sc()
    sync = FakeServoSync()
    sc.printer._objs["servo_sync"] = sync
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    gcmd = FakeGcmd(LINE_SPACING="120", SPEED="50", ACCEL="1000")
    sc.cmd_SERVO_MEASURE_STRAIN_MAP(gcmd)
    assert sync.runs == [None]
    caps = sc.printer.lookup_object("servo_capture").captures
    names = [os.path.basename(path) for path, _servos in caps]
    assert names == [
        "step_xline_y030.scap",
        "step_xline_y150.scap",
        "step_xline_y270.scap",
        "step_yline_x030.scap",
        "step_yline_x150.scap",
        "step_yline_x270.scap",
    ]
    m = _manifest(sc)
    assert m["experiment"] == "strain_map"
    assert [s["name"] for s in m["steps"]] == [n[5:-5] for n in names]
    assert m["steps"][0]["swept"] == {"y": 30.0}
    assert m["stroke_plan"]["line_spacing"] == 120.0
    g1 = [
        line
        for script in gcode.scripts
        if isinstance(script, str)
        for line in script.split("\n")
        if line.startswith("G1 ")
    ]
    strokes = [line for line in g1 if "F3000" in line]
    assert len(strokes) == 12, "6 lines, each forward and back"
    m2 = _manifest(sc)
    assert m2["stroke_plan"]["zero_sync"] is True
    assert m2["stroke_plan"]["zero_xy"] == [150.0, 150.0]


def test_strain_map_without_servo_sync_errors_loudly():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    with pytest.raises(RuntimeError, match="servo_sync"):
        sc.cmd_SERVO_MEASURE_STRAIN_MAP(FakeGcmd(LINE_SPACING="120"))


def test_strain_map_sync_zero_skips_the_zero_point():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    gcmd = FakeGcmd(LINE_SPACING="120", SYNC="0")
    sc.cmd_SERVO_MEASURE_STRAIN_MAP(gcmd)
    assert _manifest(sc)["stroke_plan"]["zero_sync"] is False


def test_strain_map_rejects_cartesian_kinematics():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    sc.printer.lookup_object("toolhead").kin.coupled_xy = lambda: False
    with pytest.raises(RuntimeError, match="coupled XY"):
        sc.cmd_SERVO_MEASURE_STRAIN_MAP(FakeGcmd())


class FakeStrainComp:
    def __init__(self):
        self.applied = []
        self.cleared = 0
        self.fits = []

    def begin_constant_offsets(self, gcmd):
        comp = self

        class Session:
            def pair_count(self):
                return 2

            def pair_motor_names(self):
                return [["motor_a"], ["motor_b"]]

            def apply(self, belt_idx, value_um):
                comp.applied.append((belt_idx, value_um))
                return 0.0

            def clear(self):
                comp.cleared += 1

        return Session()

    def fit_strain_response(self, gcmd, run_dir):
        self.fits.append(run_dir)


def test_strain_response_steps_each_pair_along_one_line_and_fits():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.printer._objs["servo_sync"] = FakeServoSync()
    comp = FakeStrainComp()
    sc.printer._objs["servo_strain_comp"] = comp
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    sc.cmd_SERVO_MEASURE_STRAIN_RESPONSE(FakeGcmd(STEP_UM="50"))
    steps = [0.0, 50.0, -50.0, 100.0, -100.0]
    assert comp.applied == (
        [(0, v) for v in steps]
        + [(0, 0.0)]
        + [(1, v) for v in steps]
        + [(1, 0.0)]
    )
    assert comp.cleared == 1
    caps = sc.printer.lookup_object("servo_capture").captures
    names = [os.path.basename(path) for path, _servos in caps]
    assert names == [
        "step_belt%s_step%d.scap" % (belt, i) for belt in "ab" for i in range(5)
    ]
    m = _manifest(sc)
    assert m["experiment"] == "strain_response"
    assert m["stroke_plan"]["response_pairs"] == [["motor_a"], ["motor_b"]]
    assert m["stroke_plan"]["y"] == 150.0
    assert m["steps"][1]["swept"] == {"belt": 0.0, "offset_um": 50.0}
    assert comp.fits == [os.path.dirname(caps[0][0])]


def test_strain_response_without_strain_comp_errors_loudly():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    with pytest.raises(RuntimeError, match="servo_strain_comp"):
        sc.cmd_SERVO_MEASURE_STRAIN_RESPONSE(FakeGcmd())


NOTCH_VALUES = {
    (0x2001, 0x41): 111,
    (0x2001, 0x42): 1,
    (0x2001, 0x43): 2,
    (0x2001, 0x44): 222,
    (0x2001, 0x45): 3,
    (0x2001, 0x46): 4,
    (0x2001, 0x47): 333,
    (0x2001, 0x48): 5,
    (0x2001, 0x49): 6,
    (0x2001, 0x4A): 444,
    (0x2001, 0x4B): 8,
    (0x2001, 0x4C): 9,
    (0x2001, 0x4D): 555,
    (0x2001, 0x4E): 11,
    (0x2001, 0x4F): 12,
}


def _str_scripts(gcode):
    return [s for s in gcode.scripts if isinstance(s, str)]


def _g1_x_present(scripts):
    return any(ln.startswith("G1 X") for s in scripts for ln in s.splitlines())


def test_harvest_writes_mode_reads_back_and_locks():
    servo_param.drain_param_writes()
    sc, gcode = make_sc(engine_values=dict(NOTCH_VALUES))
    gcmd = FakeGcmd(AXIS="X", MODE=2)
    sc.cmd_SERVO_HARVEST_NOTCHES(gcmd)
    scripts = _str_scripts(gcode)
    for motor in ("motor_a", "motor_b"):
        assert any(
            "SERVO_PARAM SERVO=%s SET=0x2001.0x31 VALUE=2 TYPE=u16" % (motor,)
            in s
            for s in scripts
        )
    mode_idx = [
        i for i, s in enumerate(scripts) if "SET=0x2001.0x31 VALUE=2" in s
    ]
    lock_idx = [
        i for i, s in enumerate(scripts) if "SET=0x2001.0x31 VALUE=0" in s
    ]
    assert mode_idx and lock_idx
    assert min(lock_idx) > max(mode_idx)
    assert _g1_x_present(scripts)
    for motor in ("motor_a", "motor_b"):
        assert (
            "%s notch1 111 Hz w1 d2 | notch2 222 Hz w3 d4" % (motor,)
            in gcmd.responses
        )
    assert any("locked (C01.30 = 0)" in r for r in gcmd.responses)


def test_harvest_readback_failure_aborts_before_lock():
    servo_param.drain_param_writes()
    sc, gcode = make_sc(handle=None)
    gcmd = FakeGcmd(AXIS="X", MODE=2)
    with pytest.raises(RuntimeError, match="notch readback failed"):
        sc.cmd_SERVO_HARVEST_NOTCHES(gcmd)
    scripts = _str_scripts(gcode)
    assert any("SET=0x2001.0x31 VALUE=2" in s for s in scripts)
    assert not any("SET=0x2001.0x31 VALUE=0" in s for s in scripts)


def test_harvest_rejects_mode_3():
    sc, _ = make_sc()
    with pytest.raises(RuntimeError, match="MODE must be 1"):
        sc.cmd_SERVO_HARVEST_NOTCHES(FakeGcmd(AXIS="X", MODE=3))


def test_ambient_records_notch_state_per_drive():
    servo_param.drain_param_writes()
    sc, _ = make_sc(engine_values={**NOTCH_VALUES, (0x2001, 0x31): 1})
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    notches = _manifest(sc)["ambient"]["notches"]
    for motor in ("motor_a", "motor_b"):
        assert notches[motor] == {
            "mode": 1,
            "notch1": {"freq_hz": 111, "width": 1, "depth": 2},
            "notch2": {"freq_hz": 222, "width": 3, "depth": 4},
            "notch3": {"freq_hz": 333, "width": 5, "depth": 6},
            "notch4": {"freq_hz": 444, "width": 8, "depth": 9},
            "notch5": {"freq_hz": 555, "width": 11, "depth": 12},
        }


def test_ambient_notch_readback_failure_is_command_error():
    servo_param.drain_param_writes()
    sc, _ = make_sc(handle=None)
    with pytest.raises(RuntimeError, match="notch readback failed"):
        sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))


def make_sc_ladder(flagged_speed=None, flag="torque_saturated"):
    """Like make_sc, but the analyze stub writes a results.json with a per-step
    flags list (built from the manifest), flagging the step whose speed gain
    is `flagged_speed` so the ladder's incremental analysis stops the climb."""
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
        "motion_engine": FakeEngine(),
        "ethercat_node n": node,
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable
    sc._prep = lambda *a, **k: None
    sc._restore = lambda *a, **k: None

    flagged_step = None
    if flagged_speed is not None:
        pos, integral = servo_calibration.GainSetAdapter.derive(flagged_speed)
        flagged_step = "ladder_p%d_s%d_i%d" % (pos, flagged_speed, integral)

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if argv[1] != "analyze":
            return
        run_dir = argv[2]
        with open(os.path.join(run_dir, "manifest.json")) as f:
            manifest = json.load(f)
        recorded = manifest["steps"]
        steps = [
            {
                "name": s["name"],
                "flags": [flag] if s["name"] == flagged_step else [],
            }
            for s in recorded
        ]
        verdict = {
            "recommended_step": recorded[-1]["name"] if recorded else None,
            "reason": "highest clean gain",
            "flags": [],
        }
        with open(os.path.join(run_dir, "results.json"), "w") as f:
            json.dump({"verdict": verdict, "steps": steps}, f)

    sc._run = fake_run
    return sc, gcode


def _step_scap(sg):
    pos, integral = servo_calibration.GainSetAdapter.derive(sg)
    return "step_ladder_p%d_s%d_i%d.scap" % (pos, sg, integral)


def _capture_names(sc):
    return [
        os.path.basename(p)
        for p, _s in sc.printer.lookup_object("servo_capture").captures
    ]


def test_ladder_stops_climbing_after_flagged_rung():
    servo_param.drain_param_writes()
    # values = [SAFE=300, 500, 600, 700, 800]; flag the third rung (700).
    sc, gcode = make_sc_ladder(flagged_speed=700)
    gcmd = FakeGcmd(AXIS="X", SAFE=300, START=500, STEP=100, MAX=800)
    sc.cmd_SERVO_GAIN_LADDER(gcmd)
    names = _capture_names(sc)
    assert _step_scap(300) in names  # SAFE baseline
    assert _step_scap(700) in names  # flagged rung ran
    assert _step_scap(800) not in names  # fourth rung never executed
    assert any(
        "climb stopped at 700" in r and "torque_saturated" in r
        for r in gcmd.responses
    )
    assert any(r.startswith("verdict:") for r in gcmd.responses)


def test_ladder_restores_prior_gains_at_end():
    servo_param.drain_param_writes()
    sc, gcode = make_sc_ladder()  # no flag -> climbs the whole ladder
    gcmd = FakeGcmd(AXIS="X", SAFE=300, START=500, STEP=100, MAX=700)
    sc.cmd_SERVO_GAIN_LADDER(gcmd)
    scripts = _str_scripts(gcode)
    # The fake drive reads every gain as 7; the restore writes VALUE=7 to
    # all three addresses after the last climbing write (rung 700's
    # integral is 1786).
    climb_idx = [
        i for i, s in enumerate(scripts) if "SET=0x2001.0x03 VALUE=1786" in s
    ]
    restore_idx = [
        i for i, s in enumerate(scripts) if "SET=0x2001.0x03 VALUE=7" in s
    ]
    assert climb_idx and restore_idx
    assert min(restore_idx) > max(climb_idx)
    assert any("restoring the pre-ladder gains" in r for r in gcmd.responses)
    assert any(r.startswith("verdict:") for r in gcmd.responses)


def test_ladder_single_param_climbs_position_and_holds_the_rest():
    servo_param.drain_param_writes()
    sc, gcode = make_sc_ladder()
    gcmd = FakeGcmd(
        AXIS="X", PARAM="position", SAFE=400, START=600, STEP=200, MAX=800
    )
    sc.cmd_SERVO_GAIN_LADDER(gcmd)
    names = _capture_names(sc)
    assert "step_ladder_position_v400.scap" in names
    assert "step_ladder_position_v800.scap" in names
    scripts = _str_scripts(gcode)
    rung = [
        s
        for s in scripts
        if "SET=0x2001.0x01 VALUE=800" in s or "VALUE=800" in s
    ]
    assert rung, "position rung 800 never written"
    held = [
        s
        for s in scripts
        if "SET=0x2001.0x02 VALUE=7" in s and "SET=0x2001.0x03 VALUE=7" in s
    ]
    assert held, "speed/integral must hold the pre-ladder value during rungs"
    assert any("restoring the pre-ladder gains" in r for r in gcmd.responses)


def test_ladder_single_param_rejects_bad_param():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc_ladder()
    gcmd = FakeGcmd(
        AXIS="X", PARAM="stiffness", SAFE=400, START=600, STEP=200, MAX=800
    )
    with pytest.raises(RuntimeError, match="PARAM must be"):
        sc.cmd_SERVO_GAIN_LADDER(gcmd)


def test_ladder_rejects_nonpositive_step():
    sc, _ = make_sc_ladder()
    with pytest.raises(RuntimeError, match="STEP must be > 0"):
        sc.cmd_SERVO_GAIN_LADDER(
            FakeGcmd(AXIS="X", SAFE=300, START=500, STEP=0, MAX=800)
        )


def test_ladder_rejects_max_below_start():
    sc, _ = make_sc_ladder()
    with pytest.raises(RuntimeError, match="must be >= START"):
        sc.cmd_SERVO_GAIN_LADDER(
            FakeGcmd(AXIS="X", SAFE=300, START=800, STEP=100, MAX=500)
        )


def test_load_config_pulls_in_servo_tuning():
    rails = [_rail("x", [_motor("motor_a", "n", 0)])]
    objs = {
        "gcode": FakeGcode(),
        "toolhead": FakeToolhead(FakeKin(rails)),
        "servo_capture": FakeServoCapture(),
        "motion_engine": FakeEngine(),
        "ethercat_node n": FakeNode("ethercat_node n", 1, {"motor_a": 0}),
    }

    class RecordingPrinter(FakePrinter):
        def __init__(self, inner_objs):
            super().__init__(inner_objs)
            self.loaded = []

        def load_object(self, config, section):
            self.loaded.append(section)

    printer = RecordingPrinter(objs)
    servo_calibration.load_config(FakeConfig(printer))
    assert printer.loaded == ["servo_tuning"]


class FakeTuner:
    def __init__(self, rho_seq):
        self.plan = {
            "x_start": 30.0,
            "x_end": 270.0,
            "y_start": 30.0,
            "y_end": 270.0,
            "zero_xy": [150.0, 150.0],
            "line_spacing": 10.0,
        }
        self.k_matrix = [[300.0, -75.0], [-75.0, 300.0]]
        self.rho_seq = list(rho_seq)
        self.rebuilds = 0
        self.applied = []
        self.stored = 0
        self.scored = []

    def matrix_rows(self):
        return [list(row) for row in self.k_matrix]

    def enable_ramp_s(self):
        return 0.0

    def rebuild_and_enable(self, gcmd):
        self.rebuilds += 1

    def score_lines(self, gcmd, run_dir, steps):
        self.scored.append((run_dir, list(steps)))
        rho = self.rho_seq.pop(0)
        return [
            {
                "s_own": 300.0 * rho,
                "s_cross": -75.0 * rho,
                "rho": rho,
                "lines": {"x": (1.0, 9.0), "y": (1.5, 8.0)},
            },
            {
                "s_own": 300.0 * rho,
                "s_cross": -75.0 * rho,
                "rho": rho,
                "lines": {"x": (1.0, 8.0), "y": (1.5, 7.0)},
            },
        ]

    def converged(self, results, tol):
        return all(abs(r["rho"] - 1.0) <= tol for r in results)

    def apply(self, results):
        self.applied.append([r["rho"] for r in results])

    def store_matrix(self):
        self.stored += 1


def make_tune_sc(rho_seq):
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.printer._objs["servo_sync"] = FakeServoSync()
    comp = FakeStrainComp()
    comp.tuner = FakeTuner(rho_seq)
    comp.begin_tune = lambda gcmd, run_raw, spacing: comp.tuner
    sc.printer._objs["servo_strain_comp"] = comp
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    return sc, comp


def test_tune_loops_xy_lines_until_converged():
    sc, comp = make_tune_sc([0.66, 1.01])
    sc.cmd_SERVO_STRAIN_COMP_TUNE(FakeGcmd(RUN="ignored"))
    tuner = comp.tuner
    assert tuner.rebuilds == 2
    assert tuner.applied == [[0.66, 0.66]]
    assert tuner.stored == 1
    caps = sc.printer.lookup_object("servo_capture").captures
    names = [os.path.basename(path) for path, _servos in caps]
    assert names == [
        "step_iter0_x.scap",
        "step_iter0_y.scap",
        "step_iter1_x.scap",
        "step_iter1_y.scap",
    ]
    assert tuner.scored[0][1] == [
        ("iter0_x", "y", 150.0),
        ("iter0_y", "x", 150.0),
    ]
    m = _manifest(sc)
    assert m["experiment"] == "strain_tune"
    # The dashboard's manifest parser types `applied` strictly (servo
    # param writes); diagnostics ride in the free-form `swept`.
    assert m["steps"][0]["applied"] == []
    assert m["steps"][0]["swept"]["s_own_a"] == pytest.approx(300.0 * 0.66)
    assert m["steps"][0]["swept"]["s_cross_a"] == pytest.approx(-75.0 * 0.66)
    assert m["steps"][0]["swept"]["rms_a_x"] == 1.0
    assert m["steps"][0]["swept"]["rms_a_y"] == 1.5
    assert m["steps"][0]["swept"]["kaa"] == 300.0
    assert m["steps"][0]["swept"]["y"] == 150.0
    assert m["steps"][0]["swept"]["x"] == 150.0


def test_tune_fails_loudly_when_it_does_not_converge():
    sc, comp = make_tune_sc([0.5] * 5)
    with pytest.raises(RuntimeError, match="did not converge"):
        sc.cmd_SERVO_STRAIN_COMP_TUNE(FakeGcmd(RUN="ignored"))
    assert comp.tuner.stored == 0


def test_tune_without_strain_comp_errors_loudly():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    with pytest.raises(RuntimeError, match="servo_strain_comp"):
        sc.cmd_SERVO_STRAIN_COMP_TUNE(FakeGcmd(RUN="ignored"))


def test_capture_warns_when_sync_loss_counter_increments():
    servo_param.drain_param_writes()
    sc, gcode = make_sc()
    engine = sc.printer.lookup_object("motion_engine")
    reads = {"n": 0}

    def sdo_read(handle, slot, index, subindex):
        if (index, subindex) == (0x2013, 0x05):
            reads["n"] += 1
            return 2, reads["n"]
        return 2, 7

    engine.sdo_read = sdo_read
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    warns = [r for r in gcode.responses if "sync loss" in r]
    assert warns, gcode.responses
    assert "C13.04" in warns[0]
    assert "motor_a +2" in warns[0]
    assert "motor_b +2" in warns[0]


def test_capture_quiet_when_sync_loss_counter_steady():
    servo_param.drain_param_writes()
    sc, gcode = make_sc()
    sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
    assert not any("sync loss" in r for r in gcode.responses)


def test_capture_sync_loss_read_failure_is_command_error():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    engine = sc.printer.lookup_object("motion_engine")

    def sdo_read(handle, slot, index, subindex):
        if (index, subindex) == (0x2013, 0x05):
            raise RuntimeError("SDO read failed: CoE abort 0x06020000")
        return 2, 7

    engine.sdo_read = sdo_read
    with pytest.raises(RuntimeError, match="C13.04"):
        sc.cmd_SERVO_MEASURE_TRACKING(FakeGcmd(AXIS="X"))
