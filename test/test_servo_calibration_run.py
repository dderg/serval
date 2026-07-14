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
    _sentinel = object()

    def __init__(self, objs):
        self._objs = objs

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


def test_revert_gain_reverts_to_the_named_gain():
    servo_param.drain_param_writes()
    sc, gcode = make_sc()
    gcmd = FakeGcmd(AXIS="Y", SPEED_GAINS="450", REVERT_GAIN="300")
    sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)
    # derive(450) writes integral 2778 mid-sweep; derive(300) writes
    # pos 480 / speed 300 / integral 4167 - only the revert writes those.
    sweep_idx = _script_indices(gcode, "VALUE=2778")
    revert_idx = _script_indices(gcode, "VALUE=4167")
    assert sweep_idx and revert_idx
    assert min(revert_idx) > max(sweep_idx)
    assert any("VALUE=480" in s for s in _str_scripts(gcode))
    assert any("reverting to speed gain 30.0 Hz" in r for r in gcmd.responses)


def test_revert_gain_defaults_to_lowest_sweep_entry():
    servo_param.drain_param_writes()
    sc, gcode = make_sc()
    gcmd = FakeGcmd(AXIS="Y", SPEED_GAINS="500,650")
    sc.cmd_SERVO_CALIBRATE_GAINS(gcmd)
    # sg=500's integral 2500 is written at step one and again by the revert,
    # after sg=650's 1923.
    revert_idx = _script_indices(gcode, "VALUE=2500")
    high_idx = _script_indices(gcode, "VALUE=1923")
    assert revert_idx and high_idx
    assert max(revert_idx) > max(high_idx)
    assert any("reverting to speed gain 50.0 Hz" in r for r in gcmd.responses)


def test_revert_gain_out_of_range_is_command_error():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    gcmd = FakeGcmd(AXIS="Y", SPEED_GAINS="450", REVERT_GAIN="50")
    with pytest.raises(RuntimeError, match="REVERT_GAIN 50 outside"):
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
    gcmd = FakeGcmd(LINE_SPACING="120", SPEED="50", ACCEL="1000", PROBE="0")
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
        sc.cmd_SERVO_MEASURE_STRAIN_MAP(FakeGcmd(LINE_SPACING="120", PROBE="0"))


def test_strain_map_sync_zero_skips_the_zero_point():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    gcmd = FakeGcmd(LINE_SPACING="120", SYNC="0", PROBE="0")
    sc.cmd_SERVO_MEASURE_STRAIN_MAP(gcmd)
    assert _manifest(sc)["stroke_plan"]["zero_sync"] is False


class FakeBeltPair:
    def __init__(self, axis, motors):
        self._axis = axis
        self._motors = motors

    def axis_name(self):
        return self._axis

    def motor_names(self):
        return self._motors


class FakeStrainComp:
    def __init__(self):
        self.offsets = []
        self.cleared = 0
        self.pairs = [
            FakeBeltPair("x", ["motor_a", "motor_a1"]),
            FakeBeltPair("y", ["motor_b", "motor_b1"]),
        ]

    def belt_pairs_for_probe(self, gcmd):
        return self.pairs

    def set_probe_offset(self, gcmd, pair, value_um):
        self.offsets.append((pair.axis_name(), value_um))

    def clear_probe_offset(self, gcmd, pair):
        self.cleared += 1


def test_strain_map_probe_lines_record_the_applied_offsets():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.printer._objs["servo_sync"] = FakeServoSync()
    strain_comp = FakeStrainComp()
    sc.printer._objs["servo_strain_comp"] = strain_comp
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    gcmd = FakeGcmd(LINE_SPACING="120")
    sc.cmd_SERVO_MEASURE_STRAIN_MAP(gcmd)
    assert strain_comp.offsets == [
        ("x", 50.0),
        ("x", -50.0),
        ("y", 50.0),
        ("y", -50.0),
    ]
    assert strain_comp.cleared == 4
    m = _manifest(sc)
    probes = [s for s in m["steps"] if s["applied"]]
    assert [s["name"] for s in probes] == [
        "probe_x_plus",
        "probe_x_minus",
        "probe_y_plus",
        "probe_y_minus",
    ]
    assert probes[0]["swept"] == {"y": 150.0}
    assert probes[0]["applied"] == [
        {"motors": ["motor_a", "motor_a1"], "offset_um": 50.0}
    ]
    caps = sc.printer.lookup_object("servo_capture").captures
    names = [os.path.basename(path) for path, _servos in caps]
    assert names[-4:] == [
        "step_probe_x_plus.scap",
        "step_probe_x_minus.scap",
        "step_probe_y_plus.scap",
        "step_probe_y_minus.scap",
    ]


def test_strain_map_probe_without_strain_comp_errors_loudly():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.printer._objs["servo_sync"] = FakeServoSync()
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    with pytest.raises(RuntimeError, match="PROBE=0"):
        sc.cmd_SERVO_MEASURE_STRAIN_MAP(FakeGcmd(LINE_SPACING="120"))


def test_strain_map_rejects_cartesian_kinematics():
    servo_param.drain_param_writes()
    sc, _gcode = make_sc()
    sc.bounds = {"X": (30.0, 270.0), "Y": (30.0, 270.0)}
    sc.printer.lookup_object("toolhead").kin.coupled_xy = lambda: False
    with pytest.raises(RuntimeError, match="coupled XY"):
        sc.cmd_SERVO_MEASURE_STRAIN_MAP(FakeGcmd())


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
        "climb stopped at speed gain 700" in r and "torque_saturated" in r
        for r in gcmd.responses
    )
    assert any(r.startswith("verdict:") for r in gcmd.responses)


def test_ladder_applies_safe_at_end():
    servo_param.drain_param_writes()
    sc, gcode = make_sc_ladder()  # no flag -> climbs the whole ladder
    gcmd = FakeGcmd(AXIS="X", SAFE=300, START=500, STEP=100, MAX=700)
    sc.cmd_SERVO_GAIN_LADDER(gcmd)
    scripts = _str_scripts(gcode)
    # SAFE integral = round(1250000/300) = 4167, unique to the SAFE rung; a
    # climbing rung (500) has integral 2500. The final SAFE application must
    # land after every climbing write.
    safe_idx = [
        i for i, s in enumerate(scripts) if "SET=0x2001.0x03 VALUE=4167" in s
    ]
    climb_idx = [
        i for i, s in enumerate(scripts) if "SET=0x2001.0x03 VALUE=2500" in s
    ]
    assert safe_idx and climb_idx
    assert max(safe_idx) > max(climb_idx)
    assert any("SAFE speed gain 300 applied" in r for r in gcmd.responses)
    assert any(r.startswith("verdict:") for r in gcmd.responses)


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
