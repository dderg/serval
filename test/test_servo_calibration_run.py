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
