import tempfile

import pytest

from klippy.extras import servo_axis, servo_calibration


class FakeGcode:
    error = RuntimeError

    def __init__(self):
        self.scripts = []

    def register_command(self, name, func, desc=None):
        pass

    def run_script_from_command(self, script):
        self.scripts.append(script)


class FakeServoCapture:
    def __init__(self):
        self.captures = []

    def start_capture_to(self, path, servos):
        self.captures.append((path, list(servos)))

    def stop_capture(self):
        return self.captures[-1][0], 1000, 250


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def respond_info(self, msg):
        self.responses.append(msg)


class FakePrinter:
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


class FakeKin:
    kind = "corexy"

    def __init__(self, rails):
        self.rails = rails

    def coupled_xy(self):
        return True


class FakeToolhead:
    def __init__(self, kin):
        self._kin = kin

    def get_kinematics(self):
        return self._kin


class FakeNode:
    name = "ethercat_node n"

    def get_engine_handle(self):
        return 1

    def get_slot_for_motor(self, motor_name):
        return 0


class FakeEngine:
    def sdo_read(self, handle, slot, index, subindex):
        return 2, 7


def make_calibration():
    gcode = FakeGcode()
    motor = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    motor.motor_name = "motor_a"
    motor.node_name = "n"
    motor.chain_index = 0
    motor.invert_direction = False
    motor.rotation_distance = 40.0
    motor.encoder_counts_per_rev = 131072
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis x"
    rail.axis = "x"
    rail.motors = [motor]
    printer = FakePrinter(
        {
            "gcode": gcode,
            "servo_capture": FakeServoCapture(),
            "toolhead": FakeToolhead(FakeKin([rail])),
            "motion_engine": FakeEngine(),
            "ethercat_node n": FakeNode(),
        }
    )
    sc = servo_calibration.ServoCalibration(FakeConfig(printer))
    sc.captures_root = tempfile.mkdtemp()
    return sc, gcode


def with_active_run(sc):
    run = servo_calibration.ExperimentRun(
        tempfile.mkdtemp(), "20260710_000000", {"steps": []}
    )
    sc._active_run = run
    return run


def _writes(gcode, addr):
    out = []
    for s in gcode.scripts:
        for line in s.splitlines():
            if "SET=" + addr in line:
                out.append(line)
    return out


def test_single_gain_adapter_holds_other_two_fixed():
    sc, gcode = make_calibration()
    original = {"position": 400, "speed": 500, "integral": 2500}
    adapter = servo_calibration.SingleGainAdapter(
        sc, ["motor_a"], "speed", "refine", original, 500
    )
    swept, applied = adapter.apply(650)
    assert swept == {"speed": 650}
    values = {a["addr"]: a["value"] for a in applied}
    assert values["0x2001.0x01"] == 400
    assert values["0x2001.0x02"] == 650
    assert values["0x2001.0x03"] == 2500


def test_single_gain_adapter_revert_restores_original_triple():
    sc, gcode = make_calibration()
    original = {"position": 400, "speed": 500, "integral": 2500}
    adapter = servo_calibration.SingleGainAdapter(
        sc, ["motor_a"], "speed", "refine", original, 500
    )
    adapter.apply(650)
    adapter.revert()
    for addr, val in (
        ("0x2001.0x01", "400"),
        ("0x2001.0x02", "500"),
        ("0x2001.0x03", "2500"),
    ):
        assert _writes(gcode, addr)[-1].split("VALUE=")[1].split()[0] == val


def test_single_gain_adapter_describe_flags_the_current_value():
    sc, _ = make_calibration()
    original = {"position": 400, "speed": 500, "integral": 2500}
    adapter = servo_calibration.SingleGainAdapter(
        sc, ["motor_a"], "speed", "refine", original, 500
    )
    assert "<- current" in adapter.describe(0, 500, 3, ["motor_a"])
    assert "<- current" not in adapter.describe(1, 650, 3, ["motor_a"])


def test_inertia_ratio_adapter_apply_and_revert():
    sc, gcode = make_calibration()
    adapter = servo_calibration.InertiaRatioAdapter(
        sc, ["motor_a", "motor_b"], "inertia", 100
    )
    swept, applied = adapter.apply(150)
    assert swept == {"inertia_ratio": 150}
    assert applied == [
        {
            "servo": "motor_a",
            "addr": "0x2000.0x07",
            "type": "u16",
            "value": 150,
        },
        {
            "servo": "motor_b",
            "addr": "0x2000.0x07",
            "type": "u16",
            "value": 150,
        },
    ]
    adapter.revert()
    writes = _writes(gcode, "0x2000.0x07")
    assert writes[-1].split("VALUE=")[1].split()[0] == "100"
    assert "SERVO=motor_a " in writes[-2]
    assert "SERVO=motor_b " in writes[-1]


def test_motion_accel_adapter_writes_nothing():
    adapter = servo_calibration.MotionAccelAdapter("accel")
    assert adapter.step_name(5000) == "accel_a5000"
    swept, applied = adapter.apply(5000)
    assert swept == {"accel": 5000}
    assert applied == []
    adapter.revert()  # no-op, must not raise


def test_sweep_engine_runs_apply_capture_strokes_in_order():
    sc, gcode = make_calibration()
    run = with_active_run(sc)
    cap = sc.printer.lookup_object("servo_capture")
    adapter = servo_calibration.MotionAccelAdapter("accel")
    order = []
    steps = sc._engine.run(
        adapter,
        [1000, 2000],
        ["motor_a"],
        lambda v: order.append(("strokes", v)),
        FakeGcmd(),
    )
    assert [s.name for s in steps] == ["accel_a1000", "accel_a2000"]
    assert [s.swept for s in steps] == [{"accel": 1000}, {"accel": 2000}]
    assert [servos for _p, servos in cap.captures] == [
        ["motor_a"],
        ["motor_a"],
    ]
    assert [c["name"] for c in run.manifest["steps"]] == [
        "accel_a1000",
        "accel_a2000",
    ]


def test_sweep_engine_propagates_stroke_failures():
    sc, _ = make_calibration()
    with_active_run(sc)
    adapter = servo_calibration.MotionAccelAdapter("accel")

    def boom(v):
        raise RuntimeError("stroke exploded")

    with pytest.raises(RuntimeError, match="stroke exploded"):
        sc._engine.run(adapter, [1000], ["motor_a"], boom, FakeGcmd())


def test_run_sweep_with_revert_reverts_even_on_failure():
    sc, gcode = make_calibration()
    with_active_run(sc)
    adapter = servo_calibration.InertiaRatioAdapter(
        sc, ["motor_a"], "inertia", 100
    )
    reverted = []

    def boom(v):
        raise RuntimeError("stroke exploded")

    with pytest.raises(RuntimeError, match="stroke exploded"):
        sc._run_sweep_with_revert(
            adapter,
            [150],
            ["motor_a"],
            boom,
            FakeGcmd(),
            lambda: reverted.append(True),
        )
    assert reverted == [True]
    writes = _writes(gcode, "0x2000.0x07")
    assert writes[-1].split("VALUE=")[1].split()[0] == "100"
