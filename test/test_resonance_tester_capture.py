import os

import pytest
from fakes import (
    FakeConfig,
    FakeGcmd,
    FakeGcode,
    FakeKin,
    FakePrinter,
    FakeServoCapture,
    FakeToolhead,
)

from klippy.extras import resonance_tester


class FakeAclient:
    def __init__(self):
        self.finished = False
        self.samples = [(0.0, 1.0, 2.0, 3.0), (0.001, 4.0, 5.0, 6.0)]

    def finish_measurements(self):
        self.finished = True

    def has_valid_samples(self):
        return True

    def get_samples(self):
        return self.samples


class FakeChip:
    def __init__(self, name):
        self.name = name
        self.clients = []

    def start_internal_client(self):
        client = FakeAclient()
        self.clients.append(client)
        return client


class FakeBuzz:
    def __init__(self):
        self.calls = []
        self.events = None

    def run_sweep(self, gcmd, axis_name, *args):
        self.calls.append((axis_name, args))
        if self.events is not None:
            self.events.append("buzz")
        return args[5]


class FakeServoMotor:
    def __init__(self, name):
        self._name = name

    def get_motor_name(self):
        return self._name


def _servo_rail(axis, motor_names):
    from klippy.extras.servo_axis import ServoRail

    rail = ServoRail.__new__(ServoRail)
    rail.axis = axis
    rail.motors = [FakeServoMotor(n) for n in motor_names]
    return rail


def make_tester(values=None):
    printer = FakePrinter()
    gcode = FakeGcode()
    events = []
    buzz = FakeBuzz()
    buzz.events = events
    toolhead = FakeToolhead()
    printer.objects["gcode"] = gcode
    printer.objects["resonance_buzz"] = buzz
    printer.objects["servo_capture"] = FakeServoCapture(events)
    printer.objects["toolhead"] = toolhead
    cfg_values = {"accel_chip": "adxl345"}
    if values:
        cfg_values.update(values)
    tester = resonance_tester.ResonanceTester(
        FakeConfig(printer, values=cfg_values)
    )
    return tester, printer, buzz, toolhead


def test_registers_commands():
    _, printer, _, _ = make_tester()
    gcode = printer.objects["gcode"]
    assert "TEST_RESONANCES" in gcode.commands
    assert "SHAPER_CALIBRATE" in gcode.commands
    assert "MEASURE_AXES_NOISE" in gcode.commands


def test_parse_axis_cardinal():
    gcmd = FakeGcmd(error=RuntimeError)
    for name in ("x", "y", "z", "X", "Z"):
        axis = resonance_tester._parse_axis(gcmd, name)
        assert axis.buzz_axis() == name.lower()


def test_parse_axis_pure_vectors_map_to_named_axes():
    gcmd = FakeGcmd(error=RuntimeError)
    assert resonance_tester._parse_axis(gcmd, "1,0").buzz_axis() == "x"
    assert resonance_tester._parse_axis(gcmd, "0,1").buzz_axis() == "y"


def test_parse_axis_diagonal_not_implemented():
    gcmd = FakeGcmd(error=RuntimeError)
    with pytest.raises(RuntimeError, match="diagonal buzz is not implemented"):
        resonance_tester._parse_axis(gcmd, "1,1")


def test_parse_axis_bad_format():
    gcmd = FakeGcmd(error=RuntimeError)
    with pytest.raises(RuntimeError, match="Invalid format"):
        resonance_tester._parse_axis(gcmd, "bogus")


def test_parse_axis_missing_is_required():
    gcmd = FakeGcmd(error=RuntimeError)
    with pytest.raises(RuntimeError, match="AXIS parameter is required"):
        resonance_tester._parse_axis(gcmd, None)


def test_run_test_excites_axis_and_captures():
    tester, printer, buzz, toolhead = make_tester()
    chip = FakeChip("adxl345")
    tester.accel_chips = [("xy", chip)]
    gcmd = FakeGcmd(error=RuntimeError)
    sweep = tester._parse_sweep(gcmd)
    axes = [resonance_tester.TestAxis("x")]

    tester._run_test(gcmd, axes, None, sweep)

    assert len(buzz.calls) == 1
    assert buzz.calls[0][0] == "x"
    assert len(chip.clients) == 1
    assert chip.clients[0].finished


def test_run_test_skips_chip_that_does_not_match_axis():
    tester, printer, buzz, toolhead = make_tester()
    x_chip = FakeChip("adxl_x")
    tester.accel_chips = [("x", x_chip)]
    gcmd = FakeGcmd(error=RuntimeError)
    sweep = tester._parse_sweep(gcmd)

    tester._run_test(gcmd, [resonance_tester.TestAxis("y")], None, sweep)

    assert buzz.calls and buzz.calls[0][0] == "y"
    assert x_chip.clients == []


def test_run_test_writes_raw_data_when_named():
    tester, printer, buzz, toolhead = make_tester()
    chip = FakeChip("adxl345")
    tester.accel_chips = [("xy", chip)]
    gcmd = FakeGcmd(error=RuntimeError)
    sweep = tester._parse_sweep(gcmd)

    tester._run_test(
        gcmd,
        [resonance_tester.TestAxis("x")],
        None,
        sweep,
        raw_name_suffix="probe1",
    )

    raw_name = "/tmp/raw_data_x_adxl345_probe1.csv"
    with open(raw_name) as f:
        lines = f.read().splitlines()
    os.remove(raw_name)
    assert lines[0].startswith("# chirp freq_start=")
    assert "accel_per_hz=" in lines[0]
    assert "graph_max_freq" not in lines[0]
    assert lines[1] == "#time,accel_x,accel_y,accel_z"
    assert len(lines) == 2 + len(chip.clients[0].samples)
    assert any(r.startswith("Raw accelerometer data") for r in gcmd.responses)


def test_raw_data_header_carries_graph_max_freq():
    tester, printer, buzz, toolhead = make_tester(
        values={"graph_max_freq": 450.0}
    )
    chip = FakeChip("adxl345")
    tester.accel_chips = [("xy", chip)]
    gcmd = FakeGcmd(error=RuntimeError)
    sweep = tester._parse_sweep(gcmd)

    tester._run_test(
        gcmd,
        [resonance_tester.TestAxis("x")],
        None,
        sweep,
        raw_name_suffix="gmf1",
    )

    raw_name = "/tmp/raw_data_x_adxl345_gmf1.csv"
    with open(raw_name) as f:
        header = f.readline().strip()
    os.remove(raw_name)
    assert header.endswith("graph_max_freq=450.0")


def test_run_test_brackets_servo_capture_around_buzz():
    tester, printer, buzz, toolhead = make_tester()
    chip = FakeChip("beacon")
    tester.accel_chips = [("xy", chip)]
    rails = [
        _servo_rail("x", ["motor_a", "motor_a1"]),
        _servo_rail("y", ["motor_b", "motor_b1"]),
    ]
    toolhead.kin = FakeKin(
        rails=rails,
        lanes=[(i, rail.axis, rail.motors) for i, rail in enumerate(rails)],
        coupled_xy=True,
    )
    scap = printer.objects["servo_capture"]
    gcmd = FakeGcmd(error=RuntimeError)
    sweep = tester._parse_sweep(gcmd)

    tester._run_test(
        gcmd,
        [resonance_tester.TestAxis("y")],
        None,
        sweep,
        capture_name_suffix="cap1",
    )

    assert scap.starts == [
        (
            "/tmp/raw_servo_y_cap1.scap",
            ["motor_a", "motor_a1", "motor_b", "motor_b1"],
        )
    ]
    assert scap.events == ["capture_start", "buzz", "capture_stop"]
    assert any(r.startswith("Servo encoder capture") for r in gcmd.responses)


def test_run_test_skips_servo_capture_without_servo_rails():
    tester, printer, buzz, toolhead = make_tester()
    tester.accel_chips = [("xy", FakeChip("adxl345"))]
    scap = printer.objects["servo_capture"]
    gcmd = FakeGcmd(error=RuntimeError)
    sweep = tester._parse_sweep(gcmd)

    tester._run_test(gcmd, [resonance_tester.TestAxis("x")], None, sweep)

    assert scap.starts == []
    assert scap.events == ["buzz"]


def test_run_test_moves_to_probe_point():
    tester, printer, buzz, toolhead = make_tester()
    tester.accel_chips = [("xy", FakeChip("adxl345"))]
    gcmd = FakeGcmd(error=RuntimeError)
    sweep = tester._parse_sweep(gcmd)

    tester._run_test(
        gcmd,
        [resonance_tester.TestAxis("x")],
        None,
        sweep,
        test_point=[10.0, 20.0, 5.0],
    )

    manual_moves = [c for c in toolhead.calls if c[0] == "manual_move"]
    assert manual_moves == [
        ("manual_move", (10.0, 20.0, 5.0), tester.move_speed)
    ]
