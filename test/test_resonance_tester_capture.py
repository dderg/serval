import os

import pytest

from klippy.extras import resonance_tester


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def get_float(self, name, default=None, **kw):
        return self._params.get(name, default)

    def respond_info(self, msg):
        self.responses.append(msg)


class FakeGcode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, func, desc=None):
        self.commands[name] = func


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


class FakeServoCapture:
    def __init__(self, events):
        self.events = events
        self.starts = []
        self.capture_dir = "/captures"

    def capture_path(self, name):
        return os.path.join(self.capture_dir, name + ".scap")

    def start_capture_to(self, path, servos):
        self.events.append("capture_start")
        self.starts.append((path, list(servos)))
        return path

    def stop_capture(self):
        self.events.append("capture_stop")
        return ("/tmp/fake.scap", 4321, 250.0)


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


class FakeServoKin:
    def __init__(self, rails):
        self.rails = rails

    def lanes(self):
        return [
            (lane_idx, rail.axis, rail.motors)
            for lane_idx, rail in enumerate(self.rails)
        ]

    def coupled_xy(self):
        return True


class FakeToolhead:
    def __init__(self):
        self.moves = []
        self.dwells = []
        self.waited = 0
        self._kin = object()

    def get_kinematics(self):
        return self._kin

    def manual_move(self, point, speed):
        self.moves.append((point, speed))

    def wait_moves(self):
        self.waited += 1

    def dwell(self, t):
        self.dwells.append(t)


class FakeReactor:
    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        pass


class FakePrinter:
    config_error = RuntimeError
    command_error = RuntimeError

    def __init__(self):
        self._objs = {}
        self.event_handlers = {}
        self._reactor = FakeReactor()

    def get_reactor(self):
        return self._reactor

    def lookup_object(self, name, default="__raise__"):
        if name in self._objs:
            return self._objs[name]
        if default == "__raise__":
            raise KeyError(name)
        return default

    def register_event_handler(self, event, cb):
        self.event_handlers[event] = cb

    def load_object(self, config, name, default="__raise__"):
        return self.lookup_object(name, default)


class FakeConfig:
    error = RuntimeError

    def __init__(self, printer, values):
        self._printer = printer
        self._values = values

    def get_printer(self):
        return self._printer

    def get(self, name, default=None):
        return self._values.get(name, default)

    def getfloat(self, name, default=None, **kw):
        return self._values.get(name, default)

    def getlists(self, name, default=None, **kw):
        return self._values.get(name, default)


def make_tester(values=None):
    printer = FakePrinter()
    gcode = FakeGcode()
    events = []
    buzz = FakeBuzz()
    buzz.events = events
    toolhead = FakeToolhead()
    printer._objs["gcode"] = gcode
    printer._objs["resonance_buzz"] = buzz
    printer._objs["servo_capture"] = FakeServoCapture(events)
    printer._objs["toolhead"] = toolhead
    cfg_values = {"accel_chip": "adxl345"}
    if values:
        cfg_values.update(values)
    tester = resonance_tester.ResonanceTester(FakeConfig(printer, cfg_values))
    return tester, printer, buzz, toolhead


def test_registers_commands():
    _, printer, _, _ = make_tester()
    gcode = printer._objs["gcode"]
    assert "TEST_RESONANCES" in gcode.commands
    assert "SHAPER_CALIBRATE" in gcode.commands
    assert "MEASURE_AXES_NOISE" in gcode.commands


def test_parse_axis_cardinal():
    gcmd = FakeGcmd()
    for name in ("x", "y", "z", "X", "Z"):
        axis = resonance_tester._parse_axis(gcmd, name)
        assert axis.buzz_axis() == name.lower()


def test_parse_axis_pure_vectors_map_to_named_axes():
    gcmd = FakeGcmd()
    assert resonance_tester._parse_axis(gcmd, "1,0").buzz_axis() == "x"
    assert resonance_tester._parse_axis(gcmd, "0,1").buzz_axis() == "y"


def test_parse_axis_diagonal_not_implemented():
    gcmd = FakeGcmd()
    with pytest.raises(RuntimeError, match="diagonal buzz is not implemented"):
        resonance_tester._parse_axis(gcmd, "1,1")


def test_parse_axis_bad_format():
    gcmd = FakeGcmd()
    with pytest.raises(RuntimeError, match="Invalid format"):
        resonance_tester._parse_axis(gcmd, "bogus")


def test_parse_axis_missing_is_required():
    gcmd = FakeGcmd()
    with pytest.raises(RuntimeError, match="AXIS parameter is required"):
        resonance_tester._parse_axis(gcmd, None)


def test_run_test_excites_axis_and_captures():
    tester, printer, buzz, toolhead = make_tester()
    chip = FakeChip("adxl345")
    tester.accel_chips = [("xy", chip)]
    gcmd = FakeGcmd()
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
    gcmd = FakeGcmd()
    sweep = tester._parse_sweep(gcmd)

    tester._run_test(gcmd, [resonance_tester.TestAxis("y")], None, sweep)

    assert buzz.calls and buzz.calls[0][0] == "y"
    assert x_chip.clients == []


def test_run_test_writes_raw_data_when_named():
    tester, printer, buzz, toolhead = make_tester()
    chip = FakeChip("adxl345")
    tester.accel_chips = [("xy", chip)]
    gcmd = FakeGcmd()
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
    gcmd = FakeGcmd()
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
    toolhead._kin = FakeServoKin(
        [
            _servo_rail("x", ["motor_a", "motor_a1"]),
            _servo_rail("y", ["motor_b", "motor_b1"]),
        ]
    )
    scap = printer._objs["servo_capture"]
    gcmd = FakeGcmd()
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
            "/captures/raw_servo_y_cap1.scap",
            ["motor_a", "motor_a1", "motor_b", "motor_b1"],
        )
    ]
    assert scap.events == ["capture_start", "buzz", "capture_stop"]
    assert any(r.startswith("Servo encoder capture") for r in gcmd.responses)


def test_run_test_skips_servo_capture_without_servo_rails():
    tester, printer, buzz, toolhead = make_tester()
    tester.accel_chips = [("xy", FakeChip("adxl345"))]
    scap = printer._objs["servo_capture"]
    gcmd = FakeGcmd()
    sweep = tester._parse_sweep(gcmd)

    tester._run_test(gcmd, [resonance_tester.TestAxis("x")], None, sweep)

    assert scap.starts == []
    assert scap.events == ["buzz"]


def test_run_test_moves_to_probe_point():
    tester, printer, buzz, toolhead = make_tester()
    tester.accel_chips = [("xy", FakeChip("adxl345"))]
    gcmd = FakeGcmd()
    sweep = tester._parse_sweep(gcmd)

    tester._run_test(
        gcmd,
        [resonance_tester.TestAxis("x")],
        None,
        sweep,
        test_point=[10.0, 20.0, 5.0],
    )

    assert toolhead.moves == [([10.0, 20.0, 5.0], tester.move_speed)]
