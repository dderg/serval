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
        self.written = []

    def finish_measurements(self):
        self.finished = True

    def has_valid_samples(self):
        return True

    def write_to_file(self, name):
        self.written.append(name)


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

    def run_sweep(self, gcmd, axis_name, *args):
        self.calls.append((axis_name, args))
        return args[2]


class FakeToolhead:
    def __init__(self):
        self.moves = []
        self.dwells = []
        self.waited = 0

    def manual_move(self, point, speed):
        self.moves.append((point, speed))

    def wait_moves(self):
        self.waited += 1

    def dwell(self, t):
        self.dwells.append(t)


class FakePrinter:
    config_error = RuntimeError

    def __init__(self):
        self._objs = {}
        self.event_handlers = {}

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
    buzz = FakeBuzz()
    toolhead = FakeToolhead()
    printer._objs["gcode"] = gcode
    printer._objs["resonance_buzz"] = buzz
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

    assert chip.clients[0].written
    assert chip.clients[0].written[0].endswith("probe1.csv")


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
