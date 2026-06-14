import collections

from klippy.extras.gcode_move import GCodeMove

Coord = collections.namedtuple("Coord", ("x", "y", "z", "e"))


class FakeGCode:
    Coord = Coord

    def register_command(self, *args, **kwargs):
        pass


class FakeToolhead:
    def get_position(self):
        return [1.0, 2.0, 3.0, 4.0]


class FakeBridgeOk:
    def __init__(self, axes):
        self._axes = axes

    def query_motor_positions(self):
        return dict(self._axes)


class FakeBridgeRaises:
    def query_motor_positions(self):
        raise RuntimeError("mcu timeout")


class FakeGCmd:
    def __init__(self):
        self.responses = []

    def respond_info(self, msg):
        self.responses.append(msg)

    def error(self, msg):
        return RuntimeError(msg)


class FakePrinter:
    def __init__(self):
        self.objects = {"gcode": FakeGCode()}
        self.event_handlers = {}

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def register_event_handler(self, event, handler):
        self.event_handlers[event] = handler


class FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer


def _build(bridge=None, toolhead=True):
    printer = FakePrinter()
    if toolhead:
        printer.add_object("toolhead", FakeToolhead())
    if bridge is not None:
        printer.add_object("motion_bridge", bridge)
    return GCodeMove(FakeConfig(printer))


def test_get_position_reports_measured_cartesian():
    gm = _build(
        bridge=FakeBridgeOk(
            {
                "x": (10.0, 0.0),
                "y": (20.0, 0.0),
                "z": (5.0, 0.0),
                "e": (2.0, 0.0),
            }
        )
    )
    gcmd = FakeGCmd()
    gm.cmd_GET_POSITION(gcmd)
    assert len(gcmd.responses) == 1
    text = gcmd.responses[0]
    assert "X:10.000000" in text
    assert "Y:20.000000" in text
    assert "Z:5.000000" in text
    assert "E:2.000000" in text
    assert "ERR" not in text


def test_get_position_reports_err_on_query_failure_without_raising():
    gm = _build(bridge=FakeBridgeRaises())
    gcmd = FakeGCmd()
    gm.cmd_GET_POSITION(gcmd)
    assert len(gcmd.responses) == 1
    text = gcmd.responses[0]
    assert "ERR" in text
    assert "mcu timeout" in text


def test_get_position_reports_err_without_bridge():
    gm = _build(bridge=None)
    gcmd = FakeGCmd()
    gm.cmd_GET_POSITION(gcmd)
    assert "ERR" in gcmd.responses[0]


def test_get_position_raises_when_not_ready():
    gm = _build(bridge=None, toolhead=False)
    gcmd = FakeGCmd()
    try:
        gm.cmd_GET_POSITION(gcmd)
    except RuntimeError as e:
        assert "not ready" in str(e)
    else:
        raise AssertionError("expected gcmd.error to be raised")
