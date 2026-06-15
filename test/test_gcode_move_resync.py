import collections

from klippy.extras.gcode_move import GCodeMove

Coord = collections.namedtuple("Coord", ("x", "y", "z", "e"))


class FakeGCode:
    Coord = Coord

    def register_command(self, *args, **kwargs):
        pass


class FakeGCmd:
    def __init__(self, params):
        self._params = params

    def get_command_parameters(self):
        return self._params

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


class ResyncToolhead:
    def __init__(self, printer, actual, dirty=True):
        self._printer = printer
        self._pos = list(actual)
        self._dirty = dirty
        self.calls = []
        self.moves = []
        self.curves = []

    def get_position(self):
        return list(self._pos)

    def resync_parked_servos(self):
        self.calls.append("resync")
        if not self._dirty:
            return
        self._dirty = False
        handler = self._printer.event_handlers.get("toolhead:set_position")
        if handler is not None:
            handler()

    def move(self, newpos, speed):
        self.calls.append("move")
        self.moves.append(list(newpos))
        self._pos = list(newpos)

    def move_curve(self, newpos, interior, submit, speed):
        self.calls.append("move_curve")
        self.curves.append((list(newpos), [list(c) for c in interior]))
        self._pos = list(newpos)


def _build(actual, dirty=True, stale=None):
    printer = FakePrinter()
    th = ResyncToolhead(printer, actual, dirty=dirty)
    printer.add_object("toolhead", th)
    gm = GCodeMove(FakeConfig(printer))
    gm._handle_ready()
    if stale is not None:
        gm.last_position = list(stale)
    return gm, th


def test_g1_absolute_resyncs_origin_before_resolving():
    gm, th = _build(
        actual=[37.27, 0.0, 10.0, 0.0], stale=[999.0, 888.0, 777.0, 0.0]
    )
    gm.cmd_G1(FakeGCmd({"X": "150", "F": "15000"}))
    assert th.calls[0] == "resync"
    assert th.moves[0] == [150.0, 0.0, 10.0, 0.0]
    assert gm.last_position == [150.0, 0.0, 10.0, 0.0]


def test_g1_carried_axis_uses_resynced_origin_not_stale():
    gm, th = _build(
        actual=[37.27, 0.0, 10.0, 0.0], stale=[999.0, 0.0, 10.0, 0.0]
    )
    gm.cmd_G1(FakeGCmd({"Y": "5"}))
    assert th.moves[0] == [37.27, 5.0, 10.0, 0.0]
    assert gm.last_position == [37.27, 5.0, 10.0, 0.0]


def test_g1_normal_move_without_dirty_servos_not_clobbered():
    gm, th = _build(actual=[10.0, 0.0, 10.0, 0.0], dirty=False)
    gm.last_position = [10.0, 0.0, 10.0, 0.0]
    gm.cmd_G1(FakeGCmd({"X": "150"}))
    assert "resync" in th.calls
    assert th.moves[0] == [150.0, 0.0, 10.0, 0.0]
    assert gm.last_position == [150.0, 0.0, 10.0, 0.0]


def test_g5_resyncs_before_resolving():
    gm, th = _build(
        actual=[37.27, 0.0, 10.0, 0.0], stale=[999.0, 0.0, 10.0, 0.0]
    )
    gm.cmd_G5(FakeGCmd({"X": "50", "Y": "0", "P": "0", "Q": "0"}))
    assert th.calls[0] == "resync"
    newpos, _interior = th.curves[0]
    assert newpos == [50.0, 0.0, 10.0, 0.0]


def test_g5_1_resyncs_before_resolving():
    gm, th = _build(
        actual=[37.27, 0.0, 10.0, 0.0], stale=[999.0, 0.0, 10.0, 0.0]
    )
    gm.cmd_G5_1(FakeGCmd({"X": "50", "I": "5", "J": "0"}))
    assert th.calls[0] == "resync"
