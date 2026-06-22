import collections

from klippy.extras.motion_report import PrinterMotionReport

Coord = collections.namedtuple("Coord", ("x", "y", "z", "e"))


class FakeGCode:
    Coord = Coord


class FakeMcu:
    def estimated_print_time(self, eventtime):
        return eventtime


class FakeEngine:
    # axes: {name: (pos, vel, accel)} from the committed trajectory, or an
    # Exception instance to raise (clock unsynced / no history).
    def __init__(self, axes):
        self._axes = axes

    def motion_state_at(self, mcu, clock=None, print_time=None):
        if isinstance(self._axes, Exception):
            raise self._axes
        return dict(self._axes)


class FakeMotion:
    mcu = FakeMcu()


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


def _build(axes=None, with_motion=True):
    printer = FakePrinter()
    if axes is not None:
        printer.add_object("motion_engine", FakeEngine(axes))
    if with_motion:
        printer.add_object("motion", FakeMotion())
    report = PrinterMotionReport(FakeConfig(printer))
    printer.event_handlers["klippy:connect"]()
    return report


def test_get_status_serves_commanded_trajectory():
    report = _build(
        {
            "x": (10.0, 1.0, 0.0),
            "y": (20.0, 0.0, 0.0),
            "z": (5.0, 0.0, 0.0),
            "e": (2.0, 3.0, 0.0),
        }
    )
    status = report.get_status(0.0)
    pos = status["live_position"]
    assert (pos.x, pos.y, pos.z, pos.e) == (10.0, 20.0, 5.0, 2.0)
    assert status["live_velocity"] == 1.0
    assert status["live_extruder_velocity"] == 3.0


def test_get_status_velocity_is_cartesian_magnitude():
    report = _build(
        {
            "x": (0.0, 3.0, 0.0),
            "y": (0.0, 4.0, 0.0),
            "z": (0.0, 0.0, 0.0),
            "e": (0.0, 0.0, 0.0),
        }
    )
    assert report.get_status(0.0)["live_velocity"] == 5.0


def test_get_status_without_engine_returns_safe_defaults():
    report = _build(axes=None)
    status = report.get_status(0.0)
    pos = status["live_position"]
    assert (pos.x, pos.y, pos.z, pos.e) == (0.0, 0.0, 0.0, 0.0)
    assert status["live_velocity"] == 0.0
    assert status["live_extruder_velocity"] == 0.0


def test_get_status_without_motion_returns_safe_defaults():
    report = _build({"x": (1.0, 0.0, 0.0)}, with_motion=False)
    status = report.get_status(0.0)
    pos = status["live_position"]
    assert (pos.x, pos.y, pos.z, pos.e) == (0.0, 0.0, 0.0, 0.0)


def test_get_status_engine_error_serves_last_good():
    report = _build(RuntimeError("clock unsynced"))
    status = report.get_status(0.0)
    pos = status["live_position"]
    assert (pos.x, pos.y, pos.z, pos.e) == (0.0, 0.0, 0.0, 0.0)


def test_get_status_partial_axes_default_to_previous_position():
    # First sample establishes a full position.
    report = _build({"x": (1.0, 0.5, 0.0)})
    status = report.get_status(0.0)
    pos = status["live_position"]
    # Missing axes hold at their previous value (0.0 here), not jump elsewhere.
    assert (pos.x, pos.y, pos.z, pos.e) == (1.0, 0.0, 0.0, 0.0)
    assert status["live_velocity"] == 0.5
    assert status["live_extruder_velocity"] == 0.0


def test_get_status_keeps_steppers_and_trapq_keys():
    report = _build({"x": (1.0, 0.0, 0.0)})
    status = report.get_status(0.0)
    assert "steppers" in status
    assert "trapq" in status
