from fakes import FakeConfig, FakeEngine, FakeGcode, FakePrinter

from klippy.extras.motion_report import PrinterMotionReport


def _build(axes=None):
    printer = FakePrinter(objects={"gcode": FakeGcode()})
    if axes is not None:
        printer.add_object(
            "motion_engine", FakeEngine(live_motor_positions=axes)
        )
    report = PrinterMotionReport(FakeConfig(printer))
    printer.event_handlers["klippy:connect"]()
    return report


def test_get_status_serves_live_position_from_engine():
    report = _build(
        {
            "x": (10.0, 1.0),
            "y": (20.0, 0.0),
            "z": (5.0, 0.0),
            "e": (2.0, 3.0),
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
            "x": (0.0, 3.0),
            "y": (0.0, 4.0),
            "z": (0.0, 0.0),
            "e": (0.0, 0.0),
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


def test_get_status_partial_axes_dict_defaults_missing():
    report = _build({"x": (1.0, 0.5)})
    status = report.get_status(0.0)
    pos = status["live_position"]
    assert (pos.x, pos.y, pos.z, pos.e) == (1.0, 0.0, 0.0, 0.0)
    assert status["live_velocity"] == 0.5
    assert status["live_extruder_velocity"] == 0.0


def test_get_status_keeps_steppers_and_trapq_keys():
    report = _build({"x": (1.0, 0.0)})
    status = report.get_status(0.0)
    assert "steppers" in status
    assert "trapq" in status
