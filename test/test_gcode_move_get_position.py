from fakes import (
    FakeConfig,
    FakeEngine,
    FakeGcmd,
    FakeGcode,
    FakePrinter,
    FakeToolhead,
)

from klippy.extras.gcode_move import GCodeMove


def _build(engine=None, toolhead=True):
    objects = {"gcode": FakeGcode()}
    if toolhead:
        objects["toolhead"] = FakeToolhead(position=[1.0, 2.0, 3.0, 4.0])
    if engine is not None:
        objects["motion_engine"] = engine
    printer = FakePrinter(objects)
    return GCodeMove(FakeConfig(printer=printer))


def test_get_position_reports_measured_cartesian():
    gm = _build(
        engine=FakeEngine(
            query_motor_positions={
                "x": (10.0, 0.0),
                "y": (20.0, 0.0),
                "z": (5.0, 0.0),
                "e": (2.0, 0.0),
            }
        )
    )
    gcmd = FakeGcmd(error=RuntimeError)
    gm.cmd_GET_POSITION(gcmd)
    assert len(gcmd.responses) == 1
    text = gcmd.responses[0]
    assert "X:10.000000" in text
    assert "Y:20.000000" in text
    assert "Z:5.000000" in text
    assert "E:2.000000" in text
    assert "ERR" not in text


def test_get_position_reports_err_on_query_failure_without_raising():
    gm = _build(engine=FakeEngine(raises=RuntimeError("mcu timeout")))
    gcmd = FakeGcmd(error=RuntimeError)
    gm.cmd_GET_POSITION(gcmd)
    assert len(gcmd.responses) == 1
    text = gcmd.responses[0]
    assert "ERR" in text
    assert "mcu timeout" in text


def test_get_position_reports_err_without_engine():
    gm = _build(engine=None)
    gcmd = FakeGcmd(error=RuntimeError)
    gm.cmd_GET_POSITION(gcmd)
    assert "ERR" in gcmd.responses[0]


def test_get_position_raises_when_not_ready():
    gm = _build(engine=None, toolhead=False)
    gcmd = FakeGcmd(error=RuntimeError)
    try:
        gm.cmd_GET_POSITION(gcmd)
    except RuntimeError as e:
        assert "not ready" in str(e)
    else:
        raise AssertionError("expected gcmd.error to be raised")
