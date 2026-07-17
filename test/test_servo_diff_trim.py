import pytest
from test_servo_calibration_awd import (
    FakeConfig,
    FakeEngine,
    FakeGcmd,
    FakeGcode,
    FakeKin,
    FakeNode,
    FakePrinter,
    FakeToolhead,
    awd_rails,
    single_drive_rails,
)

from klippy.extras import servo_diff_trim


class FakeConfigWithValues(FakeConfig):
    def __init__(self, printer, values):
        super().__init__(printer)
        self._values = values

    def get(self, name, default=None):
        return self._values.get(name, default)

    def getfloat(self, name, default=None, **kw):
        return float(self._values.get(name, default))

    def getint(self, name, default=None, **kw):
        return int(self._values.get(name, default))


class EventFakePrinter(FakePrinter):
    def __init__(self, objs, reactor=None):
        super().__init__(objs, reactor)
        self.event_handlers = {}

    def register_event_handler(self, event, handler):
        self.event_handlers.setdefault(event, []).append(handler)


class FakeConfigfile:
    def __init__(self):
        self.values = {}

    def set(self, section, option, value):
        self.values[(section, option)] = value


def make_diff_trim(config_values=None, rails=None):
    engine = FakeEngine()
    node_slots = {}
    for rail in rails or awd_rails():
        for m in rail.motors:
            node_slots.setdefault(m.node_name, {})[m.motor_name] = m.chain_index
    objs = {
        "gcode": FakeGcode(),
        "toolhead": FakeToolhead(FakeKin(rails or awd_rails())),
        "motion_engine": engine,
        "configfile": FakeConfigfile(),
    }
    for node_name, slots in node_slots.items():
        objs["ethercat_node " + node_name] = FakeNode(slots)
    printer = EventFakePrinter(objs)
    trim = servo_diff_trim.ServoDiffTrim(
        FakeConfigWithValues(printer, config_values or {})
    )
    return trim, printer, engine


def test_arms_both_belts_by_default():
    trim, _printer, engine = make_diff_trim()
    trim.cmd_SERVO_DIFF_TRIM(FakeGcmd(GAIN="0.05"))
    assert engine.trims == [
        (7, 0, 1, 50000, 150, 2000, 300),
        (7, 2, 3, 50000, 150, 2000, 300),
    ]


def test_single_belt_with_explicit_knobs():
    trim, _printer, engine = make_diff_trim()
    trim.cmd_SERVO_DIFF_TRIM(
        FakeGcmd(
            BELT="B",
            GAIN="0.2",
            MAX_OFFSET_UM="300",
            LPF_HZ="10",
            SETTLE_MS="500",
        )
    )
    assert engine.trims == [(7, 2, 3, 200000, 300, 10000, 500)]


def test_zero_gain_disarms():
    trim, _printer, engine = make_diff_trim()
    trim.cmd_SERVO_DIFF_TRIM(FakeGcmd(BELT="A", GAIN="0"))
    assert engine.trims == [(7, 0, 1, 0, 150, 2000, 300)]


def test_rejects_single_drive_belts():
    trim, _printer, engine = make_diff_trim(rails=single_drive_rails())
    with pytest.raises(RuntimeError, match="two drives per belt"):
        trim.cmd_SERVO_DIFF_TRIM(FakeGcmd(GAIN="0.05"))
    assert not engine.trims


def test_config_values_arm_at_ready():
    trim, printer, engine = make_diff_trim(
        {
            "gain": "0.1",
            "max_offset_um": "200",
            "lpf_hz": "1.5",
            "settle_ms": "400",
        }
    )
    for handler in printer.event_handlers["klippy:ready"]:
        handler()
    assert engine.trims == [
        (7, 0, 1, 100000, 200, 1500, 400),
        (7, 2, 3, 100000, 200, 1500, 400),
    ]


def test_zero_config_gain_stays_disarmed_at_ready():
    trim, printer, engine = make_diff_trim()
    for handler in printer.event_handlers["klippy:ready"]:
        handler()
    assert not engine.trims


def test_command_defaults_come_from_config():
    trim, _printer, engine = make_diff_trim(
        {
            "gain": "0.1",
            "max_offset_um": "200",
            "lpf_hz": "1.5",
            "settle_ms": "400",
        }
    )
    trim.cmd_SERVO_DIFF_TRIM(FakeGcmd(BELT="A"))
    assert engine.trims == [(7, 0, 1, 100000, 200, 1500, 400)]


def test_save_stages_current_values_for_save_config():
    trim, printer, _engine = make_diff_trim()
    trim.cmd_SERVO_DIFF_TRIM(FakeGcmd(GAIN="0.05", SETTLE_MS="1000", SAVE="1"))
    configfile = printer.lookup_object("configfile")
    assert configfile.values == {
        ("servo_diff_trim", "gain"): "0.050000",
        ("servo_diff_trim", "max_offset_um"): "150.0",
        ("servo_diff_trim", "lpf_hz"): "2.000",
        ("servo_diff_trim", "settle_ms"): "1000",
    }
