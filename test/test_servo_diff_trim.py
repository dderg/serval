import pytest
from fakes import (
    FakeConfig,
    FakeGcode,
    FakeKin,
    FakeNode,
    FakePrinter,
    FakeToolhead,
)
from fakes import FakeEngine as _FakeEngine
from fakes import FakeGcmd as _FakeGcmd

from klippy.extras import servo_axis, servo_diff_trim


class FakeGcmd(_FakeGcmd):
    error = RuntimeError


class FakeEngine(_FakeEngine):
    def __init__(self):
        super().__init__()
        self.trims = []

    def set_diff_trim(self, *args):
        self.calls.append(("set_diff_trim",) + args)
        self.trims.append(args)


def _motor(name, node_name, chain_index, invert=False):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = name
    m.node_name = node_name
    m.chain_index = chain_index
    m.invert_direction = invert
    m.rotation_distance = 40.0
    m.encoder_counts_per_rev = 131072
    return m


def _rail(axis, motors):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis " + axis
    rail.axis = axis
    rail.motors = motors
    return rail


def awd_rails(node="xy_drives", node_b=None):
    return [
        _rail(
            "x",
            [
                _motor("motor_a1", node, 1),
                _motor("motor_a", node, 0),
            ],
        ),
        _rail(
            "y",
            [
                _motor("motor_b", node_b or node, 2, invert=True),
                _motor("motor_b1", node_b or node, 3),
            ],
        ),
    ]


def single_drive_rails():
    return [
        _rail("x", [_motor("motor_a", "xy_drives", 0)]),
        _rail("y", [_motor("motor_b", "xy_drives", 1)]),
    ]


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
        "toolhead": FakeToolhead(
            FakeKin(rails or awd_rails(), coupled_xy=True)
        ),
        "motion_engine": engine,
        "configfile": FakeConfigfile(),
    }
    for node_name, slots in node_slots.items():
        objs["ethercat_node " + node_name] = FakeNode(
            name=node_name, slots=slots, handle=7
        )
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


def test_zero_gain_freezes_with_offset_kept():
    trim, _printer, engine = make_diff_trim()
    trim.cmd_SERVO_DIFF_TRIM(FakeGcmd(BELT="A", GAIN="0"))
    assert engine.trims == [(7, 0, 1, 0, 150, 2000, 300)]


def test_remove_sends_zero_max_offset():
    trim, _printer, engine = make_diff_trim()
    trim.cmd_SERVO_DIFF_TRIM(FakeGcmd(BELT="A", REMOVE="1"))
    assert engine.trims == [(7, 0, 1, 0, 0, 2000, 300)]


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
