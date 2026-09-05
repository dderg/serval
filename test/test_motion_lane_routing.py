"""Per-lane transport routing: step/dir lanes go to stepcompress, phase lanes
go to the sample-run executor, and one MCU may carry both."""

from fakes import FakeMcu, FakePrinter, FakeStepper

from klippy import motion_setup
from klippy.motion_endstop import stepcompress_stepper_oids

CONFIGURE_AXIS_ARGSTRING = (
    "kalico_configure_axis axis_idx=%c mode=%c"
    " microstep_distance=%u extrusion_per_xy_mm=%u"
    " stepper_count=%c steppers=%*s"
)
SLOT_NAMES = ("a", "b", "z")
PHASE_BUS_RATE_HZ = 2_000_000


class RecordingCommand:
    def __init__(self):
        self.sent = []

    def send(self, args):
        self.sent.append(list(args))


class RecordingMcu(FakeMcu):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.looked_up = []
        self.commands = {}

    def lookup_command(self, msgformat):
        self.looked_up.append(msgformat)
        return self.commands.setdefault(msgformat, RecordingCommand())


class FakeTmc:
    def __init__(
        self, bus_id=0, cs_pin_id=5, spi_oid=31, sensorless_homing=False
    ):
        self._config = (bus_id, cs_pin_id)
        self._spi_oid = spi_oid
        self._sensorless_homing = sensorless_homing
        self.phase_stepper_oid = None
        self.phase_group = None

    def needs_pulse_mode_windows(self):
        return self._sensorless_homing

    def get_phase_config(self):
        return self._config

    def set_phase_stepper_oid(self, oid):
        self.phase_stepper_oid = oid

    def set_phase_group(self, group):
        self.phase_group = group

    def get_spi_oid(self):
        return self._spi_oid


class FakeEngine:
    def __init__(self, caps=motion_setup.PHASE_STEPPING_CAPABILITY_BIT):
        self._caps = caps
        self.phase_buses = []
        self.phase_motors = []

    def get_mcu_capabilities(self, handle):
        return self._caps

    def register_phase_bus(self, handle, bus_id, rate):
        self.phase_buses.append((handle, bus_id, rate))

    def register_phase_motor(self, handle, motor_idx, bus_id, cs, slot):
        self.phase_motors.append((handle, motor_idx, bus_id, cs, slot))


class FakeKin:
    kind = "cartesian"


class FakeMotion:
    def __init__(self, printer, engine):
        self.printer = printer
        self.engine = engine
        self.kin = FakeKin()
        self._motor_bindings = {}


def build(phase_slots=(), tmcs=None):
    printer = FakePrinter()
    engine = FakeEngine()
    mcu = RecordingMcu(printer=printer, handle=7)
    slot_steppers = [[], [], [], []]
    for slot, name in enumerate(SLOT_NAMES):
        stepper = FakeStepper(name=name, mcu=mcu, oid=10 + slot)
        stepper.phase_stepping = slot in phase_slots
        slot_steppers[slot].append((name, stepper))
    for name, tmc in (tmcs or {}).items():
        printer.add_object("tmc5160 %s" % (name,), tmc)
    motion = FakeMotion(printer, engine)
    motion_setup._configure_one_mcu(
        motion, "mcu", mcu, 7, slot_steppers, False, 0, 1
    )
    return motion, printer, engine, mcu


def configure_axis_sends(mcu):
    cmd = mcu.commands.get(CONFIGURE_AXIS_ARGSTRING)
    return [] if cmd is None else cmd.sent


def test_a_pulse_only_mcu_takes_the_stepcompress_path():
    motion, printer, engine, mcu = build()
    assert stepcompress_stepper_oids(printer, mcu) == [10, 11, 12]
    assert configure_axis_sends(mcu) == []
    assert mcu.looked_up == []
    assert engine.phase_motors == []
    assert engine.phase_buses == []
    assert set(motion._motor_bindings) == set(SLOT_NAMES)


def test_a_mixed_mcu_splits_lanes_between_both_transports():
    tmc = FakeTmc()
    motion, printer, engine, mcu = build(phase_slots=(0,), tmcs={"a": tmc})
    assert stepcompress_stepper_oids(printer, mcu) == [11, 12]
    sends = configure_axis_sends(mcu)
    assert [args[0] for args in sends] == [0]
    assert [args[1] for args in sends] == [motion_setup.MODE_PHASE]
    assert engine.phase_motors == [(7, 0, 0, 5, 0)]
    assert engine.phase_buses == [(7, 0, PHASE_BUS_RATE_HZ)]
    assert tmc.phase_stepper_oid == 10
    assert set(motion._motor_bindings) == set(SLOT_NAMES)


def test_a_stallguard_homed_phase_lane_is_bound_to_both_transports():
    tmc = FakeTmc(sensorless_homing=True)
    motion, printer, engine, mcu = build(phase_slots=(0,), tmcs={"a": tmc})
    assert stepcompress_stepper_oids(printer, mcu) == [10, 11, 12], (
        "the phase motor's classic step queue is what a StallGuard trip move "
        "runs on, so its oid must reach the stepcompress registry too"
    )
    sends = configure_axis_sends(mcu)
    assert [args[0] for args in sends] == [0]
    assert [args[1] for args in sends] == [motion_setup.MODE_PHASE]
    assert engine.phase_motors == [(7, 0, 0, 5, 0)]
    assert engine.phase_buses == [(7, 0, PHASE_BUS_RATE_HZ)]
    assert set(motion._motor_bindings) == set(SLOT_NAMES)


def test_the_configure_axis_command_carries_no_ring_depth():
    tmc = FakeTmc()
    _motion, _printer, _engine, mcu = build(phase_slots=(0,), tmcs={"a": tmc})
    assert CONFIGURE_AXIS_ARGSTRING in mcu.looked_up
    assert not any("ring_depth" in fmt for fmt in mcu.looked_up)
    assert all(len(args) == 6 for args in configure_axis_sends(mcu))


def test_phase_lanes_on_one_bus_register_that_bus_once():
    tmcs = {
        "a": FakeTmc(bus_id=1, cs_pin_id=5),
        "b": FakeTmc(bus_id=1, cs_pin_id=6, spi_oid=32),
    }
    _motion, _printer, engine, _mcu = build(phase_slots=(0, 1), tmcs=tmcs)
    assert engine.phase_buses == [(7, 1, PHASE_BUS_RATE_HZ)]
    assert engine.phase_motors == [(7, 0, 1, 5, 0), (7, 1, 1, 6, 1)]


def test_phase_lanes_on_separate_buses_each_register_at_the_sample_rate():
    tmcs = {
        "a": FakeTmc(bus_id=0, cs_pin_id=5),
        "b": FakeTmc(bus_id=1, cs_pin_id=6, spi_oid=32),
    }
    _motion, _printer, engine, _mcu = build(phase_slots=(0, 1), tmcs=tmcs)
    assert engine.phase_buses == [
        (7, 0, PHASE_BUS_RATE_HZ),
        (7, 1, PHASE_BUS_RATE_HZ),
    ]
    assert engine.phase_motors == [(7, 0, 0, 5, 0), (7, 1, 1, 6, 1)]
