import pytest
from fakes import FakeEngine, FakeMcu, FakePrinter

from klippy.motion_endstop import (
    PROVIDER_ID_FIRST,
    MotionEndstop,
    MotorBinding,
    RemoteMotionEndstop,
    allocate_provider_id,
    register_stepcompress_steppers,
)


class FakeCommand:
    def __init__(self, response=None):
        self.sent = []
        self.response = response

    def send(self, args):
        self.sent.append(list(args))
        return self.response


class FakeRemoteMcu:
    def get_engine_handle(self):
        return 42


def _fake_mcu(engine=None, handle=7):
    if engine is None:
        engine = FakeEngine()
    printer = FakePrinter(objects={"motion_engine": engine})
    return FakeMcu(
        printer=printer,
        handle=handle,
        query_cmd=FakeCommand(),
        state_cmd=FakeCommand(
            {
                "oid": 0,
                "armed": 0,
                "pin_value": 0,
                "tripped": 0,
                "trip_clock": 0,
            }
        ),
    )


def _pin_params(mcu, pin="PA8", invert=0, pullup=1):
    return {
        "chip": mcu,
        "chip_name": "mcu",
        "pin": pin,
        "invert": invert,
        "pullup": pullup,
    }


def _connected(mcu, endstop):
    for cb in mcu.config_callbacks:
        cb()
    return endstop


def test_config_cmd_emitted():
    mcu = _fake_mcu()
    _connected(mcu, MotionEndstop(_pin_params(mcu), 3))
    assert mcu.config_cmds == [
        "config_endstop oid=0 endstop_id=3 pin=PA8 pull_up=1 invert=0"
        " motor=255 stepper=255 group=0",
        "config_trsync oid=1",
    ]


def test_local_binding_reaches_firmware_with_group_flag():
    mcu = _fake_mcu()
    binding = MotorBinding(0, 1, mcu, "stepper_x1", 21)
    _connected(
        mcu, MotionEndstop(_pin_params(mcu), 4, binding=binding, group=True)
    )
    assert mcu.config_cmds == [
        "config_endstop oid=0 endstop_id=4 pin=PA8 pull_up=1 invert=0"
        " motor=0 stepper=1 group=1",
        "config_trsync oid=1",
    ]


def test_foreign_binding_is_unbound_in_firmware_but_freezes_remotely():
    mcu = _fake_mcu(handle=7)
    motor_mcu = _fake_mcu(handle=9)
    binding = MotorBinding(0, 1, motor_mcu, "stepper_x1", 21)
    es = _connected(
        mcu, MotionEndstop(_pin_params(mcu), 4, binding=binding, group=True)
    )
    assert mcu.config_cmds == [
        "config_endstop oid=0 endstop_id=4 pin=PA8 pull_up=1 invert=0"
        " motor=255 stepper=255 group=1",
        "config_trsync oid=1",
    ]
    assert es.remote_freeze() == (9, 0, 1, 21)


def test_is_triggered_applies_invert():
    mcu = _fake_mcu()
    endstop = _connected(mcu, MotionEndstop(_pin_params(mcu, invert=1), 3))
    mcu.state_cmd.response = {"oid": 0, "armed": 0, "pin_value": 0}
    assert endstop.is_triggered() is True
    mcu.state_cmd.response = {"oid": 0, "armed": 0, "pin_value": 1}
    assert endstop.is_triggered() is False


def test_arm_sends_rest_ticks():
    mcu = _fake_mcu()
    endstop = _connected(mcu, MotionEndstop(_pin_params(mcu), 3))
    endstop.arm(0.001)
    assert mcu.query_cmd.sent == [[0, 1000]]


def test_an_unbound_endstop_arms_every_stepcompress_lane_on_its_mcu():
    mcu = _fake_mcu()
    register_stepcompress_steppers(mcu.get_printer(), mcu, [21, 22])
    endstop = _connected(mcu, MotionEndstop(_pin_params(mcu), 3))
    endstop.arm(0.001)
    assert [args for args in mcu.query_cmd.sent if len(args) == 2] == [
        [21, 1],
        [22, 1],
        [0, 1000],
    ]


def test_a_keyed_endstop_arms_only_its_own_motor():
    mcu = _fake_mcu()
    register_stepcompress_steppers(mcu.get_printer(), mcu, [21, 22])
    binding = MotorBinding(0, 1, mcu, "stepper_x1", 22)
    endstop = _connected(
        mcu, MotionEndstop(_pin_params(mcu), 4, binding=binding, group=True)
    )
    endstop.arm(0.001)
    assert [22, 1] in mcu.query_cmd.sent
    assert [21, 1] not in mcu.query_cmd.sent


def test_an_mcu_with_no_stepcompress_lanes_arms_no_mcu_side_stop():
    mcu = _fake_mcu()
    endstop = _connected(mcu, MotionEndstop(_pin_params(mcu), 3))
    endstop.arm(0.001)
    assert mcu.query_cmd.sent == [[0, 1000]]


def test_query_endstop_matches_is_triggered():
    mcu = _fake_mcu()
    endstop = _connected(mcu, MotionEndstop(_pin_params(mcu), 3))
    mcu.state_cmd.response = {"oid": 0, "armed": 0, "pin_value": 1}
    assert endstop.query_endstop(0.0) is endstop.is_triggered() is True


def test_arm_zero_period_rejected():
    mcu = _fake_mcu()
    endstop = _connected(mcu, MotionEndstop(_pin_params(mcu), 3))
    with pytest.raises(ValueError, match="rest_ticks"):
        endstop.arm(0.0)
    assert mcu.query_cmd.sent == []
    assert mcu.get_printer().lookup_object("motion_engine").calls == []


class _OrderedEngine(FakeEngine):
    def __init__(self, log):
        super().__init__()
        self._log = log

    def note_endstop_arm(self, endstop_mcu, endstop_id):
        self._log.append(("note_arm", endstop_mcu, endstop_id))


class _LoggingCommand(FakeCommand):
    def __init__(self, log):
        super().__init__()
        self._log = log

    def send(self, args):
        self._log.append(("query_endstop", list(args)))
        return super().send(args)


def test_arm_notes_engine_before_sending_query_endstop():
    log = []
    mcu = _fake_mcu(engine=_OrderedEngine(log), handle=5)
    mcu.query_cmd = _LoggingCommand(log)
    endstop = _connected(mcu, MotionEndstop(_pin_params(mcu), 3))
    endstop.arm(0.001)
    assert log == [("note_arm", 5, 3), ("query_endstop", [0, 1000])]


def test_disarm_sends_zero_rest_ticks_to_clear_suppression():
    mcu = _fake_mcu()
    endstop = _connected(mcu, MotionEndstop(_pin_params(mcu), 3))
    endstop.arm(0.001)
    endstop.disarm()
    assert mcu.query_cmd.sent == [[0, 1000], [0, 0]]


def test_query_trip_state_not_tripped():
    mcu = _fake_mcu()
    es = MotionEndstop(_pin_params(mcu), 7)
    for cb in mcu.config_callbacks:
        cb()
    assert es.query_trip_state() == {"tripped": False, "trip_clock": 0}


def test_query_trip_state_tripped_returns_latched_clock():
    mcu = _fake_mcu()
    es = MotionEndstop(_pin_params(mcu), 7)
    for cb in mcu.config_callbacks:
        cb()
    mcu.state_cmd.response = {
        "oid": 0,
        "armed": 0,
        "pin_value": 1,
        "tripped": 1,
        "trip_clock": 0xDEADBEEF,
    }
    assert es.query_trip_state() == {
        "tripped": True,
        "trip_clock": 0xDEADBEEF,
    }


def test_provider_ids_allocate_sequentially():
    printer = FakePrinter()
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST + 1
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST + 2


def _remote_setup():
    printer = FakePrinter()
    engine = FakeEngine()
    printer.add_object("motion_engine", engine)
    es = RemoteMotionEndstop(printer, FakeRemoteMcu(), trsync_oid=9)
    return engine, es


def test_remote_endstop_allocates_provider_id():
    _, es = _remote_setup()
    assert es.endstop_id >= PROVIDER_ID_FIRST


def test_remote_endstop_arm_and_disarm_delegate_to_engine():
    engine, es = _remote_setup()
    es.arm(0.001)
    es.disarm()
    assert engine.calls == [
        ("arm_remote_trigger", 42, 9, es.endstop_id),
        ("disarm_remote_trigger", es.endstop_id),
    ]


def test_remote_endstop_default_query_state():
    _, es = _remote_setup()
    assert es.is_triggered() is False
    assert es.query_endstop(0.0) is False
    assert es.engine_mcu_handle() == 42
