import pytest
from fakes import FakeMcu, FakeReactor
from fakes import FakePrinter as FakePrinterBase

from klippy.mcu import MIN_SCHEDULE_LEAD, MCU_digital_out


class FakePrinter(FakePrinterBase):
    command_error = RuntimeError

    def __init__(self):
        super().__init__(
            objects={"mcu": FakeMcu(name="fake", est_print_time=100.0)},
            reactor=FakeReactor(now=1000.0),
        )


class FakeCmd:
    def __init__(self):
        self.sent = []

    def send(self, args, minclock=0, reqclock=0):
        self.sent.append((args, minclock, reqclock))


def make_pin(est_print_time):
    pin = MCU_digital_out.__new__(MCU_digital_out)
    pin._printer = FakePrinter()
    pin._mcu = FakeMcu(name="fake", est_print_time=est_print_time)
    pin._pin = "PF15"
    pin._invert = 0
    pin._oid = 9
    pin._last_clock = 0
    pin._set_cmd = FakeCmd()
    return pin


def test_stale_print_time_is_rejected_before_reaching_the_mcu():
    pin = make_pin(est_print_time=100.0)
    with pytest.raises(RuntimeError, match="stale print_time"):
        pin.set_digital(100.0 + MIN_SCHEDULE_LEAD / 2.0, 1)
    assert pin._set_cmd.sent == []


def test_print_time_in_the_past_is_rejected():
    pin = make_pin(est_print_time=100.0)
    with pytest.raises(RuntimeError, match="stale print_time"):
        pin.set_digital(99.9, 1)


def test_print_time_with_normal_lead_is_sent():
    pin = make_pin(est_print_time=100.0)
    pin.set_digital(100.25, 1)
    assert len(pin._set_cmd.sent) == 1
    args, minclock, reqclock = pin._set_cmd.sent[0]
    assert reqclock == int(100.25 * 1e6)
