import pytest
from fakes import FakeMcu, FakeReactor
from fakes import FakePrinter as FakePrinterBase

from klippy.mcu import MIN_SCHEDULE_LEAD, MCU_digital_out
from klippy.motion import Motion

MOTION_LEAD = 0.25


class FakePrinter(FakePrinterBase):
    command_error = RuntimeError


class FakeCmd:
    def __init__(self):
        self.sent = []

    def send(self, args, minclock=0, reqclock=0):
        self.sent.append((args, minclock, reqclock))


def make_motion(mcus):
    motion = Motion.__new__(Motion)
    motion.reactor = FakeReactor(now=1000.0)
    motion.all_mcus = mcus
    motion.mcu = mcus[0]
    motion.motion_lead = MOTION_LEAD
    return motion


def make_pin(printer, mcu):
    pin = MCU_digital_out.__new__(MCU_digital_out)
    pin._printer = printer
    pin._mcu = mcu
    pin._pin = "gpiochip0/gpio2"
    pin._invert = 0
    pin._oid = 4
    pin._last_clock = 0
    pin._set_cmd = FakeCmd()
    return pin


def test_floor_leads_a_secondary_mcu_running_no_kinematic_lane():
    primary = FakeMcu(name="mcu", est_print_time=11.447)
    follower = FakeMcu(name="sc", est_print_time=12.587)
    motion = make_motion([primary, follower])
    assert motion._schedule_floor() == pytest.approx(12.587 + MOTION_LEAD)


def test_follower_lane_enable_pin_accepts_the_floor():
    primary = FakeMcu(name="mcu", est_print_time=11.447)
    follower = FakeMcu(name="sc", est_print_time=12.587)
    motion = make_motion([primary, follower])
    printer = FakePrinter(objects={"mcu": primary})
    pin = make_pin(printer, follower)
    pin.set_digital(motion._schedule_floor(), 1)
    assert len(pin._set_cmd.sent) == 1


def test_floor_over_the_primary_alone_is_stale_for_the_follower_mcu():
    primary = FakeMcu(name="mcu", est_print_time=11.447)
    follower = FakeMcu(name="sc", est_print_time=12.587)
    printer = FakePrinter(objects={"mcu": primary})
    pin = make_pin(printer, follower)
    with pytest.raises(RuntimeError, match="stale print_time"):
        pin.set_digital(
            primary.estimated_print_time(0.0) + MOTION_LEAD,
            1,
        )


def test_disconnected_non_critical_mcu_does_not_raise_the_floor():
    primary = FakeMcu(name="mcu", est_print_time=11.447)
    gone = FakeMcu(
        name="gone", est_print_time=99.0, non_critical_disconnected=True
    )
    motion = make_motion([primary, gone])
    assert motion._schedule_floor() == pytest.approx(11.447 + MOTION_LEAD)


def test_floor_keeps_the_minimum_schedule_lead_on_every_mcu():
    mcus = [
        FakeMcu(name="mcu", est_print_time=5.0),
        FakeMcu(name="a", est_print_time=5.4),
        FakeMcu(name="b", est_print_time=5.9),
    ]
    floor = make_motion(mcus)._schedule_floor()
    for mcu in mcus:
        lead = floor - mcu.estimated_print_time(0.0)
        assert lead >= MIN_SCHEDULE_LEAD
