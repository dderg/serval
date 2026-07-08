"""Characterization: TMCErrorCheck against a real TMC5160 register map.

Pins down the exact wire behavior of the periodic driver health checks:
what is read, in what order, when GSTAT flags are cleared, how UART
flakiness is tolerated, and when the printer is shut down.
"""

import pytest
from tmc_wire_harness import (
    CommandError,
    FakeConfig,
    FakeMcuTmc,
    FakePrinter,
    ops,
    writes,
)

from klippy.extras import tmc, tmc5160

OT_ERR = 1 << 25
OTPW_WARN = 1 << 26
GSTAT_RESET = 1 << 0
GSTAT_DRV_ERR = 1 << 1


@pytest.fixture
def rig():
    wire = []
    printer = FakePrinter(wire)
    config = FakeConfig("tmc5160 stepper_x", {}, printer, sections={})
    fields = tmc.FieldHelper(
        tmc5160.Fields, tmc5160.SignedFields, tmc5160.FieldFormatters
    )
    mcu_tmc = FakeMcuTmc(fields, wire)
    echeck = tmc.TMCErrorCheck(config, mcu_tmc)

    class Rig:
        pass

    r = Rig()
    r.wire, r.printer, r.mcu_tmc, r.echeck = wire, printer, mcu_tmc, echeck
    return r


def test_start_checks_reads_status_then_gstat_then_arms_timer(rig):
    did_reset = rig.echeck.start_checks()
    assert ops(rig.wire) == [
        ("read", "DRV_STATUS"),
        ("read", "GSTAT"),
        ("timer+", "_do_periodic_check"),
    ]
    assert did_reset is False
    assert rig.printer.shutdowns == []


def test_start_checks_clears_latched_gstat_and_reports_reset(rig):
    rig.mcu_tmc.reads["GSTAT"] = [GSTAT_RESET, 0]
    did_reset = rig.echeck.start_checks()
    assert ops(rig.wire) == [
        ("read", "DRV_STATUS"),
        ("read", "GSTAT"),
        ("write", "GSTAT"),
        ("read", "GSTAT"),
        ("timer+", "_do_periodic_check"),
    ]
    assert writes(rig.wire) == [("write", "GSTAT", GSTAT_RESET, None)], (
        "clear is write-back of exactly the latched error bits"
    )
    assert did_reset is True, "reset flag must reach the enable path"


def test_start_checks_drv_err_without_reset_is_cleared_but_not_a_reset(rig):
    rig.mcu_tmc.reads["GSTAT"] = [GSTAT_DRV_ERR, 0]
    assert rig.echeck.start_checks() is False


def test_restarting_checks_replaces_the_timer(rig):
    rig.echeck.start_checks()
    rig.echeck.start_checks()
    timer_events = [op for op in ops(rig.wire) if op[0].startswith("timer")]
    assert timer_events == [
        ("timer+", "_do_periodic_check"),
        ("timer-", "_do_periodic_check"),
        ("timer+", "_do_periodic_check"),
    ]


def test_stop_checks_unregisters_once_and_is_idempotent(rig):
    rig.echeck.start_checks()
    rig.echeck.stop_checks()
    rig.echeck.stop_checks()
    timer_events = [op for op in ops(rig.wire) if op[0].startswith("timer")]
    assert timer_events == [
        ("timer+", "_do_periodic_check"),
        ("timer-", "_do_periodic_check"),
    ]


def test_periodic_check_clean_reschedules_one_second_out(rig):
    rig.echeck.start_checks()
    next_time = rig.echeck._do_periodic_check(200.0)
    assert next_time == 201.0
    assert rig.printer.shutdowns == []


def test_periodic_check_error_bit_retries_three_reads_then_shuts_down(rig):
    rig.echeck.start_checks()
    del rig.wire[:]
    rig.mcu_tmc.reads["DRV_STATUS"] = OT_ERR
    next_time = rig.echeck._do_periodic_check(200.0)
    assert ops(rig.wire) == [("read", "DRV_STATUS")] * 3, (
        "a fault must survive three consecutive reads before shutdown"
    )
    assert len(rig.printer.shutdowns) == 1
    assert "ot=" in rig.printer.shutdowns[0]
    assert next_time == rig.printer.get_reactor().NEVER


def test_periodic_check_warning_bit_does_not_shut_down(rig):
    rig.echeck.start_checks()
    rig.mcu_tmc.reads["DRV_STATUS"] = OTPW_WARN
    assert rig.echeck._do_periodic_check(200.0) == 201.0
    assert rig.printer.shutdowns == []


def test_unreachable_uart_driver_skips_the_check_instead_of_shutdown(rig):
    rig.echeck.start_checks()
    rig.mcu_tmc.reads["DRV_STATUS"] = CommandError(
        "Unable to read tmc uart 'stepper_x' register DRV_STATUS"
    )
    assert rig.echeck._do_periodic_check(200.0) == 201.0
    assert rig.printer.shutdowns == []


def test_transient_uart_error_is_retried_then_succeeds(rig):
    rig.echeck.start_checks()
    rig.mcu_tmc.reads["DRV_STATUS"] = [
        CommandError("Unable to read tmc uart 'stepper_x' register"),
        0,
    ]
    assert rig.echeck._do_periodic_check(200.0) == 201.0
    assert rig.printer.shutdowns == []


def test_non_uart_read_failure_shuts_down(rig):
    rig.echeck.start_checks()
    rig.mcu_tmc.reads["DRV_STATUS"] = CommandError("SPI transfer failed")
    assert rig.echeck._do_periodic_check(200.0) == (
        rig.printer.get_reactor().NEVER
    )
    assert rig.printer.shutdowns == ["SPI transfer failed"]


def test_get_status_is_empty_until_checks_run(rig):
    assert rig.echeck.get_status() == {
        "drv_status": None,
        "temperature": None,
    }


def test_get_status_reports_nonzero_drv_status_fields(rig):
    rig.mcu_tmc.reads["DRV_STATUS"] = OTPW_WARN
    rig.echeck.start_checks()
    status = rig.echeck.get_status()
    assert status["drv_status"] == {"otpw": 1}
    assert status["temperature"] is None, "5160 has no on-die ADC register"
