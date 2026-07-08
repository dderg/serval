"""Characterization: TMCCommandHelper init/enable/disable wire sequences.

Runs the real TMCCommandHelper + TMCErrorCheck + FieldHelper over a real
TMC5160 register map; only the register wire, the stepper-enable line and
the reactor are fake. Pins the exact register writes (values, order,
print_time stamping) of the three driver lifecycle paths: connect-time
init, stepper-enable, stepper-disable.
"""

import pytest
from tmc_wire_harness import (
    CommandError,
    FakeConfig,
    FakeCurrentHelper,
    FakeEnableLine,
    FakeForceMove,
    FakeGCode,
    FakeMcuTmc,
    FakePrinter,
    FakeStepper,
    FakeStepperEnable,
    ops,
    writes,
)

from klippy.extras import tmc, tmc2130, tmc5160

# microsteps=16 (mres=4), interpolate, dedge, toff=3
CHOPCONF_RUN = 0x34000003
CHOPCONF_TOFF_OFF = 0x34000000
GCONF_MULTISTEP_FILT = 1 << 3
GSTAT_RESET = 1 << 0


class Rig:
    def __init__(self, dedicated_enable=True, step_both_edge=True):
        self.wire = []
        self.printer = FakePrinter(self.wire)
        self.gcode = FakeGCode()
        self.enable_line = FakeEnableLine(dedicated=dedicated_enable)
        self.stepper = FakeStepper(step_both_edge=step_both_edge)
        self.printer.add_object("gcode", self.gcode)
        self.printer.add_object(
            "stepper_enable", FakeStepperEnable(self.enable_line)
        )
        self.printer.add_object("force_move", FakeForceMove(self.stepper))
        sections = {}
        FakeConfig(
            "motor stepper_x", {"microsteps": 16}, self.printer, sections
        )
        config = FakeConfig("tmc5160 stepper_x", {}, self.printer, sections)
        fields = tmc.FieldHelper(
            tmc5160.Fields, tmc5160.SignedFields, tmc5160.FieldFormatters
        )
        self.mcu_tmc = FakeMcuTmc(fields, self.wire)
        self.helper = tmc.TMCCommandHelper(
            config, self.mcu_tmc, FakeCurrentHelper()
        )
        fields.set_field("toff", 3)
        fields.set_field("multistep_filt", 1)

    def boot(self):
        self.printer.fire_event("klippy:mcu_identify")
        self.printer.fire_event("klippy:connect")


def test_gcode_commands_are_registered():
    rig = Rig()
    assert [cmd for cmd, _ in rig.gcode.mux_commands] == [
        "SET_TMC_FIELD",
        "INIT_TMC",
        "SET_TMC_CURRENT",
    ]


def test_connect_writes_full_register_cache_in_insertion_order():
    rig = Rig()
    rig.boot()
    assert writes(rig.wire) == [
        ("write", "CHOPCONF", CHOPCONF_RUN, None),
        ("write", "GCONF", GCONF_MULTISTEP_FILT, None),
    ], "connect-time init is unstamped (immediate) and cache-ordered"


def test_connect_sets_dedge_only_when_stepper_steps_on_both_edges():
    rig = Rig(step_both_edge=False)
    rig.boot()
    chopconf = writes(rig.wire)[0][2]
    assert chopconf == CHOPCONF_RUN & ~(1 << 29)


def test_connect_skips_writes_when_mcu_is_disconnected():
    rig = Rig()
    rig.mcu_tmc.mcu.non_critical_disconnected = True
    rig.boot()
    assert writes(rig.wire) == []


def test_virtual_enable_connect_holds_toff_at_zero():
    rig = Rig(dedicated_enable=False)
    rig.boot()
    assert writes(rig.wire)[0] == (
        "write",
        "CHOPCONF",
        CHOPCONF_TOFF_OFF,
        None,
    ), "driver must come up disabled when enable is virtual (toff=0)"


def test_enable_without_reset_restores_only_toff_stamped_at_print_time():
    rig = Rig(dedicated_enable=False)
    rig.boot()
    del rig.wire[:]
    rig.enable_line.state_callback(10.0, True)
    assert ops(rig.wire) == [
        ("read", "DRV_STATUS"),
        ("read", "GSTAT"),
        ("timer+", "_do_periodic_check"),
        ("write", "CHOPCONF"),
    ], "reset probe runs before the driver is switched on"
    assert writes(rig.wire) == [("write", "CHOPCONF", CHOPCONF_RUN, 10.0)]


def test_enable_after_driver_reset_replays_the_full_register_cache():
    rig = Rig(dedicated_enable=False)
    rig.boot()
    rig.mcu_tmc.reads["GSTAT"] = [GSTAT_RESET, 0]
    del rig.wire[:]
    rig.enable_line.state_callback(10.0, True)
    assert writes(rig.wire) == [
        ("write", "GSTAT", GSTAT_RESET, None),
        ("write", "CHOPCONF", CHOPCONF_RUN, None),
        ("write", "GCONF", GCONF_MULTISTEP_FILT, None),
    ], "full reinit writes are unstamped even on the enable path"


def test_enable_is_inline_but_disable_is_deferred_to_the_reactor():
    rig = Rig(dedicated_enable=False)
    rig.boot()
    del rig.wire[:]
    rig.enable_line.state_callback(10.0, True)
    assert writes(rig.wire), "enable must write before the first move ships"
    del rig.wire[:]
    rig.enable_line.state_callback(11.0, False)
    assert rig.wire == [], "disable waits for the reactor"
    rig.printer.get_reactor().run_callbacks()
    assert writes(rig.wire) == [("write", "CHOPCONF", CHOPCONF_TOFF_OFF, 11.0)]
    timer_events = [op for op in ops(rig.wire) if op[0].startswith("timer")]
    assert timer_events == [("timer-", "_do_periodic_check")], (
        "health checks stop with the driver"
    )


def test_dedicated_enable_does_not_touch_toff():
    rig = Rig(dedicated_enable=True)
    rig.boot()
    del rig.wire[:]
    rig.enable_line.state_callback(10.0, True)
    assert writes(rig.wire) == [], (
        "no reset detected and no virtual toff: nothing to write"
    )
    rig.enable_line.state_callback(11.0, False)
    rig.printer.get_reactor().run_callbacks()
    assert writes(rig.wire) == []


def test_enable_failure_shuts_the_printer_down():
    rig = Rig(dedicated_enable=False)
    rig.boot()
    rig.mcu_tmc.reads["GSTAT"] = CommandError("SPI transfer failed")
    rig.enable_line.state_callback(10.0, True)
    assert len(rig.printer.shutdowns) == 1
    assert "enable failed" in rig.printer.shutdowns[0]


def test_enable_without_reset_detection_always_replays_the_registers():
    # TMC2130-generation drivers cannot clear GSTAT, so a prior driver
    # reset is unprovable — every enable must assume one happened.
    wire = []
    printer = FakePrinter(wire)
    enable_line = FakeEnableLine(dedicated=True)
    printer.add_object("gcode", FakeGCode())
    printer.add_object("stepper_enable", FakeStepperEnable(enable_line))
    printer.add_object("force_move", FakeForceMove(FakeStepper()))
    sections = {}
    FakeConfig("motor stepper_y", {"microsteps": 16}, printer, sections)
    config = FakeConfig("tmc2130 stepper_y", {}, printer, sections)
    fields = tmc.FieldHelper(
        tmc2130.Fields, tmc2130.SignedFields, tmc2130.FieldFormatters
    )
    mcu_tmc = FakeMcuTmc(fields, wire)
    tmc.TMCCommandHelper(config, mcu_tmc, FakeCurrentHelper())
    fields.set_field("toff", 4)
    printer.fire_event("klippy:mcu_identify")
    printer.fire_event("klippy:connect")
    del wire[:]
    enable_line.state_callback(10.0, True)
    assert [w[1] for w in writes(wire)] == ["CHOPCONF"], (
        "full unstamped cache replay despite no reset having been detected"
    )
    assert writes(wire)[0][3] is None
    assert printer.shutdowns == []


class FakeToolhead:
    def get_last_move_time(self):
        return 12.0


def test_init_tmc_replays_the_desired_config_stamped_at_print_time():
    rig = Rig()
    rig.boot()
    rig.printer.add_object("toolhead", FakeToolhead())
    del rig.wire[:]
    rig.helper.cmd_INIT_TMC(None)
    assert writes(rig.wire) == [
        ("write", "CHOPCONF", CHOPCONF_RUN, 12.0),
        ("write", "GCONF", GCONF_MULTISTEP_FILT, 12.0),
    ]


def test_init_tmc_fails_loudly_outside_pulse_mode():
    # A full register replay would overwrite the transient mode overrides
    # (direct_mode, StallGuard thresholds) with the desired config,
    # silently knocking the driver out of its mode.
    rig = Rig()
    rig.boot()
    for mode in (
        tmc.TMCModeTracker.PHASE_DIRECT,
        tmc.TMCModeTracker.SG_HOMING,
    ):
        rig.helper.mode_tracker.mode = mode
        del rig.wire[:]
        with pytest.raises(CommandError, match="INIT_TMC"):
            rig.helper.cmd_INIT_TMC(None)
        assert writes(rig.wire) == []


def test_mode_tracker_follows_the_enable_disable_lifecycle():
    rig = Rig(dedicated_enable=False)
    rig.boot()
    tracker = rig.helper.mode_tracker
    assert tracker.mode == tmc.TMCModeTracker.DISABLED
    rig.enable_line.state_callback(10.0, True)
    assert tracker.mode == tmc.TMCModeTracker.PULSE
    rig.enable_line.state_callback(11.0, False)
    rig.printer.get_reactor().run_callbacks()
    assert tracker.mode == tmc.TMCModeTracker.DISABLED


def test_phase_stepping_post_enable_callback_owns_the_checks():
    rig = Rig(dedicated_enable=False)
    calls = []
    rig.helper.set_post_enable_callback(lambda: calls.append("enter_phase"))
    rig.boot()
    del rig.wire[:]
    calls.clear()
    rig.enable_line.state_callback(10.0, True)
    assert calls == ["enter_phase"]
    assert writes(rig.wire) == [
        ("write", "CHOPCONF", CHOPCONF_RUN, None),
        ("write", "GCONF", GCONF_MULTISTEP_FILT, None),
    ], "phase-mode enable always does a full init, unstamped"
    timer_events = [op for op in ops(rig.wire) if op[0].startswith("timer")]
    assert timer_events == [], "no periodic checks around phase-mode entry"
