"""Characterization: TMCVirtualPinHelper StallGuard arm/disarm sequences.

The arm/disarm dance saves driver thresholds into ad-hoc instance fields,
rewrites them for a sensorless homing move, and restores them afterwards.
These tests pin the exact save/restore write sequence for both driver
generations (en_pwm_mode drivers like the 5160, sgthrs drivers like the
2209) and the interplay with phase-stepping mode.
"""

import pytest
from tmc_wire_harness import (
    CommandError,
    FakeConfig,
    FakeMcuTmc,
    FakePins,
    FakePrinter,
    writes,
)

from klippy.extras import tmc, tmc2208, tmc2209, tmc5160

GCONF_EN_PWM = 1 << 2
GCONF_DIAG1_STALL = 1 << 8
THIGH_CONFIGURED = 0x123
TPWMTHRS_CONFIGURED = 0x555
SGTHRS_CONFIGURED = 100


def build_5160():
    wire = []
    printer = FakePrinter(wire)
    printer.add_object("pins", FakePins())
    config = FakeConfig(
        "tmc5160 stepper_x", {"diag1_pin": "PA1"}, printer, sections={}
    )
    fields = tmc.FieldHelper(
        tmc5160.Fields, tmc5160.SignedFields, tmc5160.FieldFormatters
    )
    mcu_tmc = FakeMcuTmc(fields, wire)
    helper = tmc.TMCVirtualPinHelper(config, mcu_tmc)
    fields.set_field("en_pwm_mode", 1)
    fields.set_field("thigh", THIGH_CONFIGURED)
    return wire, printer, helper


def build_2209():
    wire = []
    printer = FakePrinter(wire)
    printer.add_object("pins", FakePins())
    config = FakeConfig(
        "tmc2209 stepper_x", {"diag_pin": "PA1"}, printer, sections={}
    )
    fields = tmc.FieldHelper(
        tmc2209.Fields, tmc2208.SignedFields, tmc2209.FieldFormatters
    )
    mcu_tmc = FakeMcuTmc(fields, wire)
    helper = tmc.TMCVirtualPinHelper(config, mcu_tmc)
    fields.set_field("sgthrs", SGTHRS_CONFIGURED)
    fields.set_field("tpwmthrs", TPWMTHRS_CONFIGURED)
    return wire, printer, helper


def test_5160_arm_forces_spreadcycle_routes_diag_and_opens_thresholds():
    wire, _printer, helper = build_5160()
    helper.arm()
    assert writes(wire) == [
        ("write", "GCONF", GCONF_DIAG1_STALL, None),
        ("write", "TCOOLTHRS", 0xFFFFF, None),
        ("write", "THIGH", 0, None),
    ], "en_pwm_mode cleared and diag1_stall set in one GCONF write"


def test_5160_disarm_restores_the_exact_prior_configuration():
    wire, _printer, helper = build_5160()
    helper.arm()
    del wire[:]
    helper.disarm()
    assert writes(wire) == [
        ("write", "GCONF", GCONF_EN_PWM, None),
        ("write", "TCOOLTHRS", 0, None),
        ("write", "THIGH", THIGH_CONFIGURED, None),
    ]


def test_5160_arm_disarm_round_trip_leaves_tcoolthrs_residue_in_cache():
    wire, _printer, helper = build_5160()
    fields = helper.fields
    before = dict(fields.registers)
    helper.arm()
    helper.disarm()
    residue = {
        reg: val
        for reg, val in fields.registers.items()
        if before.get(reg) != val
    }
    assert residue == {"TCOOLTHRS": 0}, (
        "arm/disarm materializes TCOOLTHRS=0 in the register cache (same as"
        " the implicit default, but a later full reinit now writes it too)"
    )


def test_2209_arm_refreshes_sgthrs_and_forces_stealthchop():
    wire, _printer, helper = build_2209()
    helper.arm()
    assert writes(wire) == [
        ("write", "SGTHRS", SGTHRS_CONFIGURED, None),
        ("write", "TPWMTHRS", 0, None),
        ("write", "GCONF", 0, None),
        ("write", "TCOOLTHRS", 0xFFFFF, None),
    ], "sgthrs drivers need stealthchop ON (en_spreadcycle=0) to stall-detect"


def test_2209_disarm_restores_thresholds():
    wire, _printer, helper = build_2209()
    helper.arm()
    del wire[:]
    helper.disarm()
    assert writes(wire) == [
        ("write", "TPWMTHRS", TPWMTHRS_CONFIGURED, None),
        ("write", "GCONF", 0, None),
        ("write", "TCOOLTHRS", 0, None),
    ]


class FakePhaseModeHelper:
    def __init__(self, sticky_active=False):
        self.active = True
        self.calls = []
        self._sticky = sticky_active

    def phase_stepping_active(self):
        return self.active

    def exit_phase_mode(self):
        self.calls.append("exit")
        if not self._sticky:
            self.active = False

    def enter_phase_mode(self):
        self.calls.append("enter")
        self.active = True


def test_arm_exits_phase_mode_before_touching_registers():
    wire, _printer, helper = build_5160()
    pmh = FakePhaseModeHelper()
    helper.phase_mode_helper = pmh
    helper.arm()
    assert pmh.calls == ["exit"]
    assert writes(wire), "register rewrites happen after the mode exit"


def test_disarm_reenters_phase_mode_after_restoring_registers():
    wire, _printer, helper = build_5160()
    pmh = FakePhaseModeHelper()
    helper.phase_mode_helper = pmh
    helper.arm()
    del wire[:]
    helper.disarm()
    assert pmh.calls == ["exit", "enter"]
    assert len(writes(wire)) == 3, "restore writes precede the re-enter"


def test_disarm_without_prior_phase_exit_does_not_reenter():
    _wire, _printer, helper = build_5160()
    pmh = FakePhaseModeHelper()
    pmh.active = False
    helper.phase_mode_helper = pmh
    helper.arm()
    helper.disarm()
    assert pmh.calls == []


def test_arm_fails_loudly_if_phase_mode_refuses_to_exit():
    wire, _printer, helper = build_5160()
    helper.phase_mode_helper = FakePhaseModeHelper(sticky_active=True)
    with pytest.raises(CommandError, match="still active"):
        helper.arm()
    assert writes(wire) == [], "no register writes on a failed mode exit"
