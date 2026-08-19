"""Characterization: TMC2240 phase-stepping mode transitions on the wire.

The TMC2240 shares the TMC5160's direct-mode machinery (same GCONF bit,
same 0x2D coil register, same SPI framing); these tests pin the
2240-specific seams: the coil register is named DIRECT_MODE, the current
helper carries the run current in IHOLD, and phase stepping is refused
on a UART-connected driver.
"""

import pytest
from tmc_wire_harness import (
    ConfigError,
    FakeConfig,
    FakeEnableLine,
    FakeGcode,
    FakeMCU,
    FakeMcuTmc,
    FakeMotionEngine,
    FakePins,
    FakePrinter,
    FakeStepperEnable,
    ops,
    writes,
)

from klippy.extras import tmc2130, tmc2240

GCONF_BASE = 1 << 3  # multistep_filt
GCONF_DIRECT_MODE = GCONF_BASE | (1 << 16)


class FakeHeaters:
    def register_monitor(self, config):
        pass


def phase_state(axis_idx=0, mode=1, phase=256, settled=1):
    return {
        "axis_idx": axis_idx,
        "mode": mode,
        "phase": phase,
        "settled": settled,
    }


class Rig:
    def __init__(self, monkeypatch):
        self.wire = []
        self.mcu = FakeMCU(self.wire)
        self.printer = FakePrinter(self.wire)
        self.printer.add_object("pins", FakePins())
        self.printer.add_object("gcode", FakeGcode())
        self.printer.add_object(
            "stepper_enable", FakeStepperEnable(FakeEnableLine())
        )
        self.printer.add_object("heaters", FakeHeaters())
        self.engine = FakeMotionEngine(self.wire)
        self.printer.add_object("motion_engine", self.engine)
        self.sections = {}
        self.mcu_tmcs = []

        def fake_spi(config, registers, fields, frequency):
            mcu_tmc = FakeMcuTmc(fields, self.wire, mcu=self.mcu)
            self.mcu_tmcs.append(mcu_tmc)
            return mcu_tmc

        monkeypatch.setattr(tmc2130, "MCU_TMC_SPI", fake_spi)

    def build_tmc(self, stepper="stepper_x", oid=7, mscnt=256, **options):
        motor_options = {"microsteps": 256, "phase_stepping": True}
        motor_options.update(options.pop("motor_options", {}))
        FakeConfig(
            "motor " + stepper, motor_options, self.printer, self.sections
        )
        tmc_options = {"rref": 12000, "run_current": 0.8}
        tmc_options.update(options)
        config = FakeConfig(
            "tmc2240 " + stepper, tmc_options, self.printer, self.sections
        )
        tmc_obj = tmc2240.TMC2240(config)
        tmc_obj.set_phase_stepper_oid(oid)
        self.mcu_tmcs[-1].reads["MSCNT"] = mscnt
        return tmc_obj

    def start_checks(self, tmc_obj):
        tmc_obj._echeck_helper.start_checks()

    def clear(self):
        del self.wire[:]


@pytest.fixture
def rig(monkeypatch):
    return Rig(monkeypatch)


def test_enter_sequence_chopconf_before_direct_mode_checks_stopped_last(rig):
    tmc_obj = rig.build_tmc()
    rig.mcu.script_query("kalico_get_phase_state", [phase_state()], oid=7)
    rig.start_checks(tmc_obj)
    rig.clear()
    tmc_obj._enter_phase_mode_single()
    assert ops(rig.wire) == [
        ("cmd", "kalico_phase_stepping_disable_spi"),
        ("write", "CHOPCONF"),
        ("write", "GCONF"),
        ("read", "MSCNT"),
        ("write", "DIRECT_MODE"),
        ("query", "kalico_get_phase_state"),
        ("cmd", "kalico_set_axis_mode"),
        ("query", "kalico_get_phase_state"),
        ("cmd", "kalico_phase_align_to"),
        ("cmd", "kalico_phase_stepping_enable_spi"),
        ("transport", "switch_axis_transport"),
        ("timer-", "_do_periodic_check"),
    ]
    assert tmc_obj.phase_stepping_active()


def test_enter_sets_direct_mode_and_forces_spreadcycle(rig):
    tmc_obj = rig.build_tmc()
    rig.mcu.script_query("kalico_get_phase_state", [phase_state()], oid=7)
    rig.clear()
    tmc_obj._enter_phase_mode_single()
    gconf_writes = [w for w in writes(rig.wire) if w[1] == "GCONF"]
    assert gconf_writes == [("write", "GCONF", GCONF_DIRECT_MODE, None)]
    assert tmc_obj.fields.registers["GCONF"] == GCONF_BASE, (
        "direct_mode is a transient override; the desired config stays clean"
        " so a full register replay cannot resurrect it out of phase mode"
    )


def test_enter_preloads_direct_mode_with_phase_matched_coil_currents(rig):
    tmc_obj = rig.build_tmc(mscnt=256)
    rig.mcu.script_query("kalico_get_phase_state", [phase_state()], oid=7)
    rig.clear()
    tmc_obj._enter_phase_mode_single()
    preload = [w for w in writes(rig.wire) if w[1] == "DIRECT_MODE"]
    coil_a, coil_b = 0, 248  # mscnt 256 of 1024 = 90 degrees
    assert preload == [
        ("write", "DIRECT_MODE", (coil_b << 16) | coil_a, None)
    ], "the TMC2240 names its 0x2D coil register DIRECT_MODE"


def test_exit_jogs_back_to_cached_mscnt_then_flips_mode_then_restarts_checks(
    rig,
):
    tmc_obj = rig.build_tmc(mscnt=300)
    rig.mcu.script_query(
        "kalico_get_phase_state", [phase_state(phase=300)], oid=7
    )
    tmc_obj._enter_phase_mode_single()
    rig.clear()
    tmc_obj.exit_phase_mode()
    assert ops(rig.wire) == [
        ("query", "kalico_get_phase_state"),
        ("cmd", "kalico_phase_jog_to"),
        ("query", "kalico_get_phase_state"),
        ("cmd", "kalico_phase_stepping_disable_spi"),
        ("write", "GCONF"),
        ("transport", "switch_axis_transport"),
        ("cmd", "kalico_set_axis_mode"),
        ("read", "DRV_STATUS"),
        ("read", "GSTAT"),
        ("timer+", "_do_periodic_check"),
    ]
    assert writes(rig.wire) == [("write", "GCONF", GCONF_BASE, None)], (
        "direct_mode cleared"
    )
    assert not tmc_obj.phase_stepping_active()


def test_phase_stepping_config_rejects_stealthchop(rig):
    with pytest.raises(ConfigError, match="incompatible"):
        rig.build_tmc(stealthchop_threshold=120.0)


def test_phase_stepping_config_requires_256_microsteps(rig):
    with pytest.raises(ConfigError, match="microsteps: 256"):
        rig.build_tmc(motor_options={"microsteps": 16})


def test_phase_stepping_config_rejects_uart(rig):
    with pytest.raises(ConfigError, match="requires SPI"):
        rig.build_tmc(uart_pin="gpiochip0/gpio9")


def test_phase_stepping_extends_standstill_timeout_to_maximum(rig):
    tmc_obj = rig.build_tmc()
    assert tmc_obj.fields.get_field("tpowerdown") == 255, (
        "direct mode has no step pulses; default tpowerdown would cut coil"
        " power mid-print"
    )


def test_direct_mode_maps_ihold_to_irun(rig):
    tmc_obj = rig.build_tmc()
    fields = tmc_obj.fields
    assert fields.get_field("ihold") == fields.get_field("irun"), (
        "no step edges in direct mode, so IRUN never applies; IHOLD carries"
        " the run current"
    )


def test_without_phase_stepping_ihold_stays_hold_current(rig):
    tmc_obj = rig.build_tmc(
        motor_options={"phase_stepping": False, "microsteps": 16},
        hold_current=0.4,
    )
    fields = tmc_obj.fields
    assert fields.get_field("ihold") < fields.get_field("irun")
    assert fields.get_field("tpowerdown") == 10
    assert not tmc_obj._phase_stepping
