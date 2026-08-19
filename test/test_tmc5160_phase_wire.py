"""Characterization: TMC5160 phase-stepping mode transitions on the wire.

Builds real TMC5160 objects (config parsing, current helper, command
helper, error check all live) with only the SPI transport replaced, and
pins the exact enter/exit sequences that were debugged on the bench:

- CHOPCONF (toff>0) must hit the chip before GCONF.direct_mode=1, or the
  bootstrap charge pump starves and the driver trips uv_cp.
- Periodic health checks must be stopped once the ISR owns the SPI bus,
  and restarted only after the handover back to pulse stepping.
- On a multi-motor group (corexy A/B), every motor is jogged to its
  cached MSCNT while still in phase mode; the mode flips to pulse only
  once, after all motors have settled.
"""

import pytest
from tmc_wire_harness import (
    CommandError,
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

from klippy.extras import tmc2130, tmc5160

GCONF_BASE = 1 << 3  # multistep_filt
GCONF_DIRECT_MODE = GCONF_BASE | (1 << 16)


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
        tmc_options = {"sense_resistor": 0.075, "run_current": 0.8}
        tmc_options.update(options)
        config = FakeConfig(
            "tmc5160 " + stepper, tmc_options, self.printer, self.sections
        )
        tmc_obj = tmc5160.TMC5160(config)
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
        ("write", "XTARGET"),
        ("query", "kalico_get_phase_state"),
        ("cmd", "kalico_set_axis_mode"),
        ("query", "kalico_get_phase_state"),
        ("cmd", "kalico_phase_align_to"),
        ("cmd", "kalico_phase_stepping_enable_spi"),
        ("transport", "switch_axis_transport"),
        ("timer-", "_do_periodic_check"),
    ]
    assert tmc_obj.phase_stepping_active()
    assert rig.engine.switches == [(0, 0, 1)], (
        "the mcu executes in phase mode before the host adopts the phase "
        "transport - an anchored lane with a Pulse mode byte is a fault"
    )


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


def test_enter_preloads_xdirect_with_phase_matched_coil_currents(rig):
    tmc_obj = rig.build_tmc(mscnt=256)
    rig.mcu.script_query("kalico_get_phase_state", [phase_state()], oid=7)
    rig.clear()
    tmc_obj._enter_phase_mode_single()
    xdirect = [w for w in writes(rig.wire) if w[1] == "XTARGET"]
    coil_a, coil_b = 0, 248  # mscnt 256 of 1024 = 90 degrees
    assert xdirect == [("write", "XTARGET", (coil_b << 16) | coil_a, None)], (
        "XDIRECT (0x2D) is addressed through its register-map alias XTARGET"
    )


def test_enter_aligns_mcu_to_the_measured_mscnt(rig):
    tmc_obj = rig.build_tmc(mscnt=300)
    rig.mcu.script_query("kalico_get_phase_state", [phase_state()], oid=7)
    tmc_obj._enter_phase_mode_single()
    align = [e for e in rig.wire if e[:2] == ("cmd", "kalico_phase_align_to")]
    assert align == [("cmd", "kalico_phase_align_to", (7, 300))]


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
    jog = [e for e in rig.wire if e[:2] == ("cmd", "kalico_phase_jog_to")]
    assert jog == [("cmd", "kalico_phase_jog_to", (7, 300, 1))], (
        "jog targets the MSCNT cached at entry, one microstep per sample"
    )
    assert writes(rig.wire) == [("write", "GCONF", GCONF_BASE, None)], (
        "direct_mode cleared"
    )
    assert not tmc_obj.phase_stepping_active()
    assert rig.engine.switches[-1] == (0, 0, 0), (
        "the host drains the sample stream and adopts the pulse transport "
        "before the mcu stops executing phase mode"
    )


def test_exit_polls_until_the_jog_settles(rig):
    tmc_obj = rig.build_tmc(mscnt=300)
    rig.mcu.script_query(
        "kalico_get_phase_state",
        [
            phase_state(phase=300),  # consumed by enter (axis_idx lookup)
            phase_state(phase=300),  # consumed by enter (mode confirm)
            phase_state(phase=300),  # exit mode check
            phase_state(phase=120, settled=0),
            phase_state(phase=280, settled=0),
            phase_state(phase=300, settled=1),
        ],
        oid=7,
    )
    tmc_obj._enter_phase_mode_single()
    rig.clear()
    tmc_obj.exit_phase_mode()
    settle_queries = [e for e in rig.wire if e[0] == "query"]
    assert len(settle_queries) == 4
    assert not tmc_obj.phase_stepping_active()


def test_exit_fails_loudly_when_the_jog_never_settles(rig):
    tmc_obj = rig.build_tmc(mscnt=300)
    rig.mcu.script_query(
        "kalico_get_phase_state",
        [phase_state(phase=300), phase_state(phase=120, settled=0)],
        oid=7,
    )
    tmc_obj._enter_phase_mode_single()
    with pytest.raises(CommandError, match="did not settle"):
        tmc_obj.exit_phase_mode()


def test_exit_fails_loudly_on_host_mcu_mode_desync(rig):
    tmc_obj = rig.build_tmc()
    rig.mcu.script_query("kalico_get_phase_state", [phase_state()], oid=7)
    tmc_obj._enter_phase_mode_single()
    rig.mcu.script_query("kalico_get_phase_state", [phase_state(mode=0)], oid=7)
    rig.clear()
    with pytest.raises(CommandError, match="desync"):
        tmc_obj.exit_phase_mode()
    assert [e for e in rig.wire if e[0] in ("write", "cmd")] == [], (
        "desync is detected before any register write or mode command"
    )


def test_exit_without_entering_is_an_error(rig):
    tmc_obj = rig.build_tmc()
    with pytest.raises(CommandError, match="not in phase mode"):
        tmc_obj.exit_phase_mode()


def build_group(rig):
    t1 = rig.build_tmc("stepper_x", oid=7, mscnt=300)
    t2 = rig.build_tmc("stepper_y", oid=8, mscnt=500)
    t1.set_phase_group([t1, t2])
    t2.set_phase_group([t1, t2])
    rig.mcu.script_query(
        "kalico_get_phase_state", [phase_state(axis_idx=0, phase=300)], oid=7
    )
    rig.mcu.script_query(
        "kalico_get_phase_state", [phase_state(axis_idx=1, phase=500)], oid=8
    )
    return t1, t2


def test_group_enter_enters_every_member(rig):
    t1, t2 = build_group(rig)
    t1.enter_phase_mode()
    assert t1._in_phase_mode() and t2._in_phase_mode()
    aligns = [e for e in rig.wire if e[:2] == ("cmd", "kalico_phase_align_to")]
    assert aligns == [
        ("cmd", "kalico_phase_align_to", (7, 300)),
        ("cmd", "kalico_phase_align_to", (8, 500)),
    ]


def test_group_exit_jogs_all_motors_before_any_mode_flip(rig):
    t1, t2 = build_group(rig)
    t1.enter_phase_mode()
    rig.clear()
    t1.exit_phase_mode()
    cmd_names = [e[1] for e in rig.wire if e[0] == "cmd"]
    last_jog = max(
        i for i, n in enumerate(cmd_names) if n == "kalico_phase_jog_to"
    )
    first_flip = min(
        i for i, n in enumerate(cmd_names) if n == "kalico_set_axis_mode"
    )
    assert cmd_names.count("kalico_phase_jog_to") == 2
    assert last_jog < first_flip, (
        "corexy A/B: both motors reach their handover phase while the group"
        " is still in phase mode; flipping one early loses steps"
    )
    assert not t1._in_phase_mode() and not t2._in_phase_mode()


def test_group_exit_flips_each_axis_once_in_sorted_order(rig):
    t1, _t2 = build_group(rig)
    t1.enter_phase_mode()
    rig.clear()
    t1.exit_phase_mode()
    flips = [e for e in rig.wire if e[:2] == ("cmd", "kalico_set_axis_mode")]
    assert flips == [
        ("cmd", "kalico_set_axis_mode", (0, 0)),
        ("cmd", "kalico_set_axis_mode", (1, 0)),
    ]


def test_group_exit_restarts_checks_for_every_member(rig):
    t1, _t2 = build_group(rig)
    t1.enter_phase_mode()
    rig.clear()
    t1.exit_phase_mode()
    restarts = [e for e in rig.wire if e == ("timer+", "_do_periodic_check")]
    assert len(restarts) == 2


def test_phase_stepping_config_rejects_stealthchop(rig):
    with pytest.raises(ConfigError, match="incompatible"):
        rig.build_tmc(stealthchop_threshold=120.0)


def test_phase_stepping_config_requires_256_microsteps(rig):
    with pytest.raises(ConfigError, match="microsteps: 256"):
        rig.build_tmc(motor_options={"microsteps": 16})


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
