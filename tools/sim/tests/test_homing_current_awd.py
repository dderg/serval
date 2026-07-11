"""Homing current on AWD CoreXY: homing one axis moves BOTH belt lanes,
so every motor on both lanes must switch to its own home_current for the
homing move and back to run_current afterwards.

Each of the four XY motors has a distinct home_current, so the expected
GLOBALSCALER value is unique per chip — a chip left at run current (the
coupled-partner-lane bug) is unambiguous in its write log.
"""

import math
import time

import pytest

from tools.sim import configs
from tools.sim.emulators.tmc5160_emulator import GLOBALSCALER
from tools.sim.world import EndstopPulser

pytestmark = pytest.mark.needs_elf

TMC5160_VREF = 0.325


def _expected_globalscaler(current: float) -> int:
    value = math.floor(
        current
        * 32
        * 256
        * configs.AWD_TMC_SENSE_RESISTOR
        * math.sqrt(2.0)
        / (32 * TMC5160_VREF)
    )
    assert 32 <= value < 256, f"test currents must map to plain values: {value}"
    return value


RUN_GS = None  # filled below to keep the expectation table in one place
HOME_GS = {
    name: _expected_globalscaler(home)
    for name, home in configs.AWD_TMC_HOME_CURRENTS.items()
}
RUN_GS = _expected_globalscaler(configs.AWD_TMC_RUN_CURRENT)
assert len(set(HOME_GS.values()) | {RUN_GS}) == 5, (
    "home/run globalscaler values must all be distinct for the assertions"
)


def _globalscaler_writes(world, motor: str):
    chip = world.tmc5160_by_cs[configs.AWD_TMC_CS_LINES[motor]]
    return chip.writes_to(GLOBALSCALER)


def _home_then_restore_seen(world, boot_counts, motor):
    writes = _globalscaler_writes(world, motor)[boot_counts[motor] :]
    return HOME_GS[motor] in writes and len(writes) > 0 and writes[-1] == RUN_GS


def _wait_for_restore(world, boot_counts, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if all(
            _home_then_restore_seen(world, boot_counts, m)
            for m in configs.AWD_TMC_CS_LINES
        ):
            return
        time.sleep(0.2)
    summary = {
        m: _globalscaler_writes(world, m)[boot_counts[m] :]
        for m in configs.AWD_TMC_CS_LINES
    }
    raise AssertionError(
        "homing-current writes never completed on every motor; "
        f"post-boot GLOBALSCALER writes: {summary}"
    )


def test_home_x_switches_current_on_all_four_awd_motors(sim_world):
    world = sim_world(
        lambda w: configs.awd_corexy_tmc_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    boot_counts = {
        m: len(_globalscaler_writes(world, m)) for m in configs.AWD_TMC_CS_LINES
    }
    for motor in configs.AWD_TMC_CS_LINES:
        assert _globalscaler_writes(world, motor)[-1:] == [RUN_GS], (
            f"motor {motor} did not boot at run current"
        )

    with EndstopPulser(world.sim_control("h7"), [(0, 10)]):
        world.gcode_ok("G28 X", timeout=120)
    _wait_for_restore(world, boot_counts)
    assert world.shutdown_line() is None

    for motor in configs.AWD_TMC_CS_LINES:
        writes = _globalscaler_writes(world, motor)[boot_counts[motor] :]
        assert HOME_GS[motor] in writes, (
            f"motor {motor}: home_current globalscaler {HOME_GS[motor]} "
            f"never written during G28 X, saw {writes}"
        )
        assert writes[-1] == RUN_GS, (
            f"motor {motor}: run_current globalscaler {RUN_GS} not restored "
            f"after G28 X, saw {writes}"
        )
        foreign = set(writes) - {HOME_GS[motor], RUN_GS}
        assert not foreign, (
            f"motor {motor}: unexpected globalscaler values {foreign} — "
            f"another motor's current was applied to this chip; saw {writes}"
        )
