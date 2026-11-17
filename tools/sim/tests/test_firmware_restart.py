"""FIRMWARE_RESTART resets every simulated MCU family: from a ready
printer, from an e-stop mid-move, and from MCUs latched in shutdown across a
host restart. The MACH_LINUX firmware resets with config_reset, which
re-executes itself."""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

ENDPOINT_DEATH = '"event":"endpoint_death"'
ENDPOINT_DEATH_WATCHDOG_ABORT = '"event":"endpoint_death_watchdog_abort"'
ENDPOINT_DEATH_GRACE_S = 5.0
SIM_BARRIER_ACK_DEADLINE_S = 60.0
MOVE_UNDERWAY_S = 1.0
X_STEP_LINE = 18
REPEATED_RESTARTS = 4
CONFIG_RESET_AFTER_ESTOP_S = 0.015
ESTOP_HOLD_S = 1.5 * CONFIG_RESET_AFTER_ESTOP_S
ESTOP_HELD = "[vtime] fault: delaying frame carrying msgid"


def _kalico_native(sim_world):
    return sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )


def _classic_stepcompress(sim_world):
    return sim_world(
        lambda w: configs.stepcompress_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )


def _moves_after_restart(world):
    world.mark_log()
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 X135 Y130 F3000")
    world.gcode_ok("M400")
    assert world.toolhead_position()[:2] == pytest.approx([135.0, 130.0])
    assert "shutdown" not in world.log_tail().lower(), world.log_tail()


def _host_rust_events(world):
    path = world.log_dir / "events" / "host-rust.jsonl"
    return path.read_text(errors="replace") if path.exists() else ""


@pytest.mark.parametrize("boot", [_kalico_native, _classic_stepcompress])
def test_firmware_restart_from_ready(sim_world, boot):
    world = boot(sim_world)
    world.firmware_restart()
    _moves_after_restart(world)


def test_repeated_firmware_restarts_of_two_mcus(sim_world):
    """Each restart re-executes both MCU processes; the sim's device-link
    table ran out on the second restart of a two-MCU world (scenario
    fuzzer, seed 57)."""
    world = _classic_stepcompress(sim_world)
    for _ in range(REPEATED_RESTARTS):
        world.firmware_restart()
    _moves_after_restart(world)


@pytest.mark.parametrize("boot", [_kalico_native, _classic_stepcompress])
def test_firmware_restart_recovers_mcus_latched_across_a_host_restart(
    sim_world, boot
):
    world = boot(sim_world)
    world.gcode("M112")
    settled = world.restart_klippy()
    assert settled in ("shutdown", "error"), (settled, world.log_tail())
    world.firmware_restart()
    _moves_after_restart(world)


@pytest.mark.parametrize("boot", [_kalico_native, _classic_stepcompress])
def test_estop_mid_move_latches_no_endpoint_death(sim_world, boot):
    world = boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 X200 F600")
    underway_ns = world.vclock.now_ns() + int(MOVE_UNDERWAY_S * 1e9)
    world.wait_until(
        lambda: world.vclock.now_ns() > underway_ns,
        timeout=2 * MOVE_UNDERWAY_S,
        what="the move to get underway",
    )
    world.emergency_stop()
    deadline_ns = world.vclock.now_ns() + int(
        (SIM_BARRIER_ACK_DEADLINE_S + ENDPOINT_DEATH_GRACE_S) * 1e9
    )
    world.wait_until(
        lambda: world.vclock.now_ns() > deadline_ns,
        timeout=2 * (SIM_BARRIER_ACK_DEADLINE_S + ENDPOINT_DEATH_GRACE_S),
        what="the barrier-ack deadline to pass",
    )
    assert ENDPOINT_DEATH not in _host_rust_events(world)
    world.firmware_restart()
    _moves_after_restart(world)


def test_firmware_restart_right_after_a_latched_endpoint_death(sim_world):
    world = _classic_stepcompress(sim_world)
    control = world.sim_control("f4")
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.inject(
        "f4",
        late_command="stepcompress_barrier",
        by_s=SIM_BARRIER_ACK_DEADLINE_S + ENDPOINT_DEATH_GRACE_S,
    )
    before = control.step_position(X_STEP_LINE)["steps"]
    world.gcode_ok("G1 X135 F3000")
    world.wait_until(
        lambda: control.step_position(X_STEP_LINE)["steps"] != before,
        timeout=10.0,
        what="the move reaching the MCU",
    )
    world.wait_until(
        lambda: ENDPOINT_DEATH in _host_rust_events(world),
        timeout=SIM_BARRIER_ACK_DEADLINE_S + ENDPOINT_DEATH_GRACE_S,
        what="the endpoint death the late barrier latches",
        poll=0.01,
    )
    grace_end_ns = world.vclock.now_ns() + int(ENDPOINT_DEATH_GRACE_S * 1e9)
    world.firmware_restart()
    world.wait_until(
        lambda: world.vclock.now_ns() > grace_end_ns,
        timeout=2 * ENDPOINT_DEATH_GRACE_S,
        what="the endpoint-death grace to pass",
    )
    assert ENDPOINT_DEATH_WATCHDOG_ABORT not in _host_rust_events(world)
    assert world.klippy_proc.poll() is None
    _moves_after_restart(world)


def test_config_reset_read_with_the_emergency_stop_resets_the_mcu(sim_world):
    """#879: klippy sends config_reset 15 ms after emergency_stop. A link
    held past that gap hands the MCU both frames in one read; the
    emergency_stop shutdown must not discard the config_reset behind it."""
    world = _kalico_native(sim_world)
    world.inject("h7", delay_command="emergency_stop", by_s=ESTOP_HOLD_S)
    world.firmware_restart()
    assert ESTOP_HELD in (world.log_dir / "h7.log").read_text(errors="replace")
    _moves_after_restart(world)
