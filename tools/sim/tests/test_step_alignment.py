import re
import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

CLOCK_FREQ = 50_000_000
NSECS_PER_TICK = 1_000_000_000 // CLOCK_FREQ
THRESHOLD_FRAC = 0.5
DISTANCE_MM = 40.0
VELOCITY_MM_S = 20.0
TOLERANCE_NS = 50_000
# The H7 motion tick runs at 10 kHz; at the sim's default virtual-clock speed
# the tick thread is starved under CPU load, so it advances virtual time in
# catch-up bursts that pack several sample periods into one and scatter the
# dispatched step cycles. Running the virtual clock slower gives the tick the
# real-time headroom to sample every period, so steps land on their analytic
# crossings (as they do for the slower-ticking F4 at default speed).
VTIME_SPEED = "0.2"


H7_STEPS_PER_MM = 80.0
F4_STEPS_PER_MM = 800.0


def _cfg(world):
    return configs.dual_mcu_alignment_config(
        world.h7_pty,
        world.f4_pty,
        str(world.gcode_dir),
    )


def _armed_start_cycles(world, mcu_name, axis) -> int:
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        world.mark_log()
        world.gcode_ok("MCU_SIM_ARMED_WINDOW MCU=%s AXIS=%d" % (mcu_name, axis))
        response = world.expect_log("MCU_SIM_ARMED_WINDOW ")
        match = re.search(r"armed=(-?\d+) occupancy=\d+ start=(\d+)", response)
        assert match, response
        if int(match.group(1)) == 1:
            return int(match.group(2))
        time.sleep(0.05)
    raise AssertionError("axis %d on %s never armed a piece" % (axis, mcu_name))


def _measure_lag(world, stepper, sim_mcu, mcu_name, axis, line, steps_per_mm):
    world.sim_control(sim_mcu).reset_step_times(line)
    world.gcode_ok(
        "MCU_SIM_CONSTANT_MOVE STEPPER=%s DISTANCE=%.9f VELOCITY=%.9f"
        % (stepper, DISTANCE_MM, VELOCITY_MM_S),
        timeout=30,
    )
    start_cycles = _armed_start_cycles(world, mcu_name, axis)
    world.gcode_ok("M400", timeout=120)

    times = world.sim_control(sim_mcu).get_step_times(line)
    count = times["count"]
    assert count > 1, "%s line %d recorded %d steps" % (sim_mcu, line, count)
    # Work in the MCU cycle domain: the armed-piece start and every dispatched
    # step time are MCU cycle counts, so the per-MCU offset between the cycle
    # counter and the shared virtual clock cancels. Step k (0-based) of a
    # constant-velocity train crosses (k + THRESHOLD_FRAC) microsteps past the
    # start, so the dispatched times form the line
    # t_k = start + (k + THRESHOLD_FRAC) * interval. A +T execution lag shifts
    # the whole line (the intercept) by T; any residual clock-rate error lives
    # in the slope. Least-squares over all N steps recovers slope and intercept
    # robustly (immune to a stray arming/retirement boundary step), and the lag
    # is read off the intercept.
    n = count
    sum_k = n * (n - 1) // 2
    sum_k2 = (n - 1) * n * (2 * n - 1) // 6
    sum_t = times["sum_cycles"]
    sum_kt = times["sum_index_cycles"]
    slope_cyc = (n * sum_kt - sum_k * sum_t) / (n * sum_k2 - sum_k * sum_k)
    intercept_cyc = (sum_t - slope_cyc * sum_k) / n
    cps_nom = CLOCK_FREQ / (steps_per_mm * VELOCITY_MM_S)
    commanded_intercept_cyc = start_cycles + THRESHOLD_FRAC * slope_cyc
    lag_ns = (intercept_cyc - commanded_intercept_cyc) * NSECS_PER_TICK
    print(
        "%s stepper=%s start_cycles=%d count=%d cps_nom=%.3f slope_obs=%.3f "
        "intercept_cyc=%.1f cmd_intercept_cyc=%.1f lag=%+.0fns"
        % (
            sim_mcu,
            stepper,
            start_cycles,
            count,
            cps_nom,
            slope_cyc,
            intercept_cyc,
            commanded_intercept_cyc,
            lag_ns,
        )
    )
    return lag_ns


def test_step_alignment_across_mcu_tickrates(sim_world, monkeypatch):
    monkeypatch.setenv("VTIME_SPEED", VTIME_SPEED)
    world = sim_world(_cfg, dual_mcu=True)
    world.gcode_ok("SET_KINEMATIC_POSITION X=100 Y=100 Z=10")
    world.gcode_ok("M400", timeout=60)

    lag_h7 = _measure_lag(world, "a", "h7", "mcu", 0, 18, H7_STEPS_PER_MM)
    lag_f4 = _measure_lag(world, "z", "f4", "bottom", 2, 15, F4_STEPS_PER_MM)

    assert abs(lag_h7) < TOLERANCE_NS, (
        "h7 step lag %+.0fns exceeds tolerance %dns (T_h7=100us pre-fix)"
        % (lag_h7, TOLERANCE_NS)
    )
    assert abs(lag_f4) < TOLERANCE_NS, (
        "f4 step lag %+.0fns exceeds tolerance %dns (T_f4=200us pre-fix)"
        % (lag_f4, TOLERANCE_NS)
    )
    assert world.shutdown_line() is None, world.shutdown_line()
