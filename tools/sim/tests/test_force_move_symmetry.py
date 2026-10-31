import pytest

from tools.sim import configs

from .test_resonance_buzz import X_STEP_LINE, _wait_steps_settled

pytestmark = pytest.mark.needs_elf


def test_symmetric_force_move_pairs_return_to_base(sim_world):
    """Repeated +/-d FORCE_MOVE pairs with a fractional-step distance must
    net zero steps. On the Trident bench each pair drifted the motor by
    ~1.3 microsteps (TMC MSCNT ground truth), which is what desynchronizes
    the belts during motors_sync measurement cycles."""

    def config(world):
        return (
            configs.minimal_config(world.h7_pty, str(world.gcode_dir))
            + "\n[force_move]\nenable_force_move: True\n"
        )

    world = sim_world(config, dual_mcu=False)
    control = world.sim_control("h7")
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("SET_STEPPER_ENABLE STEPPER=x ENABLE=1")

    x_base = control.step_position(X_STEP_LINE)["steps"]
    control.reset_step_times(X_STEP_LINE)
    # 80 steps/mm: 0.373mm = 29.84 steps, a deliberately fractional target.
    pairs = 15
    for _ in range(pairs):
        world.gcode_ok(
            "FORCE_MOVE STEPPER=x DISTANCE=0.373 VELOCITY=80 ACCEL=4000"
        )
        world.gcode_ok(
            "FORCE_MOVE STEPPER=x DISTANCE=-0.373 VELOCITY=80 ACCEL=4000"
        )
    assert _wait_steps_settled(control, X_STEP_LINE, pairs * 2 * 29) >= (
        pairs * 2 * 29
    )
    net = control.step_position(X_STEP_LINE)["steps"] - x_base
    assert net == 0, (
        f"{pairs} symmetric FORCE_MOVE pairs drifted the motor by {net} steps"
    )

    assert world.shutdown_line() is None


def test_symmetric_force_move_pairs_return_to_base_awd(sim_world):
    """Same probe on the bench topology: AWD CoreXY, nudging one motor of a
    coupled twin pair (the motors_sync measurement primitive)."""
    world = sim_world(
        lambda w: configs.awd_corexy_tmc_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    control = world.sim_control("h7")
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("SET_STEPPER_ENABLE STEPPER=a1 ENABLE=1")

    for line in (18, 7, 15, 20):
        control.reset_step_times(line)
    world.gcode_ok("FORCE_MOVE STEPPER=a1 DISTANCE=1.0 VELOCITY=80 ACCEL=4000")
    world.gcode_ok("FORCE_MOVE STEPPER=a1 DISTANCE=-1.0 VELOCITY=80 ACCEL=4000")
    counts = {
        line: _wait_steps_settled(control, line, 1, timeout=6.0)
        for line in (18, 7, 15, 20)
    }
    a1_line = max(counts, key=counts.get)
    assert counts[a1_line] > 0, f"no step line moved: {counts}"
    base = control.step_position(a1_line)["steps"]
    control.reset_step_times(a1_line)
    pairs = 15
    for _ in range(pairs):
        world.gcode_ok(
            "FORCE_MOVE STEPPER=a1 DISTANCE=0.373 VELOCITY=80 ACCEL=4000"
        )
        world.gcode_ok(
            "FORCE_MOVE STEPPER=a1 DISTANCE=-0.373 VELOCITY=80 ACCEL=4000"
        )
    assert _wait_steps_settled(control, a1_line, pairs * 2 * 29) >= (
        pairs * 2 * 29
    )
    net = control.step_position(a1_line)["steps"] - base
    assert net == 0, (
        f"{pairs} symmetric FORCE_MOVE pairs drifted motor a1 by {net} steps"
    )

    assert world.shutdown_line() is None
