"""Bed mesh surface transform end-to-end: the warp lives in the motion
engine's lowerer (docs/rewrite/toolpath-surface-transforms.md), so the
honest observer is the physical Z step counter in the shim (auto-endstop
tracker on step line 15, 800 steps/mm at 16 microsteps and 4mm rotation
distance).

The "wavy" profile spans 20..70mm, is zero at the (45,45) zero
reference, and hits +/-0.10mm at the corner nodes, so expected
corrections at mesh nodes are the stored probe values themselves.
Fade is 1..10mm toward target 0.

Each scenario boots its own world and issues at most ~3 fenced moves:
under the virtual clock, clocksync skew accumulates across M400 fences
until pieces arrive in the MCU past and an axis stalls unexecuted (the
PieceStartInPast family in sim-trip-time-resolution-handoff.md).
test_fenced_move_sequence_executes_all_axes below pins that pre-existing
limit — it fails with no mesh loaded at all.
"""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

Z_STEPS_PER_MM = 800.0
XY_STEPS_PER_MM = 80.0
STEP_LINES = {"x": 18, "y": 7, "z": 15}
TOL_MM = 0.02


def _cfg(world):
    return configs.bed_mesh_config(world.h7_pty, str(world.gcode_dir))


def _fade_factor(z):
    if z >= 10.0:
        return 0.0
    if z >= 1.0:
        return (10.0 - z) / 9.0
    return 1.0


def _settled_steps(world):
    """One motion fence, then all three axis counters."""
    world.gcode_ok("M400", timeout=60)
    world.gcode_ok("G4 P300", timeout=15)
    out = {}
    for axis, line in STEP_LINES.items():
        resp = world.sim_control("h7").send(f"get_steps line={line}")
        assert resp.startswith("steps="), resp
        out[axis] = int(resp.split()[0].split("=", 1)[1])
    return out


class MachineZ:
    """Reads physical Z from the shim step counter, anchored to the gcode
    Z declared by the initial SET_KINEMATIC_POSITION."""

    def __init__(self, world, anchor_gcode_z):
        self.world = world
        self.anchor_z = anchor_gcode_z
        self.anchor_steps = _settled_steps(world)["z"]

    def read(self):
        delta = _settled_steps(self.world)["z"] - self.anchor_steps
        return self.anchor_z + delta / Z_STEPS_PER_MM


def test_activation_is_machine_invariant_and_logged(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=45 Y=45 Z=5")
    machine = MachineZ(world, anchor_gcode_z=5.0)

    world.gcode_ok("BED_MESH_PROFILE LOAD=wavy")
    assert "bed_mesh_activated" in world.events_text()
    assert machine.read() == pytest.approx(5.0, abs=TOL_MM)
    assert world.toolhead_z() == pytest.approx(5.0, abs=TOL_MM)

    world.gcode_ok("G1 Z0.5 F300")
    assert machine.read() == pytest.approx(0.5, abs=TOL_MM)
    assert world.shutdown_line() is None


@pytest.mark.parametrize(
    "x,y,correction",
    [(20.0, 20.0, 0.10), (70.0, 20.0, -0.10), (70.0, 70.0, 0.10)],
)
def test_warp_tracks_the_mesh_at_a_node(sim_world, x, y, correction):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=45 Y=45 Z=5")
    machine = MachineZ(world, anchor_gcode_z=5.0)
    world.gcode_ok("BED_MESH_PROFILE LOAD=wavy")

    world.gcode_ok("G1 Z0.5 F300")
    world.gcode_ok(f"G1 X{x} Y{y} F3000")
    steps = _settled_steps(world)
    assert steps["x"] == pytest.approx((x - 45.0) * XY_STEPS_PER_MM, abs=8)
    assert steps["y"] == pytest.approx((y - 45.0) * XY_STEPS_PER_MM, abs=8)
    machine_z = (
        machine.anchor_z + (steps["z"] - machine.anchor_steps) / Z_STEPS_PER_MM
    )
    assert machine_z == pytest.approx(0.5 + correction, abs=TOL_MM)
    assert world.shutdown_line() is None


def test_fade_scales_and_extinguishes_the_correction(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=20 Y=20 Z=15")
    machine = MachineZ(world, anchor_gcode_z=15.0)
    world.gcode_ok("BED_MESH_PROFILE LOAD=wavy")

    world.gcode_ok("G1 X70 Y20 F3000")
    assert machine.read() == pytest.approx(15.0, abs=TOL_MM), (
        "fully faded XY move must not move Z"
    )

    world.gcode_ok("G1 Z5.5 F300")
    assert machine.read() == pytest.approx(
        5.5 + _fade_factor(5.5) * -0.10, abs=TOL_MM
    )
    assert world.shutdown_line() is None


def test_swaps_keep_machine_z_and_rebase_gcode_z(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=20 Y=20 Z=5.5")
    machine = MachineZ(world, anchor_gcode_z=5.5)

    # Machine 5.5 at (20,20) re-expressed through the warp: solve
    # z + fade(z)*0.1 = 5.5 with fade = (10-z)/9 -> z = 48.5/8.9.
    gcode_z_warped = 48.5 / 8.9
    world.gcode_ok("BED_MESH_PROFILE LOAD=wavy")
    assert machine.read() == pytest.approx(5.5, abs=TOL_MM)
    assert world.toolhead_z() == pytest.approx(gcode_z_warped, abs=1e-6)

    world.gcode_ok("BED_MESH_CLEAR")
    assert machine.read() == pytest.approx(5.5, abs=TOL_MM)
    assert world.toolhead_z() == pytest.approx(5.5, abs=1e-6)

    world.gcode_ok("BED_MESH_PROFILE LOAD=wavy")
    assert machine.read() == pytest.approx(5.5, abs=TOL_MM)
    assert world.toolhead_z() == pytest.approx(gcode_z_warped, abs=1e-6)
    assert world.shutdown_line() is None


def test_steep_mesh_warns_and_check_z_limits_gates(sim_world):
    # The gross-error gate warns instead of refusing (a warped bed still
    # prints, just loudly); CHECK_Z_LIMITS is the hard gate for macros.
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=45 Y=45 Z=5")

    world.gcode_ok("BED_MESH_PROFILE LOAD=steep")
    assert "bed_mesh_z_budget_exceeded" in world.events_text()
    # Position 5 sits at the (45,45) zero reference: activation renames
    # nothing and must not move the machine.
    assert world.toolhead_z() == pytest.approx(5.0, abs=1e-6)

    resp = world.gcode("BED_MESH_CHECK CHECK_Z_LIMITS=1")
    assert "mesh-following needs" in str(resp.get("error", ""))
    assert world.shutdown_line() is None


def test_homing_with_active_mesh_inverts_trigger_z(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=20 Y=20 Z=15")
    world.gcode_ok("BED_MESH_PROFILE LOAD=wavy")

    world.gcode_ok("G28 Z", timeout=120)
    toolhead = world.status().get("toolhead", {})
    assert "z" in toolhead.get("homed_axes", "")

    world.gcode_ok("G1 Z2 F300")
    machine = MachineZ(world, anchor_gcode_z=0.0)
    world.gcode_ok("G1 X70 Y20 F3000")
    correction_delta = _fade_factor(2.0) * (-0.10 - 0.10)
    assert machine.read() == pytest.approx(correction_delta, abs=TOL_MM)
    assert world.shutdown_line() is None


def test_fenced_move_sequence_executes_all_axes(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=120 Y=120 Z=5")
    world.gcode_ok("G1 Z0.5 F300")
    for x, y in [(20.0, 20.0), (220.0, 20.0), (220.0, 220.0)]:
        world.gcode_ok(f"G1 X{x} Y{y} F3000")
        steps = _settled_steps(world)
        assert steps["x"] == pytest.approx(
            (x - 120.0) * XY_STEPS_PER_MM, abs=8
        ), f"X stalled on the way to ({x}, {y})"
        assert steps["y"] == pytest.approx(
            (y - 120.0) * XY_STEPS_PER_MM, abs=8
        ), f"Y stalled on the way to ({x}, {y})"
    assert world.shutdown_line() is None
