"""A bed mesh active while an extruder that follows x, y, z prints.

Found while reproducing #362: with a mesh loaded, the heaters world (X/Y
without a shaper, kernel-free pressure advance on e) panicked klippy's
motion pool on the first extruding moves:
`follower_projection.rs: leader 0 has no velocity bounds ...: a variable
surface must be replaced by a spline before dispatch`. The cases separate
the leader shaping, the extruder kernel and the mesh itself.
"""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

PRINT_LINES = 40
PRINT_TIMEOUT_S = 300
EXTRUDER_SMOOTH_TIME = 0.02
SHAPED_ENDSTOPS = ("gpio200", "gpio201")
MESH_STATES = ("mesh", "no_mesh")
FATAL_EVENTS = (
    "panicked",
    "stream worker fatal",
    "stream_worker_fatal",
    "pump_piece_in_past",
    "runtime_fault",
)

MESH_SECTIONS = """
[post_processor shaper_xy]
type: smooth_bell
smooth_time: 0.019125

[bed_mesh]
mesh_min: 20, 20
mesh_max: 180, 180
probe_count: 3, 3
zero_reference_position: 100, 100

#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [bed_mesh flat]
#*# version = 1
#*# points =
#*#   0.02, 0.00, -0.02
#*#   0.01, 0.00, -0.01
#*#   -0.02, 0.00, 0.02
#*# min_x = 20.0
#*# max_x = 180.0
#*# min_y = 20.0
#*# max_y = 180.0
#*# x_count = 3
#*# y_count = 3
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = lagrange
#*# tension = 0.2
"""

LEADERS = ("unshaped_xy", "shaped_xy")
EXTRUDER_CHAINS = ("pa_only", "pa_kernel")


def _config(world, leaders, extruder):
    config = configs.heaters_config(world.h7_pty, str(world.gcode_dir))
    if extruder == "pa_kernel":
        config = configs.with_extruder_smoothing(config, EXTRUDER_SMOOTH_TIME)
    if leaders == "shaped_xy":
        for endstop in SHAPED_ENDSTOPS:
            anchor = f"endstop_pin: ^gpiochip0/{endstop}\nhoming_speed: 10\n"
            assert anchor in config, anchor
            config = config.replace(
                anchor, anchor + "post_processors: shaper_xy\n"
            )
    return config + MESH_SECTIONS


def _print_lines():
    lines = ["G90", "M83", "G1 Z0.3 F600", "G1 X60 Y60 F12000"]
    for index in range(PRINT_LINES):
        x = 140.0 if index % 2 == 0 else 60.0
        y = 60.0 + (index // 2) * 0.4
        lines.append(f"G1 X{x:.1f} Y{y:.2f} E0.3 F6000")
    return lines


@pytest.mark.parametrize("mesh", MESH_STATES)
@pytest.mark.parametrize("extruder", EXTRUDER_CHAINS)
@pytest.mark.parametrize("leaders", LEADERS)
def test_print_with_mesh_loaded(sim_world, leaders, extruder, mesh):
    world = sim_world(lambda w: _config(w, leaders, extruder), dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=100 Y=100 Z=5")
    if mesh == "mesh":
        world.gcode_ok("BED_MESH_PROFILE LOAD=flat")
    path = world.gcode_dir / "mesh_print.gcode"
    path.write_text("\n".join(_print_lines()) + "\n")
    world.print_file(path, timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=60)
    stdout = (world.log_dir / "klippy.stdout").read_text(errors="replace")
    events = world.events_text()
    for fatal in FATAL_EVENTS:
        assert fatal not in stdout and fatal not in events, stdout[-3000:]
    assert world.shutdown_line() is None, world.log_tail()[-3000:]
