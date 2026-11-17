"""Prints over a loaded bed mesh that an offline preflight sweep found
panicking the planner (#881, #883, #884), run through real klippy.

- #881: the first move of the print names Z right after
  `BED_MESH_PROFILE LOAD`, from every way a print finds its start.
- #883: a move crosses the mesh's edge at speed, where the surface turns
  flat and the warped Z steps in velocity.
- #884: the p3 Voron profile (shaped Z, square_corner_velocity) prints a
  square over the steep mesh's edge.
"""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

PRINT_TIMEOUT_S = 120
FATAL_EVENTS = ("panicked", "stream_worker_fatal", "runtime_fault")

STEEP_PROFILE = """\
#*#
#*# [bed_mesh steep]
#*# version = 1
#*# points =
#*#   0.500000, -0.500000, 0.500000
#*#   -0.500000, 0.500000, -0.500000
#*#   0.500000, -0.500000, 0.500000
#*# min_x = 10.0
#*# max_x = 290.0
#*# min_y = 10.0
#*# max_y = 290.0
#*# x_count = 3
#*# y_count = 3
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = lagrange
#*# tension = 0.2
"""

P3_PROFILE = (
    (
        "max_velocity: 1000\nmax_accel: 10000\n",
        "max_velocity: 500\nmax_accel: 15000\n",
    ),
    ("corner_deviation: 0.21\n", "square_corner_velocity: 5\n"),
    ("max_z_accel: 100\n", "max_z_accel: 350\n"),
    (
        "endstop_pin: ^gpiochip0/gpio12\nhoming_speed: 5\n",
        "endstop_pin: ^gpiochip0/gpio12\nhoming_speed: 5\npost_processors: shaper_z\n",
    ),
    (
        "type: smooth_mzv\nfrequency_hz: 50\n",
        "type: smooth_mzv\nfrequency_hz: 75.0\n",
    ),
    (
        "type: smooth_zv\nfrequency_hz: 44\n",
        "type: smooth_mzv\nfrequency_hz: 62.0\n\n"
        "[post_processor shaper_z]\ntype: smooth_mzv\nfrequency_hz: 100.0\n",
    ),
)

Z_FIRST_PRINT = "G90\nG1 X40 Y50 Z10 F6000\nG1 X100 Y100 F6000\n"
SQUARE_PRINT = (
    "SET_VELOCITY_LIMIT ACCEL=1000\nG1 X20 Y0 F12000\nG1 X20 Y20\nG1 X0 Y20\n"
    "G1 X0 Y0\nSET_VELOCITY_LIMIT ACCEL=25000\nG1 X20 Y0\n"
)


def _bench_with_steep(world):
    config = configs.bench_mesh_print_config(world.h7_pty, str(world.gcode_dir))
    return config + STEEP_PROFILE


def _p3_with_steep(world):
    config = _bench_with_steep(world)
    for before, after in P3_PROFILE:
        assert before in config, before
        config = config.replace(before, after)
    return config


def _bed_mesh(world):
    return configs.bed_mesh_config(world.h7_pty, str(world.gcode_dir))


def _print(world, name, text):
    path = world.gcode_dir / name
    path.write_text(text)
    world.print_file(path, timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=60)


def _assert_alive(world):
    stdout = (world.log_dir / "klippy.stdout").read_text(errors="replace")
    events = world.events_text()
    for fatal in FATAL_EVENTS:
        assert fatal not in stdout and fatal not in events, stdout[-3000:]
    assert world.shutdown_line() is None, world.log_tail()[-3000:]


@pytest.mark.parametrize("start", ("kinematic_position", "g92_onto_first_move"))
def test_first_move_naming_z_after_a_mesh_load(sim_world, start):
    world = sim_world(_bench_with_steep, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0")
    world.gcode_ok("BED_MESH_PROFILE LOAD=wavy")
    if start == "g92_onto_first_move":
        world.gcode_ok("G92 X40 Y50 Z10")
    _print(world, "z_first.gcode", Z_FIRST_PRINT)
    _assert_alive(world)


def test_first_move_naming_z_after_homing_and_a_mesh_load(sim_world):
    world = sim_world(_bed_mesh, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=20 Y=20 Z=15")
    world.gcode_ok("G28 Z", timeout=120)
    world.gcode_ok("BED_MESH_PROFILE LOAD=wavy")
    _print(
        world, "z_first.gcode", "G90\nG1 X40 Y50 Z10 F3000\nG1 X60 Y60 F3000\n"
    )
    _assert_alive(world)


@pytest.mark.parametrize(
    "mesh,start,end",
    (("wavy", (20, 100), (120, 100)), ("steep", (150, 150), (300, 150))),
)
def test_a_move_crossing_the_mesh_edge_at_speed(sim_world, mesh, start, end):
    world = sim_world(_bench_with_steep, dual_mcu=False)
    world.gcode_ok(f"SET_KINEMATIC_POSITION X={start[0]} Y={start[1]} Z=0.2")
    world.gcode_ok(f"BED_MESH_PROFILE LOAD={mesh}")
    _print(world, "edge.gcode", f"G1 X{end[0]} Y{end[1]} F60000\n")
    _assert_alive(world)


def test_p3_square_over_the_steep_mesh_edge(sim_world):
    world = sim_world(_p3_with_steep, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0.2")
    world.gcode_ok("BED_MESH_PROFILE LOAD=steep")
    _print(world, "square.gcode", SQUARE_PRINT)
    _assert_alive(world)
