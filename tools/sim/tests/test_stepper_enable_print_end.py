"""SET_STEPPER_ENABLE ENABLE=0 in a PRINT_END-style sequence on real firmware.

Field report: a PRINT_END macro that disables individual motors via
SET_STEPPER_ENABLE (instead of M84) crashed prints before completion.
These tests replay that pattern — per-motor disables issued straight after
queued motion with no M400 — and a mid-print disable followed by more motion
(the re-energize path).
"""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

FATAL_EVENTS = ("pump_piece_in_past", "runtime_fault", "diag.rust_fault")


def _zigzag(n, x0=100.0, y0=100.0, step=2.0, feed=9000, extrude=0.05):
    lines = []
    for i in range(n):
        x = x0 + (i % 2) * step
        y = y0 + (i // 2) * 0.4
        e = f" E{extrude}" if extrude else ""
        lines.append(f"G1 X{x:.3f} Y{y:.3f}{e} F{feed}")
    return lines


def _boot(sim_world):
    world = sim_world(
        lambda w: configs.neptune_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    return world


def _assert_clean(world, print_time):
    assert print_time > 0
    events = world.events_text()
    for fatal in FATAL_EVENTS:
        assert fatal not in events, (
            f"{fatal} during print:\n{world.log_tail()[-3000:]}"
        )
    assert world.shutdown_line() is None, world.log_tail()[-3000:]


def test_print_end_per_motor_disable(sim_world):
    world = _boot(sim_world)
    gcode = world.gcode_dir / "print_end_disable.gcode"
    lines = [
        "G90",
        "G21",
        "M83",
        "G92 E0",
        "G1 Z0.2 F600",
        *_zigzag(120),
        "; --- PRINT_END, user style: no M400 before per-motor disable ---",
        "G1 E-2 F1800",
        "G91",
        "G1 Z5 F600",
        "G90",
        "G1 X10 Y200 F6000",
        "SET_STEPPER_ENABLE STEPPER=x ENABLE=0",
        "SET_STEPPER_ENABLE STEPPER=y ENABLE=0",
        "SET_STEPPER_ENABLE STEPPER=z ENABLE=0",
        "SET_STEPPER_ENABLE STEPPER=extruder ENABLE=0",
        "M106 S0",
    ]
    gcode.write_text("\n".join(lines) + "\n")
    print_time = world.print_file(gcode, timeout=600)
    _assert_clean(world, print_time)


def test_disable_mid_print_then_more_motion(sim_world):
    world = _boot(sim_world)
    gcode = world.gcode_dir / "disable_mid_print.gcode"
    lines = [
        "G90",
        "G21",
        "M83",
        "G92 E0",
        "G1 Z0.2 F600",
        *_zigzag(80),
        "SET_STEPPER_ENABLE STEPPER=extruder ENABLE=0",
        *_zigzag(80, y0=140.0),
        "SET_STEPPER_ENABLE STEPPER=x ENABLE=0",
        *_zigzag(40, y0=180.0),
        "M84",
    ]
    gcode.write_text("\n".join(lines) + "\n")
    print_time = world.print_file(gcode, timeout=600)
    _assert_clean(world, print_time)


def test_print_end_disable_corexy_tmc(sim_world):
    world = sim_world(
        lambda w: configs.awd_corexy_tmc_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    gcode = world.gcode_dir / "print_end_disable_corexy.gcode"
    lines = [
        "G90",
        "G21",
        "G1 Z0.2 F600",
        *_zigzag(80, feed=4000, extrude=0.0),
        "SET_STEPPER_ENABLE STEPPER=a ENABLE=0",
        *_zigzag(40, y0=140.0, feed=4000, extrude=0.0),
        "; --- PRINT_END, user style: per-motor disable, no M400 ---",
        "G91",
        "G1 Z5 F600",
        "G90",
        "G1 X10 Y200 F4000",
        "SET_STEPPER_ENABLE STEPPER=a ENABLE=0",
        "SET_STEPPER_ENABLE STEPPER=a1 ENABLE=0",
        "SET_STEPPER_ENABLE STEPPER=b ENABLE=0",
        "SET_STEPPER_ENABLE STEPPER=b1 ENABLE=0",
        "SET_STEPPER_ENABLE STEPPER=z ENABLE=0",
    ]
    gcode.write_text("\n".join(lines) + "\n")
    print_time = world.print_file(gcode, timeout=600)
    _assert_clean(world, print_time)


def test_print_end_disable_stepcompress_secondary_mcu(sim_world):
    from tools.sim.tests.test_stepcompress_corexy import XyTracker

    world = sim_world(
        lambda w: configs.stepcompress_corexy_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=60 Y=60 Z=20", timeout=60)
    world.gcode_ok("M400", timeout=60)
    tracker = XyTracker(world.sim_control("f4"), (60.0, 60.0))
    gcode = world.gcode_dir / "print_end_disable_sc.gcode"
    lines = [
        "G90",
        "G21",
        "G1 Z20.2 F600",
        *_zigzag(80, x0=60.0, y0=60.0, extrude=0.0),
        "SET_STEPPER_ENABLE STEPPER=a ENABLE=0",
        *_zigzag(40, x0=60.0, y0=95.0, extrude=0.0),
        "; --- PRINT_END, user style: per-motor disable, no M400 ---",
        "G91",
        "G1 Z5 F600",
        "G90",
        "G1 X10 Y110 F9000",
        "SET_STEPPER_ENABLE STEPPER=a ENABLE=0",
        "SET_STEPPER_ENABLE STEPPER=b ENABLE=0",
        "SET_STEPPER_ENABLE STEPPER=z ENABLE=0",
    ]
    gcode.write_text("\n".join(lines) + "\n")
    print_time = world.print_file(gcode, timeout=600)
    world.gcode_ok("M400", timeout=60)
    _assert_clean(world, print_time)
    x, y = tracker.xy()
    assert abs(x - 10.0) < 0.1 and abs(y - 110.0) < 0.1, (
        f"executed motor tracks ended at ({x:.3f}, {y:.3f}), commanded park"
        " was (10, 110) — SET_STEPPER_ENABLE truncated queued motion"
    )


def _sc_world(sim_world):
    world = sim_world(
        lambda w: configs.stepcompress_corexy_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=60 Y=60 Z=20", timeout=60)
    world.gcode_ok("M400", timeout=60)
    return world


def _sc_print(world, name, tail):
    gcode = world.gcode_dir / name
    lines = [
        "G90",
        "G21",
        "G1 Z20.2 F600",
        *_zigzag(80, x0=60.0, y0=60.0, extrude=0.0),
        "G91",
        "G1 Z5 F600",
        "G90",
        "G1 X10 Y110 F9000",
        *tail,
    ]
    gcode.write_text("\n".join(lines) + "\n")
    print_time = world.print_file(gcode, timeout=600)
    world.gcode_ok("M400", timeout=60)
    _assert_clean(world, print_time)


def test_stepcompress_m84_control(sim_world):
    _sc_print(_sc_world(sim_world), "sc_m84.gcode", ["M84"])


def test_stepcompress_three_end_disables(sim_world):
    _sc_print(
        _sc_world(sim_world),
        "sc_three_end.gcode",
        [
            "SET_STEPPER_ENABLE STEPPER=a ENABLE=0",
            "SET_STEPPER_ENABLE STEPPER=b ENABLE=0",
            "SET_STEPPER_ENABLE STEPPER=z ENABLE=0",
        ],
    )


def test_stepcompress_midprint_disable_then_m84(sim_world):
    world = _sc_world(sim_world)
    gcode = world.gcode_dir / "sc_midprint.gcode"
    lines = [
        "G90",
        "G21",
        "G1 Z20.2 F600",
        *_zigzag(80, x0=60.0, y0=60.0, extrude=0.0),
        "SET_STEPPER_ENABLE STEPPER=a ENABLE=0",
        *_zigzag(40, x0=60.0, y0=95.0, extrude=0.0),
        "G91",
        "G1 Z5 F600",
        "G90",
        "G1 X10 Y110 F9000",
        "M84",
    ]
    gcode.write_text("\n".join(lines) + "\n")
    print_time = world.print_file(gcode, timeout=600)
    world.gcode_ok("M400", timeout=60)
    _assert_clean(world, print_time)


def test_stepcompress_g4_mid_print(sim_world):
    world = _sc_world(sim_world)
    gcode = world.gcode_dir / "sc_g4.gcode"
    lines = [
        "G90",
        "G21",
        "G1 Z20.2 F600",
        *_zigzag(40, x0=60.0, y0=60.0, extrude=0.0),
        "G4 P500",
        *_zigzag(40, x0=60.0, y0=80.0, extrude=0.0),
        "G4 P200",
        *_zigzag(20, x0=60.0, y0=100.0, extrude=0.0),
        "M84",
    ]
    gcode.write_text("\n".join(lines) + "\n")
    print_time = world.print_file(gcode, timeout=600)
    world.gcode_ok("M400", timeout=60)
    _assert_clean(world, print_time)


PRINT_END_MACRO = """
[gcode_macro PRINT_END]
gcode:
    G92 E0
    G1 E-2 F1800
    G91
    G1 Z5 F600
    G90
    G1 X10 Y200 F6000
    M107
    SET_STEPPER_ENABLE STEPPER=x ENABLE=0
    SET_STEPPER_ENABLE STEPPER=y ENABLE=0
    SET_STEPPER_ENABLE STEPPER=z ENABLE=0
    SET_STEPPER_ENABLE STEPPER=extruder ENABLE=0
"""


def test_print_end_macro_per_motor_disable_native_runtime(sim_world):
    """The user-reported shape verbatim: a PRINT_END gcode_macro that parks
    and disables motors individually instead of M84, invoked from the file,
    on the native motion runtime (no stepcompress)."""
    world = sim_world(
        lambda w: configs.neptune_print_config(w.h7_pty, str(w.gcode_dir))
        + PRINT_END_MACRO,
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    gcode = world.gcode_dir / "print_end_macro.gcode"
    lines = [
        "G90",
        "G21",
        "M83",
        "G92 E0",
        "G1 Z0.2 F600",
        *_zigzag(120),
        "PRINT_END",
    ]
    gcode.write_text("\n".join(lines) + "\n")
    print_time = world.print_file(gcode, timeout=600)
    _assert_clean(world, print_time)
