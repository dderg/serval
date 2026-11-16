from __future__ import annotations

import run
import soak_gcode


def test_the_soak_print_only_climbs_one_layer_at_a_time():
    heights = soak_gcode.z_values(soak_gcode.soak_lines())
    steps = [later - earlier for earlier, later in zip(heights, heights[1:])]
    assert heights[0] == 0.2
    assert min(steps) >= 0.0
    assert max(steps) <= soak_gcode.LAYER_HEIGHT_MM + 1e-6


def test_the_soak_print_starts_with_the_first_layers_whole():
    first = soak_gcode.FIRST_LAYERS.read_text().splitlines()
    lines = soak_gcode.soak_lines()
    assert lines[: len(first)] == first
    upper = soak_gcode.upper_layers(first)
    dense = soak_gcode.DENSE_TOP.read_text().splitlines()
    block = len(upper) + len(dense)
    assert len(lines) == len(first) + soak_gcode.BODY_REPEATS * block


def test_the_soak_print_is_one_the_bench_runs():
    _, stats = run.sanitize("\n".join(soak_gcode.soak_lines()))
    assert stats["kept"] > 0


def test_a_shift_moves_z_words_and_leaves_comments():
    assert (
        soak_gcode.shifted("G1 X1 Z.4 F9000 ; Z.4 kept", 1.0)
        == "G1 X1 Z1.400 F9000 ; Z.4 kept"
    )
    assert soak_gcode.shifted("G1 X12 E.5", 1.0) == "G1 X12 E.5"


def test_ensure_writes_the_print_once_and_repairs_it(tmp_path, monkeypatch):
    path = tmp_path / "generated" / "soak.gcode"
    monkeypatch.setattr(soak_gcode, "PATH", path)
    assert soak_gcode.ensure() == path
    written = path.stat().st_mtime_ns
    soak_gcode.ensure()
    assert path.stat().st_mtime_ns == written
    path.write_text("G1 X0\n")
    soak_gcode.ensure()
    assert path.read_text().splitlines() == soak_gcode.soak_lines()
