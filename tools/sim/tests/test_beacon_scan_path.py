"""Reproduce the beacon rapid-scan mesh PATH in the planner without the
beacon stream in the loop.

The bench symptom (BED_MESH_CALIBRATE stutters the first few corners) could be
either the planner starving on the dense overscan-corner polyline, or the
beacon emulator's stream stalling. This test feeds the exact path beacon's
_generate_path emits (ported in tools/sim/gen_beacon_scan.py) as a plain SD
print on the Trident's motion limits, and counts anchor_underrun (planner fell
behind playback -> stutter) and arc_run_dissolved (fitter gave up an arc run).

Streamed via SD so klippy's lookahead fills the way it does on the bench;
sending moves one-at-a-time would never build the backlog the bug needs.
"""

import pathlib

import pytest

from tools.sim import configs
from tools.sim.gen_beacon_scan import auto_overscan, generate_path

pytestmark = [pytest.mark.needs_elf]


def _write_scan_gcode(
    path: pathlib.Path,
    *,
    direction,
    speed,
    runs,
    min_xy=(25.0, 25.0),
    max_xy=(275.0, 275.0),
    count=(20, 20),
    z=2.0,
):
    # Offsets omitted: the path SHAPE (corner radii, chord density, corner
    # speeds) is independent of the beacon XY offset, which only translates
    # the whole scan. Overscan matches beacon's _handle_connect: it clamps to
    # the room the ALIGNED axis has before the machine limit.
    min_x, min_y = min_xy
    max_x, max_y = max_xy
    res_x, res_y = count
    if direction == "x":
        # aligned = X, one pass per Y row (count = res_y)
        overscan = auto_overscan(min_x, max_x, res_y, 0.0, 300.0)
    else:
        # aligned = Y, one pass per X column (count = res_x)
        overscan = auto_overscan(min_y, max_y, res_x, 0.0, 300.0)
    pts = generate_path(
        min_x, min_y, max_x, max_y, res_x, res_y, direction, overscan, 0.0, 0.0
    )
    fr = speed * 60.0
    lines = [
        "G90",
        "SET_KINEMATIC_POSITION X=%.3f Y=%.3f Z=%.3f"
        % (pts[0][0], pts[0][1], z),
    ]
    for i in range(runs):
        seq = pts if i % 2 == 0 else list(reversed(pts))
        lines += ["G1 X%.4f Y%.4f F%.0f" % (x, y, fr) for (x, y) in seq]
    lines.append("M400")
    path.write_text("\n".join(lines) + "\n")
    return len(pts)


def _scan_metrics(world):
    ev = world.events_text()
    lowered = [
        int(r["n_pieces"])
        for r in _iter_json(ev)
        if r.get("event") == "pipe_lower" and "n_pieces" in r
    ]
    return {
        "underruns": ev.count('"anchor_underrun"'),
        "arc_run_dissolved": ev.count('"arc_run_dissolved"'),
        "lowered_segments": len(lowered),
        "lowered_pieces": sum(lowered),
    }


def _iter_json(text):
    import json

    for ln in text.splitlines():
        try:
            yield json.loads(ln)
        except ValueError:
            continue


def test_beacon_scan_path_full(sim_world):
    # pipe_lower is debug-level; raise the motion engine's filter so the
    # per-segment piece count (the arc-fit signal) is captured.
    import os

    os.environ["RUST_LOG"] = "debug"
    world = sim_world(
        lambda w: configs.corexy_fast_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    gpath = world.gcode_dir / "beacon_scan_full.gcode"
    n = _write_scan_gcode(gpath, direction="y", speed=800, runs=1)
    dur = world.print_file(gpath, timeout=300)
    m = _scan_metrics(world)
    print(
        f"\n[scan] {n} input moves, "
        f"print_duration={dur:.2f}s, underruns={m['underruns']}, "
        f"arc_run_dissolved={m['arc_run_dissolved']}, "
        f"lowered_segments={m['lowered_segments']}, "
        f"lowered_pieces={m['lowered_pieces']}"
    )
    # Startup-transient probe: window depth (n) and planned barrier velocity
    # per plan call, in order. If the early corners brake to rest because the
    # lookahead window is shallow, early v_barrier ~ 0 with small n, rising as
    # the buffer fills.
    plans = [
        r
        for r in _iter_json(world.events_text())
        if r.get("event") == "pipe_plan"
    ]
    print(f"  pipe_plan calls={len(plans)}")
    for tag, sl in (("first 14", plans[:14]), ("last 4", plans[-4:])):
        print(f"  -- {tag} --")
        for r in sl:
            print(
                f"     n={r.get('n'):>4} entry_v={float(r.get('entry_v', 0)):8.1f} "
                f"v_barrier={float(r.get('v_barrier', 0)):8.1f} "
                f"barrier={r.get('barrier')} "
                f"lines={r.get('line_lo')}-{r.get('line_hi')}"
            )
    assert world.shutdown_line() is None
