"""The live host and the snapshot path must plan every corpus G-code bit for
bit alike: the same pipeline inputs (moves with their full limits, rests,
dwells, the starting rest) and the same shaped segments. The live side runs
klippy's G-code handlers into the real ``MotionEngine`` bridge with a
detached planner (``live_bridge_parity``); nothing here records or replays
the host.

The corpus is ``scripts/corpus.sh``'s: every snapshot case and the sim prints
on both bench configs, plus the dense print on every bench config and
profile. Run it with ``scripts/ci.sh live-parity``.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import harness  # noqa: E402
import live_bridge_parity  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
CASES_DIR = REPO / "snapshots" / "cases"
SIM_GCODE = REPO / "tools" / "sim" / "gcode"
SIM_PRINTS = (
    "voron_first_layers",
    "voron_dense_top_layers",
    "trident_follower_cache_regression",
    "trident_mesh_clock_regression",
)
BENCH_CONFIGS = REPO / "tools" / "bench" / "configs"
SIM_CONFIGS = (
    BENCH_CONFIGS / "fast.cfg",
    REPO / "tools" / "bench" / "printer.cfg",
)
DENSE_PRINT = SIM_GCODE / "voron_dense_top_layers.gcode"
FIRST_MOTION = re.compile(r"^\s*(G0|G1|SET_KINEMATIC_POSITION)\b", re.I | re.M)


def _snapshot_cases() -> list[tuple[str, Path, Path]]:
    cases = []
    for group in sorted(p for p in CASES_DIR.iterdir() if p.is_dir()):
        for cfg in sorted(group.glob("*.cfg")):
            for gcode in sorted(group.glob("*.gcode")):
                if gcode.read_text().strip():
                    cases.append(
                        (f"{group.name}/{cfg.stem}/{gcode.stem}", gcode, cfg)
                    )
    return cases


def corpus() -> list[tuple[str, Path, Path]]:
    sim = [
        (f"{name}/{cfg.stem}", SIM_GCODE / f"{name}.gcode", cfg)
        for name in SIM_PRINTS
        for cfg in SIM_CONFIGS
    ]
    dense = [
        (
            f"dense/{cfg.relative_to(BENCH_CONFIGS).with_suffix('')}",
            DENSE_PRINT,
            cfg,
        )
        for cfg in sorted(BENCH_CONFIGS.rglob("*.cfg"))
        if cfg.name not in {c.name for c in SIM_CONFIGS}
    ]
    return _snapshot_cases() + sim + dense


def with_start_position(gcode: str) -> str:
    """A print that never states where the toolhead is starts at the origin
    on both sides: the snapshot would otherwise anchor at its first move."""
    first = FIRST_MOTION.search(gcode)
    if first and first.group(1).upper() == "SET_KINEMATIC_POSITION":
        return gcode
    return "SET_KINEMATIC_POSITION X=0 Y=0 Z=0\n" + gcode


@pytest.fixture(scope="module")
def engine():
    return harness._import_engine()


@pytest.mark.parametrize(
    ("gcode_path", "cfg"),
    [pytest.param(gcode, cfg, id=name) for name, gcode, cfg in corpus()],
)
def test_the_live_bridge_plans_what_the_snapshot_plans(engine, gcode_path, cfg):
    divergence = live_bridge_parity.compare(
        with_start_position(gcode_path.read_text()), cfg
    )
    assert divergence is None, f"{gcode_path.name} on {cfg}: {divergence}"


def test_a_limit_the_two_sides_disagree_on_names_the_move_and_field(
    engine, tmp_path
):
    gcode = with_start_position(
        "G92 E0\nM83\nG1 X10 Y10 F6000\nG1 X50 Y10 E1\nG1 X50 Y50 E1\n"
    )
    cfg = REPO / "tools" / "bench" / "printer.cfg"
    live, config_text = live_bridge_parity.live_record(gcode, cfg)
    slower = re.sub(
        r"^max_accel\s*[:=].*$",
        "max_accel = 5000",
        config_text,
        count=1,
        flags=re.M,
    )
    divergence = live_bridge_parity.first_divergence(
        live, live_bridge_parity.snapshot_record(gcode, slower)
    )
    assert divergence is not None
    assert divergence.kind == "input"
    assert divergence.path.endswith("limits.accel_mm_s2")
    assert divergence.seq == 0
    assert (divergence.live, divergence.snapshot) == (
        "accel_mm_s2: 10000.0,",
        "accel_mm_s2: 5000.0,",
    )
