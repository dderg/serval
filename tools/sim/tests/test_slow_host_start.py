"""#983, Pi bench 2026-11-17 (b122baafe, printer.cfg): a 2.3 s lead-in at
100 mm/s, then 120 s of extruding square laps at 1-10 mm/s, aborted klippy
within 3 s of the print start (`anchor_underrun`, `anchor_low_margin`). The
stream set off on the lowerer's 8.4 ms rest hold while the planning front
still held the lead-in behind it, until it had planned much of the slow
section: 0.3-0.8 s on the Pi, past the lead.

The simulator's clock stands still while host threads compute, so planning
never costs printer time here unless SERVAL_HOST_COMPUTE_SCALE makes the
pipeline's threads wait out their CPU time, scaled to a Pi."""

import json

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

PI_COMPUTE_SCALE = 2.0
CRAWL_S = 120.0
PIECE_S = 2.0
PRINT_FEED = 6000
TRAVEL_FEED = 12000
E_PER_MM = 0.0374
STROKE_MM = 100.0
STROKE_GAP_MM = 10.0
START_XY = (40.0, 50.0)
SQUARE_HEADINGS = ((1, 0), (0, 1), (-1, 0), (0, -1))
PRINT_TIMEOUT_S = 600.0
CRAWL_EXPECTED_S = 60
FATAL_EVENTS = (
    "anchor_underrun",
    "anchor_low_margin",
    "anchor_hole_mid_motion",
    "stream_worker_fatal",
)


def _fmt(value: float) -> str:
    return f"{value:.4f}".rstrip("0").rstrip(".")


def _extrude(x: float, y: float, mm: float, feed: float) -> str:
    return f"G1 X{_fmt(x)} Y{_fmt(y)} E{_fmt(mm * E_PER_MM)} F{feed}"


def _strokes(x0: float, y0: float) -> tuple[list[str], tuple[float, float]]:
    x1 = x0 + STROKE_MM
    y1 = y0 + STROKE_GAP_MM
    lines = [
        _extrude(x1, y0, STROKE_MM, PRINT_FEED),
        _extrude(x1, y1, STROKE_GAP_MM, PRINT_FEED),
        _extrude(x0, y1, STROKE_MM, PRINT_FEED),
    ]
    return lines, (x0, y1)


def _crawl_print(factor_pct: int, crawl_feed: int, sides: int) -> list[str]:
    """The bench's crawl rows (`crawl_gcode.py`): two strokes at print speed,
    `M220 S<factor_pct>` and CRAWL_S of extruding square laps in PIECE_S
    pieces that never come to rest, `M220 S100` and two more strokes."""
    speed_mm_s = crawl_feed / 60.0 * factor_pct / 100.0
    side_mm = speed_mm_s * CRAWL_S / sides
    pieces = round(side_mm / (speed_mm_s * PIECE_S))
    piece_mm = side_mm / pieces
    x, y = START_XY
    lines = ["G90", "M83", "G1 Z0.2 F600", f"G1 X{_fmt(x)} Y{_fmt(y)} F{TRAVEL_FEED}"]
    lead_in, (x, y) = _strokes(x, y)
    lines += lead_in + [f"M220 S{factor_pct}"]
    for side in range(sides):
        dx, dy = SQUARE_HEADINGS[side % len(SQUARE_HEADINGS)]
        for _ in range(pieces):
            x += dx * piece_mm
            y += dy * piece_mm
            lines.append(_extrude(x, y, piece_mm, crawl_feed))
    lines += ["M220 S100", f"G1 X{_fmt(START_XY[0])} Y{_fmt(y + STROKE_GAP_MM)} F{TRAVEL_FEED}"]
    lead_out, _ = _strokes(START_XY[0], y + STROKE_GAP_MM)
    return lines + lead_out + ["G1 E-0.5 F2400", "G1 Z5 F600", "M400"]


def _events(world) -> list[dict]:
    events = []
    for line in world.events_text().splitlines():
        try:
            events.append(json.loads(line))
        except ValueError:
            continue
    return events


@pytest.mark.expected_seconds(CRAWL_EXPECTED_S)
@pytest.mark.parametrize(
    ("factor_pct", "crawl_feed", "sides"),
    [
        pytest.param(1, 6000, 6, id="1mm_s"),
        pytest.param(5, 3000, 6, id="2.5mm_s"),
        pytest.param(5, 6000, 6, id="5mm_s"),
        pytest.param(10, 6000, 6, id="10mm_s"),
        pytest.param(20, 6000, 12, id="20mm_s"),
    ],
)
def test_a_slow_section_after_a_short_lead_in_starts_on_a_pi(
    sim_world, monkeypatch, factor_pct, crawl_feed, sides
):
    monkeypatch.setenv("SERVAL_HOST_COMPUTE_SCALE", str(PI_COMPUTE_SCALE))
    world = sim_world(
        lambda w: configs.bench_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    gcode = world.gcode_dir / "crawl.gcode"
    gcode.write_text("\n".join(_crawl_print(factor_pct, crawl_feed, sides)) + "\n")

    world.print_file(gcode, timeout=PRINT_TIMEOUT_S)

    names = [event.get("event") for event in _events(world)]
    assert "host_compute_charged" in names, "the pipeline ran uncharged"
    fatal = [name for name in FATAL_EVENTS if name in names]
    assert not fatal, world.log_tail()[-3000:]
    assert world.shutdown_line() is None, world.log_tail()[-3000:]
