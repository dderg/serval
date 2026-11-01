"""The engine's live clock record must track each MCU's own crystal.

Both MCUs run off-nominal crystals (-20 / +30 ppm). Every clock-sync
update the engine adopts (set_clock_est_rebased) is compared against the
MCU's true counter: rate as a ppm error against the crystal, offset as the
time error of the record's anchor point. Once the first samples are in,
the rate must sit inside the clock-sync stability gate and the anchor
within 50 us, the scale of a real USB link's half round trip.
"""

import json
import statistics

import pytest

from tools.sim.tests.test_clock_resync import _dual_mcu_cartesian

pytestmark = pytest.mark.needs_elf

CRYSTAL_PPM = {"h7": -20.0, "f4": 30.0}
NOMINAL_HZ = 50e6
MOVES = 40
SETTLE_S = 10.0
CLOCK_SYNC_STABLE_PPM = 5.0
OFFSET_BOUND_S = 50e-6


def _records(world):
    by_mcu = {}
    for line in world.events_text().splitlines():
        event = json.loads(line)
        if event.get("event") == "set_clock_est_rebased":
            by_mcu.setdefault(event["mcu"], []).append(
                (event["offset_raw"], event["freq"], event["last_clock"])
            )
    return by_mcu


def test_live_clock_record_tracks_each_crystal(sim_world):
    world = sim_world(_dual_mcu_cartesian, crystal_ppm=CRYSTAL_PPM)
    world.gcode_ok("SET_KINEMATIC_POSITION X=50 Y=50 Z=50", timeout=10)
    for move in range(MOVES):
        target = 150 if move % 2 == 0 else 50
        world.gcode_ok(f"G1 X{target} Y{target} F6000", timeout=30)
    world.gcode_ok("M400", timeout=120)

    by_mcu = _records(world)
    assert len(by_mcu) == 2, by_mcu.keys()
    for mcu, records in by_mcu.items():
        late_freq = statistics.median(freq for _, freq, _ in records[-10:])
        name, ppm = min(
            CRYSTAL_PPM.items(),
            key=lambda item: abs(NOMINAL_HZ * (1 + item[1] * 1e-6) - late_freq),
        )
        scale = 1 + ppm * 1e-6
        true_hz = NOMINAL_HZ * scale
        start_sec = statistics.mode(
            round(at * scale - clock / NOMINAL_HZ) for at, _, clock in records
        )
        first_at = records[0][0]
        for at, freq, clock in records:
            if at - first_at < SETTLE_S:
                continue
            rate_ppm = (freq / true_hz - 1) * 1e6
            offset_s = (clock - (at * scale - start_sec) * NOMINAL_HZ) / true_hz
            assert abs(rate_ppm) < CLOCK_SYNC_STABLE_PPM, (
                f"{name} ({mcu}) record {at - first_at:.1f} s in is"
                f" {rate_ppm:+.2f} ppm off its crystal"
            )
            assert abs(offset_s) < OFFSET_BOUND_S, (
                f"{name} ({mcu}) record {at - first_at:.1f} s in anchors"
                f" {offset_s * 1e6:+.1f} us off the true counter"
            )
