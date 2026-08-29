"""Interactive jogs must always use the fixed startup lead."""

from __future__ import annotations

import json
import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def test_separate_jogs_do_not_accumulate_startup_delay(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=20")
    world.gcode_ok("G91")
    for _ in range(5):
        world.gcode_ok("G1 X1 F1200")
        time.sleep(0.8)

    decisions = []
    for line in world.events_text().splitlines():
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if event.get("event") == "anchor_decision":
            decisions.append(event)

    assert len(decisions) >= 5, world.events_text()[-4000:]
    leads = [event["lead_secs"] for event in decisions[-5:]]
    assert leads == pytest.approx([0.25] * 5, abs=1e-6)
    assert world.shutdown_line() is None, world.log_tail()
