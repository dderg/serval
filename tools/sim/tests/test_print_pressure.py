"""Producer-throughput gate on real firmware: the dense top layers of the
Voron cube that repeatedly exhausted the anchor lead on the Trident bench.

Virtual time runs free at PRESSURE x the wall clock instead of waiting for
klippy (host_gated=False), so every host stage must produce that multiple of
realtime — the way the bench Pi must produce 1x with a fraction of a dev
machine's single-core speed. A producer deficit surfaces here as an anchor
underrun instead of minutes into a physical print. 1.5x is the CI default;
raise it locally via SIM_PRESSURE_VTIME for a stricter pre-deploy check."""

import os
import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

GCODE = (
    pathlib.Path(__file__).resolve().parents[1]
    / "gcode"
    / "voron_dense_top_layers.gcode"
)

PRESSURE = float(os.environ.get("SIM_PRESSURE_VTIME", "1.5"))


def test_dense_top_layers_survive_pi_pressure(sim_world):
    world = sim_world(
        lambda w: configs.neptune_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
        vtime_speed=PRESSURE,
        host_gated=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    print_time = world.print_file(GCODE, timeout=600)
    assert print_time > 0

    events = world.events_text()
    for fatal in (
        "anchor_underrun",
        "anchor_low_margin",
        "stream_worker_fatal",
        "pump_piece_in_past",
        "runtime_fault",
        "diag.rust_fault",
    ):
        assert fatal not in events, (
            f"{fatal} during pressured print:\n{world.log_tail()[-3000:]}"
        )
    assert world.shutdown_line() is None
