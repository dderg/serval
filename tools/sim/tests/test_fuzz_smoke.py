"""Fixed seeds of the scenario fuzzer (tools/sim/fuzz.py), each a printer,
a print program and a timeline of user actions, held to every invariant
in tools/sim/fuzzing/checks.py. Fresh seeds run in `ci.sh sim-fuzz`.

Both MCU topologies, flat/wavy/steep meshes (with the seam moves
fuzzing/mesh_paths.py draws across them), PAUSE/RESUME, CANCEL_PRINT,
RESTART, e-stop + FIRMWARE_RESTART, link faults, SET_PRESSURE_ADVANCE and
the refused SET_INPUT_SHAPER; the first-print extras: heat-up waits on
the hotend model, object labels and EXCLUDE_OBJECT, a filament runout
(12029), M220 (12083), M221, SET_VELOCITY_LIMIT, idle gaps and
SDCARD_RESET_FILE. Left out while their bugs are open: a heater command
after a long idle gap before a later print, with an idle timeout (#993)."""

import pytest

from tools.sim.fuzzing import execute, scenario

pytestmark = pytest.mark.needs_elf

LONG_SEED_EXPECTED_S = 150
SMOKE_SEEDS = (
    10,
    11,
    35,
    63,
    12083,
    *(
        pytest.param(
            seed, marks=pytest.mark.expected_seconds(LONG_SEED_EXPECTED_S)
        )
        for seed in (20, 27, 41, 12029)
    ),
)


@pytest.mark.parametrize("seed", SMOKE_SEEDS)
def test_fuzz_seed(seed, tmp_path):
    chosen = scenario.generate(seed)
    result = execute.run_scenario(
        chosen, tmp_path / "world", tmp_path / "failure"
    )
    assert result["status"] == execute.PASS, (
        f"{scenario.describe(chosen)}\n{result['signature']}\n"
        f"{result['evidence']}\nreplay: tools/sim/run.sh fuzz --seed {seed}"
    )
