"""Fixed seeds of the scenario fuzzer (tools/sim/fuzz.py), each a printer,
a print program and a timeline of user actions, held to every invariant
in tools/sim/fuzzing/checks.py. Fresh seeds run in `ci.sh sim-fuzz`.

Both MCU topologies, flat/wavy/steep meshes (with the seam moves
fuzzing/mesh_paths.py draws across them), PAUSE/RESUME, CANCEL_PRINT,
RESTART, e-stop + FIRMWARE_RESTART, link faults, SET_PRESSURE_ADVANCE and
the refused SET_INPUT_SHAPER. Left out while their bugs are open: a
FIRMWARE_RESTART landing in an M400 (#892) and a host RESTART with heaters
on (#939; seed 18 hit it)."""

import pytest

from tools.sim.fuzzing import execute, scenario

pytestmark = pytest.mark.needs_elf

THREE_PRINTS_SEED = 27
THREE_PRINTS_EXPECTED_S = 100
SMOKE_SEEDS = (
    10,
    11,
    20,
    pytest.param(
        THREE_PRINTS_SEED,
        marks=pytest.mark.expected_seconds(THREE_PRINTS_EXPECTED_S),
    ),
    35,
    41,
    63,
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
