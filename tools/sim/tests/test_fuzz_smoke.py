"""Fixed seeds of the scenario fuzzer (tools/sim/fuzz.py), each a printer,
a print program and a timeline of user actions, held to every invariant
in tools/sim/fuzzing/checks.py. Fresh seeds run in `ci.sh sim-fuzz`."""

import pytest

from tools.sim.fuzzing import execute, scenario

pytestmark = pytest.mark.needs_elf

SMOKE_SEEDS = {}


@pytest.mark.parametrize(
    "seed", sorted(SMOKE_SEEDS), ids=lambda s: f"{s}-{SMOKE_SEEDS[s]}"
)
def test_fuzz_seed(seed, tmp_path):
    chosen = scenario.generate(seed)
    result = execute.run_scenario(
        chosen, tmp_path / "world", tmp_path / "failure"
    )
    assert result["status"] == execute.PASS, (
        f"{result['signature']}\n{result['evidence']}\n"
        f"replay: tools/sim/run.sh fuzz --seed {seed}"
    )
