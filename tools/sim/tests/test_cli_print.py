"""tools/sim/cli.py prints real slicer output on real firmware to completion
in the free-running mode like the suite's prints, and fails loudly on a
print error. The measured speedup is printed, never asserted: a wall-clock
ratio depends on the load of the box."""

import os
import pathlib
import re
import subprocess
import sys

import pytest

from tools.sim.world import UNBOUNDED_VTIME_SPEED

pytestmark = pytest.mark.needs_elf

SIM_DIR = pathlib.Path(__file__).resolve().parents[1]
CLI = SIM_DIR / "cli.py"
VORON = SIM_DIR / "gcode" / "voron_first_layers.gcode"
VORON_HEAD_LINES = 600
CLI_TIMEOUT_S = 600


def _run_cli(gcode: pathlib.Path) -> subprocess.CompletedProcess:
    env = {k: v for k, v in os.environ.items() if k != "VTIME_SPEED"}
    return subprocess.run(
        [sys.executable, str(CLI), "--gcode", str(gcode)],
        capture_output=True,
        text=True,
        timeout=CLI_TIMEOUT_S,
        env=env,
    )


def test_cli_prints_slicer_output_to_completion_free_running(tmp_path):
    gcode = tmp_path / "voron_head.gcode"
    lines = VORON.read_text().splitlines(keepends=True)
    gcode.write_text("".join(lines[:VORON_HEAD_LINES]))

    result = _run_cli(gcode)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "FAIL" not in result.stdout, result.stdout
    print_time = re.search(r"print time:\s+([0-9.]+)s", result.stdout)
    assert print_time, result.stdout
    assert float(print_time.group(1)) > 0, result.stdout
    mode = re.search(r"vtime speed:\s+([0-9.]+)x", result.stdout)
    assert mode, result.stdout
    assert float(mode.group(1)) == UNBOUNDED_VTIME_SPEED, result.stdout
    print(result.stdout)


def test_cli_fails_on_a_print_error(tmp_path):
    gcode = tmp_path / "out_of_range.gcode"
    gcode.write_text("G1 X9999 F3000\n")

    result = _run_cli(gcode)

    assert result.returncode != 0, result.stdout
    assert "FAIL" in result.stdout, result.stdout
