"""tools/sim/cli.py prints real slicer output on real firmware faster than
real time, like the suite's prints, and fails loudly on a print error."""

import pathlib
import re
import subprocess
import sys

import pytest

pytestmark = pytest.mark.needs_elf

SIM_DIR = pathlib.Path(__file__).resolve().parents[1]
CLI = SIM_DIR / "cli.py"
VORON = SIM_DIR / "gcode" / "voron_first_layers.gcode"
VORON_HEAD_LINES = 600
CLI_TIMEOUT_S = 600


def _run_cli(gcode: pathlib.Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(CLI), "--gcode", str(gcode)],
        capture_output=True,
        text=True,
        timeout=CLI_TIMEOUT_S,
    )


def test_cli_prints_slicer_output_faster_than_real_time(tmp_path):
    gcode = tmp_path / "voron_head.gcode"
    lines = VORON.read_text().splitlines(keepends=True)
    gcode.write_text("".join(lines[:VORON_HEAD_LINES]))

    result = _run_cli(gcode)

    assert result.returncode == 0, result.stdout + result.stderr
    speedup = re.search(r"speedup:\s+([0-9.]+)x", result.stdout)
    assert speedup, result.stdout
    assert float(speedup.group(1)) > 1.0, result.stdout


def test_cli_fails_on_a_print_error(tmp_path):
    gcode = tmp_path / "out_of_range.gcode"
    gcode.write_text("G1 X9999 F3000\n")

    result = _run_cli(gcode)

    assert result.returncode != 0, result.stdout
    assert "FAIL" in result.stdout, result.stdout
