#!/usr/bin/env python3
"""Simulator CLI — print G-code against real firmware, or serve a
long-lived printer for Moonraker/Mainsail.

Run through tools/sim/run.sh (which builds the Docker image), or directly
inside the sim container:

    python3 tools/sim/cli.py                      # self-test print
    python3 tools/sim/cli.py --gcode file.gcode   # print a file
    python3 tools/sim/cli.py --serve              # hold for Moonraker

For scripted scenarios (homing, probing, phase stepping, ...) use the
pytest suite: tools/sim/run.sh test
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import tempfile
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from tools.sim import configs  # noqa: E402
from tools.sim.world import SimError, SimWorld  # noqa: E402

SERVE_EXTRAS = """
[pause_resume]

[display_status]

[exclude_object]

[gcode_macro PAUSE]
rename_existing: BASE_PAUSE
gcode:
  BASE_PAUSE

[gcode_macro RESUME]
rename_existing: BASE_RESUME
gcode:
  BASE_RESUME

[gcode_macro CANCEL_PRINT]
rename_existing: BASE_CANCEL_PRINT
gcode:
  CLEAR_PAUSE
  BASE_CANCEL_PRINT
"""


def _clean_serve_dir(data_dir: pathlib.Path) -> None:
    """A supervisor restart re-enters on the same persistent volume. Drop
    transient state and reap orphaned sim processes from a prior crashed
    run; events/*.jsonl logs are deliberately preserved."""
    data_dir.mkdir(parents=True, exist_ok=True)
    for name in ("klippy.sock", "SERVE_READY"):
        try:
            (data_dir / name).unlink()
        except FileNotFoundError:
            pass
    for pat in ("klipper-h7-sim.elf", "klipper-f4-sim.elf"):
        subprocess.run(["pkill", "-f", pat], check=False)


def run_print(args) -> int:
    wall_start = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="sim_") as tmpdir:
        world = SimWorld(pathlib.Path(tmpdir), verbose=args.verbose)
        try:
            world.boot(
                configs.minimal_config(world.h7_pty, str(world.gcode_dir)),
            )
            if args.gcode:
                gcode_path = pathlib.Path(args.gcode)
            else:
                gcode_path = world.gcode_dir / "self_test.gcode"
                gcode_path.write_text(configs.SELF_TEST_GCODE)
            print_time = world.print_file(gcode_path, timeout=args.timeout)
        except SimError as e:
            print(f"FAIL: {e}")
            world.dump_diagnostics()
            return 1
        finally:
            world.shutdown()

    wall = time.monotonic() - wall_start
    print("PASS")
    print(f"  print time: {print_time:.1f}s")
    print(f"  wall time:  {wall:.1f}s")
    if print_time and wall:
        print(f"  speedup:    {print_time / wall:.1f}x")
    return 0


def run_serve(args) -> int:
    data_dir = pathlib.Path(args.data_dir or "/tmp/sim_serve")
    _clean_serve_dir(data_dir)
    world = SimWorld(data_dir, verbose=args.verbose)
    world.log_dir.mkdir(parents=True, exist_ok=True)
    try:
        cfg = configs.minimal_config(world.h7_pty, str(world.gcode_dir))
        cfg += SERVE_EXTRAS
        world.boot(cfg)
        (data_dir / "SERVE_READY").write_text(
            f"api_socket={world.api_socket}\n"
            f"klippy_log={world.klippy_log}\n"
            f"events_dir={world.log_dir / 'events'}\n"
        )
        print(f"SERVE: ready for Moonraker. api={world.api_socket}")
        while world.klippy_proc.poll() is None:
            time.sleep(1.0)
        return world.klippy_proc.returncode or 0
    except KeyboardInterrupt:
        return 0
    finally:
        world.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Kalico full-stack simulator")
    parser.add_argument("--gcode", help="G-code file to print")
    parser.add_argument(
        "--timeout",
        type=float,
        default=600,
        help="Max wall-clock seconds for the print (default: 600)",
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Long-lived interactive mode for Moonraker/Mainsail",
    )
    parser.add_argument(
        "--data-dir", help="Stable printer_data dir for --serve"
    )
    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()

    if args.serve:
        return run_serve(args)
    return run_print(args)


if __name__ == "__main__":
    sys.exit(main())
