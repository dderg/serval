"""uv run python -m tools.motion_eval CONFIG GCODE [-o out.json] [--png out.png]"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

from . import evaluate, mcu_dict


def _xyz(text):
    parts = [float(v) for v in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected X,Y,Z")
    return parts


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.motion_eval")
    ap.add_argument("config", type=pathlib.Path)
    ap.add_argument("gcode", type=pathlib.Path)
    ap.add_argument(
        "--start",
        type=_xyz,
        default=[0.0, 0.0, 0.0],
        help="toolhead position set with SET_KINEMATIC_POSITION before the"
        " gcode (default 0,0,0); the gcode must not home",
    )
    ap.add_argument("-o", "--output", type=pathlib.Path)
    ap.add_argument("--png", type=pathlib.Path)
    ap.add_argument("--dt", type=float, default=evaluate.DEFAULT_DT)
    ap.add_argument(
        "--sample-dt",
        type=float,
        default=evaluate.DEFAULT_SAMPLE_DT,
        help="time step of the series written to the JSON (0 disables)",
    )
    ap.add_argument("--mcu-config", default=mcu_dict.DEFAULT_MCU_CONFIG)
    args = ap.parse_args(argv)
    if args.png and not args.sample_dt:
        ap.error("--png needs --sample-dt > 0")

    result = evaluate.evaluate(
        args.config,
        args.gcode.read_text(),
        args.start,
        dt=args.dt,
        sample_dt=args.sample_dt,
        mcu_config=args.mcu_config,
    )
    if args.output:
        args.output.write_text(json.dumps(result))
    if args.png:
        from . import plot

        plot.render(result, args.png)
    summary = {
        k: v for k, v in result.items() if k not in ("samples", "commanded")
    }
    summary["commanded"] = {
        k: v for k, v in result["commanded"].items() if k != "moves"
    }
    json.dump(summary, sys.stdout, indent=1)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
