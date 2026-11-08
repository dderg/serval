"""python -m tools.printer_sim print <gcode> --parts <cfg> --printer <printer.cfg>"""

from __future__ import annotations

import argparse
import json
import pathlib

from . import printjob


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.printer_sim")
    sub = ap.add_subparsers(dest="command", required=True)
    pr = sub.add_parser(
        "print", help="simulate one print: report.json, wall.png, top.png"
    )
    pr.add_argument("gcode", type=pathlib.Path)
    pr.add_argument("--parts", type=pathlib.Path, required=True)
    pr.add_argument("--printer", type=pathlib.Path, required=True)
    pr.add_argument("--out", type=pathlib.Path)
    args = ap.parse_args(argv)
    out = args.out or args.gcode.with_suffix(".printer_sim")
    report = printjob.print_gcode(
        args.gcode.read_text(), args.parts, args.printer, out, args.gcode.stem
    )
    print(json.dumps(report["surface"], indent=1))
    print("wrote %s" % (out,))


if __name__ == "__main__":
    main()
