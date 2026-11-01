"""Send gcode lines to the bench printer through Moonraker.

uv run python tools/bench/gcode.py "G1 X10" "M400" [--lock-timeout S]

Each line is run in order under the bench lock; the Moonraker reply is
printed and any error stops the run with a non-zero exit.
"""

from __future__ import annotations

import argparse
import json
import urllib.error

import lock
import pi


def run_line(line: str) -> None:
    body = json.dumps({"script": line}).encode()
    try:
        reply = pi.http(
            "POST",
            "/printer/gcode/script",
            body,
            {"Content-Type": "application/json"},
        )
    except urllib.error.HTTPError as exc:
        raise pi.BenchError(
            f"{line!r}: HTTP {exc.code}: {exc.read().decode(errors='replace')}"
        ) from exc
    print(f"{line!r} -> {reply}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("lines", nargs="+")
    parser.add_argument("--dry-run", action="store_true")
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    if args.dry_run:
        for line in args.lines:
            print(f"POST {pi.MOONRAKER}/printer/gcode/script {line!r}")
        return
    with lock.bench_lock(args.lock_timeout):
        for line in args.lines:
            run_line(line)


if __name__ == "__main__":
    main()
