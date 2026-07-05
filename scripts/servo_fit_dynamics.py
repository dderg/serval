#!/usr/bin/env python3
"""Fit a dynamics profile TOML from a SERVO_MEASURE_INERTIA capture.

The SERVO_FIT_DYNAMICS macro records an excitation capture and then runs
this script, which resolves the newest capture for the given name, exports
the fitter CSV, and runs servo-ident. Each profile is written into
--out-dir under a name carrying the capture's timestamp, so a new fit never
replaces the profile a [servo_*] dynamics_profile line already points at —
switching profiles is an explicit config edit.

Usage:
  servo_fit_dynamics.py --name ident
  servo_fit_dynamics.py --name ident --rated-torque-nm 1.27 \
      --rotor-inertia-kgm2 0.000057 --rotation-distance-mm 40
"""

import argparse
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_capture import (  # noqa: E402
    CAPTURE_TS_RE,
    export_ident_csv,
    load_capture,
    resolve_newest_capture,
)


def profile_path(out_dir, name, capture_path):
    stamp = CAPTURE_TS_RE.search(os.path.basename(capture_path)).group(1)
    return os.path.join(
        os.path.expanduser(out_dir),
        "dynamics_%s_%s.toml" % (name, stamp),
    )


def ident_binary():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(repo_root, "rust", "target", "release", "servo-ident")
    if not os.path.exists(path):
        raise SystemExit(
            "%s missing — build it with: "
            "cargo build --release -p servo-ident" % (path,)
        )
    return path


def resolve_rotation_distance(args, header, drive_indices):
    if args.rotation_distance_mm is not None:
        return args.rotation_distance_mm
    distances = {
        header["drives"][i].get("rotation_distance") for i in drive_indices
    }
    if len(distances) != 1:
        raise SystemExit(
            "drives disagree on rotation_distance (%s); pass "
            "--rotation-distance-mm explicitly" % (sorted(distances),)
        )
    return distances.pop()


def ident_cmd(binary, csv_path, axes, out_path, args):
    cmd = [
        binary,
        "--capture",
        csv_path,
        "--structure",
        args.structure,
        "--axes",
        ",".join(axes),
        "--out",
        out_path,
    ]
    for flag, value in (
        ("--rated-torque-nm", args.rated_torque_nm),
        ("--rotor-inertia-kgm2", args.rotor_inertia_kgm2),
        ("--rotation-distance-mm", args.rotation_distance_mm),
    ):
        if value is not None:
            cmd += [flag, str(value)]
    return cmd


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", required=True, help="capture base name")
    p.add_argument(
        "--captures-dir", default="~/printer_data/logs/servo_captures"
    )
    p.add_argument("--out-dir", default="~/printer_data/config/servo_dynamics")
    p.add_argument("--structure", default="scalar")
    p.add_argument("--rated-torque-nm", type=float)
    p.add_argument("--rotor-inertia-kgm2", type=float)
    p.add_argument("--rotation-distance-mm", type=float)
    p.add_argument(
        "--drive",
        help="drive name to fit in a multi-drive capture "
        "(default: the first drive in the file)",
    )
    args = p.parse_args(argv)

    capture_path = resolve_newest_capture(args.captures_dir, args.name)
    if args.structure == "corexy":
        if args.drive is not None:
            raise SystemExit("--drive conflicts with --structure corexy")
        header, _, _ = load_capture(capture_path)
        drive_names = [d["name"] for d in header["drives"]]
        if len(drive_names) != 2:
            raise SystemExit(
                "corexy fit needs a 2-drive capture, got drives: %s"
                % (", ".join(drive_names),)
            )
        drive_datas = [
            (idx, load_capture(capture_path, name)[1])
            for idx, name in enumerate(drive_names)
        ]
    else:
        header, data, drive_idx = load_capture(capture_path, args.drive)
        drive_datas = [(drive_idx, data)]
    drive_indices = [idx for idx, _ in drive_datas]
    axes = [header["drives"][idx]["name"] for idx in drive_indices]
    args.rotation_distance_mm = resolve_rotation_distance(
        args, header, drive_indices
    )
    if args.rotation_distance_mm is None and args.rated_torque_nm is not None:
        print(
            "note: capture predates rotation_distance in the header; pass "
            "--rotation-distance-mm or re-capture to get the C00.06 "
            "recommendation"
        )

    out_dir = os.path.expanduser(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    out_path = profile_path(args.out_dir, args.name, capture_path)

    binary = ident_binary()
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".csv", delete=False
    ) as tmp:
        csv_path = tmp.name
    try:
        export_ident_csv(csv_path, header, drive_datas)
        proc = subprocess.run(
            ident_cmd(binary, csv_path, axes, out_path, args),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    finally:
        os.unlink(csv_path)
    sys.stdout.write(proc.stdout)
    if proc.returncode != 0:
        raise SystemExit(proc.returncode)
    if not os.path.exists(out_path):
        print(
            "no dynamics profile written (nonphysical fit - see analysis above)"
        )
        return 0
    print("profile: %s" % (out_path,))
    for axis in axes:
        print(
            "to use it: set dynamics_profile: %s under [servo_%s] and RESTART"
            % (out_path, axis.split("_")[-1])
        )
    return 0


if __name__ == "__main__":
    main()
