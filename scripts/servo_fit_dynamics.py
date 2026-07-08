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


def parse_pairs(pairs_text):
    pairs = [
        [name.strip() for name in pair.split(",") if name.strip()]
        for pair in pairs_text.split(";")
        if pair.strip()
    ]
    if len(pairs) != 2 or any(len(pair) != 2 for pair in pairs):
        raise SystemExit(
            "--pairs must name two belt pairs of two drives each, e.g. "
            "'motor_a,motor_a1;motor_b,motor_b1' (got %r)" % (pairs_text,)
        )
    return pairs


def corexy_layout(drive_names, pairs_text):
    """Resolve the axes order and servo-ident structure for a corexy fit.

    2-drive captures use the classic coupled fit in capture order. 4-drive
    (AWD) captures need --pairs to state which drives share each belt; the
    profile rows must land in slot order, so the caller passes pairs in
    slot order and the axes order is taken from them verbatim.
    """
    if len(drive_names) == 2:
        if pairs_text is not None:
            raise SystemExit("--pairs needs a 4-drive capture, this one has 2")
        return list(drive_names), "corexy"
    if len(drive_names) == 4:
        if pairs_text is None:
            raise SystemExit(
                "4-drive corexy capture needs --pairs "
                "'a0,a1;b0,b1' naming each belt's drives in slot order "
                "(capture drives: %s)" % (", ".join(drive_names),)
            )
        pairs = parse_pairs(pairs_text)
        axes = [name for pair in pairs for name in pair]
        if sorted(axes) != sorted(drive_names):
            raise SystemExit(
                "--pairs drives (%s) do not match the capture drives (%s)"
                % (", ".join(axes), ", ".join(drive_names))
            )
        return axes, "corexy-awd"
    raise SystemExit(
        "corexy fit needs a 2-drive or 4-drive (AWD, with --pairs) capture, "
        "got drives: %s" % (", ".join(drive_names),)
    )


def ident_cmd(binary, csv_path, axes, out_path, args, structure=None):
    cmd = [
        binary,
        "--capture",
        csv_path,
        "--structure",
        structure or args.structure,
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
        help="drive name to fit in a multi-drive capture",
    )
    p.add_argument(
        "--pairs",
        help="AWD corexy belt pairing in slot order, "
        "e.g. 'motor_a,motor_a1;motor_b,motor_b1'",
    )
    args = p.parse_args(argv)

    capture_path = resolve_newest_capture(args.captures_dir, args.name)
    structure = args.structure
    if args.structure == "corexy":
        if args.drive is not None:
            raise SystemExit("--drive conflicts with --structure corexy")
        header, _, _ = load_capture(capture_path)
        drive_names = [d["name"] for d in header["drives"]]
        axes, structure = corexy_layout(drive_names, args.pairs)
        drive_datas = [
            (drive_names.index(name), load_capture(capture_path, name)[1])
            for name in axes
        ]
    else:
        if args.pairs is not None:
            raise SystemExit("--pairs needs --structure corexy")
        header, data, drive_idx = load_capture(capture_path, args.drive)
        drive_names = [d["name"] for d in header["drives"]]
        if args.drive is None and len(drive_names) > 1:
            raise SystemExit(
                "capture holds %d drives (%s); pass --drive to pick which "
                "one the scalar fit describes"
                % (len(drive_names), ", ".join(drive_names))
            )
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
            ident_cmd(binary, csv_path, axes, out_path, args, structure),
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
    if structure == "scalar":
        for axis in axes:
            print(
                "to use it: set dynamics_profile: %s under [motor %s] "
                "and RESTART" % (out_path, axis)
            )
    else:
        print(
            "to use it: this is a coupled profile — set dynamics_profile: %s "
            "under [ethercat_node] and RESTART" % (out_path,)
        )
    return 0


if __name__ == "__main__":
    main()
