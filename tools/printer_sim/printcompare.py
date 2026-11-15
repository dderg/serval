"""Phase-2 end-to-end: the generated cube and tower on each parts set
under each corner setting.

  PYTHONPATH=. uv run python -m tools.printer_sim.printcompare [--jobs N]
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import pathlib

from tools.motion_eval import accel_residual

from . import gcodegen, parts, printjob

SETTINGS = {
    "scv50": (50.0, "none"),
    "floor": (None, "floor"),
    "bounded81000": (None, "bound81000"),
    "scv5": (5.0, "none"),
}
PARTS = ("voron_48v_a", "voron_48v_b")
OUT = parts.HERE / ".build" / "prints"


def printer_cfg(directory, setting, parts_name):
    """owner.cfg with this setting's corner handling and pressure advance
    tuned to the parts list's melt time constant."""
    p = parts.load(
        parts.PARTS_DIR / (parts_name + ".cfg"), accel_residual.DEFAULT_CONFIG
    )
    scv, corner = SETTINGS[setting]
    text = "[include %s]\n" % (accel_residual.DEFAULT_CONFIG,)
    text += "\n[extruder]\npressure_advance: %r\n" % (
        p.extrusion.time_constant,
    )
    if scv is not None:
        text += "\n[printer]\nsquare_corner_velocity: %r\n" % (scv,)
    section = accel_residual.corner_deviation_section(corner)
    if section is not None:
        text += "\n" + section
    path = pathlib.Path(directory) / "printer.cfg"
    path.write_text(text)
    return path


def run_one(job):
    print_name, parts_name, setting = job
    out = OUT / print_name / parts_name / setting
    out.mkdir(parents=True, exist_ok=True)
    cfg = printer_cfg(out, setting, parts_name)
    text = gcodegen.generate(gcodegen.PRINTS[print_name])
    (out / (print_name + ".gcode")).write_text(text)
    report = printjob.print_gcode(
        text,
        parts.PARTS_DIR / (parts_name + ".cfg"),
        cfg,
        out,
        "%s %s %s" % (print_name, parts_name, setting),
    )
    return job, report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--jobs", type=int, default=4)
    args = ap.parse_args(argv)
    jobs = [
        (pr, pa, se)
        for pr in gcodegen.PRINTS
        for pa in PARTS
        for se in SETTINGS
    ]
    with concurrent.futures.ProcessPoolExecutor(args.jobs) as pool:
        results = list(pool.map(run_one, jobs))
    summary = {"/".join(job): report for job, report in results}
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1))
    print(table(results))


def table(results):
    lines = [
        "| print | parts | setting | print time s | ghosting um by outer speed (median/max) | corner bulge / under-fill mm^3 (mean) | line width rel std | layer shift mm | size error mm | slips A/B/E |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for (pr, pa, se), r in results:
        s = r["surface"]
        slips = "%d / %d / %d" % (
            r["xy_motors"]["a"]["pole_slips"],
            r["xy_motors"]["b"]["pole_slips"],
            r["extruder"]["pole_slips"],
        )
        if "error" in s:
            lines.append(
                "| %s | %s | %s | %.2f | %s | | | | | %s |"
                % (pr, pa, se, r["print_time_s"], s["error"], slips)
            )
            continue
        ghost = ", ".join(
            "%s: %.1f/%.1f"
            % (v, g["amplitude_um_median"], g["amplitude_um_max"])
            for v, g in s["ghosting"].items()
        )
        c = s.get("corners", {})
        lines.append(
            "| %s | %s | %s | %.2f | %s | %.4f / %.4f | %.3f | %.3f | %.3f | %s |"
            % (
                pr,
                pa,
                se,
                r["print_time_s"],
                ghost,
                c.get("bulge_mm3_mean", float("nan")),
                c.get("underfill_mm3_mean", float("nan")),
                s["line_width"]["relative_std"],
                s["layer_shift_mm"],
                s["size_error_mm"],
                slips,
            )
        )
    return "\n".join(lines)


if __name__ == "__main__":
    main()
