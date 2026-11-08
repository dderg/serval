"""One print through the whole virtual printer: gcode -> klippy -> XY
mechanics and motors -> extruder and melt -> bead deposition -> surface
metrics and renders."""

from __future__ import annotations

import json
import math
import pathlib

import numpy as np

from tools.motion_eval import run

from . import (
    deposition,
    drive,
    extrusion,
    gcodemoves,
    mechanics,
    motor,
    parts,
    render,
    sim,
    surface,
)

EXTRUDER_RATE_SMOOTH_S = 1e-3
LOST_PART_MM = 5.0
KEEP_EVERY_LAYERS = 5


class PrintError(Exception):
    pass


def layer_height(moves):
    zs = sorted({round(m.end[2], 6) for m in moves if m.extrusion > 0.0})
    steps = np.diff([0.0] + zs)
    if len(zs) < 1 or not np.allclose(steps, steps[0], rtol=0.0, atol=1e-6):
        raise PrintError(
            "printed layers are not evenly spaced: %r" % (zs[:10],)
        )
    return float(steps[0])


def nominal_width(moves, filament_area, h):
    ratios = [
        m.extrusion / m.length_xy
        for m in moves
        if m.kind == "WALL-OUTER" and m.length_xy > 0.0
    ]
    if not ratios:
        raise PrintError("no ;TYPE:WALL-OUTER moves to measure")
    return float(np.median(ratios)) * filament_area / h


def held(times, positions, t):
    return positions[np.searchsorted(times, t, side="right")]


def smooth_rate(times, positions, grid):
    """Filament speed on the output grid: the slope of the step positions,
    averaged over EXTRUDER_RATE_SMOOTH_S."""
    pos = np.interp(grid, times, positions[1:], left=positions[0])
    rate = np.gradient(pos, grid)
    width = max(1, int(round(EXTRUDER_RATE_SMOOTH_S / (grid[1] - grid[0]))))
    return np.convolve(rate, np.ones(width) / width, mode="same")


def simulate(text, parts_path, printer_cfg):
    p = parts.load(parts_path, printer_cfg)
    if p.extrusion is None:
        raise PrintError(
            "%s has no [melt] section: no extrusion model" % (parts_path,)
        )
    ex = p.extrusion
    moves = gcodemoves.parse(text)
    raw = run.run_klippy(printer_cfg, text, gcodemoves.start_position(text))
    pairs = gcodemoves.match(moves, raw.capture["moves"])
    e = motor.Electrical.from_parts(p)
    system = mechanics.build(
        p, mechanics.calibrate(p, e.magnetic_stiffness), e.magnetic_stiffness
    )
    d = drive.from_run(raw, printer_cfg, e.pole_pairs)
    if d.extruder_steps is None:
        raise PrintError("klippy recorded no extruder steps")
    xy_res = sim.run(
        sim.Model(system, e),
        d.t0,
        drive.DT,
        d.n,
        d.step_command,
        d.command_rate,
    )
    times, positions = d.extruder_steps
    per_mm = extrusion.angle_per_mm(ex)
    grid_t = d.t0 + drive.DT * np.arange(d.n + 2)
    rate = smooth_rate(times, positions, grid_t)
    ex_res = extrusion.run(
        ex,
        d.t0,
        drive.DT,
        d.n,
        lambda t: per_mm * held(times, positions, t),
        lambda t: per_mm * np.interp(t, grid_t, rate),
    )
    return p, moves, pairs, raw, d, xy_res, ex_res


def analyse(p, moves, pairs, raw, d, xy_res, ex_res, out_dir, title):
    ex = p.extrusion
    h = layer_height(moves)
    w_nom = nominal_width(moves, ex.filament_area, h)
    nozzle = xy_res.nozzle * 1e3
    t = xy_res.t
    printed = [m for m in moves if m.extrusion > 0.0]
    lo = np.min(
        [m.start[:2] for m in printed] + [m.end[:2] for m in printed], axis=0
    )
    hi = np.max(
        [m.start[:2] for m in printed] + [m.end[:2] for m in printed], axis=0
    )
    laying = (deposition.layer_of(d.z, h) >= 0) & (
        np.diff(ex_res.volume, prepend=0.0) > 0.0
    )
    excursion = float(
        max(
            np.max(lo - nozzle[laying].min(axis=0)),
            np.max(nozzle[laying].max(axis=0) - hi),
        )
    )
    top_layer = max(m.layer for m in printed)
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    layers = None
    if excursion > LOST_PART_MM:
        metrics = {
            "error": "the nozzle laid material %.1f mm outside the part: lost steps"
            % excursion
        }
    else:
        grid = deposition.Grid.around(
            np.minimum(lo, nozzle[laying].min(axis=0)),
            np.maximum(hi, nozzle[laying].max(axis=0)),
        )
        keep = set(range(0, top_layer + 1, KEEP_EVERY_LAYERS)) | {top_layer}
        layers = deposition.lay(
            grid, nozzle, d.z, ex_res.volume, h, ex.max_bead_width, keep
        )
        speed = np.hypot(*np.gradient(nozzle, t, axis=0).T)
        with np.errstate(divide="ignore", invalid="ignore"):
            width = np.where(speed > 0.0, ex_res.flow / (speed * h), np.nan)
        wall_list = surface.walls(
            pairs, t, nozzle, width, w_nom, max(abs(v) for v in d.support)
        )
        corner_list = surface.corners(pairs, layers, w_nom)
        metrics = surface.summarize(wall_list, corner_list, w_nom)
        render.wall(out_dir / "wall.png", wall_list, h, title + ": outer wall")
        render.top(
            out_dir / "top.png",
            layers,
            (top_layer + 1) * h,
            title + ": top surface",
        )
    cap = raw.capture["moves"]
    dv = np.diff(ex_res.volume, prepend=0.0)
    commanded = sum(m.extrusion for m in moves) * ex.filament_area
    report = {
        "title": title,
        "print_time_s": gcodemoves.move_end_time(cap[-1])
        - cap[0]["print_time"],
        "layer_height_mm": h,
        "nominal_width_mm": w_nom,
        "volume_mm3": {
            "commanded": commanded,
            "out_of_nozzle_net": float(ex_res.volume[-1]),
            "out_of_nozzle_positive": float(np.maximum(dv, 0.0).sum()),
            "laid_on_layers": None
            if layers is None
            else sum(layers.flowed.values()),
            "deposited_on_grid": None
            if layers is None
            else sum(layers.deposited.values()),
        },
        "max_excursion_mm": excursion,
        "flow_mm3_s_max": float(np.max(ex_res.flow)),
        "xy_motors": {
            name: {
                "max_load_angle_deg": float(
                    math.degrees(xy_res.delta_absmax[:, m].max())
                ),
                "pole_slips": int(
                    sim.pole_slips(xy_res.delta[:, m : m + 1])[0]
                ),
            }
            for m, name in enumerate(("a", "b"))
        },
        "extruder": {
            "max_load_angle_deg": float(
                math.degrees(ex_res.delta_absmax.max())
            ),
            "pole_slips": int(sim.pole_slips(ex_res.delta[:, None])[0]),
            "max_force_n": float(ex_res.force.max()),
        },
        "surface": metrics,
    }
    (out_dir / "report.json").write_text(json.dumps(report, indent=1))
    return report


def print_gcode(text, parts_path, printer_cfg, out_dir, title):
    return analyse(*simulate(text, parts_path, printer_cfg), out_dir, title)
