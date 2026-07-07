#!/usr/bin/env python3
"""Port of beacon_klipper BeaconMeshHelper._generate_path / arc_points.

Emits the exact serpentine-with-overscan-corner polyline the beacon rapid-scan
mesh drives, as a plain G-code file, so the motion path can be exercised in the
sim WITHOUT the beacon stream in the loop. This isolates the planner/fitter
from the beacon-emulator stream-stall confound.

Geometry matches the fork verbatim (chord deviation 0.1mm corner arcs). Run
parameters default to the Trident bench config.
"""
import argparse
import math


def arc_points(cx, cy, r, start_angle, span):
    start_angle = start_angle / 180.0 * math.pi
    span = span / 180.0 * math.pi
    d_a = math.acos(1 - 0.1 / r)
    cnt = int(math.ceil(abs(span) / d_a))
    d_a = span / float(cnt)
    pts = []
    for i in range(cnt + 1):
        ang = start_angle + d_a * float(i)
        pts.append((cx + math.cos(ang) * r, cy + math.sin(ang) * r))
    return pts


def generate_path(min_x, min_y, max_x, max_y, res_x, res_y,
                  direction, overscan, x_offset, y_offset):
    settings = {
        "x": {
            "range_aligned": [min_x - x_offset, max_x - x_offset],
            "range_perpendicular": [min_y - y_offset, max_y - y_offset],
            "count": res_y,
            "swap_coord": False,
        },
        "y": {
            "range_aligned": [min_y - y_offset, max_y - y_offset],
            "range_perpendicular": [min_x - x_offset, max_x - x_offset],
            "count": res_x,
            "swap_coord": True,
        },
    }[direction]

    begin_a, end_a = settings["range_aligned"]
    begin_p, end_p = settings["range_perpendicular"]
    swap_coord = settings["swap_coord"]
    step = (end_p - begin_p) / (float(settings["count"] - 1))
    points = []
    corner_radius = min(step / 2, overscan)
    for i in range(0, settings["count"]):
        pos_p = begin_p + step * i
        even = i % 2 == 0
        pa = (begin_a, pos_p) if even else (end_a, pos_p)
        pb = (end_a, pos_p) if even else (begin_a, pos_p)
        line = (pa, pb)
        if len(points) > 0 and corner_radius > 0:
            if even:
                center = begin_a - overscan + corner_radius
                points += arc_points(center, pos_p - step + corner_radius,
                                     corner_radius, -90, -90)
                points += arc_points(center, pos_p - corner_radius,
                                     corner_radius, -180, -90)
            else:
                center = end_a + overscan - corner_radius
                points += arc_points(center, pos_p - step + corner_radius,
                                     corner_radius, -90, 90)
                points += arc_points(center, pos_p - corner_radius,
                                     corner_radius, 0, 90)
        points.append(line[0])
        points.append(line[1])

    if swap_coord:
        points = [(y, x) for (x, y) in points]
    return points


def auto_overscan(min_c, max_c, count, machine_min, machine_max):
    space = (max_c - min_c) / float(count - 1)
    return min(max(0, min_c - machine_min), max(0, machine_max - max_c),
               space + 2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min", default="25,25")
    ap.add_argument("--max", default="275,275")
    ap.add_argument("--count", default="20,20")
    ap.add_argument("--dir", default="y", choices=["x", "y"])
    ap.add_argument("--speed", type=float, default=800.0)
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--x-offset", type=float, default=0.0)
    ap.add_argument("--y-offset", type=float, default=0.0)
    ap.add_argument("--machine", default="0,0,300,300",
                    help="xmin,ymin,xmax,ymax for overscan auto-calc")
    ap.add_argument("--overscan", type=float, default=None)
    ap.add_argument("--z", type=float, default=2.0, help="scan Z height")
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()

    min_x, min_y = map(float, args.min.split(","))
    max_x, max_y = map(float, args.max.split(","))
    res_x, res_y = map(int, args.count.split(","))
    mxmin, mymin, mxmax, mymax = map(float, args.machine.split(","))

    if args.overscan is not None:
        overscan = args.overscan
    else:
        # beacon computes overscan at connect from DEFAULT res, dir-dependent
        if args.dir == "x":
            overscan = auto_overscan(min_y - args.y_offset, max_y - args.y_offset,
                                     res_x, mymin, mymax)
        else:
            overscan = auto_overscan(min_x - args.x_offset, max_x - args.x_offset,
                                     res_y, mxmin, mxmax)

    path = generate_path(min_x, min_y, max_x, max_y, res_x, res_y,
                         args.dir, overscan, args.x_offset, args.y_offset)

    with open(args.out, "w") as f:
        f.write("; beacon scan path repro\n")
        f.write("; dir=%s speed=%.0f runs=%d overscan=%.3f count=%dx%d\n"
                % (args.dir, args.speed, args.runs, overscan, res_x, res_y))
        f.write("; %d path points per run\n" % len(path))
        f.write("G90\n")
        f.write("SET_KINEMATIC_POSITION X=%.3f Y=%.3f Z=%.3f\n"
                % (path[0][0], path[0][1], args.z))
        fr = args.speed * 60.0
        for i in range(args.runs):
            seq = path if i % 2 == 0 else list(reversed(path))
            for (x, y) in seq:
                f.write("G1 X%.4f Y%.4f F%.0f\n" % (x, y, fr))
        f.write("M400\n")
    print("wrote %s: %d moves/run x %d runs" % (args.out, len(path), args.runs))


if __name__ == "__main__":
    main()
