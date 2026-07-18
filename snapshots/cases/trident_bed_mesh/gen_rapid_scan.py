"""Regenerate rapid_scan.gcode — the exact waypoint stream the Beacon plugin's
BED_MESH_CALIBRATE (rapid scan) feeds the planner on the Trident bench.

Replicates BeaconMeshHelper._generate_path / arc_points from beacon_klipper
bit-for-bit (same float expressions), then applies the same filtering klippy's
Motion.move applies before the engine sees a move: only exact zero-length
moves are dropped (klippy/motion.py `if not move.move_d: return`), so the
near-duplicate seam points where the two U-turn arcs meet survive at full
precision.

Trident parameters (printer_data/config on trident.local):
  [bed_mesh] mesh_min 25,25  mesh_max 275,275  probe_count 20,20  speed 600
  [beacon]   x_offset 0  y_offset 18  mesh_main_direction x  mesh_runs 2
             mesh_overscan unset -> auto from machine x range [-3, 300]
"""

import math
from pathlib import Path

MIN_X, MIN_Y = 25.0, 25.0
MAX_X, MAX_Y = 275.0, 275.0
RES_X, RES_Y = 20, 20
X_OFFSET, Y_OFFSET = 0.0, 18.0
MACHINE_X = (-3.0, 300.0)
SPEED_MM_S = 600.0
RUNS = 2


def arc_points(cx, cy, r, start_angle, span):
    start_angle = start_angle / 180.0 * math.pi
    span = span / 180.0 * math.pi
    d_a = math.acos(1 - 0.1 / r)
    cnt = int(math.ceil(abs(span) / d_a))
    d_a = span / float(cnt)
    points = []
    for i in range(cnt + 1):
        ang = start_angle + d_a * float(i)
        x = cx + math.cos(ang) * r
        y = cy + math.sin(ang) * r
        points.append((x, y))
    return points


def auto_overscan():
    begin, end = MIN_X - X_OFFSET, MAX_X - X_OFFSET
    space = (end - begin) / (float(RES_Y - 1))
    return min(
        max(0, begin - MACHINE_X[0]),
        max(0, MACHINE_X[1] - end),
        space + 2.0,
    )


def generate_path(overscan):
    begin_a, end_a = MIN_X - X_OFFSET, MAX_X - X_OFFSET
    begin_p, end_p = MIN_Y - Y_OFFSET, MAX_Y - Y_OFFSET
    count = RES_Y
    step = (end_p - begin_p) / (float(count - 1))
    points = []
    corner_radius = min(step / 2, overscan)
    for i in range(0, count):
        pos_p = begin_p + step * i
        even = i % 2 == 0
        pa = (begin_a, pos_p) if even else (end_a, pos_p)
        pb = (end_a, pos_p) if even else (begin_a, pos_p)
        line = (pa, pb)
        if len(points) > 0 and corner_radius > 0:
            if even:
                center = begin_a - overscan + corner_radius
                points += arc_points(
                    center,
                    pos_p - step + corner_radius,
                    corner_radius,
                    -90,
                    -90,
                )
                points += arc_points(
                    center, pos_p - corner_radius, corner_radius, -180, -90
                )
            else:
                center = end_a + overscan - corner_radius
                points += arc_points(
                    center, pos_p - step + corner_radius, corner_radius, -90, 90
                )
                points += arc_points(
                    center, pos_p - corner_radius, corner_radius, 0, 90
                )
        points.append(line[0])
        points.append(line[1])
    return points


def main():
    path = generate_path(auto_overscan())
    stream = []
    for run in range(RUNS):
        stream.extend(path if run % 2 == 0 else reversed(path))
    lines = []
    feed = f" F{SPEED_MM_S * 60:g}"
    prev = None
    dropped = 0
    for x, y in stream:
        if prev == (x, y):
            dropped += 1
            continue
        lines.append(f"G1 X{x!r} Y{y!r}{feed}")
        feed = ""
        prev = (x, y)
    out = Path(__file__).with_name("rapid_scan.gcode")
    out.write_text("\n".join(lines) + "\n")
    print(
        f"{out.name}: {len(lines)} waypoints ({dropped} exact duplicates dropped)"
    )


if __name__ == "__main__":
    main()
