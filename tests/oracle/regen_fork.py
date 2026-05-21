#!/usr/bin/env python3
"""Fork-side oracle adapter.

Runs the fork's klippy in batch mode (`-i input.gcode -o /tmp/out -d
klipper.dict -l klippy.log`) for each `tests/oracle/inputs/*.gcode`,
parses the `[bridge-trace] move:` records the fork emits in
`klippy/motion_toolhead.py:367`, and reconstructs a 100 µs trajectory
CSV with the same schema as `expected/*.csv` so `diff.py` can compare.

Why reconstruct?
- Mainline emits `queue_step` MCU commands which klipper-sim decodes into
  per-stepper step times → CSV. The fork routes through `motion_bridge_native.so`
  + the Rust planner + the Linux MCU runtime, none of which produces
  `queue_step`. The only host-side seam that survives in debug mode is the
  per-move bridge handoff line.
- The reconstruction is a **trapezoidal kinematic integration** of the
  per-move records using `max_velocity` / `max_accel` / `square_corner_velocity`
  from the cfg, with Klipper's junction-deviation cornering between moves.
  This matches mainline's geometry at the per-move handoff layer; mismatches
  on the diff therefore localise to "the fork forwarded a different move list
  to its bridge than mainline planned." It does NOT capture post-bridge
  planner/shaper output — for that, the bridge would need a separate per-piece
  trace, which today only logs endpoints (`[bridge-trace] seg-dispatch`).

Failure mode: if klippy fails to start or no `[bridge-trace] move:` lines
appear in klippy.log, we save the klippy.log to `actual_fork/<stem>.log`
and skip the CSV; `diff.py` reports "MISSING FORK CAPTURE".
"""
from __future__ import annotations
import math
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent.parent
INPUTS_DIR = ROOT / "inputs"
ACTUAL_DIR = ROOT / "actual_fork"
CFG = ROOT / "cfg" / "oracle.cfg"
KLIPPER_DICT = Path.home() / "Developer" / "klipper-sim" / "test" / "fixtures" / "klipper.dict"
PY = sys.executable

# Trajectory grid + cfg constants (must match cfg/oracle.cfg).
DT = 1e-4              # 100 µs sample grid — matches mainline CSV.
MAX_V = 300.0          # mm/s
MAX_A = 5000.0         # mm/s²
SCV = 5.0              # square_corner_velocity, mm/s
# Klipper's junction-deviation derivation (klippy/toolhead.py:792):
JUNCTION_DEVIATION = SCV * SCV * (math.sqrt(2.0) - 1.0) / MAX_A

MOVE_RE = re.compile(
    r"\[bridge-trace\] move: newpos=\[([^\]]+)\] speed=[\d.\-eE]+ "
    r"dx=([\d.\-eE]+) dy=([\d.\-eE]+) dz=([\d.\-eE]+) de=([\d.\-eE]+) "
    r"feedrate=([\d.\-eE]+)"
)


def run_klippy(input_gcode: Path, log_path: Path) -> int:
    """Run fork klippy in batch mode; klippy.log goes to log_path."""
    out_bin = Path("/tmp") / f"oracle_fork_{input_gcode.stem}.out"
    out_bin.unlink(missing_ok=True)
    log_path.unlink(missing_ok=True)
    cmd = [PY, str(REPO / "klippy" / "klippy.py"), str(CFG),
           "-i", str(input_gcode), "-o", str(out_bin),
           "-d", str(KLIPPER_DICT), "-l", str(log_path)]
    return subprocess.run(cmd, cwd=str(REPO), capture_output=True, text=True).returncode


def parse_moves(log_path: Path) -> list[tuple[float, float, float, float, float]]:
    """Return list of (dx, dy, dz, de, feedrate_mm_s) per move."""
    moves = []
    for line in log_path.read_text(errors="replace").splitlines():
        m = MOVE_RE.search(line)
        if m:
            _, dx, dy, dz, de, fr = m.groups()
            moves.append((float(dx), float(dy), float(dz), float(de), float(fr)))
    return moves


def junction_v2(prev_axes_r, axes_r, prev_delta_v2, delta_v2):
    """Klipper's junction-deviation max-start-v² (klippy/toolhead.py:91-117)."""
    if prev_axes_r is None:
        return 0.0
    cos_theta = -sum(a * b for a, b in zip(prev_axes_r[:3], axes_r[:3]))
    sin_d2 = math.sqrt(max(0.5 * (1.0 - cos_theta), 0.0))
    cos_d2 = math.sqrt(max(0.5 * (1.0 + cos_theta), 0.0))
    if (1.0 - sin_d2) <= 0.0 or cos_d2 <= 0.0:
        return 0.0
    R_jd = sin_d2 / (1.0 - sin_d2)
    jd_v2 = R_jd * JUNCTION_DEVIATION * MAX_A
    qt = 0.25 * sin_d2 / cos_d2
    return min(jd_v2, delta_v2 * qt, prev_delta_v2 * qt)


def plan_velocities(moves):
    """Forward-backward pass over (move_d, max_cruise_v, junction_v) tuples,
    matching klippy's two-pass lookahead."""
    n = len(moves)
    md = [math.sqrt(sum(c * c for c in m[:4])) for m in moves]
    cruise = [min(m[4], MAX_V) for m in moves]
    axes_r = [tuple(c / d if d > 0 else 0.0 for c in m[:4]) for m, d in zip(moves, md)]
    delta_v2 = [2.0 * MAX_A * d for d in md]
    start_v2 = [0.0] * n
    for i in range(1, n):
        start_v2[i] = junction_v2(axes_r[i - 1], axes_r[i], delta_v2[i - 1], delta_v2[i])
    # Backward pass: end of move = start of next.
    end_v2 = [0.0] * n
    for i in range(n - 1, -1, -1):
        end_v2[i] = 0.0 if i == n - 1 else start_v2[i + 1]
    # Forward pass: cap each move's start_v2 by reachable-from-prev-end.
    for i in range(n):
        if i > 0:
            start_v2[i] = min(start_v2[i], end_v2[i - 1] + delta_v2[i])
        # Now cap end_v2 by reachable-from-start.
        end_v2[i] = min(end_v2[i], start_v2[i] + delta_v2[i])
    return md, cruise, start_v2, end_v2, axes_r


def sample_trajectory(moves, md, cruise, start_v2, end_v2, axes_r) -> list[tuple]:
    """Generate (t, x, y, z, e, vx, vy, vz, ve, ax, ay, az, ae) at DT grid."""
    rows = [(0.0,) + (0.0,) * 12]   # t=0 origin
    t_cursor = 0.0
    pos = [0.0, 0.0, 0.0, 0.0]
    for i, (m, d, vcr, sv2, ev2, ar) in enumerate(zip(moves, md, cruise, start_v2, end_v2, axes_r)):
        if d <= 0:
            continue
        vs = math.sqrt(sv2); ve = math.sqrt(ev2); vc = min(vcr, math.sqrt(max(sv2, ev2) + 2 * MAX_A * d))
        # Trapezoid distances.
        d_acc = max(0.0, (vc * vc - sv2) / (2 * MAX_A))
        d_dec = max(0.0, (vc * vc - ev2) / (2 * MAX_A))
        d_cru = d - d_acc - d_dec
        if d_cru < 0:    # triangular: vc capped by available distance
            vc = math.sqrt((2 * MAX_A * d + sv2 + ev2) / 2.0)
            d_acc = (vc * vc - sv2) / (2 * MAX_A)
            d_dec = (vc * vc - ev2) / (2 * MAX_A)
            d_cru = 0.0
        t_acc = (vc - vs) / MAX_A if MAX_A > 0 else 0.0
        t_cru = d_cru / vc if vc > 0 else 0.0
        t_dec = (vc - ve) / MAX_A if MAX_A > 0 else 0.0
        t_move = t_acc + t_cru + t_dec
        # Sample on the global 100 µs grid.
        t_end = t_cursor + t_move
        # Align: next sample is ceil(t_cursor/DT)*DT (but rows already up to last-emitted t).
        last_t = rows[-1][0]
        k = int(round(last_t / DT)) + 1
        while True:
            t = k * DT
            if t > t_end + 1e-9:
                break
            tau = t - t_cursor
            if tau < t_acc:
                v = vs + MAX_A * tau
                a = MAX_A
                s = vs * tau + 0.5 * MAX_A * tau * tau
            elif tau < t_acc + t_cru:
                v = vc; a = 0.0
                s = d_acc + vc * (tau - t_acc)
            else:
                td = tau - t_acc - t_cru
                v = vc - MAX_A * td
                a = -MAX_A
                s = d_acc + d_cru + vc * td - 0.5 * MAX_A * td * td
            # Project per-axis: unit vector × scalar.
            x = pos[0] + ar[0] * s; y = pos[1] + ar[1] * s
            z = pos[2] + ar[2] * s; e = pos[3] + ar[3] * s
            vx, vy, vz, ve_ = (ar[0] * v, ar[1] * v, ar[2] * v, ar[3] * v)
            ax, ay, az, ae_ = (ar[0] * a, ar[1] * a, ar[2] * a, ar[3] * a)
            rows.append((round(t, 7), x, y, z, e, vx, vy, vz, ve_, ax, ay, az, ae_))
            k += 1
        pos = [pos[0] + ar[0] * d, pos[1] + ar[1] * d, pos[2] + ar[2] * d, pos[3] + ar[3] * d]
        t_cursor = t_end
    # Trailing rest sample matching mainline's tail (one row at final t+DT, all zero v/a).
    if rows and t_cursor > rows[-1][0]:
        rows.append((round(t_cursor, 7), pos[0], pos[1], pos[2], pos[3]) + (0.0,) * 8)
    return rows


def write_csv(path: Path, rows):
    cols = ("t", "x", "y", "z", "e", "vx", "vy", "vz", "ve", "ax", "ay", "az", "ae")
    with path.open("w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(f"{v:.6g}" for v in r) + "\n")


def process(input_gcode: Path):
    stem = input_gcode.stem
    csv_out = ACTUAL_DIR / f"{stem}.csv"
    log_out = ACTUAL_DIR / f"{stem}.log"
    csv_out.unlink(missing_ok=True)
    rc = run_klippy(input_gcode, log_out)
    moves = parse_moves(log_out) if log_out.exists() else []
    if rc != 0 or not moves:
        print(f"[{stem}] klippy rc={rc} moves={len(moves)} — see {log_out}")
        return
    md, cruise, sv2, ev2, ar = plan_velocities(moves)
    rows = sample_trajectory(moves, md, cruise, sv2, ev2, ar)
    write_csv(csv_out, rows)
    print(f"[{stem}] OK {len(moves)} moves -> {len(rows)} samples ({csv_out.name})")


def main():
    ACTUAL_DIR.mkdir(parents=True, exist_ok=True)
    for gc in sorted(INPUTS_DIR.glob("*.gcode")):
        process(gc)


if __name__ == "__main__":
    main()
