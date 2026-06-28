#!/usr/bin/env python3
"""Diagnostic: fit the scalar inertia three ways from one .scap and compare.

Regresses measured torque (6077h) against three acceleration sources, over the
same steady-plateau sample set, so the only variable is the regressor:

  cmd       commanded analytic accel (accel_cmd)  -- the current fitter
  vel_diff  d/dt of actual velocity (6063h)        -- proposed: clean a_actual
  pos_ddiff d2/dt2 of actual position (6064h)       -- the original method

For each it prints fitted mass M, rms residual, and (with motor datasheet
numbers) the implied C00.06. The physically exact relation tau = J*a_actual +
friction means the vel_diff fit should be stable across C00.06 settings, while
cmd drifts with the loop's tracking. Run the same capture taken at two C00.06
values and compare.

Usage:
  servo_fit_compare.py run.scap [--drive y] [--smooth 5] \
      [--rated-torque-nm 1.27 --rotor-inertia-kgm2 0.000057]
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_capture import FLAG_MOTION_ACTIVE, load_capture  # noqa: E402

COULOMB_DEADBAND_MM_S = 0.5
SETTLE_S = 0.012
TOL_FRAC = 0.03
TOL_FLOOR = 1.0


def steady_mask(accel_cmd, dt):
    """Trailing-window constant-commanded-accel mask, matching the Rust fitter."""
    n = len(accel_cmd)
    window = max(1, int(round(SETTLE_S / dt)))
    tol = max(
        TOL_FLOOR, TOL_FRAC * np.max(np.abs(accel_cmd)) if n else TOL_FLOOR
    )
    mask = np.zeros(n, dtype=bool)
    for k in range(window, n):
        seg = accel_cmd[k - window : k + 1]
        if seg.max() - seg.min() <= tol:
            mask[k] = True
    return mask


def scalar_fit(accel, vel, torque):
    """Least-squares tau = M*a + b*v + cf*[v>db] + cr*[v<-db]."""
    cf = (vel > COULOMB_DEADBAND_MM_S).astype(np.float64)
    cr = (vel < -COULOMB_DEADBAND_MM_S).astype(np.float64)
    design = np.column_stack([accel, vel, cf, cr])
    theta, _, _, _ = np.linalg.lstsq(design, torque, rcond=None)
    resid = torque - design @ theta
    rms = float(np.sqrt(np.mean(resid**2)))
    return theta[0], rms


def c0006(m, rated_nm, rot_dist_mm, rotor_kgm2):
    j_total = m * (rated_nm / 1000.0) * rot_dist_mm / (2.0 * np.pi)
    return (j_total - rotor_kgm2) / rotor_kgm2 * 100.0


def smooth(x, n):
    if n <= 1:
        return x
    kernel = np.ones(n) / n
    return np.convolve(x, kernel, mode="same")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture")
    p.add_argument("--drive")
    p.add_argument(
        "--smooth",
        type=int,
        default=1,
        help="moving-average width for differentiated signals",
    )
    p.add_argument("--rated-torque-nm", type=float)
    p.add_argument("--rotor-inertia-kgm2", type=float)
    p.add_argument("--rotation-distance-mm", type=float)
    args = p.parse_args(argv)

    header, data, drive_idx = load_capture(args.capture, args.drive)
    cpm = header["drives"][drive_idx]["counts_per_mm"]
    rot = args.rotation_distance_mm or header["drives"][drive_idx].get(
        "rotation_distance"
    )
    dt = header["cycle_ns"] * 1e-9

    if "accel_cmd" not in (data.dtype.names or ()):
        raise SystemExit("capture predates accel_cmd channel (need v2)")

    moving = (data["flags"] & FLAG_MOTION_ACTIVE) != 0
    t = np.arange(len(data)) * dt
    torque = data["torque_actual"].astype(np.float64)
    accel_cmd = data["accel_cmd"].astype(np.float64)
    vel_cmd = data["vel_cmd"].astype(np.float64)
    vel_act = data["velocity_actual"].astype(np.float64) / cpm
    pos_act = data["position_actual"].astype(np.float64) / cpm

    accel_vel_diff = smooth(np.gradient(vel_act, t), args.smooth)
    vel_from_pos = np.gradient(pos_act, t)
    accel_pos_ddiff = smooth(np.gradient(vel_from_pos, t), args.smooth)

    mask = steady_mask(accel_cmd, dt) & moving
    kept = int(mask.sum())
    print("capture: %s" % args.capture)
    print(
        "drive %r, counts_per_mm %.3f, dt %.6fs"
        % (header["drives"][drive_idx]["name"], cpm, dt)
    )
    print(
        "velocity check: mean |vel_actual| %.1f vs |vel_cmd| %.1f mm/s (should match)"
        % (np.mean(np.abs(vel_act[moving])), np.mean(np.abs(vel_cmd[moving])))
    )
    print("steady-plateau samples: %d/%d\n" % (kept, int(moving.sum())))

    sources = [
        ("cmd       (accel_cmd)", accel_cmd, vel_cmd),
        ("vel_diff  (d/dt 6063h)", accel_vel_diff, vel_act),
        ("pos_ddiff (d2/dt2 6064h)", accel_pos_ddiff, vel_from_pos),
    ]
    print(
        "%-26s %12s %12s %10s" % ("regressor", "mass M", "rms resid", "C00.06")
    )
    for name, accel, vel in sources:
        m, rms = scalar_fit(accel[mask], vel[mask], torque[mask])
        c = ""
        if args.rated_torque_nm and args.rotor_inertia_kgm2 and rot:
            c = "%.0f%%" % c0006(
                m, args.rated_torque_nm, rot, args.rotor_inertia_kgm2
            )
        print("%-26s %12.6g %12.3f %10s" % (name, m, rms, c))
    return 0


if __name__ == "__main__":
    main()
