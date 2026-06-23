#!/usr/bin/env python3
"""Two colinear straights: sweep the first-segment length so cruise is reached
just before / at / just after the move junction. Proves the a_t step is bound
to the CRUISE ONSET (v hits max_velocity), NOT the move boundary -- the three
a_t curves are identical and all step at the same s; the junction is transparent.

Run from the repo root after `make -f Makefile.rust motion-engine`:
    python3 _bmad-output/implementation-artifacts/investigations/cruise-onset-repro/cruise_boundary_sweep.py

Output: /tmp/viz_out/cruise_onset_overlay.png  + a printed step-location table.
"""

import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "klippy"))
sys.path.insert(0, str(ROOT))
import _motion_engine  # noqa: E402

MV, MA, JERK, SCV = (
    100.0,
    1000.0,
    4000.0,
    5.0,
)  # sane jerk so the ramp is visible


def plan(L1, L2):
    wps = [(0.0, 0.0, 0.0, MV), (L1, 0.0, 0.0, MV), (L1 + L2, 0.0, 0.0, MV)]
    sn = _motion_engine.pipeline_snapshot(wps, MV, MA, SCV, JERK, arc_fit=None)
    s = np.array(sn["kin_s"])
    v = np.array(sn["kin_v"])
    at = np.array(sn["kin_a_t"])
    m = np.concatenate([[True], np.diff(s) > 1e-9])
    return s[m], v[m], at[m]


def main():
    s, v, _ = plan(60, 60)
    d = s[int(np.argmax(v >= 99.99))]  # d_accel: rest -> cruise distance
    print(f"d_accel (rest->cruise) = {d:.3f} mm")
    cases = [
        ("cruise BEFORE junction", d + 1.5, "C0"),
        ("cruise AT junction", d, "C2"),
        ("cruise AFTER junction", d - 1.5, "C3"),
    ]

    fig, (ax, axv) = plt.subplots(
        2,
        1,
        figsize=(11, 7),
        sharex=True,
        gridspec_kw=dict(height_ratios=[2, 1]),
    )
    print("\ncase                     junction   a_t-step @   tracks")
    for name, L1, c in cases:
        s, v, at = plan(L1, 30.0)
        w = (s > d - 3.2) & (s < d + 2.5)
        ax.plot(s[w], at[w], c, lw=1.6, label=f"{name} (junction @ {L1:.1f})")
        ax.axvline(L1, color=c, ls="--", lw=1, alpha=0.6)
        axv.plot(s[w], v[w], c, lw=1.2)
        step = next(
            (
                s[i]
                for i in range(1, len(s))
                if at[i - 1] > 300 and at[i] < 300 and v[i] > 99.9
            ),
            float("nan"),
        )
        tracks = (
            "CRUISE ONSET"
            if abs(step - d) < abs(step - L1)
            else "junction(coincides)"
        )
        print(f"{name:24s}  {L1:6.2f}    {step:.3f}      {tracks}")
    ax.axvline(d, color="0.2", ls=":", lw=1.8)
    ax.axhline(0, color="0.7", lw=0.6)
    ax.set_ylabel("tangential a_t (mm/s²)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower left")
    ax.set_title(
        "Two colinear straights: a_t step lands on the CRUISE ONSET, not the junction\n"
        "dashed = each case's junction · dotted = cruise onset (same s in all 3; curves overlap)",
        fontsize=10,
    )
    axv.axvline(d, color="0.2", ls=":", lw=1.5)
    axv.set_ylabel("v (mm/s)")
    axv.set_xlabel("arclength s (mm)")
    axv.grid(alpha=0.3)
    Path("/tmp/viz_out").mkdir(exist_ok=True)
    out = "/tmp/viz_out/cruise_onset_overlay.png"
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print("\nwrote", out)


if __name__ == "__main__":
    main()
