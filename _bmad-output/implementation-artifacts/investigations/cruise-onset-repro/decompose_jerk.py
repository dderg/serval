#!/usr/bin/env python3
"""Decompose planner jerk into tangential (along-path) vs lateral (cross-path)
for any fixture, using the planner's ANALYTIC a_t (kin_a_t) -- not finite
differences of velocity (which is what scripts/viz_pipeline.py does, and which
aliases at near-stop seams).

Run from the repo root after `make -f Makefile.rust motion-engine`:
    python3 _bmad-output/implementation-artifacts/investigations/cruise-onset-repro/decompose_jerk.py [gcode]

Default fixture is /tmp/test2.gcode (fetch with:
    scp dderg@ethercatpi5.local:~/printer_data/gcodes/test2.gcode /tmp/test2.gcode).
If absent, falls back to a synthetic sharp corner.

Output: /tmp/viz_out/<name>_decomposed.png  +  a printed per-corner j_t/j_n table.

KEY RESULT (test2): every large jerk spike is TANGENTIAL (j_t); lateral j_n
(=k'*v^3, the G3 seam thing) is ~1e5, negligible. The big spikes are the
cruise on/off-ramp (a_t steps a_max->0 at v=max_velocity) -- the Motion-14 gap.
"""

import importlib.util
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[4]  # repo root (…/curvature-profile)
sys.path.insert(0, str(ROOT / "klippy"))
sys.path.insert(0, str(ROOT))
import _motion_engine  # noqa: E402

# Neptune test2 config.
MV, MA, JERK, SCV = 100.0, 1000.0, 1000000.0, 30.0


def waypoints(gcode: Path):
    if gcode.exists():
        sp = importlib.util.spec_from_file_location(
            "vp", ROOT / "scripts/viz_pipeline.py"
        )
        vp = importlib.util.module_from_spec(sp)
        sp.loader.exec_module(vp)
        return vp.parse_gcode(gcode, MV), gcode.stem
    # synthetic sharp >90-degree corner fallback
    return [
        (0.0, 0.0, 0.0, MV),
        (50.0, 0.0, 0.0, MV),
        (20.0, 50.0, 0.0, MV),
    ], "synthetic_corner"


def main():
    gcode = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/test2.gcode")
    wps, name = waypoints(gcode)
    snap = _motion_engine.pipeline_snapshot(
        wps, MV, MA, SCV, JERK, arc_fit=None
    )
    s = np.array(snap["kin_s"])
    v = np.array(snap["kin_v"])
    at = np.array(snap["kin_a_t"])
    kap = np.array(snap["kin_kappa"])
    mask = np.concatenate([[True], np.diff(s) > 1e-9])
    s, v, at, kap = s[mask], v[mask], at[mask], kap[mask]
    vs = np.maximum(v, 1e-6)
    ds = np.diff(s)
    t = np.concatenate([[0.0], np.cumsum(2 * ds / (vs[:-1] + vs[1:]))])
    a_n = v * v * kap
    jt = np.gradient(at, t)  # tangential jerk from analytic a_t
    jn = np.gradient(a_n, t)  # lateral/centripetal jerk

    fig, (av, aa, aj) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    av.plot(t, v, "C0")
    av.set_ylabel("v (mm/s)")
    av.grid(alpha=0.3)
    av.set_title(
        f"{name} — decomposed jerk (analytic a_t)  v{MV:.0f} a{MA:.0f} jerk{JERK:.0f} scv{SCV:.0f}"
    )
    aa.plot(t, at, "C3", lw=1, label="a_t (along-path, planner)")
    aa.plot(t, a_n, "C0", lw=1, label="a_n=κv² (cross-path)")
    aa.set_ylabel("accel mm/s²")
    aa.legend(fontsize=8)
    aa.grid(alpha=0.3)
    aj.plot(
        t,
        np.abs(jt),
        "C3",
        lw=1,
        label="|j_t| tangential (cruise on/off-ramp + apex)",
    )
    aj.plot(
        t, np.abs(jn), "C0", lw=1, label="|j_n| lateral κ'v³ (G3 → line seams)"
    )
    aj.set_ylabel("|jerk| mm/s³")
    aj.set_xlabel("time (s)")
    aj.legend(fontsize=8)
    aj.grid(alpha=0.3)
    aj.axhline(JERK, ls=":", c="0.5")
    aj.text(
        t[-1],
        JERK,
        " max_jerk",
        va="bottom",
        ha="right",
        fontsize=7,
        color="0.4",
    )
    Path("/tmp/viz_out").mkdir(exist_ok=True)
    out = f"/tmp/viz_out/{name}_decomposed.png"
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print("wrote", out)

    nz = np.where(np.abs(kap) > 1e-9)[0]
    if len(nz):
        for gi, g in enumerate(np.split(nz, np.where(np.diff(nz) > 1)[0] + 1)):
            apex = g[0] + int(np.argmax(np.abs(kap[g[0] : g[-1] + 1])))
            lo, hi = max(g[0] - 2, 0), min(g[-1] + 3, len(jt))
            print(
                f"corner{gi}: t=[{t[g[0]]:.3f},{t[g[-1]]:.3f}] apex v={v[apex]:.0f}  "
                f"peak|j_t|={np.abs(jt[lo:hi]).max():.2e}  peak|j_n|={np.abs(jn[lo:hi]).max():.2e}"
            )


if __name__ == "__main__":
    main()
