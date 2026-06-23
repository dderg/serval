# Handoff: cruise on/off-ramp tangential-jerk gap (a_t steps a_max→0 at max_velocity)

**Date:** 2026-06-21 · **Author handoff for:** fresh-context pickup
**Base:** `sota-motion` (PR #88 merged — `f432e585d`). Worktree branch this was found on: `curvature-profile`.

## TL;DR

The velocity planner jerk-limits the **start** of acceleration (jerk-up from rest) but **not the end** of it. When `v` reaches `max_velocity` and the move goes to cruise, tangential acceleration `a_t` **steps discontinuously from `a_max` (or whatever accel it had) to `0`** — an unbounded tangential jerk at every cruise entry. The mirror happens leaving cruise into a decel (`0 → −a_max`). This is a **C1 violation on the flat velocity ceiling**, distinct from everything T3 fixed.

It hits **every move that reaches `max_velocity`**, not just corners. It was never exercised by the `demo4` fixture (whose peaks were sub-cruise, `134 < 150`), which is why T3 shipped without catching it. The Neptune `test2.gcode` fixture exposes it (reaches cruise coming out of each corner).

This is the **next piece of motion work** — provisionally **Motion-14**. It is tangential and fixable; it is **not** the lateral/G3 clothoid issue (that remains a deliberate Non-Goal).

## What is already done and merged (PR #88 = spec-motion-12 T3 + clamp)

On `sota-motion`:
- **C2-within-run tangential continuity** (`feat(velocity): C2 tangential-jerk continuity…`, `4a1def6bb`). Per-run reconstruction carrying analytic `(v,a)`; binding-rail analytic `a_t` (scurve `accel_at` / disk rail / curvature-ceiling tracking), never finite-differenced; crossover jerk-bridge for apex (`+→−`) and valley (`−→+`) crossovers including across move boundaries; `(0,0)` pinned at rest anchors with fail-loud `RestAnchorAccel`; node sweep jerk term switched from `max_reachable_velocity` (the `(j·s²)^⅓` accel-returns-to-0 ceiling that caused the C1 `(2/9)·jerk` from-rest ride) to carried `reach_velocity_with_accel`.
- **Disk clamp** (`fix(velocity): clamp reported tangential accel to the acceleration disk`, `24dd8679d`): `a_t` bound to `±√(a_max²−a_n²)` so the reported tangential accel stays disk-feasible (killed the biclothoid-apex `|a|=210>a_max` over-shoot).
- Tests: `rust/geometry/tests/c2_continuity.rs`, `pin_rest_anchor_*` unit tests.

**Spec status:** `spec-motion-12-tangential-jerk-c2-continuity.md` T1–T3 done (T4 — CI feasibility gate + throughput non-regression — still NOT done, separate). `spec-motion-13` (delete dead SOCP) still open/separate.

## The gap to pick up (Motion-14)

`a_t` is **not** jerk-limited at the flat velocity ceiling:
- **Cruise entry** (`v → max_velocity`): `a_t` steps `a_max → 0`.
- **Cruise exit** (cruise → decel): `a_t` steps `0 → −a_max`.

### Evidence (all reproduced locally, host-only, no bench)

1. **It's tangential, not lateral/G3.** Decomposing the jerk on `test2`: every large spike is `j_t` (along-path); lateral `j_n = κ'·v³` is `~1e5`, negligible next to the `~1e10` tangential spikes.
2. **At the worst spike, `a_t` literally steps:** `+1000 → 0` over `~10 nm` of arclength (`dt ≈ 23 ns`) at `v = 100 = max_velocity`.
3. **Structural, not the jerk setting.** Reproduced at both `max_jerk = 1e6` (Neptune config) **and** `max_jerk = 4000`: `a_t` steps `894 → 0` at `v=100` in one sample either way. (With lower jerk the jerk-up hasn't even reached `a_max` before hitting the ceiling, so it steps from `894`, the value `√(2·j·v_max)`.)
4. **Bound to the cruise onset, not any move boundary.** The colinear-straights test (below) puts the cruise onset just before / at / just after a move junction: the `a_t` step stays glued to the cruise onset (`s = d_accel`) in all three cases, and the collinear junction is fully transparent (`a_t` continuous across it). So the earlier "couldn't apply the jerk limit at the boundary" framing is wrong — it's the ceiling, not the boundary.
5. **The asymmetry, visualized:** `a_t` ramps **up** jerk-limited (slope = jerk) then **drops vertically** at cruise — jerk-up limited, jerk-down a step.

## Root cause in code

The forward envelope primitive only models **jerk-up + hold-accel**, never a **jerk-down to land on a target velocity with `a=0`**:

- `rust/geometry/src/velocity/scurve.rs` — `reach_velocity_with_accel`, `breakpoints` (builds a `SevenSeg` that is jerk-up then hold-accel; `accel_at` therefore stays at `a_max` and never ramps back to 0), `velocity_at`, `accel_at`.
- `rust/geometry/src/velocity/disk.rs` — `forward_branch` / `backward_branch`: when the binding value hits the flat ceiling (`v ≈ kin.flat_ceiling`) the branch returns `a = 0` (cruise), so at the sample where the scurve value crosses the flat ceiling, `a` jumps `accel_at(≈a_max) → 0`. `eval_profile` selects the min. The crossover bridge (`build_run_bridge` / `reconstruct_flat`) only fires on **sign flips** (`+→−` apex, `−→+` valley) — the cruise transition is `+a → 0`, not a sign flip, so it is never bridged.
- `rust/geometry/src/velocity.rs` — `plan_velocity_warm_start` (node sweep + per-run reconstruction driver).

## Fix direction

Make the forward profile **jerk-down onto the flat ceiling** so `a_t` eases to `0` as `v → max_velocity`, mirroring the jerk-up it already does (and the symmetric jerk-up off the ceiling into a decel). Two plausible shapes:

- **Extend the scurve primitive** to a true reach-and-cruise seven-segment (jerk-up, hold, **jerk-down to a=0 at v_target**), so `breakpoints`/`accel_at` produce the trailing jerk-down. The flat ceiling becomes a velocity target the forward envelope lands on with `a=0`.
- **Treat the flat ceiling as a constant-v rail and bridge the `+a → 0` crossover**, extending `build_run_bridge` to also catch transitions *to/from* `a=0` at the ceiling (not only sign flips). The existing jerk-bridge math (constant-jerk arc, 1-D splice root) already does the `a`-matching; it just needs to accept `a_right = 0`.

The first is cleaner and matches the architecture (the envelope should know how to cruise). Either way the velocity profile near cruise gets a slightly earlier roll-off (you reach cruise a hair later) — a legitimate, tiny C2 cost.

**Acceptance (proposed):** `|j_t| ≤ max_jerk·(1+ε)` at cruise entry/exit; `|Δa_t|` bounded across the cruise on/off-ramp; new `c2_continuity` case on a path that *reaches* `max_velocity` (the colinear-straights fixture below) asserting no `a_t` step at the cruise onset. Note this also wants the same time-domain check T4 (spec-motion-12) owes.

## Reproduction (host-only, no bench needed)

Build the PyO3 module first: `make -f Makefile.rust motion-engine`.

**Runnable viz scripts** live next to this doc in `investigations/cruise-onset-repro/` (run from the repo root; both write PNGs to `/tmp/viz_out/`):

- **`decompose_jerk.py [gcode]`** — splits planner jerk into tangential `j_t` vs lateral `j_n` over a fixture, plotting `v / a_t,a_n / |j_t|,|j_n|` and printing a per-corner table. Defaults to `/tmp/test2.gcode` (synthetic sharp corner if absent). This is the plot that proves the spikes are tangential (`j_t ~1e6+`), not lateral (`j_n ~1e5`). → `*_decomposed.png`.
- **`cruise_boundary_sweep.py`** — the two-colinear-straights sweep; overlays the three cases (cruise before/at/after the junction) and prints the step-location table proving the `a_t` step tracks the cruise onset, not the junction. → `cruise_onset_overlay.png`.

These use the planner's **analytic** `kin_a_t` from `pipeline_snapshot`, not finite differences of velocity.

### Neptune `test2.gcode` (the original report)

- Config (Neptune `printer.cfg`): `max_velocity=100, max_accel=1000, max_jerk=1000000, square_corner_velocity=30`.
- Parsed waypoints (note: file is **relative** moves): `(0,0)→(-20,0)→(-30,-40)→(20,-30)→(20,0)`. Corners: 76°, **115°**, 79°. The "spike right after the clothoid" is the cruise onset on the straight ~3.6 mm past the corner-exit seam.
- Fetch it: `scp dderg@ethercatpi5.local:~/printer_data/gcodes/test2.gcode /tmp/test2.gcode` (neptune-bench skill has the host).

### Minimal repro — two colinear straights, cruise vs junction sweep

`v100 a1000 jerk4000`. `d_accel ≈ 7.45 mm` (rest→cruise). Build `(0,0)→(L1,0)→(L1+30,0)` for `L1 ∈ {d_accel+1.5, d_accel, d_accel−1.5}`; the `a_t` step stays at `s=7.45` in all three (junction transparent). Driver pattern:

```python
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0,"klippy"); sys.path.insert(0,"."); import _motion_engine
def plan(L1,L2,J=4000.0):
    wps=[(0.,0.,0.,100.),(L1,0.,0.,100.),(L1+L2,0.,0.,100.)]
    sn=_motion_engine.pipeline_snapshot(wps,100.,1000.,5.,J,arc_fit=None)
    s=np.array(sn["kin_s"]);v=np.array(sn["kin_v"]);a=np.array(sn["kin_a_t"])
    m=np.concatenate([[True],np.diff(s)>1e-9]); return s[m],v[m],a[m]
# a_t steps a_max->0 where v first hits 100, independent of L1.
```

The snapshot dict exposes the **analytic** `kin_s/kin_v/kin_a_t/kin_kappa/kin_dkappa_ds` — use these directly; do NOT finite-difference `kin_v` (that's what `scripts/viz_pipeline.py` does and it aliases at seams).

Saved plots from the investigation (regenerate as needed): `/tmp/viz_out/` — `test2_decomposed.png` (tangential vs lateral), `cruise_onset_overlay.png` (the 3-case overlay, junction transparent), `biclothoid_clamped.png` (the disk-clamp result).

## Related, separate, NOT this work

- **Corner-apex near-stop valley** (velocity minimum): tangential `a_t` crosses 0 over a sub-micron span → high `j_t`, but on a negligible (`±few mm/s²`) accel. The valley bridge currently *skips* the ceiling-touch near-stop case. Tiny; fold in with Motion-14 if convenient (same jerk-bridge family), or defer.
- **Lateral jerk `j_n = κ'·v³` at clothoid↔{line,arc} seams**: the clothoid is G2 not G3, so `κ'` steps. This is the **fitter shape's** responsibility and an explicit spec-motion-12 Non-Goal — do **not** add a planner-side lateral cap. Eliminating it is a G3 corner-shape spec (see `investigations/clothoid-straight-seam-discontinuity-investigation.md`). User has accepted leaving it.
- **`scripts/viz_pipeline.py` reconstructs accel/jerk by `np.gradient` of velocity** and ignores the analytic `kin_a_t/kin_j_t` the planner already emits (T2 wired them). It also divides by tiny `dt` at near-stop seams (huge artifact spikes). Wiring it to plot the analytic tracks would de-alias the panels (the tangential would be clean; lateral `j_n` would still honestly step). Nice-to-have.
- **spec-motion-12 T4** (CI feasibility gate + throughput non-regression) still owed.

## Key numbers / facts

- `d_accel` (rest→cruise, `v100 a1000 j4000`) ≈ **7.45 mm**; cruise reached at accel `√(2·j·v_max) ≈ 894 mm/s²` (jerk-up doesn't reach `a_max=1000` because `v_max=100 < a_max²/(2j)=125`).
- demo4 fixture (`v150 a200 j4000 scv5`) peaks at `v≈134 < 150` → sub-cruise → never hits this path.
- Decomposition rule: path-frame `j_t = d(a_t)/dt` (the spikes here), `j_n = d(κv²)/dt = κ'v³ + 2κv·a_t` (lateral, the G3 seam thing).
