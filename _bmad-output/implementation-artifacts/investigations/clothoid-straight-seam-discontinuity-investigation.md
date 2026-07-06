# Investigation: Discontinuity between a clothoid and the straight right after it (demo 4 viz)

## Hand-off Brief

1. **What happened.** In the pipeline viz, the seam where a clothoid blend meets the straight segment right after it shows a discontinuity. **Confirmed (measured):** curvature `κ` is continuous (0→0, the blend is correctly G2), but the curvature *rate* `dκ/ds` steps from `±σ` to `0` at the seam, so the normal/lateral jerk `j_n_geom = κ'·v³` steps with it (measured −4.39e7 → 0 mm/s³ on a default 90° corner).
2. **Where the case stands.** Root cause Confirmed by code construction + a deterministic numeric probe (`rust/geometry/tests/zz_seam_discontinuity_probe.rs`). The cause is structural: a clothoid (Euler spiral) has curvature **linear** in arclength, so it is G2 (continuous κ) but not G3 (continuous κ'); κ' is a nonzero constant on the clothoid and 0 on the straight.
3. **What's needed next.** Decide the disposition: (a) accept — per spec-motion-12 lateral jerk is an explicit planner Non-Goal, owned by the fitter shape and report-only; or (b) raise the fitter's transition shape to a G3 (κ'-continuous) curve. No velocity-side fix is in scope (spec-motion-12 forbids a planner-level lateral-jerk cap).

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-20                                                                 |
| Status           | Active                                                                     |
| System           | curvature-profile worktree; Rust geometry + motion-engine; baseline 9ebf70d7a |
| Evidence sources | Rust source (geometry fitter/path, motion-engine viz/jerk_probe), spec-motion-12, numeric probe |

## Problem Statement

User (verbatim): "if you generate a viz for demo 4, or rather generate the data so you can read it better rather than looking at png, there's one issue which is discontinuity between clothoid and straight right after it."

## Evidence Inventory

| Source   | Status     | Notes     |
| -------- | ---------- | --------- |
| `rust/geometry/src/path/clothoid.rs` | Available | `kappa(s)=κ0+σs`, `dkappa_ds(s)=σ` (constant) — the linear-curvature definition. |
| `rust/geometry/src/fitter/biclothoid.rs` | Available | Symmetric corner blend; half1 κ:0→peak, half2 κ:peak→0. |
| `rust/geometry/src/fitter/chain.rs` | Available | Chain reconstruction; up/down clothoids bridge line↔arc; down ends at κ=0. |
| `rust/geometry/src/fitter.rs` | Available | `emit_move` trims the adjacent line to the seam; `emit_reconstruction`/`emit_blend`. |
| `rust/motion-engine/src/jerk_probe.rs` | Available | `j_n_geom = dkappa_ds·v³`, `j_n_couple = 2κ·v·a_t`. |
| `_bmad-output/.../spec-motion-12-tangential-jerk-c2-continuity.md` | Available | Non-Goals: lateral jerk is the fitter's shape, planner does not cap it, viz `j_n` is report-only. |
| Numeric probe `rust/geometry/tests/zz_seam_discontinuity_probe.rs` | Available | Reproduces the κ'/j_n step deterministically (created this investigation). |
| `demo 4` G-code | Missing | Lives on the bench (`~/printer_data/gcodes`), not in the repo. Probe substitutes a default 90° corner; the phenomenon is geometry-structural, independent of the specific path. |

## Timeline of Events

| Time        | Event               | Source                | Confidence |
| ----------- | ------------------- | --------------------- | ---------- |
| build time  | Fitter emits line→clothoid→clothoid→line; each clothoid carries constant σ | `fitter.rs`, `biclothoid.rs` | Confirmed |
| sample time | viz/probe samples κ, dκ/ds per segment; κ continuous, dκ/ds steps at clothoid↔line seams | `viz.rs::sample_kinematics`, probe | Confirmed |

## Confirmed Findings

### Finding 1: The clothoid is G2 but not G3 — κ' is a nonzero constant that steps to 0 at the straight

**Evidence:** `rust/geometry/src/path/clothoid.rs:61-67` — `kappa(s)=kappa_0+sigma*s`, `dkappa_ds(_s)=sigma`. A straight (`Line`) has `kappa≡0`, `dkappa_ds≡0`.

**Detail:** At a clothoid→line seam the blend is built so κ matches (both 0), but κ' = σ on the clothoid and 0 on the line. κ' therefore has a step discontinuity of magnitude σ. This is intrinsic to a linear-curvature spiral; it is the defining property of the Euler clothoid (curvature continuity, not curvature-rate continuity).

### Finding 2: The seam is correctly continuous in position, heading, and curvature — only κ' (and j_n) breaks

**Evidence:**
- Curvature: chain down-clothoid `Clothoid::try_new(a1, h1, n_exit, kappa_arc, -sigma, l_t)` with `sigma = kappa_arc / l_t` (`chain.rs:184,200`) → `kappa(l_t) = kappa_arc - sigma·l_t = 0`. Biclothoid half2 `sigma = kappa_peak/length` (`biclothoid.rs:40,47`) → `kappa(length)=0`.
- Position/heading: `chain.rs:212-216` rejects the fit unless `seam_ok` (≤1e-6 mm) **and** `dot(down.heading_at(l_t), tm) ≥ 1−1e-6`. The adjacent line is trimmed to the seam by `fitter.rs::emit_move` (`new_start/new_end` at the clothoid endpoint).

**Detail:** So the observed discontinuity is **not** a position gap or a heading kink and **not** a curvature jump — those are all enforced. It is specifically the curvature-rate / lateral-jerk step.

### Finding 3: j_n_geom = κ'·v³ carries the step; j_n_couple = 2κ·v·a_t does not

**Evidence:** `rust/motion-engine/src/jerk_probe.rs` — `j_n_geom = dkappa_ds·v³`, `j_n_couple = 2·kappa·v·a_t`. At the seam κ→0 so the couple term →0 continuously on both sides; the geom term steps from `σ·v³` to `0`.

**Measured (probe, default 90° corner, scv=5, accel=5000, jerk=1e5):**
```
seg[1] clothoid  kappa(0)=0          kappa(L)=+2.6868e2   dk(0)=+4.5955e4  dk(L)=+4.5955e4
seg[2] clothoid  kappa(0)=+2.6868e2  kappa(L)=0           dk(0)=-4.5955e4  dk(L)=-4.5955e4
--- seam clothoid -> line at s=20.00473 ---
  clothoid  kappa=0   dkappa_ds=-4.59554e4   j_n_geom=-4.38503e7
      line  kappa=0   dkappa_ds= 0.0         j_n_geom= 0.0
max |dkappa_ds step| across any seg boundary = 4.595535e4
```
The line→clothoid (entry) seam shows the mirror step (0 → +1.83e7). Both ends of the blend carry it; the user's "straight right after" is the exit seam.

## Deduced Conclusions

### Deduction 1: The discontinuity is structural to the transition primitive, not a fitter or planner bug

**Based on:** Findings 1–3.

**Reasoning:** Every seam invariant the code actually enforces (G0 position, G1 heading, G2 curvature) holds to tolerance. The only broken quantity, κ' (hence `j_n`), is exactly the one a linear-curvature clothoid cannot make continuous. No amount of tuning the existing fitter removes it — a clothoid's κ' is constant by definition.

**Conclusion:** To make `j_n` continuous across the clothoid↔straight seam you must change the *shape* (a κ'-continuous / G3 transition), or accept the step as the clothoid's designed behavior.

### Deduction 2: This is consistent with — and currently sanctioned by — spec-motion-12

**Based on:** spec-motion-12 Non-Goals (lines 52): "Lateral jerk `j_n = κ'·v³ + 2κ·v·a_t` is owned by the fitter's shape (clothoid geometry); the planner does **not** cap velocity for it... The viz probe MAY emit `j_n` as a free diagnostic... it is report-only — never a gate."

**Conclusion:** Under the frozen architecture the lateral-jerk step is an accepted property; the active C2 work (spec-12) targets **tangential** jerk only. A planner-side lateral-jerk cap is explicitly forbidden. So eliminating the seam step is a *fitter shape* question, not a velocity-planner one.

## Source Code Trace

| Element       | Detail                                      |
| ------------- | ------------------------------------------- |
| Discontinuity origin | `rust/geometry/src/path/clothoid.rs:65` `dkappa_ds` ≡ `sigma` (clothoid) vs `Line` `dkappa_ds` ≡ 0 |
| Trigger       | Any clothoid blend abutting a straight (corner blend exit/entry; chain up/down spirals at line ends) |
| Condition     | κ matches at the seam (0) but κ' steps (σ→0), so `j_n_geom = κ'·v³` steps |
| Related files | `fitter/biclothoid.rs`, `fitter/chain.rs`, `fitter.rs::emit_move`, `motion-engine/src/jerk_probe.rs`, `motion-engine/src/viz.rs::sample_kinematics` |

## Conclusion

**Confidence:** High

The "discontinuity between the clothoid and the straight right after it" is a **curvature-rate (G3) discontinuity**, not a position, heading, or curvature break. The blend is correctly G2 — κ returns to 0 at the seam — but a clothoid's curvature is linear in arclength, so its κ' is a nonzero constant (`σ`) that steps to the straight's 0. The lateral/normal jerk `j_n_geom = κ'·v³` inherits the step (measured −4.39e7 → 0 mm/s³ at a default 90° corner; mirror step at the entry seam). This is intrinsic to using a single Euler clothoid as the transition primitive, and is consistent with spec-motion-12, which treats lateral jerk as the fitter shape's responsibility and report-only on the planner side.

## Recommended Next Steps

### Fix direction (by mechanism — disposition is a design decision)

1. **Accept / document (matches current architecture).** spec-motion-12 already classifies lateral jerk as a fitter-owned, report-only quantity and the clothoid *bounds* its magnitude. If the step magnitude is acceptable for the hardware, no code change — only a note that clothoid blends are G2, not G3, and `j_n` will step at every clothoid↔{line,arc} seam.
2. **Raise the transition shape to G3 (κ'-continuous).** Replace the linear-σ clothoid with a curvature profile whose κ' tapers to 0 at both endpoints (e.g. a smoothstep/quintic curvature ramp, or a higher-order spiral). This makes `j_n_geom` continuous at the seam. Cost: longer/blunter transitions, more complex (Fresnel-like) evaluation, and a re-derivation of the seam-closure math in `biclothoid.rs`/`chain.rs`. This is the geometric-SOTA route and a substantial fitter change — scope it as its own spec.
3. **(Out of scope) Planner-side velocity cap for j_n** — explicitly forbidden by spec-motion-12 Non-Goals; do not pursue without renegotiating the frozen intent.

### Diagnostic
- The probe `rust/geometry/tests/zz_seam_discontinuity_probe.rs` reproduces the step deterministically and prints the per-seam table; point it at demo-4-like geometry (gentler corners → longer clothoids → lower σ but the step persists) to confirm magnitude on representative input.

## Reproduction Plan

`cargo nextest run -p geometry -E 'test(probe_clothoid_straight_seam_dkappa_step)' --no-capture` from `rust/`. Builds a 90° corner, runs `fit_chain` + `plan_velocity`, prints κ / dκ/ds / j_n_geom across every clothoid↔line seam, and reports `max |dkappa_ds step|`.

## Side Findings

- **Velocity sample mismatch at the seam.** In the probe the clothoid's last sample carried `v≈9.845` while the adjacent line's first sample carried `v≈7.358` at the same `s`. **This is the real reported defect (see Follow-up), not the κ' step above.**
- The same κ' step occurs at **clothoid↔arc** seams inside chain runs (up-spiral→arc, arc→down-spiral); there κ is also continuous but κ' steps σ→0.

## Follow-up: 2026-06-20 — premise corrected by user; real defect is a tangential-accel spike from the uncommitted jerk-bridge

### New Evidence

The user clarified (with viz): the issue is **tangential acceleration** `a = v·dv/ds` spiking right after every clothoid (~310 → **4.8e6 mm/s²** with the uncommitted changes), while velocity only shows a tiny bump over a sub-micron distance. The uncommitted `velocity/{disk,scurve}.rs` + `velocity.rs` changes are a prior agent's 2-hour attempt at **spec-motion-12 T3** (coupled `(v,a)` C2 sweep + crossover jerk-bridge); they made the spike *worse*. This **supersedes** the original lateral-jerk (κ') framing — that κ' step is real but is **not** what the user is seeing.

### Additional Findings (Confirmed — measured, baseline vs current)

Probe `rust/geometry/tests/zz_seam_discontinuity_probe.rs`, three corner geometries, `a_t = v·dv/ds` reconstructed exactly as `viz_pipeline.py` does (np.gradient):

| geometry | committed baseline | current (uncommitted fix) |
| --- | --- | --- |
| tight 90° / slow | `max|a_t|`=5001≈accel, v_overshoot **0.000** | `max|a_t|`=**1.15e6**, overshoot 2.48 |
| gentle 18° / fast | 5001≈accel, overshoot **0.000** | **2.21e5**, overshoot 2.46 |
| mid 45° | 5002≈accel, overshoot **0.000** | **7.68e5**, overshoot 2.48 |

- **F4 (Confirmed):** the committed baseline has **no seam spike** — tangential accel rides exactly at `max_accel`, zero velocity overshoot, at every clothoid↔line seam. The spike is **entirely introduced by the uncommitted jerk-bridge**.
- **F5 (Confirmed):** on the current tree a single sample is wedged at the clothoid tail with `v=9.845` between neighbours of ~7.36, at Δs≈9.16e-7 mm (`rust/geometry/src/velocity/disk.rs` bridge sampling). The ~2.48 mm/s overshoot is remarkably constant across geometries → systematic, not noise.
- **F6 (Confirmed):** acceleration is reconstructed by **centered finite difference** `a = v·Δv/Δs` in `disk.rs::centered_fd_accels`/`sv_to_sva_fd` (the planner's stored `sample.a`), and again by `np.gradient` in viz. Over a sub-micron seam Δs, a ~2.48 mm/s velocity error becomes 1e5–1e6 mm/s². Both layers amplify the same overshoot.

### Root cause (two compounding faults in the uncommitted T3 work)

1. **Velocity-envelope violation by the bridge.** `disk.rs::find_bridge` fires on the ultra-short corner-blend clothoid (~0.006 mm) and `sample_bridge_uniform_t`/`refine_bridge_time` emit samples whose velocity follows the time-domain jerk polynomial `v_l + a_l·t − ½·j·t²` with `a_l,a_r` clamped to ±accel. Over a sub-micron arclength this **overshoots** `min(fwd,bwd)` (the profile must satisfy `v ≤ envelope` everywhere; the bridge breaks that), planting `v=9.845` where neighbours are ~7.36.
2. **Acceleration by finite-difference over degenerate Δs — the exact failure spec-motion-12 forbade.** The implementation derives `a_t` via `centered_fd_accels` (`v·Δv/Δs`) instead of the analytic seven-segment `accel_at` the spec mandates (T1 / **AC-G6**: "evaluate `accel_at` at every adjacent `SevenSeg` endpoint pair directly — no resampling — a step at a narrow bridge cannot fall between samples"; **R4** aliasing). Finite-differencing the overshoot over Δs≈1e-6 mm is what turns a 2.5 mm/s bump into a 1e6 spike.

### Updated Conclusion

**Confidence:** High. The acceleration spike "right after every clothoid" is **not** present in the committed code and is **not** an inherent geometry property — it is an artifact of the in-progress spec-motion-12 T3 jerk-bridge: the bridge plants a velocity overshoot on the sub-millimetre corner clothoid, and acceleration is then recovered by finite-difference over a sub-micron seam Δs (in both the planner's `sample.a` and the viz). Successive iterations of that work amplified it (≈310 → 4.8e6), matching the user's "made the spike even higher." The original lateral-jerk (κ') finding stands as a *separate, smaller, by-design* property (spec-12 Non-Goal), not the reported defect.

### Fix direction

- **Immediate:** the spike is in unmerged WIP; it is not a regression in `main`/committed code. If the T3 work is paused, the baseline is clean.
- **For the T3 implementation:** (a) the bridge must never emit `v > min(fwd_envelope, bwd_envelope)` at any sample — clamp/validate against the envelope (the existing `v_r_capped` clamp is applied only at `s_r`, not to the interior bridge samples); (b) replace the finite-difference `centered_fd_accels`/`sv_to_sva_fd` with the analytic `accel_at` evaluated at segment endpoints (spec AC-G6), so `a_t` cannot alias on a narrow bridge; (c) gate the bridge so it does not fire on sub-`ds_min` corner-blend clothoids where there is no genuine fwd/bwd tangential crossover. This is also exactly what the new `c2_feasibility_gate.rs` / `zz_independent_probe.rs` should catch — they currently assert `|Δa|` bounds but the spike still ships, so the gate's resampling is aliasing past it (R4).

### Backlog Changes

- Demo-4 geometry not needed to diagnose — reproduced synthetically. The committed-baseline ~310 the user saw on demo 4 was **not** reproduced from `9ebf70d7a` (synthetic baseline is clean); it was most likely an earlier point in the agent's bridge iterations, consistent with iterative amplification to 4.8e6.

## Follow-up: 2026-06-20 #2 — premise fully resolved; case concluded, work handed to spec-motion-12 T3

### Resolution of the reported symptom
Recovered the real fixture: `/private/tmp/demo4.gcode` + `/private/tmp/viz_demo.cfg` (corexy, `max_velocity 150`, `max_accel 200`, `max_jerk 4000`, `scv 5`). Reproduced exactly (traversal 5.583s vs the plot's 5.588s). Findings:
- The **310 accel / 7e9 jerk spikes "right after every clothoid"** in the user's 16:12 PNG were the **previous agent's broken-T3 build** (bridge overshoot firing on the corner clothoids). On the committed build they are **gone**: accel rides clean at `max_accel`, and `a_t` is continuous ±`max_accel` at every clothoid↔straight boundary (measured, all 6 seams). The clothoid→straight boundary itself was never the defect — it was a stale plot.
- **What is genuinely wrong in committed code (the real, tangential defect):** (1) accel-from-rest rides at exactly **`(2/9)·max_jerk`** (`v_ceil(s)=(jerk·s²)^(1/3)` ⇒ `da_t/dt=(2/9)·jerk`; measured 889 vs 4000) — under-driven, not a trapezoid; (2) the sub-cruise accel→decel **crossover steps `+max→−max`** (ungoverned jerk). Same root cause: `a_t` is derived from a velocity ceiling, not carried as state.

### Status
**Concluded.** This is not a seam/geometry bug — it is the C1 tangential-jerk model, i.e. exactly **spec-motion-12 T3**. Full T3 pickup context (committed T1/T2, the `(2/9)·jerk` proof, prior-attempt post-mortem, demo4 fixture, design, first steps) is written to the **Dev Log** at the foot of `spec-motion-12-tangential-jerk-c2-continuity.md`. Start the fresh session there.
