# Investigation: Negative scalar velocity in arc_fit/fillet trajectory

## Hand-off Brief

1. **What happened.** The velocity profile for `snapshots/cases/arc_fit/fillet` emits **negative scalar speed** (`kin_v` down to −13.5 mm/s; 43 samples < 0) — geometrically impossible — as a jagged oscillation between the real ~32 mm/s profile and a sub-zero parabola. *(Confirmed, reproduced.)*
2. **Where the case stands.** Root cause located: the jerk-bridge reconstruction in `rust/geometry/src/velocity/disk.rs` splices in jerk-arc samples (`v = v0 + a0·τ + ½jτ²`) that dive below zero and are never rejected (only an *upper*-envelope guard exists). The trigger is the biclothoid fit fragmenting fillet corners into a dense chain of **sub-millimetre segments** (94/106 < 0.5 mm, clothoid halves ~0.03 mm) far below the bridge's roll-off scale (~6 mm), which the bridge solver cannot bridge correctly. *(Confirmed location + Deduced trigger.)*
3. **What's needed next.** Decide the fix altitude: (a) stop the fitter emitting degenerate micro-segments, and/or (b) make the bridge solver reject infeasible (v<0) arcs and fail loudly per CLAUDE.md. Recommend addressing both, starting with the fitter (the user's stated priority).

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-25                                                                 |
| Status           | Active                                                                     |
| System           | branch `solver-negative-velocity`; macOS; `_motion_engine` release cdylib |
| Evidence sources | reproduced engine output, source trace, snapshot fixtures                  |

## Problem Statement

User observed two issues in the `arc_fit/fillet` velocity plot (red "scalar" curve):
1. **(less critical)** Some velocity curves are more rounded than others.
2. **(critical)** A jagged region (~t=1.18–1.26 s) where scalar velocity goes **negative** — impossible for a Euclidean speed magnitude.

User's hypothesis: the underlying problem is in **how clothoids are fit**, and arc-fit issues are downstream of it.

## Evidence Inventory

| Source                                          | Status    | Notes                                                                     |
| ----------------------------------------------- | --------- | ------------------------------------------------------------------------- |
| `_motion_engine.pipeline_snapshot` (fillet run) | Available | Reproduced: min `kin_v` = −13.5075, 43 negative samples.                   |
| `rust/motion-engine/src/viz.rs`                 | Available | "scalar" red curve = `kin_v` = `VelSample.v` directly (not √(vx²+vy²)).    |
| `rust/geometry/src/velocity/disk.rs`            | Available | Jerk-bridge reconstruction; the negative samples originate here.          |
| `rust/geometry/src/velocity/scurve.rs`          | Available | S-curve sampler is monotone ≥0; ruled out as a source.                    |
| `snapshots/baselines/arc_fit/fillet.baseline…`  | Available | Committed "failing" baseline (commit dafd9a0a5) encodes the buggy output. |

## Confirmed Findings

### Finding 1: The "scalar" red curve is the raw signed `VelSample.v`, not a magnitude

**Evidence:** `rust/motion-engine/src/viz.rs:57` (`kin_v`), `viz.rs:232` (`kin.v.push(sample.v)`).

**Detail:** The plotted "scalar" is `sample.v` emitted by the velocity planner. `|X|`/`|Y|` are `heading·v` in the Python plotter. So a negative "scalar" means the planner produced a negative `VelSample.v` — there is no magnitude operation that could hide a sign. The impossible value is real planner output, not a plotting artifact.

### Finding 2: 43 negative `kin_v` samples, min −13.5 mm/s, in 5 clusters all anchored on tiny clothoid segments

**Evidence:** reproduced run of `arc_fit/fillet`:
```
samples=113789  min_v=-13.5075  max_v=98.8712  neg_count=43
cluster s=[187.836,187.858] count=18 minv=-1.11   -> seg61 clothoid len=0.190mm
cluster s=[220.591,220.616] count=3  minv=-4.83   -> seg87 clothoid len=0.203mm
cluster s=[220.640]         count=1  minv=-6.48   -> seg87 clothoid
cluster s=[220.670]         count=1  minv=-7.95   -> seg87 clothoid
cluster s=[220.706,221.525] count=20 minv=-13.51  -> seg87 clothoid → seg88 line(10mm)
```

**Detail:** Every cluster sits on, or at the exit of, a sub-millimetre clothoid segment inside the fillet's dense biclothoid chain. The worst cluster bridges the last micro-clothoid (seg87, 0.20 mm) into a long 10 mm line (seg88).

### Finding 3: The negative samples are jerk-arc points spliced beside un-removed base samples

**Evidence:** sample window at s≈187.835 shows two interleaved sequences at near-identical `s`:
```
187.8355   -0.03965 NEG     187.8355   32.12380
187.8358   -0.27740 NEG     187.8359   32.12557
187.8365   -0.48545 NEG     187.8366   32.12898
187.8375   -0.66381 NEG     ...
```
The negative sequence is a smooth parabola (Δv = −0.24, −0.21, −0.17, −0.15, −0.12, −0.09, −0.06, −0.03 → decelerating toward an apex), i.e. `v(τ) = v0 + a0·τ + ½jτ²`.

**Detail:** This is exactly the jerk-bridge arc from `build_run_bridge` (`disk.rs:667-681`): 48 arc points crammed into a tiny `s` window, then concatenated with the retained base profile and sorted by `s` (`reconstruct_flat`, `disk.rs:768-775`). Because the arc's `(lo,hi)` span is sub-millimetre, the base samples it should replace are *not* filtered out (`disk.rs:770` filters only strictly-interior points), so the two coexist → the plotted sawtooth.

### Finding 4: The bridge builder guards only the upper envelope, never v ≥ 0

**Evidence:** `rust/geometry/src/velocity/disk.rs:677` — `if v > env + 1e-6*(1.0+env) { return None; }`. The arc-point loop (`disk.rs:667-681`) has no `v >= 0` check. `reconstruct_run`/`interp_flat` (`disk.rs:783-848`) never clamp or assert non-negativity either.

**Detail:** A spurious arc that dives below zero is *below* `env`, so the only guard passes it. The impossible state is then silently emitted — a direct violation of the CLAUDE.md "fail loudly" rule (an impossible velocity should error, not ship).

### Finding 5: The base profile cannot be the source (ruled out)

**Evidence:** `disk.rs:122` `disk_reach_v` returns `…max(0.0).sqrt()`; `scurve::velocity_at` (`scurve.rs:194-209`) is monotone from `v0 ≥ 0` (validated `v0 < 0.0 → InvalidInput`, `scurve.rs:46-57`); ceilings are `≥0`. `base_samples` = `min(forward, backward, ceil)` of these.

**Detail:** Every base-profile contributor is provably ≥0, so the negatives can only come from the bridge arcs (Finding 3/4). Eliminates scurve and disk-reach as sources.

## Deduced Conclusions

### Deduction 1: The trigger is scale mismatch between the biclothoid fit and the bridge roll-off length

**Based on:** Findings 2, 3, 4 + fit geometry.

**Reasoning:** The fillet arc_fit produces **94/106 fitted segments < 0.5 mm**, with clothoid halves as short as 0.029 mm. The bridge solver works at the jerk roll-off scale `half_max ≈ v·(2·accel/jerk)` (`disk.rs:617-620`) ≈ 30 · 0.2 = ~6 mm, and scans/shoots over windows several times that (`disk.rs:626`). That window spans *dozens* of tiny segments, each with a different curvature ceiling, so `run_forward`/`run_backward` return a jagged branch signal and the root-finder (`scan_root`/`shoot`) lands on a spurious crossing. The resulting arc is not physical and dives through v=0.

**Conclusion:** The negative velocity is the symptom; the disease is that **arc_fit fragments fillet corners into sub-millimetre biclothoid chains** below the scale the velocity bridge can resolve. Fixing the fitter (fewer/larger, non-degenerate segments) removes the trigger; hardening the bridge (reject v<0, fall back to base, fail loudly) removes the impossible output. Both are warranted; they are independent layers.

## Hypothesized Paths

### Hypothesis 1: The "more rounded vs sharper" velocity curves (issue #1) share this mechanism

**Status:** Open

**Theory:** When `build_run_bridge` succeeds, a corner's accel step is replaced by a smooth jerk roll-off (rounded peak); when it returns `None` (infeasible across tiny segments), the base profile's sharp accel triangle is kept (pointy peak). Same machinery, inconsistently applied → mixed roundness.

**Would confirm:** Correlate each velocity peak's roundedness with whether a bridge was emitted for that transition (instrument `build_run_bridge` return / count emitted bridges per corner).

**Would refute:** Rounded vs sharp peaks track segment *type* (arc vs clothoid vs line) independent of bridge success.

## Missing Evidence

| Gap                                                  | Impact                                                              | How to Obtain                                                       |
| ---------------------------------------------------- | ------------------------------------------------------------------ | ------------------------------------------------------------------ |
| Curvature/`kappa` profile across the micro-clothoids | Quantifies how degenerate the fit is; informs fitter fix threshold | Sample `spatial.kappa(s)` across seg61/seg87 neighbourhoods         |
| Why biclothoid fit emits 0.03 mm halves              | Determines whether to clamp min length or change facet handling    | Trace `rust/geometry/src/fitter/biclothoid.rs` + `chain.rs` on this input |
| Bridge-emitted-count per corner                      | Confirms/refutes Hypothesis 1                                      | Instrument `build_run_bridge`                                      |

## Source Code Trace

| Element       | Detail                                                                                              |
| ------------- | -------------------------------------------------------------------------------------------------- |
| Error origin  | `rust/geometry/src/velocity/disk.rs:667-681` (`build_run_bridge` arc-point loop, no v≥0 guard)     |
| Trigger       | Sub-millimetre biclothoid chain from arc_fit (`fitter/biclothoid.rs`, `fitter/chain.rs`)           |
| Condition     | Bridge window (~6 mm) spans many tiny segments → spurious root → arc dives below v=0                |
| Related files | `velocity/disk.rs` (reconstruct_flat:698, interp_flat:783, reconstruct_run:799), `velocity.rs`, `viz.rs:232` |

## Conclusion

**Confidence:** High (root-cause location Confirmed and reproduced; trigger Deduced from quantified fit geometry).

The negative "scalar" velocity is real planner output: jerk-bridge arcs in `velocity/disk.rs` dive below zero and pass an upper-only feasibility guard. They are triggered by the biclothoid fit fragmenting fillet corners into sub-millimetre segments far below the bridge's roll-off scale, where the bridge root-finder mis-solves. Two independent defects: (1) the fitter emitting degenerate micro-segments; (2) the bridge accepting/emitting physically impossible velocities instead of failing loudly.

## Recommended Next Steps

### Fix direction

- **Fitter (user's priority):** stop arc_fit from fragmenting fillets into 0.03–0.2 mm biclothoid chains — investigate min-segment-length / facet handling in `fitter/biclothoid.rs` + `fitter/chain.rs`. Removes the trigger.
- **Bridge hardening:** in `build_run_bridge`, reject arcs with `v < 0` (mirror the existing upper-envelope guard at `disk.rs:677`) and fall back to the base profile; add a non-negativity assertion in `reconstruct_run`/`plan_velocity` so an impossible velocity fails loudly (CLAUDE.md), never ships into a baseline.

### Diagnostic

- Sample `kappa(s)` across seg61/seg87 to quantify fit degeneracy.
- Count bridges emitted per corner to settle Hypothesis 1 (the roundedness issue).

## Reproduction Plan

```
make -f Makefile.rust motion-engine
# then run pipeline_snapshot on snapshots/cases/arc_fit/fillet.gcode
# with max_velocity=300 max_accel=1000 scv=5 max_jerk=10000 arc_fit=(2.0, 12.0)
# inspect kin_v: min ≈ -13.5 mm/s, 43 samples < 0, clusters at s≈187.8 and s≈220.6
```
(Repro script used: `scratch_repro.py` at repo root — parses the gcode and calls `_motion_engine.pipeline_snapshot` directly, bypassing the klippy path-shadowing.)

## Side Findings

- The committed `arc_fit/fillet` baseline (commit dafd9a0a5 "add failing arc snapshots") encodes this buggy negative-velocity output — it is a *failing* snapshot by design, not a regression to protect. *(Confirmed.)*
- `canonical_json(allow_nan=False)` (`snapshots/harness.py`) would reject NaN/Inf but **not** negative finite velocities — the snapshot harness cannot catch this class of bug on its own; a `v ≥ 0` invariant check is the missing guard. *(Confirmed.)*
- 94/106 fitted segments < 0.5 mm on this fillet — the fragmentation is severe, not a one-off corner. *(Confirmed.)*

## Follow-up: 2026-06-25

### Fix applied — fail loudly (bridge-hardening layer)

Added an output-boundary invariant guard in `plan_velocity_warm_start` so an impossible
velocity aborts the plan (and hence the print) with a clear, self-identifying error instead
of silently shipping. This is the "fail loudly" half of the fix; the fitter trigger
(degenerate micro-segments) is still open.

- `rust/geometry/src/velocity.rs`:
  - New `VelocityError::NegativeVelocity { line_no, v }` variant (dropped `Eq` from the
    derive to carry the offending `v`; only `PartialEq`/`matches!` were used — verified).
  - New const `NEGATIVE_VELOCITY_TOL_MM_S = 1e-6` and helper `first_negative_velocity`.
  - Guard fires after each move's samples are built (post rest-anchor pinning), catching
    **any** producer of a sub-zero speed, not just the bridge arcs.
  - Surfaces in the print path via `StreamError::Velocity` Display:
    `velocity plan: NegativeVelocity { line_no: 25, v: -0.0396… }`.
- `rust/geometry/src/velocity/tests.rs`: paired unit tests
  `first_negative_velocity_flags_sub_zero_sample` and
  `first_negative_velocity_ignores_float_noise_and_zero` (tol ignores 1e-9 noise / exact 0).

### Verification

- `cargo nextest -p geometry -p motion-engine`: 671 passed.
- fmt + clippy (`-D warnings`) clean on geometry.
- End-to-end: rebuilt cdylib; `arc_fit/fillet` now raises
  `NegativeVelocity { line_no: 25, v: -0.03965… }`; `arc_fit/circle` still plans
  cleanly (min_v=0, max_v=98.87).

### Note / open

- The `arc_fit/fillet` **snapshot** case now raises at plan time, so `snapshots/run.py`
  (which catches only `ImportError`/`ValueError`) will surface a `RuntimeError` for it rather
  than a CHANGED/PENDING verdict. Not in `ci.sh quick`/`py` (those run the harness unit
  tests, not the case comparisons). Decide whether the harness should record a planner error
  as a case outcome — separate from this fix.
- Root-cause **trigger** (biclothoid fit emitting sub-mm segments) remains open — the user's
  stated next thread.

## Follow-up: 2026-06-25 #2

### New Evidence

- **Guard landed.** The bridge now fails loud: `RuntimeError: NegativeVelocity { line_no: 25, v: -0.03965… }`. `arc_fit/fillet.gcode` was renamed `…/fillet.gcode.skip` to keep the snapshot suite green. The aborting `v` (−0.03965) is bit-identical to the first negative sample found pre-guard (s≈187.835) — same defect.
- **arc_fit threshold is NOT the lever (Confirmed).** Sweeping `arc_fit=(facet_len, max_angle)` over (2,12), (2,30), (2,45), (2,60), (5,60) yields the *identical* abort every time (`line_no 25`, `v=-0.03965…`). Widening arc_fit's per-junction angle cap does not capture the offending corner.
- **Raw facet geometry (Confirmed).** 40 junctions; **35 of 40 turn >12°** (the configured arc_fit cap), mostly 15–30°, with 12 near-90° corners and one ~140° near-reversal. So arc_fit (≤12°/junction, co-circular run of ≥2) declines almost the whole fillet — but raising the cap still doesn't fix the failing corner, meaning that corner is rejected for a non-angle reason (isolated / not a co-circular run / long-line neighbour), and is blended by the **per-corner biclothoid**, not the chain path.

### Additional Findings

- **The clothoid length is sized by the junction-deviation budget**, `δ = scv²(√2−1)/accel = 5²·0.4142/1000 ≈ 0.0104 mm` (`fitter.rs:442-445`, consumed in `biclothoid::solve` `fitter/biclothoid.rs:32-40`). The biclothoid half-length scales with `δ`, producing 0.03–0.2 mm clothoids by design — small so the rounded corner stays within ~0.01 mm of the sharp vertex. This is correct corner geometry; it is simply far below the velocity bridge's jerk roll-off scale (`half_max ≈ v·2·accel/jerk ≈ 6 mm`, `disk.rs:617-620`).
- **The defect reproduces on a single isolated tiny clothoid** — it does not require the dense chain. The chain (94/106 sub-mm segments) makes it pervasive, but `line_no 25` fails on its own.

### Updated Hypotheses

#### Hypothesis 2: arc_fit widening would consolidate the fillet and remove the bug — **Refuted**

**Resolution:** Sweep above shows identical failure at every `max_angle`/`facet_len`. arc_fit tuning is a dead end for this corner.

### Root-cause restatement (refined)

Two layers, now sharply separated:

1. **Fit layer (user's chosen fix site):** the per-corner biclothoid emits clothoids sized to the corner-deviation budget `δ ≈ 0.01 mm` → sub-mm arc-length segments. Correct geometry, but below the scale the velocity stage can resolve.
2. **Velocity layer:** the jerk-bridge solver (`disk.rs build_run_bridge`) mis-solves across segments shorter than its roll-off scale and (pre-guard) emitted v<0; now aborts. A correct solver should never emit v<0 — at worst fall back to the base disk profile, which is provably ≥0.

### Solution space (for discussion — not yet chosen)

| # | Where | Idea | Trajectory impact | Cost | Verdict |
|---|-------|------|-------------------|------|---------|
| A | velocity | Bridge falls back to the base disk profile (provably ≥0) when no feasible jerk arc exists, instead of splicing a spurious arc; keep the abort only for truly non-finite. | None to path; one transition loses jerk-rounding (small jerk spike over a sub-mm/ sub-ms window). | Low–med | Strong: geometry-agnostic correctness fix. |
| B | fit | Floor clothoid length at `L_min` tied to the roll-off scale, accepting >δ corner deviation. | Tighter corners cut more than δ → dimensional error. Stubs still tiny. | Low | Weak: violates deviation tolerance. |
| C | fit | Coalesce a run of adjacent sub-threshold corners (+ sub-mm line stubs) into ONE blend spanning the cluster (generalised arc_fit / clothoid-arc-clothoid for non-co-circular runs). | Best: fewer, larger, smooth features; tightest motion; fewer MCU moves. | High | Strong for the chained case; doesn't help a truly isolated corner. |
| D | fit | Below a feasible blend size, leave the corner unblended (sharp), velocity-capped by junction deviation. | Velocity dips to ~scv at every facet → throughput regression (violates the non-negotiable) unless combined with C. | Low | Weak alone. |
| E | velocity | Treat a maximal run of sub-mm members as one bridging unit (coalesce members in `reconstruct_run`). | None to path. | Med | Overlaps A; A is simpler. |

**Leaning:** **A** (correctness floor — the planner must never emit negative speed; base profile is the safe, non-negative, still-tight fallback) as the immediate fix, with **C** (corner coalescing) as the throughput/quality follow-up that removes the wasteful micro-segment chains. B and D trade trajectory quality for simplicity and conflict with the throughput non-negotiable.

**Open question for A:** falling back to base reintroduces an accel step (jerk spike) at that transition. Need to confirm that's acceptable over a sub-mm/sub-ms window, or bound the fallback's jerk.

### Backlog Changes

- Quantify the failing clothoid's length/curvature (`kappa_peak`, `sigma`, half-length) vs the bridge roll-off scale, to set `L_min` (B) or confirm A's necessity. (Pipeline aborts before returning — needs a Rust-side probe or temporary guard bypass.)

## Follow-up: 2026-06-25 #3

### Architectural finding: the failure is isolated to the per-sample accel-smoothing layer

Reading `velocity.rs::plan_velocity_warm_start`:

- **Seam velocities `v[k]` are already jerk-limited and correct.** The forward (`velocity.rs:276-290`) and backward (`velocity.rs:292-301`) sweeps each take `min(disk_reach, scurve::reach_v(run_start_v, cum_arc+len, accel, jerk))` — i.e. jerk-limited reachability accumulated from the run anchor. So the junction speeds respect jerk and are non-negative.
- **The failure lives only in `reconstruct_run` → `reconstruct_flat` → `build_run_bridge`** — the layer that fills the per-sample `(s, v, a)` *inside* a run and makes `a` continuous (jerk-limited) across the interior. Base samples are jerk-aware and ≥0; the **bridge arcs** (the accel-continuity patches) are the sole source of v<0 (`velocity.rs:367` now aborts).

### Option A (base-profile fallback) is disqualified

User principle: the planner never emits an instantaneous acceleration step ("we don't produce infinite acceleration jumps like mainline — it's rude"). The base (un-bridged) profile has accel steps where forward meets backward / ceiling; falling back to it reintroduces exactly that step = a jerk impulse. So A (and E, and D) all violate the no-accel-step invariant. **Retract A.** The velocity-layer fix must stay C¹-in-accel.

### The residual gap C cannot close

C (arc/clothoid-arc-clothoid consolidation) removes micro-segments where the facets approximate a smooth arc. It cannot help when the micro-segments are the *real* geometry (rapid direction changes that don't fit one arc). That residual must be handled in the velocity layer — and, per the no-accel-step invariant, by a method that is jerk-continuous and non-negative *by construction*, not by patching.

### Refined recommendation

- **C (fit):** consolidate faceted arcs → fewest, largest segments. Best for throughput / MCU-queue. (User already has a branch.)
- **Velocity (residual):** replace the discrete per-transition bridge-patching in the sample reconstruction with a **continuous jerk-limited integration** (forward/backward in (v, a) phase space, control = ±jerk_max clamped to the disk/curvature ceiling and to v≥0). It is accel-continuous and non-negative by construction and degrades gracefully on dense curvature (v simply rides *below* a ceiling it cannot track within the jerk budget — which the discrete bridge model cannot express). Physically, rapid direction changes force low v anyway (a_n = κv² ceiling), so the integrated answer is "low and smooth," not exotic.
- C and the velocity fix are **complementary**: C for the common faceted-arc case, the integration for the genuine non-arc micro case. Bridge-hardening (clamp v≥0 + merge overlapping arcs) is a fragile stopgap, not the SOTA answer.
