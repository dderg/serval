---
title: 'Build step 7 — limit-riding through clothoids: one continuous constant-|a| acceleration-disk velocity profile (tangential⇄centripetal budget-traded) replacing the per-move trapezoid skeleton; closed-form line/arc + adaptive 1-D ODE per clothoid; seamless straight↔clothoid blend'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: 'a8372c569319569cc381adb3b7299b427ba2059f'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-5-velocity-planning.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-6-tangential-jerk.md'
  - '{project-root}/_bmad-output/implementation-artifacts/deferred-work.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The step-5/6 sweep is a **per-move trapezoid model**: each move gets one constant ceiling and an independently-computed `(start_v, cruise_v, end_v)` double-S, stitched at node boundaries. A blended corner therefore crawls through its *entire* clothoid at the single tightest-point speed `√(a/κ_peak)`, and the entry/exit straights are planned as separate trapezoids that brake to a fixed seam speed — wasting the gentle low-κ ends and producing an acceleration that is not continuous across the straight↔clothoid seam. Build-sequence step 7, "limit-riding through clothoids — the budget-trading speed upgrade."

**Approach:** Replace the trapezoid representation with **one continuous velocity profile** `v(s)` per stop-bounded run. In every accel/decel region the acceleration **magnitude is held at `a_max`** and the *vector rotates*: `a_c = v²·|κ(s)|` centripetal, `a_t = √(a_max² − a_c²)` tangential, so `dv/ds = a_t/v`. On a straight (κ=0) all of `a_max` is tangential (recovers step-5/6); through a clothoid the budget trades smoothly into centripetal, fully centripetal at the apex, then back. Because the step-4 fitter makes κ continuous across line↔clothoid seams (κ ramps from 0) and the planner holds `|a|=a_max` across that seam, the entry-straight deceleration and the clothoid deceleration are **one ramp** — seamless. κ is closed-form per the alphabet: line (κ=0) and arc (κ const) reaches are closed-form; **the only numerical integration in the whole planner is the adaptive 1-D ODE per clothoid** (linear κ). The profile is the classic forward-backward (Dong-Stori) construction, now on the acceleration **disk** with the pointwise limit curve `v_lim(s)=√(a_max/|κ(s)|)` as the ceiling. The step-6 tangential-jerk S-curve is retained and **composes by min** (it bounds the magnitude ramps; the disk bounds the rotation).

## Boundaries & Constraints

**Always:**
- Internal rework of `geometry::velocity` only. `plan_velocity(&FitOutcome, VelocityConfig) -> Result<VelocityProfile>` keeps its signature; `fitter`, `frontend`, `path`, `segment`, `gcode`, `trajectory`, `pipeline` are byte-for-byte untouched (still a pure consumer of `CurvatureProfile` + `FitOutcome`).
- The acceleration body is the **isotropic disk** `a_t² + a_c² ≤ a_max²` with `a_max = limits.accel_mm_s2`, `a_c = v²·|κ(s)|` (the architecture's carried "global scalar XY disk"). Time-optimal accel/decel rides `|a| = a_max`; cruise is `a=0` only when the profile reaches `min(feedrate, max_velocity)`.
- Pointwise speed ceiling is `v_ceiling(s) = min(feedrate, max_velocity, v_lim(s))`, `v_lim(s) = √(a_max/|κ(s)|)` (`+∞` where κ=0). The profile equals `min(forward_reach(s), backward_reach(s), v_ceiling(s))` over each run between rest anchors (chain start/end + every pinned stop).
- **Continuity across seams is the headline invariant:** integration is *not* clamped or restarted at internal move boundaries; only rest anchors (`v=0`) and the feedrate/`max_velocity` flat ceilings break a run. Adjacent moves share the seam speed exactly (`moves[k].exit_v == moves[k+1].entry_v`), and where κ is continuous at the seam (fitter G2/Collinear) the acceleration magnitude is continuous too.
- **Per-segment reach laws:** Line (κ=0) and Arc (κ const) closed-form (`v² = (a/κ)·sin(2κs+φ)` saturating at `v_lim`); Clothoid (κ linear) the adaptive numerical ODE `dv/ds = √(a_max² − (v²κ(s))²)/v`, clamped to `v_lim(s)`. Deterministic: fixed tolerance, fixed step-control rule, no data-dependent iteration bound that changes output, no RNG.
- **Jerk composition (confirmed division):** the step-6 `scurve` primitives bound the magnitude ramps (`|a|: 0⇄a_max`); the disk bounds the rotation during the hold. They compose by taking the **more restrictive tangential acceleration** at each integration step, so a pure straight reduces *exactly* to the step-6 double-S and `J=+∞` reduces *exactly* to the pure disk profile.
- Sharp-junction stops unchanged: a `FitReport` unblended junction with reason ≠ `Collinear` pins `v=0` (and the blend-half exclusion from step 5 is preserved); non-spatial/virtual moves bracket as rest anchors exactly as today.
- All step-5 fail-loud seam guards preserved: clothoid L-consistency `σ ≈ (κ(L)−κ(0))/L`, node-coverage (`kappa_peak` location at an endpoint), non-finite κ/σ/length → `VelocityError::{Inconsistent,NonAlphabet,NonFinite}`. Config validation (`consistency_tol`, `max_jerk_mm_s3>0`/finite-or-+∞, new integration tolerance >0/finite) → `InvalidConfig`.
- **Measure the gain (the deliverable):** the profile carries traversal time `∫ ds/v`; `VelocityReport` exposes `traversal_time_s` and a `limit_ride` count (moves whose interior speed exceeds the old `√(a/κ_peak)` constant ceiling). A test asserts step-7 run time ≤ the step-5 constant-ceiling time on a representative blended-corner chain.

**Ask First:**
- Folding jerk into the rotation (bounding `da/dt` of the full vector as it swings through the clothoid) — materially harder; deferred this step.
- Anisotropic / per-axis acceleration body (the disk is the carried V1 body; per-axis is a decided non-goal).
- Moving the jerk knob to per-move `VelocityLimits`, or adding a volumetric-flow / PA-augmented (`τ·s⃛`) extruder-transient cap (still deferred — see `deferred-work.md`).
- Wiring the profile into execution lowering / `trajectory` / live path, or emitting `s(t)` time breakpoints / Fresnel position (that is the EX stage).

**Never:**
- No SOCP/Clarabel, no SLP, no fully-coupled jerk TOPP. No planning **grid** / arc-length resampling table — the per-clothoid ODE is *local and adaptive*; the run anchors stay node-based (junctions ∪ stops).
- No `point_at`/`heading_at`/`PositionProfile`/Fresnel anywhere in `velocity` — the read-contract is κ-space only (`CurvatureProfile` + `VelocityLimits` + `FitReport`).
- No lateral-jerk constraint (lateral jerk is the fitter's / geometry's to own). No edits to any step-1/2/3/4 stored type or trait.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Long straight, slack | `Line`, `L` ≫ ramp | jerk-limited ramp to `min(feed,max_v)`, cruise, ramp down (recovers step-6) | N/A |
| Straight→clothoid→straight blend | line, blend-half, blend-half, line; κ continuous at seams | one continuous `v(s)`; `|a|=a_max` continuous across both seams; interior speed > `√(a/κ_peak)` on the low-κ ends; `min` at the apex node only | N/A |
| Clothoid entered with slack | gentle blend, high feed | speed rides the disk down to apex then back up; `peak_v` near the gentle-end `v_lim`, `limit_ride` incremented | N/A |
| Arc (constant κ) | explicit `Arc` radius r | closed-form `v²=(a/κ)sin(2κs+φ)` saturating at `√(a·r)`; cruises at `√(a·r)` when long | N/A |
| Apex budget exhaustion | at κ_peak node | `a_t→0` (all budget centripetal); ramp away eases in (no instantaneous decel, unlike the box skeleton) | N/A |
| Sharp corner / virtual | `report.unblended` (≠Collinear) / non-spatial | node pinned `v=0`; neighbors ramp to/from 0 under the composed disk+jerk law | N/A |
| `J = +∞` | any chain | profile equals the pure-disk (no-jerk) limit-riding plan within tol | N/A |
| Short run, no cruise | small `L`, low end speeds | peak trimmed so the up-then-down fits `L`; never exceeds `v_ceiling` | N/A |
| Degenerate clothoid | `σ ≠ (κ(L)−κ(0))/L` | — | `Err(Inconsistent{line_no})` |
| Mid-segment κ peak | `kappa_peak().0 ∉ {0,L}` | — | `Err(NonAlphabet{line_no})` |
| Non-finite cap / bad config | κ/σ NaN/∞; `max_jerk≤0`; `integration_tol≤0` | — | `Err(NonFinite{line_no})` / `Err(InvalidConfig)` |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/velocity.rs` — **rework**: replace the per-move-ceiling node sweep + trapezoid assembly with the continuous forward-backward disk integration over rest-bounded runs. New `MoveVelocity` (profile-based, below) + `VelSample`; `VelocityReport` gains `traversal_time_s`, `limit_ride`. `VelocityConfig` gains `integration_tol`. Keep `validate_segment`, the stop-line derivation, and all fail-loud `VelocityError`s.
- `rust/geometry/src/velocity/disk.rs` — **new**: the disk law. `limit_speed(kappa,a)→v_lim`; `line_reach`/`arc_reach` (closed-form, saturating); `clothoid_reach` (adaptive RK/embedded-step ODE on `dv/ds=√(a²−(v²κ)²)/v` with linear κ), all composed-by-min with `scurve` for the jerk magnitude ramp; emits monotone `(s,v)` breakpoints. Ends with `#[cfg(test)] mod tests;`.
- `rust/geometry/src/velocity/disk/tests.rs` — **new**: closed-form arc vs numeric cross-check; clothoid ODE convergence + determinism; `v²κ ≤ a` (disk feasibility) along every sample; `J→∞` recovers pure disk; κ→0 recovers `scurve`.
- `rust/geometry/src/velocity/scurve.rs` — **keep**; jerk magnitude-ramp primitives are now called by `disk` for the min-composition. No behavior change.
- `rust/geometry/src/velocity/tests.rs` — **rework**: re-express the trapezoid-shaped assertions as profile/continuity assertions; add the matrix rows, the seam-continuity invariant, the traversal-time gain test, and the `J=∞` reduction.
- `rust/geometry/src/lib.rs` — update the `pub use velocity::{…}` re-export for the new `MoveVelocity` shape + `VelSample`.
- `_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md` — "Velocity planning" §: mark limit-riding live; constant-|a| disk profile replaces the per-move trapezoid; line/arc closed-form, clothoid the lone adaptive ODE; seamless seam via fitter κ-continuity + held `a_max`.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/velocity/disk.rs` — implement `limit_speed`, `line_reach`, `arc_reach` (closed-form `v²=(a/κ)sin(2κs+φ)` clamped to `v_lim`), `clothoid_reach` (deterministic adaptive ODE), each composing by min with `scurve` jerk ramps; return monotone `(s,v)` breakpoints incl. both endpoints. Unit tests per `disk/tests.rs`.
- [x] `rust/geometry/src/velocity.rs` — segment ceilings (now `v_lim`-aware) + fail-loud validation unchanged; partition moves into rest-bounded runs (stops/ends/virtual); forward pass (max-accel disk reach from each anchor) and backward pass (max-decel) per run; profile `= min(fwd, bwd, ceiling)`; assemble per-move `MoveVelocity{ entry_v, exit_v, peak_v, samples, accel, jerk, length, source }` with `exit_v==next.entry_v`; compute `traversal_time_s`, `limit_ride`. New `VelSample{ s, v }`; `VelocityConfig{ …, integration_tol }` (default documented) + validation.
- [x] `rust/geometry/src/lib.rs` — update `velocity` re-exports (`MoveVelocity`, `VelSample`, …).
- [x] `rust/geometry/src/velocity/tests.rs` — rework trapezoid assertions to profile+continuity; add every I/O-matrix row (incl. all `Err` paths), the seam-continuity invariant, and the traversal-time gain test.
- [x] `_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md` — update the "Velocity planning" section.

**Acceptance Criteria:**
- Given a straight→blend→straight chain with κ continuous at both seams, when planned, then `v(s)` is continuous (`exit_v==next.entry_v` to `1e-9`), every sample satisfies disk feasibility `v²·|κ(s)| ≤ a_max + tol`, and on the low-κ ends the interior speed strictly exceeds the step-5 constant ceiling `√(a/κ_peak)` (i.e. `limit_ride ≥ 1`).
- Given that same chain, when its `traversal_time_s` is compared to the step-5 constant-`√(a/κ_peak)`-ceiling traversal time computed in-test, then step-7 time ≤ step-5 time (strictly < when any blend has slack) — the upgrade is measurably not slower.
- Given a straight-only chain (κ=0 throughout), when planned at finite `J`, then the profile equals the step-6 double-S plan within `1e-6`; given `J=+∞`, then it equals the pure-disk limit-riding profile within tol.
- Given an arc of radius r, when planned, then samples obey `v ≤ √(a·r)+tol`, the closed-form `arc_reach` matches a numeric integration of the same ODE within `1e-6·v`, and a long arc cruises at `√(a·r)`.
- Given a clothoid, when `clothoid_reach` runs twice with identical inputs, then byte-identical breakpoints (determinism); when integrated then differentiated, every interior `|a|` ≤ `a_max + tol`.
- Given `σ` falsified / interior κ-peak / non-finite κ / `integration_tol ≤ 0` / `max_jerk ≤ 0`, when planned, then the matching `Err` fires (fail-loud preserved).
- (Additive elsewhere) workspace builds; `cargo nextest run -p trajectory` unchanged; `frontend`, `fitter`, `path`, `segment`, `pipeline`, `gcode` byte-for-byte untouched; `velocity` source contains no `point_at`/`heading_at`/`PositionProfile`.

## Spec Change Log

- [Review][Patch] **Silent integrator truncation (all three reviewers, HIGH/MED).** `integrate_ode` returned a partially-integrated `w` with no error when `RK_MAX_STEPS` was exhausted — a too-low reach ⇒ silently slower trajectory, violating fail-loud + the throughput non-negotiable. Fix: the reach chain (`integrate_ode`→`disk_reach_*`→`reach_v`/`reach_v_rev`/`profile_speed`/`sample_profile`) now returns `Option`, mapped to `VelocityError::Diverged { line_no }` at the `plan_velocity` boundary. The **reachable** DoS cause — an absurdly tiny `integration_tol` that makes the per-point sampler recurse to depth and re-integrate forever — is rejected loudly and fast by a new `MIN_INTEGRATION_TOL` (`1e-9`) floor in config validation (`InvalidConfig`); a `SAMPLE_MAX_POINTS` cap bounds the sampler as belt-and-suspenders. KEEP: fail-loud over silent slow-trajectory; the disk math/reductions/determinism were confirmed correct by all three reviewers. [`disk.rs`, `velocity.rs`]
- [Review][Patch] **`refine` dropped the midpoint at the depth cap (Blind Hunter, HIGH).** When `SAMPLE_MAX_DEPTH` was reached with residual interpolation error, the known-deviating midpoint was discarded, leaving an inaccurate chord. Now the midpoint is pushed best-effort at the cutoff. [`disk.rs`]
- [Review][Patch] **AC2 gain under-tested (Acceptance Auditor + Edge Hunter).** Only a single clothoid's local time was checked, not the headline `report.traversal_time_s` against a step-5 baseline. Added `limit_riding_beats_the_constant_ceiling_skeleton`, asserting `report.traversal_time_s < (straight times + clothoid crawl L/√(a/κ_peak))` — the chain-level deliverable. [`velocity/tests.rs`]
- [Review][Note] **`Diverged` is defensive depth, guarded by the tol floor.** RK4 is exact in linear-`f` regions (err≡0 ⇒ large steps accepted) and adapts only where the ODE curves, so the integrator is robustly convergent for real geometry — `RK_MAX_STEPS` exhaustion is essentially unreachable at a valid (`≥1e-9`) tolerance. The reachable bad-input boundary is therefore the `integration_tol` floor → `InvalidConfig`, which is the paired-tested guard (`invalid_config_is_rejected`).
- [Review][Reject] **`peak_v` interior-crest undershoot; trapezoid `2Δs/(v1+v2)` time; curvature-vs-feedrate tie classification; `v=0` traversal-time guard.** Bounded by the adaptive sampler's `tol` (peak), a consistent same-method comparison metric (time), step-5-inherited telemetry semantics (bound classification), or unreachable-in-practice (v=0 only at rest endpoints). No trajectory-correctness effect.

## Design Notes

**The disk ODE and why a clothoid is the only numerical case.** Bang-bang time-optimal on the disk holds `|a|=a_max`; with `a_c=v²κ`, `dv/ds = √(a_max²−(v²κ)²)/v`. Substituting `w=v²` gives `dw/√(a²−κ²w²)=2 ds`, i.e. `(1/κ)·arcsin(κw/a)=2s+C`. For **constant κ** (line κ=0 → `w=v₀²+2as`; arc → `w=(a/κ)·sin(2κs+φ)`) this integrates closed-form. For **linear κ** (clothoid) it does not — that single local, adaptive integration is the only numeric step in the planner, exactly as the architecture states.

**Why the seam is seamless.** The fitter inserts a clothoid that ramps κ from 0 at a line seam, so κ is continuous there. Holding `|a|=a_max` across the seam, `a_t=√(a²−(v²κ)²)` starts at `a_max` (κ=0) and decreases smoothly as κ rises — no acceleration-magnitude step. The decel of the entry straight and of the clothoid are literally the same ODE solution continued across the boundary; we therefore integrate per *run*, not per move, and never clamp at an internal seam.

**Jerk by min-composition (the deferred-vs-shipped line).** The disk gives the acceleration-bounded reach; `scurve` gives the jerk-bounded reach (how fast `|a|` can ramp 0⇄a_max). Taking the more restrictive `a_t` at each step keeps step 6 intact (κ=0 ⇒ disk is constant a_max ⇒ jerk binds ⇒ exact step-6 double-S) and keeps `J=∞` exact (jerk never binds ⇒ pure disk). Bounding `da/dt` *through* the rotation (woven, not min) is the harder coupled problem, deferred.

**Forward-backward on the disk.** Same Dong-Stori two-pass structure as step 5 — forward = max-accel reach from each rest anchor, backward = max-decel reach to each anchor, profile = `min(fwd,bwd,v_ceiling)`. The novelty is only the reach law (disk ODE, not `√(v²+2aL)`) and that the ceiling `v_lim(s)` now varies inside a clothoid instead of pinning the whole move at `√(a/κ_peak)`.

Sketch (one clothoid forward step, composed):
```
let v_lim = (accel / kappa(s).abs()).sqrt();           // pointwise ceiling
let a_disk = (accel*accel - (v*v*kappa(s)).powi(2)).max(0.0).sqrt();
let a_jerk = scurve_ramp_accel(v, ...);                // step-6 magnitude-ramp bound
let a_t = a_disk.min(a_jerk);                          // more restrictive wins
v = (v + a_t / v * ds).min(v_lim);                     // advance, clamp to limit curve
```

## Verification

**Commands:**
- `cargo nextest run -p geometry` — expected: reworked `velocity` + new `disk` tests pass; other geometry tests unchanged.
- `cargo nextest run -p trajectory` — expected: unchanged (proves the rework stays inside `velocity`).
- `./scripts/ci.sh rust-clippy` && `./scripts/ci.sh rust-fmt` — expected: green (`-D warnings`).
- `! grep -rnE 'point_at|heading_at|PositionProfile' rust/geometry/src/velocity.rs rust/geometry/src/velocity/` — expected: no matches.
- `git diff --stat` on `rust/geometry/src/{path,frontend.rs,fitter.rs,segment.rs,pipeline.rs}` and `rust/gcode` — expected: empty.

## Suggested Review Order

**Design intent (start here)**

- Entry point: the whole pass — config validation, ceilings, node sweep on the disk, then per-move profile assembly + the gain metric.
  [`velocity.rs:82`](../../rust/geometry/src/velocity.rs#L82)

**The disk law — closed-form line/arc + the lone adaptive ODE (highest-leverage math)**

- Constant-κ closed form `w=(a/κ)sin(2κs+φ)` saturating at `v_lim` — line (κ=0) and arc.
  [`disk.rs:54`](../../rust/geometry/src/velocity/disk.rs#L54)
- The clothoid disk ODE `dw/ds=2√(a²−κ²w²)`, adaptive RK4 step-doubling; fail-loud `None` on budget exhaustion.
  [`disk.rs:76`](../../rust/geometry/src/velocity/disk.rs#L76)
- Jerk by min-composition: disk reach ∧ step-6 `scurve` magnitude-ramp reach.
  [`disk.rs:125`](../../rust/geometry/src/velocity/disk.rs#L125)

**Continuous profile + seam continuity**

- `profile_speed = min(fwd, bwd, jerk_fwd, jerk_bwd, ceiling)` — the pointwise limit-riding value.
  [`disk.rs:137`](../../rust/geometry/src/velocity/disk.rs#L137)
- Adaptive sampler; pushes the deviating midpoint at the depth/points cap (review patch).
  [`disk.rs:156`](../../rust/geometry/src/velocity/disk.rs#L156)
- Node sweep uses disk reaches; shared node ⇒ `exit_v==next.entry_v` (seamless), fail-loud `Diverged`.
  [`velocity.rs:175`](../../rust/geometry/src/velocity.rs#L175)

**Fail-loud boundary (review patches)**

- `integration_tol` floor `MIN_INTEGRATION_TOL` rejects DoS-tiny tolerances as `InvalidConfig`.
  [`velocity.rs:90`](../../rust/geometry/src/velocity.rs#L90)
- Step-5 fail-loud guards preserved (L-consistency, node-coverage, non-finite).
  [`velocity.rs:231`](../../rust/geometry/src/velocity.rs#L231)

**Tests (peripherals)**

- Headline AC: clothoid rides above the constant ceiling, faster than the crawl.
  [`tests.rs:134`](../../rust/geometry/src/velocity/tests.rs#L134)
- Seam continuity across a line→clothoid→clothoid→line chain.
  [`tests.rs:154`](../../rust/geometry/src/velocity/tests.rs#L154)
- Chain-level gain: `report.traversal_time_s` beats the step-5 constant-ceiling skeleton.
  [`tests.rs:414`](../../rust/geometry/src/velocity/tests.rs#L414)
