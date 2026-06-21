# Investigation: corner-blend jerk discontinuity (rampdown + clothoid traversal)

## Hand-off Brief

1. **What happened.** With corner blending enabled, the viz traces show acceleration discontinuities (jerk spikes ~1e10) at accel→decel reversals and corners, and clothoids traversed at near-constant velocity instead of a smooth tangential↔centripetal handoff. **Confirmed root cause (one defect, both symptoms):** the `geometry` velocity planner has no acceleration continuity across move boundaries — jerk enters only as a velocity ceiling (`scurve::max_reachable_velocity`), and each move's s-curve ramp lands at `a=0` on the junction. So acceleration is discontinuous at every stitch (jerk spikes), and the straight ramps tangential accel to 0 before the clothoid (no trade).
2. **Where the case stands.** Claim 1 (reversal/corner jerk spike) **Confirmed**. Claim 2 (a→0 into clothoid) **Confirmed** as the same no-acceleration-boundary-state mechanism (Finding 4); the sub-mm-blend hypothesis is **refuted** per the user's correction. Pivotal architectural finding: **two planners coexist** — viz *and* the current production bridge (`PyMotionEngine → bridge → stream_planner → stream`) use the *geometry* planner; the jerk-continuous SOTA planner `temporal`/TOPP (carries `v`+`a` on the actual NURBS path) is built but not reachable from the Python entry points.
3. **What's needed next.** Claim 2 **FIXED** (2026-06-19, run-anchored jerk in `geometry::velocity` — see Follow-up #4). Re-run viz to confirm the corners; Claim 1 (straight-reversal cusp) remains an optional follow-up (the deferred `peak_velocity`/apex work). The Option A/B architecture debate is moot for the XY stage — the specced design (`spec-motion-6/7`) is the per-case geometry planner; `temporal` is off-architecture for this stage.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-19                                                                 |
| Status           | Active — root cause Confirmed (Claim 1); Claim 2 mechanism Deduced, data gap |
| System           | Branch `curvature-profile`; Rust `geometry` crate (`velocity.rs`, `velocity/disk.rs`, `velocity/scurve.rs`); PyO3 viz (`motion-engine/src/viz.rs`, `scripts/viz_pipeline.py`) |
| Evidence sources | Source code (geometry velocity planner, viz), user viz plots (5 mm square @ SCV 50; larger multi-segment shape), prior investigation `scv-clothoid-instant-corner-investigation.md` |

## Problem Statement

User, after implementing corner blending and running viz tests, reports two issues:

- **(1)** Jerk is only enforced/applied during acceleration **rampup**, not **rampdown**. Going from acceleration to deceleration spikes jerk.
- **(2)** Jerk is applied when **decelerating into a clothoid**, and it shouldn't be — a clothoid has continuous acceleration throughout. The design intent was to slowly transition tangential acceleration into centripetal and back. Instead the planner reduces acceleration to ~0, then traverses the clothoid at apparently constant velocity.

Evidence images: (Image 1) 5 mm square @ SCV 50 — sawtooth velocity, accel humps peaking ~2000 with spikes to 10000 at corners, two jerk spikes ~1.5e10 at the outer velocity peaks. (Image 2) larger multi-segment shape — trapezoidal velocity segments, accel spikes ~10000 at corners, multiple jerk spikes 1–5e10.

## Evidence Inventory

| Source | Status | Notes |
| ------ | ------ | ----- |
| `rust/geometry/src/velocity.rs` | Available | Forward/back velocity pass; jerk enters at `:245` only as a `jerk_bound` report counter; profile from `disk::sample_profile` |
| `rust/geometry/src/velocity/scurve.rs` | Available | `max_reachable_velocity` — jerk-limited *velocity bound*, not a time profile; returns a scalar speed |
| `rust/geometry/src/velocity/disk.rs` | Available | Disk constraint `w'=2√(a²−κ²w²)`; jerk applied as `.min(scurve::max_reachable_velocity)` at `:127-128,:133-134,:141-150`; `sample_profile` emits `(s,v)` |
| `rust/motion-engine/src/viz.rs` | Available | `:32` calls `geometry::plan_velocity`; `sample_kinematics` emits `kin_s/v/kappa/heading` |
| `scripts/viz_pipeline.py` | Available | `_build_time_series` (`:171-204`): `a_tan=v·dv/ds`, `a_cen=v²·κ`, `jerk=∇(a,t)` — faithful reconstruction of planned kinematics |
| User viz plots (Image 1, Image 2) | Available | Symptom evidence — accel discontinuities and jerk spikes |
| `temporal` crate (TOPP/jerk SOCP) | Available, not on path | Jerk-aware SOTA planner exists but is NOT invoked by viz/streaming (prior Finding 5) |
| Per-sample `(s,v,κ)` dump for user's exact geometry | **Missing** | Needed to settle Claim 2 mechanism (decel-to-apex-then-flat vs. genuine trade) |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Settle Claim 2: dump `(s,v,κ,a_tan,a_cen)` across a single blended corner on the user's geometry | High | Open | Distinguishes "decelerate to apex ceiling on straight, then ride flat" from "trade through clothoid" |
| 2 | Confirm `temporal`/TOPP is still off the viz/streaming path on current HEAD | High | Open | Prior Finding 5 — re-verify; decides fix direction (wire in temporal vs. make geometry jerk-continuous) |
| 3 | Trace why accel reaches 10000 (=5× accel hump peak) at corners — centripetal spike on a sub-mm blend vs. ∇ artifact over tiny dt | Medium | Open | At SCV 50 on a 5 mm square the blend is sub-mm; near-instant heading change ⇒ large `v²κ` |
| 4 | Determine whether the accel→decel apex sharpness is the binding `min(forward,backward)` crossover | Medium | Open | velocity/disk.rs `profile_speed` takes elementwise min; the crossover is C0 in v(s) ⇒ C(-1) in accel |

## Confirmed Findings

### Finding 1: Jerk shapes each monotonic ramp, but the planner stitches ramps with `min()` — the stitch points are C0 cusps in v(s) ⇒ discontinuous acceleration

**Evidence:** `rust/geometry/src/velocity/disk.rs:125-135` (`reach_v`/`reach_v_rev` = `disk.min(scurve::max_reachable_velocity)`); `:137-153` (`profile_speed` = `min(forward, backward, jerk_forward, jerk_backward, ceiling)`); `rust/geometry/src/velocity/scurve.rs:1-16` (returns a scalar reachable *speed*); `rust/geometry/src/velocity.rs:245-248` (jerk used only to bump a `jerk_bound` report counter).

**Detail:** The planner output is `MoveVelocity.samples: Vec<VelSample{s,v}>` — a velocity-vs-arclength curve. Jerk is NOT absent from the *shape*: `scurve::max_reachable_velocity` in its **triangular regime** (`scurve.rs:3-8`, `length <= triangular_distance`) holds jerk constant, so within a single accelerating (or decelerating) ramp `a_tan = v·dv/ds` rises **linearly** — a proper constant-jerk ramp. This is why the straights show linear accel ramps, not the flat plateau a pure disk constraint (`w'=2a`, κ=0) would give. **The gap is across ramps, not within them.** `profile_speed` builds `v(s)` as the elementwise `min()` of a forward profile (shapes the accel ramp) and a backward profile (shapes the decel ramp). At their crossover — the velocity peak (accel→decel reversal) and each junction/corner — `v(s)` has a **cusp** (slope discontinuity). Differentiating a cusp gives an acceleration step (`+a` → `−a`), hence the unbounded jerk. Nothing in the planner enforces *acceleration continuity across the stitch*; the jerk limit only bounds how fast speed builds inside one ramp. This single mechanism explains both reported symptoms.

### Finding 2: The viz accel/jerk traces are faithful reconstructions, so the spikes are real planner discontinuities, not plotting noise

**Evidence:** `scripts/viz_pipeline.py:192-202` — `a_tangential = v*np.gradient(v,s)`, `a_centripetal = v**2*kappa`, `jerk = np.gradient(a, t)`.

**Detail:** Acceleration is computed from the planned `v(s)` and path `κ(s)`; jerk is its time-derivative. The ~1e10 jerk magnitude is `Δa/Δt` across an acceleration step where `Δt` between adjacent samples is tiny — finite-difference inflation of a **genuine acceleration discontinuity**. The discontinuity's existence is real; only its plotted magnitude is mesh-dependent. (Claim 1's symptom is therefore a true property of the planned motion.)

### Finding 3: Two velocity planners coexist — viz AND the current production bridge ride the *geometry* one; the jerk-aware `temporal`/TOPP one is built but off the PyMotionEngine path

**Evidence:**
- geometry path: `rust/motion-engine/src/viz.rs:32` (`geometry::plan_velocity`); `rust/motion-engine/src/stream.rs:168-169` (`fit_chain` + `geometry::plan_velocity_warm_start`); production routing `lib.rs:56-57` exposes only `PyMotionEngine` + `viz::pipeline_snapshot`; `PyMotionEngine` lives in `bridge.rs`, which uses `stream_planner::StreamPlannerHandle` (`bridge.rs:23`); `stream_planner` is built on `stream::{StreamConfig, StreamState}` (`stream_planner.rs:10`) — i.e. the geometry planner.
- temporal path: `rust/trajectory/src/plan_velocity.rs:18-26,50` ingests `temporal::multi::SegmentInput` + `GridStrategy`; `trajectory/src/streaming/state.rs` and `motion-engine/src/planner.rs` ("anytime temporal replan", `planner.rs:757`) drive it. `temporal::topp` is a jerk-constrained TOPP (Consolini-Locatelli SOCP relaxation + Lee 2024 SLP outer iteration — `docs/research/jerk-constrained-socp-relaxation-tightness.md`), ingests the **actual path** as NURBS (`topp/path.rs`: `C(u)`, `c'`, `c''`, `c'''` on an `ArclengthGrid`), and carries an **acceleration** boundary condition `a_start` across windows/junctions (`topp/constraints.rs:53-57` `EndpointConditions{v_start, v_end, a_start}`).

**Detail:** The graphs the user is plotting come from the geometry disk/s-curve planner (viz), which by construction (Finding 1) bounds reachable *speed* by jerk without producing jerk-continuous motion and has **no acceleration state across boundaries** (Finding 4). The `temporal` planner is the SOTA jerk-limited generator that *does* carry acceleration continuously on the real path, but it is **not** reachable from the Python entry points — `PyMotionEngine`'s streaming goes `bridge → stream_planner → stream::StreamState → geometry::plan_velocity_warm_start`. So both viz and the current production bridge exhibit the jerk discontinuity; it is not a viz-only artifact. *(Matches prior Finding 5 and the project-context note that the pipeline is "not yet end-to-end.")*

### Finding 4: The geometry planner forces acceleration to ZERO at every move boundary — this is why accel drops to 0 entering the clothoid (Claim 2's real mechanism)

**Evidence:** `rust/geometry/src/velocity/scurve.rs:1-16` (`max_reachable_velocity` is a jerk-limited *velocity bound* whose defining endpoint condition is arrival at the target speed with zero acceleration — the `+jerk` tail of an s-curve brings `a→0`); `velocity.rs:188-204` (per-junction the planner pins only a *velocity* `v[k]`, never an acceleration); `velocity/disk.rs:131-135,141-152` (the backward profile `reach_v_rev`/`jerk_backward` shapes the decel to land on `v[k]`).

**Detail:** The planner state across moves is a velocity vector `v[0..=n]` — there is **no acceleration boundary variable**. Each move's deceleration is shaped by the s-curve bound to *arrive at the junction velocity with `a_tan = 0`* (standard s-curve endpoint property: `dv/ds = a_tan/v → 0` at the target). For a straight→clothoid junction the clothoid starts at `κ=0`, so the pinned junction velocity is the feedrate (not curvature-limited; `velocity.rs:200-202` gives `limit_speed(0)=∞`), but the *downstream* curvature ceiling forces `v` down, and the backward pass lands that deceleration on the junction with `a_tan→0`. The printer therefore enters the clothoid at **zero acceleration**, then has to rebuild centripetal accel from scratch as `κ` ramps — so the tangential→centripetal *trade* never gets to happen (the tangential accel is spent on the straight before the blend starts). This is the same root cause as Claim 1: **no C0 acceleration continuity across a profile stitch.** Fixing acceleration-continuity addresses both symptoms. *(The sub-mm blend size is NOT the cause — confirmed against the user's correction; the trade would be precluded at any blend size by the a=0 boundary condition.)*

## Deduced Conclusions

### Deduction 1: Claim 1 ("jerk on rampup, spike on rampdown/reversal") follows from the min-of-bounds profile

**Based on:** Findings 1, 2 and `velocity/disk.rs:137-153`.

**Reasoning:** Near a low-speed endpoint the binding cap is `scurve::max_reachable_velocity` from that endpoint; in its triangular regime (`scurve.rs:3-8`) acceleration ramps linearly (constant jerk) — this is the visible rounded accel hump on rampup in Image 1. The accel→decel **apex** is where the forward profile and the backward profile cross; `profile_speed` takes their elementwise `min`, producing a C0 cusp in `v(s)` at the crossover. Differentiating a cusp gives a sign-flipping acceleration step ⇒ the jerk spike at the velocity peak (Image 1 spikes sit exactly at the tall velocity peaks). The jerk *bound* is symmetric (applied forward and backward), but the trajectory is never reconstructed as jerk-continuous, so the reversal is never smoothed.

**Conclusion:** "Jerk enforced only on rampup" is precisely correct as an observation: the bound rounds speed-building from endpoints, but the reversal/peak is an unsmoothed crossover.

## Hypothesized Paths

### Hypothesis 1: Claim 2 — the blend is traversed at constant v because the profile decelerates to the apex curvature ceiling on the straight approach, then rides flat

**Status:** Refuted (sub-mm premise) / superseded by Finding 4.

**Theory (original):** The constant-v-through-clothoid was caused by the profile finishing its deceleration on the (sub-mm) straight approach and riding the apex curvature ceiling flat.

**Resolution:** The user corrected the premise — the trade should happen at sub-mm too, so blend size is not the cause. Investigation re-anchored: the true mechanism (Finding 4) is that the geometry planner carries **no acceleration boundary state** and lands every deceleration on the junction with `a_tan→0` (s-curve endpoint condition). The clothoid is therefore entered at zero acceleration regardless of blend size, precluding the tangential→centripetal trade. The "reduce acceleration rate at the end of the straight" the user observed is this `a→0` landing.

### Hypothesis 2: The corner accel spikes (10000 ≈ 5× hump peak) are real centripetal accel on the sub-mm blend, not a ∇ artifact

**Status:** Open

**Theory:** `a_centripetal = v²·κ` with a large `κ` over a tiny blend produces accel well above the tangential accel limit because the disk constraint caps *combined* accel via the curvature ceiling, but the plotted spike may also be inflated by `np.gradient` over near-coincident samples at the corner.

**Would confirm:** Compare `a_scalar` peak to `accel/κ_peak` at the corner; if they match, it's physical (curvature-limited), and the planner is honoring the disk limit but the velocity it permits is high.

**Would refute:** The spike exceeds `accel` and `accel/κ` both — pointing at a sampling/∇ artifact or a missing combined-accel clamp.

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Root cause | `rust/geometry/src/velocity/scurve.rs` + `disk.rs:125-153` — jerk modeled as a velocity ceiling, never an acceleration-shaping/time constraint; profile is `min()` of forward/backward/ceiling reaches |
| Trigger | Any accel→decel reversal (peak) or junction; magnified at corners where curvature ceiling binds |
| Condition | Active on the viz/streaming path because both use `geometry::plan_velocity` (`viz.rs:32`), not `temporal`/TOPP |
| Related files | `velocity.rs`, `velocity/disk.rs`, `velocity/scurve.rs`, `motion-engine/src/viz.rs`, `scripts/viz_pipeline.py`, `rust/temporal/*` (off-path jerk planner) |

## Conclusion

**Confidence:** Claim 1: **High** (Confirmed). Claim 2: **High** (Confirmed mechanism — Finding 4, no acceleration boundary state). Two-planner split & production routing: **High** (Confirmed via the `lib.rs`→`bridge`→`stream_planner`→`stream` import chain).

Both symptoms are one defect: the geometry velocity planner (used by viz *and* the current production bridge) is a per-move *speed-bounding* planner with **no acceleration continuity across boundaries**. Jerk enters only as a velocity ceiling (`scurve::max_reachable_velocity`), and every move's ramp lands at `a=0` on the junction. So acceleration is discontinuous at each stitch (reversal apex, corner) → the jerk spikes; and the straight ramps tangential accel to 0 before the clothoid → no tangential→centripetal trade. The jerk-continuous SOTA planner that fixes both *by construction* — `temporal`/TOPP, carrying `v` and `a` on the actual NURBS path — already exists but is not reachable from the Python entry points. The choice is Option A (custom-case the geometry planner — incremental but in tension with the throughput mandate) vs Option B (route through `temporal` — SOTA, already built). Recommended first move: point viz at `temporal` and re-read the graphs before committing to either.

## Recommended Next Steps

Both symptoms reduce to one defect: **the geometry planner has no acceleration-continuity across profile stitches** (no `a` boundary state; s-curve bounds land each ramp at `a=0`). Two ways to fix, with very different cost/quality:

### Option A — Custom-case the geometry planner (local smoothing)

Detect each stitch (reversal apex, and each junction type: line→line stop, line→clothoid, clothoid→clothoid apex, clothoid→line) and splice a jerk-limited acceleration transition, carrying an acceleration boundary condition `a[k]` across moves instead of assuming 0.

- **Pros:** incremental; no SOCP compute; gets viz "looking right" quickly; keeps the existing planner.
- **Cons:** combinatorial special-casing (every new geometry pairing is a new case); you'd be re-deriving acceleration-continuity piecemeal on top of a `min`-of-bounds heuristic that is **not** time-optimal; high risk of missed cases and silent throughput loss; maintains a *second* planner that diverges from `temporal`. **Tension with CLAUDE.md's "never ship a cheaper algorithm that yields a measurably slower trajectory."** A local smooth that lowers a velocity peak to kill a cusp trades throughput for smoothness — exactly the trade the project forbids by default.

### Option B — Holistic: use `temporal`/TOPP (already built)

`temporal` is the jerk-constrained time-optimal planner (Consolini-Locatelli SOCP + Lee 2024 SLP; `docs/research/jerk-constrained-socp-relaxation-tightness.md`). It carries `v` **and** `a` across boundaries (`EndpointConditions`), operates on the actual NURBS path on a grid, and produces jerk-continuous motion *by construction* — so Claim 1 (reversal spike) and Claim 2 (a=0 into clothoid, tangential→centripetal trade) both dissolve without special cases.

- **Pros:** SOTA, throughput-aligned, single planner, no per-geometry casework; the trade is automatic.
- **Cons:** SOCP/SLP host compute per window; more complex; relaxation is conjectural-tight (mitigated by the SLP fallback already specced); needs wiring into viz now and the production bridge later.

### Recommendation (highest value, lowest cost): **point viz at `temporal` before fixing anything**

`temporal` already exists and is the intended planner. The cheapest way to "look at the graphs again" is to add a viz path that runs `trajectory::plan_velocity` (temporal) on the same fitted chain, so you can see whether the jerk-continuous planner already does what you want. That single experiment tells you whether the answer is "wire temporal through" (likely) or "temporal also has a gap to fix" — before investing in Option A casework that may be throwaway.

### Diagnostic (if you still want to characterize the geometry planner)

1. Per-sample dump (`s, v, κ, a_tan=v·dv/ds, a_cen=v²κ`) across one blended corner — visualizes the `a→0` landing (Finding 4) directly (Backlog #1).
2. Check `a_scalar` peak vs `accel/κ_peak` at a corner (Backlog #3 / Hyp 2) — confirms the 10000 spike is physical centripetal vs ∇ artifact.

## Reproduction Plan

1. Reproduce Image 1: `pipeline_snapshot(<5 mm square waypoints>, max_velocity, max_accel, square_corner_velocity=50, arc_fit=None)`.
2. Extend `viz.rs::sample_kinematics` (or a scratch harness) to also emit per-sample `a_tan`/`a_cen`, or compute them in `_build_time_series` and print around each junction.
3. Inspect: (a) jerk spike coincides with velocity peak (Claim 1); (b) `a_tan→0` before vs. through the clothoid (Claim 2).
4. Distill into a `geometry` unit test asserting acceleration continuity expectations once the target behavior is decided.

## Side Findings

- The `temporal` crate already carries jerk through TOPP (`limits.rs`, `topp/constraints.rs`); the gap is integration, not absence of a jerk-aware planner. *(Deduced — relevant to the throughput-SOTA constraint: a jerk-continuous SOCP trajectory is the SOTA target.)*
- `VelocityConfig::max_jerk_mm_s3` defaults to `100_000.0` with a `TODO` noting the jerk floor is an open tuning question (`velocity.rs:27-28`). *(Confirmed.)*

## Follow-up: 2026-06-19 #2 — design direction: analytic per-case (kill NURBS/grid/temporal?)

### Direction the user is steering toward

Abandon discretization/NURBS/SOCP entirely. Handle each segment+junction combination analytically (closed-form per case), carrying `v` and `a` across boundaries. Rationale: no path resampling ⇒ faster, more precise, and (claimed) better throughput. Open to deleting `temporal` if a per-case analytic planner subsumes it. "Single planner" is NOT a differentiator between the options — both end states are one planner; the real axis is **closed-form-per-case vs numerical-optimization-on-a-grid** (concession: prior framing was wrong).

### Confirmed facts that bound the decision

- **The current geometry planner already numerically integrates clothoids** — `disk::disk_reach_w` (`velocity/disk.rs:112-119`) branches: `sigma == 0` (constant curvature: line or arc) → closed form `const_kappa_reach_w`; `sigma != 0` (clothoid, κ linear) → adaptive **RK4** `integrate_ode` (`:76-110`). So "no sampling" is already false for curved segments *today*, independent of temporal. The disk ODE `w' = 2√(a²−κ²w²)` with κ linear is Riccati-type — no elementary closed form. **(Confirmed.)**
- Segment alphabet is small: constant-curvature (line κ=0, arc κ=const) and clothoid (κ linear). Junction cases are a handful (L↔L, L↔C, C↔C apex, C↔L, corner-stop). The *combination count* is not the hard part. **(Confirmed.)**
- Adding a jerk limit introduces unavoidable **geometric jerk** even at constant |a|: the Frenet frame rotates at rate `vκ`, so `|da/dt| ≥ |a|·vκ` while following any curved segment. Jerk-continuity on a clothoid is therefore a coupled differential constraint in `v, v', v'', κ, σ` — not algebraic. **(Deduced.)**

### Decisive open question (make-or-break for killing temporal)

**Does a closed-form (or cheap 1-D root-find) jerk-limited, acceleration-continuous speed law exist for a clothoid segment under the combined-accel disk constraint, with prescribed `(v, a)` at both ends?**
- **If yes:** per-case wins decisively — temporal can die, and the user's speed/precision/throughput thesis holds.
- **If no (only iterative):** a per-case scheme re-derives temporal's SLP per geometry, with no guarantee of time-optimality. A non-optimal hand-rolled profile risks a *measurably slower trajectory* — the exact thing CLAUDE.md forbids. The throughput mandate cuts both ways: SOCP+SLP exists *because* jerk-time-optimal is non-convex (`docs/research/jerk-constrained-socp-relaxation-tightness.md`).

### Recommended spike (math, not build)

Derive the jerk-limited speed law for ONE canonical corner (L→C→L, a-continuous, ride the disk boundary). Determine: (a) closed-form vs iterative; (b) time-optimal vs feasible-heuristic; (c) traversal time vs `temporal` on the same corner. That single derivation answers the architecture question before any planner is deleted. Keep `temporal` as benchmark/fallback until the clothoid law is proven closed-form AND time-competitive.

## Follow-up: 2026-06-19 #3 — SPEC evidence: the per-case design is documented; both claims are regressions, not open questions

The spike above is **already answered by the shipped specs** (`spec-motion-6-tangential-jerk.md`, `spec-motion-7-limit-riding.md`, both `status: done`, `frozen-after-approval`). No new math is needed; the design exists and the implementation drifted from it.

### Finding 5: The per-case, no-solver architecture is the *documented* design — lateral jerk is geometry's, only tangential jerk is limited (closed-form)

**Evidence:** `spec-motion-6:40` ("**No fully-coupled jerk TOPP / SOCP / SLP; no per-axis or lateral jerk constraint** (lateral jerk is free on a clothoid — geometry owns it)"); `spec-motion-7:42` ("No SOCP/Clarabel, no SLP, no fully-coupled jerk TOPP. **No planning grid / arc-length resampling table**"); `spec-motion-7:2,102` (alphabet: line κ=0 and arc κ=const closed-form, **clothoid the only numerical case** — a local adaptive 1-D ODE).

**Detail:** The user's stance ("kill temporal, per-case, no universal solver, no sampling") is **literally the frozen spec** for `geometry::velocity`. Tangential jerk = closed-form double-S on `s(t)` (step 6). Lateral jerk = owned by the fitter's clothoid (κ ramps continuously ⇒ `a_c=v²κ` continuous). The only numerical integration by design is the per-clothoid disk ODE. `temporal` (SOCP/SLP/grid) is the *opposite* architecture and contradicts these specs for the XY velocity stage — it is not the intended planner for this path.

### Finding 6: Claim 2 (a→0 before clothoid, constant-v traversal) directly violates step-7's HEADLINE invariant

**Evidence:** `spec-motion-7:18,20,28,104` — "the entry-straight deceleration and the clothoid deceleration are **one ramp** — seamless"; "Continuity across seams is the headline invariant: integration is **not** clamped or restarted at internal move boundaries; only rest anchors (v=0) and the feedrate/max_velocity flat ceilings break a run." Step 7 explicitly *replaced* the per-move trapezoid that "crawls through its entire clothoid at √(a/κ_peak)" and "brake[s] to a fixed seam speed" — which is exactly the observed (regressed) behavior.

**Detail:** The intended behavior is to hold `|a|=a_max` across the straight→clothoid seam, trading tangential→centripetal (`a_t=√(a_max²−(v²κ)²)`). Observed "decelerate to a≈0 on the straight, then constant v through the clothoid" is the pre-step-7 trapezoid behavior. So step-7 limit-riding is **not taking effect** through the seam (regression or a binding constraint defeating it).

### Finding 7: Claim 1 (reversal jerk spike) — step-7's forward-backward `min`-of-reaches reintroduces an acceleration-discontinuous interior peak that step-6's `apex` primitive was built to prevent

**Evidence:** `spec-motion-6:61,71` specced `scurve.rs` with `max_reachable_velocity` (reach) **and** `peak_velocity` (apex — "monotone bisection trims the peak below the ceiling so the bounded-jerk up-then-down reversal fits L"). Current `rust/geometry/src/velocity/scurve.rs` contains **only `max_reachable_velocity`** — `peak_velocity`/apex is gone. The emitted profile is `disk::profile_speed = min(fwd, bwd, jerk_fwd, jerk_bwd, ceiling)` (`disk.rs:137-153`) sampled by `sample_profile`.

**Detail:** At an interior velocity peak (accel→decel reversal not pinned to a flat ceiling), `fwd` is rising (`a_t>0`) and `bwd` is falling (`a_t<0`); their `min` crosses at a **C0 cusp** ⇒ `a_t` jumps `+→−` ⇒ unbounded jerk. The `min`-of-reaches envelope is correct for accel-bang-bang (step 5, where `a_t` is *allowed* to jump) but **not** jerk-continuous at interior peaks. Step 6's `apex` produced a single jerk-continuous double-S over the reversal; the step-7 rework to run-based `min`-of-reaches dropped it. **Test gap:** no existing AC asserts acceleration continuity at an interior reversal — step-6/7 ACs check node velocities and traversal time, so the cusp ships green.

### Updated conclusion & fix direction

Both claims are **regressions from frozen, documented intent** in the `geometry::velocity` per-case planner — not architecture-decisions-still-open. The fix is within that planner; no `temporal`, no SOCP/SLP, no grid (all explicit non-goals). Direction:
- **Claim 1:** restore a jerk-continuous reversal — reconstruct the interior peak as a double-S (re-introduce/relocate step-6 `peak_velocity` apex into the run-based profile, or post-process the `min`-of-reaches envelope to round each interior cusp). Add the missing acceleration-continuity AC at reversals.
- **Claim 2 (user's TOP priority — "we apply jerk on deceleration before a clothoid; we should not"):** **Confirmed mechanism** — `disk::profile_speed` (`disk.rs:137-153`) reconstructs each move independently and includes `jerk_backward = scurve::max_reachable_velocity(exit, rest, …)`. As `rest→0` (approaching the move's exit) this term forces `v→exit` with `a_t→0` — i.e. the tangential jerk reach *lands zero acceleration at every move exit*. For a line→clothoid seam the exit is the seam, so `a_t→0` there, defeating the disk's `|a|=a_max` limit-ride. The disk term `backward` would carry `a_t=a_max` to the seam, but `min(backward, jerk_backward)=jerk_backward` near the seam, so **jerk binds and kills the decel**. Root cause is unified with Finding 4: the reconstruction assumes `a_t=0` at *every* move boundary (both `jerk_forward` and `jerk_backward` ramp from/to zero accel at the move endpoints), with no acceleration state carried across internal seams. **Correct behavior:** `a_t→0` should be landed ONLY at true run anchors (stop `v=0`, or flat-ceiling cruise where `a=0` is right); across an internal clothoid seam the deceleration must continue at `|a|=a_max` and rotate (disk-governed), exactly as `spec-motion-7` specced "integrate per *run*, never clamp at an internal seam." Fix: suppress the jerk-landing at internal (non-anchor) seams / integrate the decel as one continuous run carrying `a_t=a_max` across the seam. No per-sample dump needed — the mechanism is in the source.
- **Honest caveat (by design):** step-7 defers "jerk *through* the rotation" (`spec-motion-7:36,106`) — only `|a|`-ramp jerk is bounded, by `min`-composition; lateral jerk from frame rotation through a clothoid is geometry's and is not bounded by the planner. The user has already accepted this (lateral jerk = geometry). The spikes in evidence are tangential (reversal) + the seam regression, both in-scope to fix per-case.

### Backlog changes
- Promote: fix Claim 1 (jerk-continuous reversal) and add the accel-continuity reversal test — the user's chosen first target.
- Demote: the "derive clothoid jerk law" spike and "wire temporal into viz" — superseded; design is specced and temporal is off-architecture for this stage.

## Follow-up: 2026-06-19 #4 — FIX implemented (Option B / per-run integration), Claim 2 resolved

### Change

Run-anchored the tangential-jerk reach in `geometry::velocity`, realizing `spec-motion-7`'s "integrate per run, never clamp at an internal seam" (the disk reach was already run-continuous; only the jerk term re-based per move).

- `rust/geometry/src/velocity.rs`: partition moves into runs bounded by true rest anchors (v=0 stops + chain ends; `is_anchor`); precompute per-move `run_start_v`, `arc_from_run_start`, `arc_to_run_end`. Forward/backward node sweeps now compose `disk::disk_reach_v[_rev]` (from node velocities) with a **run-anchored** `scurve::max_reachable_velocity(anchor_v, cumulative_arc, …)`. Reconstruction passes a `disk::JerkAnchors` so the per-move profile's jerk ramps are measured from the run anchors.
- `rust/geometry/src/velocity/disk.rs`: replaced the per-move `reach_v`/`reach_v_rev` (which bundled per-move jerk) with `disk_reach_v_rev` (disk-only) + a `JerkAnchors` struct; `profile_speed`/`refine`/`sample_profile` take `JerkAnchors`. Jerk `fwd`/`bwd` now use `anchor_v` + cumulative-arc offsets, so `a_t→0` is landed only at run anchors.
- Result: deceleration into a clothoid rides `|a|=a_max` across the seam (disk-governed tangential→centripetal trade); the jerk limit no longer forces `a_t→0` before the curve.

### Verification (all green)

- `cargo nextest run -p geometry` — 271 pass (270 prior + new AC).
- New AC `velocity::tests::decel_into_clothoid_holds_acceleration_across_the_seam`: on a line→clothoid→line chain, tangential accel at the line/clothoid seam rides `< -0.5·a_max` (was ~0 before the fix).
- `cargo nextest run -p trajectory` — 438 pass incl. `continuous_throughput_repro` (no downstream/throughput regression).
- `./scripts/ci.sh rust-clippy` (`-D warnings`) clean; `rust-fmt` clean.
- All locked invariants intact: seam continuity, forward-backward accel-feasibility, `J=∞` recovers pure disk, limit-riding beats the constant-ceiling skeleton, determinism.

### Scope / remaining

- **Claim 2 (decel into clothoid):** fixed.
- **Claim 1 (jerk spike at interior STRAIGHT velocity peaks):** NOT addressed — the forward/backward `min` still cusps at a straight reversal. This is the deferred `peak_velocity`/apex reconstruction (`spec-motion-6` had it; the step-7 rework dropped it). Clothoid apex reversals are smooth (disk brings `a_t→0` at κ_peak), so corner blends are fine; only mid-straight peaks spike. Optional next task.
- Boundary note: run-anchored jerk uses the local move's `accel` in the cumulative reach — exact when accel is constant across the run (the common case), a reasonable bound under per-move accel changes.
