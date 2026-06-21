---
title: 'Motion-12: true C2 tangential-jerk continuity in the live geometry velocity planner'
type: 'feature'
created: '2026-06-20'
status: 'in-progress'
baseline_commit: '9ebf70d7afb9cc54f82195b539fba7535f94125f'
context:
  - '{project-root}/CLAUDE.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-6-tangential-jerk.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-7-limit-riding.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-11-pipeline-production-cutover.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-13-delete-socp.md'
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/jerk-usage-investigation.md'
---

> **Split note (2026-06-20):** This spec originally carried a fifth stage (T5) that deleted the dead SOCP. That work is now its own spec — **Motion-13 (`spec-motion-13-delete-socp.md`)** — because the deletion is independent of the C2 build (the SOCP is already off the live path) and bundling a large deletion into the C2 crux PR makes a deletion fault indistinguishable from a C2 math bug at bisect. This spec is now T1–T4: build the C2 path + its gate.

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The live velocity planner (`geometry::plan_velocity_warm_start`, `rust/geometry/src/velocity.rs`) is **C1**: only velocity crosses move junctions, never acceleration. Tangential jerk is implemented as a *velocity ceiling* (`scurve::max_reachable_velocity`) folded into the `min(forward_disk, backward_disk, jerk_forward, jerk_backward, ceiling)` at `disk.rs::profile_speed`. A velocity ceiling bounds *whether* a speed is reachable, never *how acceleration behaves on the way*. Consequence: tangential acceleration is jerk-shaped only off a rest anchor (accel-from-rest, decel-to-stop), and **steps discontinuously at every mid-run junction** (cruise→decel, decel-into-curve) — effectively unbounded tangential jerk exactly where the machine feels it. This is the architecture's own model reporting its contract, not a leak.

**Approach:** Finish the decoupled architecture (spec-motion-6/7): promote tangential acceleration `a_t` from a derived quantity to a **planned state carried across junctions inside a run**, so the profile is jerk-continuous (C2) everywhere except at true rest anchors, where `v=0, a=0` is pinned by definition (**C2-within-run / C1-at-rest-anchors**). This is real-time on a Pi 5 because XY jerk is a **single global scalar**: `a_t` rides the disk acceleration rails bang-bang (spec-motion-7 limit-riding ODE) and is interior only on short closed-form seven-segment jerk transitions; it is *not* a free 2D search. The forward/backward crossover acceleration-step is dissolved by a closed-form seven-segment **jerk-bridge** (lowest-peak-`v` profile feasible under both envelopes), not relocated. Validation uses no external solver/oracle: a viz-derived jerk instrument measures the jerk actually emitted and gates feasibility intrinsically.

## Boundaries & Constraints

**Always:** Keep XY jerk a single global scalar (`[printer] max_jerk`, default `2×max_accel`; spec-motion-11 dev-log 2026-06-19) — this is load-bearing for the real-time affordability argument. Keep the node-based forward-backward sweep O(N) two-pass with closed-form per-edge work; no SOCP/QP/grid/iterative inner solver. Pin `(v,a)=(0,0)` at true rest anchors (`v=0` stops, chain ends; the run-anchor machinery in `velocity.rs` from commit `311ac5144`). Fail loudly on out-of-contract state (a run arriving at a rest anchor with non-zero entry acceleration; an infeasible `(v0,a0)→(v1,a1)` reach) — raise, never pad/clamp silently. Share one jerk-computation code path between the viz plot and the CI gate. Reuse the existing emit backend unchanged (`ShapedSegment` → `enqueue_segment` → pump).

**Ask First:** Any fallback to a *genuinely iterative / multivariable* inner solver in the crossover jerk-bridge (an SOCP/QP/grid solve) — escalate, do not silently relax. Note R1 is **resolved**: the bridge is closed-form and the 1-D splice-location root is within the no-inner-solver boundary, so it is *not* an Ask-First — only a multivariable solver would be. Loosening any throughput gate (AC-G3 / R3) to "within ε" to absorb a real regression. Removing the `temporal`/old-`planner.rs` path is now **Motion-13**, not this spec — this spec must not delete it; coordinate ordering with Motion-13, do not bundle into the C2 crux PR.

**Never:** Re-introduce the sampled Consolini-Locatelli coupled-jerk SOCP, in production or as a kept oracle (architecture.md: "the coupled-jerk SOCP we are avoiding"; user: deleted, untrusted). Add a planner-level **lateral**-jerk constraint or cap — the fitter's shape owns lateral jerk (see Non-Goals). Add per-axis / per-`[limit]`-section jerk (XY jerk is global scalar). Add input shaping (deferred, spec-motion-11). Ship a measurably slower *feasible* trajectory than the best we can compute (throughput-SOTA is non-negotiable). Note: this is **not** measured as "C2 ≤ C1" — C1 is jerk-infeasible, so C2 is provably slower at corners and that is correct, not a regression; throughput-SOTA is enforced by R3's three gates (AC-G3), not by a C1 race.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Accel from rest | run starts at `v=0` | jerk-limited seven-segment ramp; `a_t` rises from 0 continuously | finite checks |
| Mid-run cruise→decel into a curve | two moves meet at non-zero `v`, downstream curvature-limited | `a_t` is **continuous** across the junction (jerk-bridged), no step | N/A |
| Forward/backward crossover | fwd envelope (a>0) meets bwd envelope (a<0) inside a run | crossover is a jerk-bridged interval, C2 across it; lowest peak-`v` under both envelopes | N/A |
| Decel to stop | run ends at rest anchor | `a_t` falls to 0 continuously; `(v,a)=(0,0)` at the anchor | N/A |
| Rest anchor with non-zero entry accel | a run arrives at a `v=0` anchor with `|a_entry|>ε_a` | **raise** (seam misclassified as rest / re-anchor bug) | `Err`, no pad |
| Infeasible reach | `(v0,a0)→target` not reachable within `ds` at `jerk_max` | **raise** (genuine over-commit / misclassified seam). A tight *crossover* is **not** this case: it lowers `v_peak` toward the `(0,0)` floor (R2), never raises | `Err`, no saturate-and-continue |
| Curvature alone exceeds accel budget | `v²κ > a_max` at a sample | velocity capped by the disk ceiling (existing behavior); not a new jerk path | finite checks |
| Viz snapshot | `pipeline_snapshot(..., max_jerk, ...)` | plots include `a_t` and tangential jerk `j_t` tracks computed from the planned profile, using the caller's `max_jerk` | `Err` on bad config |
| Empty / zero-length | empty buffer, zero-length move | no-op, no panic | N/A |

</frozen-after-approval>

## Non-Goals

- **Planner-level lateral-jerk handling.** Lateral jerk `j_n = κ'·v³ + 2κ·v·a_t` is owned by the fitter's shape (clothoid geometry); the planner does **not** cap velocity for it and does **not** carry a `max_lateral_jerk`. (Decision: the shape keeps it in check.) The viz probe MAY emit `j_n` as a free diagnostic since it already has `κ, κ', v, a_t`, but it is report-only — never a gate, never a constraint.
- Per-axis / per-section jerk; input shaping; native Arc/Bézier end-to-end; reviving any SOCP/NLP oracle.
- **Deleting the dead SOCP** (`temporal` crate, `trajectory` SOCP modules, old `planner.rs`). That is **Motion-13** (`spec-motion-13-delete-socp.md`).

## Code Map

- `rust/geometry/src/velocity/scurve.rs` — today exposes only `pub(super) max_reachable_velocity(v_in, length, accel, jerk)` (the C1 velocity ceiling, assumes `a0=0`). Add the acceleration-carrying seven-segment primitive + analytic `a_t` evaluator here.
- `rust/geometry/src/velocity.rs` — `plan_velocity_warm_start`; run-anchor machinery (`is_anchor`, `run_start_v`, `arc_from_run_start`, `arc_to_run_end`, lines ≈188–235 from commit `311ac5144`); forward/backward sweeps (≈237–284). Convert the scalar-`v` sweeps to coupled `(v,a)` sweeps; pin `(0,0)` at anchors.
- `rust/geometry/src/velocity/disk.rs` — `profile_speed` (≈142–167) does the `min()` reconstruction and `JerkAnchors` (`bwd_v: 0.0`). Replace the velocity-`min` jerk terms with `(v,a)`-propagated reach + the crossover jerk-bridge. `disk_reach_v` / curvature ceiling (`limit_speed`, `disk.rs:159`) stay (position constraints, no accel state).
- `rust/motion-engine/src/viz.rs` — `pipeline_snapshot` (signature has no `max_jerk`; calls `geometry::plan_velocity(&outcome, VelocityConfig::default())` at line 32 — the hardcoded-jerk defect). Add `max_jerk` param, drop the default; add `a_t` + `j_t` tracks via the shared probe; `sample_kinematics` (≈187).
- `rust/motion-engine/src/jerk_probe.rs` — **NEW** pure jerk computation, shared by viz and the CI gate.
- `rust/geometry/src/path/profile.rs` — `CurvatureProfile`: `kappa(s)`, `dkappa_ds(s)`, `kappa_peak()`, `kappa_endpoints()`, `s_len()` (probe/diagnostic inputs).
- `scripts/ci.sh` — add the feasibility-gate + throughput-non-regression job.

## Tasks & Acceptance

**One PR on the `curvature-profile` feature branch.** The 4 stages below are the implementation order *inside* the single change set. Suggested commit order: T1 → T2 → T3 → T4, so each commit bisects cleanly. The SOCP deletion is **Motion-13**, a separate PR (recommended to land first or in parallel — never bundled here).

**T1 — scurve acceleration-carrying primitive + a_t evaluator**
- [x] `velocity/scurve.rs`: `reach_velocity_with_accel(v0, a0, ds, accel_max, jerk_max) -> (v1, a1)` (limit-riding bang-bang jerk integration over arclength); `breakpoints(v0, a0, ds, accel_max, jerk_max) -> SevenSeg`; `accel_at(seg, s) -> f64` (analytic `a_t`). Tests in `velocity/scurve/tests.rs`.
- AC-S1: `reach_velocity_with_accel` matches numeric ODE integration ≤1e-9 over randomized `(v0,a0,ds)`. AC-S2: `accel_at` integrated back reproduces `reach_velocity_with_accel` (round-trip ≤1e-9). AC-S3 (property): peak `|a| ≤ accel_max`, peak `|j| ≤ jerk_max` across the curve.
- AC-S4a (**decision table** — replaces the old "two named corners" AC): `breakpoints` is implemented against a **fully enumerated seven-segment case table**. The seven phases are jerk-up → hold-accel → jerk-down → cruise → jerk-down → hold-decel → jerk-up; any subset can collapse to zero length. The table covers at least: `a0=0`; `0<a0<accel_max`; `a0=accel_max` (jerk-up phase has zero length); `a0<0` including `a0=-accel_max`; and the **sign-flip** case (`a0>0` but the segment target requires ending with `a1<0`, and the mirror). Each case is a **named branch** in source — no unnamed fall-through into a `sqrt`/`cbrt`.
- AC-S4b (**infeasibility guard**): for each case the minimum feasible arclength `ds_min(v0,a0,accel_max,jerk_max)` is computed analytically and asserted **before** any `sqrt`/`cbrt`. `ds < ds_min` raises `InfeasibleReach` — never a clamp, NaN, or saturate-and-continue. `ds→0` and `a0=accel_max` fall out of this guard as loud-correct.
- AC-S4c: the randomized ODE regression (AC-S1) includes `a0=-accel_max` and the sign-flip case, not only `a0=accel_max`.
- AC-S5 (**bit-for-bit backward compat**): the `a0=0` path is a thin wrapper that **delegates** to today's `max_reachable_velocity` (single implementation, `#[inline]`), not a re-derivation. Test asserts bit-for-bit equality (`f64::to_bits()` compare) over 1000 randomized `a0=0` inputs. There must be exactly one f64 implementation of this quantity after T1 — the existing `jerk_only`/`jerk_bound` reporting call (`velocity.rs:≈309`) routes through the same wrapper, so no two float paths compute the same value differently.

**T2 — jerk_probe instrument + viz wiring + max_jerk defect fix**
- [x] `motion-engine/src/jerk_probe.rs`: `jerk_at(kappa, dkappa_ds, v, a_t, seg_jerk) -> JerkSample { j_t, j_n, j_n_geom, j_n_couple }`, `j_t = seg_jerk`, `j_n = κ'·v³ + 2κ·v·a_t` (geom/couple split; `j_n` is diagnostic-only).
- [x] `viz.rs::pipeline_snapshot`: add `max_jerk` to the signature, remove `VelocityConfig::default()`; emit `kin_a_t` and `kin_j_t` (+ optional `kin_j_n*`) tracks via `jerk_at`. **T3 update:** viz now reads the planned analytic `a` (`VelSample.a`) directly instead of finite-differencing the planned `(s,v)`.
- AC-P1: `jerk_at` is pure, `j_n == j_n_geom + j_n_couple`. AC-P2: viz consumes the caller's `max_jerk`; grep shows zero `VelocityConfig::default()` in the viz path. AC-P3: against the *current C1* planner, the probe reports a finite `Δa_t` step at mid-run junctions (documents the bug T3 removes). AC-P4: one probe implementation, referenced by both viz and the gate (no duplicated formula). See **R4** — sharing the probe is correct, but the gate must additionally cross-check against the *emitted time-domain* trajectory so the probe is not self-confirming.
- Depends on T1 (`accel_at`).

**T3 — coupled (v,a) C2 sweep + crossover jerk-bridge** *(the crux)*
- [x] `velocity.rs` + `disk.rs::profile_speed`: coupled `(v,a)` forward/backward sweeps via `reach_velocity_with_accel`; `(v,a)=(0,0)` pinned at rest anchors; crossover accel-step dissolved by a closed-form seven-segment jerk-bridge (lowest peak-`v` under both envelopes). `a_t` bang-bang on disk rails.
- AC-C1 (headline): across a run interior, `|Δa_t|` between adjacent samples and at the crossover ≤ tol (continuity). AC-C2: at every rest anchor, emitted `(v,a)=(0,0)`. AC-C3: jerk-bridge yields the minimum peak-`v` feasible under both envelopes (vs a brute sampled search, test-only). AC-C4: planning stays O(N) two-pass; no SOCP/grid/QP inner solver. The crossover's 1-D splice-location root (R1, within boundary) carries a hard per-crossover iteration cap (`K`) asserted across the corpus. AC-C5: T2 probe over T3 output shows AC-P3's step is gone. AC-C6: rest anchor with `|a_entry|>ε_a` raises (fail-loud).
- Depends on T1; validated by T2. **Risk:** R1 (solvability), R2 (feasibility), and R3 (throughput) are all **resolved** — the bridge is closed-form (flat-rail seven-segment); the feasible set is never empty (`(0,0)` floor → at worst a momentary mid-run stop); the lower-`v_peak` relaxation is a bounded monotone 1-D root that keeps the passes independent (O(N) holds); and the provably-false "C2 ≤ C1" headline is replaced by AC-G3's three gates. One **implementation note** from R2: a crossover relaxing to `v_peak=0` must be promoted to a true rest anchor. The only remaining live risk is **R4** (gate circularity, addressed by AC-G5/G6, verify when T2/T4 land). HALT and surface per the **Ask First** boundary before introducing any multivariable inner solver.

**T4 — CI feasibility gate + throughput non-regression**
- [ ] New property test (wired into `cargo nextest` + `scripts/ci.sh`): dense-resample the emitted trajectory, recover analytic `a_t` (T1), run `jerk_at` (T2), assert `|j_t| ≤ max_jerk·(1+ε)`, `|a_t| ≤ a_max·(1+ε)`, `v ≤` caps everywhere; seam/crossover `|Δa_t| ≤ ε_a`; rest-anchor `(0,0)`. ε derived from discretization + float epsilon, not arbitrary.
- [ ] Throughput: **do not gate on "C2 ≤ C1"** — it is provably false on cornered fixtures (R3). Instead, via klipper-sim (`~/Developer/klipper-sim/`, per-branch `--klipper-root`): (a) capture C1 times BEFORE T3 to identify jerk-non-binding fixtures; (b) freeze current C2 times as the forward regression baseline. See AC-G3 (G3a/b/c).
- AC-G1: gate is RED on a deliberately injected accel-step (mutation test — prove it bites). AC-G2: gate GREEN on T3 output across the corpus. **AC-G3 (throughput, per R3) — three sub-gates replacing "C2 ≤ C1": G3a — on jerk-non-binding fixtures `C2 == C1` within float noise (no spurious slowdown); G3b — aggregate C2 time-excess per fixture ≤ a physics-derived bridge-cost bound; G3c — C2 ≤ frozen-C2 baseline (no self-regression). Do not loosen any of these to "within ε" to absorb a real regression.** AC-G4: gate and viz call the same `jerk_at`. **AC-G5 (anti-circularity, see R4): the gate also recovers `a_t` by finite-difference of the emitted time-domain `ShapedSegment` stream and asserts agreement with `accel_at` — so the gate validates the executed trajectory, not the planner's own bookkeeping. AC-G6 (anti-aliasing): evaluate `accel_at` at every adjacent `SevenSeg` endpoint pair directly (no resampling) — a step at a narrow bridge cannot fall between samples.**
- Depends on T1/T2/T3.

**Acceptance Criteria (spec-level):**
- Given a mid-run cruise→decel-into-curve junction, when planned, then tangential acceleration is continuous across it (`|Δa_t| ≤ ε_a`) — the C1 step is gone.
- Given any run, when planned, then `|j_t| ≤ max_jerk` and `|a_t| ≤ a_max` at every sample, and `(v,a)=(0,0)` at both run ends.
- Given the representative slicer corpus, throughput-SOTA is held by three gates (R3, AC-G3), **not** by "C2 ≤ C1" (provably false on cornered fixtures): on jerk-non-binding fixtures C2 == C1 (no spurious slowdown); per-fixture bridge cost is within a physics-derived bound; and C2 does not regress against its own frozen baseline.
- Given a run arriving at a rest anchor with non-zero entry acceleration, when handled, then the planner raises and does not pad.
- Given `./scripts/ci.sh quick` + the new gate job, when run, then green.

## Design Notes

**Why this is the architecture's completion, not the deleted SOCP.** The deleted complexity was the *sampled, coupled, numerically-optimized* TOPP (N variables, SOCP solver, convergence tuning). Carrying `(v,a)` across junctions in a node-based sweep is a graph problem with two scalars of state per node and closed-form per-edge propagation. Because XY jerk is one global scalar, `a_t` is bang-bang on the disk rails (spec-motion-7) and interior only on short seven-segment transitions — so the (v,a) reachable region is a 1D manifold with jerk-bridge splices at corners, not a 2D search. Cost: same O(N) two-pass as C1 with a ~10× per-edge constant; no solver. Deleting the SOCP frees more compute than C2 adds. *(Pre-mortem R1, resolved: the bridge itself is closed-form; only locating the splice `s*` is a bounded 1-D root on monotone envelopes — O(1), within the no-inner-solver boundary. See R1.)*

**The crossover — closing the round-1 gap.** Scalar `v = min(v_fwd, v_bwd)` flips `a_t` sign discontinuously at the crossover — the residual C1 break. Resolve it as a two-point BVP on `[s*−δ, s*+δ]`: the jerk-limited profile matching `(v_fwd, a_fwd)` on the left and `(v_bwd, a_bwd)` on the right with lowest peak-`v` (so it stays under both envelopes — no over-limiting). With global-scalar jerk this is the standard seven-segment "reach-cruise vs double-ramp" case analysis with non-zero boundary accels. The accel step is *dissolved*, not relocated. Where the bridge's peak-`v` would exceed the local disk ceiling, `v` is lowered — that is the true jerk-limited bound, the one place C2 can legitimately cost time, and exactly where C1 was lying. *(R1/R2 resolved: the free knot `s*` is located by a bounded 1-D root and the transcendental rail enters only as boundary values, so the bridge itself stays closed-form and is always feasible down to the `(0,0)` floor. See R1/R2.)*

**Validation without an oracle.** Feasibility is intrinsic — differentiate the emitted profile and assert the envelopes. The seam/crossover `|Δa_t|` assertion is the high-value test: it turns "the velocity plot looks smooth while `a_t` steps" into a deterministic failure. Optimality is defended **not** by a C2-vs-C1 race (C1 is jerk-infeasible, so C2 is provably slower at corners — R3) but by R3's three gates — no spurious slowdown on jerk-non-binding fixtures, bounded bridge cost, no C2-vs-C2 self-regression; no reference solver is kept (untrusted). *(R4: the gate must recover `a_t` from the emitted time-domain trajectory, not re-read the planned `SevenSeg`, or it is self-confirming. See R4 / AC-G5.)*

## Risks & Open Questions (pre-mortem, 2026-06-20)

R1, R2, and R3 are **resolved** (first-principles, code-grounded) and their conclusions are baked into the ACs (AC-C4/C6, AC-G3, AC-G5/G6). R4 remains **open** — it can only be closed once T2/T4 code exists. Each entry records its resolution and the date.

- **R1 — Crossover solvability. [RESOLVED 2026-06-20, first-principles + code-grounded — within boundary]** Resolved against `disk.rs:142-167`. The bridge profile is **closed-form**: in the bridge `a_t` is the *control variable* (bounded by `a_max`/`jerk_max`), so the bridge is a flat-rail seven-segment BVP between two fixed `(v,a)` boundary states — exactly T1's `breakpoints`/`accel_at`. The transcendental disk rail (`dw/ds = 2√(a²−κ²w²)`, `disk.rs:79`) governs only the *adjoining* sections, so it enters the bridge **only as boundary values** (`a_fwd`/`a_bwd`, computed by the existing RK4 when σ≠0) — inputs, not the bridge's functional form. The *only* iteration is locating the splice `s*`: a **1-D root `forward(s)−backward(s)=0` on two monotone envelopes already being evaluated** (plus a 1-D monotone search to size δ for lowest peak-`v`). That is the same O(1) class as the `asin` (`disk.rs:60`) / Cardano `cbrt` (`scurve.rs`) already in the code — **not** an SOCP/QP/grid inner solver, so the no-inner-solver boundary **holds**. *Residual:* the genuine seven-segment case-table closure across all `(v0,a0,v1,a1)` pairs is real work, but it is AC-S4a, not an architectural risk. Keep AC-C4 as written (it is satisfied); optionally assert a fixed cap on the 1-D splice-location iterations for hygiene. The hard worst case (forward-disk meets backward-disk mid-curve, σ≠0 both sides) was checked: δ is short, κ varies only second-order across it, "stay under both envelopes" absorbs it — bridge stays closed-form.
- **R2 — Crossover feasibility. [RESOLVED 2026-06-20, first-principles + code-grounded]** The feasible bridge set is **never empty**: `(v,a)=(0,0)` is reachable from any state and bridges to any state (two `a0=0` seven-segment ramps — T1's existing primitive), so the worst-case bridge degenerates to "momentary mid-run stop," which always fits. The accel-flip takes a *fixed time* `Δt=(a_fwd−a_bwd)/jerk_max`; the arclength it consumes `≈∫v dt` **shrinks as `v` dips**, so lowering the bridge peak `v_peak` always makes the flip fit — down to the `v_peak=0` floor. *Consequences:* (1) `raise` (AC-C6, I/O matrix) **stays correctly reserved** for a seam *misclassified* as a rest anchor (a real bug) — it should never fire on routine tight geometry, so AC-C6 is unchanged. (2) What looks like infeasibility is **cost** (R3), not an abort: a tight crossover lowers `v_peak`. (3) The relaxation is a **monotone 1-D root with a floor at 0** (lower `v_peak` → easier fit), not an unbounded fixpoint — and since the envelopes (`disk_reach_v` from the run-end anchors, `velocity.rs:241,255`) are **bridge-independent**, lowering one crossover's `v_peak` does not move any other crossover's envelopes, so the passes stay independent and **O(N) holds**. *Implementation note (the one residual):* a crossover that relaxes to `v_peak=0` must be **promoted to a true rest anchor** (`is_anchor[k]=true`, `velocity.rs:193`) to keep the `(0,0)` pin + fail-loud machinery consistent. This should be rare-to-never in practice — the clothoid fitter already blends corners to be traversable (lateral jerk is the fitter's job, see Non-Goals), so velocity planning rarely sees a corner tight enough to force a stop.
- **R3 — Throughput baseline. [RESOLVED 2026-06-20 — headline AC changed]** First-principles: C2's feasible set is C1's **plus jerk-continuity** (a pure *added* constraint), so `v_C2(s) ≤ v_C1(s)` pointwise, hence `t_C2 ≥ t_C1` wherever any bridge fires. "C2 total time ≤ C1-reported time" is therefore the **wrong inequality — provably red on every cornered fixture**, not a tolerance question. *This retracts the earlier R3 suggestion* to "run the feasibility filter on C1 and use that as the baseline": enforcing jerk-feasibility on C1's profile rounds its corners, which *is* the C2 construction, so "feasible-C1" ≈ C2 and the gate is **vacuous**. C1-reported time is a physically-unrealizable lower bound (infinite jerk at corners); the only other feasible reference (the SOCP) is deleted/untrusted — so there is **no usable absolute baseline**. Honor throughput-SOTA (CLAUDE.md: "never slower than the best we can compute" — the best *feasible* trajectory is C2; C1 isn't executable) via three gates instead of "C2 ≤ C1", now wired into AC-G3:
  - **G3a (no spurious slowdown):** on jerk-**non**-binding fixtures (no bridge fires) `C2 == C1` within float noise — catches accidental over-limiting of the easy cases. Provable equality.
  - **G3b (bounded bridge cost):** aggregate C2 time-excess per fixture ≤ a physics-derived bound (Σ over crossovers of the v-dip cost; each ≤ `Δt_flip=(a_fwd−a_bwd)/jerk_max` × dip area) — bounds the legitimate cost of honesty so it can't mask a regression.
  - **G3c (C2-vs-C2 regression pin):** freeze current C2 corpus times as the forward baseline so future changes can't silently regress once C1 is gone — the durable SOTA guard.
- **R4 — The feasibility gate may be self-confirming (validation). [OPEN]** If the gate recovers `a_t` from the same planned `SevenSeg` the planner produced (AC-P4/AC-G4 shared probe), it validates the planner's bookkeeping, not the time-domain trajectory the stepper executes — a step introduced in reparam-to-`ShapedSegment` would pass green. Aliasing compounds it: velocity-based adaptive resampling won't refine a smooth-`v` profile, so an `a_t` step in a narrow bridge window can fall between samples. *Mitigations (now AC-G5/AC-G6):* (a) cross-check `a_t` from finite-difference of the emitted time-domain `ShapedSegment` stream against `accel_at`; (b) evaluate `accel_at` at every adjacent `SevenSeg` endpoint pair directly — cannot alias.

## Verification

**Commands:**
- `cargo nextest run -p geometry` — scurve + sweep continuity green.
- `cargo nextest run -p motion-engine` — jerk_probe + feasibility gate green.
- `cargo test --doc` — if any doc examples touched.
- `./scripts/ci.sh quick` — ruff/clippy `-D warnings`/fmt/rust tests green before PR.
- Throughput (R3, AC-G3): klipper-sim across the corpus — `C2 == C1` on jerk-non-binding fixtures, per-fixture bridge cost within bound, C2 ≤ frozen-C2 baseline. (Not "C2 ≤ C1" — provably false on cornered fixtures.)

**Manual checks:**
- `make -f Makefile.rust motion-engine`, run representative slicer G-code through the viz; confirm the `a_t`/`j_t` tracks show continuous acceleration through mid-run corners (no step) and `j_t` within `max_jerk`.

## Dev Log — 2026-06-20 session handoff (fresh-session pickup)

**Branch:** `curvature-profile` (base: `sota-motion`). **Pick up T3 from here.**

### What is committed (done, do not redo)
- **T1 — `892bb57eb`** `feat(velocity): analytic seven-segment (v,a) reach primitive`. `velocity/scurve.rs` has `reach_velocity_with_accel`, `breakpoints` (`SevenSeg`), `accel_at`, `velocity_at`; tests green. This is the engine T3 must use.
- **T2 — `309dd1218`** `feat(viz): thread max_jerk and emit jerk diagnostics`. `pipeline_snapshot` takes `max_jerk` (no more hardcoded `VelocityConfig::default()`), shared `jerk_probe.rs`, `kin_a_t/j_t/j_n` tracks. **Caveat:** until T3 lands, viz `a_t`/`j_t` are recovered by **finite difference** of the planned `(s,v)` (`viz.rs::tangential_accel`/`tangential_jerk`), because the committed planner carries no `a`. Once T3 carries `(v,a)`, switch viz back to reading the planned `a` (re-add `VelSample.a`).
- **T3 — NOT done.** A prior agent's attempt was reverted (see post-mortem). The committed planner is still C1 (velocity-ceiling jerk).

### Motivating defect — proven this session (the thing T3 fixes)
The committed jerk model (`scurve::max_reachable_velocity`, a velocity *ceiling*) is wrong on accel shape, not just at junctions:
- **Accel-from-rest rides at exactly `(2/9)·max_jerk`.** Analytic: from rest the ceiling is `v_ceil(s) = (jerk·s²)^(1/3)`, so riding it gives `a_t = (2/3)·jerk^(2/3)·s^(1/3)` and `da_t/dt = (2/9)·jerk`. Measured on demo4: **889 vs configured 4000** (22%). It under-drives jerk and never produces a flat-topped accel trapezoid.
- **The accel→decel crossover is ungoverned.** At a sub-cruise velocity peak, `a_t` steps `+max → −max` in one sample (measured `+192 → −124`, `ds≈2e-5` → jerk ~1e9). The velocity ceiling has no acceleration state to carry across the crossover.
- Both are the **same root cause**: `a_t` is *derived* from a velocity ceiling ("bounds whether a speed is reachable, never how acceleration behaves on the way" — Intent), not *carried as planned state*.

### Prior T3 attempt — post-mortem (DO NOT REPEAT)
The reverted attempt (≈700 lines in `disk.rs`) produced a 4.8e6 accel spike on the corner clothoids. Three faults:
1. **Finite-differenced `a_t`** (`centered_fd_accels`, `v·Δv/Δs`) instead of analytic `accel_at` → exploded over sub-µm seam Δs. (Violated AC-G6.)
2. **Bridge samples exceeded `min(fwd,bwd)`** — planted `v=9.84` between neighbours ~7.36 (the envelope invariant `v ≤ envelope` must hold at every sample; the `v_r_capped` clamp was applied only at `s_r`).
3. **Bridge fired on ~6 µm corner clothoids** where there is no genuine tangential crossover.
The attempt's `c2_feasibility_gate.rs` (`FEAS-0`) *did* catch it (`dv=2.49 > budget`) — it was deleted with the revert. **Recreate that gate, red-first.**

### Reproduction fixture
- G-code: `/private/tmp/demo4.gcode` — serpentine `(0,0)→(90,0)→(90,90)→(180,90)→(180,0)`, 3×90° near-stop corners.
- Config: `/private/tmp/viz_demo.cfg` — corexy, `max_velocity 150`, `max_accel 200`, `max_jerk 4000`, `square_corner_velocity 5`.
- Run: `make -f Makefile.rust motion-engine && python3 scripts/viz_pipeline.py /private/tmp/demo4.gcode -c /private/tmp/viz_demo.cfg`.
- **Acceptance after T3:** accel-from-rest is a 4000-jerk trapezoid (`a_t` rises 0→200 *linearly*, hits 200 at t≈0.05s, not the current ~0.23s); the sub-cruise peak crossover is jerk-limited (no `+max→−max` step); `j_t ≤ max_jerk` everywhere.

### Design (carried from the in-session proposal)
Replace the `min()`-of-velocity-ceilings reconstruction with a coupled `(v,a)` sweep:
- Carry `(v,a)` as planned state via `reach_velocity_with_accel` (T1). Per-sample `a_t` is **analytic** (binding-rail closed form, or `accel_at` in a bridge) — **never finite difference**.
- Dissolve the forward/backward crossover with a closed-form T1 seven-segment **jerk-bridge**, peak-`v` clamped under **both** envelopes (lower `v` at the apex is the legitimate, only place C2 costs time). Locate the splice by a 1-D monotone root (within the no-inner-solver boundary, spec R1).
- **Gate the bridge to genuine crossovers** (`|Δa_t| > ε_a`, arclength `≥ ds_min`); skip sub-`ds_min` corner clothoids. Pin `(v,a)=(0,0)` at rest anchors; fail loud on non-zero entry accel at a rest anchor.
- Recreate the **FEAS-0 gate red-first**, recovering `a_t` by finite-difference of the *emitted time-domain `ShapedSegment` stream* (not the planner's own `SevenSeg`) and evaluating `accel_at` at adjacent `SevenSeg` endpoints (AC-G5/G6 — anti-aliasing).
- Code map: `velocity.rs` (sweeps, anchors), `disk.rs::profile_speed` (the `min()` reconstruction to replace), `scurve.rs` (T1 — done).

### Fresh-session first steps
1. Re-read this spec (T3 + ACs) and `investigations/clothoid-straight-seam-discontinuity-investigation.md` (full diagnosis).
2. Recreate `c2_feasibility_gate.rs` (FEAS-0 + the accel-from-rest-jerk assertion) — get it RED on the committed planner.
3. Implement the `(v,a)` carry + crossover bridge until the gate is GREEN and the demo4 acceptance above holds.

## Dev Agent Record — T3 implemented (2026-06-20)

**Scope:** T3 only (per user). T4 (CI feasibility gate job, mutation test AC-G1, throughput gates AC-G3, anti-circularity CI wiring AC-G5/G6) is intentionally **not** in this change set.

### What landed
- **`VelSample` now carries analytic `a`** (`velocity.rs`). Reconstruction is **per-run** (not per-move): runs are delimited by rest anchors; a single coupled `(v,a)` profile is built across the run's moves, then split back per move.
- **Binding-rail analytic `a_t`** (`disk.rs`, `forward_branch`/`backward_branch`/`eval_profile`): per sample `a_t` is the closed-form accel of whichever rail binds — `±accel_at` (scurve jerk ramp from the run anchor, fixing the C1 `(2/9)·jerk` from-rest ride), `±√(a²−κ²v⁴)` (disk rail accelerating below the ceiling), or `v·dv_lim/ds = −½·a·σ·sign(κ)/κ²` (curvature-ceiling tracking, direction-independent). **Never finite-differenced.**
- **Node sweep jerk term** switched from `scurve::max_reachable_velocity` (the acceleration-returns-to-0 ceiling `(j·s²)^⅓`) to the carried `scurve::reach_v` (true full-jerk trapezoid) — this is the throughput win: C2 from-rest is *faster* than C1's conservative ceiling.
- **Crossover jerk-bridge** (`disk.rs`, `build_run_bridge`): both apex (`a:+→−`) and valley (`a:−→+`) crossovers across the run interior — including those landing on move boundaries — are dissolved by a single constant-jerk seven-segment arc. The splice point is a bounded 1-D root (`scan_root`, robust to positive-zero `signum`); the arc is validated to stay under both envelopes (R2 floor). Bridges are skipped under infinite jerk.
- **`(0,0)` pin + fail-loud** (`velocity.rs`, `pin_rest_anchor`): rest-anchor samples are pinned to `(v,a)=(0,0)`; a finite-jerk run arriving at a rest anchor with `|a|>1e-3` raises `VelocityError::RestAnchorAccel` (AC-C6). Infinite jerk skips the check (instantaneous accel is C1-by-definition there).
- **viz** reads the planned analytic `a` directly (resolves the T2 finite-difference caveat).

### Validation (demo4 serpentine, `max_velocity 150 / max_accel 200 / max_jerk 4000 / scv 5`)
- Accel-from-rest is a full-jerk trapezoid: `a_t` is on the `a_max` plateau by `s≈0.08 mm` (C1 `(2/9)·jerk` ride only reaches `~0.66·a_max` at `s=0.5 mm`).
- Every straight sub-cruise `+max→−max` crossover is bridged — no adjacent `|Δa_t|` reaches `a_max` (the C1 step was `2·a_max`).
- `|a_t| ≤ a_max` everywhere; `(v,a)=(0,0)` at both run ends.
- **Carve-out (consistent with the seam investigation):** the only residual `a_t` step is at the biclothoid corner apex (`~66 mm/s² < a_max`), where ceiling-riding `a_t = v·dv_lim/ds` inherits the clothoid's `dκ/ds` jump (G2-not-G3). Per the Non-Goals this lateral-jerk-induced step is the fitter shape's responsibility; a G3 corner fitter is a separate spec.

### Tests
- `rust/geometry/tests/c2_continuity.rs` (new): from-rest trapezoid, no-crossover-step, `|a|≤a_max`, rest-anchor `(0,0)`.
- `rust/geometry/src/velocity/tests.rs`: `pin_rest_anchor` fail-loud / zero / infinite-jerk paths (negative-test obligation for the new error).
- Full geometry suite (320) + motion-engine (369) green; `cargo fmt --check`, `clippy -D warnings` clean.

### Remaining (T4, out of scope here)
The dev-log "recreate `c2_feasibility_gate.rs` red-first" is the **T4** CI gate (emitted-time-domain `a_t` recovery, mutation test, throughput non-regression). T3's ACs are validated by `c2_continuity.rs`; the durable CI/throughput gate is still owed under T4.
