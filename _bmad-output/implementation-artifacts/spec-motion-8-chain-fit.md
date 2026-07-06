---
title: 'Build step 8 — global continuous-κ chain fit: faceted-arc detection + transition-spiral reconstruction (clothoid→arc→clothoid) replacing the per-corner sawtooth across runs of co-circular facets; per-corner biclothoid retained as the fallback'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: 'd28fa2ce0fdd5a9d34004fb962df4545e10a816a'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-4-corner-fitter.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-7-limit-riding.md'
  - '{project-root}/_bmad-output/implementation-artifacts/deferred-work.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The step-4 fitter blends every corner **independently**, reserving only **half** each leg's arclength and ramping κ `0→κ_peak→0` per junction. On a slicer's faceted arc — a smooth curve emitted as a run of many short `Line` facets (Klipper #4228) — this makes κ(s) a **sawtooth**: it snaps back to 0 between facets, and the tiny half-leg budget forces *tight* blends (large `κ_peak`), so the step-7 limit-rider dips to `√(a/κ_peak)` at every facet apex. The toolhead crawls through what is geometrically one gentle arc. Build-sequence step 8, "the global continuous-κ chain fit."

**Approach:** Add a **chain-aware** fitter entry `fit_chain(&[Move], ChainFitConfig) -> FitOutcome` (IR→IR, stateless). It detects **maximal runs** of consecutive short `Line` facets that turn the same direction and are **co-circular** within tolerance, and replaces each run with one **transition-spiral reconstruction** `Clothoid(κ:0→κ_arc) · Arc(κ_arc) · Clothoid(κ_arc→0)`: κ rises once, holds at the gentle `κ_arc = 1/R`, falls once — **G2 continuous through the whole run** and `κ=0` at both run boundaries. The arc body cruises at `√(a·R) ≫ √(a/κ_peak)`, so chain speed rises and the residual-κ cap stops binding at every facet. Isolated corners, non-circular runs, and over-budget runs **fall through to the unchanged step-4 per-corner biclothoid** — `fit_corners` is retained verbatim as the measurement baseline.

## Boundaries & Constraints

**Always:**
- Internal-to-`geometry::fitter` only. `fit_corners`, its `biclothoid` solver, and all step-1/2/3 stored types (`path::{Line,Arc,Clothoid,Segment,PathSegment}`, `frontend::Move`, `VelocityLimits`, lowering) are **byte-for-byte untouched**. New code is additive: a `chain` submodule + the `fit_chain` entry + re-exports. `velocity`, `path`, `frontend`, `segment`, `gcode`, `trajectory` untouched.
- Output is again a `Move` chain consumed unchanged by `plan_velocity`: a reconstructed run emits `[trimmed Line, Clothoid up, Arc, Clothoid down, trimmed Line]`; **no `FitReport.unblended` entry is pushed for an internal run seam** (so the velocity planner never pins `v=0` mid-run — internal clothoid↔arc↔clothoid seams are κ-continuous, not stops).
- **Run = ≥2 consecutive junctions** each: both sides spatial `Line`; `θ ∈ (θ_min, θ_max)`; **same turn direction** (turn-normal cross-product sign constant across the run); and whose **vertices are co-circular** — least-squares circle fit over the run vertices with max radial residual `≤ cocircular_tol` (default a small multiple of the per-junction δ). A run of <2 such junctions is **not a chain** → its corner(s) take the step-4 per-corner path.
- **Reconstruction (per run):** fit center `O`, radius `R` (⇒ `κ_arc = 1/R`) to the vertices in the run's tangent-spanned plane (`u,v`). Entry/exit **transition clothoids** ramp κ linearly `0⇄κ_arc` over `L_t = min(√(24·R·δ), end_leg_budget)` (road-spiral shift `p = L_t²/(24R) ≤ δ` keeps the reconstruction inside the deviation tube); `σ = κ_arc/L_t`. The middle `Arc(O,u,v,R,…)` carries the residual sweep between the two spiral ends, same turn sign. `δ = min` per-junction `SCV²(√2−1)/a` over the run (reuse `junction_deviation`). End-leg trim budgets = half the outer legs' arclength (identical reservation rule to step 4 → independent runs/corners never overlap).
- **Fit-or-fall-through, never fail-loud on geometry:** if the circle fit residual exceeds `cocircular_tol`, or `L_t` underflows the budget, or any spiral/arc shift would exceed δ, or `Arc/Clothoid::try_new` would reject — the run is **abandoned**: each of its junctions falls through to the step-4 per-corner classifier (which itself blends-or-leaves-sharp). A reconstructed run increments a new `FitReport.chains` counter and `blended` is **not** double-counted for its internal junctions.
- **Seam exactness via the lowering oracle** (step-4 AC-1 discipline): `up_clothoid.point_at(0)` = trimmed-entry-leg end with heading `t_in`; `arc.point_at(0)` = `up_clothoid.point_at(L_t)` with matched heading **and** κ=κ_arc; `down_clothoid.point_at(L_t)` = trimmed-exit-leg start with heading `t_out`. Verified in tests against `PositionProfile`.
- **Extrusion conservation:** the run's reconstructed segments inherit follower axes from their covered source facets; follower **ratios are rescaled** so total ΔE over the run equals the pre-fit ΔE (the step-4 `trim/L` discipline generalized to the run's source-arclength↔reconstructed-arclength ratio).
- Fail loud only on a genuine internal invariant break (a `try_new` rejection on a segment the fit math deemed valid, a non-finite computed quantity) via the existing `FitError::Internal { line_no, source }`. Unit tests in a separate file.

**Ask First:**
- **Variable-κ (non-circular) clothoid splines** — a run whose vertices are *not* co-circular (a general faceted spline with monotonically varying κ) reconstructed as a true Bertolazzi–Frego G2 clothoid spline. V1 detects only **co-circular** runs and abandons the rest to per-corner. The general spline is the clean next add.
- **Arc-incident runs** (a slicer-emitted `G2/G3 Arc` adjacent to facets, or merging across an explicit `Arc`) — V1 runs are pure `Line`-facet chains; an `Arc` move breaks a run exactly as a non-spatial move does.
- Moving chain thresholds (`min_run_junctions`, `cocircular_tol`, the spiral-length rule) onto a user knob, or any change to a step-1/2/3 stored type or the `plan_velocity` read-contract.

**Never:**
- No velocity solver, S-curve, speed cap, or timing — geometry only (step 5+ owns velocity). No capping accel for ringing / consulting a shaper.
- No editing `fit_corners` or `biclothoid` behavior; no `NURBS`/`CubicSegment`/legacy fit path; no new `Segment` variant (reconstruct into the existing `Line`/`Arc`/`Clothoid` alphabet only).
- No fail-loud on a run that won't reconstruct — that is a normal fall-through to per-corner.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Faceted arc, slack | ≥3 short co-circular same-turn `Line`s | one `Clothoid·Arc·Clothoid`; κ continuous 0→κ_arc→0; `chains==1`; no internal unblended entry | N/A |
| Isolated 90° corner | two long `Line`s, one junction | step-4 per-corner biclothoid (unchanged); `chains==0`, `blended==1` | N/A |
| Two facets only | 2 co-circular junctions min | reconstructed if ≥`min_run_junctions` (default 2) else per-corner each | N/A |
| Non-co-circular run | facets with varying κ (residual > tol) | run abandoned → each junction per-corner (blend or sharp) | N/A |
| Mixed turn direction | facets that switch turn sign | run splits at the sign change; each same-turn sub-run evaluated independently | N/A |
| Tight faceted arc, small R | budget-bound `L_t` | spiral shortened to `end_leg_budget`; if shift would exceed δ → abandon run → per-corner | N/A |
| Arc / virtual move inside | `Arc` or `spatial=None` mid-chain | breaks the run; segments on each side evaluated separately | N/A |
| Collinear within a run | one interior θ ≤ θ_min | not a corner; the co-circular run continues across it (still one arc) | N/A |
| Reconstruction faster | faceted arc with slack | run-time(reconstructed) < run-time(`fit_corners` output) via `plan_velocity` | N/A |
| Single move / empty | `len ≤ 1` | returned unchanged | N/A |
| Internal break | fit-valid `Arc`/`Clothoid` rejected by `try_new` | typed `FitError::Internal` with source line | `FitError` |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/fitter.rs` — **add** `pub fn fit_chain(&[Move], ChainFitConfig) -> Result<FitOutcome, FitError>` + `ChainFitConfig { corner: CornerFitConfig, min_run_junctions, cocircular_tol }` (Default-constructible); `FitReport` gains `pub chains: u32`. Walk junctions; group maximal co-circular same-turn `Line`-facet runs; reconstruct each via `chain::reconstruct`; for junctions not consumed by a run, reuse the existing `classify_junction`/`emit_blend`/`emit_move`. `fit_corners` and all step-4 functions **unchanged**.
- `rust/geometry/src/fitter/chain.rs` — **new**: run detection (`detect_runs`: same-turn-sign grouping + least-squares circle fit + residual gate), and `reconstruct` (center/R, `L_t` from the spiral-shift bound capped by budget, build the two transition `Clothoid`s + the `Arc`, oracle-verified seams, follower rescale; returns the reconstructed `Move` span or `None` to abandon). Local circle-fit + planar-basis helpers. Ends with `#[cfg(test)] mod tests;`.
- `rust/geometry/src/fitter/chain/tests.rs` — **new**: detection (co-circular accept / residual reject / turn-sign split / arc-break), reconstruction G2 + deviation ≤ δ + seam-oracle exactness, extrusion conservation across a run, the fall-through matrix rows, and the headline **gain** test (reconstructed run beats `fit_corners` via `plan_velocity`).
- `rust/geometry/src/fitter/biclothoid.rs` — **read-only**: `solve` reused verbatim as the per-corner fallback.
- `rust/geometry/src/path/{arc.rs,clothoid.rs,lowering.rs}` — **read-only**: `Arc::try_new`, `Clothoid::try_new`, `PositionProfile` (seam oracle).
- `rust/geometry/src/lib.rs` — re-export `fit_chain`, `ChainFitConfig` (additive; existing fitter re-exports unchanged).
- `_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md` — mark build step 8 implemented in the "Build sequence" and the chain-fit note in "Carried, not yet pressure-tested" (dumb-gcode chain-vs-corner reconstruction now pressure-tested for the co-circular case).

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/fitter/chain.rs` — `detect_runs` (same-turn grouping, least-squares circle fit, `cocircular_tol` residual gate, `min_run_junctions` floor) and `reconstruct` (center/R, `L_t = min(√(24Rδ), budget)` with shift ≤ δ guard, two transition `Clothoid`s + `Arc`, oracle-verified seams, follower-ratio rescale; `None` to abandon). Unit tests per `chain/tests.rs`.
- [x] `rust/geometry/src/fitter.rs` — add `ChainFitConfig`, `FitReport.chains`, and `fit_chain`: detect runs, emit reconstructions for accepted runs, delegate every other junction to the unchanged per-corner path; end-leg trim reservation identical to step 4; assemble the output `Move` chain.
- [x] `rust/geometry/src/lib.rs` — additive re-exports (`fit_chain`, `ChainFitConfig`).
- [x] `rust/geometry/src/fitter/chain/tests.rs` — every I/O-matrix row (incl. the `FitError` negative test), seam-oracle exactness, extrusion conservation, and the gain test vs `fit_corners`.
- [x] `_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md` — mark step 8 implemented.

**Acceptance Criteria:**
- Given ≥3 short co-circular same-turn `Line` facets with slack, when `fit_chain` runs, then the run becomes exactly one `[Clothoid up, Arc, Clothoid down]` span (`report.chains == 1`), κ is continuous across both transitions and the arc seams (`0→κ_arc→0`, equal at each seam to `1e-9`), every seam reproduces via `PositionProfile` to tol, and **no** internal junction appears in `report.unblended`.
- Given that same chain, when both `fit_chain` and `fit_corners` outputs are run through `plan_velocity`, then `traversal_time_s(fit_chain) < traversal_time_s(fit_corners)` and `fit_chain` shows `√(a·R)` cruise where `fit_corners` dips to `√(a/κ_peak)` at each facet apex — the upgrade is measurably faster.
- Given a run whose vertices are not co-circular (residual > `cocircular_tol`), or whose spiral shift would exceed δ, or that is shorter than `min_run_junctions`, when `fit_chain` runs, then it is abandoned and every junction takes the step-4 per-corner result (blend or sharp) — `report.chains` unincremented, output identical to `fit_corners` for that span.
- Given a run interrupted by an `Arc`/virtual move or a turn-sign reversal, when `fit_chain` runs, then the run splits at the interruption and each same-turn co-circular sub-run is evaluated independently.
- Given total follower ΔE over a faceted-arc run, when reconstructed, then ΔE equals the pre-fit sum within `1e-9` (rescaled ratios conserve extrusion exactly).
- Given a fit-valid run whose constructed `Arc`/`Clothoid` is rejected by `try_new`, when `fit_chain` runs, then `FitError::Internal` fires with the run's source line (fail-loud preserved); given an unreconstructable run, **no** `FitError`.
- (Additive) workspace builds; `cargo nextest run -p trajectory` unchanged; `fit_corners`, `biclothoid`, `velocity`, `path`, `frontend`, `segment`, `gcode` byte-for-byte untouched.

## Spec Change Log

- **Circle fit is the facet-line INCIRCLE (tangent to chords), not the circumcircle through vertices (geometry-forced).** The frozen "Always" says "fit center O, radius R to the vertices." A circumcircle through the vertices makes every boundary facet a *secant* (perp distance from O = the apothem `R·cos(α/2) < R`), and a clothoid easement from a straight line can only land *tangentially* on a circle whose radius = (perp distance from O to the line) − shift. So an easement from a chord lands on the **incircle** (radius ≈ apothem), never the circumcircle. Implementation therefore least-squares-fits the circle **tangent to the facet lines** (`n_i·(O−q_i)=ρ`, a 3×3 linear solve). This is exact and *closes by construction*: O is equidistant from every facet line, so the entry and exit easements are congruent and reference the same arc — the seam lands to machine precision for uniform faceting (the dominant #4228 case; Klipper's own arc interpolation emits uniform facets). Intent ("reconstruct the circle the run samples as a transition-spiral arc, G2, faster than per-corner") fully preserved; only the *which circle* realization changed. `arc.radius ≈ R·cos(α/2) ≈ R`, so the speed gain `√(a·R)` holds. [`chain.rs`]
- **The un-faceting tube is the faceting amplitude, not the per-corner δ (clarification).** Un-faceting a slicer arc *necessarily* deviates from the literal chords by ~the faceting sagitta (the slicer already accepted that error approximating its true arc). So `δ` bounds only the **transition-spiral shift** `p = L_t²/24R ≤ δ` (where the reconstruction leaves the sampled arc); the **`cocircular_tol`** residual gate (default 5e-3 mm) governs fidelity to the *sampled* geometry, and the closure seam check (`SEAM_TOL` 1e-6 mm) is the real accept/reject — non-co-circular runs miss closure and abandon to per-corner. [`chain.rs`, `fitter.rs`]
- **Boundary reservation moved from a fixed half-leg to an overlap guard (geometry-forced).** The frozen "end-leg trim budgets = half the outer legs' arclength" cannot hold: the incircle touches each chord at its *midpoint*, so the entry spiral inherently consumes the chord's second half plus the tangent offset — always `> ½` the facet. Implementation lets the run consume up to the whole boundary facet and, in `fit_chain`, **discards any run whose surviving per-corner boundary blend would overlap the consumed region** (`blend_trim > reserve`); discarded runs fall through to per-corner. Whole-chain runs (no boundary junction) reconstruct freely. Same non-overlap guarantee as step 4, enforced by check rather than by a fixed budget. [`fitter.rs`]
- **`ρ_arc` solved exactly from the easement quadratic; arc anchored G1-exact via the oracle.** `ρ_arc = ½(ρ + √(ρ²−L_t²/6))` (the consistent root of `p = L_t²/24ρ_arc`), and the arc centre is placed at `up.point_at(L_t) + ρ_arc·inward(up.heading_at(L_t))` so the clothoid→arc seam is tangent to floating-point exactness (`is_orthonormal` holds by construction), rather than reusing the incircle centre directly. Removes the small-angle drift the frozen sketch's `Arc::try_new(o,…)` would carry. [`chain.rs`]
- **`FitError::Internal` on a post-gate `try_new` rejection is defensive/unreachable under the finiteness gates (KEEP — fail-loud preserved).** Every constructed `Clothoid`/`Arc` is gated (`ρ_arc>0`, `L_t>0`, `Δ>ANGLE_EPS`, orthonormal-by-construction, finite ρ/δ) before `try_new`, so a rejection cannot occur for gated input — non-finite/degenerate fits return `Ok(None)` (abandon) first. The `internal()` mapping is wired exactly as step 4's and would fire on a genuine invariant break; the reachable half of the AC ("unreconstructable run ⇒ no `FitError`") is tested via `non_cocircular_run_falls_through_to_per_corner`. Mirrors step-4's accepted defensive-branch dispositions. [`chain.rs`]
- **`FitReport.chains` field forces a one-line ripple in `velocity/tests.rs` (test helper only).** Adding `pub chains: u32` to `FitReport` requires the velocity test helper that builds a `FitReport` by explicit fields to add `chains: 0`. `velocity.rs` source is byte-for-byte untouched; only the test constructor changed. [`velocity/tests.rs`]

### Review Findings (3-layer adversarial pass, 2026-06-18)

Blind hunter (diff only) + edge-case hunter (diff + project) + acceptance auditor (diff + spec + context). Auditor verdict **ACCEPT-WITH-PATCHES**; no `intent_gap`, no loopback (the four geometry-forced Change-Log deviations were confirmed intent-preserving).

- [x] [Review][Patch] **Seam closure failed for coarse faceting — the `ρ_arc` quadratic was only the small-angle easement solution (edge-case hunter, HIGH; surfaced as `seam_head/seam_tail=false`).** The frozen `p = L_t²/24R` shift is a small-angle approximation, so the spiral's *exact* (Fresnel) perpendicular offset `c_in` ≠ the incircle radius `ρ`, leaving the spiral start `s0` off `lines[0]` by an error that scales with facet-angle² — a real position gap that `seam_ok` (1e-6) correctly rejected, so anything coarser than ~7°/facet silently fell back to per-corner (a throughput loss). **Fix:** replace the quadratic with `solve_rho_arc` — a deterministic 60-step bisection that finds `ρ_arc` so the *exact* `c_in(ρ_arc) = ρ`, closing both seams to machine precision at any faceting angle. New test `coarse_facets_still_reconstruct_landing_on_the_last_chord` (5 facets / ~12°). [`chain.rs`]
- [x] [Review][Patch] **N=3 minimum run made the co-circularity residual gate vacuous (edge-case hunter, P1/P4).** At the minimum run length the incircle solve is exactly determined (3 lines ⇒ 3 constraints ⇒ residual ≡ 0), so three chords of three *different* circles would reconstruct as the triangle incircle and gross-cut the corner. **Fix:** added `vertices_within_tube` — every interior vertex's radial deviation from the reconstructed arc must be within `2·(local chord sagitta) + cocircular_tol`, the real un-faceting fidelity bound that holds at any N. New test `non_cocircular_triple_is_rejected_by_the_vertex_tube`. [`chain.rs`]
- [x] [Review][Patch] **Latent `sqrt(negative)` at the radius clamp (blind hunter MED-11 / edge-case P2).** Obsoleted by the bisection rewrite (the quadratic is gone); the bisection evaluates only `cos`/Fresnel, no radicand. [`chain.rs`]
- [x] [Review][Patch] **AC-2 mechanism / shift-≤-δ under-tested (acceptance auditor gap-1/2).** Added `transition_shift_stays_within_delta` (asserts `L_t²/24R ≤ δ`) and the coarse-facet heading test (confirms `down` exits along the *last chord* — refutes the blind hunter's HIGH-4 claim that the tail-heading gate is unsatisfiable; the net turn equals the chord-to-chord total by construction). [`chain/tests.rs`]
- [Review][Reject] **Blind hunter HIGH-4 (tail-heading gate unsatisfiable for real facets).** Refuted: `delta_sweep = theta_run − l_t/ρ_arc` ties the reconstruction's net turn to the *chord* total `theta_run`, so `down` exits exactly along `t_m`; the `chains==1` assertions (incl. coarse n=5) prove reconstruction fires. **Reject** MED-10 (negative `delta_sweep` should fail-loud) — the frozen "Never: no fail-loud on a run that won't reconstruct" mandates graceful abandon, not error. **Reject** the extrusion instantaneous-rate concern — total-ΔE conservation is the codebase's established convention (step-4 `trim/L`); `run_followers` matches it exactly. **Reject** the `normalize` zero-guard — inputs are unit perpendiculars by construction (`turn_normal`/cross), and any residual non-finite is caught by the `rho.is_finite()` gate or `try_new`.
- [Review][Defer] Greedy `detect_runs` end-non-backtracking; chain-abandon observability; `solve3` conditioning-vs-singularity. All appended to `deferred-work.md` (backstopped, not incorrect output).

## Design Notes

**Why arc-with-transition-spirals is the right V1 chain primitive.** A slicer's faceted arc is N chords inscribed in one circle of radius R. The step-4 per-corner fitter cannot see the circle: it blends each chord-pair locally, returns κ to 0 between them, and — starved of budget on short legs — picks a *tight* `κ_peak ≫ 1/R`, so the limit-rider throttles to `√(a/κ_peak)` N times. Reconstructing the circle and entering/leaving it through two clothoid transition spirals gives the textbook time-optimal path for a circular arc: κ holds at the gentle `1/R`, cruise at `√(a·R)`, G2 at every seam. It needs no new segment type — `Clothoid·Arc·Clothoid` is already the alphabet, and the arc body's deviation from the original vertices is ~0 (they lie on the fitted circle), so reconstruction *reduces* path error versus the chords while raising speed.

**The spiral-shift deviation bound.** A transition spiral of length `L_t` ramping κ `0→1/R` shifts the arc inward from the tangent by `p ≈ L_t²/(24R)` (standard road/rail surveying). Choosing `L_t = √(24Rδ)` makes `p = δ` exactly — the gentlest transition the deviation tube allows; capping `L_t` at the end-leg budget shortens it (less shift, still ≤ δ) when the outer legs are short. If even the budget-capped spiral cannot fit inside δ, the run is abandoned to per-corner.

**Co-circularity is the detection gate.** Group consecutive same-turn `Line`-facet junctions, least-squares-fit a circle to their vertices, accept the run iff the max radial residual ≤ `cocircular_tol`. This is the "dumb-gcode chain reconstruction" the architecture carried as not-yet-pressure-tested — V1 pressure-tests the co-circular case; varying-κ splines are deferred (Ask First).

Sketch (one accepted run):
```
let (o, r) = fit_circle(&vertices)?;          // least-squares; residual ≤ tol
let kappa_arc = 1.0 / r;
let l_t = (24.0 * r * delta).sqrt().min(end_leg_budget);
if l_t * l_t / (24.0 * r) > delta + tol { return None; }   // shift exceeds δ ⇒ abandon
let up   = Clothoid::try_new(entry_pt, t_in, n, 0.0, kappa_arc / l_t, l_t)?;
let arc  = Arc::try_new(o, u, v, r, start_angle, residual_sweep)?;
let down = Clothoid::try_new(arc.point_at(arc.s_len()), .., kappa_arc, -kappa_arc / l_t, l_t)?;
```

## Verification

**Commands:**
- `cd rust && cargo nextest run -p geometry` — expected: new `fitter::chain::tests` pass; existing `fitter::tests`/`velocity` unchanged.
- `cd rust && cargo nextest run -p trajectory` — expected: unchanged (proves the change stays inside `geometry::fitter`).
- `cd rust && cargo clippy -p geometry --all-targets -- -D warnings` && `cargo fmt --check` — expected: clean.
- `git diff --stat` on `rust/geometry/src/fitter/biclothoid.rs`, `rust/geometry/src/velocity*`, `rust/geometry/src/path`, `rust/geometry/src/frontend.rs`, `rust/gcode` — expected: empty.

## Suggested Review Order

**The chain-fit entry + run assembly (start here)**

- Entry point — detect runs, drop overlap-conflicting runs, splice reconstructions with the unchanged per-corner path.
  [`fitter.rs:164`](../../rust/geometry/src/fitter.rs#L164)
- Boundary-overlap guard — a run is discarded if a surviving per-corner boundary blend would collide (replaces the geometry-impossible fixed half-leg reserve).
  [`fitter.rs:177`](../../rust/geometry/src/fitter.rs#L177)
- Per-move role + reconstruction emit (clothoid·arc·clothoid carry the conserved follower ratios).
  [`fitter.rs:251`](../../rust/geometry/src/fitter.rs#L251)

**Run detection (faceted-chain reconstruction — the geometric heart)**

- `detect_runs` — greedy same-turn co-circular grouping; reconstruct-or-advance.
  [`chain.rs:32`](../../rust/geometry/src/fitter/chain.rs#L32)
- `grow_run` — maximal span under same-plane / same-turn-sign / non-reversal; collinear junctions continue the run.
  [`chain.rs:61`](../../rust/geometry/src/fitter/chain.rs#L61)
- `reconstruct` — incircle fit → `l_t`/`ρ_arc` → transition spirals + arc (oracle-anchored G1) → fail-loud abandon gates.
  [`chain.rs:110`](../../rust/geometry/src/fitter/chain.rs#L110)

**The exact-closure math (highest-leverage; review-driven)**

- `solve_rho_arc` — deterministic bisection so the spiral's exact Fresnel offset equals the incircle radius; closes the seam at any faceting angle (the small-angle quadratic did not).
  [`chain.rs:284`](../../rust/geometry/src/fitter/chain.rs#L284)
- `vertices_within_tube` — the real co-circularity guard at the minimum run length, where the incircle residual is vacuous.
  [`chain.rs:386`](../../rust/geometry/src/fitter/chain.rs#L386)
- `incircle` — 3×3 linear least-squares circle tangent to the facet lines (closes by the equidistant property).
  [`chain.rs:240`](../../rust/geometry/src/fitter/chain.rs#L240)

**Tests (peripherals)**

- Headline gain: reconstructed run beats the per-corner sawtooth via `plan_velocity`.
  [`tests.rs:174`](../../rust/geometry/src/fitter/chain/tests.rs#L174)
- G2 + seam-oracle exactness; coarse-facet closure; non-co-circular-triple rejection.
  [`tests.rs:129`](../../rust/geometry/src/fitter/chain/tests.rs#L129)
