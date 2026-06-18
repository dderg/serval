---
title: 'Build step 5 — velocity-planning skeleton: node-based forward-backward sweep + closed-form caps (curvature + corner-stop), constant-speed-per-feature, no jerk'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: '99951f99b842e4c9e30fa3dad685bed32dc1734e'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md'
  - '{project-root}/_bmad-output/implementation-artifacts/deferred-work.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The step-4 fitter emits a uniform `Vec<Move>` of `{Line|Arc|Clothoid}` with curvature read off `CurvatureProfile`, but nothing assigns it speed yet: every move would run at an undefined/feedrate-only velocity, tight arcs and blended corners would over-accelerate (`a_c = v²κ` unbounded), and sharp unblended junctions have no stop. Build-sequence step 5.

**Approach:** Add a **stateless** `geometry::velocity` pass, `plan_velocity(&FitOutcome, …) -> Result<VelocityProfile>`, that plans a full move chain offline (batch, off the live path, like steps 1–4). It is the **mainline-parity skeleton**: one constant speed *ceiling* per move from closed-form caps — `min(feedrate, max_velocity, √(a/κ_peak))` — then the classic two-pass **forward-backward sweep** (Dong-Stori globally optimal for accel+curvature) over a node set (one node per move boundary) producing per-move `(start_v, cruise_v, end_v)` trapezoids. Unblended sharp junctions (read from `FitReport`) pin `v=0`. The sweep reads **κ-space only** (`CurvatureProfile`); `σ` is validated-and-finite but **uncalled** in the speed law (jerk is step 6); position/Fresnel is barred. No SOCP, no S-curve, no limit-riding within a segment, no volumetric-flow cap — those are steps 6–8 / 5+.

## Boundaries & Constraints

**Always:**
- New module `geometry::velocity`, stateless free functions; strictly additive (new file + tests only). Output is a fresh `VelocityProfile`; the input `FitOutcome` move chain is unchanged.
- Caps composed as `min`: per-move ceiling `= min(feedrate_mm_s, max_velocity_mm_s, curvature_cap)` where `curvature_cap = √(accel / |κ_peak|)` (κ_peak from `CurvatureProfile::kappa_peak().1`; `+∞` when `κ_peak == 0`, i.e. lines). This single law caps arcs, clothoids, and blended corners uniformly.
- Forward-backward feasibility: between adjacent nodes, `|v_out² − v_in²| ≤ 2·a·L` (accel never exceeded); node speed ≤ each adjacent move's ceiling; chain starts and ends at rest (`v=0`).
- Sharp junction = full stop: a junction the `FitReport` lists as unblended with any reason **except `Collinear`** pins `v=0` at that node (safe floor — no tangent-break at speed). `Collinear` is κ-continuous → no extra cap.
- Read-contract discipline: the `velocity` module calls only `CurvatureProfile` methods + reads `VelocityLimits`/`FitReport`; it must **not** import or call `PositionProfile`/`point_at`/`heading_at` (Fresnel barred from the planner).
- Fail loud (return `Err`) on degenerate IR reaching the planner: clothoid L-consistency `σ ≈ (κ(L)−κ(0))/L` violated; a `kappa_peak` location not at a segment endpoint (mid-segment κ extremum ⇒ non-alphabet segment slipped in); non-finite `κ_peak`/`σ`; non-positive segment length.
- Deterministic: identical input ⇒ identical `VelocityProfile`.

**Ask First:**
- Moving the planner out of `geometry` (e.g. a new crate / `temporal`) or any change that would let it touch `PositionProfile`.
- Introducing per-segment speed *variation* (limit-riding), an S-curve/jerk term, or a non-zero junction-deviation corner speed — all are later steps; adding them here breaks the skeleton's scope.

**Never:**
- No edits to any step-1/2/3/4 stored type or trait (`PathSegment`, `Segment`, `Line/Arc/Clothoid`, `CurvatureProfile`, `PositionProfile`, lowering, `Move`, `VelocityLimits`, `FitOutcome`). Pure consumer.
- No SOCP/Clarabel, no `temporal`/`trajectory` crate changes, no live-path/klippy/bridge wiring, no execution lowering to time.
- No volumetric-flow / extruder-velocity cap, no PA-augmented transient, no follower-ratio bound (deferred to step 5+, see `deferred-work.md`); followers are passed through untouched.
- No planning grid / arc-length sampling table; the sweep is node-based (junctions only).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Straight line | `Line`, high feedrate | ceiling = `min(feedrate, max_velocity)`; trapezoid accel→cruise→decel | N/A |
| Arc | `Arc` radius r, accel a | cruise capped at `√(a·r)` within tol | N/A |
| Blended corner | `Clothoid` half, κ_peak | cruise capped at `√(a/κ_peak)` (≈ SCV); κ-continuous to neighbors, no stop | N/A |
| Sharp corner | junction in `report.unblended`, reason `NearReversal`/`ZeroDeviation`/`NoBudget`/`ArcIncident`/`NonSpatial` | node speed pinned `v=0`; neighbors ramp to/from 0 | N/A |
| Short move | L too small to reach ceiling | cruise = trapezoid apex `√((v_in²+v_out²)/2 + a·L)` < ceiling | N/A |
| 0 or 1 move | empty / single move | empty profile / single move bracketed by rest | N/A |
| Degenerate clothoid | `σ ≠ (κ(L)−κ(0))/L` | — | `Err(VelocityError::Inconsistent)` |
| Mid-segment κ peak | `kappa_peak().0 ∉ {0, L}` | — | `Err(VelocityError::NonAlphabet)` |
| Non-finite cap | `κ_peak`/`σ` NaN/∞ | — | `Err(VelocityError::NonFinite)` |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/velocity.rs` — **new**: `VelocityProfile`, `MoveVelocity`, `VelocityReport`, `VelocityError`, `plan_velocity` (caps + forward-backward sweep + trapezoid apex + junction-stop derivation). Ends with `#[cfg(test)] mod tests;`.
- `rust/geometry/src/velocity/tests.rs` — **new**: unit tests (`use super::*;`), one per I/O-matrix row + the ACs below.
- `rust/geometry/src/lib.rs` — add `pub mod velocity;` (alongside `fitter`).
- `rust/geometry/src/path/profile.rs` — `CurvatureProfile` (`s_len`, `kappa_peak`→`(s,|κ|)`, `kappa_endpoints`→signed, `dkappa_ds`): the **only** read contract the sweep consumes.
- `rust/geometry/src/fitter.rs` — `FitOutcome{moves,report}`, `FitReport{unblended:Vec<UnblendedJunction{line_no,reason}>}`, `UnblendReason`: the input + junction-stop source.
- `rust/geometry/src/frontend.rs` — `Move{segment,feedrate_mm_s,limits,source}`, `VelocityLimits{max_velocity_mm_s,accel_mm_s2,square_corner_velocity_mm_s}`: per-move caps; `source.start_line` keys junction-stop correlation.
- `_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md` — "Velocity planning" §: node-based sweep, Dong-Stori optimality, node-coverage + L-consistency invariants, σ-carried-uncalled.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/velocity.rs` — define `VelocityProfile { moves: Vec<MoveVelocity> }`, `MoveVelocity { start_v, cruise_v, end_v, ceiling, accel, length, source: SourceRange }`, `VelocityReport { stops, curvature_bound, feedrate_bound }`, `VelocityError` (`Inconsistent`/`NonAlphabet`/`NonFinite`, each `{ line_no }`). Implement `plan_velocity(outcome: &FitOutcome, config: VelocityConfig) -> Result<VelocityProfile, VelocityError>` with `VelocityConfig { consistency_tol }` (dimensionless, default `1e-6`; scaled by length for the mm endpoint check).
- [x] `rust/geometry/src/velocity.rs` — ceiling pass: per move validate κ/σ invariants (fail loud), compute `ceiling = min(feedrate, max_velocity, curvature_cap)`; record which cap binds in `VelocityReport`.
- [x] `rust/geometry/src/velocity.rs` — junction-stop derivation: build the stop-line `HashSet` from `report.unblended` (reason ≠ `Collinear`); a move-boundary node is `v=0` iff the downstream move's `source.start_line` is in that set **and** the downstream move is not a blend-half clothoid (clothoids appear only as fitter blend-halves, which share a leg's source line — see Spec Change Log); non-spatial moves bracket as stops.
- [x] `rust/geometry/src/velocity.rs` — sweep: init node speeds to adjacent ceilings (0 at stops/ends); forward pass `v[k]=min(v[k], √(v[k-1]²+2a·L))`; backward pass `v[k]=min(v[k], √(v[k+1]²+2a·L))`; per move set `start_v/end_v` from nodes and `cruise_v = min(ceiling, √((start_v²+end_v²)/2 + a·L))`.
- [x] `rust/geometry/src/lib.rs` — `pub mod velocity;`.
- [x] `rust/geometry/src/velocity/tests.rs` — paired tests for every I/O-matrix row (incl. all three fail-loud `Err` paths) and the ACs.

**Acceptance Criteria:**
- Given an arc of radius r with accel a and a feedrate above `√(a·r)`, when planned, then its `cruise_v == √(a·r)` within tol and `ceiling` reflects the curvature cap.
- Given any planned move, when checked, then `cruise_v ≤ ceiling`, `start_v/end_v ≤` each adjacent ceiling, and `|end_v² − start_v²| ≤ 2·accel·length` (+tol) — forward-backward feasibility holds chain-wide; the chain starts and ends at `v=0`.
- Given a junction reported `NearReversal` (or `ZeroDeviation`/`NoBudget`), when planned, then the shared node speed is `0` and both neighbors ramp to/from `0`.
- Given a clothoid with `σ` falsified to break `σ=(κ(L)−κ(0))/L`, when planned, then `Err(VelocityError::Inconsistent{line_no})`; given a segment whose `kappa_peak` location is interior, then `Err(NonAlphabet)`.
- Given the same `FitOutcome` twice, when planned, then byte-identical `VelocityProfile` (determinism).
- (Additive) workspace builds; `cargo nextest run -p trajectory` unchanged; step-1/2/3/4 `path` types, `frontend`, `fitter`, `classify.rs`, and the `gcode` crate byte-for-byte untouched; the `velocity` module source contains no `point_at`/`heading_at`/`PositionProfile`.

## Spec Change Log

- [Review][Patch] **Stop leaked into an adjacent blend entry (Blind Hunter).** The junction-stop derivation keyed purely on `downstream.source.start_line ∈ stop_lines`. But when a stopped move is also the *upstream* leg of a blend, the blend's first clothoid half inherits that move's source line (`emit_blend` sets `half1.source = m_in.source`), so the seam between the stopped move and `half1` was spuriously pinned `v=0` — a stop *inside* a legitimate blend (throughput loss). Fix: a real stop's downstream is always a `Line`/`Arc`/virtual (inputs carry no clothoids; clothoids appear only as fitter blend-halves), so stop-detection now additionally requires `!matches!(downstream.segment.spatial, Some(Segment::Clothoid(_)))`. Known-bad avoided: a sharp corner immediately followed by a blendable corner no longer dwells at the blend entry. Paired test `stop_does_not_leak_into_adjacent_blend_entry`. KEEP: junction classification stays κ-space + `FitReport`-derived (no `PositionProfile`). [`velocity.rs`]
- [Review][Patch] **Units conflation in the endpoint tolerance (Blind Hunter).** The node-coverage check reused the dimensionless `consistency_tol` as a millimetre tolerance on `s_peak`. Now scaled by length (`endpoint_tol = tol * length`); behaviour unchanged for the exact alphabet (`s_peak` is literal `0.0`/`length`), conflation removed. [`velocity.rs`]
- [Review][Patch] **O(n·m) `stop_lines` membership (Blind Hunter).** `Vec::contains` in the per-move loop → `HashSet` (O(1); membership-only, determinism preserved). [`velocity.rs`]
- [Review][Reject] Negative `kappa_peak` disabling the cap, infinite ceiling, NaN/negative feedrate — all impossible at the planner boundary: `kappa_peak` returns magnitude (all three impls), `max_velocity` and `feedrate` are validated finite-positive upstream (`frontend.rs`). Chain-ends-at-rest is intended batch scope (streaming continuity is a later step).

## Design Notes

**Why `geometry::velocity`, not `temporal`.** Steps 1–4 built the whole new pipeline additively in `geometry`, off the live path; step 5 continues that — the sweep consumes `geometry::Move`/`CurvatureProfile` and needs no `clarabel`. The architecture's "replaces the SOCP" is about the *eventual* live path, not where the skeleton lands. True `PositionProfile`-barred-by-privacy enforcement needs a step-1/2 public-API change (out of scope), so here it is a convention + an AC/grep.

**The three caps are one law.** `a_c = v²κ ≤ a` ⇒ `v ≤ √(a/κ)`. Lines (κ=0) → `+∞` → feedrate/max_velocity bind. Arcs → `√(a·r)`. Blended corners are just clothoid segments whose `κ_peak` *is* the corner cap (≈ SCV by the step-4 δ construction) — no separate corner-speed formula. A tangent-continuous κ-step (line↔arc seam) needs no special junction cap: the tighter side's per-move ceiling + the sweep already bind it. Only a *tangent discontinuity* (sharp unblended corner) needs the `v=0` pin, and those are exactly the non-`Collinear` `FitReport` entries.

**Constant-speed-per-feature = one ceiling per move.** Each curved move uses its single `κ_peak` ceiling for its whole length (conservative). Step 7 (limit-riding) later lets speed ride `√(a/κ(s))` *within* a clothoid; swapping that rule in must not touch this sweep's structure — hence `σ` rides the contract now (validated, finite) but is uncalled by the v1 speed law.

**Node-coverage canary.** For the `{Line|Arc|Clothoid}` alphabet κ is monotonic, so `kappa_peak().0 ∈ {0, L}` always; asserting it is the cheap guard that a non-linear-κ segment (which would let `√(a/κ)` collapse mid-segment, a Pham discontinuity switching point) never reaches the sweep.

Sketch (one move):
```
let (s_peak, k_peak) = seg.kappa_peak();          // |κ|, at an endpoint
if !(s_peak == 0.0 || (s_peak - L).abs() <= tol) { return Err(NonAlphabet); }
let cap = if k_peak == 0.0 { f64::INFINITY } else { (accel / k_peak).sqrt() };
let ceiling = feedrate.min(max_velocity).min(cap);
```

## Verification

**Commands:**
- `cargo nextest run -p geometry` — expected: new `velocity` tests pass, existing geometry tests unchanged.
- `cargo nextest run -p trajectory` — expected: unchanged (proves additivity).
- `./scripts/ci.sh rust-clippy` && `./scripts/ci.sh rust-fmt` — expected: green (`-D warnings`).
- `! grep -rE 'point_at|heading_at|PositionProfile' rust/geometry/src/velocity.rs rust/geometry/src/velocity/` — expected: no matches (read-contract discipline).
- `git diff --stat` on `rust/geometry/src/{path,frontend.rs,fitter.rs,classify.rs}` and `rust/gcode` — expected: empty.

## Suggested Review Order

**Design intent (start here)**

- Entry point: the whole pass — ceiling caps, junction stops, then the sweep, all κ-space only.
  [`velocity.rs:59`](../../rust/geometry/src/velocity.rs#L59)

**Closed-form caps (the speed law)**

- The one law `v ≤ √(a/κ_peak)`; `+∞` for lines so feedrate/max_velocity bind via the outer `min`.
  [`velocity.rs:90`](../../rust/geometry/src/velocity.rs#L90)

**Junction stops (highest-risk: the reviewed bug lives here)**

- Stop iff downstream is a non-`Collinear` unblended junction **and not a blend-half clothoid** (the leaked-stop fix).
  [`velocity.rs:126`](../../rust/geometry/src/velocity.rs#L126)

**Forward-backward sweep + trapezoid**

- Two monotone `min` passes (Dong-Stori optimal); accel-feasible by construction.
  [`velocity.rs:134`](../../rust/geometry/src/velocity.rs#L134)
- Trapezoid apex `√((v_in²+v_out²)/2 + a·L)`, capped at the ceiling.
  [`velocity.rs:147`](../../rust/geometry/src/velocity.rs#L147)

**Fail-loud seam guards**

- Node-coverage (length-scaled endpoint tol) + L-consistency `σ=(κ_L−κ_0)/L`; generic over `CurvatureProfile` so it is mock-testable.
  [`velocity.rs:163`](../../rust/geometry/src/velocity.rs#L163)

**Wiring & tests (peripherals)**

- Additive registration — one `pub mod` + re-export.
  [`lib.rs:14`](../../rust/geometry/src/lib.rs#L14)
- Regression test for the leaked-stop fix.
  [`tests.rs:162`](../../rust/geometry/src/velocity/tests.rs#L162)
- Chain-wide feasibility AC (accel budget, ceilings, rest-at-ends).
  [`tests.rs:255`](../../rust/geometry/src/velocity/tests.rs#L255)
