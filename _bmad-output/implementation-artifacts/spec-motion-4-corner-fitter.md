---
title: 'Build step 4 — fitter v1: per-corner symmetric biclothoid blends (line→line), δ from SCV, graceful fall-through'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: '7d31474a3c34715c1fcc39be65c95dc668973574'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/SPEC.md'
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-3-frontend-gcode.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Step 3 turns parsed G-code into a chain of typed `Move`s (`Line`/`Arc`), but a sharp corner between two collinear-broken `Line`s is a tangent discontinuity — infinite κ — so the step-5 velocity law `v=√(a/κ)` would stop the toolhead dead at every vertex. Mainline's junction-deviation is a virtual-circle *speed* heuristic that inserts no actual blend and provably throttles faceted arcs (#4228). Nothing in the typed-IR path refits corners into continuous motion yet (build-sequence step 4).

**Approach:** Add a **stateless** `geometry::fitter` middleware pass (IR→IR): `fit_corners(&[Move]) -> FitOutcome`. At each junction between two spatial `Line` moves it inserts a **symmetric biclothoid** — two `Clothoid` halves ramping κ `0→κ_peak→0` — that is G1-collinear with the trimmed legs and G2 (κ-continuous) through the blend, deviating from the vertex by ≤ δ. δ is **derived per junction** from the existing `VelocityLimits` via Klipper's `δ = SCV²·(√2−1)/a` (no new user knob; SCV reused). The fitter is a **cost-reducing optimizer, not a gate**: any corner it cannot blend (too short, too sharp, collinear, arc-involved) is left as a sharp junction for the step-5 speed cap — **never a fail-loud**. Strictly additive: a new module + tests, no edits to step-1/2/3 types or the live path.

## Boundaries & Constraints

**Always:**
- New module `geometry::fitter`, **stateless** free functions. Input `&[Move]` (step-3 output); output a `FitOutcome { moves: Vec<Move>, report: FitReport }` where blended corners are expanded to `[trimmed Line, Clothoid up, Clothoid down, trimmed Line]` and unblended ones pass through unchanged. The output is again a `Move` chain (uniform pipeline; step 5 reads κ from it).
- A **corner** is a junction between two consecutive moves whose `spatial` are both `Segment::Line`, with deflection `θ = angle(t_in, t_out) ∈ (θ_min, θ_max)` (`t_in` = incoming end-heading, `t_out` = outgoing start-heading via `PositionProfile::heading_at`). Collinear (`θ ≤ θ_min`, default 1e-3 rad) ⇒ pass through (already G1). Near-reversal (`θ ≥ θ_max`, default `π − 1e-3`) ⇒ leave sharp.
- **δ per junction** `= min(δ(in), δ(out))`, `δ(m) = scv²·(√2−1)/a` from that move's `VelocityLimits`. `δ = 0` (SCV 0 ⇒ stop-at-corner) ⇒ leave sharp. The fitter adds no user config beyond `CornerFitConfig { theta_min_rad, theta_max_rad }` (Default-constructible; constants for tests).
- **Biclothoid geometry** (planar, in the plane the two tangents span): `u = t_in`, `v = normalize(t_out − (t_out·t_in)·t_in)` (the turn direction). Each half turns `θ/2`: `κ_peak·L = θ`, `σ = κ_peak/L`, so `L = θ/κ_peak`, `σ = κ_peak²/θ`. Half 1: `Clothoid{ start_pose=A′, u, v, kappa_0=0, sigma=+κ_peak/L, length=L }`; half 2 starts at the apex `M`, `kappa_0=κ_peak`, `sigma=−κ_peak/L`, ending at `B′` with heading `t_out`.
- **κ_peak selection** = the gentlest blend that both fits the budget and stays within δ: seed `κ_δ = θ²/(24δ)` (road-design shift form), Newton-refine so actual apex deviation `|P−M| = δ`; if the resulting per-leg tangent trim `T` exceeds the budget, raise κ_peak to the tightest that fits (smaller deviation, still ≤ δ, slower corner). **Trim budget = half** the arclength of each incident segment (reserves the other half for that segment's far end — guarantees independent per-corner blends never overlap). No fit within budget ⇒ leave sharp.
- **Seam exactness:** placement reuses the step-2 `PositionProfile` evaluator as the oracle — `half1.point_at(0)=A′` heading `t_in`, `half2.point_at(L)=B′` heading `t_out`, apex tangent continuous — so fitter and execution lowering never disagree (the step-3 AC-FE-1 discipline).
- **Follower / metadata threading:** trimming a `Line` preserves its follower `ratio` (ratio is per-unit-length). Each clothoid half inherits its source-side leg's `followers`, `feedrate_mm_s`, `limits`, `source` (up←in, down←out). This conserves total ΔE across the corner to within the δ-tube.
- Fail loud only on a genuine internal invariant break (a constructed `Clothoid`/`Line` that fails `try_new`, a non-finite computed quantity) via a typed `FitError` carrying the junction's source line — **not** on an unblendable corner (that is a normal pass-through, recorded in `FitReport`). Unit tests in a separate file.

**Ask First:**
- **Arc seams** (line↔arc transition spirals, arc↔arc) — different geometry (a single clothoid half ramping `0→κ_arc`, arc trimming by angle). V1 passes `Arc` moves through untouched and leaves any arc-incident junction sharp. Clean later add.
- 3D corners beyond the tangent-spanned plane, chain/continuous-κ merging across runs of corners (build-sequence step 8), or any change to a step-1/2/3 stored type (`PathSegment`, `Segment`, `Clothoid`, `VelocityLimits`, lowering).
- Emitting explicit per-junction records (κ⁻,κ⁺,σ⁻,σ⁺,G1-flag) for the step-5 read-contract — V1 leaves continuity derivable from the geometry + `FitReport`.

**Never:**
- A velocity solver, S-curve, speed cap, or any timing — step 5+. The fitter touches geometry only.
- Capping acceleration for ringing / consulting any shaper model (shaper-agnostic by authority).
- Faceting, NURBS/`CubicSegment`/`CornerBlendSlot`/`BlendFamily`/legacy `FitterParams` (the old path being replaced) — build native `path::{Line,Clothoid}` only.
- Fail-loud on an unblendable corner, or mutating the live `submit_move`/`classify` path.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 90° line→line corner, δ>0 | two `Line`s, θ=π/2, legs long | biclothoid inserted; G1+G2; apex deviation ≤ δ; κ_peak≈θ²/(24δ) | N/A |
| Collinear junction | θ ≤ θ_min | pass through, no blend | N/A |
| Near-reversal | θ ≥ θ_max | leave sharp, recorded in report | N/A |
| SCV = 0 | δ=0 | leave sharp (stop-at-corner) | N/A |
| Short leg | trim budget < tightest fitting blend | leave sharp | N/A |
| Tight-but-fits | δ-optimal trim > budget, tighter blend fits | blend with raised κ_peak; deviation < δ | N/A |
| Arc-incident junction | one side `Arc` | both moves pass through; junction left sharp | N/A |
| Virtual move (retraction) | `spatial=None` between two lines | passes through; breaks the spatial chain (no blend across it) | N/A |
| Single move / empty | `len ≤ 1` | returned unchanged | N/A |
| Internal break | constructed `Clothoid` fails `try_new` | typed `FitError` with source line | `FitError` |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/path/clothoid.rs` -- `Clothoid::try_new(start_pose,u,v,kappa_0,sigma,length)` — the blend constructor (read-only).
- `rust/geometry/src/path/lowering.rs:7` -- `PositionProfile { point_at, heading_at }`; impls for `Line`/`Arc`/`Clothoid`/`Segment` — the tangent + placement oracle (read-only).
- `rust/geometry/src/path/profile.rs` -- `CurvatureProfile` (`s_len`, `kappa_endpoints`) — used to read leg lengths and verify κ continuity.
- `rust/geometry/src/path/{mod.rs,line.rs}` -- `Segment::Line`, `PathSegment::try_new`, `Line::try_new`/`length()` — match on `spatial`, build trimmed legs.
- `rust/geometry/src/frontend.rs:55` -- `Move { segment, feedrate_mm_s, limits, source }`, `VelocityLimits { …, accel_mm_s2, square_corner_velocity_mm_s }` — input/output element + δ source.
- `rust/geometry/src/segment.rs:32` -- `FollowerDemand`, `SourceRange` — reuse verbatim. (`CornerBlendSlot`/`CubicSegment` here are the legacy path — do not touch.)
- `rust/geometry/src/lib.rs:5` -- `pub mod fitter;` + re-exports — the only tracked-file edit outside the new module.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/fitter.rs` -- `CornerFitConfig`, `FitOutcome`, `FitReport`, `FitError`, `fit_corners` (two-pass chain walk, junction classification, δ-from-SCV, follower/metadata threading, output assembly), and local vector helpers.
- [x] `rust/geometry/src/fitter/biclothoid.rs` -- the symmetric-biclothoid solver: θ + turn-plane, exact self-similar κ_peak (deviation = δ), trim-budget fit-or-tighten, placement of the two `Clothoid` halves, oracle-verified endpoints. Returns the two halves + trim, or `None` (no fit).
- [x] `rust/geometry/src/fitter/tests.rs` -- the I/O matrix (every row) + the ACs below.
- [x] `rust/geometry/src/lib.rs` -- register `fitter` + re-export `fit_corners`, `FitOutcome`, `FitReport`, `CornerFitConfig`, `FitError`, `UnblendReason`, `UnblendedJunction`.

**Acceptance Criteria:**
- (AC-1, seam) For a blended corner, `half1.point_at(0)` = trimmed-line-A end and `half2.point_at(s_len)` = trimmed-line-B start within tol, and headings match `t_in`/`t_out` — the blend reproduces the leg seams via the step-2 evaluator.
- (AC-2, continuity) κ is continuous through the blend: `line.kappa_end=0 = half1.kappa(0)`, `half1.kappa(L)=half2.kappa(0)=κ_peak`, `half2.kappa(L)=0=line.kappa_start`; and the legs/clothoids are G1 (consecutive headings collinear within tol).
- (AC-3, deviation) apex deviation from the original vertex ≤ δ for every blended corner, and `≈ δ` when the budget is not binding; `δ = SCV²(√2−1)/a` matches Klipper for the move's limits.
- (AC-4, fall-through) collinear, near-reversal, SCV=0, short-leg, and arc-incident junctions all pass through with the corner left sharp and a `FitReport` entry naming the reason — **no `FitError`**.
- (AC-5, extrusion) total follower ΔE across a blended corner equals the pre-fit ΔE within the δ-tube tolerance; a travel→travel corner blends with zero followers.
- (AC-6, additive) workspace builds; `cargo nextest run -p trajectory` unchanged; step-1/2/3 `path` types, `frontend`, `classify.rs`, and the `gcode` crate are byte-for-byte untouched.
- Each fail-loud row returns its exact typed `FitError` and emits no blend (negative-test obligation).

## Spec Change Log

- **Exact self-similar κ_peak, not Newton-on-the-road-seed (implementation method).** The frozen "Always" specified seeding `κ_δ = θ²/(24δ)` and Newton-refining `|P−M| = δ`. Implementation realizes the *same* guarantee (apex vertex-deviation exactly δ when the budget is slack) in closed form: the symmetric biclothoid is **self-similar for fixed θ** (scaling all lengths by `k` preserves θ, scales trim and deviation by `k`, scales κ_peak by `1/k`). One canonical figure at `L=1` (built with the real Fresnel `point_at`) yields `(trim_ref, deviation_ref)`, then `trim = min(trim_ref·δ/deviation_ref, budget)` and `κ_peak = trim_ref·θ/trim` — no iteration, deviation exactly δ. Outcome identical/better than Newton; intent preserved.
- **Follower ratio rescaled by `trim/L`, not raw-inherited (honors the conservation goal).** The frozen line said each clothoid half "inherits its source-side leg's followers … conserves ΔE … within the δ-tube." Keeping the raw per-unit-length ratio would over-extrude by `(L−trim)/trim` (10–20 % at sharp corners, since clothoid arclength `L` exceeds the trimmed tangent length `trim`), violating the conservation the same sentence calls for. Implementation inherits the follower *axis* from the source side but scales the *ratio* by `trim/L`, conserving total ΔE **exactly** (not merely within the δ-tube) — `extrusion_is_conserved_across_a_blend` asserts equality to 1e-9.
- **No SCV/κ_peak floor; the sharp fallback is a full stop, so any finite blend wins.** An early implementation pass added a floor (`κ_peak ≤ a/SCV²`, blend speed ≥ SCV); it was wrong and removed. A biclothoid held to vertex-deviation δ is necessarily pointier than the circular arc of the same δ, so its apex speed sits just below SCV — the floor rejected essentially every blend. But the fallback for an unblended corner under the step-5 law `v=√(a/κ)` is `κ=∞ ⇒ v=0` (a full stop), which every finite blend beats (and which the SPEC's "no zero-velocity dwell while extruding" forbids). Fall-through reasons are therefore `Collinear`, `NearReversal`, `ZeroDeviation` (SCV 0), `ArcIncident`, `NonSpatial`, and `NoBudget` (half-leg ≤ 1e-9 mm). Budget-bound corners tighten and blend (`short_leg_tightens_blend_below_delta`).
- **Road-form `κ_peak ≈ θ²/(24δ)` is a small-angle asymptotic the exact construction supersedes — not asserted.** It diverges fast (≈4.5× at θ=60°) because it is the road-design arc-*shift*, not the corner vertex-deviation. The binding, tested guarantee is `deviation ≤ δ` (`= δ` when the budget is slack), via the exact Fresnel apex.

### Review Findings (code review 2026-06-18)

- [x] [Review][Patch] **Fully-consumed interior leg was silently dropped (observability gap).** All three reviewers: when a short interior line is trimmed from both ends by budget-bound blends, `emit_move` returns without pushing (a zero-length `Line` can't be emitted — `Line::try_new` rejects `len==0`). Edge-case hunter + acceptance auditor confirmed this is **geometrically sound and ΔE-conserving** (the two flanking clothoid halves inherit the consumed leg's follower ratio and absorb its extrusion), so it is not a correctness bug — but a silent move drop sits against the fail-loud/observability stance and the frozen line's own "recorded in `FitReport`." **Resolved:** added `FitReport.consumed_legs`; `emit_move` signals the drop and `fit_corners` counts it. New test `extrusion_conserved_when_short_leg_is_consumed_by_two_blends` (3 moves, short middle leg, both corners budget-bound) asserts `consumed_legs == 1` **and** ΔE conserved to 1e-9 through the drop — closing the auditor's "multi-corner conservation untested" gap. [`fitter.rs`]
- [x] [Review][Patch] **Negative-test obligation for `FitError::Internal` was unproven (acceptance auditor REJECT basis).** The matrix's "Internal break ⇒ `FitError`" row and the AC's negative-test obligation had no test. **Resolved:** `non_finite_line_yields_fit_error_with_source_line` hand-builds a `Move` whose `Line` carries a NaN coordinate (admitted because `Line::try_new` guards only `len==0`, not NaN), which propagates to `Clothoid::try_new` in `canonical` → `DegenerateClothoid`, and asserts the wrapped `FitError::Internal { line_no: 2, source: DegenerateClothoid }` — proving both the loud path and the source-line attribution. [`fitter/tests.rs`]
- [x] [Review][Reject] Blind hunter: `junction_deviation` divides by `accel` with no zero guard (accel=0 ⇒ inf). **Disproven:** `VelocityLimits::check` (the only constructor) requires `accel` finite and `> 0`; limits arrive pre-validated by the frozen contract.
- [x] [Review][Reject] Edge-case hunter: `CornerFitConfig` fields unvalidated (a hand-built NaN band misbehaves). Latent, no callers (only `Default`, which is valid); adding a validator with no consumer is premature for V1.
- [x] [Review][Reject] Blind hunter: `canonical`'s `(0,0)` degenerate return is mislabeled `NoBudget`. The `end_heading[1]≈0` path only triggers at θ→0 / θ→π, already filtered by `theta_min`/`theta_max` before `solve` — unreachable defensive code; the label never manifests.
- [x] [Review][Reject] Blind hunter: `turn_normal`'s `None`→`Collinear` branch is unreachable for in-range θ (redundant with the `theta_min` gate). Defensive, not a bug. Unused-import suspicion disproven by green `clippy -D warnings`.

**Acceptance Auditor verdict:** REJECT pending the two test gaps, with "closing the two test gaps flips this to ACCEPT" — both now closed (the two `[Patch]` entries). Implementation correctness and all four Spec Change Log deviations were audited SOUND; strictly-additive confirmed at the diff level; AC-1..AC-6 satisfied with paired tests.

### Review Findings (independent /bmad-code-review pass, 2026-06-18)

Second, independent 3-layer pass on the post-patch state. **Verdict: ACCEPT** (Acceptance Auditor) — all six ACs + the negative-test obligation satisfied with paired tests, every frozen boundary respected, strictly-additive confirmed. 1 defer, 10 dismissed.

- [x] [Review][Defer] **Near-`theta_max` blends produce extreme `κ_peak` (~1e5–1e6 /mm) and inflated follower ratio (~877× at the π−1e-3 bound, growing as θ→π).** Edge-case hunter. The fitter constructs valid clothoids and conserves ΔE exactly, but hands the velocity planner a near-singular curvature and a large follower ratio with no upper bound (`validate_followers` enforces only finite-and-nonzero). This is the **same class** as the step-3 deferred *tiny-segment follower-ratio blow-up* — geometry has no velocity/flow limit to check against; the principled cap is the extruder-velocity / volumetric-flow bound in the velocity planner (step 5+). `theta_max` is frozen, so the fitter is not narrowed here. Appended to `deferred-work.md`. [`fitter/biclothoid.rs`, `fitter.rs`]
- [x] [Review][Dismiss] **Blind hunter HIGH — "consumed leg → blends no longer meet, continuity broken."** Disproven: `budget = 0.5·min(line_in, line_out)` includes the shared leg, so for an interior leg both flanking trims are each `≤ 0.5·s_len(leg)` ⇒ `trim_start+trim_end ≤ s_len(leg)` ⇒ `new_len ≥ 0`; a leg is dropped only when `new_len ≤ 1e-9 mm`, which *is* the residual gap, so the clothoids meet to within 1 nm (sub-physical). Edge-case hunter (with code execution) confirmed continuity + extrusion hold. The drop is already observable via `FitReport.consumed_legs`.
- [x] [Review][Dismiss] Blind hunter (×8) + auditor note: `emit_blend` scale wrong under budget-clamp (conservation is exact regardless — `ratio·(trim/L)·L = ratio·trim`, proven by the multi-corner test); `canonical` `(0,0)` sentinel mislabel and `corner_x` sign (both unreachable for in-range θ — pre-filtered by `theta_min`/`theta_max` — and guarded → `None` → graceful); self-similar `κ_peak` chain (verified correct by AC-2/AC-3 tests); `turn_normal` dead branch (harmless defensive guard); unused trait imports (disproven — `clippy -D warnings` green); `non_finite` test brittleness (pass-1 classification fails deterministically before any emit); `accel=0` division (`VelocityLimits::check` pre-validates accel finite & >0); trajectory test flake (pre-existing nondeterminism — trajectory has no dependency edge on `geometry::fitter`).

## Design Notes

**Why biclothoid, not Bézier or a Klipper speed-only JD.** Two equal clothoid halves give the only primitive that is simultaneously G1+G2 with straight legs *and* holds a closed-form linear κ the step-5 sweep reads directly (no root-find — the κ_peak is the apex endpoint). Because the symmetric biclothoid is self-similar for fixed θ, κ_peak for an exact vertex-deviation δ is closed-form (one canonical Fresnel figure + a scale), no iteration (see Spec Change Log). Mainline JD inserts *no* geometry and caps faceted arcs (#4228) — this pass is what removes that artifact.

**Fit-or-tighten, never fail.** δ sets the *gentlest* blend (largest L, lowest κ_peak, fastest corner). If that overruns the half-leg budget, a *tighter* blend (larger κ_peak, shorter L) still sits inside δ — only slower. Walking κ_peak up until the trim fits is graceful degradation toward the sharp-corner cap; the floor is "no blend fits ⇒ leave sharp for step 5." This is exactly "κ continuous where it fits."

## Verification

**Commands:**
- `cd rust && cargo nextest run -p geometry` -- expected: all geometry tests pass incl. new `fitter::tests`.
- `cd rust && cargo clippy -p geometry --all-targets -- -D warnings` -- expected: clean.
- `cd rust && cargo nextest run -p trajectory` -- expected: unchanged/green (additive-slice proof).
- `cd rust && cargo fmt --check` -- expected: clean.

## Suggested Review Order

**The fitter pass (parsed move chain → refit chain with clothoid blends)**

- Entry point — the two-pass chain walk: classify each junction, then emit trimmed legs + clothoid halves; counts blended/consumed/unblended.
  [`fitter.rs:69`](../../rust/geometry/src/fitter.rs#L69)

- Junction classification — the fall-through ladder (arc/virtual → collinear → near-reversal → δ-from-SCV → turn-plane → solve); never fails loud on an unblendable corner.
  [`fitter.rs:116`](../../rust/geometry/src/fitter.rs#L116)

- δ derived from the move's existing `VelocityLimits` (Klipper `δ = SCV²(√2−1)/a`) — no new user knob; SCV 0 ⇒ stop-at-corner.
  [`fitter.rs:241`](../../rust/geometry/src/fitter.rs#L241)

**The biclothoid geometry (the technical heart)**

- The solver: δ-optimal-or-budget-tightened trim, exact self-similar κ_peak, world placement of the two `Clothoid` halves via the step-2 evaluator.
  [`biclothoid.rs:15`](../../rust/geometry/src/fitter/biclothoid.rs#L15)

- The canonical figure: one `L=1` biclothoid built with the real Fresnel `point_at`, yielding `(trim_ref, deviation_ref)` for the self-similar scale.
  [`biclothoid.rs:52`](../../rust/geometry/src/fitter/biclothoid.rs#L52)

**Output threading (extrusion + observability)**

- `emit_blend` — each half inherits its source-leg metadata; follower ratio rescaled by `trim/L` for **exact** ΔE conservation.
  [`fitter.rs:201`](../../rust/geometry/src/fitter.rs#L201)

- `emit_move` — trims a line at either/both ends; signals a fully-consumed leg (review patch) instead of dropping it silently.
  [`fitter.rs:165`](../../rust/geometry/src/fitter.rs#L165)

- `FitReport` — `blended` / `unblended` (reasoned) / `consumed_legs`: the observability surface.
  [`fitter.rs:47`](../../rust/geometry/src/fitter.rs#L47)

**Registration + key tests**

- Additive module registration + re-export (the only tracked-file change).
  [`lib.rs:5`](../../rust/geometry/src/lib.rs#L5)

- The seam test (AC-1): blend endpoints/headings reproduce the trimmed-leg seams via the step-2 evaluator.
  [`tests.rs:85`](../../rust/geometry/src/fitter/tests.rs#L85)

- Deviation = δ when slack (AC-3), exact-conservation through a consumed leg (review patch), and the `FitError` negative test (review patch).
  [`tests.rs:298`](../../rust/geometry/src/fitter/tests.rs#L298)
