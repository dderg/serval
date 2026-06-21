---
title: 'Build step 9 — profile lowering (the EX/EV stage): compose the geometry + the planned v(s) into one fixed-rate position-vs-time stream via a constant-accel-per-breakpoint s(t); Fresnel position, virtual-path followers, finite-time stops — the walking skeleton now runs end-to-end'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: 'c5643cab2'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-2-execution-lowering.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-7-limit-riding.md'
  - '{project-root}/_bmad-output/implementation-artifacts/deferred-work.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The pipeline now has geometry (`FitOutcome` — typed `Line|Arc|Clothoid` + follower channel) **and** a planned speed profile (`VelocityProfile` — a continuous `v(s)` per move, step 7), but nothing joins them. Execution lowering is still only step-2's `lower_constant_speed`, which takes a **scalar** speed for a **single** segment and ignores the planned profile entirely. So the walking skeleton does not yet run end-to-end: there is no position-vs-time output that reflects the planned velocities. Build-sequence step 9, the **EX/EV node** (`geometry + s(t) → fixed-rate position-vs-time`), realizing **CAP-4**.

**Approach:** Add a profile-driven lowering pass that consumes the geometry and the velocity profile together and emits **one fixed-rate `Vec<LoweredSample>` stream** spanning the whole move sequence. Per move, invert the profile's `(s, v)` breakpoints into a time map `s(t)` under a **constant tangential acceleration per breakpoint interval** (`aᵢ = (vᵢ₊₁²−vᵢ²)/(2Δs)`): `s(t)=sᵢ+vᵢ·dt+½aᵢ·dt²`, interval duration `Δtᵢ=(vᵢ₊₁−vᵢ)/aᵢ` (= `2Δs/(vᵢ+vᵢ₊₁)`, exactly velocity's own trapezoid quadrature — so the two time integrators agree to rounding, and a `v=0` stop is reached in *finite* time). Sweep a single global time grid `tₖ=k/rate_hz`, locate the bracketing move+interval, evaluate position via the existing step-2 `PositionProfile` (clothoid by **Fresnel**, no arc-length table) and followers as `ratio·s(t)`. Seam speeds are already continuous (`exit_v==next.entry_v`), so time accumulates monotonically with no dwell or clamp inserted.

## Boundaries & Constraints

**Always:**
- New, additive surface only. `lower_constant_speed`, `PositionProfile`, `LoweredSample`, every step-1/2/3/4/5/6/7/8 stored type, and `velocity`/`fitter`/`frontend`/`path`/`segment`/`gcode`/`pipeline`/`trajectory` are byte-for-byte untouched. The new pass lives in a **new top-level module** `geometry::execution` (it reads the velocity-layer `VelocityProfile`, so it must sit *above* `path` — it imports `PositionProfile` downward, never the reverse; placing it in `path/lowering.rs` would invert the layer graph).
- Geometry and profile align by index: `geometry.moves[j]` ↔ `profile.moves[j]`. Fail loud on a length mismatch and on any per-move `source` (`SourceRange`) mismatch — a desynced pairing is a bug, never silently lowered.
- Time model is **constant tangential acceleration per profile breakpoint interval** — the exact inverse of `velocity::traversal_time`'s trapezoid quadrature. Therefore the lowered stream's total duration equals `profile.report.traversal_time_s` to rounding, and a rest node (`v=0`) is reached and left in finite time. No reparameterization grid is created: the per-move time table is the profile's own **adaptive** breakpoints (O(breakpoints)), not a uniform arc-length resampling.
- Clothoid position comes from the step-2 Fresnel `PositionProfile` (`fresnel::clothoid_offset`), never a per-segment arc-length table (CAP-4). Virtual-path moves (`spatial: None`, `virtual_path_mm: Some`) lower with `position: None` and followers `ratio·s(t)`.
- The output is one ascending-`t_s` stream for the whole sequence sampled at `dt=1/rate_hz`, starting at `t=0`; move seams are not special (sampled by the global grid, so no duplicated seam sample). `LoweredSample` shape (`t_s`, `position: Option<[f64;3]>`, `followers: Vec<f64>`) is reused unchanged.
- Fail loud (`GeometryError::InvalidLowering { reason }`) on: non-finite/≤0 `rate_hz`; move-count or `source` mismatch; a per-move sample list that is not monotone-nondecreasing in `s`, does not start at `s=0`, does not end at `s≈s_len`, or carries negative/non-finite `v`; a zero-progress interval (`vᵢ=vᵢ₊₁=0` with `Δs>0`); a non-finite spatial anchor (reuse `spatial_anchors_finite`). The fixed-rate evaluator must reproduce the planned path **within step tolerance** at the configured frequency.

**Ask First:**
- Per-axis time-polynomial / spline breakpoint emission (vs the dense fixed-rate sample stream) — if a downstream MCU transport wants polynomial segments rather than samples.
- Wiring `lower_profile` into `trajectory` / the live streaming path, or carrying `axis_index` on follower output (step-2 deferred-work item — followers stay a positional `Vec<f64>` in `segment.followers` order here).
- Shaper / pressure-advance post-processors, the volumetric-flow / PA-augmented transient cap, bed-mesh Z field, `z_thermal_adjust`, `exclude_object` (CAP-5/6/7/8 — all later stages).

**Never:**
- No re-introduced **planning** arc-length grid / 1024-point reparameterization table (the EV fixed-rate *time* grid is explicitly the separate, allowed evaluator grid). No velocity re-planning or re-derivation in lowering — it is a pure consumer of the already-planned `v(s)`.
- No `point_at`/`heading_at`/`PositionProfile`/Fresnel leaking *into* `velocity` (the κ-only read-contract stays intact). No edits to any stored geometry/velocity type. No dwell, start-time padding, or speed clamp inserted to "fix" a profile — an infeasible profile fails loud, never gets massaged.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Trapezoid line | one `Line`, accel→cruise→decel `v(s)` | fixed-rate stream; per interval `s(t)=v₀t+½at²` exact; final position == line end; positions all on the line | N/A |
| Blended-corner chain | line→clothoid→clothoid→line, `v(s)` continuous | one monotone-`t_s` stream; position continuous across each seam (matches `point_at` at the shared node); followers monotone non-decreasing | N/A |
| Full-stop node | `v=0` at an interior junction | finite time into/out of rest (constant-accel); no infinite gap; no two consecutive equal `t_s` (no dwell) | N/A |
| Pure retraction | `spatial: None`, `virtual_path_mm` | `position: None`; followers advance as `ratio·s(t)` | N/A |
| Empty profile | 0 moves | empty `Vec` | N/A |
| Time-grid total | any chain | last `t_s` == `profile.report.traversal_time_s` (to rounding) | N/A |
| Bad rate | `rate_hz` ≤0 / non-finite | — | `Err(InvalidLowering)` |
| Desynced inputs | `moves.len()` differ, or `source` mismatch at any `j` | — | `Err(InvalidLowering)` |
| Bad profile samples | `s` non-monotone / `s₀≠0` / `s_last≠s_len` / `v<0` / non-finite / zero-progress interval | — | `Err(InvalidLowering)` |
| Non-finite anchor | spatial anchor NaN/∞ | — | `Err(InvalidLowering)` |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/execution.rs` — **new**: `pub fn lower_profile(geometry: &FitOutcome, profile: &VelocityProfile, rate_hz: f64) -> Result<Vec<LoweredSample>, GeometryError>`. Validate `rate_hz` + index alignment (count + per-`j` `source`); per move build the constant-accel `s(t)` map from `MoveVelocity.samples` (with the sample-list validation above); a single global ascending sweep over `tₖ=k/rate_hz` that locates move+interval, evaluates `s(t)`, then `position` via `PositionProfile` (Fresnel clothoid; `None` for virtual) and `followers = ratio·s`. Ends with `#[cfg(test)] mod tests;`.
- `rust/geometry/src/execution/tests.rs` — **new**: every I/O-matrix row incl. all `Err` paths; the `traversal_time_s` cross-check; the constant-accel exactness check; seam-position continuity; finite-stop timing; virtual-path followers.
- `rust/geometry/src/path/lowering.rs` — **unchanged**; `lower_profile` *imports* `LoweredSample` + `PositionProfile` from here. `lower_constant_speed` is retained as the step-2 single-segment observability tool.
- `rust/geometry/src/lib.rs` — add `pub mod execution;` and re-export `execution::lower_profile`.
- `_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md` — add **build-sequence step 9** and mark the **EX/EV** pipeline node live (profile-driven fixed-rate lowering; constant-accel-per-breakpoint `s(t)`; Fresnel position; finite-time stops; walking skeleton end-to-end).

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/execution.rs` — implement `lower_profile`: input validation + index/source alignment; per-move constant-accel `s(t)` from profile breakpoints with the sample-list fail-loud guards; global fixed-rate sweep; position via `PositionProfile` (Fresnel / `None` virtual) + `ratio·s` followers. Unit tests per `execution/tests.rs`.
- [x] `rust/geometry/src/execution/tests.rs` — all matrix rows (incl. every `Err`), the `traversal_time_s` cross-check, constant-accel exactness, seam continuity, finite-stop, virtual-path followers.
- [x] `rust/geometry/src/lib.rs` — `pub mod execution;` + re-export `lower_profile`.
- [x] `_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md` — add step 9 / mark EX/EV live.

**Acceptance Criteria:**
- Given a single `Line` with an accel→cruise→decel profile, when lowered at `rate_hz`, then every sample position lies on the line, the reconstructed `s(t)` equals `v₀·t+½a·t²` within `1e-9` on each constant-accel interval, the final sample position equals the line end within `1e-9`, and the last `t_s` equals `profile.report.traversal_time_s` within `1e-9`.
- Given any planned chain, when lowered, then `t_s` is strictly ascending at `1/rate_hz` spacing and the stream's total duration (last `t_s`) equals `report.traversal_time_s` within `1e-9` (the lowering time model is the exact inverse of velocity's trapezoid quadrature).
- Given a line→clothoid→clothoid→line blend with `v(s)` continuous, when lowered, then position is continuous across each move seam (the bracketing samples agree with `point_at` at the shared node within `1e-6`) and the follower stream is monotone non-decreasing.
- Given a chain with a full-stop node (`v=0`), when lowered, then the bracketing intervals take finite time matching the constant-accel value and no two consecutive samples share a `t_s` (no dwell, no padding).
- Given a pure-retraction virtual move, when lowered, then `position` is `None` and followers advance as `ratio·s(t)`.
- Given `rate_hz≤0`, mismatched move counts, a mismatched `source`, non-monotone/`s₀≠0`/`s_last≠s_len`/negative/non-finite samples, a zero-progress interval, or a non-finite anchor, when lowered, then the matching `Err(InvalidLowering)` fires (fail-loud).
- (Additive elsewhere) workspace builds; `cargo nextest run -p trajectory` unchanged; `velocity`/`fitter`/`frontend`/`path`/`segment`/`pipeline`/`gcode` byte-for-byte untouched; `velocity` source still contains no `point_at`/`heading_at`/`PositionProfile`.

## Spec Change Log

- [Review][Patch] **Non-finite `s_len` bypassed the span guard → silent single garbage sample (Blind Hunter, MED; corroborated by Edge Hunter's `spatial_anchors_finite` field-coverage note).** A NaN/Inf segment length (e.g. a hand-built `Arc` with NaN `radius`, whose anchor fields pass `spatial_anchors_finite`) made every `(s_last − s_len)` comparison false (NaN compares false), so the malformed move slipped through; the NaN then propagated to `total_t`/`count`, and `(NaN).ceil() as usize == 0` emitted one bogus sample instead of failing loud — a direct fail-loud-non-negotiable violation. Fix: explicit `s_len.is_finite() && s_len > 0.0` guard per move (reason `"segment length is not finite and positive"`), which also subsumes the Arc-`radius`/Clothoid-`sigma` anchor-coverage gap since both flow into `s_len`. Paired test `non_finite_segment_length_is_rejected`. [`execution.rs`]
- [Review][Patch] **Sample-count guard bounded `count`, not the actual `n+1` element count, and admitted a non-finite `count` (Blind Hunter #4 + Edge Hunter #1, MED/latent).** `count >= usize::MAX as f64` left `Vec::with_capacity(n+1)` able to wrap and did not reject a `+inf` count. Hardened to `!count.is_finite() || count + 1.0 >= usize::MAX as f64`. [`execution.rs`]
- [Review][Patch] **Seam-continuity AC tested only a Lipschitz no-jump bound, not the literal "agree with `point_at` at the shared node" (Acceptance Auditor F1, MED).** Added `seam_samples_match_point_at_at_the_shared_node`: asserts `point_at(s_len)` of move j equals `point_at(0)` of move j+1 within `1e-6` and that the lowered stream passes through that node. [`execution/tests.rs`]
- [Review][Patch] **`½at²` exactness checked only on a single-interval synthetic profile (Acceptance Auditor F3, LOW).** Added `constant_accel_inversion_is_exact_across_multiple_intervals` (accel→decel two-interval profile) verifying piecewise `s(t)` across the breakpoint seam + phase reset. [`execution/tests.rs`]
- [Review][Defer] **Unbounded-but-addressable sample count from absurdly-tiny-positive `v_sum`.** `v_sum > 0` admits ~`1e-12` velocities → enormous-but-finite `n` that OOM-aborts rather than returning `InvalidLowering`. Not reachable from real `plan_velocity` (velocity floor `1e-9`), and identical in shape to the pre-existing `lower_constant_speed`; a deliberate max-samples cap is a separate decision. Recorded in `deferred-work.md`.
- [Review][Reject] **Same-formula `traversal_time` cross-check (Auditor F2 — by-design per spec), evaluator `s` clamp (Auditor F4 — allowed evaluator clamp, not a feasibility clamp), tail duplicate-timestamp (Blind #2 — benign in any non-degenerate rate regime, caught by the strictly-ascending test), tiny-rate→`dt=inf` (Edge #2 — defined degenerate behavior), and float nits (Blind #1/#5/#6).** No trajectory-correctness effect; most self-withdrawn by the reviewers. KEEP: the constant-accel time model is deliberately the exact inverse of `velocity::traversal_time` (the two-integrator agreement is the headline cross-check, not an independence claim).

## Design Notes

**Constant-accel-per-breakpoint `s(t)` (the inversion).** The profile gives monotone `(sᵢ,vᵢ)` breakpoints per move (incl. both endpoints). Over `[sᵢ,sᵢ₊₁]` take constant tangential accel `aᵢ=(vᵢ₊₁²−vᵢ²)/(2Δs)`; then `v(t)=vᵢ+aᵢ·dt`, `s(t)=sᵢ+vᵢ·dt+½aᵢ·dt²`, `dt=t−tᵢ`. Interval duration `Δtᵢ=(vᵢ₊₁−vᵢ)/aᵢ=2Δs/(vᵢ+vᵢ₊₁)` (`=Δs/vᵢ` when `aᵢ≈0`). That `2Δs/(v₁+v₂)` is *identically* the formula `velocity::traversal_time` integrates, so `Σ Δtᵢ == report.traversal_time_s` to rounding — the two integrators are the same quadrature, which is both the headline cross-check and why this is the correct between-breakpoint law. A `v=0` rest node is reached in finite time (constant decel), unlike a v-linear-in-`s` model (which diverges as `v→0`).

**Why a new `execution` module, not `path/lowering.rs`.** `velocity` and `fitter` depend on `path`; `lower_profile` reads the velocity-layer `VelocityProfile` *and* the path-layer `PositionProfile`, so it must sit above `path`. Putting it in `path/` would force `path → velocity`, a cycle. `execution` imports `PositionProfile`/`LoweredSample` downward and is imported by nobody below it.

**This is EX *and* EV.** The architecture's EX ("`geometry + s(t) → time-polynomials`") and EV ("fixed-rate evaluator") collapse into one pass here: we emit dense fixed-rate samples directly rather than intermediate polynomials (polynomial emission is an Ask-First). The fixed-rate *time* grid is the legitimate evaluator grid the architecture explicitly distinguishes from the deleted planning arc-length table; per-move time tables are the profile's own adaptive breakpoints, not a uniform reparameterization.

Sketch (one fixed-rate query, within a move):
```
let i = bracket_interval(t);                 // breakpoint interval containing t
let dt = t - t_at[i];
let a  = (v[i+1]*v[i+1] - v[i]*v[i]) / (2.0 * (s[i+1] - s[i]));
let s_t = s[i] + v[i]*dt + 0.5*a*dt*dt;       // clamp to s[i+1] at the seam
let position = seg.spatial.as_ref().map(|g| g.point_at(s_t));   // Fresnel for clothoid
let followers = seg.followers.iter().map(|f| f.ratio * s_t).collect();
```

## Verification

**Commands:**
- `cargo nextest run -p geometry` — expected: new `execution` tests pass; all other geometry tests unchanged.
- `cargo nextest run -p trajectory` — expected: unchanged (proves the pass stays inside `geometry`, additive).
- `./scripts/ci.sh rust-clippy && ./scripts/ci.sh rust-fmt` — expected: green (`-D warnings`).
- `! grep -rnE 'point_at|heading_at|PositionProfile' rust/geometry/src/velocity.rs rust/geometry/src/velocity/` — expected: no matches.
- `git diff --stat` on `rust/geometry/src/{velocity.rs,velocity,fitter.rs,frontend.rs,segment.rs,pipeline.rs,path}` and `rust/gcode` — expected: empty (only `execution.rs`, `execution/tests.rs`, `lib.rs`, and the architecture doc change).

## Suggested Review Order

**Design intent (start here)**

- Entry point: the whole pass — validate, build constant-accel phases per move, sweep the fixed-rate grid.
  [`execution.rs:17`](../../rust/geometry/src/execution.rs#L17)

**The s(t) inversion — the highest-leverage math**

- Per breakpoint interval: constant-accel `a=(v1²−v0²)/2Δs`, duration `2Δs/(v0+v1)` (= velocity's own trapezoid quadrature → finite-time stops, time agrees with `report`).
  [`execution.rs:86`](../../rust/geometry/src/execution.rs#L86)
- Fixed-rate sweep + monotone phase-advance pointer; `s(t)=s0+v0·dt+½a·dt²` clamped to the interval.
  [`execution.rs:118`](../../rust/geometry/src/execution.rs#L118)

**Fail-loud boundary (κ-space producer can't reach these, but they guard hand-built / future input)**

- Segment-length finiteness — added in review; a NaN `s_len` (e.g. NaN `Arc.radius`) would otherwise emit a silent garbage sample.
  [`execution.rs:53`](../../rust/geometry/src/execution.rs#L53)
- Sample-list guards: span both endpoints, strictly-increasing `s`, finite/non-negative `v`, no zero-progress stall.
  [`execution.rs:59`](../../rust/geometry/src/execution.rs#L59)
- Addressable-count guard — hardened in review to bound the actual `n+1` and reject non-finite counts.
  [`execution.rs:106`](../../rust/geometry/src/execution.rs#L106)
- Local anchor-finiteness check (keeps `path/lowering.rs` byte-for-byte untouched).
  [`execution.rs:136`](../../rust/geometry/src/execution.rs#L136)

**Wiring**

- New top-level module above `path`, re-exported.
  [`lib.rs:5`](../../rust/geometry/src/lib.rs#L5)

**Tests (peripherals)**

- Headline: stream duration equals `report.traversal_time_s` on a real planned line.
  [`tests.rs:126`](../../rust/geometry/src/execution/tests.rs#L126)
- Two-integrator agreement across a blended-corner chain (Fresnel `point_at` exercised).
  [`tests.rs:173`](../../rust/geometry/src/execution/tests.rs#L173)
- Direct shared-node `point_at` seam agreement (review patch — literal AC).
  [`tests.rs:437`](../../rust/geometry/src/execution/tests.rs#L437)
- Constant-accel `½at²` exactness across multiple intervals (review patch).
  [`tests.rs:477`](../../rust/geometry/src/execution/tests.rs#L477)
- Finite-time stop, no dwell; virtual-path `position: None`; every fail-loud `Err` path.
  [`tests.rs:364`](../../rust/geometry/src/execution/tests.rs#L364)
