---
title: 'Build step 3 — typed-IR move builders (parsed G0/1/2/3 → Line/Arc + per-move limits, extrusion as follower, drop G5)'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: 'e6b361614bdd80e182b97486f2b6da505684441a'
context:
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/SPEC.md'
  - '{project-root}/_bmad-output/specs/spec-motion-pipeline-rewrite/architecture.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-1-typed-segment-ir.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-2-execution-lowering.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Steps 1–2 built the typed-segment IR (`geometry::path::{Line,Arc,PathSegment}`) and its lowering, but nothing constructs that IR from G-code. The architecture's "front-end" must NOT be a second G-code parser: the live path already parses G-code in Klipper's Python dispatch (`klippy/gcode.py` — one unified registry shared by built-ins, extended commands, and macros, so `SET_VELOCITY_LIMIT`, coordinate transforms, and user overrides of `G1` all already work), then crosses the PyO3 boundary at parsed-move granularity (`motion.py` → `bridge.rs::submit_move(dx,dy,dz,de,feedrate)` → `classify_and_build` → NURBS `CubicSegment`). The Rust `gcode` lexer is offline-only (tests/fuzz/compat). What's missing is the **typed-IR analog of `classify_and_build`**: stateless Rust builders that turn one already-parsed move into a typed `Line`/`Arc` + extruder follower + the move's kinematic limits — including native arcs (no faceting) and the SCV the current path drops.

**Approach:** Add a `geometry::frontend` module of **stateless** builders (Klipper keeps all modal state — position, abs/rel, extrude mode, active accel/SCV). `line_move(start, end, e_delta, …)` → `Segment::Line` or a virtual retraction move; `arc_move(start, end, i, j, ccw, …)` → a native `Segment::Arc` via closed-form center/sweep (cross-checked against step-2 `point_at`). Extrusion rides as a `FollowerDemand` (`ratio = ΔE / arclength`), mirroring `classify_and_build`. Each builder returns `Move { segment: PathSegment, feedrate_mm_s, limits: VelocityLimits, source }`, where `VelocityLimits { max_velocity, accel, square_corner_velocity }` carries the per-move caps Klipper hands down (feedrate stays the per-move `F` request; `VELOCITY`/`ACCEL`/`SQUARE_CORNER_VELOCITY` from `SET_VELOCITY_LIMIT` are the modal ceilings). Strictly additive: build + test only; no klippy/bridge wiring and no planner cutover (the typed-IR planner is steps 4–8), exactly as steps 1–2 stayed off the live path.

## Boundaries & Constraints

**Always:**
- New module `geometry::frontend`, **stateless** free functions — no interpreter, no token consumption, no modal state (Klipper owns it). Inputs are already-resolved absolute machine coordinates + deltas, as Python computes them today.
- Output `Move { segment: PathSegment, feedrate_mm_s: f64, limits: VelocityLimits, source: SourceRange }`. Feedrate rides alongside the frozen step-1 `PathSegment` (which carries no feedrate); `source` is the single G-code line (`start_line == end_line`).
- `VelocityLimits { max_velocity_mm_s, accel_mm_s2, square_corner_velocity_mm_s }` with a validating `try_new`: `max_velocity` and `accel` finite and `> 0`; `square_corner_velocity` finite and `>= 0` (`0` = skip corner blending entirely / stop at corners, matching Klipper's `minval=0`). SCV is the per-corner δ the fitter (step 4) consumes; dropping it is what makes faceted arcs throttle.
- `line_move(start, end, e_delta, extruder_axis, feedrate_mm_s, limits, source)`: spatial displacement > 1e-9 ⇒ `Line::try_new` wrapped in `Segment::Line`; `|ΔE| > 1e-9` ⇒ `FollowerDemand { extruder_axis, ΔE/spatial_distance }`; no spatial + `|ΔE| > 1e-9` ⇒ `PathSegment::try_new_virtual([sign(ΔE)], |ΔE|)`; no spatial + no E ⇒ typed error (a no-op is a caller bug — Klipper filters feedrate-only lines before calling).
- `arc_move(start, end, i, j, ccw, e_delta, extruder_axis, feedrate_mm_s, limits, source)` (plane XY): center `= (start.x+i, start.y+j)`; `radius = hypot(i,j)`; reject if `|hypot(end−center) − radius|` exceeds tolerance; `start_angle = atan2(start−center)`; sweep normalized to `(0, 2π]` for `ccw` / `[−2π, 0)` otherwise; `start == end` ⇒ full circle `±2π`; reject `start.z != end.z` (helical, fail loud). Emit `Arc::try_new(center3, u=[1,0,0], v=[0,1,0], radius, start_angle, sweep)` so the IR reproduces start→end (verified vs step-2 lowering). Extrusion follower `ΔE / arclength`.
- Fail loudly via a typed `FrontendError` carrying the source line; build no `Move` on any error. Unit tests in a separate file (`frontend/tests.rs`).

**Ask First:**
- Any change to a step-1/step-2 stored field or trait (`PathSegment`, `Segment`, `CurvatureProfile`, lowering). The builders are pure consumers.
- Supporting follower letters beyond the single `extruder_axis` arg (A/B/C extra axes / multi-extruder), or R-form arc input (`G2 R…`), or arc planes other than XY (G18/G19) — each a clean later add; not in this slice.
- Wiring the builders into `bridge.rs`/klippy handlers or cutting the live `submit_move` path over from NURBS. That is the integration step, gated on the typed-IR planner (steps 4–8).

**Never:**
- A G-code lexer/tokenizer/interpreter, `gcode::Token` consumption, or any modal state in Rust — Klipper's dispatch is the parser (reuse it; keep macros/overrides/extended commands/transforms working). The offline `gcode` crate is untouched and stays offline.
- Corner detection, junction records, clothoid blends, fitting, velocity solving, S-curve, faceting an arc into lines — steps 4–8. One move in ⇒ one `Line`/`Arc` out.
- Replacing/renaming the legacy `classify`/NURBS path or the step-1/2 `path` types.
- Exposing a G5 builder (dropped).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Linear move | `start=(0,0,0)`, `end=(10,0,0)`, `e_delta=1` | `Line`; follower `1/10`; `Move` carries feedrate+limits | N/A |
| Travel (no E) | spatial move, `e_delta=0` | `Line`, no follower | N/A |
| Pure retraction | `start==end`, `e_delta=-2` | virtual move, `virtual_path_mm=2`, follower ratio −1 | N/A |
| Zero motion | `start==end`, `e_delta=0` | reject (caller bug) | `ZeroMotion` |
| Arc CW / CCW | I/J, `ccw=false` / `true`, XY | `Arc`, sweep <0 / >0; follower; κ=1/r | N/A |
| Full circle | `start==end`, I/J given | sweep `±2π` | N/A |
| Helical arc | `start.z != end.z` | reject | `HelicalArc` |
| Inconsistent arc | `hypot(end−center) ≠ radius` | reject | `ArcRadiusMismatch` |
| Degenerate arc | `i==j==0` (radius 0) | reject | wraps `GeometryError::DegenerateArc` |
| Bad limits | `accel`/`max_velocity` ≤0, `scv` <0, or non-finite | reject | `InvalidLimits` |
| Bad feedrate | `feedrate_mm_s` ≤0 or non-finite | reject | `InvalidFeedrate` |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/path/{mod,line,arc}.rs` -- `PathSegment::try_new`/`try_new_virtual`, `Line::try_new`, `Arc::try_new(origin,u,v,radius,start_angle,sweep)` — the build targets (read-only). Arc param: `point = origin + r(cosθ·u + sinθ·v)`, `θ(s)=start_angle+sign(sweep)·s/r`.
- `rust/geometry/src/segment.rs:32` -- `FollowerDemand { axis_index, ratio }`; `:218` `SourceRange` — reuse verbatim.
- `rust/motion-engine/src/classify.rs:23` -- `classify_and_build` — the precedent at the SAME parsed-move boundary (spatial vs virtual split, `ratio = delta/distance`, 1e-9 epsilon). Mirror onto the typed IR; do not modify.
- `rust/motion-engine/src/bridge.rs` `submit_move`, `klippy/motion.py` (`SET_VELOCITY_LIMIT`: `square_corner_velocity`, `runtime_accel`) -- where the eventual integration will call these builders and source the limits; **not edited in this slice**.
- `rust/geometry/src/path/lowering.rs` -- step-2 `point_at` — the oracle for AC-FE-2 (arc start/end).
- architecture build-step 3 + "Line↔arc seams"; decided non-goals (G5 drop, helical fail-loud, mainline `gcode_arcs` faceting is the artifact removed).

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/frontend.rs` -- `VelocityLimits` (+ `try_new`), `MoveContext`, `Move`, `FrontendError`, `line_move` (line + virtual + follower), and `arc_move` dispatch.
- [x] `rust/geometry/src/frontend/arc.rs` -- I/J→`Arc` conversion: center/radius, radius-consistency + helical guards, signed-sweep normalization, full circle, `[1,0,0]/[0,1,0]` basis.
- [x] `rust/geometry/src/lib.rs` -- `pub mod frontend;` + re-export `Move`, `MoveContext`, `VelocityLimits`, `FrontendError`, `line_move`, `arc_move`.
- [x] `rust/geometry/src/frontend/tests.rs` -- the I/O matrix (every row incl. each typed error) + the ACs below.

**Acceptance Criteria:**
- (AC-FE-1, the seam) For a generated `Arc`, step-2 `point_at(0)` equals `start` and `point_at(s_len)` equals `end` within tol; the I/J→IR conversion and execution lowering never silently disagree.
- (AC-FE-2) The generated `Arc` reports `kappa ≡ 1/hypot(i,j)` and `s_len == radius·|sweep|`; `ccw=false` yields negative sweep, `ccw=true` positive.
- (AC-FE-3) For a linear move, `follower.ratio · segment.s_len() == ΔE`; for pure retraction the virtual move recovers `|ΔE|` and `ratio·s_len == ΔE`.
- (AC-FE-4) `Move` faithfully carries `feedrate_mm_s` and `limits` (incl. `square_corner_velocity`); `VelocityLimits::try_new` rejects non-finite/non-positive fields.
- (AC-FE-5, additive proof) Workspace builds; `cargo nextest run -p trajectory` is unchanged; step-1/2 `path` types, `CurvatureProfile`, `classify.rs`, and the `gcode` crate are untouched.
- Each matrix fail-loud row returns its exact typed `FrontendError` and builds no `Move` (negative-test obligation).

## Spec Change Log

- **SCV lower bound `>= 0` — RATIFIED (human, 2026-06-18).** Implemented `square_corner_velocity >= 0` (allowing `0`) while keeping `max_velocity > 0` and `accel > 0`. Reason: Klipper's `square_corner_velocity` is `minval=0` — `0` means skip corner blending entirely (stop at corners), a valid setting; rejecting it would break the Klipper-compat the pivot exists to preserve. Surfaced at review as a deviation from the frozen "`> 0`" wording; the human ratified `>= 0`, and the frozen "Always" line + the matrix "Bad limits" row were updated to match (renegotiated intent).
- **Implementation deviation (signature grouping).** The frozen "Always" listed flat positional args (`line_move(start, end, e_delta, extruder_axis, feedrate_mm_s, limits, source)`). The per-move context (`extruder_axis`, `feedrate_mm_s`, `limits`, `source`) is grouped into a `MoveContext` struct so `arc_move` stays within the workspace clippy `too_many_arguments` threshold. Inputs are identical; only their packaging changed.

### Review Findings (code review 2026-06-18)

- [x] [Review][Patch] **Non-finite input not guarded (fail-loud gap).** Edge-case hunter (#1/#2/#6): a NaN/Inf `start`/`end`/`e_delta` on a line became a silent virtual retraction, and a NaN arc-`Z` slipped past the `.abs() > eps` helical gate to be silently dropped. This is the same boundary steps 1→2 logged and fixed at the lowering entry; the new builder entry needed the same guard. **Resolved:** `line_move`/`arc_move` reject non-finite `start`/`end`/`i`/`j`/`e_delta` with a new `FrontendError::NonFiniteInput` before any geometry; paired tests `line_move_non_finite_coordinate_rejected`, `arc_move_non_finite_rejected_before_helical_check`. [`frontend.rs`]
- [x] [Review][Patch] **Arc radius-consistency tolerance was radius-proportional.** Edge-case hunter (#3): `1e-4 + 1e-4·r` ballooned to 0.1 mm slop at r=1000 (radial inconsistency is radius-*independent*). **Resolved:** retuned to `1e-3 mm + 1e-6·r` — an absolute floor with a negligible relative term (~2e-3 mm at r=1000, 50× tighter), still tolerant of slicer coordinate quantization. [`frontend/arc.rs`]
- [x] [Review][Reject] Blind hunter flagged `CurvatureProfile`/`PositionProfile` as unused imports → **false positive**: `s_len`/`kappa`/`point_at` are trait methods (`arc.s_len()` in `arc_move`), and `clippy -D warnings` is green, proving they're used.
- [x] [Review][Defer] Tiny-but-nonzero arc/line length carrying real extrusion yields a huge follower ratio (extruder velocity spike). Sub-physical, mirrors existing `classify.rs`; the principled bound is the planner's extruder-velocity cap (geometry has no velocity limit) — deferred to that consumer.
- [x] [Review][Reject] Full-circle knife-edge at ~1e-12 rad endpoint jitter — inherent to the `start==end ⇒ full circle` convention, sub-physical.

**Acceptance Auditor verdict: ACCEPT** — all ACs satisfied with paired tests; strictly additive (`classify.rs`, the `gcode` crate, and step-1/2 `path` types byte-for-byte unchanged); both logged deviations sound.

## Design Notes

**Why stateless builders, not a Rust front-end interpreter.** Investigation of the live path: Klipper's `gcode.py` is a single unified dispatch (built-ins + extended commands + macros share one registry; classic-vs-`KEY=VALUE` param style is chosen at registration via `is_traditional_gcode`), and the PyO3 boundary already sits at parsed-move granularity (`submit_move(dx,dy,dz,de,feedrate)`). A Rust tokenizer would duplicate that parser and forfeit macros/overrides/extended-commands/transforms. So step 3 is the typed-IR twin of `classify_and_build`: Python parses and owns modal state; Rust gets resolved numbers and builds geometry. This also dissolves the `SET_VELOCITY_LIMIT`/SCV parsing problem — `motion.py` already parses it; the limits arrive as `VelocityLimits` args.

**Arc conversion is exact against lowering.** `u=[1,0,0]`, `v=[0,1,0]` (right-handed about +Z) makes `θ(s)=start_angle+sign(sweep)·s/r` reproduce the gcode endpoints by construction, with CCW(+sweep)↔`ccw=true` falling out directly; AC-FE-1 ties this to step-2 `point_at`. Native arc, never faceted — the win over mainline `gcode_arcs`.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p geometry` -- expected: all geometry tests pass incl. new `frontend::tests`.
- `cd rust && cargo clippy -p geometry --all-targets -- -D warnings` -- expected: clean.
- `cd rust && cargo nextest run -p trajectory` -- expected: unchanged/green (additive-slice proof).
- `cd rust && cargo fmt --check` -- expected: clean.

## Suggested Review Order

**The builder boundary (parsed move → typed IR)**

- Entry point — the linear builder: spatial `Line` vs virtual retraction, extruder follower `ΔE/distance`, fail-loud zero-motion.
  [`frontend.rs:92`](../../rust/geometry/src/frontend.rs#L92)

- The arc builder: validates inputs, delegates the geometry, attaches the follower over arc length.
  [`frontend.rs:137`](../../rust/geometry/src/frontend.rs#L137)

- Non-finite input guard (review patch) — fail loud before any geometry; the fix that closes the lowering-boundary parity gap.
  [`frontend.rs:101`](../../rust/geometry/src/frontend.rs#L101)

**The arc math (I/J → planar `Arc`, exact against lowering)**

- Center/radius, helical + radius-consistency guards, basis choice so the IR reproduces the gcode endpoints.
  [`arc.rs:12`](../../rust/geometry/src/frontend/arc.rs#L12)

- Signed-sweep normalization — CCW→(0,2π], CW→[−2π,0), full circle ±2π.
  [`arc.rs:48`](../../rust/geometry/src/frontend/arc.rs#L48)

- Radius-consistency tolerance (review patch) — absolute floor + negligible relative term, no longer ballooning at large radii.
  [`arc.rs:44`](../../rust/geometry/src/frontend/arc.rs#L44)

**Output + error contract**

- `Move` carries feedrate + `VelocityLimits` (incl. SCV) alongside the frozen `PathSegment`; `VelocityLimits::check` is the fail-loud limit gate.
  [`frontend.rs:10`](../../rust/geometry/src/frontend.rs#L10)

- Typed `FrontendError` — every variant carries the source line.
  [`frontend.rs:64`](../../rust/geometry/src/frontend.rs#L64)

**Registration + tests**

- Additive module registration + re-export (the only tracked-file change).
  [`lib.rs:5`](../../rust/geometry/src/lib.rs#L5)

- The seam test (AC-FE-1): generated `Arc` lowered via step-2 `point_at` reproduces gcode start/end.
  [`tests.rs:165`](../../rust/geometry/src/frontend/tests.rs#L165)

- Non-finite negative tests (review patch).
  [`tests.rs:247`](../../rust/geometry/src/frontend/tests.rs#L247)
