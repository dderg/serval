---
title: 'Motion-11: wire the new geometry Move pipeline into production behind submit_move; remove the host NURBS planner'
type: 'feature'
created: '2026-06-19'
status: 'in-progress'
baseline_commit: '156c66214e14bff7833035205accf735a40637c7'
context: ['{project-root}/CLAUDE.md', '{project-root}/_bmad-output/implementation-artifacts/spec-motion-10-pipeline-integration.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The new geometry pipeline (`fit_chain → plan_velocity → lower_profile`, specs 4–9) is end-to-end proven in Rust (spec-10) but **dormant** — nothing in production runs it. The live path behind the PyO3 `submit_move` seam still builds host NURBS `CubicSegment`s and plans them with the `trajectory`/`temporal` velocity engine, which stops at every geometric corner. The new pipeline blends corners (clothoid·arc·clothoid), so cutting over is strictly an upgrade.

**Approach:** At the `submit_move` seam, build geometry `Move`s (`frontend::line_move`) instead of `CubicSegment`s, drive them through the new pipeline with a **bounded look-ahead buffer** that preserves cross-move velocity continuity (back-to-back collinear jogs do not stop between them), and lower the result onto the **existing, unchanged MCU emit backend** (`ShapedSegment` → `enqueue_segment` → kinematics lane-mixing → clock projection → cubic-Bézier `PieceEntry` → pump → MCU ring). Then delete the host NURBS planner path. The goal is **correctness/safety parity** (follower/axis mapping, MCU emit format, fail-loudly) and getting real slicer G-code to print — **not** trajectory-time parity, and **not** input shaping (deferred).

## Boundaries & Constraints

**Always:** Keep the PyO3 signatures (`submit_move`/`submit_bezier`/`submit_quadratic`) and the Python wrapper/`motion.py` callers byte-for-byte unchanged. Reuse the existing emit backend from `enqueue_segment` downward verbatim (kinematics, `host_time_to_mcu_clock` projection, `PieceEntry` cubic-Bézier format, pump, MCU ring). Preserve follower/extruder axis→motor mapping and `motor_mask` semantics. Populate the new pipeline's `VelocityLimits` (max_velocity, accel, square_corner_velocity) from the existing `PlannerConfig`/`RuntimeCaps`. Keep the planner thread architecture (`PlannerHandle`, crossbeam channel, `Dwell`/`Flush`/`UpdateRuntimeCaps`/`Shutdown` handling). Fail loudly on out-of-contract input (late segment / start-time-in-past → error, never pad).

**Ask First:** Changing any PyO3 signature or the `PieceEntry`/MCU wire format. Touching the MCU-side NURBS evaluator (`c-api`/`nurbs.h`/`runtime` eval) or the `nurbs` crate itself. Deleting the `trajectory`/`temporal` crates wholesale (vs. retaining their kinematics/projection/emit parts as a parts-bin). The G5/curve handling strategy (flatten-to-line-facets vs. fail-loudly) if the recommendation below is rejected.

**Never:** Add input shaping in this spec (explicitly deferred — print without any). Add a Rust G-code reducer or parse G-code text (the lexer stays in Python). Remove or modify the MCU-side NURBS eval, the `nurbs` crate, or the cbindgen headers. Ship a path that **stops between consecutive collinear moves** (that is a regression vs. the old path, not an upgrade). Re-emit or mutate pieces already committed to the MCU.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Single G1 line | one `submit_move(dx,dy,dz,0,feed)` | builds `Move`; on flush, lowers to cubic-Bézier `PieceEntry`(s); spatial axes move; prints | finite checks; else `Err` |
| Back-to-back collinear jogs | two `submit_move` same direction | **no stop between them** — committed pieces are position- and velocity-continuous across the junction | N/A |
| Cornered polyline | several `submit_move` forming corners | corners blended via clothoid (no per-corner full stop); slows through corner per curvature cap | N/A |
| Extruding move | `submit_move` with `de != 0` | follower E mapped to its registry axis & `motor_mask`; extruder displacement monotone, endpoint matches `de` | N/A |
| Flush / Dwell / program end | `PlannerMsg::Flush`/`Dwell` | entire look-ahead buffer committed, decelerating to rest; no move stranded uncommitted | N/A |
| Curve (G5/quadratic) | `submit_bezier`/`submit_quadratic` | flattened to short line facets fed as `Move`s (fit_chain re-smooths) — see Design Notes | finite checks; else `Err` |
| Late segment | move whose start time is already in the past | raise a clear error (do not advance/pad the start time) | `Err`/`PyRuntimeError` |
| Empty / zero-displacement | flush with empty buffer; zero-length move | no-op, no panic, no emitted pieces | N/A |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/bridge.rs` -- `submit_move`/`submit_bezier`/`submit_quadratic` (≈3264–3415) + the `dispatch` closure & `project` clock closure (≈3060–3151). Build `Move` here; route to planner. Keep dispatch/project.
- `rust/motion-engine/src/classify.rs` -- currently builds `CubicSegment` + `FollowerDemand` from deltas. Repurpose to build a geometry `Move` (`frontend::line_move`) + follower mapping; drop the NURBS construction (`build_cubic`).
- `rust/geometry/src/velocity.rs` -- `plan_velocity` is **rest-to-rest only** today (`v[0]=v[n]=0`, velocity.rs:157). **Stage-1 prerequisite:** add a streaming **entry velocity** (`v[0]=entry_v`); fail loudly if the pinned entry can't brake to `v=0` within the buffer. The pipeline is **C1** (only `v[k]` crosses junctions, no accel state) — so entry velocity is the *only* boundary datum streaming needs.
- `rust/geometry/src/execution.rs` -- `lower_profile` already warm-start-compatible (integrates any `v(s)` with `v_sum>0`); times emit from 0 and are offset by the dispatched `t0` in the planner.
- `rust/motion-engine/src/planner.rs` -- `PlannerHandle`, `PlannerMsg`, `run_loop`. **Replace** `ShaperState` (the `trajectory` streaming engine) with a new look-ahead `VecDeque<Move>` streaming layer: re-plan the uncommitted window warm-started from the dispatched velocity (terminal `v=0`), commit on the existing real-time cadence (`LEAD`/`sync_instant`/idle-timeout reused). Keep channel/thread/Dwell/Flush/caps message surface.
- `rust/motion-engine/src/lowering.rs` -- **NEW** adapter: `(FitOutcome, VelocityProfile, t0)` → `ShapedSegment` (per-axis piecewise-cubic time curves + followers + `motor_mask`), at controlled fidelity.
- `rust/motion-engine/src/enqueue.rs` -- `enqueue_segment(&ShapedSegment, …)` + `lane_curve` (kinematics) + `flatten_bezier_pieces`. **Reused unchanged** — the emit contract the adapter targets.
- `rust/motion-engine/src/config.rs` -- `PlannerConfig`/`LimitSection`/`RuntimeCaps`. Source for `VelocityLimits`.
- `rust/geometry/src/{frontend,fitter}.rs` -- `line_move`, `fit_chain` driven by the planner.
- `rust/geometry/src/pipeline.rs`, `rust/geometry/src/segment.rs` -- legacy host `GeometryPipeline`/`CubicSegment` — **removal candidates** once no consumer remains.
- `rust/trajectory/src/streaming/`, `rust/temporal/` -- the old NURBS+SOCP velocity/streaming engine. **Consumption removed** (motion-engine no longer calls `append_and_replan`); `ShapedSegment`/emit types retained as the emit contract.
- `rust/motion-engine/tests/{streaming_replan,follower_lane_e2e,binding_report_e2e,pump_loop,z_hop_cold_boot}.rs` -- e2e tests on the old planner internals; update to the new pipeline.

## Tasks & Acceptance

**Stage 1 — velocity-planner streaming warm-start (geometry crate, prerequisite):** ✅ done
- [x] `rust/geometry/src/velocity.rs` -- add a streaming entry velocity to `plan_velocity` (`v[0]=entry_v`, clamped to the first move's caps; terminal `v=0`). After the backward pass, if the pinned `entry_v` exceeds what can brake to rest within the window, return a new `VelocityError::OverCommitted{line_no}` (fail loudly — insufficient look-ahead). Rest-to-rest = `entry_v=0`. Unit-test warm-start continuity + the over-commit error in `velocity/tests.rs`.

**Stage 2 — streaming engine + lowering adapter (motion-engine):**
- [x] `rust/motion-engine/src/lowering.rs` -- **NEW** adapter `lower_move(gm, vm, t_start, start_pos, fit_tol)` → `ShapedSegment` per-axis cubic-Hermite time curves (exact for lines; subdivided-to-tol for arc/clothoid; followers + constant-hold axes). 7 unit tests green (endpoint fidelity, follower delta, arc tolerance, virtual extrude, monotone+speed-cap).
- [x] `rust/motion-engine/src/stream.rs` -- **NEW** `StreamState` streaming core: `VecDeque<Move>` look-ahead, `fit_chain → plan_velocity_warm_start`, **clean-seam (non-blended) prefix commit** (Option A), odometer + entry-velocity carry, force-to-rest. 5 unit tests green (collinear continuity, blend never split, flush-to-rest, extrusion odometer, time-contiguity).
- [x] `rust/motion-engine/src/stream_planner.rs` -- **NEW** `StreamPlannerHandle`: streaming `run_loop` over `StreamState` (additive, alongside the old `planner.rs`). Idle-drain cadence (`LEAD`/`sync_instant`), `Move`/`Flush`/`Dwell`/`StreamOpen`/`Reset`/**`HomeDrip`**/**`Nudge`** (Option A — homing/nudge on the new pipeline)/`Shutdown`. Clock anchoring delegated to the existing `dispatch`/`Anchor` (auto-re-anchors on stream reset, fails loud on lateness). 5 standalone tests green (contiguous trajectory, dwell gap, stream-open reset, homing endpoint, nudge dispatch+time-advance).

**Stage 3 — cutover + NURBS removal:** *(remaining phase)*
- [x] `rust/motion-engine/src/config.rs` -- `path_velocity_limits()` (spatial caps + runtime-caps override + default scv) and `path_velocity_config()` (spatial jerk). Clippy-clean.
- [x] `rust/motion-engine/src/classify.rs` -- **additive** `build_move()` builds `geometry::Move` (line + optional extruder follower). `classify_and_build` retained until the bridge switch.
- [ ] `rust/motion-engine/src/stream_planner.rs` -- add `HomeDrip`/`Nudge` handlers on the new pipeline (Option A).
- [ ] `rust/motion-engine/src/bridge.rs` -- switch `init_planner`/`submit_*` to `StreamPlannerHandle` + `build_move` (facet `bezier`/`quadratic`); wire flush/dwell/stream-open/caps. Keep `dispatch`/`project` closures.
- [ ] remove host NURBS planner: delete old `planner.rs` path, `GeometryPipeline`/host `CubicSegment` + `trajectory`/`temporal` velocity-engine consumption once no caller remains; keep the `nurbs` crate, MCU eval, reused emit/kinematics.
- [ ] `rust/motion-engine/tests/*` -- migrate the ~50 e2e tests to the new planner; add an end-to-end test for the I/O-matrix invariants. Full `./scripts/ci.sh quick` + 3 MCU target builds green.

**Acceptance Criteria:**
- Given two consecutive collinear `submit_move`s, when the buffer is planned and committed, then the emitted `PieceEntry` stream is position-continuous and velocity-continuous across the move junction (no zero-velocity dwell between them).
- Given a cornered polyline, when planned, then at least one clothoid blend is produced and no full stop is emitted at the corner (speed dips below cruise but stays > 0).
- Given an extruding move, when lowered, then the follower lands on the correct registry axis/`motor_mask`, its displacement is monotone, and the endpoint equals the commanded `de` within tolerance.
- Given a `Flush`/`Dwell`/`Shutdown`, when handled, then the entire look-ahead buffer is committed and decelerates to rest with no stranded move.
- Given a move whose start time is in the past, when received, then the planner raises a clear error and does not pad the start time.
- Given the cutover is complete, when grepping the host build, then no production code references `GeometryPipeline`/host `CubicSegment`, and `./scripts/ci.sh quick` is green.

## Spec Change Log

- **2026-06-19 — streaming is a reimplementation, not a wiring task (discovered in step-03).** The new geometry pipeline is a *batch* planner with no streaming/commit/clock/continuity machinery; all of that lives in `trajectory::streaming::ShaperState` (~1200 lines), built around `CubicSegment` (NURBS) + the `trajectory` crate's SOCP `plan_velocity`. User confirmed **Path A** (full cutover) — NURBS can't solve TOPP in real time, so feeding the old engine is rejected. Two findings make A tractable: (a) the new pipeline is **C1** (only velocity crosses junctions — no C2/`start_d2_override` needed); (b) `lower_profile` already accepts a warm-started `v(s)`. Amendment (non-frozen only): added the **Stage-1 warm-start prerequisite** to `plan_velocity` (rest-to-rest today, velocity.rs:157), restructured Tasks into 3 stages, recorded `ShaperState` *replacement*. Frozen intent unchanged. **KEEP:** the emit-backend reuse boundary (`ShapedSegment`→`enqueue_segment`→pump) and the C1 insight — both load-bearing.

- **2026-06-19 — motion limits read from `[printer]`, applied per-move; `[limit]` sections retired from the live path.** On the bench the stream pipeline ignored the configured accel/jerk: `path_velocity_limits()`/`path_velocity_config()` collapsed every spatial `[limit]` section to a single scalar via `min()`, so the slowest axis (Z) throttled XY, and follower-containing sections were dropped. Per the *"limits are global for XY now"* decision, the source of truth moved to mainline-style `[printer]` keys: `max_velocity`, `max_accel`, a new `max_jerk` (default `2×max_accel`), and `max_z_velocity`/`max_z_accel` (default to global, capped at it — matching mainline cartesian validation). New `config::CartesianLimits` carries them; `submit_move` resolves caps **per move by direction** (`for_move`: pure-XY → gantry caps, Z-bearing → Z caps projected by the Z direction cosine, mainline `limit_speed` rule) and folds in `RuntimeCaps` (M204/`SET_VELOCITY_LIMIT`). `init_planner` gained a `cartesian_limits` arg. `[limit <name>]` sections are still parsed so configs load but are **inert** — never validated (the init `to_temporal_limits()` gate is gone; the per-axis temporal planner is test-only) and never consulted by the stream; the extruder rides the trajectory uncapped. The superseded `path_velocity_limits()`/`path_velocity_config()` remain on `PlannerConfig` for the temporal planner's tests only. **Frontend compat:** Mainsail's `getKinematics` reads `configfile.settings.printer.kinematics` and, once `[printer]` had readable options, resolved a missing key to `'none'` and dropped the toolhead/jog panels; `load_kinematics` now mirrors the authoritative `[kinematics]` type onto `[printer]`'s reported settings (defaulted read — `configfile` records non-None defaults) so the panel stays. Commits 7876d96b4 · 627e2ddb4 · c13ce5ea3 · cdbe43fe1.

## Design Notes

**Reuse boundary (the load-bearing decision).** The emit backend consumes a `ShapedSegment{axes: Vec<ScalarNurbs<f64>>, followers, motor_mask}` and below it everything is validated and kept. The new adapter's only job is to produce that `ShapedSegment` from the new plan — so kinematics lane-mixing, clock projection, Bézier flatten, and the pump/MCU ring are reused **unchanged**. Position-vs-time along a limit-ridden clothoid is not exactly piecewise-cubic; the adapter samples (as `lower_profile` does) and fits per-axis piecewise-cubic time curves to a stated tolerance (subdivide to bound error). `flatten_bezier_pieces` already asserts ≤ cubic — respect it.

**Streaming model (C1, warm-start).** The new pipeline is **C1** — only velocity is shared across junctions (`plan_velocity` carries `v[k]`, no acceleration state), so the dispatched-boundary continuity datum is a single **entry velocity**; no C2/accel pinning (the old engine's `start_d2_override`) is needed. Maintain an uncommitted `VecDeque<Move>`. On each arrival, re-plan the whole uncommitted window warm-started from the dispatched exit velocity (terminal `v=0` = worst-case future). Fast TOPP makes full re-solve viable, so the bounded-freeze-index neutrality machinery is *not* ported. Commit/emit on the existing real-time cadence (`LEAD`/`sync_instant`/idle-timeout); already-emitted pieces are never rewritten because the next replan is *pinned* to the dispatched velocity. **Commit at move boundaries** — to bound commit latency without mid-clothoid splitting, long moves are pre-faceted at the bridge into bounded-duration collinear sub-moves (`fit_chain` treats them as collinear → no spurious stop). If a pinned entry velocity cannot brake to rest within the buffer, fail loudly (`OverCommitted`) rather than emit an unstoppable trajectory. The lazy-lock commit refinement (à la Klipper `flush_lookahead`) is deferred.

**Curves (G5/quadratic).** The new frontend models only Line/Arc/Clothoid. Recommended V1: flatten `submit_bezier`/`submit_quadratic` control points to short line facets (de Casteljau) and feed them as `Move`s — `fit_chain` re-smooths corners, so the print stays continuous. Alternative (if rejected): fail loudly as unsupported. Either way, no host NURBS curve survives.

**Deferred (out of scope, into `deferred-work.md`):** input shaping; full lazy-lock streaming; native Arc/Bézier move types end-to-end (vs. faceting); throughput-regression gate.

## Verification

**Commands:**
- `cargo nextest run -p motion-engine` -- expected: updated e2e suite green (continuity, follower, flush-to-rest).
- `cargo nextest run -p geometry` -- expected: spec-10 integration suite still green.
- `cargo build -p motion-engine` then `grep -rn "GeometryPipeline\|CubicSegment" rust/*/src` -- expected: no production hits remain.
- `./scripts/ci.sh quick` -- expected: fully green (clippy `-D warnings`, fmt, rust tests) before PR.

**Manual checks:**
- Build the cdylib (`make -f Makefile.rust motion-engine`) and run representative slicer G-code through the offline simulator / a bench; confirm it prints and corners are blended (no per-corner stop) — manual, outside CI.
