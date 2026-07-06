---
title: 'Pressure Advance on the live (stream/geometry/lowering) motion pipeline'
type: 'feature'
created: '2026-06-22'
baseline_commit: '94f5a39c60e8621b7486cff2f22ed6d909144768'
status: 'done'
context:
  - '{project-root}/_bmad-output/specs/spec-pressure-advance-port/SPEC.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-11-pipeline-production-cutover.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The production motion path is `StreamPlanner → StreamState::commit → geometry::plan_velocity_warm_start → lowering::lower_move` (wired from the PyO3 bridge at `bridge.rs:3207`). It applies **no** Pressure Advance — followers are emitted as raw position curves — and it consumes **no** post-processor at all (the rail does not exist on this path; input shaping is deferred by spec-motion-11). `[post_processor]` config is parsed and stored in the bridge but never reaches the running planner, so `SET_POST_PROCESSOR` silently no-ops. Print-quality parity needs linear PA here.

**Approach:** Add PA as an **emit-time** correction `e(t) += k·ė(t)` on follower axes inside `lower_move`, behind a minimal post-processor application seam so future models (tanh) and input shaping ride the same rail. Deliver post-processor config into `StreamState` and route live retune to the planner thread. Add the missing **extrude-only** velocity/accel limit for pure-E (virtual-path) moves in the geometry velocity planner. The XYZ velocity planner is already follower-blind, so PA and extruder limits are purely additive and cannot cost XYZ trajectory time.

## Boundaries & Constraints

**Always:**
- PA is applied only at emit (`lower_move`), after velocity planning. It must never feed `plan_velocity_warm_start`; the follower-blind XYZ planner stays follower-blind.
- Extrude-only limits apply **only** to virtual-path (pure-E) moves, mainline-style: `limit_speed(max_e_velocity·inv_extrude_r, max_e_accel·inv_extrude_r)`. Co-move XYZ velocity/accel must be provably unaffected.
- Reuse mainline field names verbatim: `max_extrude_only_velocity`, `max_extrude_only_accel`.
- On co-moves the extruder is a pure follower with no limit of its own (Klipper-style); PA may push it past its kinematic limits and that is accepted.
- Post-processors apply to an axis in `[axis] post_processors:` declaration order, type-agnostic — PA and (future) input shaping are not special-cased relative to each other.
- Fail loudly on unexpected planner state per project rule; do not pad or recover.

**Ask First:**
- If wiring the post-processor rail forces a change to the `ShapedSegment` contract or the MCU emit backend (it should not — PA stays inside `lower_move`'s per-axis curve construction).
- If achieving live retune requires re-emitting already-committed output (forbidden) rather than applying to new plans only.

**Never:**
- Delete or modify the dead old pipeline (`trajectory::beta`/`streaming`/`emit_shaped`/`post_processor`, `temporal::multi`/`topp`, old `planner.rs::Planner`). That removal is spec-motion-11's "delete the host NURBS planner path", not this spec.
- Ship tanh PA, `smooth_time`, `max_extrude_only_distance`, or `instantaneous_corner_velocity` (CAP-4 provides the seam only).
- Add any plan-time/solver-time extruder constraint, or revive the SOCP solver.
- Constrain XYZ velocity/accel by any extruder limit.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Co-move, PA k>0 | XYZ+E move, follower on E | Emitted E curve = base + k·ė; XYZ axes byte-identical to k=0 | N/A |
| Co-move, PA k=0 | XYZ+E move | E curve identical to no-PA (no-op gain) | N/A |
| Pure-E move | retraction, virtual_path_mm set | Velocity/accel capped by `max_extrude_only_*·inv_extrude_r` | N/A |
| Co-move + pure-E limits set | extrude-only fields configured | XYZ caps unchanged vs. unset | N/A |
| Live `SET_POST_PROCESSOR NAME=pa K=…` | command mid-stream | New k applies to plans committed after the command; held/committed output unchanged | reject if k non-finite/negative |
| Unknown post-processor name on axis | config references missing name | Reject at config build (existing bridge check) | raise config error |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/lowering.rs:180` `lower_move` -- live emit; per-axis cubic-Hermite from sampled `(pos, vel)`. PA injection site for follower axes (`axis_state` at `:128`, follower loop `:194`, `:209`).
- `rust/motion-engine/src/stream.rs:13` `StreamConfig` / `:146` `commit` -- carries config into the planner thread and drives fit→plan→lower. Where per-axis PA gains + extrude-only config must be delivered and applied.
- `rust/motion-engine/src/stream_planner.rs:56` `spawn` / `:319` `run_loop` -- live planner thread; add a retune `StreamMsg` so `update_post_processor` reaches it.
- `rust/motion-engine/src/bridge.rs:3634` `update_post_processor` -- currently mutates stored config only; must notify the live `StreamPlannerHandle`. `:2479-2526` builds/stores `PostProcessorSet` (config parse already exists).
- `rust/geometry/src/velocity.rs:107` `plan_velocity_warm_start` / virtual branch `:155-164` -- extrude-only limit application site for pure-E moves. `VelocityConfig` at `:17`.
- `rust/geometry/src/frontend.rs:10` `VelocityLimits` (`max_velocity_mm_s`, `accel_mm_s2`) / `:185` `extruder_follower` -- extruder limit fields home; follower ratio source.
- `rust/geometry/src/segment.rs:33` `FollowerDemand` (`axis_index`, `ratio`) -- live follower model; PA gain is keyed per axis via the post-processor config, not added here.
- `klippy/extras/post_processor.py`, `klippy/motion.py:629-648,737-755` -- config section + `cmd_SET_POST_PROCESSOR` (already present; verify they target the live engine path).
- Reference only (dead pipeline — do not edit): `rust/trajectory/src/post_processor.rs:201` `apply_derivative_gain` is the correctness exemplar for the gain math.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/velocity.rs` -- added `max_extrude_only_velocity_mm_s`/`max_extrude_only_accel_mm_s2` (INFINITY = unlimited) to `VelocityConfig`; virtual-path branch caps v/accel by them (`inv_extrude_r`=1 since `virtual_path_mm` is the extruder coordinate); spatial branch untouched; rejects non-positive/NaN. -- CAP-3.
- [x] `rust/motion-engine/src/lowering.rs` -- `lower_move` takes `&[CompiledChain]`; follower axes get `pos += k·ė`, `slope += k·ë` (`ë = ratio·phase.accel`). Fit grid refined from **base** curves (`with_pa=false`) so PA never perturbs XYZ pieces; PA applied only at final piece build. Seam: new model = new `PostProcessorType` variant + `type` string + correction fn. -- CAP-1, CAP-4.
- [x] `rust/motion-engine/src/stream.rs` -- `StreamState` carries an `AxisChainSet` (runtime-swappable via `set_axis_chains`); `commit` passes `&axis_chains.chains` to `lower_move`. Extrude-only limits ride `StreamConfig.velocity`. -- CAP-1, CAP-3.
- [x] `rust/motion-engine/src/stream_planner.rs` + `bridge.rs` -- added `StreamMsg::SetAxisChains` + `StreamPlannerHandle::update_axis_chains`; `bridge::update_post_processor` recompiles the `AxisChainSet` and pushes it to the live handle (new plans only). `bridge::init_planner` gained `max_extrude_only_velocity/accel` params; spawn compiles the `AxisChainSet`. -- CAP-2, CAP-3.
- [x] `klippy/kinematics/extruder.py` + `klippy/motion.py` -- (option B) un-rejected `max_extrude_only_velocity/accel` in `[extruder]`, read them, and forward the primary extruder's values to `init_planner`. -- CAP-3.
- [x] tests -- `lowering/tests.rs` (PA shift + XYZ byte-identical, k=0≡no-PP); `stream_planner/tests.rs` (live retune applies to plans after the swap, held output unmutated); `geometry/src/velocity/tests.rs` (extrude-only v/accel caps pure-E, leaves spatial moves untouched); `test_extruder_split.py` (fields read). -- covers I/O matrix.
- [x] negative tests -- `post_processor/tests.rs` + `config/tests.rs`: non-finite/negative `k` rejected at runtime `set_param` and config build; `geometry` rejects non-positive extrude-only limits. -- fail-loudly obligation.

**Acceptance Criteria:**
- Given a co-extruding move planned with PA k>0, when emitted, then the extruder axis equals base + k·ė while every XYZ axis is byte-for-byte identical to the k=0 emit.
- Given a pure-E retraction with extrude-only fields set, when planned, then its velocity/accel respect `max_extrude_only_*·inv_extrude_r`, and a co-move's XYZ caps are unchanged from the unset case.
- Given a running stream, when `SET_POST_PROCESSOR NAME=pa K=v` is issued, then only plans committed after the command reflect v; already-committed output is unchanged.
- Given a config that adds a tanh model, when integrated, then only a new `PostProcessorType` variant + `type` string + correction fn are needed — no change to velocity planning, limits, or stream wiring.

## Spec Change Log

### 2026-06-22 — Implementation discoveries (3 spec/reality conflicts)

1. **Wrong pipeline (the headline fallacy).** Source SPEC targeted the dead `trajectory`/`temporal` path; the live path is `StreamPlanner → StreamState::commit → geometry::plan_velocity_warm_start → lowering::lower_move`. Spec rewritten against it (see Design Notes). Known-bad avoided: deleting "dead" `topp` code that is live, and "confirming wiring" that doesn't exist.
2. **Decoupling is free, not a deletion.** `plan_velocity_warm_start` is follower-blind, so emit-time PA cannot cost XYZ time by construction. The "delete dead coupling" task became a non-task; dead-pipeline removal stays owned by spec-motion-11.
3. **CAP-3 field-name conflict (resolved by user → option B).** The fork had deprecated `[extruder] max_extrude_only_velocity/accel` in favor of `[limit] axes: e` (`extruder.py` rejected them). User chose **B**: re-enable the verbatim mainline fields. Implemented by un-rejecting + reading them in `extruder.py` and forwarding via `motion.py` → `init_planner`. v1 sources the **primary** extruder's pair into a single planner-wide extrude-only cap (multi-extruder per-axis limits are out of scope).

KEEP: emit-time-only PA with a base-curve-refined fit grid is what makes XYZ byte-identical — do not let PA influence `span_residual`.

## Design Notes

**The fallacy this spec corrects (load-bearing — keep).** The source SPEC and its code-map (`_bmad-output/specs/spec-pressure-advance-port/`) were written against the **wrong pipeline**. They assumed `temporal::topp` was a dead removed solver and PA already lived on the new path via `apply_derivative_gain` + the `emit_shaped` follower branch, leaving only "wiring confirmation + delete dead coupling." Investigation showed otherwise:
- The live PyO3 path is `StreamPlanner`/`stream`/`geometry`/`lowering`. The `trajectory::beta`/`temporal::multi`/`topp` SOCP path is **not wired to the bridge** — it is the old pipeline spec-motion-11 will delete. `topp` was never "the removed TOPP"; it was the live SOCP per-chain solver of the *old* path. None of the cited PA code runs in production.
- Therefore PA on the live path is **additive new work**, not re-lighting. The dreaded extruder→XYZ coupling does not exist here: `plan_velocity_warm_start` is follower-blind, so emit-time PA structurally cannot cost XYZ time — the non-negotiable is satisfied by construction, not by deletion.
- The post-processor rail itself is absent on the live path (input shaping deferred by spec-motion-11). PA is its first consumer; build the seam so shaping rides it later.

**Gain math (golden reference).** With follower position `e(t)=ratio·s(t)`, linear PA emits `e(t)+k·ė(t)`. At each Hermite sample inject `pos' = pos + k·v`, `vel' = v + k·a` for the follower axis (mirrors `apply_derivative_gain` `p + k·p'` in the dead pipeline). `smooth_time` is unneeded: the base trajectory is already C²-continuous, so PA injects no velocity discontinuity to smear.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p motion-engine -p geometry` -- expected: new PA/extrude-only tests green.
- `cd rust && cargo nextest run` -- expected: full suite green (old-pipeline tests untouched).
- `./scripts/ci.sh quick` -- expected: green (ruff, rust-test, clippy -D warnings, fmt, watchdog).
- `./scripts/ci.sh py` -- expected: green (touches `klippy/` config surface).

## Suggested Review Order

**PA correction (start here — the design intent)**

- Emit-time PA on follower axes; `with_pa` gates the gain so XYZ axes never see it.
  [`lowering.rs:122`](../../rust/motion-engine/src/lowering.rs#L122)

- `lower_move` takes per-axis `CompiledChain` — the model-dispatch seam (CAP-4).
  [`lowering.rs:194`](../../rust/motion-engine/src/lowering.rs#L194)

- Per-axis gains built from chains; `get().map_or(0.0)` makes empty/short chains a no-op.
  [`lowering.rs:217`](../../rust/motion-engine/src/lowering.rs#L217)

**XYZ byte-identical guarantee**

- Fit grid refined from BASE curves (`with_pa=false`) so PA can't perturb XYZ pieces.
  [`lowering-tests:214`](../../rust/motion-engine/src/lowering/tests.rs#L214)

**Extrude-only limit (CAP-3)**

- Caps v/accel only in the virtual-path (pure-E) arm; spatial untouched.
  [`velocity.rs:172`](../../rust/geometry/src/velocity.rs#L172)

- Config fields, INFINITY = unlimited.
  [`velocity.rs:21`](../../rust/geometry/src/velocity.rs#L21)

**Live retune (CAP-2)**

- `StreamState` holds a runtime-swappable `AxisChainSet`.
  [`stream.rs:62`](../../rust/motion-engine/src/stream.rs#L62)

- `commit` feeds current chains to `lower_move`.
  [`stream.rs:179`](../../rust/motion-engine/src/stream.rs#L179)

- Handle method + message apply the swap between commits (new plans only).
  [`stream_planner.rs:151`](../../rust/motion-engine/src/stream_planner.rs#L151)

- Bridge recompiles and pushes chains to the live planner.
  [`bridge.rs:3664`](../../rust/motion-engine/src/bridge.rs#L3664)

**Fail-loud validation**

- `k` routed through validating `set_param` at config build.
  [`config.rs:165`](../../rust/motion-engine/src/config.rs#L165)

- `BadParam`: reject non-finite/negative `k` at runtime.
  [`post_processor.rs:46`](../../rust/trajectory/src/post_processor.rs#L46)

- `init_planner` validates extrude-only params; spawn compiles the chain set.
  [`bridge.rs:2527`](../../rust/motion-engine/src/bridge.rs#L2527)

**Python config surface (CAP-3 option B)**

- `[extruder]` reads verbatim mainline field names.
  [`extruder.py:29`](../../klippy/kinematics/extruder.py#L29)

- Forwarded to the planner via `init_planner`.
  [`motion.py:867`](../../klippy/motion.py#L867)

**Tests (peripheral)**

- Live retune applies after the swap; held output unmutated.
  [`stream_planner-tests:233`](../../rust/motion-engine/src/stream_planner/tests.rs#L233)

- Extrude-only caps pure-E; negative `k` rejected.
  [`velocity-tests:330`](../../rust/geometry/src/velocity/tests.rs#L330) · [`post_processor-tests:65`](../../rust/trajectory/src/post_processor/tests.rs#L65)
