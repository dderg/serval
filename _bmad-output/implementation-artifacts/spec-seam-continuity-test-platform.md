---
title: 'Streaming-planner seam-continuity test platform'
type: 'feature'
created: '2026-06-24'
status: 'done'
baseline_commit: 'd8258310cb123d25084aaed628a4ba260710d74c'
context:
  - '{project-root}/_bmad-output/specs/spec-seam-continuity-test-platform/SPEC.md'
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/junction-position-discontinuity-investigation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Streaming commits in the motion planner can break C0 trajectory continuity at a commit seam — a cadence-dependent corner re-fit (Y, |Δ|≈0.1545 mm on the Voron cube at commit cap≤24) re-derives the continuation's start position off the committed endpoint, tripping the fail-loud `panic!` in `check_junction_position_continuity` and aborting the print. Today this surfaces only as intermittent bench crashes; the only offline tool is one ad-hoc example at a single fixed cap.

**Approach:** Build a reusable, deterministic, offline harness that drives any gcode through the **real** `StreamState` commit/seam/fit path under arbitrary commit schedules (fixed/swept/random caps + forced commits), lowers each committed segment to `PieceEntry` coeffs through the **real** `enqueue` path, and asserts continuity using the **same comparison** the production check runs — so a harness pass implies the bench will not panic. Ship a checked-in regression pinning the cube seam plus a schedule fuzzer with shrinking. Failing seam cases stay **real and red**; this platform only detects and pins the bug, it does not fix the planner.

## Boundaries & Constraints

**Always:**
- Drive the real `StreamState` (`push` / `commit(force)` / `buffered`) and the real `enqueue_segment` lowering — never a mock or parallel planner.
- Reuse the production C0 comparison: factor the `check_junction_position_continuity` compare+bookkeeping into one shared unit that both `pump.rs` and the harness call. No second, divergent metric.
- Deterministic & offline: seedable, byte-identical replay, runs in seconds, no MCU, no real clock, no threads. Use a fixed-frequency `project` closure mirroring production tick-projection semantics (truncation).
- Failing seam cases are **real red tests** — never `#[ignore]`d, `xfail`'d, skipped, or baselined to green.
- Every fuzzer discovery shrinks to a minimal, replayable `(gcode + commit schedule)` repro, persisted for deterministic re-run.

**Ask First:**
- Any change to production planner geometry/seam logic to make a test pass — fixing the seam bug is a separate spec; HALT.
- Introducing any expected-failure / quarantine / suppression mechanism — forbidden by the contract; HALT.

**Never:**
- Fix the planner seam discontinuity itself (detect & pin only).
- MCU/hardware-in-the-loop, real EtherCAT, or real clock-sync/timing.
- Full-print simulation, throughput, or performance benchmarking.
- Test the pump/dispatch/backpressure subsystems directly (their effect is modeled via `commit(force=true)`).
- Synthetic/generated random gcode geometry — v1 fuzzes commit schedules over a curated real-gcode corpus only.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Known cube seam | `crash_short_cube.gcode`, cap≤24 | SeamReport: 1 FATAL boundary, axis=Y, |Δ|≈0.15450 mm, host t≈5.208962, prev/next pos, commit index, source line | N/A (this is the asserted red case) |
| Clean cadence | same gcode, cap≥32 | 0 FATAL boundaries, worst |Δ|=0.0 | N/A |
| Forced commit at non-clean seam | `commit(force=true)` at arbitrary buffer depth, then replan | seam checked identically to a clean seam; descriptor emitted with the same fields if continuity breaks | N/A |
| Fuzz discovery | random caps + forced-commit points over corpus | a minimal `(gcode, schedule)` repro emitted and persisted | shrink failures persist to `proptest-regressions/` |
| Harness pass | any case with 0 FATAL | implies production `check_junction_position_continuity` would not panic (same shared compare) | N/A |
| Empty/late commit | `commit` returns empty `Vec<ShapedSegment>` | no boundary recorded, no false positive | N/A |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/stream.rs` -- `StreamState` (`push`, `commit(force: bool) -> Result<Vec<ShapedSegment>>`, `buffered`, `new`); the commit/seam/fit path under test. `commit(false)`=opportunistic to latest clean seam; `commit(true)`=flush-to-rest (models ring-empty starvation). No cap param — cadence is caller-driven off `buffered()`.
- `rust/motion-engine/src/enqueue.rs` -- `enqueue_segment<P: Fn(u32,f64)->u64>(seg, mcu_configs, t0, fresh_stream, host_now, lead_secs, project, max_piece_secs)`; lowers a `ShapedSegment` to `PieceEntry` coeffs. `project` is the host-secs→MCU-tick closure.
- `rust/motion-engine/src/pump.rs` -- `check_junction_position_continuity` (~:349): compares next `coeffs[0]` (P0) vs stored `JunctionEnd.end_pos` (prev `coeffs[3]`, P3), gated on `motor_mask == 0`, FATAL ≥ `JUNCTION_POSITION_FATAL_MM`=0.1. Fields: axis, mcu, prev_end, next_start, jump, host times, source lines. **Factor the compare+`junction_ends` bookkeeping into a shared unit here.**
- `rust/runtime/src/piece_ring.rs` -- `PieceEntry { start_time: u64, coeffs: [f32;4], duration: f32, motor_mask: u8 }`; `coeffs[0]`=P0 start, `coeffs[3]`=P3 end; relativized when `motor_mask != 0`.
- `rust/motion-engine/examples/repro_junction.rs` -- existing ad-hoc repro (gcode parse, `Pos`, `build_move`, cap loop) to generalize into the shared harness.
- `rust/motion-engine/Cargo.toml` -- `proptest` already a dev-dep; `test-support` feature exists; `FileFailurePersistence` → `proptest-regressions/` is the established shrinking idiom.

## Tasks & Acceptance

**Execution:**
- [x] `rust/motion-engine/src/pump.rs` -- extract the C0 seam comparison + `junction_ends` bookkeeping into a reusable `pub(crate)` unit; have the panic path call it; expose it for the harness to drive. Rationale: the trustworthiness invariant — one comparison, no divergence.
- [x] `rust/motion-engine/src/seam_harness.rs` (+ `seam_harness/tests.rs`), `test-support`-gated -- the reusable harness: `parse_gcode_to_moves`, a `CommitSchedule` (fixed cap / swept caps / randomized cadence / forced-commit injection at a given buffer depth), a driver that pushes moves and commits per schedule, lowers each committed `ShapedSegment` via real `enqueue_segment` with a deterministic `project` closure + offline `McuAxisConfig`, runs the shared checker over the lowered piece stream, and returns a `SeamReport` of `SeamDescriptor { axis, delta_mm, host_t, prev_pos, next_pos, commit_index, source_line }`. C0 mandatory; add C1 velocity and curvature/blend-budget invariance as best-effort orders.
- [x] `rust/motion-engine/src/lib.rs` -- register `seam_harness` under the `test-support` gate.
- [x] `rust/motion-engine/tests/gcode/crash_short_cube.gcode` -- check in the 501-move fixture (from session scratchpad) that reproduces the seam.
- [x] `rust/motion-engine/tests/seam_continuity.rs` -- deterministic regression: cube at cap≤24 → 1 FATAL Y boundary |Δ|≈0.15450 mm (asserts CAP-1 report fields); cap≥32 → 0 FATAL; a forced-commit case checked identically. Real red tests on current HEAD, no `#[ignore]`/xfail/baseline.
- [x] `rust/motion-engine/tests/seam_schedule_fuzz.rs` -- proptest fuzzer over the corpus: random caps + forced-commit points; `FileFailurePersistence` → `proptest-regressions/` for shrink+replay; independently finds a cadence/forced-commit seam on buggy HEAD and shrinks to a minimal `(gcode, schedule)` smaller than the full cube.
- [x] `rust/motion-engine/examples/repro_junction.rs` -- refactor to call the shared harness (drop the duplicated gcode/`Pos`/`build_move`/cap-loop) so example and tests share one driver.

**Acceptance Criteria:**
- Given the same `(gcode, schedule, seed)`, when the harness runs repeatedly, then the `SeamReport` is byte-identical, completes in seconds, and uses no MCU/real-clock/threads.
- Given current HEAD, when `cargo nextest run -p motion-engine -E 'test(seam)'` runs, then the cube regression FAILS as a real red test (Y |Δ|≈0.15450 mm) with no case `#[ignore]`d/xfail'd/baselined, and the failing count is visible.
- Given a harness run reporting 0 FATAL boundaries, when the same pieces reach production, then `check_junction_position_continuity` would not panic, because both invoke the same shared comparison.
- Given the schedule fuzzer on buggy HEAD, when it discovers a seam, then it emits and persists a minimal replayable `(gcode, schedule)` repro.

## Design Notes

- **Why this makes the suite red (intended):** CAP-6 + the no-suppression constraint mean `cargo nextest -p motion-engine` (inside `ci.sh quick`) stays **red** until the planner seam fix lands. This is deliberate per the project fail-loud mandate and the 2026-06-24 "embrace the red" decision — the red count is the honest measure of the bug, shrinking monotonically as fixes land. It is the one knowing exception to the green-gate merge rule on this branch.
- **Cap semantics:** `commit()` has no cap argument; "cap" = the `buffered()` threshold at which the driver calls `commit(false)`. Forced commits = `commit(true)` at a chosen buffer depth, modeling ring-empty starvation.
- **Deterministic project:** mirror production with a fixed-frequency truncating closure, e.g. `|_mcu, hs| (hs * FREQ_HZ).trunc() as u64`; build minimal offline `McuAxisConfig`s so the lowering path is identical to production minus the live clock.
- **Piece-level, not ShapedSegment-only:** assert on the lowered `coeffs[3]`→`coeffs[0]` comparison (resolved open question #1) so there is no ShapedSegment-only blind spot for lowering-introduced gaps.

## Verification

**Commands:**
- `cargo nextest run -p motion-engine -E 'test(seam)'` -- expected: cube regression FAILS red on current HEAD with Y |Δ|≈0.15450 mm; deterministic across repeats; clean-cadence (cap≥32) case passes.
- `cargo run --release -p motion-engine --example repro_junction -- rust/motion-engine/tests/gcode/crash_short_cube.gcode --cap 8` -- expected: still reports `FATAL 1=Y |Δ|≈0.15450mm` via the shared harness.
- `cargo nextest run -p motion-engine && ./scripts/ci.sh rust-clippy && cargo fmt --all --check` -- expected: clippy clean, fmt clean; only the seam regressions are red (by design).

## Suggested Review Order

**Trustworthiness invariant (the shared comparison)**

- Entry point — the C0 seam value + fatal predicate both pump and harness read.
  [`pump.rs:355`](../../rust/motion-engine/src/pump.rs#L355)

- The one gate + bookkeeping + comparison unit, extracted for reuse.
  [`pump.rs:398`](../../rust/motion-engine/src/pump.rs#L398)

- Production panic path now consumes the shared seam (no parallel metric).
  [`pump.rs:437`](../../rust/motion-engine/src/pump.rs#L437)

- `run_pump` drives the same `observe` — behavior-preserving refactor.
  [`pump.rs:566`](../../rust/motion-engine/src/pump.rs#L566)

**The harness (the platform)**

- The driver: real `StreamState` commit/seam path under a schedule.
  [`seam_harness.rs:356`](../../rust/motion-engine/src/seam_harness.rs#L356)

- Lowers each segment via the real `enqueue` path, checks via shared `observe`.
  [`seam_harness.rs:300`](../../rust/motion-engine/src/seam_harness.rs#L300)

- Schedule model: fixed/varying caps + forced-commit injection (starvation).
  [`seam_harness.rs:51`](../../rust/motion-engine/src/seam_harness.rs#L51)

- Report shape — C0 `delta_mm` is the production quantity; C1 best-effort.
  [`seam_harness.rs:96`](../../rust/motion-engine/src/seam_harness.rs#L96)

- gcode → planner moves, mirroring the bench frontend.
  [`seam_harness.rs:211`](../../rust/motion-engine/src/seam_harness.rs#L211)

**Tests, fuzzer, wiring (peripherals)**

- Checked-in red regression: cube seam at cap≤24, clean at cap≥32.
  [`seam_continuity.rs:52`](../../rust/motion-engine/tests/seam_continuity.rs#L52)

- Forced-commit-then-replan continuity case.
  [`seam_continuity.rs:82`](../../rust/motion-engine/tests/seam_continuity.rs#L82)

- proptest schedule fuzzer with shrinking → persisted regression.
  [`seam_schedule_fuzz.rs:36`](../../rust/motion-engine/tests/seam_schedule_fuzz.rs#L36)

- Harness module registered under the `test-support` gate.
  [`lib.rs:46`](../../rust/motion-engine/src/lib.rs#L46)
