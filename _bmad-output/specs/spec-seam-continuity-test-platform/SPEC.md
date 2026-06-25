---
id: SPEC-seam-continuity-test-platform
companions:
  - ../../implementation-artifacts/investigations/junction-position-discontinuity-investigation.md
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only.

# Streaming-planner seam-continuity test platform

## Why

A pain to solve, against a non-negotiable mandate (throughput). Streaming commits in the motion planner can break trajectory continuity at a commit seam: the committed segment's endpoint and the re-fit continuation's start diverge — just root-caused as a cadence-dependent corner re-fit (Y, |Δ|≈0.1545 mm on the Voron cube) that trips the fail-loud `panic!` in `check_junction_position_continuity` and aborts the print. Today this whole class surfaces only as intermittent bench crashes; our only offline tool is one ad-hoc example (`examples/repro_junction`) at a single fixed commit cap. The unavoidable worst case is a **forced commit under ring-empty starvation** — we must commit a possibly-unfinished corner to keep the MCU fed, then replan it — and continuity must still hold there. We need a reusable harness that makes this bug class deterministic, regression-pinnable, and fuzz-discoverable offline, so seam discontinuities are caught in CI instead of on the printer.

## Capabilities

- id: CAP-1
  intent: A developer can drive any gcode file through the real `StreamState` commit/seam/fit path and get a continuity report for every commit boundary.
  success: Running it on `crash_short_cube.gcode` reproduces the known seam (Y, |Δ|≈0.1545 mm) and reports axis, magnitude, time, position, commit index, and source gcode line — deterministically, in seconds, no bench/MCU.

- id: CAP-2
  intent: A developer can run the same gcode under many commit schedules — fixed cap, swept caps, randomized cadence, and forced commits (`commit(force=true)`) injected at arbitrary buffer depths to model ring-empty starvation.
  success: One gcode yields continuity checks across the schedule space; a forced commit at a non-clean seam followed by replan is checked identically to a clean seam; the cube seam is shown to appear only within a characterizable cap range (≤24), matching the bench.

- id: CAP-3
  intent: At each commit seam the harness lowers segments to `PieceEntry` coeffs through the real `enqueue` path (with a deterministic `project` host-time→tick closure) and asserts continuity on the exact quantities the production check compares — C0 position (prev `coeffs[3]` vs next `coeffs[0]`, mandatory), with C1 velocity and curvature/blend-budget invariance as added orders — against a configurable tolerance.
  success: A violation emits a minimal seam descriptor whose fields match the bench panic (axis, prev-end vs next-start position, host time, source lines); a harness pass implies the production `check_junction_position_continuity` would not panic, because it runs the same comparison on the same lowered pieces (no ShapedSegment-only blind spot for lowering-introduced gaps).

- id: CAP-4
  intent: Known seam discontinuities are encoded as deterministic checked-in tests that fail on the buggy planner and pass once fixed.
  success: A committed test reproduces the 0.1545 mm cube seam (red on current HEAD) and goes green when the planner seam fix lands, with no flakiness across repeated runs.

- id: CAP-5
  intent: A developer can fuzz the commit schedule (random caps + forced-commit points) over a curated real-gcode corpus to surface unknown seam discontinuities, with automatic shrinking to a minimal reproducing (gcode + schedule) pair.
  success: Run against the current buggy planner, the fuzzer independently finds a cadence-dependent or forced-commit seam break and emits a minimal repro materially smaller than the full cube (fewer moves and/or a single forced-commit point).

- id: CAP-6
  intent: The platform surfaces the complete set and count of currently-failing seam cases as real, red, failing tests — the honest, visible extent of the bug — and is expected to be largely red on day one.
  success: A run reports every failing seam as a genuine test failure (none skipped, `#[ignore]`d, xfail'd, or baselined to green); the failing count is visible and shrinks as planner fixes land.

## Constraints

- Must drive the real `StreamState` commit/seam/fit code path (the same `commit`, `fit_chain_with_head_restore`, head-trim, `entry_v`/`committed_head_len` machinery the bench runs); a parallel or mock planner would test nothing.
- Must cover the forced-commit-then-replan path (`commit(force=true)` at arbitrary buffer depth): ring-empty starvation makes forced commits unavoidable in production and is where continuity is hardest; testing only clean zero-curvature seams misses the crux.
- Deterministic and offline: every generated case reproduces byte-identically and runs in seconds with no MCU, no real clock, no threads or wall-clock dependence — fuzzing requires seedable determinism and replay.
- Continuity assertions must match the production fail-loud check's semantics and fields, so a harness pass implies the bench will not panic; a divergent metric that passes while the bench fails is worse than nothing.
- Every fuzzer discovery must shrink to a minimal, replayable repro (gcode + commit schedule); an unshrunk find is unactionable.
- Failing seam cases fail loud as real red tests — never skipped, `#[ignore]`d, xfail'd, or baselined to green. A suite kept green by suppressing known failures hides the bug; the red count is the truth we fix against (mirrors the project's fail-loud mandate). This rules out any expected-failure/quarantine mechanism.

## Non-goals

- Fixing the planner seam discontinuity itself — this platform only detects and pins it.
- MCU/hardware-in-the-loop, real EtherCAT transport, or real clock-sync/timing.
- Full print simulation, throughput, or performance benchmarking.
- Testing the pump/dispatch/backpressure subsystems directly; their effect (forced commits under starvation) is modeled via `commit(force=true)`, but the pump itself is out of scope.
- Synthetic/generated random gcode geometry — v1 fuzzes commit schedules over a curated real-gcode corpus; gcode generation is a deferred fast-follow.

## Success signal

The platform's first runs are expected to be **largely red**: it surfaces that seam discontinuities are widespread across commit cadences, forced-commit points, and geometries — not isolated to the one cube seam — as **real failing tests**, instead of hiding them. The red count is the honest measure of the bug; success is that it is visible and shrinks monotonically toward zero as planner fixes land. The intermittent "bench crash from a seam re-fit" stops being a bench-only discovery; it becomes a red board we watch turn green.

## Assumptions

- C0 position is the mandatory gate (it is what fail-loud panics); C1 velocity and curvature-budget invariance are included, with curvature-invariance allowed to be best-effort if not cheaply observable from the chosen seam representation.
