# Investigation: SCV "does not generate a clothoid" + crash at SCV=50

## Hand-off Brief

1. **What happened.** The user reports that `square_corner_velocity` (SCV) appears not to produce a clothoid — corners look like mainline's instant corner — and that raising SCV "crashed at 50." Evidence shows the planner **does** generate a per-corner biclothoid blend driven by SCV, but the blend size scales with `scv²/accel`, so at the default SCV (5 mm/s) it is micron-scale and visually indistinguishable from a sharp corner (Confirmed).
2. **Where the case stands.** The "no clothoid / instant corner" premise is **refined, largely refuted**: clothoids are generated; they are just sub-visible at low SCV (Confirmed). The "crash at 50" root cause is **Hypothesized** — most likely a velocity-planner divergence/over-commit at the high cornering speeds a large blend permits — and is blocked on the actual error text.
3. **What's needed next.** Re-run the SCV=50 case and capture the exact failure (viz Python traceback / `PyRuntimeError` string, or on-bench `StreamError` via the `query-logs` skill). That one artifact discriminates between the three crash hypotheses below.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-19                                                                 |
| Status           | Active — premise resolved, crash hypothesized (data gap)                   |
| System           | Branch `curvature-profile`; Rust `geometry` + `motion-engine` crates; PyO3 viz + streaming planner |
| Evidence sources | Source code (geometry/motion-engine), git log, graphify graph             |

## Problem Statement

User: "it seems to not use SCV to generate a clothoid, but instead does the same thing that mainline does, and just does an instant corner at that velocity, or maybe my scv value is too low to see, but when I tried bumping it to a higher value it just crashed at 50."

Two distinct claims:
- **(A)** SCV does not generate a clothoid; corners behave like mainline (instant corner).
- **(B)** Raising SCV crashes "at 50."

## Evidence Inventory

| Source                                   | Status    | Notes                                                                     |
| ---------------------------------------- | --------- | ------------------------------------------------------------------------- |
| `rust/geometry/src/fitter.rs`            | Available | Per-junction biclothoid blending; `junction_deviation = scv²(√2−1)/accel` |
| `rust/geometry/src/fitter/biclothoid.rs` | Available | Blend `trim = min(trim_ref·delta/deviation_ref, budget)`; `delta ∝ scv²`  |
| `rust/geometry/src/fitter/chain.rs`      | Available | `[arc_fit]` faceted-run reconstruction (separate from per-corner blend)   |
| `rust/geometry/src/velocity.rs`          | Available | Forward/back velocity pass; unblended corner ⇒ full stop; `Diverged`/`OverCommitted` error paths |
| `rust/motion-engine/src/viz.rs`          | Available | `pipeline_snapshot` → `fit_chain` + `plan_velocity`; errors → `PyRuntimeError` |
| `rust/motion-engine/src/stream.rs`       | Available | On-printer path uses the **same** `fit_chain` + `plan_velocity_warm_start` |
| Actual SCV=50 crash output               | **Missing** | The single decisive artifact — see Missing Evidence                      |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Capture exact SCV=50 error (viz traceback or bench `StreamError`) | High | Open | Discriminates crash hypotheses H-C1/H-C2/H-C3 |
| 2 | Inspect `disk::reach_v` / `disk::limit_speed` / `sample_profile` for divergence at high reachable velocity | High | Open | `velocity/disk.rs` — source of `VelocityError::Diverged` |
| 3 | Check blend over-consumption when two corners share one short move (trim_A + trim_B > middle length) | Medium | Open | Possible overlapping/degenerate geometry feeding the velocity validator |
| 4 | Confirm whether temporal SOCP planner is wired into streaming at all | Low | Open | Streaming uses geometry velocity planner, not `temporal` — relevant to throughput-SOTA constraint |

## Confirmed Findings

### Finding 1: A per-corner biclothoid blend is generated, and it IS driven by SCV

**Evidence:** `rust/geometry/src/fitter.rs:294-341` (`classify_junction`), `:419-422` (`junction_deviation`).

**Detail:** Every line→line junction is classified; the blend half-deviation budget is
`delta = junction_deviation(limits) = scv²·(√2−1)/accel` (`fitter.rs:419-422`). A non-zero `delta` with sufficient budget yields `JunctionPlan::Blend(biclothoid)`. SCV is a direct multiplicative input to the corner geometry. The premise "it does not use SCV to generate a clothoid" is **refuted** at the code level.

### Finding 2: Blend size scales with `scv²/accel`, so at default SCV it is micron-scale

**Evidence:** `rust/geometry/src/fitter/biclothoid.rs:32-33` — `trim = (trim_ref·delta/deviation_ref).min(budget)`, with `delta ∝ scv²`.

**Detail:** For SCV=5 mm/s, accel=3000 mm/s²: `delta = 25·0.414/3000 ≈ 3.45 µm`. The resulting blend trim is sub-10µm — geometrically invisible in a path plot, so the corner *looks* like a sharp/instant corner. To make the blend ~1 mm wide at accel=3000 requires SCV ≈ 80–85 mm/s. This explains both "maybe my scv value is too low to see" (correct) and why the user had to push SCV high to see any rounding. The "instant corner" appearance at default SCV is **expected behavior, not a defect**.

### Finding 3: The fitter has two independent clothoid mechanisms; `[arc_fit]` is the faceted-arc path and is OFF by default

**Evidence:** `rust/geometry/src/fitter.rs:184-269` (`fit_chain`) + `rust/geometry/src/fitter/chain.rs:32-62` (`detect_runs`); `klippy/arc_fit_config.py:6-7` (section absent ⇒ `None`); commit `3cfa24abc` ("opt-in `[arc_fit]`, off by default").

**Detail:** (a) **Per-corner biclothoid blend** — always active in `fit_chain` via `classify_junction`, driven by SCV (Finding 1). (b) **`[arc_fit]` faceted-run reconstruction** — `chain::detect_runs` returns empty unless `arc_fit` is `Some` (`chain.rs:36-38`); it fits slicer-emitted polyline arcs (≥`min_run_junctions` short facets) into up-clothoid + arc + down-clothoid, sized by tolerance, not SCV. A single sharp corner between two long moves is handled only by mechanism (a).

### Finding 4: An UNblended corner is a full STOP, not a mainline-style nonzero SCV junction velocity

**Evidence:** `rust/geometry/src/velocity.rs:124-130` (stop set = unblended, `reason != Collinear`), `:193` (`report.stops += 1`, `v[k]` left at 0); `rust/temporal/src/multi/junction.rs:16-24` ("a non-collinear junction is exactly a chain boundary / full stop").

**Detail:** Where mainline carries a nonzero junction velocity through an unrounded corner, this rewrite brings any corner the blender did not round (NoBudget/ZeroDeviation/NearReversal) to a **dead stop**. A successfully blended corner is G1-continuous and carries a curvature-limited velocity instead. This is a meaningful behavioral difference from mainline and bears on the "instant corner at that velocity" wording.

### Finding 5: viz and on-printer streaming share the same fitter + velocity planner

**Evidence:** `rust/motion-engine/src/viz.rs:29,33` vs `rust/motion-engine/src/stream.rs:168-169`.

**Detail:** Both call `fit_chain(...)` then `plan_velocity[_warm_start](...)` from the `geometry` crate. The `temporal` SOCP planner is **not** invoked in the streaming commit path. Analysis on the viz path therefore transfers directly to print behavior. (Side note for the throughput-SOTA constraint: streaming currently rides the disk/s-curve forward-backward planner, not the TOPP SOCP — see Backlog #4.)

## Deduced Conclusions

### Deduction 1: Higher SCV widens the blend, which *raises* attainable corner speed

**Based on:** Findings 1, 2, and `velocity.rs:198-202` (boundary velocity = curvature-limited `disk::limit_speed(kappa, accel)`).

**Reasoning:** `trim ∝ scv²` ⇒ peak curvature `kappa_peak = trim_ref·theta/trim` *decreases* as SCV rises ⇒ the curvature-limited boundary speed `√(accel/kappa)` *increases*. So SCV behaves as a corner-rounding-deviation budget that buys cornering speed — the intended SOTA semantics, consistent and monotone.

**Conclusion:** The crash at high SCV is unlikely to be in blend *geometry* (which stays finite and well-conditioned as trim grows) and is more likely in the *velocity* stage now asked to carry much higher corner speeds, or in over-consumption of short moves by fat blends.

## Hypothesized Paths

### Hypothesis C1: Velocity-planner divergence/over-commit at high cornering speed (most likely)

**Status:** Open

**Theory:** A fat blend (SCV=50) permits a high curvature-limited corner speed. The disk/s-curve integrator (`velocity/disk.rs`, via `reach_v`/`reach_v_rev`/`sample_profile`) fails to converge → `VelocityError::Diverged`, or in streaming the pinned committed `entry_v` cannot brake to a downstream stop within the look-ahead → `VelocityError::OverCommitted` (`velocity.rs:182-186, 206-227`). In viz this surfaces as `PyRuntimeError("... Diverged/OverCommitted ...")`.

**Supporting indicators:** `viz.rs:34` maps any `plan_velocity` error to `PyRuntimeError`; the error enum has exactly these failure modes; Deduction 1 puts the stress on the velocity stage.

**Would confirm:** SCV=50 error text contains `Diverged` or `OverCommitted` (or the viz `PyRuntimeError` wrapping them).

**Would refute:** Error is a Python-level exception unrelated to `plan_velocity`, or a hard abort with a Rust panic backtrace.

### Hypothesis C2: Fat blends over-consume a short shared move → degenerate/overlapping geometry

**Status:** Open

**Theory:** `budget = 0.5·min(len_in, len_out)` is computed per junction from original line lengths (`fitter.rs:332`). Two adjacent corners on one short middle move can each claim up to half its length; combined trims can exceed the move, producing overlapping/back-tracking blend segments. The velocity validator then rejects them (`validate_segment` → `NonAlphabet`/`Inconsistent`, `velocity.rs:280-305`) or `lower_move` fails (`stream.rs:178`).

**Supporting indicators:** `emit_move` only guards `new_len <= 0` by dropping the *line* (`fitter.rs:361-363`); it does not detect blend overlap across the dropped move.

**Would confirm:** Error is `NonAlphabet`/`Inconsistent`, or repro requires closely-spaced corners (short moves); single isolated corner at SCV=50 does NOT crash.

**Would refute:** A single isolated corner with long approach/exit also crashes at SCV=50.

### Hypothesis C3: Hard panic/abort somewhere on the hot path

**Status:** Open (lowest prior)

**Theory:** An `expect`/index/`unwrap` aborts under SCV=50 inputs.

**Supporting indicators:** Few — constructors (`Clothoid::try_new`, `velocity.rs`) are guarded and return `Err` rather than panic; workspace is `panic=abort` so a panic would show a Rust backtrace, not a clean Python exception.

**Would confirm:** SCV=50 produces a Rust panic message / SIGABRT rather than a `ValueError`/`RuntimeError`.

**Would refute:** Failure is a clean Python exception with a `VelocityError`/`FitError` debug string.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Exact SCV=50 error (string / traceback / exit signal) | Discriminates C1 vs C2 vs C3 in one shot | Re-run the SCV=50 case; capture viz `PyRuntimeError`/`ValueError` text, or on-bench `StreamError` via the `query-logs` skill |
| Whether "50" means SCV=50 mm/s and the exact accel/feedrate/geometry used | Lets us recompute `delta`, `trim`, `budget` and reproduce deterministically | User's `printer.cfg` limits + the G-code/waypoints used for the viz run |
| Whether crash is in viz or on the printer | Scopes C1's `OverCommitted` (streaming-only) vs `Diverged` (both) | User confirmation of which path was exercised |

## Source Code Trace

| Element       | Detail                                                                                          |
| ------------- | ----------------------------------------------------------------------------------------------- |
| Error origin  | Premise (A): `rust/geometry/src/fitter/biclothoid.rs:32-33` (blend size ∝ scv²). Crash (B): unconfirmed — most likely `rust/geometry/src/velocity.rs` (`Diverged`/`OverCommitted`) |
| Trigger       | (A) low SCV ⇒ µm blend ⇒ looks instant. (B) high SCV ⇒ fat blend ⇒ high corner speed / short-move over-consumption |
| Condition     | (A) any corner at default SCV. (B) SCV≈50 with the user's accel/geometry                          |
| Related files | `fitter.rs`, `fitter/biclothoid.rs`, `fitter/chain.rs`, `velocity.rs`, `velocity/disk.rs`, `viz.rs`, `stream.rs`, `klippy/arc_fit_config.py` |

## Conclusion

**Confidence:** Premise (A): **High** (Confirmed). Crash (B): **Low** (Hypothesized, clear data gap).

- **(A) "No clothoid / instant corner" is a misreading of correct behavior.** The planner generates a per-corner biclothoid blend whose width scales as `scv²/accel`; at the default SCV=5 mm/s it is single-digit-µm and invisible. This is distinct from mainline, which does not round the path at all — but visually the µm-scale rounding is indistinguishable from a sharp corner. To make rounding visible, SCV must be raised substantially (≈80 mm/s at accel=3000 for ~1 mm), or `[arc_fit]` enabled for slicer-faceted arcs. A genuine behavioral difference from mainline also exists: an *unblended* corner here is a full stop, not a nonzero junction velocity (Finding 4).
- **(B) The crash at SCV=50 is the real defect to chase** and is most plausibly a velocity-planner divergence/over-commit (H-C1) at the high cornering speed a fat blend permits, or short-move over-consumption (H-C2). It is blocked on the exact error text.

## Recommended Next Steps

### Diagnostic (highest value)

Re-run the SCV=50 scenario and capture the exact failure:
- **viz path:** run `pipeline_snapshot(...)` (or the viz script) with `square_corner_velocity=50` and copy the full Python traceback — the `PyRuntimeError`/`ValueError` message embeds the `VelocityError`/`FitError` debug variant.
- **bench path:** reproduce on the printer and pull the `StreamError` via the `query-logs` skill (per project policy, not a `klippy.log` grep).
- Provide the accel/feedrate and the waypoints/G-code used, so `delta`, `trim`, and `budget` can be recomputed and the crash reproduced deterministically in a unit test.

### Fix direction (pending the error)

- If **C1 `Diverged`**: harden / re-tune the disk-`reach_v` integrator (`velocity/disk.rs`) for high reachable velocities; add a unit test at the SCV that triggers it.
- If **C1 `OverCommitted`**: the look-ahead window cannot brake the committed entry velocity — surface as the loud error it already is, and confirm whether the window/keep-secs sizing is the actual bug.
- If **C2**: add blend-overlap detection across a dropped short move in `fit_chain`/`emit_move` (`fitter.rs:343-377`) and a paired negative test.

## Reproduction Plan

1. Pick the user's waypoints + limits (accel, feedrate). 2. Call `pipeline_snapshot(waypoints, max_velocity, max_accel, square_corner_velocity=50, arc_fit=None)`. 3. Expected: reproduces the crash; capture the error variant. 4. Bisect SCV (e.g. 5 → 20 → 35 → 50) to find the threshold; correlate with `trim → budget` clamp and with the per-move boundary velocity to localize C1 vs C2. 5. Distill to a `geometry` unit test (`velocity/tests.rs` or `fitter/tests.rs`) asserting the exact error.

## Side Findings

- **Streaming does not use the `temporal` SOCP planner** (Finding 5). The commit path rides the `geometry` disk/s-curve forward-backward velocity planner. Given the project's non-negotiable throughput-SOTA constraint, whether/when TOPP is wired into streaming is worth an explicit decision (Backlog #4). Evidence: `stream.rs:168-169`. *(Deduced — could be intentional staging; not the user's issue.)*
- **`junction_deviation` uses the classic Klipper formula** `scv²(√2−1)/accel` (`fitter.rs:419-422`), so SCV semantics match mainline's deviation budget even though the downstream geometry (biclothoid) and the unblended-corner handling (full stop) differ. *(Confirmed.)*

## Follow-up: 2026-06-19

### New Evidence

- **Finding 6 (Confirmed): the viz path emits NO structured logs.** `scripts/viz_pipeline.py:356` calls `_motion_engine.pipeline_snapshot(...)` directly and lets any Rust error propagate as a `PyRuntimeError`/`ValueError` to the terminal's stderr; there is no `structured_log`/`tracing` call in the script or `viz.rs`. This answers "why can't you see the logs from the last crash" — **there are none for this path**; VL carries the host/MCU motion pipeline, not the offline viz CLI. The terminal traceback is the sole artifact. (Local VL `/health` is empty — VL is not running on this dev box either.)
- **Finding 7 (Confirmed): the Rust planner does NOT crash under a wide SCV/limit/geometry sweep.** Driving the locally-built `klippy/_motion_engine.so` (2026-06-19 17:55) directly:
  - Isolated 90° corner: OK for SCV ∈ {5..100}; `blended=1` at every SCV; traversal time *decreases* with SCV (0.633→0.516 s) — confirms Deduction 1.
  - 1 mm zigzag, acute V, faceted parabola (arc_fit), dense circle (arc_fit on/off): OK for SCV ∈ {5..150}.
  - Limit sweep at SCV=50 across `(max_v,max_a)` and low `max_v`: all OK.
  - numpy render stage (`_build_time_series`): finite, monotone `t`, no NaN/inf jerk — corner/circle/arc_fit at SCV {5,50,150}.

### Additional Findings

- The crash is **input-specific** (user's exact G-code geometry and/or `printer.cfg` limits), or a **stale bench build** — the local `.so` is from today; an older bench `.so` could carry a since-fixed bug (cf. bench-firmware-flow: bench state must match git HEAD).

### Updated Hypotheses

- **H-C1 (Diverged/OverCommitted):** weakened. `OverCommitted` cannot fire in viz (`plan_velocity`, `entry_v=0`). `Diverged` not reproduced. Possible only on the user's specific geometry.
- **H-C2 (short-move over-consumption):** not reproduced (1 mm zigzag fine); weakened, not refuted.
- **H-C4 (NEW): stale bench `.so`.** Confirm by rebuilding viz's `_motion_engine.so` on the bench at current HEAD and re-running.

### Backlog Changes

- New top item: obtain the user's terminal traceback OR (`printer.cfg` limits + the exact G-code file) — the only inputs that reproduce.

### Updated Conclusion

"Why no logs" is **resolved (Confirmed)**: the viz CLI writes nothing to the structured pipeline; the crash text exists only on the user's terminal. The crash itself remains **unreproduced** despite an exhaustive local sweep — pointing at the user's specific G-code/config or a stale bench build. Decisive next artifact: the terminal traceback, or the exact inputs.
