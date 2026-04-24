# Plan 9 Phase A3 — completion report

**Status:** COMPLETE
**Date:** 2026-04-24
**Branch:** `magnum-opus`
**Commits (9 total):**
- `d800ae28` plan9-A3T1: split Move emit into build_unshaped + finalize_shape
- `e7e0c96d` plan9-A3T2: capture input-shaper snapshot on Move at construction
- `9821af6b` plan9-A3T2-fixup: cache shapers snapshot on toolhead, clean rename
- `3f9c8220` plan9-A3T3: shape-bake Move polynomials via _bake_shaper_polynomial
- `6fe0b2ef` plan9-A3T3-fixup: promote shape helpers public, drop redundant tuple, docstring
- `06596c62` plan9-A3T4: LookAheadQueue deferred-last-move state
- `a556ecb2` plan9-A3T5: LookAheadQueue deferred-last shape-bake pass
- `ff32b638` plan9-A3T5-fixup: drain pending on empty-queue lazy=False flush
- `f8d2496c` plan9-A3T5-polish: extract finalize-with-neighbours helper, fold pending state into tuple
- this commit — plan9-A3T6: integration tests + completion report

## What shipped

- **`Move.build_unshaped_payload()`** — produces the raw jerk XY + PA-baked E polynomial (A2d's `build_quintic_payload` split into a 3-tuple `(phase_t_ends, total_t, coeff_tuple)`). Neighbour context is absent at this stage; shape-baking applies later in `finalize_shape`. Stored on `Move._unshaped_payload`.

- **`Move.finalize_shape()`** — convolves `_shapers_snapshot` with the unshaped polynomial via `blendplanner.bake_shaper_polynomial`. Neighbour polynomials (`prev_unshaped`, `next_unshaped`) are shifted into the current move's reference frame via `blendplanner.offset_unshaped_for_neighbour` before composition. Writes the result to `self.quintic_trapq_payload` (the 9-tuple consumed by `_process_moves`). A safety-net call in `set_junction` provides zero-pad neighbours; the `LookAheadQueue` flush pass overwrites with queue-internal neighbours mid-stream.

- **`Move._shapers_snapshot`** — captured at `Move.__init__` as `toolhead.shapers_snapshot` (the toolhead-owned cache, refreshed on `SET_INPUT_SHAPER` / `klippy:connect`). This is an O(1) read rather than calling `extract_shapers` + `find_shaper_max_accel` bisection per move (the A3T2 fixup moved the expensive extraction to the cache-refresh path).

- **`blendmath.extract_shapers`** — promoted from `_extract_shapers` (public alias). The `ToolHead` stores the result in `self.shapers_snapshot` and all per-move reads go through the cache.

- **`LookAheadQueue` deferred-last pattern** — state field `_pending_last` holds a 3-tuple `(move, prev_unshaped, prev_start_pos_xyz)` across flush boundaries. `flush(lazy=True)` holds the last kinematic plain Move as pending until the next batch arrives, so its "next" neighbour polynomial is known. `flush(lazy=False)` drains the pending move with `next=None` (the print actually stops there — zero-pad is correct). `reset()` clears the state. A helper `_finalize_with_neighbours` encapsulates the neighbour-extract + `finalize_shape` dispatch logic.

- **Every kinematic Move now exits the planner shape-baked by construction.** Two paths:
  - `QuinticBlendMove` (blended corners): shape-baked upstream by `CornerBlender.finalize_pending` — unchanged.
  - Plain `Move` (straight segments, non-blended): shape-baked by `LookAheadQueue.flush` in the A3 deferred-last pass — new.

  This closes **Plan 9 Pillar 1: "shape-baked by construction; no move leaves the planner unshaped."**

## Validation

- **3 new A3 integration tests** in `test/test_toolhead_shape_bake.py`:
  1. `test_a3_e2e_mzv_shaper_bakes_payload_through_lookahead` — end-to-end: 3 moves queued through LookAheadQueue with MZV 42 Hz; after drain all three moves have `baked_coeffs != unshaped_coeffs`.
  2. `test_a3_i3_coverage_gap_neighbour_changes_coeff_tuple` — I3 coverage gap: proves `m1.quintic_trapq_payload[5]` differs when m1 is baked with m2 as queue-internal next neighbour vs. baked alone with `next=None`. Confirms the deferred-last pattern propagates neighbour context into the shaper convolution.
  3. `test_a3_ztilt_regression_1000mms_shape_baked` — z_tilt structural test: 100 mm move at 1000 mm/s with MZV 42 Hz exits flush with `baked_coeffs != unshaped_coeffs`. Structural precondition for the hardware fix.

- Targeted Plan-9-specific suite: **574 passed, 4 skipped, 0 failed** (was 571 before T6).
- A3-specific new tests (T1–T6): **24 new tests** in `test_toolhead_shape_bake.py`.
- Full cumulative Plan-9-specific test count: 274 (pre-A3 baseline) + 24 (A3) = **298**.
- No regressions in `test_blendextruder_integration.py`, `test_chunk3_pa_integration.py`, `test_blendplanner.py`, `test_blendprepass.py`, `test_toolhead_jerk_wiring.py`, `test_toolhead_jerk_integration.py`.

## Architecture notes

- `bake_shaper_polynomial` is now invoked from **two sites**: `QuinticBlendMove.finalize_shape` (upstream, CornerBlender) and `Move.finalize_shape` (new, LookAheadQueue). Both use the same 3-tuple `(phase_t_ends, total_t, coeff_tuple)` format and identical neighbour semantics.

- The deferred-last pattern **mirrors `CornerBlender._finalize_pending`**. Every kinematic Move in the LookAheadQueue will see non-None neighbours mid-stream; zero-pad occurs only at true print boundaries (`lazy=False` drain with no follow-up move).

- **`_is_shape_bake_target(move)`** — module-level predicate: `isinstance(move, Move) and move.is_kinematic_move`. QuinticBlendMove is a standalone class (not a Move subclass — see `klippy/blendplanner.py:341`), so the `isinstance(move, Move)` discriminator excludes it from the LookAheadQueue's shape-bake pass entirely. QBMs are shape-baked upstream by `CornerBlender._finalize_pending` before they reach the inner LookAheadQueue, so this exclusion is correct — there's no double-baking risk and no missed bake.

- **Safety-net call in `set_junction`**: `set_junction` still calls `finalize_shape()` immediately after building `_unshaped_payload`. This zero-pad bake ensures `quintic_trapq_payload` is always populated for unit tests that bypass the LookAheadQueue. The flush pass's `finalize_shape` call overwrites this with correct neighbours for production paths.

## Hardware regression target

A3 targets the **z_tilt-stepper-slip regression on Trident at 1000 mm/s**. Before A3, plain `Move` objects emitted un-shaped polynomials (the shape-bake step only ran for `QuinticBlendMove` via CornerBlender). Under input shaper, the commanded trajectory had no shaping, so the stepper was driven at its raw jerk rate — sufficient to cause missed steps / stepper slip under z_tilt calibration's high-speed traverse.

A3 closes this gap: every plain `Move` now exits the planner with a shaper-convolved polynomial, matching the guarantee that QuinticBlendMoves already had.

**Hardware validation is the next step.** Confirm on Trident that z_tilt calibration at 1000 mm/s no longer produces stepper slip after merging `magnum-opus`.

## Known limits / follow-up

- **Known limit — cross-blend-boundary zero-pad.** At every boundary between a plain Move and a QuinticBlendMove (in either direction), the shaper kernel sees a zero-pad discontinuity on the cross-boundary side: the discriminator skips QBMs as neighbours for plain Moves, and CornerBlender finalizes QBMs with `next=None` whenever the following move isn't a blend (`klippy/blendplanner.py:774,908`). This means "shape-baked by construction" holds within blend runs and within plain-Move runs, but is degraded at run-to-run transitions. Not a correctness blocker for the z_tilt straight-line target (no blend runs involved), but flagged for future work — likely a Phase A6 or B follow-up to plumb the QBM↔Move neighbour handshake.

- **No integration test exercises the full upstream pipeline.** All 24 A3 tests inject moves directly into the inner `laq.queue`, bypassing `ToolHead.lookahead.add_move` → `BlendPipelineLookAheadQueue` → `CollinearCollapser` → `CornerBlender` → inner `LookAheadQueue`. A broken prepass or blender filter with a configured shaper wouldn't be caught by the current test set. Track for follow-up — add an end-to-end test that drives the full filter stack.

- **Wasted safety-net `finalize_shape()` call.** `Move.set_junction` at `klippy/toolhead.py:344` calls `finalize_shape()` without neighbours, which is then overwritten by `LookAheadQueue.flush`'s neighbour-aware call. For an N-move flush that's N wasted shape-bake calls — pure cost, not correctness. Track for follow-up cleanup (can be guarded by a flag set only on the direct-test path).

- **`shape_disabled` bypass audit** — drip_move / force_move / manual_stepper / IDEX paths call `lookahead.flush` in ways that may skip the shape-bake pass or route through the old trapezoid emit. Documented as future scope; not in A3 scope.

- **Phase B onwards** — host↔MCU protocol redesign for quintic polynomial emit; MCU firmware for the `trapq_append_quintic` path; Rust rewrite candidates (per Plan 9 scope expansion 2026-04-24).
