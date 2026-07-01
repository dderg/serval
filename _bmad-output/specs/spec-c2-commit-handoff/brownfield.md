# Brownfield: current commit-path mechanism

The load-bearing map of the streaming commit path the implementer must work within. All paths are `rust/motion-engine/src/` unless noted.

## Commit loop — `stream.rs` `StreamState::commit`
- Fits the buffered window: `fit_chain_with_head_restore(&moves, chain, committed_head_len)`.
- Warm-starts velocity: `plan_velocity_warm_start(&outcome, velocity, entry_v)` — `entry_v` is the **only** state carried across the seam (a scalar speed).
- Lowers each move to a `ShapedSegment`, advancing an odometer for absolute position.
- Picks `commit_count` at a clean seam strictly before the finality barrier (`profile.barrier`).
- On commit, sets `entry_v = exit_v` at the seam and recomputes `committed_head_len`.

## The κ=0 restriction — `stream.rs:704-707` `is_clean_seam`
```
A non-forced commit may cut wherever the fit output resumes a straight line body
(zero curvature: an unblended seam or the exit of a blend) — never inside a blend,
where curvature is nonzero and the velocity warm-start, which carries only a scalar
entry speed, would be invalid.
```
`is_clean_seam` returns true only when the resuming move is `Segment::Line` or its source line is in the `unblended` set. This is the single rule that forbids mid-arc commits; CAP-1 relaxes it once CAP-2/CAP-3 make a nonzero-κ resume well-defined.

## Finality barrier — `stream.rs` `brake_to_rest_setback`
Holds the commit boundary back by a jerk-limited braking distance from the buffer's peak feedrate, so every committed body is "a function of geometry alone, final under append and output-equivalent to a full re-plan — positions exactly, seam timing within the velocity stage's tolerance." This guarantee must survive the change.

## Precedent to follow — `head_len_restore`
The existing window-invariance mechanism for **corners**: a commit that trims the head of the resuming move records the trimmed length in `StreamState::committed_head_len`; `fit_chain_with_head_restore` (`rust/geometry/src/fitter.rs`) adds it back into the leading junction's blend budget so the corner re-fits to the same curvature it had pre-commit. The curvature-carry for arcs is the same shape of fix one level up: carry the boundary curvature state, not just a trimmed length. Background: `docs/rewrite/windowed-fit-ceiling-jitter.md`.

## Warm-start — `rust/geometry/src/velocity.rs:133` `plan_velocity_warm_start(outcome, velocity, entry_v)`
Already reads per-segment curvature internally (`seg.kappa_endpoints()`, `seg.dkappa_ds(0)`, `kappa_peak`) to curvature-limit cornering — but treats the window entry as a fresh κ=0 start. CAP-3 seeds it with the carried entry curvature.

## Fail-loud guard — `pump.rs` `check_junction_position_continuity`
Aborts on a junction position discontinuity (`JUNCTION_POSITION_FATAL_MM = 0.1`). The curvature-match contract is enforced against this guard's tolerance; a divergent resume must trip it, not be padded. This guard firing on `{mcu_id:1, axis:0}` (EtherCAT servo X) is the crash this whole line of work originated from.

## Regression anchor
`arc_fit_voron_cube_perimeter_is_c0_at_every_commit_cadence` in `motion-engine/src/seam_test_harness/tests.rs` replays the crash fixture at caps `[1, 2, 4, 8, 16, 64, 100000]` and asserts `rep.fatal() == 0`. The whole-buffer fit (cap 100000) is already clean for the validated quality fix; small caps are what this spec must bring green.
