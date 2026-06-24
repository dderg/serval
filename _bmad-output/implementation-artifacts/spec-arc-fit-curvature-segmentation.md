---
title: 'Arc detection by curvature-profile (deviation-gated) segmentation'
type: 'feature'
created: '2026-06-24'
status: 'in-progress'
baseline_commit: 'e995c80718b20cacd8b4e0e780470e4d3b054189'
context:
  - '{project-root}/_bmad-output/specs/spec-arc-fit-curvature-segmentation/SPEC.md'
  - '{project-root}/_bmad-output/specs/spec-arc-fit-curvature-segmentation/failure-analysis.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The fitter shatters sliced circles and rounded corners (dense short G1 facets) into a clothoid forest — 0 arcs by default, ≤7 wide-open, ~600 clothoids. `grow_run` grows a maximal same-turn-sign run and `reconstruct` fits **one** inscribed circle to the whole thing, so a perimeter loop is 6 mm off a circle → rejected, and the start advances by one and re-fits the same maximal run (O(n²)). The forest has no straight `Line` commit seam, so the streaming buffer can't drain → O(n²) re-plan → `PieceStartInPast` on the bench.

**Approach:** Replace the growth criterion with **deviation-gated segmentation**: walk facets once, grow a run while it stays within `deviation_tol` of a single circle (or a line); emit each co-circular span of ≥ `min_run` facets as an `Arc` with clothoid blends to its neighbors, leaving straights as `Line`. Reshape the arc-fit config to the two knobs this needs — `{deviation_tol, min_run}` — dropping the facet-length and max-turn proxies.

## Boundaries & Constraints

**Always:** Output stays `Line`/`Arc`/`Clothoid`, G2-continuous. Single near-linear forward pass — no re-grow-from-every-start. A genuine sharp corner (single-vertex, < `min_run`) is never absorbed into an arc; it takes the existing biclothoid corner path. Fitted spans stay within `deviation_tol` of the original facets. Segmentation is idempotent. Arc-fit stays **disabled by default** (production `ChainFitConfig::default()` unchanged). Change is confined to the geometry fitter.

**Ask First:** If no `deviation_tol` isolates the neptune circle without also hallucinating an arc across a real corner (the make-or-break), HALT — the approach needs rethinking before more code. Any change to the biclothoid corner blend, the velocity planner, or streaming commit behavior.

**Never:** Flipping arc-fit on by default. Biarc / G1 joins. Native G2/G3 arc input. The commit re-plan-amplification safety net. Touching the `is_clean_seam` seam fix.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Co-circular fillet | ≥ `min_run` facets within `deviation_tol` of one circle | one `Arc` + entry/exit clothoid blends | N/A |
| Straight run | facets within `deviation_tol` of a line | `Line`, no arc | N/A |
| Sharp corner | single-vertex turn (< `min_run`) | biclothoid corner blend, not arc | N/A |
| Multi-feature perimeter | arc · straight · arc, same turn sign | segmented `Arc │ Line │ Arc`, not one rejected circle | N/A |
| Co-circular but short | < `min_run` co-circular facets | not an arc; corner/line path | N/A |

</frozen-after-approval>

## Code Map

- `geometry/src/fitter.rs` -- `ArcFitConfig` reshape to `{deviation_tol_mm, min_run_facets}`; `ChainFitConfig` drops `cocircular_tol` + `min_run_junctions`; `with_arc_fit(deviation_tol, min_run)`.
- `geometry/src/fitter/chain.rs` -- `detect_runs`/`grow_run`: replace growth with deviation-gated segmentation (`incircle` normal-equations are incremental — extend O(1)/facet); `reconstruct`: clamp `l_t` so head/tail feet seat within the bounding facets; tolerance reads `deviation_tol`.
- `geometry/src/fitter/chain/tests.rs` -- replace the angle/length-gate tests; add corner/line/idempotence/perimeter.
- `motion-engine/tests/arc_fit_neptune.rs` -- red acceptance (must go green).
- `motion-engine/src/bridge.rs`, `motion-engine/src/viz.rs` -- host arc-fit knob → `(deviation_tol, min_run)`.
- `motion-engine/examples/{analyze_arc_fit,repro_plan_stall}.rs` -- measurement harnesses.

## Tasks & Acceptance

**Execution:**
- [ ] `geometry/src/fitter.rs` -- reshape `ArcFitConfig` to `{deviation_tol_mm, min_run_facets}`, rewrite `with_arc_fit`, remove `facet_len_max_mm`/`max_turn_rad`/`cocircular_tol`/`min_run_junctions`, point reconstruct tolerance at `deviation_tol`.
- [ ] `geometry/src/fitter/chain.rs` -- replace `grow_run`/`detect_runs` growth with single-pass deviation-gated segmentation: grow while ≤ `deviation_tol` of one circle, classify line-vs-arc by also testing a line fit, emit co-circular ≥ `min_run` spans, resume from each break.
- [ ] `geometry/src/fitter/chain.rs` -- clamp `reconstruct`'s `l_t`/anchor so the entry/exit spirals seat within the bounding facets (removes the `seam_ok`/consumption rejections on real fillets).
- [ ] `geometry/src/fitter/chain/tests.rs` -- replace `sharp_corners_rejected_by_angle_gate` + `long_facets_rejected_by_length_gate` with `deviation`/`min_run` equivalents; add `corner_not_absorbed`, `near_straight_is_line`, `segmentation_is_idempotent`, `multi_feature_perimeter_segments`.
- [ ] `motion-engine/src/bridge.rs` + `motion-engine/src/viz.rs` -- carry the host arc-fit knob as `(deviation_tol, min_run)`.

**Acceptance Criteria:**
- Given `neptune_crash_short.gcode` with arc-fit enabled, when fit, then the largest arc ≥ 8 mm and ≤ 40 junctions are biclothoid-blended — both `arc_fit_neptune` tests pass.
- Given a perimeter of arc·straight·arc, when segmented, then it yields ≥ 2 `Arc` spans separated by a `Line`, not one rejected circle.
- Given any fit output, when re-fit, then it reproduces (idempotent).
- Given the neptune gcode driven through `StreamState` (`repro_plan_stall`) with arc-fit on, when measured, then worst-commit compute drops materially versus the clothoid-forest baseline (no O(n²) blowup).
- Given arc-fit disabled (default), when the full suite runs, then every previously-passing test still passes (production path unchanged).

## Design Notes

`deviation_tol` is the single quality knob — sweep it on the neptune fixture first (too loose absorbs a real corner into an arc; too tight shatters the circle); this sweep is the make-or-break and settles the spec's one open question. `incircle`'s `ata`/`atb` are sums over facets, so growth extends in O(1) per facet — one forward pass. `reconstruct` already emits G2, seam-exact output for clean arcs (`reconstruction_is_g2_and_seam_exact`); the work is feeding it clean co-circular spans and clamping `l_t`, not rebuilding the blend.

## Verification

**Commands:**
- `cargo nextest run -p motion-engine -E 'test(arc_fit)'` -- both reds now PASS.
- `cargo nextest run -p geometry` -- green under the reshaped config.
- `cargo run --release -p motion-engine --features test-support --example analyze_arc_fit -- motion-engine/tests/gcode/neptune_crash_short.gcode` -- circle shows as a multi-mm arc; clothoid count collapses.
- `cargo run --release -p motion-engine --features test-support --example repro_plan_stall -- motion-engine/tests/gcode/neptune_crash_short.gcode --arc <tol>,<min>` -- worst-commit drops vs `--arc` off.
- `./scripts/ci.sh rust-clippy && ./scripts/ci.sh rust-fmt` -- clean.
