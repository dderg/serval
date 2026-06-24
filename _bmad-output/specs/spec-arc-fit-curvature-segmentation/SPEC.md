---
id: SPEC-arc-fit-curvature-segmentation
companions:
  - failure-analysis.md
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only.

# Arc detection by curvature-profile segmentation

## Why

A pain to solve. Slicers approximate circles and rounded corners as dense short G1 line facets. The geometry fitter is supposed to re-recognize those as arcs and ease in/out with clothoids, but its arc detector barely fires — on the `COLD_Voron_Design_Cube` short gcode it produces **0 arcs by default and ≤7 even wide-open**, leaving ~600 per-junction clothoid blends. That clothoid forest is a fully-blended region with **no straight `Line` commit seam**, so the streaming planner's buffer cannot drain there: it re-plans the whole growing backlog on every push (O(n²)), the velocity solve balloons to ~740 ms on the Pi, the MCU plays past the committed pieces, and the bench faults `PieceStartInPast`. Instrumenting the real fitter pinned **two defects**: greedy non-co-circular run growth (one inscribed circle is fit to a whole 51-facet perimeter → 6 mm residual → reject) and fragile clothoid-arc-clothoid seating (genuine 4–11-facet fillets pass the circle test, then fail to seat the entry spiral on their short bounding facets). The decided fix is to replace the detection primitive with **curvature-profile segmentation**: estimate curvature along the polyline and cut it into line / arc / clothoid spans from that signal. See `failure-analysis.md`.

## Capabilities

- id: CAP-1
  intent: The fitter recognizes a sliced circular feature — a circle or a rounded corner arriving as dense short facets — as an arc, instead of a forest of per-junction clothoid blends.
  success: On `neptune_crash_short.gcode` with arc-fit enabled, the ~13 mm circle fits as a single arc (largest arc ≥ 8 mm) and ≤ 40 junctions remain biclothoid-blended — both `tests/arc_fit_neptune.rs` red tests pass.

- id: CAP-2
  intent: An isolated co-circular run reconstructs reliably into its line→clothoid→arc→clothoid→line form; the clothoid blend is derived from the measured transition span, not seated by trial.
  success: Every fillet of 4–11 facets that lies on a common circle within tolerance produces an arc — zero `seam_ok` / consumption rejections on co-circular input — verified by a checked-in fixture test.

- id: CAP-3
  intent: The line/arc/clothoid classification of a path is stable regardless of where a streaming commit window subdivides it.
  success: Fitting any sub-window of the neptune path reproduces the same span classification as the full-path fit (within the window), across commit caps 1..64 in the seam harness.

- id: CAP-4
  intent: Arc detection is configured by the two knobs the curvature primitive actually needs — the co-circular deviation tolerance and the minimum run size — replacing the old facet-length and max-turn-angle parameters.
  success: `with_arc_fit` and the host knob take `(deviation_tol, min_run_facets)`; `facet_len_max` and `max_turn` are gone. Default `min_run_facets` = 3 (= 2 turning junctions). A long facet reads as low curvature → `Line` with no length cap; a single-vertex sharp corner falls below `min_run` → corner blend with no angle cap; a 3-facet co-circular fillet is recognized, and raising `min_run` excludes shorter fillets.

## Constraints

- Output stays `Line` / `Arc` / `Clothoid` with continuous curvature (G2). No bare biarc / G1 joins where curvature steps.
- The same deviation tolerance separates all three classes — within tolerance of a line → `Line`; else within tolerance of a circle and ≥ `min_run` → `Arc`; else corner/clothoid path — so no curvature-floor or angle knob is introduced beyond `{deviation_tol, min_run}`.
- Fitted spans stay within a bounded chord deviation of the original facets — the printed geometry is not distorted (deviation budget: see open question).
- A genuine sharp corner is never absorbed into an arc; corners still take the biclothoid corner-blend path.
- Single near-linear forward pass. No O(n²) — the current re-grow-from-every-start scan is itself part of the stall this work removes.
- Idempotent: re-fitting the output reproduces it.
- Drives the real `fit_chain` / `StreamState` path; validated by the offline seam/arc test platform (`arc_fit_neptune` + the repro/census tooling), never a mock.
- Change is confined to the geometry fitter's detection/reconstruction (`chain.rs`). The biclothoid corner-blend path, the velocity planner, and the streaming commit logic keep their current behavior.

## Non-goals

- The continuity/stall fix for the arc-fit-**disabled** (pure-clothoid) path. That path must also never produce a discontinuity or stall, but it is separate deferred work.
- Changing the production default: arc-fit stays **disabled by default** for now. This work makes detection correct when enabled; flipping the default on is out of scope.
- The commit re-plan amplification safety net (short-circuiting `commit(false)` when the committable frontier has not advanced) — a separate throughput fix.
- Biarc / G1-only fitting; native G2/G3 arc-input streaming from slicer arc commands.
- Re-litigating the `is_clean_seam` seam-continuity fix (done) or the MCU tick-projection (TickChain).

## Success signal

With arc-fit enabled, the cube's circle and rounded corners fit as arcs and the clothoid forest collapses (both `arc_fit_neptune` reds green). Driven through the real `StreamState` commit path, the worst single-commit compute on `neptune_crash_short` drops from the O(n²) forest blowup (~76 ms offline on a dev host, ~740 ms on the Pi) to a bounded value — removing the commit-starvation that faults `PieceStartInPast` on the bench for this geometry.

## Assumptions

- Per-vertex curvature estimated from short, coordinate-quantized slicer facets can be denoised enough to segment reliably without hallucinating an arc across a real corner. This is the make-or-break risk and the first thing to prototype.
- The existing `Arc` / `Clothoid` segment types and the `reconstruct` blend math are reusable; only the detection/segmentation primitive and the seating derivation change.

## Open Questions

- What value for the deviation tolerance? It is now the single quality knob — it bounds part distortion *and* draws the line/arc/corner boundaries. `cocircular_tol` is 5 µm today; confirm that value (or a new default) survives real curvature noise without hallucinating arcs across corners.
