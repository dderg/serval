---
id: SPEC-arc-fit-curvature-segmentation
companions:
  - failure-analysis.md
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only.

# Arc detection by facet uniformity

## Why

A pain to solve. Slicers approximate circles and rounded corners as dense short G1 line facets. The geometry fitter is supposed to re-recognize those as arcs and ease in/out with clothoids, but its arc detector barely fires — on the `COLD_Voron_Design_Cube` short gcode it produces **0 arcs by default and ≤7 even wide-open**, leaving ~600 per-junction clothoid blends. That forest is a fully-blended region with **no straight `Line` commit seam**, so the streaming buffer cannot drain: it re-plans the growing backlog on every push (O(n²)), the velocity solve balloons to ~740 ms on the Pi, and the bench faults `PieceStartInPast`.

The first attempt — least-squares circle-fitting within a deviation tolerance ("curvature-profile segmentation") — was built and **empirically hit its make-or-break**: on real slicer output the curved features are not clean circles to the fit's precision. Chord lengths along an arc swing ±~23%, and the "arc" bands fuse straight wall edges (a 9.86 mm straight sits inside one half-circle band) and corners. So least-squares circle-fitting both **misses** real arcs (the fused straight wrecks the fit → 3–6 mm residual) and **accepts** non-arcs (a few mis-angled straights satisfy "within tol of *some* big circle"), while a single deviation tolerance does double duty and inverts (looser tol → fewer arcs).

The corrected primitive is **facet uniformity**: a printed circular arc is a run of facets with equal chord **length**, equal turn **angle**, and equal **extrusion-per-mm**, each within a tolerance. Constant chord + constant angle is exactly the chord form of a circle (`R = L / (2·sin(Δθ/2))`); constant extrusion-per-mm keeps the run inside one printed feature. Segment by breaking the run wherever any of the three departs — which cleanly cuts at fused straights (length jump), corners (angle jump), and feature boundaries (E/mm jump). See `failure-analysis.md`.

## Capabilities

- id: CAP-1
  intent: The fitter recognizes a sliced circular feature — a circle or rounded corner arriving as dense short facets — as an arc, instead of a forest of per-junction clothoid blends.
  success: On `neptune_crash_short.gcode` with arc-fit enabled, the center circle fits as a single arc (largest arc ≥ 8 mm) and ≤ 40 junctions remain biclothoid-blended — both `tests/arc_fit_neptune.rs` red tests pass.

- id: CAP-2
  intent: A uniform run reconstructs reliably into its line→clothoid→arc→clothoid→line form, fed a clean run that no longer fuses straights or corners.
  success: A uniform run of ≥ `min_run` facets always produces an arc — zero `seam_ok` / consumption rejections on a uniform run — verified by a checked-in fixture test.

- id: CAP-3
  intent: The arc/line/clothoid classification of a path is stable regardless of where a streaming commit window subdivides it.
  success: Fitting any sub-window of the neptune path reproduces the same span classification as the full-path fit (within the window), across commit caps 1..64 in the seam harness.

- id: CAP-4
  intent: Arc detection is configured by the uniformity tolerances the primitive needs — chord-length, turn-angle, extrusion-per-mm, and the minimum run size.
  success: `with_arc_fit` and the host `[arc_fit]` knob take `(length_tol, angle_tol, epmm_tol, min_run)`; the old `deviation_tol` / `facet_len` / `max_turn` parameters are gone. A run of uniform facets is recognized as an arc; a fused straight (length jump), a corner (angle jump), or a feature change (E/mm jump) breaks the run at that facet.

- id: CAP-5
  intent: Extrusion-per-mm uniformity confines an arc run to a single printed feature, so a fused travel move or a different-width neighbor cannot be absorbed into the arc.
  success: A run spanning an E/mm change beyond `epmm_tol` is split there; a non-extruding move (E/mm = 0) never joins an extruding arc. The offline harness carries real per-facet extrusion (today it zeroes it) so this is testable offline.

## Constraints

- Output stays `Line` / `Arc` / `Clothoid` with continuous curvature (G2). No bare biarc / G1 joins.
- Detection is by facet **uniformity**, not least-squares circle deviation: a run is an arc iff its facets share chord length, turn angle, and extrusion-per-mm each within tolerance; the radius follows from `R = L/(2·sin(Δθ/2))`.
- Break the run wherever any uniformity signal departs beyond its tolerance — this *is* the segmentation; fused straights, corners, and feature boundaries all break it.
- Tolerances compare each facet against the run's running average (not just its neighbor), so a genuine arc with the slicer's ±~23% chord swing still holds together while a 20× length jump breaks it.
- A genuine sharp corner is never absorbed (its angle jump breaks the run).
- Single near-linear forward pass; no O(n²) re-grow.
- Idempotent: re-fitting the output reproduces it.
- Drives the real `fit_chain` / `StreamState` path, validated by the offline seam/arc platform — which must be extended to carry per-facet extrusion (currently `parse_gcode_to_moves` hardcodes E = 0) so the E/mm signal is testable offline.
- Change is confined to the geometry fitter (`chain.rs`) plus the harness extrusion fix; the corner-blend path, velocity planner, and streaming commit keep their behavior.

## Non-goals

- The continuity/stall fix for the arc-fit-**disabled** (pure-clothoid) path — separate deferred work.
- Changing the production default: arc-fit stays **disabled by default**; flipping it on is out of scope.
- The commit re-plan-amplification safety net (short-circuit `commit(false)` when the frontier hasn't advanced).
- Least-squares circle-fitting as the detection gate (tried, abandoned — see Why); biarc / G1 fitting; native G2/G3 arc-input streaming.
- Re-litigating the `is_clean_seam` seam fix (done) or the MCU tick-projection (TickChain).

## Success signal

With arc-fit enabled, the cube's center circle and rounded corners fit as arcs and the clothoid forest collapses (both `arc_fit_neptune` reds green), with no straight wall edge mistaken for an arc. Driven through the real `StreamState` path, the worst single-commit compute on `neptune_crash_short` drops from the O(n²) forest blowup (~76 ms offline, ~740 ms on the Pi) to a bounded value — removing the commit-starvation that faults `PieceStartInPast`.

## Assumptions

- Real slicer chords vary (±~23% observed) and bands fuse straights/corners, so the uniformity tolerances must be generous enough (vs. the running average) to hold a genuine arc together yet tight enough to break at a fused straight or corner. Finding the three tolerance values on the real fixture is the make-or-break.
- The `Arc` / `Clothoid` types and the `reconstruct` blend math are reusable once fed a clean uniform run.

## Open Questions

- The three uniformity tolerance values — chord-length (relative %), turn-angle (degrees), extrusion-per-mm (relative %) — what defaults survive real slicer variation without over- or under-segmenting on the cube fixture?
