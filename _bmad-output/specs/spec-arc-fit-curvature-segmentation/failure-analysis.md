# Failure analysis — why arc detection doesn't fire, and the curvature-profile fix

Companion to `SPEC.md`. Instrumented findings from the real `geometry/src/fitter/chain.rs` on `motion-engine/tests/gcode/neptune_crash_short.gcode` (310 moves; a short `COLD_Voron_Design_Cube` slice with a ~13 mm circle, rounded corners, and straight edges).

## Empirical baseline (segment census, `fit_chain`)

| config | Arc | Clothoid | chains | blended | max_arc |
| --- | --- | --- | --- | --- | --- |
| default (`arc_fit: None`) | 0 | 610 | 0 | 305 | 0 mm |
| `with_arc_fit(1mm, 12°)` | 0 | 610 | 0 | 305 | 0 mm |
| `with_arc_fit(2mm, 90°)` | 5 | 600 | 5 | 295 | 1.53 mm |
| `with_arc_fit(∞, 180°)` | 7 | 596 | 7 | 291 | 2.00 mm |
| `with_arc_fit(2mm, 90°)` + `cocircular_tol` 5 µm→1 mm | 5 | 600 | 5 | 295 | 1.53 mm |

The 13 mm circle (~41 mm circumference) is never captured — the largest arc found is a 2 mm nub. Loosening `cocircular_tol` 200× changes nothing, so the residual gate is **not** the limiter.

## Current pipeline

`fit_chain_with_head_restore` → `detect_runs` → `grow_run` (maximal same-turn-sign run) → `reconstruct` (fit one inscribed circle via `incircle`, wrap with entry/exit clothoid spirals).

## Defect 1 — greedy, non-co-circular growth (dominant: ~1015 rejects)

`grow_run` extends a run while the turn sign is consistent and facets are short, and its `theta ≤ theta_min` clause **absorbs near-straight facets** instead of breaking. So a whole perimeter loop becomes one run. Instrumented trace:

```
grow_run start=1 end=51 facets=51 turning=50
  REJECT residual 6.1057 > tol 0.0050      ← one circle, 6mm off a 51-facet perimeter
grow_run start=2 end=51 facets=50 turning=49
  REJECT residual 6.1023 > tol 0.0050
grow_run start=3 end=51 facets=49 ...        (same end=51, re-fails, ~O(n²))
```

`reconstruct` asks "is this whole run **one** circle?" — a rounded rectangle isn't, so residual ≈ 6 mm → reject. `detect_runs` then advances the start by one and **re-grows to the same `end`**, re-fitting one circle and failing again: O(n²) repeated identical failures that never try the individual arcs inside the run. The circle and fillets are swallowed into perimeter runs and never isolated. (Note: the L1–L51 run has 50 *turning* junctions and 0 straight breaks — the perimeter is continuously curving but is several arcs of different radius joined by gentle transitions, not one circle. So the break criterion must be **loss of co-circularity**, which also covers true straights, not merely "break at a straight.")

## Defect 2 — fragile spiral seating (~491 + ~138 rejects)

Runs that *are* genuinely circular — the isolated fillets, **4–11 facets** — pass the circle/residual test and then fail downstream:

- `head seam not on line` (491): `seam_ok(s0, head_len, lines[0])` — the entry clothoid's foot `s0` lands off the first facet.
- `consumption oob` (138): head/tail consumption outside the bounding facet length.

Root: the clothoid length `l_t = min(√(24·ρ·δ), len_first, len_last, 0.5·ρ)` and the `spiral_anchor_offset` placement overshoot the short bounding facets of a real fillet. So even a correctly-identified arc is thrown away.

## The fix — curvature-profile segmentation

Replace the detection primitive:

1. Estimate curvature κ at each interior vertex (e.g. circumscribed circle of the facet triplet), producing a 1-D κ(s) profile along the polyline.
2. Segment that profile into spans: κ ≈ 0 → `Line`; κ ≈ const ≠ 0 → `Arc` (ρ = 1/κ); κ ramping ≈ linearly → `Clothoid` transition.
3. Fit each span once; emit `line → clothoid → arc → clothoid → line`.

Why this addresses both defects and the streaming needs:

- **Defect 1:** segmentation cuts at curvature *changes*, so a perimeter splits into its constituent arcs/lines automatically — the circle and each fillet become their own span. No "one circle per maximal run."
- **Defect 2:** the transition span where κ ramps **is** the clothoid, by measurement — no trial seating of a spiral onto facets, so the `seam_ok`/consumption rejections disappear.
- **Window-stable (CAP-3):** κ is a *local* measurement, so the classification of a span is independent of where a streaming window starts — unlike greedy left-to-right growth.
- **Cost:** a single near-linear forward pass — removes the O(n²) re-grow that is itself part of the stall.
- **Output vocabulary:** line/arc/clothoid with G2 continuity is exactly what κ-segmentation produces; biarc (G1) would not.

### Acceptance guards (must hold, beyond the neptune reds)

- **Deviation bound:** every fitted arc/clothoid stays within a tight band of the original chords — the part is not distorted.
- **No corner absorption:** a genuine sharp corner is never classified as arc/clothoid; it still goes through the biclothoid corner blend.
- **Idempotence:** re-fitting the output reproduces it.

### Risk

κ from short, coordinate-quantized slicer facets is noisy; naive thresholding can hallucinate an arc across a corner or shatter one circle. Denoising κ robustly is the make-or-break and the first thing to prototype.

## Code map

- `geometry/src/fitter/chain.rs` — `detect_runs` (32), `grow_run` (64), `reconstruct` (124), `incircle` (254): the detection/reconstruction to rework.
- `geometry/src/fitter.rs` — `ArcFitConfig` (35: `facet_len_max_mm`, `max_turn_rad`), `ChainFitConfig` (`arc_fit: None` default 54; `min_run_junctions` 2; `cocircular_tol` 5 µm), `with_arc_fit` (60). CAP-4 reshapes this: `ArcFitConfig`/`with_arc_fit`/the host knob (`bridge.rs` `arc_fit: Option<(f64, f64)>`) become `(deviation_tol, min_run_facets)` — `facet_len_max_mm` and `max_turn_rad` are removed (curvature subsumes them); `cocircular_tol` becomes the exposed `deviation_tol`; `min_run_junctions` becomes the exposed `min_run_facets`.
- `motion-engine/tests/arc_fit_neptune.rs` — the red acceptance tests (`max_arc ≥ 8mm`, `blended ≤ 40`).
- `motion-engine/tests/gcode/neptune_crash_short.gcode` — the fixture.
- `motion-engine/examples/analyze_arc_fit.rs` — segment-type census across configs.
- `motion-engine/examples/repro_plan_stall.rs` — times the real commit path; measures the O(n²) blowup the fix must remove.
