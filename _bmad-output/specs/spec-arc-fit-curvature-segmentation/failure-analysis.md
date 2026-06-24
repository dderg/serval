# Failure analysis — why arc detection doesn't fire, and the uniformity fix

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

## Attempt 1 (built, superseded): least-squares circle segmentation

Replaced greedy `grow_run` with a bounded turning-band + co-circular (least-squares `incircle`, residual ≤ `deviation_tol`) segmentation, config reshaped to `{deviation_tol, min_run}`. This killed the unbounded growth (no more 138-facet / 4654° runaway) but **hit its make-or-break**: arc count stays ~4, max 2 mm, the center circle never fit, at *any* `deviation_tol` from 0.005–4 mm. Measured per-facet on the fixture, here is why least-squares circle-fitting is the wrong gate:

- **Bands fuse non-circular geometry.** The half-circle band `[162..173]` contains a 9.86 mm straight wall (`FACET 167 len=9.861 turn=1.78°`) wedged between 0.3–0.4 mm curve facets. One circle fit through curve + straight → 3 mm residual → reject. The real arc is never isolated.
- **Real arc chords are not uniform.** The big curving runs turn consistently (~7°) but chord length swings **±~23%** (alternating ~1.13 / ~0.77 mm). A tight residual gate rejects them; a loose one admits junk.
- **A single `deviation_tol` inverts.** It did double duty (circle gate ↑arcs vs a chord-deviation line-exclusion ↓arcs), so *looser* tol gave *fewer* arcs. The line-exclusion was removed (knob is now a clean upper bound), but the count stays capped by reconstruct seating regardless.
- **Loose tol → false positives.** A few mis-angled straights satisfy "within tol of *some* big circle" and get fit as arcs that aren't.

## The fix — facet uniformity

A printed circular arc is a run of facets with constant chord **length**, constant turn **angle**, and constant **extrusion-per-mm**, each within a tolerance. Constant length + constant angle is the chord form of a circle (`R = L/(2·sin(Δθ/2))`) — not a heuristic, the circle's definition in chord space. Constant E/mm keeps the run inside one printed feature.

Segment by walking facets and breaking the run wherever length, angle, or E/mm departs from the run's **running average** beyond its tolerance. This cleanly cuts the fused 9.86 mm straight (length jump 20×), corners (angle jump to ~90°), and feature boundaries (E/mm change, incl. travels at E/mm = 0). Then fit/emit the arc over each clean run. Compare against the running average (not the neighbor) so the ±23% chord swing of a genuine arc holds together.

Why it beats least-squares:

- **Isolates** the real arc from fused straights/corners — the segmentation LS got wrong → catches the center circle.
- **Rejects** mis-angled-straight false positives that LS accepts at loose tol.
- **Window-stable (CAP-3):** a local measurement; single forward pass; no O(n²).

### Harness gap (blocks CAP-5)
The offline harness zeroes extrusion: `parse_gcode_to_moves` calls `build_move(..., 0.0, ...)`, so every facet reads E/mm = 0 offline. Carry per-facet extrusion through before the E/mm signal can be tested offline.

### Acceptance guards (beyond the neptune reds)
- No straight wall edge mistaken for an arc (uniformity rejects it).
- A genuine sharp corner never absorbed (angle jump breaks the run).
- Idempotence: re-fitting reproduces.

### Risk
The three tolerances are the make-or-break: too tight under-segments (shatters the ±23% arc), too loose over-segments / admits non-arcs. Tune on the fixture; the visualizer is the primary check.

## Code map

- `geometry/src/fitter/chain.rs` — `detect_runs` + `grow_turning_band` + `grow_cocircular_span` (the Attempt-1 segmenter to replace with the uniformity walk), `reconstruct` (reused once fed clean runs), `circle_fit` / `incircle` (radius extraction).
- `geometry/src/fitter.rs` — `ArcFitConfig { deviation_tol_mm, min_run_facets }`, `with_arc_fit` → CAP-4 reshapes to `(length_tol, angle_tol, epmm_tol, min_run)`; host knob `bridge.rs` / `viz.rs` `arc_fit: Option<(...)>` and `klippy/arc_fit_config.py` `[arc_fit]` parser carry the same tuple.
- `motion-engine/src/seam_harness.rs` — `parse_gcode_to_moves` zeroes E (the harness gap above); must carry per-facet extrusion.
- `motion-engine/tests/arc_fit_neptune.rs` — red acceptance (`max_arc ≥ 8mm`, `blended ≤ 40`); `…/gcode/neptune_crash_short.gcode` fixture.
- `motion-engine/examples/analyze_arc_fit.rs` — segment census + per-facet `len/turn/epmm` dump (`DUMP_FACETS=1`); `repro_plan_stall.rs` — times the real commit path.
