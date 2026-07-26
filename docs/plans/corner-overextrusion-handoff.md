# Corner over-extrusion — investigation handoff

Status: parked (2026-07-25). Theory worked out and simulated; no pipeline code touched.
Branch: `extrusion-at-corners` (off `sota-motion`).

## Problem statement

The nozzle center is what we trace, but flow is commanded per mm of centerline
(`w·h`/mm), which assumes a fresh strip of width `w` ahead. At a corner part of
that strip is already filled by the incoming leg, so the bead overlaps itself.
The sharper the turn, the bigger the overlap. Observed on prints as a small
bulge just **after** every corner (user sees it consistently, even at 90°).

Framing that survived the discussion: there is no goal of producing a sharp
outer corner — the nozzle is round ("glorified glue gun"), outer corners are
rounded no matter what. The only defect to solve is the over-extrusion /
post-corner bulge. The problem is fully 2D: the printer fills the circle
around the nozzle, where it can.

## Established results (all verified numerically, r = w/2)

1. **Miter overlap.** For a sharp vertex with turn angle θ, the double-fed
   inner kite has area `r²·tan(θ/2)`. Diverges toward 180° (a reversal
   re-extrudes the whole line). As equivalent bead length: `(w/4)tan(θ/2)` —
   at w=0.45: 90° → 0.11 mm, 170° → 1.29 mm of extra bead in one spot.

2. **Volume is globally conserved.** The outer miter void is congruent to the
   inner overlap kite (180° rotation about the vertex maps one onto the
   other), at *every* angle. So there is no net over-extrusion; the defect is
   **transport**: material is deposited inside the corner while the void is
   outside, and melt can only leave the nozzle forward.

3. **Reachable vs stranded split.** Of the void, only the circular sector the
   nozzle footprint sweeps at the vertex (`r²·θ/2`) can be filled by squish.
   The rest is a needle no round footprint can enter. Stranded volume:
   `r²(tan(θ/2) − θ/2)·h`. Stranded fraction `1 − (θ/2)/tan(θ/2)`:
   30° → 2 %, 90° → 21 %, 135° → 49 %, 170° → 87 %. This matches the user's
   intuition that "red = empty space" works at 90° but fails toward 180°.

4. **Rounded paths.** An arc of centerline radius R ≥ w/2 deposits exactly
   `w` of area per mm (annulus identity) — zero excess. For R < w/2 the local
   overfeed rate has closed form:
   `ė(s) = (R − w/2)²/(2R) · h`, `R = 1/κ(s)`, zero when κ ≤ 2/w.
   The vertex tan-formula is the R→0 limit. This gives a natural smoothing
   anchor: peak curvature 2/w (R = w/2 = 0.225 mm at 0.45 width); smoothing
   beyond that gains nothing flow-wise.

5. **Bulge mechanism reproduced in sim.** Union-growth deposition model:
   per step the nozzle gets credit only for newly covered area; the deficit
   rides as carried melt/pressure and bleeds forward with decay length λ.
   Produces exactly the print signature: flat bead into the corner, bulge
   starting at the exit, exponential decay (~λ) down the exit leg.
   With λ = 0.8 mm: 170° peaks +50 % width ~1 mm past the vertex; 90° only
   +3 % (geometric term alone — real 90° bulges are larger, so pressure
   dynamics likely stack on top).

6. **E follower is correct under smoothing** (user confirmed): it extrudes at
   the correct rate over the shortened smoothed path. No "mechanism 2";
   path-shortening over-feed does not exist in our pipeline.

## Compensation sketch (not built)

- Post-shaper, where κ(s) of the *executed* path is known: compute ė(s) from
  the closed form above, low-pass with carry length λ shifted after the
  high-κ region (surplus exits forward), subtract from the E rate.
- This is a deliberate trade, not a correction: the stranded volume is
  unplaceable, so removing it trades the bulge for a slightly lighter corner
  (outer corner rounded either way). Must be a no-op when the executed path
  already has κ ≤ 2/w.
- Config needs: filament diameter (already have `[extruder]
  filament_diameter`), plus w and h. Nozzle diameter NOT needed. w·h can be
  inferred per move from gcode: `w·h = (π d_f²/4)·dE/ds`; with h from config
  or z-deltas, w falls out per move (handles Arachne variable width; w is
  squared in the formula so this matters).
- Extruder accel worry is unfounded at these magnitudes: stranded volume at
  90° ≈ 0.9 µm of filament, 170° ≈ 42 µm. Spread over λ at 200 mm/s → peak
  E-rate ~0.6 mm/s filament, an order below normal PA transients. Only
  constraint: correction kernel must be C² smooth (raised cosine) so PA's
  `α·Ë` term stays tame. An impulse at the vertex would be both wrong
  physically and accel-hostile.

## Open questions (blockers for the patch — settle these first)

1. **Does the mechanism exist on the shaped path at all?** Our corner rounding
   comes from the smoothing/shaper convolution, so R_eff ≈ O(v·smooth_time) —
   at 100–300 mm/s this is likely ≫ w/2 = 0.225 mm, making κ_max < 2/w and
   ė ≡ 0 everywhere: the compensation would be a no-op, and the observed
   bulge would be pressure dynamics, not geometry. **First action on resume:
   measure κ_max(s) on a shaped 90°/170° corner at real print speed** (sim
   harness builds shaped tracks). Note the counter-consideration: at low
   corner speeds R_eff shrinks — the term may exist only for slow/sharp
   corners, which is also where blobs are worst.
2. The 90° gap: sim says +3 %, prints show a visible bulge. Either λ is much
   longer than 0.8 mm, or most of the 90° bulge is PA/pressure transient, not
   geometry. Discriminating test (designed, not run): print corners at
   **constant speed through the vertex** — a PA transient needs a speed
   change; the geometric term does not. If the bulge survives constant-speed
   corners, it's geometric.
3. λ is a melt-pressure property (probably PA-time-adjacent, speed-scaled).
   Wrong λ misplaces but conserves the correction — gentle failure. Calibrate
   from bulge decay length on the constant-speed corner print.

## Related work in this repo

- `docs/plans/corner-deviation-budget.md` — makes corner deviation (mm) the
  primary budget with kernel-σ² deduction. Directly adjacent: point 4 above
  gives a *physical* anchor for choosing that budget — the blend radius at
  which flow error vanishes is R = w/2 (κ_peak = 2/w), i.e. the deviation
  budget could be derived from extrusion width instead of guessed.
  Conversely, that plan's kernel-variance machinery is the natural source
  for the executed κ(s) needed by open question 1.
- User's original observation that started this: shallow corners get *more*
  smoothing than they need. A constant-peak-curvature target (κ_peak = 2/w)
  instead of angle-proportional smoothing is the geometric optimum from
  point 4 — but see open question 1: the smoothing kernel may not expose κ
  as a free knob (R_eff scales with v·smooth_time).
