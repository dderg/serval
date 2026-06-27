---
stepsCompleted: [1, 2, 3, 4]
session_active: false
workflow_completed: true
inputDocuments: []
session_topic: 'Fitting a clothoid transition between a tangent line and a fixed circular arc with curvature continuity'
session_goals: 'Generate candidate strategies to resolve the over-constrained fit (G0/G1/G2 continuity at both joins vs. a slicer-fixed arc) within a tolerance band'
selected_approach: 'progressive-flow'
techniques_used: ['First Principles Thinking', 'What If Scenarios', 'Constraint Mapping', 'Morphological Analysis', 'Solution Matrix']
ideas_generated: []
context_file: ''
---

# Brainstorming Session Results

**Facilitator:** dderg
**Date:** 2026-06-26

## Session Overview

**Topic:** Fitting a clothoid transition between a tangent straight and a circular arc so curvature changes gradually (line → clothoid → arc → clothoid → line).

**Goals:** Find good strategies for the *fitting* problem — clothoids already exist. The hard part: inserting a clothoid for G2 (curvature) continuity forces the arc to be displaced (spiral shift p ≈ L²/(24R)). With the arc fixed by the slicer, the four continuity constraints (position, tangent, curvature, at both joins) are over-determined. Something must give.

### Session Setup

- Canonical geometry agreed: line → clothoid → arc → clothoid → line.
- Clothoids implemented already; this session is about the constrained fit, not shape selection.
- Constraint trap: cannot achieve G2 continuity at both joins while leaving the commanded arc (center, radius, endpoints) untouched. Candidate release valves: move arc, shrink radius, consume straights, or relax to near-G2 within tolerance.
- Downstream context: curvature drives the centripetal accel limit feeding SOCP/SLP velocity optimization; must plan cheaply on a Pi; must stay within the printer's path tolerance band.

## Phase 1 — First Principles + What-If (ideas)

**Spec confirmed:** True G2 continuity everywhere is a hard requirement (not a jerk budget). Slicer fillets must be smoothed with the same clothoid treatment as line↔line corners — no second-class paths. Arc source is BOTH: sometimes fixed gcode (G2/G3), sometimes our own arc-fit of a polyline.

**[P1 #1] Continuity-is-really-jerk (reframe, demoted to fallback lever)**
_Concept_: G2 discontinuity → step in centripetal accel v²κ → jerk impulse → resonance the shaper must fight + forces accel limiting. So the physical driver is bounded jerk. A residual Δκ costs v²·Δκ; the planner already slows v in tight curves, shrinking the penalty.
_Novelty_: Converts a hard equality into a budget — but user wants true continuity, so this is kept only as a graceful-degradation lever for the degenerate cases, not the primary plan.

**[P1 #2] The shift law — smoothness needs runway (the pivot)**
_Concept_: A clothoid from a line places the arc center at height R+p, p≈L²/(24R), tangent point slides back ≈L/2. A clothoid transition EXISTS only if dist(O,line)=R+p>R. A tangent fillet has dist(O,line)=R exactly → p=0 → L=0 → no clothoid fits. To make room: move circle away from line, or shrink R.
_Novelty_: Reframes "impossible over-constrained fit" as "the fillet has zero runway by construction"; the fix is manufacturing the shift p, and there are exactly two physical levers.

**[P1 #3] The governing tradeoff law: deviation ∝ L²**
_Concept_: p≈L²/(24R). Smoothness (gentleness of κ ramp) scales with L; geometric deviation scales with L². The tolerance tube caps p, hence caps L, hence caps achievable smoothness. The tolerance band IS the smoothness budget.
_Novelty_: Gives a single closed-form knob linking the user's tolerance setting directly to maximum achievable motion smoothness — and quadratic scaling means there are sharply diminishing returns on widening the tube.

**[P1 #4] What-moves is an OUTPUT, not an input (the key reframe)**
_Concept_: Don't pick the lever. Frame the corner as: find the G2 clothoid–arc–clothoid that stays in the tolerance corridor around the commanded line–arc–line while minimizing deviation. What's fixed vs. free falls out of the solve. The optimizer leaves geometry unchanged when it can and moves it minimally when forced.
_Novelty_: Dissolves the agonizing "which thing should move?" decision — it becomes a constrained-fit result. Matches the project's "tightest trajectory we can compute" mandate. User doesn't have to commit to a heuristic.

**[P1 #5] It often doesn't have to change at all**
_Concept_: (a) Non-tangent joins (line meets arc at a real angle / G1 corner) already have runway — insert clothoid–arc–clothoid into the wedge with ZERO arc deviation, identical to rounding a line↔line corner. Likely the common gcode case. (b) Even for tangent fillets, p≈L²/24R is often sub-tolerance (R=10, L=2 → p≈0.017 mm). So "arc changes" frequently means "by less than tolerance."
_Novelty_: Shrinks the genuinely-hard problem to one narrow case — a tangent fillet with zero runway where p exceeds tolerance (tight R and/or long gentle clothoid). Everything else is already easy.

**[P1 #6] Vocabulary of deviation-absorption (the optimizer's move set)**
_Concept_: When deviation is unavoidable, the distinct places to spend it: tighten arc (R↓, center fixed); push arc out (R fixed, center drifts p); slide/offset the straight; split error across line+arc (halves worst case); re-fit the whole corridor discarding the slicer's arc identity.
_Novelty_: Turns vague "move the arc" into an enumerated, optimizable move set — these become the decision variables / fallbacks rather than mutually exclusive philosophies.

## Phase 1 — Scope decision (narrows the problem)

**Deliverable scope:** We OWN the arc-fitting (we fit arcs to the slicer polyline ourselves) — this is the first deliverable. Genuine G2/G3 gcode can be dropped or simply not refit: every modern slicer already fits arcs on top of the mesh, so they're an approximation on the slicer side anyway. Conclusion: we are NOT handed fixed arcs; we generate them, so we control the geometry end-to-end. The "fixed arc with pinned endpoint" hard case (case 1) is descoped for v1.

**Key consequence:** A standard arc-spline / biarc fit is G1 — neighbors meet tangentially, curvature jumps at every junction. So "fit circles first, insert clothoids after" hits the zero-runway (p=0) problem at EVERY junction, not rarely. Fighting our own output. Escape: make G2 a property of the FIT, not a post-process.

## Phase 3 (pulled early) — Concrete fitting recipes [direction (a)]

**[R-Ia] Clothoid-pair fitting — the G2 analog of a biarc**
_Concept_: Biarcs (two arcs matching endpoints+tangents) are the standard G1 arc-spline primitive. Upgrade to a clothoid pair matching endpoints + tangents + CURVATURES at both ends. Fit these to polyline spans instead of circles. G2 by construction; no insertion, no shift to absorb.
_Novelty_: Reuses the mature biarc-fitting machinery (subdivision, tolerance test) but swaps the primitive — G2 falls out for free instead of being retrofitted.

**[R-Ib] Clothoid–arc–clothoid as a single fitting element**
_Concept_: Same as R-Ia but allow a constant-κ arc in the middle for long gentle spans (cheaper traversal, fewer DOF). Short spans collapse to a pure clothoid pair; the arc only materializes when the span is long enough to earn it. One element type spans tiny-corner → long-sweep.
_Novelty_: Unifies line, clothoid, and arc as degenerate cases of one primitive — simplifies the planner's segment vocabulary and the fitter's case analysis.

**[R-IIa] Fit arcs, then global relaxation**
_Concept_: Keep arc-fitting, insert clothoids at junctions, let the shifts propagate through the whole arc chain as one smoothing relaxation. Every arc nudges slightly; spline settles. Because we own ALL arcs, no pinned endpoint fights back; deviation self-distributes.
_Novelty_: Treats the chain as a coupled system rather than independent junctions — a junction's shift is shared by its neighbors instead of dumped on one arc.

**[R-IIb] Arc-fit with deliberate runway bias**
_Concept_: Tell the arc-fitter to leave the gap — fit arcs slightly tighter/spaced so the needed p≈L²/24R already exists between neighbors; the clothoid just fills it. Arc-fit error budget and clothoid-shift budget merge into ONE shared tolerance spent optimally.
_Novelty_: Co-designs the two stages instead of letting stage 2 inherit stage 1's zero-runway mistakes.

**[R-X] Inflections are the cheapest place to be G2 (cross-cutting)**
_Concept_: Two-ended cancellation is strongest at curvature sign changes. Through an inflection a single clothoid sweeps κ from +1/R₁ through 0 to −1/R₂; the two setbacks land on opposite sides and lateral drift balances. Same-sign curvature steps are the expensive junctions.
_Novelty_: Reclassifies the difficulty map — inflections (intuitively scary) are actually free; monotone-curvature steps are where the deviation budget is spent.

## Phase 3 — Prior art synthesis (research agent findings)

Full cited report archived from research agent. Key load-bearing findings:

**[PA #1] Weld-then-fillet dodges the knot-curvature problem entirely**
_Concept_: Biclothoid-fillet-per-corner (Wen & Shpitalni 2018, high-speed CNC): weld polyline → lines+arcs (G1) first, then fillet every corner. Curvature BCs at each fillet are known exactly (0 on line side, 1/R on arc side). No curvature estimation, no coupling, no global solve. O(1)/corner, ~42µs C++, natively arc-length parameterized.
_Novelty_: Eliminates Decision A (Axis 3 knot-curvature dilemma) by construction — the dilemma only exists in the direct-fit architecture.

**[PA #2] Direct clothoid-spline fit has an Achilles heel on OUR input**
_Concept_: McCrae–Singh greedy clothoid fitting estimates knot curvature via Menger/circumradius from 3–5 vertices. Report: "nearly useless" on sub-mm, chord-spaced slicer vertices without heavy pre-filtering (Gaussian/Savitzky-Golay window 5–9). Our dense polyline is the pathological input.
_Novelty_: The most "general" method is the most fragile on exactly our data; argues for weld-then-fillet or a robust pre-filter.

**[PA #3] Clothoid is the correct primitive for the tightness mandate**
_Concept_: PH quintics (Farouki) and quintic Béziers are cheaper with CLOSED-FORM arc length, but curvature is degree-3 polynomial → oscillates (non-monotone). Clothoid curvature is linear → monotone → Euler-optimal fairness → lowest curvature peak → fastest traversal. Oscillating-curvature primitives hand the velocity planner phantom extrema to decelerate for.
_Novelty_: Disqualifies the cheaper alternatives for the main path on throughput grounds, not taste — directly from the project's non-negotiable.

**[PA #4] Fillet sizing = tolerance-first; setback p≈L²/24R, L≈√(24Rδ)**
_Concept_: Two sizing strategies — tolerance-first (fix deviation δ, derive L) vs speed-first (derive L from feedrate/accel). Since velocity planning is downstream/out-of-scope, tolerance-first is correct for us: L≈√(24Rδ), then clamp L to flanking segment lengths.
_Novelty_: Confirms the fitter sizes purely on geometry/tolerance, cleanly decoupled from the (out-of-scope) velocity solver.

**[PA #5] The real hard sub-problem: fillet overlap resolution**
_Concept_: When corners cluster on short segments (flanking straight shorter than runway L≈√(24Rδ)), adjacent fillets collide. Neither canonical paper solves this cleanly. Resolution (shrink both / merge / subdivide) is OUR engineering, and the genuine edge-case engine regardless of architecture.
_Novelty_: Pinpoints where the actual novel work is — not the happy-path fit, but tight-corner-cluster collision handling.

## Phase 4 (emerging) — The architectural fork: robustness vs tightness

**[FORK A] Weld-then-fillet** — Pass1 greedy G1 weld → lines+arcs (Gribov-style); Pass2 biclothoid/clothoid fillet at every junction. Robust, no curvature estimation, streaming, CNC-proven. Con: two-stage error stacking (G1 arc approx error + fillet error) → may not be absolute tightest.

**[FORK B] Direct clothoid-spline fit** — single pass, clothoid segments fit to polyline with estimated/relaxed knot curvatures (McCrae–Singh / Binninger–Sorkine-Hornung local monotone-curvature). Potentially tighter, fewer elements. Con: noisy-curvature problem on slicer input, more complex, needs pre-filtering.

Central tension mirrors the project mandate: A = robust+fast+proven but possibly looser; B = potentially tighter but fragile on our input and more complex.

Library note: Bertolazzi `Clothoids` (C++11, github.com/ebertolazzi/Clothoids) provides ClothoidCurve, BiArc, G1/G2 solvers, distance primitives — reference implementation for whichever fork.

## Scope correction (turn): fitter contract vs solver

"Can compute / trajectory time" is the SOLVER's responsibility, not the fitter's. Fitter contract = emit lines / clothoids / arcs with CONTINUOUS CURVATURE, within tolerance. A G2 path just makes the solver's job easier. Drop all throughput/trajectory-time reasoning from fitter design. Fitter quality bar = {G2, in-band, fair (low/monotone curvature)}. This strengthens the robust weld-then-fillet choice; the "B is a tighter trajectory" argument was never the fitter's concern.

## Phase 4 (diverging) — Fillet overlap resolution, grounded in arc_fit/fillet.gcode

Real data (snapshots/cases/arc_fit/fillet.gcode) shows all three regimes at once:
- Clean long straights (e.g. X98.586 Y126.414→Y118.559, 7.8mm) → stay lines.
- Rounded corner cluster (X120.819,124.426 → ... → X119.934,125.457, 5 short segs ~90°) → welds to an arc + fillets.
- Genuine sharp tip (X122.874,124.81 → X122.795,125.037 → X122.715,124.81, ~0.2mm segs) → runway L≈√(24Rδ) dwarfs the segment. THE overlap case, real not hypothetical.

**[OV #1] G2 ≠ low curvature; it means CONTINUOUS curvature (the reframe)**
_Concept_: A sharp tip is allowed to be a biclothoid with high PEAK curvature, as long as κ ramps up and down continuously. The solver slows for high κ — fitter's job is only to keep κ continuous through the sharpness, not to avoid sharpness.
_Novelty_: Collapses most "overlap failures" into non-problems — the fitter never needs to refuse a sharp corner, only to keep curvature continuous.

**[OV #2] Overlap resolution as a fallback ladder (not a minefield)**
_Concept_: (1) Decimate first — Douglas–Peucker/Gribov drops within-tolerance vertices; most clutter vanishes pre-fillet. (2) Full fillet L=√(24Rδ). (3) Shrink to available runway, ASYMMETRIC L_in≠L_out so each side uses its own runway. (4) Merge when even a segment-scale biclothoid won't fit — the short segment is a sample of a curve, fit ONE biclothoid across the cluster within tolerance. (5) Never drop to G1 — terminal fallback is a tiny high-κ biclothoid, never a curvature step.
_Novelty_: Turns the one genuinely hard sub-problem into a deterministic monotone ladder where the G2 contract holds at every rung by construction.

**[OV #3] Inflection guard rail on merges**
_Concept_: Merges must not cross a turning-sign change. Split clusters at inflections (the cheap G2 points, R-X), merge only within monotone-turning runs — else an S-curve smears into a single wrong-handed arc.
_Novelty_: Makes the merge step (step 4) safe — inflections become the natural, principled cluster boundaries.

**[OV #4] Causal forward-fit — Architecture B WITHOUT curvature estimation (may dissolve the A/B fork)**
_Concept_: Single greedy causal pass. Each new element INHERITS its start curvature from the previous element's end curvature (exact, not estimated); its curvature-rate and length are free, chosen to track the polyline within tolerance. Grow current clothoid until tolerance breaks, close, start next inheriting κ_end. G2 holds at every handoff by construction; NO curvature estimation anywhere — curvature emerges from the fit and propagates forward. Line=κ≡0, arc=κ≡const, clothoid=κ ramp — one element type, selected by what fits. Unifies weld+fillet into one pass. Honors "fitting not solving" (local/causal/greedy).
_Novelty_: Kills B's only real weakness (noisy Menger estimation) by replacing estimation with inheritance — gets B's single-pass directness + A's robustness.

**[OV #4-resolved] No closure problem, no dead-end — the two catches dissolve (turn)**
_Concept_: (1) There is NO closed loop in the stream. A perimeter enters from a travel/seam move and exits to one; the geometric close point (seam) is just another junction with a predecessor and successor, blended by the same causal rule. Even a 360° circle is travel-in → arc → travel-out; entry/exit are different moves, so end-curvature need not rendezvous with start-curvature. (2) "Greedy paints into a corner" dissolves because clothoid curvature-RATE is unbounded (fitter promises G2 = continuous κ, NOT G3 = continuous κ'); an element inheriting high κ_start meeting a straight can always ramp κ→0 over an arbitrarily short length within tolerance. No geometric dead-end; the stream can always continue. Tangent junction → smooth clothoid blend; non-tangent → sharper blend; same rule.
_Novelty_: Removes the ONLY known structural blocker for causal forward-fit (OV#4). The unified single-pass causal fitter now has no identified showstopper.

## Phase 4 (diverging) — The growth / decision rule (heart of the causal fitter)

Per-step state carried forward: end position, end heading, end curvature κ_cur. Element being built = clothoid pinned at (pos, heading, κ_cur) with exactly 2 free DOF: curvature-rate κ′ and length L. Line = (κ′=0, κ_cur=0); arc = (κ′=0, κ_cur≠0); clothoid = (κ′≠0).

**[GR #1] Occam snapping — the anti-wiggle fairness rule**
_Concept_: At each growth step try the simplest primitive that holds tolerance, in order line → arc → clothoid. Line legal only if κ_cur=0; arc only if κ≡κ_cur. Pick simplest that fits window within δ covering most points. Stops noisy slicer data generating a forest of micro-clothoids. Inheritance forces transitions: leaving a curve for a wall, κ_cur≠0 forbids a line → clothoid ramp (κ_cur→0) forced first, then line. arc→clothoid→line emerges.
_Novelty_: A complexity-ordered preference turns "fairness" into a deterministic, cheap per-step decision instead of a curvature-smoothing post-pass.

**[GR #2] Lookahead is NOT zero — reserve a runway before corners (corrects "pure causal")**
_Concept_: Zero-lookahead greedy grows the incoming line all the way to the corner vertex, commits it, then can only round the corner on the EXIT side → cramped asymmetric fillet. To place a symmetric biclothoid you must stop the line one runway-length before the vertex → need bounded lookahead ≈ one runway L≈√(24Rδ) (sub-mm to a few mm). Small fixed window, still streaming, still "fitting not solving."
_Novelty_: Walks back the pure-causal assumption with a concrete failure case; pins the exact (small, bounded) lookahead needed and why.

**[GR #3] No corner detector — just a κ_max threshold**
_Concept_: Corner-vs-curve isn't binary. A corner is where turning demands curvature above κ_max → stop fitting geometry, clamp the runway (OV#5): reserve L, κ_peak=θ/L, emit symmetric biclothoid, G2 automatic. Below κ_max it's a fittable curve.
_Novelty_: Replaces a brittle corner-detection heuristic with one physical threshold that also sets the clamp behavior.

**[GR #4] Fit the curvature SIGNAL, not the 2-D curve (the elegant reformulation)**
_Concept_: Clothoid = linear κ(s), so a fair clothoid spline = piecewise-LINEAR approximation of curvature-vs-arclength (equivalently piecewise-quadratic turning angle θ(s)). 2-D fair-curve fitting collapses to 1-D piecewise-linear segmentation (sliding-window/bottom-up). G2 = κ(s) continuous = segments connect (trivial). Turning angle is an INTEGRAL of noisy data → far more robust than pointwise Menger curvature (dodges B's Achilles heel a 2nd way). Caveat: κ-space tolerance ≠ position-space tolerance — need the mapping or a position-space check on top.
_Novelty_: Reduces the whole fitter to a 1-D signal-segmentation problem with built-in G2 and built-in noise robustness; candidate core algorithm.

**[GR #5] Candidate per-step fit mechanics (to prototype/compare)**
_Concept_: (i) endpoint-greedy: extend window, fit 2-DOF (κ′,L) least-squares, grow until max deviation>δ, commit (McCrae–Singh style). (ii) incremental 2-DOF refit: Gauss-Newton seeded from previous step for cheapness. (iii) κ(s)-signal segmentation (GR#4). All local, all honor "fitting not solving."
_Novelty_: Three concrete, comparable implementations of the same contract — a clean prototype/benchmark matrix rather than one bet.

**[OV #5] At a line→line corner you pick L, not R — the math self-regulates**
_Concept_: Setback L=√(24Rδ) assumes a given R. At a sharp tip (two lines, no arc) there is no R. Symmetric biclothoid through turn angle θ: θ=κ_peak·L, and δ grows with L. Given θ (fixed) and runway L (fixed by segments), κ_peak=θ/L falls out and δ SHRINKS as L shrinks. Clamping L to the runway automatically keeps an isolated corner in-tolerance AND G2; only consequence is higher κ_peak (solver's problem).
_Novelty_: Proves the "merge" fallback is NOT needed for isolated sharp corners — clamp always succeeds. Merge is only for corner CLUSTERING (neighboring corners closer than combined runways).

**[OV #6] Decimation must be arc-aware and shares the tolerance budget**
_Concept_: Plain Douglas–Peucker approximates with lines only; need line+arc decimation (Gribov). Decimate at δ then fillet at δ → worst case 2δ; must split one budget across stages, or measure decimation error against the FINAL filleted curve not the raw polyline. In causal-forward-fit (OV#4) this tension largely vanishes — greedy growth IS the decimation, one budget, one stage.
_Novelty_: Surfaces a silent double-counting of tolerance that a naive two-stage pipeline would violate; and shows the unified pass avoids it.

---

# Convergence — Synthesis & Recommendation

## How the problem dissolved

The session started at "you can't insert a clothoid for G2 without moving the fixed arc — over-constrained." Three reframes collapsed that:

1. **Continuity = continuous curvature, not low curvature.** G2 lets a sharp tip be a high-κ biclothoid; the solver slows for it. The fitter's only job is keeping κ continuous. [OV#1]
2. **We own the fitter and the arcs.** Genuine G2/G3 gcode is descoped (slicers already approximate arcs on the mesh). So nothing is handed to us fixed — the "fixed arc" trap was self-inflicted by fitting circles first and inserting clothoids after. [scope]
3. **Curvature is inherited, not estimated.** A causal forward pass inherits each element's start curvature from its predecessor's end curvature — exact, no Menger estimation — so G2 holds by construction and the noisy-slicer-data failure mode of classic clothoid-spline fitting never occurs. [OV#4]

## Recommended architecture (v1)

**A single-pass, causal, unified fitter.** One element type with 2 free DOF (curvature-rate κ′, length L); line/arc/clothoid are its degenerate cases. Walk the stream; each element inherits κ_start from the previous element's κ_end; grow until tolerance breaks; emit; continue.

Properties that make this the frontrunner:
- Produces canonical `line → clothoid → arc → clothoid → line` as an **emergent** property — no special cases.
- **No curvature estimation** (inheritance replaces it) → robust to dense sub-mm slicer polylines.
- **No closed-loop / rendezvous problem** — perimeters are streams with a predecessor/successor at every junction, including the seam. [OV#4-resolved]
- **No greedy dead-end** — clothoid κ′ is unbounded (G2, not G3, is promised), so the stream can always continue. [OV#4-resolved]
- Honors the **fitter/solver contract**: emit G2 geometry in-tolerance; trajectory time is the solver's concern.
- Lets us **deprecate the two-stage weld-then-fillet** mental model (and possibly the `[arc_fit]` toggle) — fitting and filleting are one rule.

## Design rules for the heart (the growth/decision step)

- **Occam snapping [GR#1]:** try line → arc → clothoid in complexity order; pick the simplest that holds δ. This is the anti-wiggle fairness rule.
- **Bounded lookahead ≈ one runway L≈√(24Rδ) [GR#2]:** required — pure zero-lookahead builds cramped, exit-only corners. Small fixed window; still streaming.
- **κ_max threshold instead of a corner detector [GR#3]:** above κ_max, clamp the runway (κ_peak = θ/L, symmetric biclothoid, G2 automatic); below, fit the curve.
- **Overlap = a fallback ladder [OV#2], not a minefield:** decimate (arc-aware) → full fillet → shrink (asymmetric L_in≠L_out) → merge across cluster → never drop to G1. Merges must not cross an inflection [OV#3]. Clamp alone handles isolated sharp corners; merge is only for corner *clustering* [OV#5].
- **Tolerance budgeting [OV#6]:** a naive decimate-then-fillet double-counts δ (worst case 2δ); the unified pass avoids this since growth *is* decimation.

## The one open fork — settle by prototype + measurement, not argument

| Heart implementation | Pro | Con |
|---|---|---|
| **GR#4 — curvature-signal segmentation** (1-D piecewise-linear fit of κ(s) / piecewise-quadratic θ(s)) | cheap; noise-robust (turning angle integrates out jitter); G2 trivial | controls error in κ-space → needs mapping to position-space tolerance (drift risk) |
| **GR#5 — direct position-space greedy fit** (2-DOF (κ′,L) least-squares per window) | guarantees the deviation tolerance directly | heavier per step; curvature fairness is secondary, must be enforced |

Likely answer: **GR#4 as the segmenter with a GR#5-style position-space deviation check as the guard** — get cheap+robust segmentation, but never trust it past the position band.

## Action plan

1. **Prototype the causal unified fitter** offline (klipper-sim) against `snapshots/cases/arc_fit/{fillet,circle,straight_to_arc_clothoid}.gcode`. Implement both heart variants (GR#4 and GR#5) behind one interface.
2. **Measure** on real gcode: max position deviation vs δ, curvature continuity (κ jumps should be ~0), element count, peak κ at the sharp tip, per-corner compute time.
3. **Validate the canonical shape** — confirm `straight_to_arc_clothoid.gcode` reproduces line→clothoid→arc→line and `circle.gcode` gets entry/exit clothoids at the seam.
4. **Stress overlap** — corner clusters / the sharp tip in `fillet.gcode`; confirm the fallback ladder keeps G2 and stays in-tolerance.
5. **Reference implementation** — lean on Bertolazzi `Clothoids` (C++11) primitives + the G1 solver (arXiv:1305.6644); biclothoid sizing per Wen & Shpitalni 2018; setback p≈L²/24R per Walton & Meek 1992.
6. **Decide the fork** from the measurements; keep the loser as a fallback only if data warrants.

## Out of scope / parked
- Genuine G2/G3 gcode refitting (dropped for v1).
- The velocity solver (separate contract; consumes G2 geometry).
- G3 (curvature-rate) continuity — not required; would only smooth the jerk profile, costs more, deferred.
- Global clothoid-spline optimization (Bertolazzi 2018 global) — tighter but not streaming; offline-only, not v1.

## Key references (from background research)
- Wen & Shpitalni 2018 — biclothoid fillets for high-speed CNC (rank-1 architectural match).
- Bertolazzi & Frego — fast G1 clothoid fitting (arXiv:1305.6644) + `Clothoids` C++ library.
- McCrae & Singh 2008/2013 — greedy piecewise-clothoid fitting (and its Menger-estimation Achilles heel on dense input).
- Binninger & Sorkine-Hornung 2022 — local G2 clothoid spline, monotone curvature.
- Walton & Meek 1992 — clothoid transition spiral, setback formula.
