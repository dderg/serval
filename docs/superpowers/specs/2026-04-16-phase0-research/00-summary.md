# Phase 0 Research — Cross-Bucket Synthesis

**Date:** 2026-04-16
**Scope:** prior art, ecosystem history, and integration risks for replacing Klipper/Kalico's `square_corner_velocity` model with real corner blending.
**Input:** five parallel research reports (A–E) in this directory.
**Output:** architectural recommendation and staged roll-out plan for Phase 1 design.

---

## TL;DR

1. **Real corner arc blending is battle-tested industrial prior art.** LinuxCNC's `blendmath.c` (Ellenberg 2014) does exactly what we want, with a single user-facing tolerance parameter and a derivation we can adopt verbatim. Siemens and Fanuc hide equivalent math behind menus of modes; all four industrial controllers converge on **chord deviation in path units** as the user's one knob.
2. **Nobody in the mainstream FDM ecosystem has done this.** Marlin, RRF, Smoothieware, Prusa-Firmware-Buddy, Prusa-Firmware, and upstream Klipper all use some variant of junction-cap-on-zero-duration-turn. The one exception is **Prunt**, an independent new motion controller that ships degree-15 Bézier blends with a single `M205 D<mm>` config. Bambu's 2025 H2C/H2D firmware likely does something similar but is closed-source.
3. **Academic consensus: symmetric cubic Bézier (G² continuity) is best-of-class.** Biarc is the simpler G¹ fallback. Both have closed-form formulas suitable for an O(1) per-corner look-ahead pass. Quintic/PH-quintic is deferred — probably unnecessary once input shaping handles sub-audio resonance.
4. **The "cheap path" is viable.** Bucket E's finding: fine-segmented linear approximation of blend arcs (≈0.2 mm segments for 10 µm chord error at R=0.5 mm) leaves the entire existing C pipeline — `trapq`, `itersolve`, every `kin_*.c`, `stepcompress`, `kin_shaper`, pressure advance — **untouched**. Input shaping regains its line-based mathematical grounding. This converts what looked like a planner rewrite into a planner *preprocessor*.
5. **Piezoid's `work-peraxis_*` branches are the prior-art Klipper baseline.** Kalico already carries the `limited_corexy`/`limited_cartesian` half of his work; we do **not** carry his 5-line cornering patch (`76ba4bee`) that keeps speed constant across curves and fixes Klipper #4228's stutter symptom. **Benchmark this patch first** — if it captures most of the user-visible pain on high-performance hardware, the full rewrite has a much higher bar to clear.

---

## Headlines by bucket

**A — Industrial CNC** → `A-industrial-cnc.md`
LinuxCNC `src/emc/tp/blendmath.c` is the direct model. For a corner with half-angle θ between two segments of lengths L₁, L₂ and tolerance `P` (chord deviation), it inserts a G¹ circular arc of radius `R = tan(θ) · min(P/(1−sin θ), L₁, L₂)` tangent to both segments. Cornering velocity is capped by a centripetal budget (50% of `max_accel` tangential, ~86% normal) plus an S-curve jerk floor `R ≥ v^(3/2)/√j`. Look-ahead walks back up to 50 segments. A mandatory **naive-CAM collinearity collapser** merges short CAM chords before arc fitting — this prepass is *critical* and missing it would break slicer output. Siemens CYCLE832 wraps similar math behind one `TOL` knob because even CNC operators don't want mode selection.

**B — FDM firmware siblings** → `B-fdm-firmware.md`
Everyone except Prunt and Bambu does zero-duration-corner + velocity cap. Marlin's documented bugs #11672/#12491/#16184 (S-curve + JD interaction) are cautionary: **any higher-order motion profile layered on top of a junction-cap corner creates velocity jumps**. Our arc blending must *replace* the junction cap, not coexist with it. Prunt's approach (degree-15 Bézier, `M205 D<mm>`, radius-limited) is the one working FDM reference — worth a focused second-pass read if we go G² or native-arc later.

**C — Academic literature** → `C-academic-literature.md`
Symmetric cubic Bézier corner smoothing (Zhao/Fan/Bi lineage) gives G² continuity with closed-form control points, max-deviation formula, and v_max = f(r, α, a, j). O(1) per corner; real-time-safe on our target hardware. Biarc is the simpler G¹ baseline with trivial error formulas (`ε = d · (1 − sin(α/2)) / cos(α/2)`, `v_max = √(a · r)`) and maps directly onto G2/G3 semantics the printer ecosystem already speaks. **Literature gap:** combined corner smoothing + input shaping derivation has not been published — potential Kalico-original contribution.

**D — Klipper/Kalico ecosystem history** → `D-klipper-ecosystem.md`
Piezoid's 5-line patch is the must-benchmark baseline. No one has ever implemented real arc blending in any Klipper fork (Butyugin floated it in #2030, Oct 2019; never coded). Three maintainer arguments our design must directly address:
- **O'Connor's battle-tested gate (#4228):** needs widespread multi-hardware test results before default switch. → Ship behind a flag; collect data.
- **Butyugin's physical-model objection (#4228):** no heuristic coefficient patches. → Our derivation is LinuxCNC's geometry; every number is a real physical quantity.
- **SCV-bundles-three-concerns (Discourse #7298):** SCV currently also encodes linear-PA minimum flow and input-shaper smoothing contracts, not just kinematics. → Our design must either provide separate knobs for those concerns or prove they fall out of the geometry naturally.

**E — Kalico pipeline pitfalls** → `E-pipeline-pitfalls.md`
Cheap-path finding is the scope-changer. If we commit to fine-segmented linear approximation, the top 3 native-arc risks (shaper convolution on curved paths, coupled multi-move look-ahead redesign, trapq polymorphism across ~12 C files + public ABI) all evaporate — they become *future optimization work*, not v1 dependencies. Step-rate ceiling pressure is real and measurable but bounded; we budget for it and size segment length to fit.

---

## Convergent findings (supported by multiple buckets)

- **One user knob: chord deviation in mm.** A (LinuxCNC, Siemens, Mach), B (Prunt's `M205 D<mm>`), and C (academic papers frame error in chord deviation) all converge. Our parameter should be `corner_deviation` or similar, **not** radius and **not** velocity.
- **Velocity cap is centripetal-budget-based: `v ≤ √(a_n · R)`.** Appears in A (LinuxCNC explicitly splits a_tan/a_normal), C (biarc, Bézier formulas), and is implicit in the Grbl JD formula already. No controversy.
- **S-curve / jerk floor matters.** LinuxCNC imposes `R ≥ v^(3/2)/√j`; academic literature concurs. Kalico has no explicit jerk knob but has input shaping, which sets an effective jerk ceiling. **Open question:** derive the effective jerk limit the shaper imposes, then fold it into the velocity cap.
- **Prepass/collapse of short collinear segments is mandatory.** LinuxCNC's naive-CAM, Prunt's preprocessing, academic papers on real-time feasibility all flag this. Slicer output will otherwise defeat the blend.

---

## Architectural recommendation for Phase 1

**Adopt LinuxCNC's model** for the geometric derivation (G¹ tangent arc, chord-tolerance-driven radius, centripetal-budget velocity cap, naive-CAM prepass, bounded look-ahead), **executed via fine-segmented linear approximation** (Bucket E's cheap path) so the existing `trapq`/shaper/kinematics/stepcompress pipeline runs unchanged. Single user-facing parameter replaces SCV.

This buys us:
- Battle-tested geometric math (preempts maintainer objection 1).
- Clean physical model, every number is real (preempts objection 2).
- Decouples kinematics from linear-PA min-flow and shaper smoothing (addresses objection 3 — those become separate knobs).
- Minimal pipeline risk — no shaper-on-arc validation required, no trapq ABI change, no per-kinematics rewrite.
- Clear upgrade path to G² Bézier and native-arc primitives as future stages if/when measurements demand them.

---

## Staged roll-out plan

**The fork itself is the opt-in gate.** We do not add runtime feature flags or compat toggles inside the fork; old code paths get cleanly replaced. "Chunked, digestible work" is preserved via a reviewable PR stack on a feature branch that merges to the fork's main when the chain is complete. Feature-flag strategy is only reopened if/when we pursue upstream contribution (Kalico main or Klipper).

**Piezoid's 5-line cornering patch (`76ba4bee`) is documented as a known fallback.** We do **not** benchmark it first. It's a symptomatic fix for #4228's circle stutter; it does not address SCV's architectural miscategorization or the shaper-as-structural-element problem, so a "pass" on its benchmark would not change our decision to do the proper rewrite. If Stage 1 stalls or fails unexpectedly, Piezoid's patch is a potential short-term fallback to ship while we regroup.

- **Stage 1 (replacement, ~3–6 weeks, chunked):** Build cheap-path corner blending on a feature branch as a stack of reviewable PRs:
  1. Add `corner_deviation` config parsing (unused).
  2. Implement LinuxCNC-style blend geometry in a standalone module with unit tests (`blendmath.py` or similar).
  3. Implement naive-CAM collinearity prepass.
  4. Wire geometry + prepass into the planner output as fine-segmented blend arcs.
  5. Remove `square_corner_velocity`, `_calc_junction_deviation`, and now-dead code paths.
  6. Update Shake&Tune integration / shaper calibrator's `offset_90` model.
  7. Docs and example configs.

  Merge the feature branch to the fork's main when the chain is complete and validated. No runtime coexistence with SCV.
- **Stage 2 (optional, future) — native-arc primitive:** Native arcs in `trapq` + per-kinematics support, *only if* step-rate ceiling or segment-count pressure becomes measurable. **Pre-design note:** before committing, evaluate measuring resonances at **multiple axis angles** (not just X and Y) to characterize cross-axis coupling. On a true arc, per-axis commanded velocity is sinusoidal with X and Y 90° out of phase; any cross-coupled resonance (gantry sway, belt coupling) not captured by today's per-axis shaper tuning could show up here. Multi-angle IS measurement + a direction-aware shaper is a likely Stage 2 prerequisite.
- **Stage 3 (optional, future) — G² upgrade:** Replace G¹ arcs with symmetric cubic Bézier blends *only if* post-shaper jerk discontinuity is measurable or audible.

Each stage gets its own brainstorm → spec → plan cycle. Stage -1 is small enough to treat as a spike.

---

## Open questions (unresolved in Phase 0)

1. **Effective jerk limit from input shaping.** Derive it analytically; validate empirically. Determines the jerk floor we use in the velocity cap.
2. **Step-rate ceiling budget with fine-segmentation.** Measure at representative small-R / high-v corners on target hardware (user's high-perf Voron-class printer).
3. **G2/G3-from-slicer interaction with corner-blending arcs.** Currently `gcode_arcs.py` segments G2/G3 into G1s before the planner; do our corner blends compose cleanly with that, or do we need to short-circuit and pass radius through?
4. **Linear-PA minimum-flow decoupling.** If SCV goes away, linear-PA loses the minimum-flow contract it bundled. Design a replacement (likely an extruder-local parameter).
5. **Shake&Tune integration.** After SCV is removed, does `find_shaper_max_accel`'s `offset_90` term still make sense, or does it need a reformulation tied to the blend geometry?
6. **Fork vs upstream PR strategy.** Given D's summary of maintainer objections, is the target to merge upstream eventually, or live as a Kalico-only feature? Informs naming, deprecation schedule, and test volume.

---

## Recommended next step

Begin **Stage 1 brainstorming**. Candidate sub-specs within Stage 1 (each could be its own design + implementation plan):
- **Blend geometry module** — pure-math standalone: given `(prev_move, next_move, corner_deviation, accel_limit, shaper_jerk_floor)` return blend arc parameters + fine-segmented polyline + velocity cap. Unit-testable in isolation.
- **Naive-CAM prepass** — slicer-output short-segment collinearity collapser, feeding the blend geometry module.
- **Planner integration** — wiring geometry output into `toolhead.py` / `LookAheadQueue`, emitting fine-segment moves through `trapq`.
- **SCV / JD removal** — delete `_calc_junction_deviation`, `square_corner_velocity` config, related `SET_VELOCITY_LIMIT` args. Now-dead code cleanup.
- **Shake&Tune / shaper_calibrate** — update `offset_90` and `find_shaper_max_accel` to reflect the new kinematic model.
- **Cross-stage decisions** — parameter name (`corner_deviation`? `corner_tolerance`? `blend_deviation`?), config docs, example configs, chunking order for the PR stack.
