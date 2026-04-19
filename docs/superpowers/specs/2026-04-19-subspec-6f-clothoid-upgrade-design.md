# Sub-spec 6f — G³ Clothoid Upgrade (Conditional)

**Date:** 2026-04-19
**Status:** deferred — do not start without a triggering condition
**Depends on:** 6d + 6e shipped and hardware-validated
**Unlocks:** the academic state-of-art for corner-smoothing time (Tajima-Sencer lineage)

---

## Trigger condition

**Do not implement this sub-spec unless 6d+6e hardware testing shows measurable ringing at curvature-ramp regions of the quintic blend** that the shaper cannot fully cancel. Concretely:

- Print a test part with sharp-corner geometry (`sharp_short.gcode` pattern or equivalent)
- Measure surface quality with macro photography or a profilometer at the corner entry/exit regions
- If residual ringing amplitude > 0.05mm at the commanded chord-deviation region where κ is ramping, 6f is justified
- If no ringing visible, 6f is a solution to a non-existent problem — stop here

The quintic's curvature is smooth (κ'(t) is bounded) but κ'(t) has a discontinuity at the blend endpoints (κ goes from 0 inside blend to 0 outside, fine — but the *rate of change of* κ steps). That jerk-like event is what a G³ clothoid solves.

---

## What a clothoid provides

**Euler spiral / clothoid:** curvature linear in arc length, `κ(s) = a·s`. Joining two straight lines via two clothoid segments (plus optionally an arc in the middle for very sharp corners) gives:

- G³ continuity (κ' also continuous at endpoints)
- Bounded jerk along the blend (`j ≤ a·v²`, controllable)
- Measurably smoother post-shaper behavior at the 100-Hz-and-above tail of the shaper response

Academic claim (Tajima & Sencer 2016, 2020): **15–25% corner-speed improvement over Zhao-style cubic Bézier at matched tolerance**, and 5–10% improvement over quintic Hermite. On FDM at printer-grade tolerances, expect the lower end of this — maybe 3–7% aggregate print-time improvement over 6d+6e.

---

## Why it's deferred

1. **Fresnel integrals on the hot path.** Clothoid position requires `∫ cos(π t² / 2) dt` and `sin(...) dt`. No closed form. Standard implementation: Bertolazzi & Frego 2015's rational approximation, ~5–7 floating-point operations per sample, ~40 LOC to implement well. On a Pi-class host, this is fine. On an embedded MCU step-generator it's borderline — but our architecture puts blend generation on the Pi side (not MCU), so this is acceptable.

2. **Harder math, more tests.** Clothoid curvature profile is linear in `s`, not `t` — so you have to reparameterize. Arc-length-aware sampling is mandatory (can't rely on `t` uniformity). ~4–5× test LOC vs quintic.

3. **Marginal gain over quintic.** Our 6d+6e data says quintic has 25% post-shaper deviation improvement over arcs. The incremental gain from quintic to clothoid is smaller (5–15% by published estimates). If 6g (inverse-shaper pre-compensation) lands successfully, both quintic and clothoid reach near-perfect post-shaper match — shape differences become dominated by commanded-path smoothness for the inverse filter's conditioning, not post-shaper deviation directly.

4. **Nobody's shipped it in FDM.** Tajima-Sencer's work is in 5-axis CNC where servo dynamics differ substantially from shaper-cancelled resonant FDM systems. Applicability is not automatic.

---

## Minimum scope if triggered

Scope is comparable to 6d + a small fraction of 6e:

```
klippy/blendclothoid.py                (~350 LOC)
  - ClothoidBlend dataclass
  - clothoid_geometry(prev_dir, next_dir, L_prev, L_next, ε, a_max, j_eff)
  - segment_clothoid(c, max_chord_err)       ← requires Fresnel integrals
  - fresnel_C(t), fresnel_S(t)               ← Bertolazzi-Frego rational approx
  - interpolate_extruder_clothoid(...)

klippy/blendplanner.py                  (modify)
  - Extend shape selector: α > some_threshold → clothoid instead of quintic
  - Or: `[blend_arc] preferred_shape = quintic | clothoid` config option

test/test_blendclothoid.py              (~700 LOC)
  - Fresnel numeric accuracy tests
  - Curvature linearity
  - Endpoint G³ continuity
  - Same property sweeps as test_blendquintic
```

**Estimated effort if triggered:** 3–4 weeks. Most of the complexity is in the Fresnel evaluator and its conditioning; the geometric assembly reuses patterns from 6d.

---

## Decision process

Before starting 6f:

1. **Read the hardware evidence.** Is ringing visible in quintic-blended corners on your printer?
2. **If no ringing visible:** stop. 6g is a better use of the same time budget.
3. **If ringing is visible but 6g hasn't shipped yet:** do 6g first — it may eliminate the ringing without needing a curve upgrade.
4. **If ringing persists after 6g:** then 6f is justified. Revisit.

This sub-spec exists primarily as a **placeholder for the decision**, not as an implementation plan. The implementation plan gets written when (if) the trigger condition fires.

---

## Open questions (don't resolve until triggered)

- Is the Bertolazzi-Frego Fresnel approximation accurate enough at single-precision, or does our Pi need double? (Single is typically fine for printer tolerances.)
- Does the clothoid's arc-length parameterization break E-axis coupling in a way that quintic's doesn't? (Likely no — same issue either way, same Gauss-Legendre solution.)
- Should clothoid replace quintic in 6e's selection rule, or sit as a third option behind a config knob? (Depends on how much better it measurably is on real prints.)
