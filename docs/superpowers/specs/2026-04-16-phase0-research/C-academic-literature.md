# C — Academic Literature on Corner Smoothing and Feedrate Transitions

**Scope:** Prior art relevant to replacing Kalico's zero-duration corner model (junction-deviation / square-corner-velocity) with real arc blending of configurable radius. Focus: closed-form error/feedrate bounds, real-time feasibility on embedded hardware, and interaction with input shaping.

**Date:** 2026-04-16

---

## 1. Executive Summary

1. **The CNC community has solved this problem five different ways over ~25 years.** The canonical local corner smoothing pipeline is well-established: detect corner → insert a transition curve (arc, biarc, cubic/quintic Bézier, B-spline, PH-curve, or clothoid) sized by a user *tolerance* ε → cap feedrate at that corner using centripetal acceleration and jerk limits. All the major approaches share the same overall shape; they trade geometric continuity (G1/G2/G3) against compute cost and parameter count.
2. **Closed-form error/speed bounds exist for the simple shapes.** For a single circular arc substituted at angle α with side-length *d*: deviation ε = d·(1 − sin(α/2))/cos(α/2) (equivalently, radius r = d·tan(α/2) when ε is specified), and centripetal-limited speed v_max = √(a_max · r). For cubic Béziers with symmetric control points these relationships become cubic in *d*; Zhao et al. 2013 and Fan–Lee–Ruan 2019 give them explicitly.
3. **Biarc is the cheapest, G1 only.** Cubic/quintic Bézier (G2/G3 with analytic control-point formulas) is the current industrial default for high-speed machining centers — dominant body of literature 2008–2023. PH (Pythagorean-hodograph) curves have the unique property of analytic arc length, which simplifies feedrate integration but at higher math complexity. Clothoids give linear curvature but require Fresnel integrals.
4. **Feedrate planning is a separate, coupled problem.** Erkorkmaz & Altintas (2001) established the jerk-limited 7-phase S-curve reference; modern lookahead interpolators (Tsai, Nien, Yau 2008 onward) combine S-curve planning with per-corner v_max caps computed from the inserted geometry. This is "look-ahead with curvature-limited velocity" — exactly the shape of a 3D-printer planner.
5. **The literature gap relevant to us:** very little academic work combines corner smoothing with input shaping (ZV/MZV/EI convolutional shapers à la Singer–Seering 1990). The interaction is under-explored and is likely a Kalico-specific contribution opportunity. Marlin/Klipper-family work lives largely in forum/blog discussions, not in peer review.

---

## 2. Per-Approach Sections

### 2.1 Biarc Blending (G1)

**What it does.** Replaces the sharp corner with two circular arcs sharing a tangent at their joining point, each tangent to one incoming linear segment. Symmetric biarc = both arcs same radius (most common). Asymmetric biarc allows different radii to fit unequal segment lengths or to match endpoint curvatures when the neighbors are themselves curves.

**Error bound.** For the *symmetric* single-arc case with corner half-angle β = (π − α)/2 and tangent length *d*: deviation ε_max = d · (1 − sin(α/2)) / cos(α/2). Inverting: given ε, d = ε · cos(α/2) / (1 − sin(α/2)); arc radius r = d · tan(α/2). Corner speed v_max = √(a_max · r). True biarcs (two arcs) preserve these relations with slight modification; Meek & Walton (1992) and Bolton (1975) are the canonical references. Error-bounded biarc approximation of general curves: Šír, Feichtinger, Jüttler (2006), *CAD*.

**Compute cost.** Trivial — a few trig evaluations and one sqrt per corner. O(1) per junction, embeddable in any planner look-ahead pass without batch optimization.

**Typical use.** Early NC controllers, legacy G-code post-processors, lithography/plotter control. Also road/rail alignment.

**Recommended papers.**
- Meek, Walton, "Approximation of discrete data by G¹ arc splines," *CAD* 24(6), 1992.
- Šír, Feichtinger, Jüttler, "Approximating curves and their offsets using biarcs and Pythagorean hodograph quintics," *CAD* 38, 2006.
- Piegl, Tiller, "Biarc approximation of NURBS curves," *CAD* 34, 2002. See <https://www.sciencedirect.com/science/article/abs/pii/001044859490099X>.

### 2.2 Cubic Bézier / B-spline Corner Transitions (G2)

**What it does.** Inserts a cubic (4 control points) or higher Bézier/B-spline at each corner with the outer two control points on the incoming/outgoing segments and inner two chosen to enforce curvature continuity. Zhao et al. (2013) introduced the practical "symmetric cubic Bézier with coincident middle control points" pattern that yields analytic max-curvature and max-deviation.

**Error bound (Zhao-style symmetric cubic Bézier).** For symmetric cubic with control-point spacing *d* along each edge at corner angle α: max deviation ε = (d/2) · cos(α/2). Max curvature κ_max (at the midpoint) has a closed form in *d* and α. Corner speed v_max = min(√(a_max / κ_max), limits from jerk). See Fan, Lee, Ruan 2019 ([Chinese J. Mech. Eng.](https://cjme.springeropen.com/articles/10.1186/s10033-019-0360-8)) for explicit formulas.

**Compute cost.** Still O(1) per corner in the analytic-control-point variant; a handful of multiplies. Real-time on 100 MHz ARM is uncontroversial.

**Trade-offs.** G2 buys continuous acceleration (no step) at the cost of segments overlapping the original path — the transition is wider than a biarc for equal ε. B-spline variants (Bi et al. 2012, Sencer et al.) generalize to multi-segment smoothing windows but need more parameters.

**Recommended papers.**
- Bi, Zhao, Chen, Ding, "A general, fast and robust B-spline fitting scheme for micro-line tool path under chord error constraint," *Sci. China Tech. Sci.*, 2019.
- Zhao, Zhu, Ding, "A real-time look-ahead interpolation methodology with curvature-continuous B-spline transition scheme for CNC machining of short line segments," *Int. J. Mach. Tools Manuf.*, 2013. <https://www.sciencedirect.com/science/article/abs/pii/S0890695512001885>
- Fan, Lee, Ruan, "An Optimal Feed Interpolator Based on G² Continuous Bézier Curves for High-Speed Machining of Linear Tool Path," *Chinese J. Mech. Eng.* 32, 2019. DOI:10.1186/s10033-019-0360-8.
- Yutkowitz, Chester, US Patent 6,922,606 (2005) — Siemens' commercial implementation using quartic splines.

### 2.3 Quintic Bézier / PH / Polynomial Blends (G3 and higher)

**What it does.** Quintic polynomials (6 control points) give two additional degrees of freedom beyond cubic, enough to enforce curvature-derivative continuity (G3 / C³ in arc-length). Removes the jerk step at the transition boundary, which matters when jerk excites mechanical resonances. Pythagorean-hodograph (PH) quintics are a special family where arc length is a polynomial in the parameter — huge advantage for real-time feedrate integration.

**Error bound.** Farouki (2012–2016) gives closed-form deviation and extremum-curvature for PH quintic G² corners as functions of corner angle α and side-length *d*. For symmetric quintic Bézier C³: similar structure to cubic but with additional terms ensuring κ′ = 0 at both ends; see Sencer, Ishizaki, Shamoto 2015.

**Compute cost.** O(1) per corner but with substantially more math than cubic (~3–5× the arithmetic). PH curves add Horner-scheme polynomial evaluation for the arc-length integral — still cheap, but non-trivial to implement correctly.

**When worth it.** When the drive system has strong resonances that get excited by acceleration steps (jerk impulses). For 3D printing, this overlaps heavily with what input shaping already addresses, which is why G3+ corner smoothing is less obviously valuable for Kalico than for 5-axis machining centers.

**Recommended papers.**
- Erkorkmaz, Altintas, "High speed CNC system design. Part I: Jerk limited trajectory generation and quintic spline interpolation," *Int. J. Mach. Tools Manuf.* 41, 2001. <https://www.sciencedirect.com/science/article/abs/pii/S0890695501000025>
- Farouki, Manni, Sestini et al., "Real-time CNC interpolators for Pythagorean-hodograph curves," *CAGD* 13, 1996.
- Farouki, "Construction of G² rounded corners with Pythagorean-hodograph curves," 2014 <https://escholarship.org/uc/item/6fq8n655>.
- Sencer, Ishizaki, Shamoto, "A curvature optimal sharp corner smoothing algorithm for high-speed feed motion generation of NC systems along linear tool paths," *Int. J. Adv. Manuf. Tech.* 76, 2015. DOI:10.1007/s00170-014-6386-2.
- Tajima, Sencer, "Kinematic corner smoothing for high-speed machine tools," *Int. J. Mach. Tools Manuf.* 108, 2016.

### 2.4 Clothoid / Euler-Spiral Blends

**What it does.** Clothoid segments have curvature linear in arc length (κ(s) = a·s), so joining two straight lines via two clothoids gives continuously varying curvature between 0 and 1/r_min. Makes physical sense: constant rate of curvature change = bounded jerk at constant speed.

**Error bound.** No simple closed form — requires Fresnel integrals C(t), S(t). Numerical evaluation via rational approximation (Bertolazzi–Frego 2015, "The clothoid computation: a simple and efficient numerical algorithm") is fast but non-trivial on embedded. Walton–Meek (2009) analyze G²-clothoid corner blends.

**Typical use.** Roadway/railway design, wheeled-robot path planning. Some modern CNC work: "A newly developed corner smoothing methodology based on clothoid splines for high-speed machine tools" (Sencer group, 2020).

**Compute cost.** Fresnel evaluation per point — acceptable on RPi host, borderline on MCU-side hot path.

### 2.5 S-Curve (Jerk-Limited) Feedrate Profiles Through Transitions

**What it does.** Orthogonal to the geometry question: given a blend shape with a v_max constraint at its center, compute a velocity-vs-time profile that respects a_max and j_max over the full trajectory. The canonical reference is Erkorkmaz–Altintas 7-phase trapezoidal-jerk profile. Look-ahead planners (Chen, Ji, Tao 2013; Tsai–Nien–Yau 2008) extend this to multi-block scheduling with corner-velocity caps from the geometry step.

**Error bound / formula.** Not a geometric error bound; instead feedrate is chosen to satisfy *simultaneously*:
- chord error: ε_chord ≈ T_s² · v² · κ / 8 ≤ ε_tol
- centripetal: κ · v² ≤ a_n_max
- tangential: a_t ≤ a_t_max
- jerk: |j_t|, |j_n| ≤ j_max

**Compute cost.** Per-block O(1) velocity computation; look-ahead window typically 10–100 blocks (Klipper uses ~64 move look-ahead). Readily real-time on RPi-class.

**Recommended papers.**
- Erkorkmaz, Altintas 2001 (above).
- Tsai, Nien, Yau, "Development of an integrated look-ahead dynamics-based NURBS interpolator for high-precision machinery," *CAD* 40, 2008. DOI:10.1016/j.cad.2007.11.006
- Chen, Ji, Tao, Wei, "Look-Ahead Algorithm with Whole S-Curve Acceleration and Deceleration," *J. Appl. Math.*, 2013. <https://journals.sagepub.com/doi/10.1155/2013/974152>
- Dong, Ferreira, Stori, "Feed-rate optimization with jerk constraints for generating minimum-time trajectories along predefined tool paths," *Int. J. Mach. Tools Manuf.* 47, 2007.

### 2.6 Real-Time Constraint Satisfaction / Error-Constrained Blending

Two architectures dominate:

- **Local / incremental corner smoothing.** Each corner treated independently, one look-ahead pass. Closed-form control points; analytic v_max. O(1) per corner. This is what Klipper's current junction-deviation already does at the feedrate level, but without geometric insertion. Papers: Bi 2012, Zhao 2013, Fan 2019, Tajima–Sencer 2016.
- **Global / batch optimization.** Treat the whole path as a single optimization (commonly convex in v², solved via LP/SOCP) with curvature, acceleration, jerk constraints. Papers: Dong–Ferreira–Stori 2007, Sencer–Altintas–Croft 2008. Impractical on MCU; possible on RPi for offline slicing, not for real-time streaming.

**Error-constrained formulation (directly relevant to Kalico):** user specifies tolerance ε; solver maximizes v at each corner subject to geometric deviation ≤ ε and dynamic constraints. Canonical treatment: Farouki–Manni 2016, "Efficient high-speed cornering motions based on continuously-variable feedrates" Parts I & II, *Int. J. Adv. Manuf. Tech.* 85–86. <https://escholarship.org/content/qt2v5420wr/qt2v5420wr_noSplash_83debb6f7d7e4d7d1c0c25bfad61a6d3.pdf>

### 2.7 Surveys

- Wang, Liu, Zhao, "Corner smoothing for CNC machining of linear tool path: A review," *JAMST* 2023. <http://www.jamstjournal.com/en/article/doi/10.51393/j.jamst.2023001> — the single best recent survey covering all five families above, with comparison tables.
- Fan, Xu, Liu, Zhang, "Toolpath Interpolation and Smoothing for CNC Machining of Freeform Surfaces: A Review," *Machine Intelligence Research* 16, 2019. DOI:10.1007/s11633-019-1190-y.
- Ravankar et al., "Path Smoothing Techniques in Robot Navigation: State-of-the-Art, Current and Future Challenges," *Sensors* 18, 2018. <https://pmc.ncbi.nlm.nih.gov/articles/PMC6165411/> — robotics angle, covers clothoids.

---

## 3. Key Insights for Kalico

1. **The user-facing parameter should stay as a tolerance ε in mm, not a radius.** All serious CNC literature parameterizes on tolerance; the geometry-to-radius mapping is closed-form per angle. This lets the same config produce tight corners on fine features and sweeping corners on long walls automatically.
2. **Prioritize symmetric cubic Bézier (Zhao-style) as the default.** Gives G² continuity (continuous acceleration → no jerk spike), has closed-form control points, deviation, max-curvature, and v_max given (ε, α, a_max). O(1) per corner in the look-ahead pass. This is the 2010s industrial consensus for a reason.
3. **Also implement biarc as a fallback / debug mode.** Biarc is the simplest baseline, matches arc-based GCode (G2/G3) semantics printers already understand, and its error formulas are trivial to verify. Useful for A/B testing correctness and for very constrained MCU execution paths.
4. **Defer quintic / PH / clothoid.** Input shaping already addresses the resonance issues that G³+ smoothing targets at CNC scale. Adding G³ geometry on top of a ZV/MZV shaper is theoretically interesting but unlikely to produce visible print-quality gains proportionate to code complexity.
5. **Do the geometry insertion on the RPi-side planner, not the MCU.** Blend curve generation is a look-ahead-pass concern. The MCU's stepcompress path only needs to consume the resulting (straight, arc) segment stream, which it already does for G2/G3.
6. **Compute v_max from v² = a_max · r_min at the corner.** This matches Klipper's existing square_corner_velocity intuition (which is secretly v² = 2·a·ε_jd) but replaces the virtual arc with a real one. Consistent with existing user mental model.
7. **Explicitly consider the shaped-vs-unshaped interaction.** When input shaping is active, the *commanded* path differs from the *physical* path by a convolution-with-impulses. A corner in the commanded path becomes a smeared corner physically. Aggressive geometric corner smoothing on top of ZV shaping may over-smooth. Literature gap: check whether anyone has published on combined shaping + corner smoothing. This is a Kalico-original contribution opportunity.
8. **Do not over-design for minimum-segment (micro-line) pathologies.** Those are a big deal in 5-axis machining of freeform surfaces (short-segment literature). For FDM slicer output, segments are usually long enough that overlap-elimination papers (Han et al. 2024) are overkill.
9. **Look-ahead interactions:** the inserted blend curve consumes length from the adjacent straight segments. If adjacent segments are shorter than required blend tangent-length, the planner must either asymmetrize (asymmetric biarc/Bézier literature, Sencer 2015) or shrink ε adaptively. Handle this explicitly.
10. **Validation strategy:** replay known test corner angles 15°/45°/90°/135° with known ε and compare measured deviation against closed-form prediction; this is standard practice in the Bi/Zhao papers and directly transferable.

### Prioritization recommendation

**Priority 1 — Symmetric cubic Bézier corner smoothing (Zhao/Fan style)** with closed-form control points and analytic v_max. Best modern cost/benefit, G² continuity, O(1) per corner. Drop-in replacement for junction-deviation while keeping the same user mental model (tolerance parameter).

**Priority 2 — Biarc fallback** for validation, for cases where the cubic Bézier transition would overlap (short adjacent segments), and for direct compatibility with G2/G3 G-code semantics.

**Priority 3 (later, optional) — PH-quintic** only if we encounter a resonance class that input shaping can't fix and that cubic Bézier's acceleration-continuity can't. Unlikely to be the critical path.

---

## 4. Recommended Reading List

1. Wang et al., "Corner smoothing for CNC machining of linear tool path: A review," *JAMST*, 2023. DOI:10.51393/j.jamst.2023001. <http://www.jamstjournal.com/en/article/doi/10.51393/j.jamst.2023001>
2. Erkorkmaz, Altintas, "High speed CNC system design. Part I: Jerk limited trajectory generation and quintic spline interpolation," *IJMTM* 41, 2001. <https://www.sciencedirect.com/science/article/abs/pii/S0890695501000025>
3. Fan, Lee, Ruan, "An Optimal Feed Interpolator Based on G² Continuous Bézier Curves for High-Speed Machining of Linear Tool Path," *Chinese J. Mech. Eng.* 32, 2019. DOI:10.1186/s10033-019-0360-8. <https://cjme.springeropen.com/articles/10.1186/s10033-019-0360-8>
4. Zhao, Zhu, Ding, "A real-time look-ahead interpolation methodology with curvature-continuous B-spline transition scheme for CNC machining of short line segments," *IJMTM* 65, 2013. <https://www.sciencedirect.com/science/article/abs/pii/S0890695512001885>
5. Bi, Wang, Zhu, Ding, "A practical continuous-curvature Bézier transition algorithm for high-speed machining of linear tool path," in *ICIRA 2011*. <https://www.researchgate.net/publication/221105323>
6. Sencer, Ishizaki, Shamoto, "A curvature optimal sharp corner smoothing algorithm for high-speed feed motion generation of NC systems along linear tool paths," *Int. J. Adv. Manuf. Tech.* 76, 2015. DOI:10.1007/s00170-014-6386-2. <https://link.springer.com/article/10.1007/s00170-014-6386-2>
7. Tajima, Sencer, "Kinematic corner smoothing for high-speed machine tools," *IJMTM* 108, 2016. <https://www.sciencedirect.com/science/article/abs/pii/S0890695516300608>
8. Beudaert, Lavernhe, Tournier, "5-axis local corner rounding of linear tool path discontinuities," *IJMTM* 73, 2013. <https://hal.science/hal-00843635>
9. Farouki, Manni, Sestini et al., "Efficient high-speed cornering motions based on continuously-variable feedrates, I & II," *Int. J. Adv. Manuf. Tech.* 85–86, 2016. DOI:10.1007/s00170-016-8740-z and 10.1007/s00170-016-8741-y. <https://escholarship.org/content/qt2v5420wr/qt2v5420wr_noSplash_83debb6f7d7e4d7d1c0c25bfad61a6d3.pdf>
10. Tsai, Nien, Yau, "Development of an integrated look-ahead dynamics-based NURBS interpolator for high-precision machinery," *CAD* 40, 2008. DOI:10.1016/j.cad.2007.11.006
11. Meek, Walton, "Approximation of discrete data by G¹ arc splines," *CAD* 24, 1992 — biarc foundations.
12. Šír, Feichtinger, Jüttler, "Approximating curves and their offsets using biarcs and Pythagorean hodograph quintics," *CAD* 38, 2006. <https://www.researchgate.net/publication/220583713>
13. Sencer, Altintas, Croft, "Feed optimization for five-axis CNC machine tools with drive constraints," *IJMTM* 48, 2008.
14. Chen, Ji, Tao, Wei, "Look-Ahead Algorithm with Whole S-Curve Acceleration and Deceleration," *J. Appl. Math.*, 2013. <https://journals.sagepub.com/doi/10.1155/2013/974152>
15. Singer, Seering, "Preshaping Command Inputs to Reduce System Vibration," *ASME J. Dyn. Sys.*, 1990 — foundational input shaping. <https://code.eng.buffalo.edu/tdf/papers/acc_tut.pdf> (tutorial covering lineage)
16. Ravankar et al., "Path Smoothing Techniques in Robot Navigation: State-of-the-Art, Current and Future Challenges," *Sensors* 18, 2018. <https://pmc.ncbi.nlm.nih.gov/articles/PMC6165411/>

---

## 5. References (inline URLs)

- Survey: <http://www.jamstjournal.com/en/article/doi/10.51393/j.jamst.2023001>
- Fan/Lee/Ruan Bézier G²: <https://cjme.springeropen.com/articles/10.1186/s10033-019-0360-8>
- Erkorkmaz/Altintas quintic + S-curve: <https://www.sciencedirect.com/science/article/abs/pii/S0890695501000025>
- Zhao B-spline look-ahead: <https://www.sciencedirect.com/science/article/abs/pii/S0890695512001885>
- Sencer curvature-optimal corner: <https://link.springer.com/article/10.1007/s00170-014-6386-2>
- Tajima/Sencer kinematic corner smoothing: <https://www.sciencedirect.com/science/article/abs/pii/S0890695516300608>
- Beudaert 5-axis corner rounding: <https://hal.science/hal-00843635>
- Farouki cornering I/II: <https://escholarship.org/content/qt2v5420wr/qt2v5420wr_noSplash_83debb6f7d7e4d7d1c0c25bfad61a6d3.pdf>
- Farouki PH G² corners: <https://escholarship.org/uc/item/6fq8n655>
- Chen S-curve look-ahead: <https://journals.sagepub.com/doi/10.1155/2013/974152>
- Real-time local optimal Bézier (Jin et al. 2021 IEEE Access): <https://ieeexplore.ieee.org/document/9590511/>
- Path smoothing in robot navigation (survey): <https://pmc.ncbi.nlm.nih.gov/articles/PMC6165411/>
- Freeform-surface toolpath review (Fan 2019 MIR): <https://link.springer.com/article/10.1007/s11633-019-1190-y>
- Klipper junction deviation context: <https://www.klipper3d.org/Kinematics.html>, <https://github.com/Klipper3d/klipper/issues/468>
- Input shaping tutorial (Singhose/Seering lineage): <https://code.eng.buffalo.edu/tdf/papers/acc_tut.pdf>
