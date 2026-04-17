# Industrial CNC Corner Blending — Prior Art (Phase 0)

## 1. Executive summary

- **LinuxCNC inserts a literal circular ("spherical") blend arc** sized from a chord-deviation tolerance `P`. Tangent-continuous (G1) only — curvature steps 0→1/R at the entry, but `v = √(a_n_max · R)` bounds the lateral acceleration step. Math lives in `blendmath.c::blendComputeParameters`.
- **"Tolerance" is universally chord deviation** (max perpendicular distance from programmed corner). LinuxCNC `P`, Siemens `CTOL` / `SD42465`, and Fanuc P1730 (via `G05.1 Q1 Rx`) all resolve to "how far may the real path deviate, in path units."
- **Look-ahead is mandatory.** LinuxCNC defaults 50 segments; Fanuc markets multi-block; Siemens activates Look Ahead implicitly with G64. Blend radius *caps* cornering velocity; look-ahead decelerates upstream to reach that cap.
- **Continuity varies**: LinuxCNC and Mach3/4 are tangent-only; Siemens G643 (axis-individual) and G645 (tangential-transition) are curvature-smoothed; COMPCAD/COMPCURV fits polynomial splines over linear clusters and implicitly activates G642.
- **Dominant failure mode on every platform: short segments and cascading corners.** LinuxCNC's naive-CAM simplifier and Siemens's COMPCAD compressor exist because naive arc fitting over 100 µm CAM chords cannot produce usable cornering velocities.

---

## 2. LinuxCNC — `G64 P<tol> Q<naive>`

Primary sources (clone of `github.com/LinuxCNC/linuxcnc` @ HEAD, 2026-04-16): `src/emc/tp/{blendmath.c (1871 LOC, Ellenberg 2014), blendmath.h, spherical_arc.c/.h, tp.c (4386 LOC), tc_types.h}` and `src/emc/task/emccanon.cc`.

### 2.1 Primitive: "spherical blend arc"

LinuxCNC fits an **arc of a circle** tangent to both adjacent segments. The "spherical" label comes from `spherical_arc.c` implementing it via **SLERP** of the two center-to-endpoint radius vectors — this scales to higher dimensions and supports a tiny Archimedean-spiral `spiral = (radius1 − radius0)` term to absorb numerical radius mismatch (`spherical_arc.c:70-81`). Binormal `n̂ = û₁ × û₂` gives the plane (`blendmath.c:1117-1128`). Four variants exist (`blendmath.h:25-31`: `BLEND_LINE_LINE`, `BLEND_LINE_ARC`, `BLEND_ARC_LINE`, `BLEND_ARC_ARC`), all reducing to arc-in-bisector with endpoints pulled back by `d_plan = R / tan θ`.

**Continuity is G1 (tangent), not G2.** Curvature steps from 0 to 1/R at the entry. LinuxCNC compensates by capping `v_plan` so centripetal accel `v²/R` stays inside ~87 % of the acceleration budget, reserving ~50 % for tangential accel (`blendmath.h:21-23`: `BLEND_ACC_RATIO_TANGENTIAL = 0.5`, normal = √(1−0.25) ≈ 0.866). The S-curve branch additionally enforces `R_jerk_min = v^(3/2) / √j_max` (`blendmath.c:1181-1190`) — the minimum radius such that the curvature step does not produce an unbounded discrete-time jerk.

### 2.2 Tolerance semantics (`P` parameter)

Canonical derivation (`blendmath.c:1137-1210`): `h_tol = P/(1−sin θ)`, `d_tol = cos θ · h_tol`, `R_geom = tan θ · d_geom` where `d_geom = min(d_tol, L1, L2)`. `θ` is half the supplement of the corner angle (`findIntersectionAngle`, L397-417), so `θ→0` for a U-turn, `θ→π/2` for colinear. `P` is literally chord deviation: *"the actual path will be no more than P away from the programmed endpoint"* (LinuxCNC G-code docs). Final planned radius `R_plan = max(R_accel, R_blend, R_jerk_min)` clipped by `R_geom` — the smaller of what tolerance allows and what segment length allows. Short segments silently shrink radius below what `P` would otherwise permit.

### 2.3 `Q` (naive CAM detector)

`Q` is a *separate* collinearity consolidator in the G-code interpreter, not the TP. `emccanon.cc::linkable()` (lines 1141-1175) tests whether each buffered point's perpendicular distance from the line between endpoint and new point is `< naivecamTolerance`; if so, up to 100 linear moves merge into one canonical segment. Runs *before* arc blending, XYZ-only, and is the documented fix for CAM posts emitting 10 000+ tiny colinear G1s.

### 2.4 Look-ahead

`tpRunOptimization` (`tp.c:1820`) is a "rising-tide" backward pass: walks back through up to `ARC_BLEND_OPTIMIZATION_DEPTH + 2` segments (default 50), computing `vs_back = √(finalvel² + 2aL)` (or S-curve equivalent) clipped by each segment's `kink_vel`. Stops at the *second* non-tangent segment (`tp.c:1855-1865`). Terminal conditions (`tc_types.h:26-29`): `STOP` (G61), `EXACT` (G61.1), `PARABOLIC` (G64 Q-only fallback), `TANGENT` (G64 P>0 arc-blend path). `ARC_BLEND_GAP_CYCLES = 4`: arcs absorb pre-corner segments shorter than 4 servo cycles (`blendCheckConsume`). `ARC_BLEND_RAMP_FREQ = 100 Hz`: segments shorter than 10 ms use constant-accel ramping, reducing jerk excitation from CAM chord noise.

### 2.5 Documented trade-offs and failure modes

- `ARC_BLEND_FALLBACK_ENABLE = 0` by default; INI docs explicitly state the fallback speed estimate "is rough, and it seems that just disabling it gives better performance."
- Short segments collapse radius (`R_geom = tan θ · min(L1, L2)`) → cornering velocity collapses. Same failure mode Kalico will hit.
- Cascading corners: look-ahead gives up after two non-tangent blocks.
- Rotary-axis motion (`tcRotaryMotionCheck`, `tp.c:1714`) disables arc blending entirely → parabolic fallback.
- `theta_tan` near 0 or π/2 (`blendmath.c:614-622`) aborts blend (near-U-turn and near-tangent both special-cased).
- `ARC_MIN_RADIUS` / `ARC_MIN_ANGLE` reject degenerate arcs (`spherical_arc.c:50-77`).

---

## 3. Fanuc — HPCC / AICC / AIAPC / HSSS

Vendor docs are deliberately thin on primitives; trust-but-verify applies.

- **HPCC** (historical, RISC co-processor) did "multi-block look-ahead acceleration/deceleration" with **bell-shaped accel/decel curves** and "interpolation for smooth curves" (A-78395E summary). Primitive not publicly disclosed; academic reverse-engineering describes polynomial/spline fitting — functionally similar to Siemens COMPCURV.
- **AICC / AIAPC** superseded HPCC on modern CPUs. Activated by `G05.1 Q1 Rx` (AICC) or `G08 P1` (AIAPC). **HSSS (Nano Smoothing)** fits NURBS/smoothed splines to CAM-emitted short segments.

**Tolerance:** `G05.1 Q1 Rx`, `x ∈ 1..10`, selects one of 10 preset profiles that write parameter **1730** (acceleration-before-interpolation) to values `{8400, 8016, 7616, 7192, 6742, 6260, 5738, 5162, 4514, 3756}`. Vendor-published example: "Using R1 (speed priority), a 90° corner at 10 000 mm/min produces an approximate deviation of 0.15 mm" — so the user knob is an acceleration, but the effect manifests as chord deviation. AICC must be engaged *before* G43 tool-length-comp and re-engaged per tool.

**Look-ahead:** marketed but buffer size not published. Community reports ~200 blocks on HPCC, 1000+ on 30i/31i AICC.

**Failure modes:** tool-number-dependent tuning (same program may need different `Rx` per tool); short-segment feed drop is ubiquitous on forums — Fanuc's answer is **Smooth Tolerance Control / Nano Smoothing**, a spline fit over short-segment clusters.

---

## 4. Siemens Sinumerik — G641/G642/G643/G644/G645, COMPCAD/COMPCURV, CYCLE832

Primary sources: SINUMERIK 840D sl Programming Manual §12.2, Function Manual B1 §G641 and §G642/G643, Operating Manual §CYCLE832 (URLs in References).

### Primitives per mode

| Mode | Primitive | Continuity | Tolerance semantic |
|---|---|---|---|
| G60  / G9 | Exact stop | n/a | zero |
| G64  | Look-ahead feedrate only, no corner rounding geometry | G0 (kink) | — |
| G641 | Path-criterion rounding; ADIS = rounding radius, ADISPOS = rapid variant | G1 (tangent) | ADIS (length) |
| G642 | Defined-tolerance rounding, path-wise; radius chosen so **smallest axis tolerance governs** | G1/G2 depending on option | SD42465 `$SC_SMOOTH_CONTUR_TOL` (chord deviation) |
| G643 | Axis-individual rounding, block-internal (no separate rounding block); different tolerances per axis and for orientation | G1 (tangent) per axis | SD42465 + SD42466 orientation tol |
| G644 | Rounding with greatest-possible dynamic response; deactivates G642/G643 tolerance | G1 | machine dynamics |
| G645 | "Tangential-transition rounding" — smooths also tangent transitions that already have curvature steps | G2 (curvature) | CTOL / OTOL |

From manualslib page 192 (G642 vs G643), direct quote: *"The rounding path is determined on the basis of the shortest distance for rounding all axes"* (G642) vs *"Each axis may have a different rounding path. The rounding travels are taken into account axis-specifically and block-internally (⇒ no separate rounding block)"* (G643).

### NC-block compressor (COMPON/COMPCURV/COMPCAD)

Requires the polynomial-interpolation option. Fits a polynomial — COMPON = C¹, COMPCURV = C², COMPCAD = CAD-optimized C² with surface-quality weighting — over clusters of short G1 blocks. Per Siemens support: *"When using NC block compressor with COMPCAD or COMPCURV, G642 is always firmly selected."* Compressor replaces segments; G642 rounds whatever corners remain. Direct analogue of LinuxCNC's naive-CAM detector, but higher-order.

### CYCLE832 & look-ahead

`CYCLE832(TOL, _TOLM, _TOLM_ORI)`: `TOL` is chord deviation in path units, `_TOLM` selects roughing/semi-finish/finish/deselect; the cycle picks the G64/G642/G645/COMPCAD combo and writes `SD42465`/`SD42466`. Typical tables: 0.05 mm roughing, 0.005 mm finishing. Look-ahead (B1) activates implicitly with any G64 variant; 840D sl documents 100+ blocks. Explicit trade-off: G644 = dynamics (fastest/loosest), G643 = axial accuracy, G645 = surface finish.

---

## 5. Mach3 / Mach4 — Constant Velocity (CV)

Mach3/4 does **not** insert a blend arc. "CV mode" means the planner does not decelerate to zero at block boundaries — it overlaps the decel of segment N with the accel of segment N+1; the cut shape is determined by axis dynamics, not by a geometric primitive. This is **parabolic blending**, i.e. essentially what Kalico does today via `square_corner_velocity` / junction deviation.

User-facing parameters:

- **CV Dist Tolerance** — "look-ahead window"; segments together shorter than this get merged. Recommended 0.01–0.03 for sharp corners, 0.5–2.0 for rounded. This is *not* a chord-deviation tolerance — actual deviation depends on acceleration.
- **Stop CV on angles above X°** — exact-stop threshold; sharper corners bypass CV.
- **CV Feedrate** — optional cornering-velocity override.

MachMotion explicitly concedes the cut shape "will differ between machines and programs, requiring adjustments and testing." The CV deviation is not bounded by a user-specifiable geometric tolerance — which is exactly the complaint Kalico users have about junction deviation.

---

## 6. Key insights for Kalico

1. **Tolerance = chord deviation in path units.** Every serious controller normalizes on this. Don't invent acceleration-indexed tolerances (Fanuc) — that's the confusion CYCLE832 exists to paper over.
2. **Tangent-continuous (G1) circular arcs are sufficient.** LinuxCNC ships G1 arcs and handles the curvature step by clipping to `v = √(a_n_max · R)`. G2 (Siemens G645, biarcs) matters only for visible-finish milling; printers don't need it.
3. **Adopt LinuxCNC's formula verbatim** (`blendmath.c:1141-1151`): with `θ = (π − corner_angle)/2`, `h_tol = P/(1−sin θ)`, `d_tol = cos θ · h_tol`, `R_geom = tan θ · min(d_tol, L1, L2)`.
4. **Radius clipped by segment length on both sides**, not just tolerance: `R ≤ tan θ · min(L1, L2)`. Tiny segments collapse R → velocity, which the look-ahead must see.
5. **Naive-CAM pre-pass is table stakes.** 10–100 µm CAM chords will dominate the cornering-velocity limit if you don't consolidate colinear segments first (LinuxCNC `Q` / Siemens COMPCAD).
6. **Look-ahead depth ~50 is enough** if optimization walks back only to the second non-tangent corner (LinuxCNC default). An order of magnitude less than Fanuc marketing claims but empirically sufficient.
7. **Reserve ~50 % of accel budget tangential, ~85 % normal** (LinuxCNC `BLEND_ACC_RATIO_*`). With S-curve/input-shaper model, additionally enforce `R ≥ v^(3/2)/√j_max` so the G1 curvature step doesn't produce a jerk spike that input shapers must smear.
8. **Bail gracefully on edge cases.** Near-U-turn (θ→0), near-tangent (θ→π/2), extruder-only motion, exact-stop requests → fall back to current junction-deviation. Don't try to "save" pathological corners.
9. **Absorb tiny pre-corner segments** (LinuxCNC `ARC_BLEND_GAP_CYCLES = 4`): shorter than a few control cycles → let the arc consume the stub.
10. **One user-facing knob** (chord tolerance, mm). CYCLE832's existence proves users don't want 5 blend modes; wrap variants in presets later if needed.

---

## 7. References

**LinuxCNC** (source: `github.com/LinuxCNC/linuxcnc` @ HEAD): `src/emc/tp/blendmath.c` (`blendComputeParameters` L1137, `blendParamKinematics` L642); `blendmath.h` L21-31; `spherical_arc.c` (`arcInitFromPoints` L21); `tp.c` (`tpRunOptimization` L1820, `tpComputeOptimalVelocity` L1748, blend velocity L2340-2503); `tc_types.h` L26-29; `emc/task/emccanon.cc` L1141-1175 (naive-CAM). Docs: G64 <https://linuxcnc.org/docs/html/gcode/g-code.html#gcode:g64>, INI <https://linuxcnc.org/docs/html/config/ini-config.html>.

**Fanuc**: *0i-MODEL F Plus Parameter Manual* B-64700EN <https://www.fryermachine.com/pdf/Fanuc%20Series%200i%20MODEL%20F%20Plus%20Parameter%20Manual_B-64700EN_01.pdf>; HPCC A-78395E excerpt <https://www.scribd.com/document/555230156/A-78395E-02>; Markoski, *FANUC AI High-Speed Modes Simplified* <https://www.linkedin.com/pulse/fanuc-ai-high-speed-modes-simplified-tim-markoski>.

**Siemens**: 840D sl Programming Manual §12.2 <https://www.manualslib.com/manual/1585350/Siemens-Sinumerik-840d-Sl.html?page=295>; Function Manual B1 G641 <https://www.manualslib.com/manual/1636342/Siemens-Sinumerik-840d-Sl.html?page=190>, G642/G643 <https://www.manualslib.com/manual/1636342/Siemens-Sinumerik-840d-Sl.html?page=192>; Operating Manual CYCLE832 <https://www.manualslib.com/manual/1175411/Siemens-Sinumerik-840d-Sl.html?page=490>; Fundamentals PM <https://support.industry.siemens.com/cs/attachments/57038573/PG_0911_en_en-US.pdf>.

**Mach3/4**: *Mach3 CV Settings v2* <https://www.machsupport.com/wp-content/uploads/2013/02/Mach3_CVSettings_v2.pdf>; MachMotion *CV in Mach4* <https://support.machmotion.com/books/software/page/constant-velocity-in-mach4>.

**Academic (background)**: Yuen & Altintas, "Corner rounding of linear five-axis tool path by dual PH curves blending," *IJMTM* 2014 <https://www.sciencedirect.com/science/article/abs/pii/S0890695514001394>; Bi et al., "5-axis local corner rounding of linear tool path discontinuities," *IJMTM* 2013 <https://www.sciencedirect.com/science/article/abs/pii/S0890695513000886>.
