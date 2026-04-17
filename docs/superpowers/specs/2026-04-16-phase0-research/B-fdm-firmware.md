# Phase 0 Research — B: FDM Firmware Cornering Survey

Date: 2026-04-16
Scope: Prior art for Kalico's corner-arc-blending motion-planner rewrite. Sibling FDM firmwares only.

---

## 1. Executive Summary

- **Every mainstream FDM firmware (Marlin, RepRapFirmware, Smoothieware, Prusa-Firmware MK3, Prusa-Firmware-Buddy, and Klipper upstream) handles corners the same way Kalico does: a zero-duration direction change with a capped junction velocity.** None of them emit an actual blended arc between two G1 segments.
- The two cap strategies in the wild are **classic jerk / "instantaneous-speed-change"** (Marlin `CLASSIC_JERK`, RRF `M566`, Prusa-Firmware MK3 8-bit) and **junction deviation** (Marlin `JUNCTION_DEVIATION`, Smoothieware, Klipper, Prusa-Firmware-Buddy MK4). Both are velocity caps on a sharp vertex — only the math differs.
- `G2`/`G3` arc commands in every firmware surveyed are **re-segmented into short lines before the planner sees them**. They are not treated as a primitive by the trapezoidal lookahead.
- **The one firmware that does real arc (Bézier) corner blending is Prunt** — an independent new motion-controller project (Prunt Board 3), not a Kalico/Marlin sibling. It replaces each sharp vertex with a degree-15 Bézier constrained by a user-set max path deviation (`M205 D<mm>`). This is exactly the architecture Kalico is considering, and it is the only production example in the FDM space.
- Bambu Lab has publicly described a proprietary "device-side curve planning enhancement" (H2C/H2D, firmware ≥ 01.02.00.00, 2025) that causes Bambu Studio to **auto-disable slicer-side arc fitting** when the printer supports it. Details are closed-source, but the behavior strongly suggests firmware-side smoothing at/through arcs; no public confirmation of true corner-arc blending of G1s.

---

## 2. Per-firmware findings

### 2.1 RepRapFirmware (Duet3D, dev branch)

Source: `src/Movement/DDA.cpp`, `DDA.h` on `Duet3D/RepRapFirmware@dev`.

- **Corner primitive: zero-duration direction change + per-axis velocity cap.** In `DDA::MatchSpeeds()` (DDA.cpp ≈ line 1045), for every drive whose `directionVector` differs between this move and the next, the planner computes
  ```
  totalFraction = |dirVec[drive] - next.dirVec[drive]|
  jerk          = totalFraction * targetNextSpeed
  allowedJerk   = Platform::GetInstantDv(drive)        // configured via M566
  if (jerk > allowedJerk) targetNextSpeed = allowedJerk / totalFraction
  ```
  The join between DDAs is an instantaneous direction change whose magnitude is capped by `M566`. No arc is inserted.
- **Config parameter: `M566 X<mm/min> Y<mm/min> Z<mm/min> E<mm/min>`** — "maximum allowable instantaneous speed change" per drive. Note the units: `mm/min` (unlike Marlin's `mm/s`), a common slicer misconfiguration source.
- **Motion order:** RRF is a second-order (trapezoidal) planner with `M593` input shaping in 3.4+, extruder-synchronized with pressure advance in 3.5+.
- **David Crocker's articulated philosophy** (from Duet forum threads and `docs.duet3d.com/.../Third_order_motion`): S-curve / higher-order accel is pointless unless you also eliminate instantaneous speed changes at corners, because capped jerk already demands infinite acceleration. RRF therefore sticks with trapezoidal + jerk and adds input shaping as the vibration mitigation. Notably, a 3.5.2-era forum thread discusses an adaptive-jerk prototype: a `M566 J<deviation>` extension where jerk rises for near-straight joins (large included angle) and stays low for sharp corners — still a cap, not a blend.
- **Interaction with input shaping:** shaping smooths the step, cornering jerk still drives the step amplitude. There is no arc primitive to shape against.

### 2.2 Marlin 2.1.x

Source: `Marlin/src/module/planner.cpp`, `Marlin/src/gcode/motion/G2_G3.cpp`.

- **Corner primitive: zero-duration direction change + cap** via one of:
  - `HAS_JUNCTION_DEVIATION` (default on modern boards) — same centripetal-arc trick Klipper imported from Grbl.
  - `HAS_CLASSIC_JERK` — per-axis instantaneous speed change (Marlin historical default, still used by Prusa MK3).
  These two are mutually exclusive in `planner.cpp` (`#if HAS_JUNCTION_DEVIATION` vs `#if HAS_CLASSIC_JERK`). Both feed into `reverse_pass_kernel()` (planner.cpp ≈ line 1050–1090) which only caps `max_entry_speed_sqr`; nothing arc-like is emitted.
- **Config parameters:**
  - `JUNCTION_DEVIATION_MM` (`M205 J<mm>`, default 0.013 mm on most boards).
  - `DEFAULT_XJERK/YJERK/ZJERK/EJERK` (`M205 X Y Z E`, `mm/s`) when classic jerk is used.
- **G2/G3:** `plan_arc()` in `G2_G3.cpp` segments each arc into short lines governed by `MIN_ARC_SEGMENT_MM` / `MAX_ARC_SEGMENT_MM` and pushes them via `planner.buffer_line()`. The planner itself never sees an arc.
- **S-curve + junction deviation interaction:** `S_CURVE_ACCELERATION` has a well-known broken interaction with junction deviation (Marlin issues #11672, #12491, #16184) — S-curve emits velocity jumps at direction changes because the junction-deviation cap assumes trapezoidal. This is a concrete cautionary tale for Kalico: a velocity-cap cornering model and a higher-order acceleration profile do not compose cleanly.
- **Linear advance / pressure advance:** applied in the stepper ISR, orthogonal to cornering, but corner velocity drops cause extrusion rate steps that linear advance must chase.

### 2.3 Smoothieware

Source: `src/modules/robot/Planner.cpp` on `Smoothieware/Smoothieware@edge`.

- **Corner primitive: zero-duration direction change + junction-deviation cap.** Lines 164–168 carry the original Grbl comment ("Compute maximum allowable entry speed at junction by centripetal acceleration approximation. Let a circle be tangent to both previous and current path line segments..."). The cap is applied in `append_block()` at line ≈ 217:
  ```
  vmax_junction = min(vmax_junction,
                      sqrtf(acceleration * junction_deviation * sin_theta_d2
                            / (1.0f - sin_theta_d2)))
  ```
- **Config parameters:** `junction_deviation` (default 0.05 mm), `z_junction_deviation` (disabled by default), `minimum_planner_speed` (default 0.0 mm/s). All in the Smoothieware config file; no `M205 J` at runtime.
- No input shaping, no pressure advance (only extruder accel limit). G2/G3 is segmented before the planner (arc module).

### 2.4 Prusa-Firmware-Buddy (MK4, XL, MINI, CORE One)

Source: `include/marlin/Configuration_MK4.h`, `include/marlin/Configuration_MK4_adv.h` on `prusa3d/Prusa-Firmware-Buddy@master`.

- **Planner: Marlin 2.0 fork.** Prusa describes it as "Marlin 2.0 with significant changes to comply with Prusa in-house developed technologies" (e.g. Input Shaper, Pressure Advance, MBL). The cornering algorithm is unmodified Marlin junction deviation.
- **Config:** `JUNCTION_DEVIATION` enabled; `CLASSIC_JERK` disabled. `JUNCTION_DEVIATION_MM` default is tight (≈ 0.02 mm on MK4 profiles) to pair with their Input Shaper tune. `S_CURVE_ACCELERATION` is not enabled — Input Shaper is the vibration story instead.
- **Interaction with PrusaSlicer "Precise wall" / arc fitting:** PrusaSlicer emits G2/G3 when arc fitting is enabled; Buddy firmware re-segments via the standard Marlin `plan_arc()`. Corner artifacts the user sees at geometry vertices are still governed by `JUNCTION_DEVIATION_MM` plus Input Shaper, not by an arc blend.
- No public evidence of corner-arc-blending work in the Buddy repo.

### 2.5 Prusa-Firmware (MK3 8-bit Marlin fork)

Source: `prusa3d/Prusa-Firmware@MK3` — `Firmware/variants/1_75mm_MK3*.h`.

- **Corner primitive: classic jerk.** `CLASSIC_JERK` is set; `JUNCTION_DEVIATION` is commented out. Defaults: `DEFAULT_XJERK 8.0`, `DEFAULT_YJERK 8.0`, `DEFAULT_ZJERK 2.0`, `DEFAULT_EJERK 5`. `JUNCTION_DEVIATION_MM 0.02` exists but is inside the disabled branch. `S_CURVE_ACCELERATION` is off.
- Planner is stock Marlin 1.x-era trapezoidal; no input shaping, no corner arc work. This is historically the archetypical reference for "classic jerk", the thing Klipper deliberately moved away from with JD.

### 2.6 Bambu Lab (X1, P1, A1, H2D, H2C/H2S)

Proprietary firmware; no source access. Public surface:

- **Slicer-side arc fitting (G2/G3 emission)** is standard and Orca/Bambu Studio's default for external perimeters (Bambu Studio release notes 1.9.1+). Firmware re-segments (standard approach).
- **"Device-side curve planning enhancement"** introduced with H2C/H2D firmware ≥ 01.02.00.00 (Bambu Studio V2.5.3, 2025). When Bambu Studio syncs to a supporting printer, it **auto-disables arc fitting** in the slicer. Reading between the lines, this implies the firmware's planner wants continuous segmented geometry so it can apply its own smoothing/blending in-device; if the slicer already fit arcs, the firmware's re-segmenter and its smoother would fight. This is consistent with — but does not prove — firmware-side corner blending.
- HN discussions (items 40292273, 40294048) speculate Bambu does firmware-side smoothing that lets them run very high accels without visible corner ringing; nobody has posted disassembly evidence of real corner-arc insertion. Treat as "probable advanced blending, mechanism unconfirmed."

### 2.7 Klipper upstream — recent activity

Source: `Klipper3d/klipper` PR history through 2026-04-16.

- **PR #6747** ("toolhead: Fixed junction deviation calculation for straight segments", merged Nov 2024) — removes hardcoded `±0.999999` clamps in the centripetal cap so near-straight and near-reversing junctions are computed exactly. Still the same velocity-cap model.
- **PR #6657** ("gcode_arc: use arc generator — reduce blocking time", 2024) and **PR #6984** ("RFC: gcode_arcs add max error support", 2025) — both improve the arc-to-line segmenter; arcs still land in the planner as short G1s.
- **PR #7235** / **PR #7231** (2026) — input-shaper matrix fixes and MZV generalization. Orthogonal to cornering.
- **No upstream work on real corner-arc blending, Bézier smoothing, or higher-order motion profiles.** The planner still emits the classic trapezoid with JD at junctions. If Kalico builds this, we are ahead of upstream by a full architectural generation.

---

## 3. Comparison table

| Firmware              | Corner primitive                 | Config parameter (unit)                 | Real arc at corners? | Input shaping? | Notes |
|-----------------------|----------------------------------|-----------------------------------------|----------------------|----------------|-------|
| RepRapFirmware 3.6    | zero-dur dir change, per-axis cap | `M566 X/Y/Z/E` (mm/min, per-axis `InstantDv`) | No                   | Yes (`M593`, ZVD/EI)  | DDA.cpp `MatchSpeeds()` L1045; dc42 explicitly rejects S-curve without solving the jerk step |
| Marlin 2.1 (JD)       | zero-dur dir change, JD cap       | `M205 J<mm>` (`JUNCTION_DEVIATION_MM`)  | No                   | Yes (Input Shaper)    | Grbl-derived centripetal approx; JD + S-curve is broken |
| Marlin 2.1 (classic)  | zero-dur dir change, per-axis cap | `M205 X/Y/Z/E` (mm/s)                   | No                   | Yes                   | Mutually exclusive with JD |
| Smoothieware          | zero-dur dir change, JD cap       | `junction_deviation` (mm, config file)  | No                   | No                    | Original Grbl JD port |
| Prusa-Firmware-Buddy  | zero-dur dir change, JD cap       | `JUNCTION_DEVIATION_MM` ≈ 0.02 mm       | No                   | Yes (Prusa IS + PA)   | Marlin 2.0 fork |
| Prusa-Firmware MK3    | zero-dur dir change, classic jerk | `DEFAULT_*JERK` 8/8/2/5 mm/s            | No                   | No                    | 8-bit legacy Marlin |
| Bambu Lab (H2C+)      | unknown; likely firmware-side smoothing | proprietary                        | Unclear (probable blending) | Yes (proprietary) | Auto-disables slicer arc fitting when firmware curve-planning is active |
| Klipper upstream      | zero-dur dir change, JD cap       | `square_corner_velocity` (mm/s) / JD    | No                   | Yes (input_shaper)    | PR #6747 (2024) tightened math; no blending work |
| **Prunt** (reference) | **Bézier corner blend**           | **`M205 D<mm>`** (path deviation)       | **Yes (degree-15 Bézier)** | Yes (higher-order profile) | Independent project; proves the concept in FDM |

---

## 4. Key insights for Kalico

1. **The design space is unoccupied among Kalico's direct siblings.** Every Marlin/RRF/Smoothie/Buddy/MK3/upstream-Klipper variant surveyed caps velocity on a sharp vertex. Real arc blending at corners in an FDM planner is new territory — except for Prunt.
2. **Prunt is the prior art to read carefully.** It uses a degree-15 Bézier (chosen for G⁴ continuity through the blend so velocity, accel, jerk, snap and crackle are all continuous), user-controlled by a single `M205 D<mm>` path-deviation parameter. Kalico should strongly consider a similar user-facing contract: one scalar, units of mm, semantics "max perpendicular deviation of the blended path from the original vertex." That's the same mental model users already have for `junction_deviation_mm`, so the migration is intuitive.
3. **The `square_corner_velocity` / `junction_deviation` UX transfers.** We do not need to invent a new config term. If the new planner still accepts `square_corner_velocity` but reinterprets its meaning as "arc blend radius derived from acceleration and this velocity," backward compatibility is cheap.
4. **Marlin's S-curve + JD bug (#11672, #12491, #16184) is a direct warning.** Their bug was: add a higher-order accel profile on top of a velocity-cap cornering model → the profile re-introduces velocity steps at corners that the cap assumed away. Kalico's temptation will be "add arc blending alongside the existing trapezoidal planner." That will re-create Marlin's failure mode. Arc blending must *replace* the junction-cap, not wrap it.
5. **dc42's philosophical point cuts both ways.** He argues higher-order motion is pointless while corners are still zero-duration. Kalico's rewrite lets us invert his critique into a pitch: we're fixing the *actual* root cause (the velocity step at the vertex) instead of papering over it with input shaping.
6. **Arc blending composes well with input shaping and pressure advance, unlike velocity caps.** A blend replaces one discontinuous join with a continuous C² (or C⁴, à la Prunt) curve; the shaper sees a smooth velocity trajectory instead of a step; PA sees smooth extruder velocity instead of an extrusion step. All three existing tools get *easier*, not harder, to tune. This is a strong product story.
7. **G2/G3 arc commands are irrelevant to this rewrite.** Every firmware surveyed (including Klipper) segments them into lines before the planner. Kalico's arc-blending planner will see pre-segmented G1s just like today. The blend happens *between* G1s, not *on* G2s.
8. **Bambu is the commercial validation.** Whatever exactly their "curve planning enhancement" does, Bambu sells printers that run at 500 mm/s + 20 k mm/s² with good surfaces, and they actively moved corner/arc smoothing *into the firmware*. The market has decided that firmware-side blending is worth it.
9. **Two calibration stories to adopt early:** (a) deviation-vs-dimensional-accuracy tradeoff test (e.g. OrcaSlicer's "cornering calib"), lets users pick `max_deviation` visually. (b) resonance + corner blend combined test — because once corners are smooth, a lot of existing IS tunes will be too aggressive and the corner ringing that input-shaping was masking disappears, letting users relax shaper strength. Prunt's docs hint at this and it maps to Kalico's existing `resonance_tester`.
10. **Hardware gotcha: floating-point throughput on the MCU.** RRF's `DDA::MatchSpeeds` and Klipper's JD are both lightweight scalar math per junction. A Bézier blend is more expensive (and injects extra steps into the queue). Prunt side-steps this with hardware step timers. Kalico should budget blend generation on the host (where it belongs) and measure lookahead queue depth — the additional pseudo-segments from a blend will consume planner/toolhead buffer.

---

## 5. References

- Kalico / Klipper planner background:
  - [Klipper3d/klipper — toolhead: Fixed junction deviation calculation for straight segments (PR #6747)](https://github.com/Klipper3d/klipper/pull/6747)
  - [Klipper discourse — "The Myth of G2/G3 Arc Commands"](https://klipper.discourse.group/t/the-myth-of-g2-g3-arc-commands/24335)
  - [Klipper issue #202 — Support for G2/G3 Controlled Arc Move](https://github.com/Klipper3d/klipper/issues/202)
- RepRapFirmware / Duet:
  - [Duet3D/RepRapFirmware — DDA.cpp (dev branch)](https://github.com/Duet3D/RepRapFirmware/blob/dev/src/Movement/DDA.cpp) — `MatchSpeeds()` ≈ L1045, `DoLookahead()` ≈ L931
  - [Duet3D/RepRapFirmware — DDA.h](https://github.com/Duet3D/RepRapFirmware/blob/dev/src/Movement/DDA.h)
  - [Duet3D docs — Third-order motion](https://docs.duet3d.com/User_manual/RepRapFirmware/Third_order_motion)
  - [Duet3D docs — Input shaping (M593)](https://docs.duet3d.com/User_manual/Tuning/Input_shaping)
  - [Duet3D forum — 6th-order jerk-controlled motion planning (dc42)](https://forum.duet3d.com/topic/4802/6th-order-jerk-controlled-motion-planning)
  - [Duet3D forum — FW 3.5.2 High jerk for circular paths, not corners (adaptive-jerk prototype)](https://forum.duet3d.com/topic/36505/fw-3-5-2-high-jerk-good-for-circular-path-not-for-corners)
- Marlin:
  - [MarlinFirmware/Marlin — planner.cpp (2.1.x)](https://github.com/MarlinFirmware/Marlin/blob/2.1.x/Marlin/src/module/planner.cpp) — `reverse_pass_kernel()` ≈ L1050–1090
  - [MarlinFirmware/Marlin — G2_G3.cpp](https://github.com/MarlinFirmware/Marlin/blob/2.1.x/Marlin/src/gcode/motion/G2_G3.cpp) — `plan_arc()`
  - [Marlin M205 reference](https://marlinfw.org/docs/gcode/M205.html)
  - [Marlin issue #11672 — JD + S-curve no effect](https://github.com/MarlinFirmware/Marlin/issues/11672)
  - [Marlin issue #12491 — S-curve velocity jumps on direction change](https://github.com/MarlinFirmware/Marlin/issues/12491)
  - [Marlin issue #16184 — S-curve not working with JD](https://github.com/MarlinFirmware/Marlin/issues/16184)
- Smoothieware:
  - [Smoothieware/Smoothieware — Planner.cpp](https://github.com/Smoothieware/Smoothieware/blob/edge/src/modules/robot/Planner.cpp) — JD approximation L164–217
- Prusa:
  - [prusa3d/Prusa-Firmware-Buddy — Configuration_MK4.h](https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/include/marlin/Configuration_MK4.h)
  - [prusa3d/Prusa-Firmware — MK3 variants](https://github.com/prusa3d/Prusa-Firmware/tree/MK3/Firmware/variants) — classic jerk 8/8/2/5
- Bambu:
  - [Bambu Studio V2.5.3 release note (device-side curve planning enhancement)](https://wiki.bambulab.com/en/software/bambu-studio/release/release-note-2-5-3)
  - [Bambu Wiki — Arc move](https://wiki.bambulab.com/en/software/bambu-studio/acr-move)
  - [HN discussion 40292273](https://news.ycombinator.com/item?id=40292273)
  - [HN discussion 40294048](https://news.ycombinator.com/item?id=40294048)
- Prunt (real arc blending, the prior art of interest):
  - [Prunt — Features](https://prunt3d.com/docs/features/)
  - [Prunt — G-Code Reference (`M205 D` path deviation)](https://prunt3d.com/docs/gcode_reference/)
  - [Hackaday — "Keeping Snap And Crackle Under Control With Prunt Printer Firmware" (2025-06-18)](https://hackaday.com/2025/06/18/keeping-snap-and-crackle-under-control-with-prunt-printer-firmware/)
- Calibration story:
  - [OrcaSlicer wiki — cornering calib](https://github.com/OrcaSlicer/OrcaSlicer/wiki/cornering-calib)
