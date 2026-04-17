# j_eff Derivation — Design Spec

**Date:** 2026-04-17
**Scope:** Stage 1 sub-spec #2 of the corner-blending fork (see `2026-04-16-phase0-research/00-summary.md`).
**Status:** Design approved, ready for implementation planning.

---

## Purpose

Derive the effective jerk ceiling `j_eff` that bounds traversal velocity through a blend arc such that Kalico's per-axis input shapers do not have their smoothness assumptions violated.

Fills the "placeholder `j_eff` input to `blend_geometry`" left by `2026-04-16-blend-geometry-module-design.md`. Also produces a second bound (`v_step_cap`) that protects the per-axis acceleration step commanded at arc entry/exit. Together with the existing centripetal bound these three bounds determine `v_cap` on every blend corner.

The derivation is physics-based, cites canonical sources for every non-trivial claim, and anchors its numeric constants to calibrations that already exist in Kalico (`find_shaper_max_accel`, `TARGET_SMOOTHING`). No new user-facing configuration is introduced.

## Non-goals

Not this spec:

- Empirical hardware calibration of `j_eff` (scope (B): analytical derivation + documented validation recipe; the recipe is an artifact of this spec, not a code deliverable).
- A runtime calibration macro analogous to `SHAPER_CALIBRATE` (Stage 2 follow-up if and only if hardware measurements show the analytical bound needs per-machine tuning).
- Changes to the core of `klippy/blendmath.py` (`blend_geometry`, `segment_arc`, `BlendArc`). Only the adapter layer `blend_from_moves` is extended.
- Adding Z-axis input shaping to Kalico. The derivation is axis-agnostic and Z-ready, but actually enabling Z shaping is its own work.
- Planner integration into `toolhead.py` / `LookAheadQueue`.
- SCV / `junction_deviation` removal.

## Prior art (with sources)

The problem has been solved twice in the motion-control world with a consistent principle — LinuxCNC and Prunt both use **`v ≤ (R² · j_max)^(1/3)`** (equivalently `R ≥ v^(3/2) / √j_max`) to bound rotation jerk — but **both treat `j_max` as a user-configured INI/YAML parameter**, not derived from any physical model:

- **LinuxCNC** ([`src/emc/tp/blendmath.c` L1178–1199](https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/tp/blendmath.c#L1178-L1199)): the jerk `emcmotStatus->jerk` is read from `[TRAJ]MAX_LINEAR_JERK` in the INI file ([initraj.cc](https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/ini/initraj.cc#L180)). Default is 1e9 (effectively unbounded). The physical derivation LinuxCNC cites in the code comment is the geometric identity `j = v³/R² (worst case at corner transitions)`.
- **Prunt** ([`prunt-motion_planner-planner-kinematic_limiter.adb` L59–103](https://github.com/Prunt3D/prunt/blob/master/src/prunt-motion_planner-planner-kinematic_limiter.adb)): `Jerk_Max`, `Snap_Max`, `Crackle_Max` are independent user-configured scalars in `User_Config_Motion_Planner`. Input shaping is a separate downstream stage; the planner jerk bound and the shaper are not reconciled.

Phase 0's claim that "no published derivation combines input shaping with corner blending" is partially incorrect — **Cho, Song, Lee et al. (2018)** combine input shaping with corner-rounding algorithmically ([Int. J. Adv. Manuf. Technol. 97, 105–116](https://link.springer.com/article/10.1007/s00170-018-1922-0)). However, **no peer-reviewed closed-form derivation of a jerk bound from Singer-family shaper properties exists** to the best of our search. The derivation below is a Kalico-original contribution, grounded in each step's physics but not previously synthesized in this form.

Kalico already has a shaper-aware acceleration bound: `find_shaper_max_accel` in `klippy/extras/shaper_calibrate.py` bisects for the `max_accel` at which `_get_shaper_smoothing(shaper, accel, scv) ≤ TARGET_SMOOTHING = 0.12 mm`. This is a **deviation bound at discrete corners**, not a jerk bound, but it defines the acceleration ceiling each shaper tolerates at steady-state. We reuse this value (`A_axis`) as the per-axis anchor for both new bounds below.

## Module layout

File: `klippy/blendshaper.py`. Python 3, `math` and `dataclasses` only. Zero Kalico imports in the core function; adapter lives alongside `blend_from_moves` in `klippy/blendmath.py`.

```
klippy/blendshaper.py
├─ AxisShaperSnapshot  (dataclass)               # per-axis inputs to the core math
│    axis            str     "x" | "y" | "z" | …
│    shaper_type     str     "zv" | "mzv" | "zvd" | "ei" | "2hump_ei" | "3hump_ei" | None
│    shaper_freq     float   Hz; 0 or None means unshaped
│    damping_ratio   float   ζ, typically 0.1
│    A_axis          float   mm/s² from find_shaper_max_accel
│
├─ ShaperBounds  (dataclass)                     # output of the core math
│    j_eff          float    scalar jerk, plugged into blend_geometry unchanged
│    v_step_cap     float    per-axis entry-step cap, applied post-hoc
│
├─ shaper_span(shaper_type, freq, damping_ratio) -> float
│     # returns T_shaper_axis per the pulse-span table
│
├─ axis_projections(n_hat) -> dict[str, float]
│     # |n̂·ê_axis| at arc entry for each axis (for Bound (b))
│
├─ axis_in_plane(p_hat) -> dict[str, float]
│     # √(1 − |p̂·ê_axis|²) = projection of ê_axis onto the arc plane
│     # (for Bound (c); equals 1 for fully in-plane axes, 0 for out-of-plane)
│
└─ compute_shaper_bounds(
      shapers:       Iterable[AxisShaperSnapshot],
      R:             float,
      n_hat:         Vec3,                 # arc normal at entry, unit vector
      p_hat:         Vec3,                 # arc plane normal, unit vector
   ) -> ShaperBounds
```

`AxisShaperSnapshot` is a plain-data carrier; all Kalico-specific extraction happens in `blend_from_moves`'s adapter code (which reads `toolhead.lookup_object("input_shaper").get_shapers()` and calls `find_shaper_max_accel` per axis).

`blend_geometry` is **unchanged**. Its existing `j_eff` parameter receives the scalar output of `compute_shaper_bounds`. The per-axis entry-step cap is applied in `blend_from_moves` after `blend_geometry` returns.

## Algorithm

### Conventions

Consistent with `blendmath.py`:

- `Vec3 = tuple[float, float, float]`.
- `prev_dir`, `next_dir`: head-to-tail unit direction vectors.
- Arc plane is spanned by `prev_dir` and `next_dir`; its normal `p̂ = prev_dir × next_dir` (direction ambiguous for collinear/U-turn, but those cases return early from `blend_geometry` before this module is consulted).
- `n̂` at entry = the inward normal from `entry_pt` to `center`; computed in `blend_geometry` and passed in.
- `ê_a` ∈ {x̂, ŷ, ẑ}: world-frame basis vectors.
- `t̂` = unit tangent, = `prev_dir` at entry, rotates along the arc.

### Per-axis shaper inputs

For each axis `a ∈ {x, y, z}` that has a configured shaper (`shaper_freq > 0`):

- `f_a` = shaper frequency (Hz).
- `ζ_a` = damping ratio (0.1 default).
- `t_d_a = 1 / (f_a · √(1 − ζ_a²))` = damped resonance period.
- `T_a` = pulse-sequence span, set by shaper type:

  | Shaper type | T_a | Source |
  |---|---|---|
  | ZV        | 0.5 · t_d_a | Singer & Seering 1990, [DOI 10.1115/1.2894142](https://doi.org/10.1115/1.2894142) |
  | MZV       | 0.75 · t_d_a | Butyugin's Klipper variant; see `shaper_defs.py:get_mzv_shaper` |
  | ZVD       | 1.0 · t_d_a | Singer & Seering 1990 |
  | EI        | 1.0 · t_d_a | Singhose, Seering & Singer 1994 |
  | 2HUMP_EI  | 1.5 · t_d_a | Singhose, Porter, Tuttle, Singer 1997, [DOI 10.1115/1.2801257](https://doi.org/10.1115/1.2801257) |
  | 3HUMP_EI  | 2.0 · t_d_a | Singhose 1997 (same) |

  These match `klippy/extras/shaper_defs.py` exactly (independently verified against the canonical references).

- `A_a` = `find_shaper_max_accel(shaper_a, scv=0)` from `klippy/extras/shaper_calibrate.py`. This is the max steady-state acceleration the axis's shaper tolerates at the existing `TARGET_SMOOTHING = 0.12 mm` calibration.

Axes with `shaper_freq = 0` contribute no bound and are skipped.

### Bound (a): total centripetal

Unchanged from `blend_geometry`. Protects the toolhead's vector acceleration budget using LinuxCNC's Pythagorean split:

```
a_n_max = (√3 / 2) · a_max
v_centripetal ≤ √(a_n_max · R)
```

`a_max` is the machine-wide scalar from `max_accel` config. This bound exists because the TOTAL vector acceleration magnitude during arc traversal must stay within the machine's motor/belt budget, irrespective of how that budget projects onto individual axes.

### Bound (b): per-axis entry-step

At arc entry, commanded acceleration steps from 0 to `a_n = (v²/R) · n̂`. The projection onto axis `a` is `(v²/R) · |n̂·ê_a|` (standard linear-algebra projection of a vector onto a basis direction).

For the shaper on axis `a` to not over-smooth the commanded acceleration (i.e., the axis stays within its calibrated tolerance), the magnitude of the commanded step must not exceed `A_a`:

```
(v² / R) · |n̂·ê_a|  ≤  A_a
v  ≤  √( A_a · R / |n̂·ê_a| )
```

Taking the min over all shaped axes that have significant `|n̂·ê_a|`:

```
v_step_cap  =  min_{a : shaped, |n̂·ê_a| > ε} √( A_a · R / |n̂·ê_a| )
```

Small-projection axes (|n̂·ê_a| < ε = 1e-9) are skipped to avoid divide-by-zero; those axes trivially pass the bound. If no shaped axis has significant projection, `v_step_cap = +∞`.

### Bound (c): per-axis rotation jerk

During constant-speed rotation on the arc, the jerk vector has magnitude `|j| = v³/R²`, directed along the tangent (antiparallel to velocity). Derivation (standard Frenet–Serret, verified in [Wikipedia - Frenet–Serret formulas](https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas) and textbook references):

```
a(t) = v̇ · t̂ + (v²/R) · n̂
      = (v²/R) · n̂                   (constant speed: v̇ = 0)
j(t) = d/dt a(t) = (v²/R) · dn̂/dt
     = (v²/R) · (-κ·v·t̂)              (Frenet–Serret: dn̂/dt = -κ·v·t̂, κ = 1/R)
     = -(v³/R²) · t̂
```

So `|j| = v³/R²`, direction = `−t̂`. [Independently verified against Wikipedia - Jerk (physics), UCF OpenStax 4.4, and Kleppner & Kolenkow Mechanics 2e Problem 1.22.]

The tangent `t̂` rotates through the arc plane as the toolhead sweeps from `entry_pt` to `exit_pt`. For each axis, the worst-case projection of `t̂` onto `ê_a` over the arc sweep equals the magnitude of `ê_a`'s in-plane component:

```
axis_in_plane_a = √(1 − |p̂·ê_a|²)
```

where `p̂` is the arc plane normal. For a fully in-plane axis (e.g., x̂ on an XY arc), `axis_in_plane_a = 1`. For a fully out-of-plane axis (ẑ on an XY arc), `axis_in_plane_a = 0`. The worst-case jerk projection on axis `a` is therefore `(v³/R²) · axis_in_plane_a`. Strictly, this assumes the arc sweeps through the direction of maximum projection; for small-deflection corners the sweep may not, but small-deflection corners have large `R` and are centripetal-bound anyway — the bound stays correct (possibly conservative) in that regime.

Requiring that the worst-case jerk, window-averaged through the shaper's span `T_a`, stays within the axis's tolerated acceleration ceiling (expressed as "jerk = acceleration-ceiling / time-window"):

```
(v³ / R²) · axis_in_plane_a  ≤  A_a / T_a
v  ≤  ( R² · A_a / (T_a · axis_in_plane_a) )^(1/3)
```

Taking the min over shaped axes with `axis_in_plane_a > ε` (small-projection axes are skipped to avoid divide-by-zero; they contribute no bound):

```
j_eff_binding  =  min_{a : shaped, axis_in_plane_a > ε} ( A_a / (T_a · axis_in_plane_a) )
v_jerk  ≤  ( R² · j_eff_binding )^(1/3)  =  ( R · √j_eff_binding )^(2/3)
```

The final form is algebraically identical to LinuxCNC's `v_jerk = (R·√j_eff)^(2/3)`, which is what `blend_geometry`'s existing `j_eff` parameter expects. We pass `j_eff = j_eff_binding` and the existing formula produces the correct answer.

For the 99%-case of planar XY arcs on a standard printer, `axis_in_plane_x = axis_in_plane_y = 1` and the formula collapses to `j_eff_binding = min(A_x/T_x, A_y/T_y)`. The `axis_in_plane` factor matters only for skewed arcs (e.g., XYZ-combined motion) or if Z shaping is later enabled on a machine with non-axis-aligned arc planes.

**Caveat on Bound (c)**: the `A_a / T_a` identification treats the shaper as a window-averaging filter of timescale `T_a`. This is a bandwidth heuristic, not a strict peak-jerk bound. A true delta-train shaper is a comb filter (all-pass outside its notches; see [Cole 2012, Automatica](https://www.sciencedirect.com/science/article/abs/pii/S0005109812002567)) — convolving a commanded step with positive impulses spanning `T_a` produces a staircase whose average slope is `Δa/T_a` but whose per-segment slope between impulses is formally unbounded. For physically realizable commanded inputs (not true delta functions), the bound is defensible. The validation recipe below addresses pathological cases empirically.

### Combined cap

```
v_cap = min( v_centripetal, v_step_cap, v_jerk )
```

`v_centripetal` comes from `blend_geometry` as-is.
`v_jerk` comes from `blend_geometry` using `j_eff` from `compute_shaper_bounds`.
`v_step_cap` is applied in the adapter after `blend_geometry` returns.

### Resolving the R / n_hat circularity

`compute_shaper_bounds` needs `R`, `n_hat`, and `p_hat`, but `blend_geometry` determines those using `j_eff`, which is what we're trying to compute. The resolution:

1. Call `blend_geometry` with an **initial placeholder** `j_eff = +∞` → obtain `R_0`, `n_hat_0`, `p_hat_0`. At this stage `R_0` is determined by `corner_deviation` and the `R_mid` cap only.
2. Call `compute_shaper_bounds(shapers, R_0, n_hat_0, p_hat_0)` → obtain `j_eff_1`, `v_step_cap_1`.
3. Call `blend_geometry` again with `j_eff_1` → obtain final `R`, `n_hat`, `p_hat`, `v_cap_jerk`. The `R_tol` and `R_mid` caps are monotone in `corner_deviation` and segment lengths, unaffected by `j_eff`, so `R` is stable across iteration.
4. Apply `v_step_cap = min(v_step_cap_1, √(A_a·R/|n̂·ê_a|) for final R, n̂)` — a second evaluation of Bound (b) with the final geometry. In practice Bound (b) is nearly R-independent for small R changes, so one evaluation suffices; the spec requires two for correctness.

The iteration is O(1) (fixed two passes). No fixed-point solver.

### Adapter sketch

`blend_from_moves` in `klippy/blendmath.py` gains an optional `toolhead` parameter and extends its existing logic:

```python
def blend_from_moves(prev_move, next_move, corner_deviation, toolhead=None):
    # ... existing guards (non-kinematic moves, axes_r extraction) ...
    shapers = _extract_shapers(toolhead)   # -> list[AxisShaperSnapshot]; empty if toolhead is None
    arc_0 = blend_geometry(..., j_eff=float("inf"))
    if arc_0 is None or arc_0.R == 0.0 or not shapers:
        return arc_0  # collinear, U-turn, or no shapers: nothing more to do.
    bounds = blendshaper.compute_shaper_bounds(
        shapers=shapers,
        R=arc_0.R,
        n_hat=vnormalize(vsub(arc_0.center, arc_0.entry_pt)),
        p_hat=arc_0.plane_normal,
    )
    arc = blend_geometry(..., j_eff=bounds.j_eff)
    v_cap = min(arc.v_cap, bounds.v_step_cap)
    return replace(arc, v_cap=v_cap)   # dataclass.replace; BlendArc is frozen
```

`_extract_shapers` is a small helper that reads `toolhead.lookup_object("input_shaper").get_shapers()` and runs `find_shaper_max_accel(shaper, scv=0)` per axis. Returning an empty list (no shapers, or `toolhead` passed as `None`) keeps `blend_from_moves` callable from pytest without Kalico infrastructure, preserving the existing test-in-isolation property.

## Degenerate cases

1. **No axis has a shaper configured** — `compute_shaper_bounds` returns `j_eff = +∞`, `v_step_cap = +∞`. `blend_geometry`'s jerk bound becomes vacuous; only the centripetal bound binds. Identical to passing a large `j_eff` manually.
2. **Only out-of-plane axes are shaped** (e.g., arc in XY plane, only Z is shaped — hypothetical future): the in-plane rotation-jerk bound (c) has no axes to reduce over → `j_eff = +∞`. The entry-step bound (b) may still apply if `n̂·ẑ ≠ 0`, but for a planar XY arc `n̂` lies in the XY plane so `n̂·ẑ = 0` — that axis is skipped. Result: Z-only shaping contributes nothing to XY arcs, which is correct.
3. **Arc plane normal is zero** (collinear or U-turn) — `blend_geometry` already returns early (None or zero-radius). `compute_shaper_bounds` is not called.
4. **Very tight arcs with small in-plane projection for some axis** (|n̂·ê_a| < 1e-9): skip that axis from Bound (b) to avoid divide-by-zero. Bound (c) still applies because it uses worst-case `|t̂·ê_a| = 1`.
5. **SCV removal changes `find_shaper_max_accel` signature** — the adapter absorbs this; the core function takes `A_axis` pre-computed, so is insulated.

## Testing

`test/test_blendshaper.py`.

1. **Unit tests (analytical fixtures)**:
   - ZV at 100 Hz, ζ=0.1, A=5000 → compute `T_a`, verify `j_eff = A/T` against hand-derived value.
   - Two-axis asymmetric: X=ZV@150Hz, Y=ZV@60Hz → verify Y's (smaller A/T) binds.
   - Unshaped axis: shaper_freq=0 → axis contributes no bound.
   - Mixed shaper types on the two axes: verify each axis uses its own T.
2. **Projection tests**:
   - 90° XY corner: n̂ = (1,1,0)/√2 → |n̂·x̂| = |n̂·ŷ| = 1/√2. Verify Bound (b) evaluates symmetrically.
   - 30° corner: asymmetric projections; verify formula.
   - Z-projected corner (arc in XZ plane): verify axis projections pick up Z correctly; unshaped Z contributes no bound.
3. **Bound combination**:
   - Construct a corner where Bound (a) binds (large R, loose jerk); verify.
   - Construct a corner where Bound (b) binds (tight corner, tight shaper); verify.
   - Construct a corner where Bound (c) binds (small R, large accel); verify.
4. **Integration tests** (in `test/test_blendmath.py` — extends existing fixtures):
   - End-to-end `blend_from_moves` with a mocked toolhead exposing realistic shapers; verify `v_cap` matches the expected min of the three bounds.
   - R/n_hat iteration: verify R is stable across the two-pass iteration.
5. **Numeric sanity against user's hardware profile** (all quantities in Kalico's native units: mm, s, mm/s, mm/s², mm/s³):
   - X = ZV @ 150 Hz, Y = ZV @ 80 Hz, ζ = 0.1, `max_accel = 50000 mm/s²`.
   - `T_x = 0.5 / (150·√(1−0.01)) ≈ 3.35 ms`;  `T_y = 0.5 / (80·√(1−0.01)) ≈ 6.28 ms`.
   - `A_x`, `A_y` come from `find_shaper_max_accel` — hypothetical illustrative values: `A_x = 12000`, `A_y = 6000` (Y is stricter because lower frequency).
   - At a representative 90° corner with `R = 0.5 mm`: `|n̂·ê_x| = |n̂·ê_y| = 1/√2`; `axis_in_plane_x = axis_in_plane_y = 1`.
   - Bound (a): `v_centripetal = √((√3/2) · 50000 · 0.5) = √21650 ≈ 147 mm/s`.
   - Bound (b): `v_step_cap = min(√(A_x · 0.5 · √2), √(A_y · 0.5 · √2)) = min(√(8485), √(4243)) ≈ min(92, 65) = 65 mm/s`. Y binds.
   - Bound (c): `j_eff = min(A_x/T_x, A_y/T_y) = min(3.58e6, 9.55e5) ≈ 9.55e5 mm/s³`; `v_jerk = (0.25 · 9.55e5)^(1/3) ≈ (2.39e5)^(1/3) ≈ 62 mm/s`. Y binds.
   - **Final v_cap ≈ 62 mm/s at R=0.5mm**, set by rotation jerk on Y axis.
   - Cross-check: this is well below current SCV=70's junction-velocity-at-90° of ~169 mm/s, which makes sense — we're deliberately trading some corner speed for smoothness at the 5% residual-vibration tolerance.
   - These numbers are **illustrative** — real `A_x`/`A_y` come from calibration. The test suite performs the computation end-to-end and verifies the three bounds combine as expected.
6. **Monotonicity properties**:
   - Lower shaper frequency → lower `j_eff`.
   - Higher damping ratio → lower `j_eff` (t_d grows).
   - Tighter `TARGET_SMOOTHING` → lower `A_a` → lower `j_eff`.

## Validation recipe (deliverable per scope (B))

This is a **documented protocol** a user follows on real hardware once planner integration lands. No code in this spec; the recipe lives in the spec for reference.

**Procedure:**

1. Flash the Kalico build with this spec + Blend Geometry Module + planner integration landed.
2. Print a calibration pattern: a square with N=90° corners at configurable size (10 mm outer, 8 mm inner). Run at a fixed feedrate `v_test` approaching the predicted `v_cap` for each corner radius.
3. Attach an accelerometer (already required for shaper calibration).
4. Capture `ACCELEROMETER_QUERY` (or `MEASURE_AXES_NOISE`) over the corner sweeps.

**Pass criterion:**

The shaped residual vibration amplitude during and immediately after each corner must not exceed `1/SHAPER_VIBRATION_REDUCTION = 1/20 = 5%` of the unshaped amplitude the same motion would produce. This matches Kalico's existing design tolerance in `shaper_defs.py:SHAPER_VIBRATION_REDUCTION`. If residual exceeds 5%, the analytical `j_eff` was too aggressive for the actual hardware; document the failure mode and tighten empirically.

**Expected signal shape:**

- Step-in at corner entry → mild transient of ≤ 1 shaper period duration → settle to steady-state rotation level → mild transient at exit → settle to zero.
- No ringing beyond the shaper's designed residual-vibration tolerance.
- Total residual vibration RMS over the corner sweep should be within a factor of 2 of the RMS from a same-speed straight-line traverse.

**Fallback (deferred to Stage 2 if needed):**

If the analytical `j_eff` is too optimistic on some hardware, a future calibration macro could bisect for the largest `j_eff` under which the pass criterion holds. This is scoped **out** of the current spec.

**This recipe runs only when planner integration lands.** It is NOT a blocker for this sub-spec's merge; it is a blocker for the Stage 1 completion gate defined in the Phase 0 summary.

## Dependencies

**Must be landed before this can be wired end-to-end:**
- Blend Geometry Module (`klippy/blendmath.py`) — ✅ already landed on `blend-arc`.
- The SCV-removal sub-spec decides whether `find_shaper_max_accel`'s `scv` parameter stays; this spec passes `scv=0` and will adapt if the signature changes.

**This spec does not block:**
- Naive-CAM prepass sub-spec (independent; can run in parallel).
- Shake&Tune / shaper-calibrator-rework sub-spec (independent).

**This spec blocks:**
- Planner integration sub-spec — that sub-spec consumes the `ShaperBounds` output.
- Any Stage 1 wrap validation — the recipe here is the hardware gate.

## Open questions

1. **`find_shaper_max_accel` with `scv=0`**: the SCV-removal sub-spec will remove `scv` as an input. This spec pre-empts that by passing zero; the expected behavior is the same ceiling minus the linear-PA minimum-flow term. Cross-check this is correct in the implementation plan.
2. **Multi-mode shapers**: if the user's hardware has a secondary resonance outside the shaper's tuned frequency, Bound (c)'s window-averaging heuristic understates actual jerk. The validation recipe catches this empirically; analytical treatment is Stage 2.
3. **Peak-vs-average jerk**: Bound (c) uses the window-averaged jerk. Peak jerk inside the shaper's inter-impulse intervals can exceed the average; this is the Cole 2012 caveat. For the user's target regime (single sharp spike, 125–200 Hz) the secondary-lobe excitation is expected to be small; document and test.
4. **Iterative R**: the two-pass iteration assumes R doesn't change much between passes. If `R_tol` and `R_mid` both dominate, this is trivially true. If `j_eff` ever becomes the binding radius determinant (which would require changing `blend_geometry`'s radius selection — out of scope here), the iteration needs a convergence proof.

## Validation gate

Per Phase 0: hardware measurement on a representative printer before the Stage 1 branch merges to fork main. This spec defines the protocol; the gate itself lives at Stage 1 wrap.

## References

**Physics / math:**
- [Wikipedia — Jerk (physics)](https://en.wikipedia.org/wiki/Jerk_(physics))
- [Wikipedia — Circular motion](https://en.wikipedia.org/wiki/Circular_motion)
- [Wikipedia — Frenet–Serret formulas](https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas)
- Sparavigna, ["Jerk and Hyperjerk in a Rotating Frame," arXiv:1503.07051](https://arxiv.org/pdf/1503.07051)

**Input shaping:**
- Singer & Seering, "Preshaping Command Inputs to Reduce System Vibration," ASME J. Dyn. Sys. Meas. Control 112(1), 1990, [DOI 10.1115/1.2894142](https://doi.org/10.1115/1.2894142).
- Singhose, Porter, Tuttle, Singer, "Vibration Reduction Using Multi-Hump Input Shapers," ASME J. Dyn. Sys. Meas. Control 119(2), 1997, [DOI 10.1115/1.2801257](https://doi.org/10.1115/1.2801257).
- Cole, "A class of low-pass FIR input shaping filters achieving exact residual vibration cancelation," Automatica 2012, [link](https://www.sciencedirect.com/science/article/abs/pii/S0005109812002567).
- Singhose 2009, "Command Shaping for Flexible Systems: A Review of the First 50 Years," [DOI 10.1007/s12541-009-0084-2](https://doi.org/10.1007/s12541-009-0084-2).

**Motion-control prior art:**
- LinuxCNC blendmath.c: <https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/tp/blendmath.c>
- LinuxCNC INI docs: <https://linuxcnc.org/docs/devel/html/config/ini-config.html>
- Prunt kinematic_limiter.adb: <https://github.com/Prunt3D/prunt/blob/master/src/prunt-motion_planner-planner-kinematic_limiter.adb>
- Cho, Song, Lee et al., "Input shaping-based corner rounding algorithm for machining short line segments," Int. J. Adv. Manuf. Technol. 97, 2018, [DOI 10.1007/s00170-018-1922-0](https://link.springer.com/article/10.1007/s00170-018-1922-0).

**Kalico internals referenced:**
- `klippy/extras/shaper_defs.py` — shaper pulse definitions.
- `klippy/extras/shaper_calibrate.py` — `_get_shaper_smoothing`, `find_shaper_max_accel`, `TARGET_SMOOTHING`.
- `klippy/extras/input_shaper.py` — per-axis `AxisInputShaper` / `InputShaper` API.
- `klippy/blendmath.py` — consumer of `j_eff`; the adapter extension lives here.
