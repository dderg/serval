# Plan 8 Chunk 3 — Bake extruder shape + pressure advance into planner

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Planner emits an E (extruder) polynomial alongside each XY polynomial, with the input-shaping kernel AND pressure advance already composed into it. Post-hoc extruder convolution (`kin_extruder.c`) retires.

**Architecture:**

After Chunk 2, the planner emits XY as a piecewise polynomial with `n_phases` sub-intervals (up to 32). Chunk 3 adds an E polynomial over the **same breakpoints**, with coefficients computed as:

For **linear PA** (`pressure_advance_model = linear`):
```
E(t) = (integrated XY arc-length polynomial) + k * (XY velocity polynomial)
```
Both operands are polynomials already on hand. This is exact polynomial arithmetic, no approximation.

For **non-linear PA** (`tanh`, `recipr` models — per `kinematics/extruder.py:176-240`, `kin_extruder.c:182-203`):
```
E(t) = E_linear(t) + nonlinear_offset * f(v_xy(t) / v_lin)
```
where f is `tanh` or `1 − 1/(1 + ·)`. Since f composed with a polynomial is **not a polynomial**, we fit the nonlinear correction piecewise using Chebyshev polynomials per Phase 0 §6.2:
- Degree 4
- 1-5 pieces per existing XY sub-interval (adaptive on velocity-span breakpoints at `v = v_lin` and `v = 2.5 * v_lin`)
- Reject if `max|ε_cheb| * nonlinear_offset > 1 µm` of filament

The cascade identity is preserved **by construction**: both XY and E are computed from the same source `QuinticShape` velocity polynomial. Any numerical drift between them is impossible.

**Spec:** `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md` §7 Chunk 3.

**Research:**
- `docs/superpowers/plans/plan8-research/pa_piecewise_fit.md` — Chebyshev fit parameters
- `docs/superpowers/plans/plan8-research/bs_polynomial_composer.md` — piecewise structure

**Branch:** `magnum-opus`. Commit with actual system time. NO Co-Authored-By trailers.

**Prerequisites:** Chunk 2 complete. Struct move carries piecewise XY polynomial with per-phase coefficients.

---

## Task structure

Chunk 3 has three stages:

- **Stage A** (Tasks 1-4): E polynomial slot in struct move + linear PA composer
- **Stage B** (Tasks 5-7): non-linear PA piecewise Chebyshev composer
- **Stage C** (Tasks 8-11): retire `kin_extruder.c` convolution, wire extruder step-gen to read E polynomial, final regression

---

## Stage A — Struct slot + linear PA baking

### Task 1: Add E polynomial storage to struct move

**Files:**
- `klippy/chelper/trapq.h` — add E polynomial per-phase coefficient slot
- `klippy/chelper/trapq.c` — update `trapq_append_quintic` to accept E coefficients
- `klippy/chelper/__init__.py` — update FFI signature

**Design:** The `struct move_quintic_phase` currently holds XY coefficients in `c[k].x, c[k].y, c[k].z`. The `.z` slot is used. Add a fourth axis: **`.e`**, making `struct coord` a 4-wide { x, y, z, e } vector.

```c
struct coord {
    union {
        struct { double x, y, z, e; };
        double axis[4];
    };
};
```

This adds one double per phase-coefficient × axis × piece. Memory impact: `32 pieces × 15 coeffs × 1 new axis = 480 extra doubles = 3.8 KB per move`. Total move size grows from ~15 KB to ~19 KB. Fine.

- [ ] **Step 1** — Grep every reader of `struct coord` to understand blast radius:

```bash
grep -rn 'struct coord\|\.axis\[\|\.x.*,.*\.y.*,.*\.z' klippy/chelper/ | head -30
```

- [ ] **Step 2** — Change `struct coord` to 4-wide. Most sites that read `.x/.y/.z` are unaffected; sites that use `.axis[3]` (array-style) need to know axis 3 now means E, not "past end".

- [ ] **Step 3** — `move_axis_phase_polynomial` takes `axis = 'x'/'y'/'z'/'e'` (today only xyz). Extend to accept 'e'.

- [ ] **Step 4** — `trapq_append_quintic` signature: `coeff_buf` length grows from `n_phases * 15 * 3` to `n_phases * 15 * 4`. Update callers (`blendquintic.py compose_phase_polynomials`).

- [ ] **Step 5** — For unshaped / unshaped+unsupported-PA moves, E coefficients default to `axes_r.e * (start_v * tau + 0.5 * accel * tau²)` (same form as X/Y/Z, just with the extruder axis ratio) — equivalent to today's `linear_as_quintic` behavior for the extruder axis.

- [ ] **Step 6** — Rebuild, run existing tests. Expected: degenerate-quintic emits unchanged E polynomial (matches today's trapq_append output for the extruder channel via kinematics/extruder.py:772).

- [ ] **Step 7** — Commit:

```
chunk3: struct coord gains .e axis — E polynomial storage in every phase
```

---

### Task 2: Linear PA composer

**Files:**
- Create: `klippy/chelper/linear_pa_compose.h`, `.c`
- Update `blendquintic.py` to call the composer when PA is configured with `model = linear`

**Math:**

Given XY piecewise polynomial (after bs_compose / fir_compose / pass-through) and linear PA coefficient `k`:

For each phase:
1. Compute v_xy(t) = derivative of XY position polynomial (per-axis)
2. Compute XY arc-length polynomial (integral of |v_xy(t)| — for straight segments this is `sqrt(r_x² + r_y²) * distance`; for curved quintic blends this is more involved but already computed by the Plan 5 blendmath as the arc_length scalar)
3. E(t) = scaled_arc_length(t) + k * v_xy_projected(t)

Where scaled_arc_length is the extruder advance per XY distance (the `extr_r` factor from `kinematics/extruder.py:772`), and v_xy_projected is the XY speed projected onto the extruder motion direction.

For straight segments (axes_r has a constant direction), this simplifies to coefficient-wise operations:
```
E_c[k] = extr_r * X_c[k] + k_pa * (k+1) * X_c[k+1]   (for k = 0 .. max-1)
E_c[max] = extr_r * X_c[max]                           (velocity term falls off top end)
```
Where k_pa is the PA coefficient and the derivative `(k+1) * c[k+1]` is the standard polynomial-derivative rule.

- [ ] **Step 1** — Implement `linear_pa_compose` taking XY piecewise polynomial + PA params, producing E coefficients per phase. Pure polynomial arithmetic.

- [ ] **Step 2** — Unit tests in `test/test_linear_pa_compose.py`:
  - Pure cruise at v=50 mm/s with PA=0.05: E velocity matches v (no pa contribution at constant v) + steady extruder rate
  - Accel ramp: E has PA kick during accel (c[2] reflects PA contribution)
  - Zero-PA: E equals scaled XY

- [ ] **Step 3** — Commit:

```
chunk3: linear_pa_compose — linear PA polynomial bakery
```

---

### Task 3: Wire linear_pa_compose into the planner emit path

**Files:**
- `klippy/blendquintic.py` — after bs_compose / fir_compose, run linear_pa_compose on the result to produce E
- `klippy/kinematics/extruder.py` — the extruder-side data pipeline sends PA params to the planner

- [ ] **Step 1** — Thread PA config (model + coefficient + nonlinear_offset + linearization_velocity) from `kinematics/extruder.py` through to the CornerBlender.

- [ ] **Step 2** — In `compose_phase_polynomials`, after XY baking, call `linear_pa_compose` for linear PA; stub for non-linear (next Task 5).

- [ ] **Step 3** — Integration test: accel move with PA=0.05 linear, compare E-motion at step times to today's kin_extruder.c output. Tolerance: <0.1 µm filament.

- [ ] **Step 4** — Commit:

```
chunk3: planner emits linear-PA-baked E polynomial
```

---

### Task 4: Update `kinematics/extruder.py:772` emit to use new pipeline

**Files:**
- `klippy/kinematics/extruder.py`

The current emit at `:772` uses `append_trapezoid_as_quintic` (post-Chunk-1) which emits unshaped polynomial. Now the polynomial should come from the CornerBlender — extruder just reads the planner's E polynomial.

- [ ] **Step 1** — For **kinematic moves** (is_kinematic_move), E motion is already in the planner's polynomial. The extruder's emit site just syncs its trapq to the toolhead's — no separate trapq_append needed.

- [ ] **Step 2** — For **extrude-only moves** (is_kinematic_move is False), planner emits E-only polynomial with XY.e component only.

- [ ] **Step 3** — Verify via test: extrude-only move produces identical stepper output to today.

- [ ] **Step 4** — Commit:

```
chunk3: extruder.py consumes planner E polynomial directly
```

---

## Stage B — Non-linear PA (tanh, recipr)

### Task 5: Implement Chebyshev piecewise fitter

**Files:**
- Create: `klippy/chelper/cheb_fit.h`, `.c`
- `klippy/extras/cheb_fit.py` — Python wrapper

**Math per pa_piecewise_fit.md:**
- Input: function `f(v)` on interval `[v_lo, v_hi]`
- Output: degree-4 Chebyshev polynomial approximation
- Adaptive split at specified v breakpoints (for tanh: v = v_lin, v = 2.5 * v_lin)
- Error bound: return max|ε_cheb|; caller rejects fit if above threshold

- [ ] **Step 1** — Implement degree-4 Chebyshev fit. Use closed-form coefficients (5 Chebyshev nodes of second kind).

- [ ] **Step 2** — Adaptive split: given a breakpoint set, return piecewise fit over each sub-interval.

- [ ] **Step 3** — Mitigation for v=0 edge wart per research: fit `f(v) - f(0)` then add `f(0)` as a post-composition correction (keeps `f_approx(0) = f(0)` exact).

- [ ] **Step 4** — Tests: tanh fit over [0, 12.5] with breakpoints at {1, 2.5} — 5-piece fit, max error < 2e-4.

- [ ] **Step 5** — Commit:

```
chunk3: cheb_fit — degree-4 Chebyshev piecewise polynomial fitter
```

---

### Task 6: Nonlinear PA composer

**Files:**
- Create: `klippy/chelper/nonlinear_pa_compose.h`, `.c`

**Approach:**
1. For each XY phase piece, compute v_xy(tau) polynomial (degree ≤ 14).
2. Determine velocity range `[v_min, v_max]` over the phase (polynomial extrema — easy via derivative roots or just interval arithmetic).
3. Fit `nonlinear_offset * f(v / v_lin)` as piecewise Chebyshev over this v-range with split at `v_lin` and `2.5 * v_lin` if they fall inside.
4. Compose: E_nonlinear(tau) = fit(v_xy(tau)). Since the Chebyshev fit is a polynomial in v, and v_xy(tau) is a polynomial in tau, the composition is polynomial arithmetic. Piece boundaries in `tau` arise where `v_xy(tau)` crosses a v-split point.

- [ ] **Step 1** — Implement.

- [ ] **Step 2** — Unit tests: accel ramp through v=v_lin, verify E position matches today's kin_extruder.c output within 1 µm.

- [ ] **Step 3** — Acceptance enforcement: if Chebyshev error * nonlinear_offset > 1 µm filament, fall back to a finer fit (degree up to 6, or more pieces). If still can't meet threshold, log a warning.

- [ ] **Step 4** — Commit:

```
chunk3: nonlinear_pa_compose — tanh / recipr PA via piecewise Chebyshev
```

---

### Task 7: Wire nonlinear_pa_compose into emit

**Files:**
- `klippy/blendquintic.py`

- [ ] **Step 1** — Dispatch: `pa_model == 'linear'` → `linear_pa_compose`; `pa_model in ('tanh', 'recipr')` → `nonlinear_pa_compose`.

- [ ] **Step 2** — Integration test: `pa_model = tanh`, accel through saturation, compare to today's output.

- [ ] **Step 3** — Commit:

```
chunk3: planner emits tanh / recipr PA via piecewise Chebyshev
```

---

## Stage C — Retire `kin_extruder.c` convolution + final wiring

### Task 8: Extruder step-gen reads polynomial directly

**Files:**
- `klippy/chelper/kin_extruder.c`

**Goal:** Delete the PA convolution loop (`pa_range_integrate`, `shaper_pa_range_integrate`, `pa_move_integrate`). `extruder_calc_position` becomes a straight polynomial eval on axis='e'.

- [ ] **Step 1** — Rewrite `extruder_calc_position` to delegate to `move_get_coord` with axis='e'.

- [ ] **Step 2** — Delete `pa_range_integrate`, `shaper_pa_range_integrate`, `pa_move_integrate`, `struct smoother sm[3]`, `struct shaper_pulses sp[3]` from `struct extruder_stepper`.

- [ ] **Step 3** — Keep the PA model struct `pressure_advance_params` — it's still read by the planner for config, not at step-gen time. The `pa_func` pointers can be deleted if unused.

- [ ] **Step 4** — FFI binding cleanup in `__init__.py`: remove `extruder_set_smoother_params`, `extruder_set_shaper_params`. Verify Python callers all gone.

- [ ] **Step 5** — Rebuild; full regression. Expected: extruder tests pass; any test that exercises the old fused kernel needs rewriting.

- [ ] **Step 6** — Commit:

```
chunk3: kin_extruder.c retires convolution — step-gen is polynomial eval on axis=e
```

---

### Task 9: Audit remaining `kin_extruder.c` code for PA still needed

**Files:**
- `klippy/chelper/kin_extruder.c`

Some parts of kin_extruder.c might still be needed (stepper kinematic callback, step generator init, trapq-register). Audit what stays vs goes.

- [ ] **Step 1** — List every public function in `kin_extruder.c`. Classify: stays (step-gen infrastructure) or goes (convolution).

- [ ] **Step 2** — Delete what goes. Leave a thin file with just the step-gen wrapper.

- [ ] **Step 3** — Commit:

```
chunk3: kin_extruder.c slimmed to step-gen wrapper
```

---

### Task 10: Test rewrite

**Files:**
- `test/test_blendextruder.py` — main PA test file
- `test/test_plan5_integration.py` — cascade-identity tests

- [ ] **Step 1** — Rewrite cascade-identity tests: verify planner-baked E polynomial matches planned XY to <1 µm across sample points. The "cascade through two kernel stages" concept no longer applies — there's ONE composer that produces both.

- [ ] **Step 2** — Rewrite linear-PA test: planner emits E polynomial; evaluate at step times; compare to today's output.

- [ ] **Step 3** — Rewrite tanh/recipr tests: same pattern.

- [ ] **Step 4** — Commit:

```
chunk3: rewrite PA tests against baked-in planner
```

---

### Task 11: Final sim + kalico regression

- [ ] **Step 1** — klipper-sim: `pytest test/`. Expected: all pass.

- [ ] **Step 2** — Kalico: `pytest test/ --ignore=test/klippy`. Expected: same pre-existing skip/fail counts as pre-Chunk-3.

- [ ] **Step 3** — Summary commit with any regressions documented.

- [ ] **Step 4** — (user-driven, not gated) HW test on Trident with a PA-sensitive pattern.

---

## Exit criteria

- `kin_extruder.c`'s convolution loops are deleted.
- Every extruder move produces its E polynomial at planner-emit time, composed with the configured PA model.
- Linear PA is exact; tanh / recipr PA is piecewise Chebyshev with <1 µm filament error.
- Cascade identity is preserved by shared velocity polynomial source.
- All target tests pass. klipper-sim + kalico suite pass.
- No post-hoc extruder convolution remains.

## Not in Chunk 3

- New PA models beyond linear / tanh / recipr.
- Extruder hardware limits (`max_extruder_accel`, `max_extruder_rpm`) — Plan 3 scope.
- Velocity-cap re-derivation from the baked polynomial (falls out naturally; if issues, follow-up task).

---

## Summary of Plan 8 completion at Chunk 3 end

When Chunk 3 lands, the complete Plan 8 end-state is reached:

- Every move in the trapq is a unified piecewise polynomial.
- The planner emits motion with input shaping + pressure advance both baked in.
- `kin_shaper.c` is deleted.
- `kin_extruder.c` is a thin step-gen wrapper (~50 lines vs ~350 today).
- The 3 pillars of Magnum Opus are complete on the XY side; extruder hardware limits (Pillar 3 extended scope) are follow-up.

Ready for HW validation and future extensions (ethercat / real-time stepper, phase stepping) to build on a clean, unified motion representation.
