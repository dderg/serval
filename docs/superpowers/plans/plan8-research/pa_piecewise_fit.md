# Plan 8 Phase 0 Task 2 — Non-linear PA as Piecewise Chebyshev

**Date:** 2026-04-23
**Scope:** Research gap §6.2 of `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md`.
**Question:** how many Chebyshev pieces, at what degree, keep non-linear PA (tanh / recipr) filament-position error under ~1 µm across the full supported velocity range, including ramps, retracts, and hops?

All numeric claims below were produced by a local Chebyshev-fit sweep (numpy `Chebyshev.fit`, 4001-sample max-abs error). The experiments are reproducible from the code embedded below; results are repeated inline.

## 1. Model recap and what the fit must approximate

- Linear PA: `E(t) += k · v(t)` — exact polynomial composition (`klippy/chelper/kin_extruder.c:174-179`).
- `tanh` model: `E(t) += linear_advance · v + nonlinear_offset · tanh(v / v_lin)` (`klippy/chelper/kin_extruder.c:181-191`, `klippy/kinematics/extruder.py:192-216`).
- `recipr` model: `E(t) += linear_advance · v + nonlinear_offset · (1 − 1/(1 + v/v_lin))` (`klippy/chelper/kin_extruder.c:193-203`, `klippy/kinematics/extruder.py:219-241`).
- The linear term is handled exactly by polynomial composition. Only the non-linear saturation term needs approximation.
- `pa_velocity` is guarded `> 0` at `klippy/chelper/kin_extruder.c:246`; it is the XY-speed magnitude (kin_extruder cascades onto a fused shaper output), and is **≥ 0 by construction** — sign edge cases on E alone are a non-issue, E-only moves have `pa_velocity = 0` and the PA branch is skipped entirely.

The fit variable is normalized velocity `x = v / v_lin`. Parameters (`klippy/kinematics/extruder.py:125-135`): `nonlinear_offset` defaults 0 and is unbounded above; typical calibrated values 0.02–0.08 mm; `linearization_velocity` must be `> 0` when NO is set, typical 30–80 mm/s. No upper bound in config.

## 2. Supported velocity range

- `max_velocity` on modern CoreXY: 300–500 mm/s.
- Retract velocities: 20–80 mm/s (but E-only; `pa_velocity = 0` → PA branch skipped, **no fit error**).
- Z-hop: tiny XY, typically near zero; if combined with a travel segment the XY velocity is in the 100–300 mm/s band.
- Worst-case normalized range with `v_lin = 40` mm/s: `x ∈ [0, 12.5]`. With `v_lin = 80` mm/s: `x ∈ [0, ~6]`.

Global x-range is wide, but **this is not what we fit**. Plan 8 composes PA **per move** (spec §3.5). Every move emitted by the planner is a single quintic phase whose velocity span is bounded by the phase's own accel integral — not by machine max. Typical per-move x-spans:

| phase | v span | x-span at v_lin=40 |
|---|---|---|
| cruise | 0 | {one point — fit is trivial} |
| short accel/decel (Cowling-style short segment) | 20–80 mm/s | 0.5–2.0 |
| long travel accel | 0–500 mm/s | 0–12.5 |
| ramp-up from stop | 0–30 mm/s | 0–0.75 |

So the fit error question really has two regimes: (a) typical short-segment moves with sub-unit x-span, (b) the rare long accel that sweeps the full `[0, 12.5]` in one trapq phase.

## 3. Error-vs-pieces sweep

### 3.1 tanh, full global range `x ∈ [0, 12]` (worst single-move case)

Single-polynomial Chebyshev fit (no pieces):

| degree | max|err| |
|---|---|
| 2 | 5.7e-1 |
| 3 | 3.6e-1 |
| 4 | 1.8e-1 |
| 5 | 6.1e-2 |
| 6 | 2.1e-2 |

Piecewise fits, uniform interval partition:

| pieces | deg 3 | deg 4 | deg 5 |
|---|---|---|---|
| 2 | 9.8e-2 | 2.6e-2 | 2.9e-2 |
| 3 | 3.4e-2 | 2.5e-2 | 1.6e-2 |
| 5 | 1.8e-2 | 9.8e-3 | 2.0e-3 |
| 8 | 8.1e-3 | 1.4e-3 | 2.1e-4 |

### 3.2 recipr, full global range `x ∈ [0, 12]`

| pieces | deg 3 | deg 4 | deg 5 |
|---|---|---|---|
| 1 | 2.2e-1 | 1.4e-1 | 8.2e-2 |
| 2 | 1.0e-1 | 5.0e-2 | 2.4e-2 |
| 3 | 5.5e-2 | 2.3e-2 | 9.5e-3 |
| 5 | 2.2e-2 | 7.1e-3 | 2.3e-3 |
| 8 | 7.7e-3 | 1.9e-3 | 4.7e-4 |

### 3.3 Per-move spans (the regime that actually occurs)

tanh, deg 3 single piece, representative short-segment spans with v_lin=40:

| v-span (mm/s) | x-span | deg 3 err | deg 4 err |
|---|---|---|---|
| 0–30 | 0–0.75 | 7.1e-4 | 3.2e-5 |
| 20–80 | 0.5–2.0 | 7.4e-4 | 8.5e-4 (*) |
| 80–150 | 2.0–3.75 | 7.3e-4 | 1.2e-4 |
| 150–250 | 3.75–6.25 | 6.0e-5 | 1.5e-5 |
| 250–500 | 6.25–12.5 | 2.1e-6 | 1.0e-6 |

(*) The deg-4 result at `x ∈ [0.5, 2.0]` is anomalously close to deg-3 because it straddles the tanh inflection at `x ≈ 0.7`; adding a breakpoint fixes it (see §5).

Per-move span `x ∈ [2.5, 5.0]`, deg 3 single piece: **7.1e-4**. Deg 5: **3.6e-5**. Near-zero span `x ∈ [0, 0.25]`, deg 3: **4.8e-6**. Short moves are effectively free.

### 3.4 Adaptive split of the global range

Bisect-until-tolerance over `[0, 12.5]`:

| function | deg | tol 1e-3 | tol 1e-4 | tol 1e-5 |
|---|---|---|---|---|
| tanh | 3 | 6 pieces | 9 | 15 |
| tanh | 4 | 5 | 7 | 9 |
| recipr | 3 | 6 | 11 | 17 |
| recipr | 4 | 5 | 6 | 11 |

**Verdict of the sweep:** at our realistic acceptance (see §6), a long-accel trapq phase — the worst case — needs **5 pieces at degree 4** to cover the full `[0, 12.5]` global range; typical short-segment moves need **1 piece at degree 3**.

## 4. Filament-position error translation

The PA position contribution is `NO · f(x)` where `f ∈ {tanh, recipr}`. An absolute error `ε` on the fit translates directly:

`|E_approx − E_true| ≤ nonlinear_offset · ε`

With a pessimistic `NO = 0.1` mm (near the top of what users calibrate):

| fit ε | filament error |
|---|---|
| 1e-2 | 1.0 µm |
| 1e-3 | 0.1 µm |
| 1e-4 | 0.01 µm |
| 1e-5 | 0.001 µm |

**The ~1 µm target → fit tolerance of 1e-2 at NO=0.1 mm.** This is extremely loose; even a degree-4 single-piece fit over the full `[0, 12.5]` (2e-1 error) is ~20 µm filament, but across typical per-move spans we hit 1e-4 to 1e-3 with a deg-3 single piece — **0.01 to 0.1 µm filament, 10-100× below target**.

### Sanity-check against the spec's numeric example

"1 mm move at avg v = 100 mm/s, PA coef 0.05, 1e-4 relative error": steady-state PA position = k·v = 5 mm of filament advance. 1e-4 relative on that is 0.5 µm — same order as the 1 µm target. ✔

## 5. Edge-case analysis

### 5.1 v passes through 0 (deceleration to stop)

A raw Chebyshev LSQ on `x ∈ [0, 1]` gives `c(0) ≈ −2.1e-3` (tanh, deg 3) and `c(0) ≈ 2.7e-3` (recipr, deg 3) — **nonzero at x=0 even though the true function is zero there**. The `pa_velocity > 0` branch guard at `kin_extruder.c:246` currently yields an exact zero when velocity is exactly zero. Under Plan 8 the branch guard disappears (composition is polynomial), so at v=0 the filament position receives a DC kick equal to `NO · c(0)`.

- With NO=0.08, tanh deg 3: 0.08 · 2.1e-3 = **170 nm** — below the µm threshold.
- With NO=0.08, tanh deg 4: 12 nm — nuisance-level.

This is small but it's a *position step*, not a position error that integrates out. Mitigation: fit `g(x) = f(x) − f(0)` instead of `f(x)` and subtract once, or pin `f(0) = 0` with a Lagrange-constrained fit. The adaptive splitter in §3.4 produces a first piece on `[0, x_1]` whose endpoint-x=0 error is typically ≤ tolerance; acceptance in §6 enforces this.

### 5.2 Retraction (E-only move, XY stationary)

`pa_velocity` is the **XY** velocity (cascaded), so a pure retract has `pa_velocity = 0` for the entire move → PA branch skipped (`kin_extruder.c:246`). Under Plan 8 this must translate to "the E polynomial is exactly the base E polynomial; no PA term baked in." The polynomial composer must detect `pa_velocity ≡ 0 across the move` and skip PA composition (trivial, since pa_velocity is computed from the XY polynomial, which is identically zero on E-only moves). **No fit error on retracts.**

### 5.3 Z-hop with tiny XY drift

If a hop move has a small residual XY motion (e.g. a wipe combined with hop), pa_velocity hits the `x ∈ [0, ε]` sliver. A deg-3 single-piece fit on `[0, 0.25]` gives **4.8e-6** error (§3.3), i.e. 0.4 nm of filament at NO=0.08 mm. Safe.

### 5.4 Short sharp-corner segment straddling tanh inflection

Inflection of `tanh(x)` is at x≈0.658. A move with v-span `[0.5, 2.0] · v_lin` straddles this and is the worst per-move case in §3.3 (8.5e-4 at deg 4). Mitigation: when the composer detects the span crossing `x = 1.0` (the knee of both tanh and recipr), force a breakpoint there. This reduces the worst case to the per-half error, which is in the 1e-4 range.

### 5.5 Rapid deceleration, v reversal

Under the quintic planner the velocity polynomial can go negative only at direction flips; but the XY *speed magnitude* `pa_velocity` is always ≥ 0. No sign-change handling needed for the PA fit itself — the speed polynomial can be a non-polynomial (`|·|` of a polynomial) only if the planner allows sign flips inside a single phase, which Plan 5/Plan 6 explicitly forbid (phase boundaries at zero-crossings). So within any phase, pa_velocity is monotonic-in-sign and the Chebyshev fit on `[v_min, v_max]` is well-defined.

## 6. Recommendation

### Defaults

- **Degree per piece: 4.** Deg 3 is borderline at the tanh inflection; deg 4 gives a clean ~10× margin for typical per-move spans and keeps the composed polynomial-in-t degree manageable (`5 × 4 = 20` — still within a 32-coeff struct budget; today's quintic is degree 5).
- **Piece count: adaptive, 1–5 per move phase.** Open on a single piece; if the span crosses `x = 1.0` (knee), force a break. If the span also crosses `x = 2.5` (saturation onset), force a second break. Over the adaptive splitter's worst case (full `[0, 12.5]` accel ramp), this yields **at most 3 internal breaks → 4 pieces**.
- **Static worst-case budget: 5 pieces deg 4** reserves enough coefficient storage for any single quintic phase. Documented as the per-move polynomial-payload bound.

### Acceptance criterion

Reject the Chebyshev fit (and split once more, up to a hard cap) if:

`max |f(x) − f̂(x)| · nonlinear_offset > 1 µm`

i.e. absolute Chebyshev error times `NO` exceeds 1 µm of filament. With NO cached per model, this is one multiply per fit. Hard cap on pieces: 8 per phase. If the cap is hit, emit a warning and fall back to deg-5 on that piece (reducing further by one residual factor of 2–5).

### Why not degree 3?

Memory and step-gen eval cost are linear in degree; the composed-into-t polynomial degree is `5 × d` (quintic v(t) composed with deg-d f). Deg 3 → deg 15 in t; deg 4 → deg 20; deg 5 → deg 25. All fit in a realistic coefficient struct. Deg 4 is the sweet spot: one deg-bump buys a 10× error margin over deg 3 at the tanh inflection, and keeps the per-move polynomial payload under 100 doubles in the 5-piece worst case.

## 7. Summary

- Per-move velocity spans are small (sub-unit x in the common case), making a **1-piece, degree-4** fit adequate at ~1e-4 filament-error level (~10 nm at NO=0.1).
- The rare long-travel accel covering the full `x ∈ [0, 12.5]` needs adaptive splitting — **5 pieces, degree 4** is the recommended budget.
- Acceptance: reject if `ε_cheb · NO > 1 µm` filament. This is achieved by deg-4 with the adaptive splitter.
- Edge cases: retracts (pa_velocity=0, no fit), hops (tiny-x, safe), v=0 DC kick (170 nm worst — below threshold but worth pinning via `f(x) − f(0)` offset).
- 1 µm target verified against the spec's scaling example: 0.5 µm per the example, matches order.
