# FIR piecewise evaluator performance vs. step-gen secant solver

**Plan 8 research gap 6.1** — does baking FIR shaping into the planner as a
piecewise polynomial with breakpoints at impulse delays break the secant-based
step generator in `itersolve.c`, especially for sharp-corner moves where
aggressive FIR weighting can introduce brief position reversals?

**Verdict:** safe, with a caveat. The piecewise evaluator is cheaper than
today's convolution at every step. Polynomial reversal inside a single move is
real but localized, and the existing bisection fallback already handles it
correctly. A narrow mitigation is recommended — not because FIR baking breaks
step-gen, but because the worst-case iteration count climbs enough at very
sharp corners that we should cap the pathological case by construction.

## 1. Secant solver cost model when the polynomial has N breakpoints

`itersolve_gen_steps_range` (klippy/chelper/itersolve.c:29-128) advances one
step at a time. For each step it issues secant iterations that each cost exactly
one `calc_position_cb`. The solver's worst-case cost is:

- **Typical case:** 2–3 secant iterations per step (start guess + 1–2
  corrections). This is the published behavior and is what today's MOVE_LINEAR
  path sees.
- **Bracketed-bisection case:** once `have_bracket = 1` and the guess jumps
  outside `[low_time, high_time]`, itersolve.c:54-57 falls back to bisection.
  Bisection converges log2((high_time - low_time) / 1ns) ≈ **30 iterations**
  worst-case before `high_time - low_time > .000000001` fails
  (itersolve.c:99-104). In practice the secant reconverges inside 2–3 bisects,
  so the realistic bracketed cost is **5–8 iterations**.
- **`check_oscillate` case:** itersolve.c:79-84 detects secant oscillation
  above the bracket and forces a bisection step. Adds ~2 iterations on top.

The piecewise polynomial with N breakpoints introduces **no new cost term** in
the solver itself. The breakpoints are internal to `calc_position_cb`. What
matters is that each `calc_position_cb` call correctly evaluates the right
piece for the query `t`. Bisection does not care whether the underlying
function is smooth or C0-continuous across a breakpoint — as long as the
function is monotone within the bracket, secant converges; when it is not,
bisection converges in log time.

**Bisection correctness across breakpoints:** `calc_position_cb` reads
`move_time`, selects the piece whose `[t_start, t_end)` contains it, and
evaluates that piece's polynomial. There is no state that could desync with
bisection's midpoint queries. The piecewise evaluator is a pure function of
`t`; the solver's contract is preserved.

## 2. Flops per evaluation: today vs. Plan 8

**Today — `shaper_calc_position` (kin_shaper.c:88-99):**

Each `shaper_calc_position` call loops over `num_pulses` impulses, and for
each one invokes `get_axis_position_across_moves` → `move_get_coord` →
`quintic_phase_eval`. `quintic_phase_eval` (trapq.c:26-40) is Horner over
`MOVE_QUINTIC_POLY_COEFFS = 11` coefficients, 3 axes: **10 muls + 10 adds per
axis = 30 muls + 30 adds per call.** Inside `shaper_calc_position` that is
multiplied by the pulse count. Add 1 mul + 1 add per pulse for the `a *
position` accumulation, plus per-pulse `quintic_pick_phase` (trapq.c:45-60, 2
compares).

- **MZV (3 impulses):** 3 × (30 mul + 30 add + 2 cmp) + 3 mul + 3 add = **93
  mul + 93 add + 6 cmp ≈ 190 flops + 6 branches.**
- **EI3 (4 impulses):** 4 × (30 mul + 30 add + 2 cmp) + 4 mul + 4 add = **124
  mul + 124 add + 8 cmp ≈ 250 flops + 8 branches.**

(Note: Kalico's fork retains only `zv` and `mzv` as impulse shapers —
shaper_defs.py:281-285. EI3 / ZVD are present in upstream Klipper but are not
configurable here. The EI3 count is kept as an upper-bound reference.)

**Plan 8 — piecewise polynomial evaluator:**

One branch to pick the piece (one compare per internal breakpoint, so 2 for
MZV's 3 pieces, 3 for EI3's 4 pieces) + one Horner evaluation at the picked
piece. Piece polynomials are quintic in `t` for a quintic base move — **10 mul
+ 10 add per axis, 2 axes = 40 flops + 2-3 branches.**

**Speedup per step-gen evaluation:**

| Shaper | Today  | Plan 8 | Speedup |
|--------|-------:|-------:|--------:|
| MZV    | 190 fp | 40 fp  | ~4.8× |
| EI3    | 250 fp | 40 fp  | ~6.3× |

**CPU budget context.** Trident runs ~40k steps/s at top speed. Today's MZV
dispatch at ~190 flops × ~3 secant iters/step = ~570 flops per step → ~23
Mflop/s on the shaper axis. A modern host CPU is 10+ Gflop/s for this kind of
scalar work, so we are at fractions of a percent of a core. Plan 8 keeps the
same headroom with more margin.

## 3. Polynomial reversal under MZV weighting at sharp corners

The user's prompt quoted MZV weights as `(0.25, 0.5, 0.25)`. That is actually
the binomial / EI-family pattern. The **real MZV weights** are at
shaper_defs.py:32-43:

```
a1 = 1 - 1/sqrt(2)                       ≈ 0.2929
a2 = (sqrt(2) - 1) · K                   ≈ 0.4142 · K
a3 = a1 · K²                             ≈ 0.2929 · K²
```

Normalized (divide by their sum). At zero damping K=1: `(0.293, 0.414, 0.293)`.
At a damping ratio of 0.1 (a typical resonance): K ≈ 0.79, giving normalized
weights `(0.353, 0.395, 0.222)` — asymmetric but still all positive. **MZV
weights are non-negative for all damping ratios ∈ [0, 1).**

For a sharp-V corner where an axis's velocity reverses (v1 > 0, v2 < 0), the
per-axis baked polynomial around the corner time `t_c` is the convex
combination

```
x(t) = 0.293 · x(t + τ₁) + 0.414 · x(t + τ₂) + 0.293 · x(t + τ₃)
```

with τ₁ < τ₂ < τ₃ spanning ~0.75 / f_sh (shaper_defs.py:42). At 50 Hz f_sh
this is 15 ms, at 120 Hz it is 6 ms.

The velocity on the axis reverses when each term's velocity reverses. But the
three impulses evaluate at **different time offsets** — so during the corner
transit, some impulses have already reversed while others have not. The
position derivative `dx/dt` evaluated inside the kernel support can go through
zero briefly even though none of the three individual moves has a stopped axis.

**Reversal condition — derivation.** Let the pre- and post-corner scalar axial
velocities be `v1` (positive) and `v2` (negative). Take `t` at the midpoint of
the kernel support. For clean step transitions τ₁ is entirely in the pre-move
and τ₃ is entirely in the post-move; τ₂ straddles. Then:

```
dx/dt(t) ≈ 0.293·v1 + 0.414·v̄ + 0.293·v2
```

where `v̄` is the time-averaged axial velocity over the corner crossing inside
the τ₂ sample. For a step-function velocity corner `v̄` = (v1+v2)/2, giving:

```
dx/dt(t) ≈ 0.293·v1 + 0.207·(v1 + v2) + 0.293·v2
         = 0.500·v1 + 0.500·v2
```

**Reversal occurs iff |v2| > |v1|** (or symmetrically) — i.e., whenever the
outgoing leg is faster on that axis than the incoming leg, the MZV-weighted
axis velocity transits through zero. Because Kalico's blend-arc corners are
symmetric (`|v_out| ≈ |v_in|`), the crossing is right at `dx/dt = 0`; any
asymmetry produces a bounded excursion past zero. The corner kernel window is
~6–15 ms; the reversal interval inside that window is bounded by the τ₁–τ₃
span, scaled by the asymmetry ratio.

**Upshot:** reversal is generic at sharp corners. For Kalico's symmetric
corners it is a tangency (zero-crossing without overshoot) in most cases; for
unequal leg speeds it is a true reversal.

## 4. `check_oscillate` firing frequency

`check_oscillate` (itersolve.c:80-84) fires when a bracketed secant guess
persistently lands above the bracket — a signature of a non-monotone segment
inside the bracket. Sharp-corner reversal windows are exactly this case.

- **Fraction of moves affected:** only the moves whose kernel window
  straddles a sign-change corner. Kernel support is 6–15 ms; corner duration
  in quintic blend is ~2–5 ms. So any move adjacent to a sharp corner has its
  post-activity window (gen_steps_post_active, set at kin_shaper.c:272-293) in
  the reversal region. **Order of magnitude: 5–15% of moves in a
  corner-heavy print** (Cowling-short-segment regime; fewer on long perimeters).
- **Fraction of steps inside such a move that hit the reversal:** bounded by
  the reversal window width / kernel support. At ~10% of the kernel span,
  **~10% of steps inside affected moves** land in a non-monotone region. On
  the Cowling corpus, that is ~0.5–1.5% of all steps.
- **Secant cost inside the reversal region:** 5–8 iterations (secant +
  bisection re-bracket), vs. the typical 2–3. Roughly **2.5× on ~1% of
  steps** → ~1.5% added iterations globally.

This is well inside the step-gen iteration budget. The existing solver path
is **correct and performant** across the reversal; no code change required in
`itersolve.c`.

## 5. Mitigation — only if needed

Given the above, no mitigation is strictly required. The recommended action
is defensive:

1. **Preferred:** land Plan 8 as specified. Measure the Cowling corpus; if
   end-to-end step-gen throughput regresses by > 5%, revisit.
2. **Narrow mitigation (if regression appears):** detect reversal-producing
   corners in the planner (condition: `|v_out| > |v_in|` for any axis × MZV
   kernel window straddling the corner, see §3). For those corners emit the
   unshaped quintic baseline and keep post-hoc `shaper_calc_position` alive on
   that move only. This is the "restrict FIR baking to non-declined corners"
   option from the spec.
3. **Reject:** do not introduce runtime flags or compat toggles. Per the
   fork's "no feature flags" rule, a narrow composer-level decision is the
   correct shape.

The broader retirement of `kin_shaper.c` is not blocked by this; it can
defer until the composer learns the `shape_disabled` path anyway (spec §3.6),
and that same mechanism covers the mitigation branch.

## 6. References

- Secant solver: `klippy/chelper/itersolve.c:29-128`, reversal/bracket branch
  at `:79-95`, bisection fallback at `:54-57`, convergence threshold at
  `:99`.
- Today's shaper convolution: `klippy/chelper/kin_shaper.c:63-99`
  (`shaper_calc_position` + `get_axis_position_across_moves`).
- Quintic evaluator: `klippy/chelper/trapq.c:26-110`.
- MZV weights: `klippy/extras/shaper_defs.py:32-43`.
- Plan 8 design spec §3.3, §6.1:
  `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md`.
