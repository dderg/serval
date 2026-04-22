# Plan 5 — Direct-quintic step generation + Pillar 1 feedforward inverse shaper

**Date**: 2026-04-22
**Branch**: `magnum-opus`
**Pillars**: Pillar 1 (feedforward inverse shaper) + Pillar 2 completion (direct-quintic step generation)
**Status**: design

## Goal

Retire the polyline intermediate representation that the quintic corner
primitive currently produces (`klippy/blendplanner.py::CornerBlender._emit_blend`)
and feed the quintic to step generation as a first-class C^2-continuous
primitive. Simultaneously land the feedforward inverse input shaper so
that the commanded trajectory after the full cascade (`h ⊛ w ⊛ x_planned`)
equals the planned path in the motion passband — i.e. the toolhead
actually traces what we planned, rather than a low-pass-smeared
approximation of it.

The two pieces couple at the stepper-query layer in
`klippy/chelper/kin_shaper.c` and share the same lookahead-window
extension. Bundling them into one plan lets us ship one coherent
change, HW-test once, and avoid interim states where either piece is
non-trivially degraded.

## Context

After Plan 4, the state on `magnum-opus` is:

- Quintic corner primitive (`klippy/blendquintic.py`) works, produces
  a C^2-continuous `QuinticShape` with analytic curvature, `v_cap_fn`,
  and polyline subdivision.
- `CornerBlender._emit_blend` (`klippy/blendplanner.py`) converts the
  quintic to a polyline of linear sub-moves and appends them to
  `trapq` as ordinary trapezoidal moves. This polyline is the
  intermediate representation Plan 5 removes.
- Smooth-IS shaper family (`smooth_zv` / `smooth_mzv` / `smooth_ei` /
  `smooth_2hump_ei` / `smooth_zvd_ei` / `smooth_si`) is the Kalico
  forward path. Plan 4 wired the smooth-IS velocity cap into the
  quintic's `v_cap_fn` via `_compute_A_axis_smooth_is`.
- The classic FIR shapers (`zv` / `mzv` / `ei` / `zvd` / `2hump_ei` /
  `3hump_ei`) remain in tree as secondary options; Plan 5 does **not**
  touch them.
- Plan 3's extruder-cap plumbing (`blendextruder.cap_move`) is live.
- `trapq`'s only primitive is the linear-velocity trapezoid move
  (`klippy/chelper/trapq.h:15-21`); the `calc_position_cb` query
  interface (`klippy/chelper/itersolve.h:12`) returns a scalar axis
  position. The smoother-integration path in
  `klippy/chelper/integrate.c:51-63` computes three polynomial moments
  assuming exactly this linear form — any direct-quintic design must
  extend it.

## Research anchors

Four research artifacts produced in advance of this spec, all under
`docs/superpowers/plans/plan5-derivations/`:

- `direct_quintic_architecture.md` — trapq/step-gen audit plus three
  architecture options for direct-quintic. Recommendation: **Option A**
  (tagged union inside `struct move`). Options B (parallel curveq) and
  C (generic degree-5 polynomial) both hit blockers.
- `fir_companion_kernel.md` — failed-inversion analysis for the old
  smooth-IS family. Finding: 5 of 6 kernels have spectral zeros inside
  the passband, making finite-support FIR inversion mathematically
  impossible. Motivation for the replacement family below.
- `new_shaper_family.md` — cardinal B-spline chain family (Besset &
  Béarée 2017) as replacement. Five variants `bs1` through `bs5`
  controlled by a single integer order `m ∈ {1, …, 5}`. Closed-form
  forward kernel, closed-form FIR inverse, `G = ‖h‖₁ ≤ 2.84`
  worst-case.
- `saturation_feedback.md` — planning-side cap derivation (L¹-L∞
  bound on `h ⊛ ẍ`). Final formula corrected in
  `per_axis_saturation_derivation.md` to
  `v_sat(s) = sqrt(a_max / (G_worst(s) · κ(s)))` with
  `G_worst(s) = max_axes G_axis · (|proj_t|+|proj_n|)`.
- `lookahead_extension_audit.md` — `step_generation_scan_time`
  plumbing plan. Additive-only, ~175 LOC, no structural rewiring.
  Highest risk: `t_h == 0` guard for disabled-shaper configs.

## Why now

**The polyline is architecturally dead weight.** Every Plan-4
deliverable that protected the polyline boundary (bandwidth cap,
per-sub-move extruder cap) was reverted at user request on 2026-04-21
as "dead code for architecture we don't plan to ship." Continuing to
emit polyline moves is paying the cost (κ-step discontinuities at
every boundary, HF jerk content, segment-count proportional to curve
length) for a representation we intend to remove.

**The existing smooth-IS family is a dead end for feedforward.** The
Pillar 1 research round proved that 5 of 6 smooth-IS kernels cannot
be FIR-inverted — the cascade identity is mathematically impossible
because of spectral zeros inside the passband. Shipping Pillar 1 on
the existing family means either `smooth_zv`-only support or no
feedforward at all.

**The feedforward inverse is what makes direct-quintic worthwhile.**
Without Pillar 1, the toolhead still traces a shaper-smeared version
of whatever we plan. Direct-quintic cleans up HF content at the
segment boundaries, but the fundamental tracking error (corner
fidelity ≠ planned fidelity) remains. With Pillar 1, the cascade is
identity-in-the-passband and the toolhead traces exactly what the
planner produced. This is the magnum-opus corner-fidelity
differentiator.

**EtherCAT forward-compatibility.** Position-command servo drives
(EtherCAT, CANopen 402) track command signals exactly, unlike
microstepped step/dir drivers which mask small shaping errors below
the step granularity. A closed-form parametric trajectory (quintic)
plus a closed-form fused shaper kernel maps naturally onto a
1-10 kHz PDO position stream. The architecture shift in Plan 5 is
the foundation for that future work.

## Scope

### In scope (7 deliverables)

1. **[D1] Replace the smooth-IS shaper family with the cardinal
   B-spline chain (`bs1`-`bs5`).** Includes `struct smoother`
   piecewise redesign.

2. **[D2] Direct-quintic step generation.** Extend `trapq`'s
   `struct move` with a quintic primitive via tagged union. Emit
   position-in-t polynomials per phase (accel / cruise / decel) at
   emit time — skip compositional-at-query-time approach. Rewrite
   `smoother_antiderivatives` from 3 to 11 moments. Dispatch in
   `trapq.c` + review `kin_shaper.c` and `kin_extruder.c` for
   direct struct access (other 7 kinematics files need zero
   changes). Swap `blendplanner._emit_blend` to emit a single
   quintic trapq entry. Update klipper-sim deserializer in the
   same batch.

3. **[D3] Feedforward inverse shaper (Pillar 1), all axes.**
   Closed-form FIR inverse per B-spline variant, fused with forward
   kernel (`k_fused = h ⊛ w`) at `kin_shaper.c` query layer. Applied
   to XY **and E axes** for PA-sync coherence.

4. **[D4] Saturation cap.** `v_cap_fn` replaces `a_max` with
   `a_max / G_worst` where `G_worst = max proj |axis| · ‖h_axis‖₁`
   across shaped axes. `AxisShaperSnapshot.A_axis` also becomes
   `a_mech_max / G` so rotation-jerk cap absorbs the correction.

5. **[D5] Lookahead extension.** Widen `step_generation_scan_time`
   to accommodate `T_sm + T_h = 3·T_sm`. Add `t_h == 0` guard for
   disabled-shaper configs.

6. **[D6] Config migration.** Old `shaper_type = smooth_*` values
   error with a clear message mapping to the new `bs*` equivalent.
   No back-compat aliases (fork-as-gate policy). Frontend
   coordination (Mainsail, Fluidd, Moonraker) announced before
   landing.

7. **[D7] Unified v(s) along the curve (Pillar 2b).** Per-s velocity
   profile for the quintic trapq entry. Absorbs Plan 3's extruder
   cap into `v_cap_fn(s)` directly (no separate `cap_move` pass).
   Boundary matching with outer lookahead. See
   `plan5-derivations/unified_v_of_s.md` for the derivation
   (pending research-subagent completion at spec-write time).

### Out of scope (deferred)

- **Classic FIR shapers in tree** (`zv`, `mzv`). These stay unchanged;
  Pillar 1 inverse does not apply to impulse trains (no
  finite-support FIR inverse exists for an impulse train). Note:
  `ei`, `zvd`, `2hump_ei`, `3hump_ei` have already been removed from
  `INPUT_SHAPERS` in `klippy/extras/shaper_defs.py` — only `zv` and
  `mzv` remain on the FIR path.
- **EtherCAT integration.** The architecture this plan establishes
  supports future EtherCAT work, but wiring up an actual EtherCAT
  backend is a separate project.
- **Shape-pluggable strategy interface.** Quintic stays hard-wired
  as the blend primitive.
- **Adaptive per-move shaper variant selection.** Choosing `bs1` for
  sharp corners and `bs5` for smooth features on a per-move basis
  is a future optimization; Plan 5 ships a single variant per config.

**Extruder inverse is now IN scope.** Earlier spec revisions scoped
the extruder out with the argument "extruder follows physical path
via PA, not stepper-commanded position." An adversarial review
showed this argument conflates PA's physical-path compensation with
the shaper convolution that `kin_extruder.c::extruder_calc_position`
(`:184-226`) runs on the E stepper's commanded position. Post-D1,
the extruder uses the same `bs*` forward kernel as the XY axes. If
XY gains a feedforward inverse and E does not, the physical XY
traces the plan faithfully while E lags by ~`T_sm/2` — breaking
PA phase synchronization. D3 therefore covers **all** axes (XY +
E); one fused kernel `k_fused = h ⊛ w` is precomputed per
shaper-reset and applied by every consumer of `struct smoother`.

**All kinematics are now IN scope for D2b.** Earlier spec revisions
deferred polar and rotary_delta. Review expanded the fanout to
include `kin_corexz.c`, `kin_deltesian.c`, `kin_idex.c`,
`kin_winch.c` (IDEX in particular has real Kalico users). D2b
dispatches `m->kind` in every `kin_*.c::*_stepper_kinematics_calc_position`
that currently reads `move_get_coord`. See D2b details below.

## Deliverable details

### D1. Cardinal B-spline chain shaper family

**Rationale.** See `new_shaper_family.md` for full derivation. The
B-spline chain has no spectral zeros in `[0, f_sh]` when `T_sm >
(m+1) / f_sh`, which is the necessary condition for a finite-support
FIR inverse. Single integer parameter `m` smoothly interpolates
from narrow/fast/weak (bs1) to wide/slow/strong (bs5).

**Per-variant parameters** at `f_sh = 40 Hz`, `ζ = 0.1`, 5% residual
target:

| variant | order `m` | stages `m+1` | `T_sm` [ms] | `F_m = T_sm · f_sh` |
|---------|----------:|-------------:|------------:|--------------------:|
| bs1     |         1 |            2 |       38.88 |              1.5553 |
| bs2     |         2 |            3 |       48.65 |              1.9462 |
| bs3     |         3 |            4 |       56.30 |              2.2519 |
| bs4     |         4 |            5 |       62.65 |              2.5061 |
| bs5     |         5 |            6 |       68.13 |              2.7252 |

**Forward kernel** (Curry-Schoenberg 1966 Theorem 2, equivalent to
Unser-Aldroubi-Eden 1993 eq. 10): cardinal B-spline of order `m` on
canonical support `[0, m+1]`, rescaled to `[-T_sm/2, +T_sm/2]` with
unit integral. Closed form per sub-interval, stored as piecewise
polynomial coefficient list in the existing `struct smoother`
representation (`klippy/chelper/integrate.h:8-13`) — the family fits
the existing data structure if we extend from a single polynomial
piece per kernel to `m+1` pieces (piecewise form). Alternative: sample
and refit as a single higher-degree polynomial on the full support —
feasible for bs1/bs2 (degree ≤ 2), increasingly poor conditioning for
bs5 (degree 5 over a wide window). **Chosen: piecewise form.** Extend
`struct smoother` to hold an array of piece descriptors.

**Inverse kernel**: Besset-Béarée 2017 §III eq. 17-18 gives the
closed-form FIR inverse on the sub-band where `|W| > 0`. Support
`T_h = 2 · T_sm` per variant, cosine-taper windowed. Measured per
`new_shaper_family.md §4.3` at default conservative passband
`pb_max = 0.3 · f_sh = 12 Hz`:

| variant | `T_h` [ms] | `G = ‖h‖₁` | passband err on [0, 0.3·f_sh] | HF amp sup\|H(ω)\| on [f_sh, 3·f_sh] |
|---------|-----------:|-----------:|------------------------------:|-------------------------------------:|
| bs1     |      77.77 |      1.933 |                         4.79% |                                 0.08 |
| bs2     |      97.31 |      1.921 |                         3.13% |                                 0.05 |
| bs3     |      112.6 |      2.003 |                         3.17% |                                 0.04 |
| bs4     |      125.3 |      1.991 |                         3.28% |                                 0.04 |
| bs5     |      136.3 |      1.951 |                         0.54% |                                 0.03 |

**Worst-case `G` across all variants: 2.00.** At the more aggressive
wider passband `pb_max = 0.5 · f_sh = 20 Hz`, `G_worst = 2.84` (bs4).
HF amplification below 1 for all variants — the cosine taper prevents
amplification above `f_sh`, a win over naive Tikhonov-regularized
inverses.

**Fused kernel optimization.** Since the inverse always composes
with the forward (`k_fused = h ⊛ w`), the query-time implementation
precomputes `k_fused` once at shaper-reset and applies it as a single
convolution. Runtime cost is one `smoother_calc_position` call with
a kernel of support `T_sm + T_h = 3 · T_sm` (up from `T_sm` for the
forward-only path). See D3 below for integration with `kin_shaper.c`.

**A_axis per variant.** The B-spline second central moment has
exact closed form `σ²_T = T_sm² / (12 · (m+1))` (Unser-Aldroubi-Eden
1993 eq. 13; verified numerically in `new_shaper_family.md §10`
`verify_all()`). Combined with `A_axis = 2 · target_smoothing / σ²_T`
from `docs/superpowers/plans/plan4-derivations/A_axis_smooth_is.md`:

| variant | `m` | `σ²_T / T_sm²` | A_axis @ ts=0.12, f=40Hz |
|---------|----:|---------------:|-------------------------:|
| bs1     |   1 |        0.04167 |                     3810 |
| bs2     |   2 |        0.02778 |                     3650 |
| bs3     |   3 |        0.02083 |                     3635 |
| bs4     |   4 |        0.01667 |                     3668 |
| bs5     |   5 |        0.01389 |                     3723 |

A_axis is **non-monotone with minimum at bs3** (3810 → 3650 → 3635
→ 3668 → 3723), because `T_sm` grows roughly as `√(m+1)` per the 5%
residual target, so `σ²_T = T_sm²/(12(m+1))` stays nearly constant.
Values fall in the 3600-3820 mm/s² band — lower than old `smooth_zv`
(5700) but comparable to old `smooth_ei` (~4020). Users on old
`smooth_zvd_ei` (2610) will gain A_axis.

**Piecewise-kernel A_axis proof obligation.** The `σ_T²` formula
derived in `A_axis_smooth_is.md` was for single-polynomial smoothers.
For piecewise-polynomial kernels, the second-moment is a global
property (integral over the whole support) — so the closed form holds
unchanged. D1 implementation must verify this numerically against
the per-variant values above as part of the regression gate.

**Implementation.** Replace `INPUT_SMOOTHERS` at
`klippy/extras/shaper_defs.py:214-221`. Each new entry has
`init_func = _get_bs{N}_smoother(freq, damping, normalize_coeffs)`
returning the piecewise-polynomial description plus T_sm. Rest of the
pipeline (`ShaperCalibrate.find_smoother_max_accel`,
`blendshaper.shaper_span`, `_compute_A_axis_smooth_is`) works against
the new family via the same polynomial-moment code path as before,
modulo the piecewise extension below.

**`struct smoother` piecewise redesign (D1 sub-task).** The current
`struct smoother` at `klippy/chelper/integrate.h:8-13` holds a single
flat polynomial coefficient array (`c0[12], c1[12], c2[12]`) for up
to degree 11. Plan 5 stores the **fused kernel**
`k_fused = h ⊛ w`, not the bare forward kernel. See the resolution
memo `plan5-derivations/fused_kernel_storage_resolution.md` for the
derivation; key result: fused kernel is stored as **9 equal-width
pieces × degree 5** via least-squares fit at shaper-reset time
(Path C).

Why not store `h` and `w` separately? The inverse `h` is a long FIR
tap array (~11259 taps for bs3 at stepgen dt; obtained via windowed
IFFT of `1/W(ω)`). It has no compact closed-form piecewise polynomial
representation. Convolving it with the 4-piece `w` directly would
produce ~11259 shifted polynomials. **Path C** (compute h as FIR in
Python at shaper-reset, numerically convolve with w, least-squares
fit the resulting `k_fused(t)` to 9 × degree-5 piecewise polynomial)
was verified in the resolution memo to match the exact FIR cascade
passband error to 4 significant figures (e.g., bs3: 3.218% fit vs
3.218% exact; bs5: 0.546% fit vs 0.541% exact). Degrees 7/11 give no
improvement; 9 pieces suffices for all bs1..bs5 variants.

Struct changes:

1. Replace flat coefficient arrays with
   `struct smoother_piece { double coeffs[6]; double t_start, t_end; smoother_antiderivatives m_start, m_end; }` —
   antiderivative endpoints cached per-piece to accelerate
   `range_integrate` queries.
2. `struct smoother` holds `n_pieces ≤ 9` plus the piece array.
3. `calc_antiderivatives` (`integrate.c`) gains a piece-index lookup
   per sample (linear scan through ≤ 9 pieces — faster than binary
   search for this count).
4. `range_integrate` (`kin_shaper.c:105-160`) must correctly handle
   queries spanning piece boundaries. Test with random samples
   crossing boundaries.
5. **FFI signature change.**
   `input_shaper_set_smoother_params(sk, axis, n, a[], t_sm)` becomes
   `input_shaper_set_smoother_params(sk, axis, n_pieces, piece_buf[9·8], t_sm)`
   (flat buffer of 9 pieces × 8 doubles each). Versioned to avoid
   silent ABI mismatch.
   `kin_extruder.c::extruder_set_smoothing_params` needs the same
   extension.
6. **Numerical conditioning at degree 5.** Moment arithmetic `t^5`
   at `t = 0.5 · T_sm ≈ 0.034` yields `t^5 ≈ 4.5e-8`; after
   normalization by `T_sm^5`, differences `h^5 - (-h)^5` for
   symmetric integration bounds must avoid catastrophic cancellation.
   Use Horner-form evaluation or symbolic simplification (odd powers
   over symmetric intervals vanish).

**Storage cost.** Current `struct smoother` ~400 B; new ~2.2 KB
(9 pieces × ~32 B + overhead). Comfortable L1 cache fit for the
shaper's 2 XY instances + 1 extruder instance.

**Piecewise A_axis proof obligation (from §D1 A_axis).** The
second-moment formula `σ²_T = ∫ t² w(t) dt` is integral-over-full-support
regardless of kernel piece count. The D1 A_axis table is therefore
valid for piecewise kernels as long as `w(t)` has the correct shape —
which the closed-form B-spline construction guarantees
(Unser-Aldroubi-Eden 1993 eq. 13). Numerical verification against
the per-variant table in §D1 is the regression gate.

**Tests.**
- `test/test_shaper_defs.py`: kernel integrates to unity, is even,
  is C^{m-1} at piece boundaries, spectrum matches closed-form sinc^n.
- `test/test_shaper_calibrate.py`: `find_smoother_max_accel` returns
  `A_axis` matching the closed-form σ²_T computation to 1e-9.
- `test/test_blendshaper.py`: `compute_shaper_bounds` under each
  `bs*` variant produces finite bounds that scale physically with
  `shaper_freq`.

### D2. Direct-quintic step generation

**Architecture.** Option A from `direct_quintic_architecture.md`:
tagged union inside `struct move`. Staged as three sub-phases.

**D2a: Extend `smoother_antiderivatives` and emit position-in-t
polynomials at emit time (Option B).**

`klippy/chelper/integrate.c:51-63` currently computes three moments
`(m0, m1, m2)` assuming `x(t) = start + axes_r · (start_v·t +
half_accel·t²)`. The arch review
(`plan5-derivations/REVIEW_2_ARCH.md` issue #1) flagged that a naive
"compose quintic(s) with trapezoid s(t) at query time" produces a
**degree-10 polynomial in t** during accel/decel phases (quintic × 
degree-2 s(t)), not degree 5. Requiring 11 moments + per-phase
dispatch inside `integrate_move` makes the query inner loop
significantly slower.

**Chosen approach: compose once at emit time in Python.** The
`CornerBlender._emit_blend` step computes `position(t)` per phase
(accel / cruise / decel) as explicit polynomial coefficients in t,
and stores them directly in the trapq move. The C-side smoother
integrator never sees the s(t) trapezoid or the quintic(s) form —
only a plain `position(t) = Σ c_k · t^k` per phase with known
degree per phase.

Degrees per phase:
- Accel: degree 10 (quintic ∘ degree-2 s(t))
- Cruise: degree 5 (quintic ∘ linear s(t))
- Decel: degree 10

Implementation:
- Extend `calc_antiderivatives` struct in
  `klippy/chelper/integrate.h` from 3 to **11 fields**
  `(m_0 … m_10)`.
- Update `integrate_move` inner loop to evaluate
  `Σ_k=0..10 c_k · m_k` per sample.
- `integrate_move` also learns phase dispatch: when the integration
  window crosses `t_accel_end` or `t_decel_start`, split the
  integral and use the appropriate phase's coefficients on each
  side.
- Linear moves: `c_3 … c_10 = 0`, phase boundaries degenerate —
  existing behavior falls out naturally. Per-sample cost for
  linear moves rises modestly (10-ish multiplies by zero that the
  compiler may or may not eliminate) — profile and confirm.

**Per-sample cost estimate.** Current: 3 FMAs + 1 phase-boundary
check per linear-move sample. Post-D2a: 11 FMAs + up to 2
phase-boundary checks per quintic-move sample. Approximately 3.5×
per-sample cost on quintic moves; near-zero overhead on linear
moves if the compiler optimizes the zero-coefficient branches.
Combined with the fused-kernel 3× width from D3, total query cost
multiplier on quintic moves: **~10×** current. Trident sysload
budget must accommodate this or the piecewise-kernel piece-dispatch
must be hot-cached; profile before landing.

**Numerical conditioning at degree 10.** `t^10` at `t ≈ 0.04 s`
(typical move duration) is ~1e-14 — at the edge of double precision.
Use Horner-form polynomial evaluation throughout; avoid direct `t^k`
power computation. Validate against a Python reference using
numpy.polynomial which handles conditioning explicitly.

**Regression gate for D2a.** Two levels of test:
1. **Linear moves bit-identical.** Before any quintic moves enter
   the queue, verify `smoother_calc_position` output on linear
   moves matches a saved golden dataset to bit-exact.
2. **Quintic moves round-trip.** Python-side emit a representative
   quintic, run C-side `integrate_move` at 100 sample points,
   verify position to ≤ 1e-9 mm against numpy.polynomial reference.
   (Linear-bit-identical gate #1 doesn't exercise the new
   c_3..c_10 paths — this gate does.)

**D2b: Tagged union in `struct move`.**

Post-review (`REVIEW_2_ARCH.md` issue #3), the quintic variant
stores explicit per-phase `position(t)` polynomial coefficients
rather than a compositional (quintic coefs + s(t) trapezoid) form.
Trade-off: bigger struct, but simpler query path and no
composition at every step-gen sample.

```c
enum move_kind {
    MOVE_LINEAR = 0,                   /* existing trapq primitive */
    MOVE_QUINTIC_POLY_T = 1,           /* Plan 5: per-phase poly-in-t */
};

/* Per-phase position polynomial: x(t) = Σ c_k · (t - t_phase_start)^k.
 * 11 coeffs per axis (degree 10 for accel/decel, c_6..c_10 = 0 for cruise). */
struct move_quintic_phase {
    double t_end;                      /* phase end (relative to move start) */
    struct coord c[11];                /* per-axis polynomial coefficients */
};

struct move {
    double print_time, move_t;
    enum move_kind kind;
    struct coord start_pos;
    union {
        struct {  /* MOVE_LINEAR */
            double start_v, half_accel;
            struct coord axes_r;
        } lin;
        struct {  /* MOVE_QUINTIC_POLY_T */
            double arc_length;
            struct move_quintic_phase accel, cruise, decel;
            /* Precomputed for upstream junction-cap computation: */
            double v_cap_min;         /* min_s v_cap(s) — the Z-option cap */
        } quintic;
    } u;
    struct list_node node;
};
```

**Size:** 3 phases × (11 coeffs × 3 axes + 1 time bound) = 102 doubles
+ ~3 doubles metadata = **~840 bytes per quintic move** (~6× a linear
move). Cruise phase's `c_6..c_10` are zero and can be omitted in a
"short phase" variant if cache pressure is a real problem (Plan 6
optimization; ship the full 840 B for simplicity first).

**Kinematics fanout is smaller than the prior spec claimed.**
Post-review (`REVIEW_2_ARCH.md` bonus finding): only **3 C files**
actually need changes for quintic support, not all 10 `kin_*.c`:

- `klippy/chelper/trapq.c` — `move_get_coord` and `move_get_distance`
  branch on `kind`. Every downstream `kin_*.c` calls `move_get_coord`
  and gets the right answer automatically.
- `klippy/chelper/integrate.c` — 11-moment extension + phase
  dispatch (D2a).
- `klippy/chelper/kin_shaper.c` — consumes positions for shaper
  convolution; works through `move_get_coord`, but may need
  review for any direct access to `m->axes_r` or `m->start_v`.
- `klippy/chelper/kin_extruder.c` — same as `kin_shaper.c` — uses
  `smoother_calc_position` on XY axes. Review for direct struct
  access.

The other seven kinematics files (`kin_cartesian.c`, `kin_corexy.c`,
`kin_corexz.c`, `kin_delta.c`, `kin_deltesian.c`, `kin_idex.c`,
`kin_polar.c`, `kin_rotary_delta.c`, `kin_winch.c`) use
`move_get_coord` as the only entry point and therefore need **zero
changes**. This is a significant scope reduction from the 10-file
assumption.

**Inlining discipline.** The current `move_get_coord` is inline at
`trapq.c:31-39`. Adding a `kind` branch either (a) keeps it inline
and each caller re-inlines the branch (I-cache bloat) or (b)
de-inlines and adds a function-call cost per step-gen query.
Decision: **benchmark both before implementation**; fall back to (b)
if (a) measurably regresses step-gen throughput.

D2c's initial emit can fill accel/decel with zero-duration phases
(bypass TOPP) and store the cruise polynomial as the full move
— bootstraps D2 independently of D7.

**Dispatch points to change:**
- `trapq.c::move_get_distance` — return arc-length for quintic
  kind (precomputed, stored in `arc_length` field); existing
  formula for linear.
- `trapq.c::move_get_coord` — evaluate per-phase `position(t)`
  polynomial for quintic kind; existing formula for linear.
- `trapq.c::trapq_extract_old` — ensure kind is preserved through
  `struct pull_move` projections for motion_report serialization.
- `itersolve.c::itersolve_gen_steps_range` — no change; calls
  `calc_position_cb` which dispatches through `move_get_coord`.
- **`itersolve.c:140-142` `check_active`** — reads `m->axes_r.[xyz]`
  directly to decide whether a stepper is active on this move. On
  a quintic move, linear-union access is uninitialized → silent
  miscompile. Dispatch on `kind`: for quintic, check the per-axis
  polynomial `c[1..10]` for any nonzero coefficient. Missed by
  earlier revisions; `REVIEW_3_C_INTEGRATION.md` critical finding.
- `kin_shaper.c` and `kin_extruder.c` — review for any direct
  access to `m->axes_r` / `m->start_v` / `m->half_accel` that
  assumes the linear form. Replace with `move_get_coord` /
  `move_get_distance` calls where present.
- All other `kin_*.c` files: **no changes needed** (they already
  route through `move_get_coord`).

**Hard invariant: `MOVE_LINEAR = 0`.** `itersolve.c:256-267`
synthesizes a `struct move` via `memset(m, 0, sizeof(*m))`. This
produces a valid linear move with zero velocity/accel by accident.
Preserving this behavior requires `MOVE_LINEAR = 0` (the enum
value). Document this invariant in `trapq.h` as a load-bearing
comment — anyone reordering the enum silently breaks the synthesis.

**Python-side `trapq_append` callers.** The C-side FFI signature
must not break existing Python callers. `REVIEW_3_SCOPE_RISK.md`
enumerated 5:

- `klippy/extras/force_move.py`
- `klippy/extras/manual_stepper.py`
- `klippy/extras/trad_rack.py`
- `klippy/kinematics/extruder.py`
- `klippy/toolhead.py`

All five pass linear-move parameters (start_v, accel, axes_r, …).
Strategy: keep the existing `trapq_append(start_v, accel, axes_r…)`
entry point unchanged — it emits a linear move as today. Add a new
`trapq_append_quintic(…)` entry point for Plan 5's quintic emit.
Zero-change for all five callers.

D2b's scope note: "3 C files + 0 changes to existing Python callers."

**D2c: Emit quintic directly from `blendplanner`.**

Replace `CornerBlender._emit_blend`'s polyline loop with a single
`trapq_append_quintic(print_time, duration, start_pos, c1..c5)` call
per blend. The quintic coefficients come from `QuinticShape`'s
existing `_deriv_Q` / Bernstein-to-monomial basis conversion (add the
converter method).

Remove:
- `QuinticShape.polyline()` — no longer needed.
- `blendplanner._resolve_chord_err` — polyline density no longer a
  thing.
- Any sub-move iteration in `_emit_blend`.

Keep:
- `v_cap_fn` — now queried at move-step-gen time for per-s velocity
  bound rather than used for sub-move caps.

**Single-move duration.** A quintic blend of arc-length `L` at
velocity `v(s)` takes `T = ∫₀^L ds / v(s)`. Per-s velocity profile
is the subject of D7 (unified v(s), Pillar 2b — see below). D2c
emits the quintic with the v(s) profile D7 produces.

**Tests.**
- `test/test_trapq.py`: quintic trapq entries round-trip position
  queries correctly — sample at 1000 points along the move, verify
  against `QuinticShape.position(s)` to 1e-9.
- `test/test_itersolve.py`: step generation against a quintic move
  matches against polyline-equivalent within 1 step (microstep of
  rounding).
- `test/test_blendplanner.py`: end-to-end — corner at 45°/90°/120°
  produces one quintic trapq entry (not N polyline entries); actual
  stepper outputs match the polyline reference ±1 step.

**klipper-sim parity.** The batch-sim harness at
`~/Developer/klipper-sim/` reads `trapq` state. D2b tagged-union
change breaks its deserializer. Update klipper-sim in the same
commit batch as D2b — it's the primary offline validation path for
planner changes (59 tests per `reference_klipper_sim.md`), so losing
it mid-implementation makes regression diagnosis much harder.

**motion_report schema.** `motion_report` emits trapq-move structures
via websocket; any external consumer (Mainsail, Fluidd, Moonraker)
that parses these will see a schema change. Emit a `version: 2`
field. **Announce the change to frontend projects before landing**
(GitHub issue on Mainsail, Fluidd, Moonraker repos referencing the
Plan 5 PR) so consumers can version-guard rather than break silently.

### D3. Feedforward inverse shaper (Pillar 1) — all axes

**Architecture.** Option C from the Plan 4 brainstorm (preserved in
Plan 4 spec line 129-138): fuse `pre_distort` with forward shaper at
stepper-query layer. `trapq` holds the planned physical trajectory
unchanged; pre-distort + forward shaper compose into a single
query-time operator. PA reads `trapq` unchanged. Plan 3's extruder
cap reads `move.accel` (planned) unchanged.

**All axes, not XY only.** Earlier revisions scoped D3 to XY axes
only with the argument "extruder follows physical path via PA, not
stepper-commanded position." The review showed that
`kin_extruder.c::extruder_calc_position` (`:184-226`) runs the same
`smoother_calc_position` on the XY-axis commanded positions and
derives the E stepper position from them. Post-D1, the extruder's
smoother is a `bs*` variant. If XY gets feedforward inverse and E
does not, XY traces the plan faithfully while E lags by ~`T_sm/2`
— breaks PA phase synchronization. Therefore: **D3 applies the same
fused kernel `k_fused = h ⊛ w` to all shaped axes (X, Y, E).**

Effort note: single shared `k_fused` computation at shaper-reset
serves all axes; per-axis `struct input_shaper` instances each hold
a pointer to it. Incremental cost vs XY-only is minimal — one extra
smoother-slot assignment in `input_shaper_set_smoother_params`.

**C-side implementation.** In `klippy/chelper/kin_shaper.c`:

- Extend `struct smoother` (or add `struct fused_smoother`) to hold
  the precomputed `k_fused = h ⊛ w` coefficients. Support width
  `T_fused = T_sm + T_h`.
- `k_fused` is piecewise polynomial (convolution of two piecewise
  polynomials is piecewise polynomial with more pieces — for bs_m
  with m+1 pieces convolved with FIR inverse of similar piece count,
  the fused has ≤ 2(m+1) pieces). The piecewise `struct smoother`
  extension in D1 handles this representation.
- `input_shaper_set_smoother_params` (FFI, called by Python at
  config) receives both `w` and `h` coefficients, computes `k_fused`
  once, stores it in every axis' smoother slot.
- `shaper_{x,y,xy}_calc_position` and `extruder_calc_position` apply
  `k_fused` via the same `integrate_move` machinery (piecewise
  polynomial kernel evaluates over `move.kind == QUINTIC` directly
  with the 6-moment extension from D2a).

**Python-side plumbing.** `klippy/extras/input_shaper.py` computes
`h` at config time from the B-spline variant (closed-form per
`new_shaper_family.md` §4), passes both to C. Re-invoked on
`SET_INPUT_SHAPER` live-tuning, same as today.

**`v_jerk` cap flows through `A_axis`.** D4's saturation cap replaces
`a_max` with `a_max / G` in the centripetal bound. The rotation-jerk
cap `v_jerk = (j_max / κ²)^(1/3)` uses `j_eff` computed from `A_axis`
via `compute_shaper_bounds` — **to stay consistent, `A_axis` in the
saturation regime must also be `a_mech_max / G`, not `a_mech_max`**.
Set `AxisShaperSnapshot.A_axis` to the inverse-corrected value at
`_extract_shapers` time when `inverse_G > 1.0`. Then `j_eff` and
`v_jerk` absorb the cap automatically — no separate jerk logic in
`v_cap_fn`.

**Graceful degradation.** If `shaper_type == "none"` or
`target_smoothing == 0`, set `h = δ(t)` (Kronecker identity) and
`k_fused = w`. The query path stays on the single code branch;
forward-only behavior falls out naturally. Verified in D1 + D4:
`limits.shapers = []` means `G_worst = 1.0` and `a_eff = a_max` —
reduces to current Plan 4 behavior.

**Tests.**
- `test/test_kin_shaper.py` (or equivalent C-level unit): cascade
  identity — commanded position through `(k_fused)` equals planned
  position to passband error ≤ 2%.
- `test/test_input_shaper.py`: `SET_INPUT_SHAPER` with each `bs*`
  variant produces a valid `k_fused` and doesn't stall step
  generation.
- `test/test_kin_extruder.py`: extruder-path `k_fused` applied;
  XY vs E time-alignment preserved (`|t_xy - t_E| ≤ 1 us` over a
  representative move).
- Integration: `test/test_blendquintic.py` adds a test that a
  quintic corner produces toolhead position matching `QuinticShape`
  within feedforward-corrected tolerance.

### D4. Saturation cap

**Formula.** Derived from first principles in
`plan5-derivations/per_axis_saturation_derivation.md`:

```
v_cap_fn_plan5(s) = min(
    v_max,
    sqrt( a_max / (G_worst(s) · κ(s)) ),    # Pillar 1 saturation cap
    (j_max / κ(s)²)^(1/3),                   # rotation-jerk cap
    v_step_cap(s),                           # shaper-bandwidth cap
)
```

with the **orientation-dependent per-axis bound**:

```
G_worst(s) = max_axes  G_axis · (|proj_t(s)| + |proj_n(s)|)
```

where `proj_t = t̂(s) · ê_axis` and `proj_n = n̂(s) · ê_axis` are the
tangent and normal Frenet-frame projections onto each shaped axis,
and `G_axis = ‖h_axis‖₁` is the L¹ norm of that axis' inverse
kernel.

**Derivation:** path accel is `ẍ(s) = v̇·t̂ + v²κ·n̂` with `v̇` and
`v²κ` independently bounded by `a_max`. Projecting onto a Cartesian
axis gives `ẍ_axis = v̇·proj_t + v²κ·proj_n`, and since `v̇` and
`v²κ` are independent scalar constraints with any sign, the worst-case
per-axis `|ẍ_axis|` is `|proj_t|·a_max + |proj_n|·a_max =
(|proj_t|+|proj_n|)·a_max`. Applying L¹-L∞ bound with inverse
kernel: `|commanded_axis|_∞ ≤ G_axis · (|proj_t|+|proj_n|)·a_max`.
Setting this ≤ mechanical `a_max` and solving for v gives the
formula above. Monte Carlo-verified in the derivation artifact.

**No √2 factor:** prior spec revisions used `√2·|proj_n|·G` or
`√2·|proj|·G`, both incorrect. The first is unsafe at tangent-aligned
corners (reports +∞ when truth is finite); the second is
over-conservative by factor √2 on normal-axis-only cases. The
sum-of-projections form is tight.

**Orientation matters:** at a 90° corner with `v̇ = v²κ = a_max`:
- axis-aligned (proj_t=1, proj_n=0): v_sat bounded by tangent term only
- diagonal (proj_t = proj_n = 1/√2): v_sat tighter by factor √2 than axis-aligned
- perpendicular (proj_t=0, proj_n=1): v_sat bounded by normal term only

For bs3 (G=2.003), κ=0.03, a_max=5000: v_sat ranges 242.6-288.5 mm/s
depending on corner orientation.

**Implementation.**
- Extend `AxisShaperSnapshot` (`klippy/blendshaper.py:28-38`) with a
  new `inverse_G: float = 1.0` field.
- Populate in `_extract_shapers` (`klippy/blendmath.py`) from each
  variant's published `G` constant.
- Apply in `QuinticShape.v_cap_fn` (`klippy/blendquintic.py:582`):
  compute `(|proj_t|+|proj_n|)` per shaped axis from the current
  Frenet frame (already available via `_point_frame`); take the max
  weighted by `G_axis`; result is `G_worst(s)`. Then
  `a_eff = a_max / G_worst(s)` per-s (not a single scalar).
- Because `G_worst(s)` varies with s (tangent and normal change
  along the curve), the cap is genuinely per-s — no single scalar
  can replace it. This matches D7's v_cap_fn(s) composition.

Default value `inverse_G = 1.0` means no inverse — the cap reduces to
the existing Plan 4 form. `bs*` variants ship with `inverse_G` set to
the Table in D1 (1.92 to 2.00 at conservative `pb_max = 0.3·f_sh`;
2.53 to 2.84 at wider `pb_max = 0.5·f_sh`). The operating `pb_max`
is a shaper-config parameter.

**Tests.**
- `test/test_blendquintic.py`: v_cap with `inverse_G = 2.0` at a 90°
  corner is `1/sqrt(2)` of the `inverse_G = 1.0` value.
- `test/test_blendshaper.py`: G projection across asymmetric-axis
  configs correct.

### D5. Lookahead extension

**Current state** (from `lookahead_extension_audit.md` §1):
`kin_flush_delay = max(kin_flush_times ∪ {1 ms})`, evaluated in
`klippy/toolhead.py:809-816`. At Trident's default PA-dominated
configuration, realized value is ~40 ms (PA floor, not shaper).

**Plan 5 delta.** Shaper registers `max(pre_active, post_active) =
(T_sm + T_h)/2 + |t_offs|`. With `T_h = 2 · T_sm`, the shaper
contribution triples (from `T_sm/2` to `1.5 · T_sm`). For `bs3 @ 40
Hz` (`T_sm = 56 ms`): shaper contribution ≈ 84 ms, becoming the
binding constraint. For `bs1` (`T_sm = 39 ms`): ~58 ms, also past
the PA floor.

**Implementation.**
- `klippy/chelper/kin_shaper.c::shaper_note_generation_time`
  (`:267-293`): pass through `T_h` value, compute
  `pre_active = T_fused/2 + |t_offs|`, `post_active = T_fused/2 -
  t_offs`. Add guard: if `t_h == 0` (shaper disabled), skip the
  inverse contribution entirely.
- `klippy/extras/input_shaper.py` computes `T_h` from the variant
  and passes to C alongside `w`.
- No changes required in `klippy/toolhead.py` (the `max()` reduction
  absorbs the widened value automatically).
- No changes required in `itersolve.c` (per-stepper
  `gen_steps_pre/post_active` gating already uses whatever value the
  shaper registered).

**User-visible impact.**
- M400 wait-for-finish takes `T_fused/2 ≈ T_sm` longer than
  pre-Plan-5 (~30-70 ms depending on variant). Not perceptible.
- `SET_INPUT_SHAPER` live-tuning flushes and re-establishes the
  lookahead window; the pause is now ~40-70 ms longer. Still
  sub-second.
- Emergency stop: step queue drains within the extended window; no
  change to physical stop time (limited by deceleration, not
  queue).

**Tests.**
- `test/test_toolhead.py`: M400 under `bs3` waits within ±5 ms of the
  computed `T_fused/2`.
- `test/test_input_shaper.py`: `SET_INPUT_SHAPER shaper_type=bs5`
  while moving doesn't produce `Timer too close` or mid-move
  corruption.

### D6. Config migration

**Policy.** Per the fork-as-gate rule, no back-compat aliases. Old
`shaper_type = smooth_*` values error at config-load with a clear
message:

```
Error: shaper_type 'smooth_mzv' was replaced in Magnum Opus with the
cardinal B-spline chain family. Use shaper_type = 'bs2' for
equivalent behavior. See migration table in Kalico docs.
```

**Migration table.**

| Old | Closest `bs*` | Notes |
|-----|---------------|-------|
| `smooth_zv` | `bs1` | bs1 has wider T_sm but invertibility; same 5% residual |
| `smooth_mzv` | `bs2` | bs2 is the direct analog |
| `smooth_ei` | `bs2` or `bs3` | bs3 has slightly better rejection |
| `smooth_2hump_ei` | `bs4` | bs4 matches robustness range |
| `smooth_zvd_ei` | `bs5` | bs5 is the widest; closest match |
| `smooth_si` | `bs3` | Default-ish choice |

**Implementation.**
- `klippy/extras/input_shaper.py` validates `shaper_type` against
  the current allowed set. For each retired `smooth_*` name,
  produce a friendly error pointing at the `bs*` replacement.
- `klippy/extras/shaper_calibrate.py` — update
  `AUTOTUNE_SHAPERS` / `SHAPER_CALIBRATE` commands to recommend
  `bs*` variants instead of `smooth_*`. `REVIEW_3_SCOPE_RISK.md`
  flagged that this file references the 6 retired names around
  lines 29-38; recommendation logic must be updated, not just the
  name list. Include a default-variant heuristic (e.g. bs2 for
  typical 0.05-damping setups; bs3 for wider notch).
- `klippy/extras/motion_report.py` — emit `kind` field in the
  websocket payload. Bump schema version.
- `klippy/extras/tap_analysis.py` — reviewer found this file
  iterates trapq moves. Update to skip or handle `kind = quintic`
  gracefully.
- Document the table in `docs/Resonance_Compensation.md` (existing
  Klipper doc — rewrite the SIS section for Magnum Opus).
- Classic FIR names in tree (`zv`, `mzv`) stay valid and unchanged.
  Note: `ei`, `zvd`, `2hump_ei`, `3hump_ei` have already been
  removed from `INPUT_SHAPERS` in an earlier Kalico change — they
  are not re-added by Plan 5 (FIR feedforward inverse for impulse
  trains is not tractable).

**GitHub coordination tasks** (wall-clock, not engineering):
- Mainsail issue: motion_report v2 schema announcement
- Fluidd issue: same
- Moonraker issue: deserializer update if it parses trapq directly

**Tests.**
- `test/test_input_shaper.py`: loading a config with
  `shaper_type = smooth_mzv` raises `configparser.Error` with the
  migration hint in the message.

### D7. Unified v(s) along the curve (Pillar 2b)

**Rationale.** D2c emits the quintic as a single trapq entry.
Without D7 that entry carries a single scalar velocity — per the
research memo (`plan5-derivations/unified_v_of_s.md §1.2`) this is
actually **mathematically unsafe**: the current aggregator cap
`min(prev.cruise, nxt.cruise, v_cap(L/2))` uses the midpoint of the
quintic, but `v_cap(s)` has its minimum at the curvature *shoulder*
(s ≈ 0.18·L and 0.82·L), not the midpoint. The baseline polyline
was locally overshooting the centripetal cap by up to ~5× at the
shoulders and was only safe in practice because of corner_deviation's
small default value plus the brief shoulder timeslice. Direct-quintic
inherits this mathematically dangerous single-cap approximation
unless D7 fixes it. D7 also closes the Magnum Opus architecture
loop: Plan 3's extruder cap is absorbed into `v_cap_fn(s)` directly,
eliminating the separate `cap_move` pass.

**Algorithm.** TOPP (Time-Optimal Path Parameterization — Pham 2014).
Dense-grid `N ≈ 128-256` samples of `v_cap(s)`, forward and backward
acceleration-limited passes, return optimal `v*(s)`. Runs at emit
time (Python side, ~0.1 ms per blend). Closed-form attempted but
rejected: `v_cap(s)` is a pointwise `min` of 5 algebraic branches
with break-points depending on G, active shaper, extruder flow
ratio, and jerk — no analytic inverse exists.

**Upstream junction-cap contract (Option Z).** The outer planner
(`LookAheadQueue` in `klippy/toolhead.py`) picks cruise velocities
at every junction using a forward+backward pass. Before Plan 5,
the blend's cap fed into this decision was `v_cap_fn(L/2)`
(midpoint) — mathematically unsafe because the true minimum is at
the shoulder. **Plan 5 Z-option fix:** compute `v_cap_min =
min_s v_cap(s)` at junction-cap-computation time (sample
v_cap_fn densely, take minimum) and feed THAT into the junction
decision. The lookahead's forward+backward pass then naturally
produces cruise velocities compatible with the blend's tightest
point. TOPP later runs inside the blend taking `(v_in, v_out)`
as fixed inputs — no retract / re-plan machinery needed. This
closes the loop cleanly within the existing single-pass planner
architecture.

Cost: one extra dense sampling of `v_cap_fn(s)` per blend at
junction-time (~128 samples × 5-branch `min`). Negligible compared
to the TOPP pass that already runs at emit time.

(Research memo `unified_v_of_s.md §7` proposed a 2-pass retract
alternative; `REVIEW_2_ARCH.md` issue #2 correctly flagged that
`LookAheadQueue` has no retract API. Option Z avoids the issue
entirely by giving the planner the correct number upstream.)

**Cap composition** (from `unified_v_of_s.md §3`):
```
v_cap(s) = min(
    v_max,
    v_sat(s)   = sqrt( a_max / (G · κ(s)) ),      [Pillar 1 saturation]
    v_jerk(s)  = (j_eff / κ(s)²)^(1/3),            [Pillar 2 rotation-jerk]
    v_step(s)  = sqrt( A_axis · R(s) / |n̂ · ê| ),  [Pillar 2 shaper-rejection]
    v_extr(s)  = cap_k( k(s), ... )                [Plan 3 flow cap]
)
```
`v_sat` subsumes Pillar 2's original centripetal cap when `G ≥ 1`.

**Representation (chosen): per-phase position-in-t polynomials.**
TOPP produces a trapezoid-in-s profile (conceptual: (v_in,
cruise_v, v_out, accel_end_s, decel_start_s, a_max)). Rather than
storing those trapezoid parameters and composing quintic(s(t)) at
query time (which produces degree-10 in t and needs 11 moments +
phase dispatch in the C integrator), we **pre-compose in Python at
emit time** and store the resulting `position(t)` polynomial per
phase directly. This is the Option B described in D2a and the
struct described in D2b.

Rationale: closed-form composition of a quintic-in-s with a
degree-2 s(t) is straightforward in Python (numpy.polynomial
handles it) but fragile in C at degree 10 with double-precision
conditioning. Pre-composing trades ~840 B per move (vs ~192 B for
trapezoid-only) for a much simpler, faster query path.

`REVIEW_2_MATH.md` item #3 found that a faithful trapezoid-in-s
approximation is actually only **1.4% slower than TOPP-optimal**
on the worked 90° case (not 12% as `unified_v_of_s.md §4.3`
initially claimed). Trapezoid-in-s is therefore near-optimal;
piecewise-poly v(s) upgrade path deferred as Plan 6 only if HW
shows the 1.4% gap matters.

**Storage layout:** defined in D2b's `MOVE_QUINTIC_POLY_T` struct
(`move_quintic_phase` × 3 phases with 11 polynomial coefficients
per axis per phase, plus `v_cap_min` for the Z-option junction-cap
contract).

**Implementation.**
- `klippy/blendquintic.py`: extend `v_cap_fn` to compose all 5 cap
  sources per-s (currently centripetal + jerk; add v_sat, v_step,
  v_extr). Add `QuinticShape.v_cap_min()` that samples v_cap_fn at
  a dense grid and returns the minimum (for the Z-option upstream
  junction cap). Add `QuinticShape.compute_topp_profile(v_in,
  v_out, a_max, flow_k_in, flow_k_out) -> (accel_poly, cruise_poly,
  decel_poly, t_accel_end, t_decel_start)` — runs dense-grid TOPP,
  fits trapezoid-in-s, composes quintic(s(t)) per phase in Python
  using numpy.polynomial, returns per-phase polynomial coefficients
  in t.
- `klippy/blendmath.py`: `suppressed_junction_v` (or equivalent
  junction-cap helper) consumes `v_cap_min` from the blend. The
  Z-option contract — junction decisions use the blend's true
  minimum, not the midpoint approximation.
- `klippy/chelper/trapq.h` / `trapq.c`: add `MOVE_QUINTIC_POLY_T`
  variant with per-phase polynomial coefficients + `v_cap_min`
  field. `move_get_coord` dispatches on `kind`, evaluates per-phase
  polynomial using Horner form.
- `klippy/chelper/integrate.c`: extend moment count 3 → 11 + phase
  dispatch at the integration-window boundary (D2a).
- `klippy/blendextruder.py`: `cap_move` retires. Its logic migrates
  into a per-s `v_extr(s)` contribution used by TOPP.
- `klippy/blendplanner.py::CornerBlender._emit_blend`: compute
  `v_cap_min`, feed to junction-cap. Then call
  `compute_topp_profile` and emit `MOVE_QUINTIC_POLY_T`.
- `klippy/toolhead.py`: **no lookahead retract needed** under
  Option Z. Existing forward+backward lookahead handles the
  blend-binding case correctly once it receives the right cap
  number upstream. Ill-posed cases (`v_cap_min = 0` or similar
  degeneracy) surface as plan errors at emit time.

**Tests.**
- `test/test_blendquintic.py`: TOPP profile respects `v_cap(s)`
  pointwise to 1e-6; `v(0) = v_in`, `v(L) = v_out`; `|v̇| ≤ a_max`.
- `test/test_blendquintic.py`: shoulder minimum correctly detected
  — 90° corner produces `v_cap_min < v_cap(L/2)`.
- `test/test_blendquintic.py`: `v_cap_min` plumbed to junction
  cap — straight-into-corner sequence shows prev decelerating
  to `v_cap_min` before the blend.
- `test/test_blendplanner.py`: asymmetric corners produce asymmetric
  profiles.
- `test/test_blendextruder.py`: `cap_move` removed; equivalent
  behavior preserved via per-s `v_extr`.
- `test/test_trapq.py`: round-trip — emit a representative
  quintic, evaluate `move_get_coord` at 1000 sample points, verify
  position matches Python reference to 1e-9 mm.
- Performance: `Voron_Design_Cube_v7_ABS_22m13s` under bs3 runs in
  ≤ old polyline-era time, measurably faster at tight corners.

**Plan 3 extruder cap absorption.** Remove `blendextruder.cap_move`
from `toolhead.move`; fold its logic into `v_extr(s)` per-s
contribution used inside TOPP. Flow ratio `k(s)` is the linear
interpolation of `prev.axes_r[3]` to `nxt.axes_r[3]`. Net: no more
separate cap pass; the quintic's v(s) respects flow-ratio and
extruder-accel bounds along the blend — correct-by-construction.

## Effort estimate

Two rounds of adversarial review
(`plan5-derivations/REVIEW_2026-04-22.md`, `REVIEW_2_MATH.md`,
`REVIEW_2_ARCH.md`) tightened the estimates below. Net: kinematics
fanout dropped from 10 to 3 files (-2 days); composition-at-emit-time
added complexity in D2a (+1 day); Option Z removed lookahead
retract work from D7 (-2 days).

- D1 (B-spline family + Path C fused kernel): **6-8 days** —
  piecewise `struct smoother` refactor (ABI + FFI + integration
  rewrite), forward kernel implementation, FIR inverse
  computation in Python, least-squares fit of `k_fused` to 9 ×
  degree-5 pieces, A_axis plumbing, conditioning validation,
  tests.
- D2 (direct-quintic step gen, 3 C files + klipper-sim Python
  shim): **8-11 days** — extend `struct move` tagged union, emit
  per-phase polynomial-in-t at emit time (Python-side composition),
  rewrite `smoother_antiderivatives` to 11 moments + phase
  dispatch, dispatch `itersolve.c::check_active` on `kind`
  (critical finding), add `trapq_append_quintic` entry point
  (keep existing `trapq_append` unchanged to protect all 5 Python
  callers), review `kin_shaper.c` / `kin_extruder.c` for direct
  struct access, swap blendplanner emit, klipper-sim Python Move
  shim update, regression suite (linear bit-identical + quintic
  round-trip + check_active verification).
  Estimate revised up ~40% from prior revision per
  `REVIEW_3_C_INTEGRATION.md` effort-reality-check.
- D3 (feedforward inverse, XY + E): **3-4 days** — Python-side `h`
  computation, C-side fused-kernel struct + precomputation, FFI
  wiring, extruder-path integration, A_axis-flows-to-j_eff
  plumbing.
- D4 (saturation cap, sum-of-projections): **0.5 day** —
  `AxisShaperSnapshot.inverse_G` field + per-s `G_worst(s)`
  computation from Frenet projections in `v_cap_fn`.
- D5 (lookahead extension): **1 day** — `shaper_note_generation_time`
  update + `t_h == 0` guard.
- D6 (config migration + frontend announcement): **1-1.5 days** —
  error messages + Resonance_Compensation.md rewrite + GitHub
  issues on Mainsail/Fluidd/Moonraker.
- D7 (Pillar 2b unified v(s), Option Z): **3-4 days** — compose
  `v_cap_fn(s)` from all 5 cap sources, `v_cap_min` helper, TOPP
  dense-grid implementation, per-phase polynomial composition in
  Python, `blendextruder.cap_move` absorption, junction-cap
  plumbing to lookahead.
- Integration + HW smoke: **5-7 days** — expected iteration on
  calibration/tuning now that corners behave fundamentally
  differently (quintic-faithful tracking vs. shaper-smeared).

**Total: 6-8 weeks engineering** under subagent-driven-development
dispatch. Individual deliverables parallelizable where they don't
conflict (D1 / D3 / D4 / D5 / D6 are largely parallel; D2 is the
long pole — D7 depends on D2's tagged-union and then runs quickly).

**MVP slice (if time pressure):**
Per `REVIEW_3_SCOPE_RISK.md` recommendation, a sensible implementation
order is **D1 → D3 → D2 → D7**. After D1 + D3 land (~2 weeks),
Magnum Opus has a working new shaper family with feedforward inverse
on the existing polyline path — no trapq surgery yet, user-visible
improvement already shippable. D2 and D7 can then follow sequentially
as capacity allows. This gives a clean fallback point if the C-side
refactor hits unexpected complexity.

## Validation

**Integrated-only per Magnum Opus testing philosophy.** After all 7
deliverables land:

- **Unit tests**: all new tests from each deliverable plus existing
  suites pass.
- **Cascade identity test**: plan a known trajectory, run through
  the full pipeline, compare actual stepper outputs to the planned
  position. Passband error ≤ 2% on `[0, 0.5·f_sh]`.
- **Batch-sim**: `Voron_Design_Cube_v7_ABS_22m13s` under `bs2` (≈
  old `smooth_mzv` replacement) on magnum-opus config produces
  valid stepper output, no sysload regression.
- **Corner fidelity test**: print a test pattern with sharp 45°,
  90°, 120° corners. Physical corners on the part should trace the
  quintic, not a shaper-smoothed arc. Measure with calipers or
  visual inspection.
- **HW smoke**: user runs one full print; pass if no ringing
  regressions, no `Timer too close` / `send-too-old`, sysload < 2.0,
  print quality visually matches or exceeds pre-Plan-5 state.

## Risks

1. **Piecewise `struct smoother` refactor is invasive.** D1 extends
   the single-polynomial representation to piecewise; all consumers
   (`shaper_calibrate.py`, `blendshaper.py`, C integration) must
   handle it. Regression potential on classic FIR shapers if the
   abstraction leaks.

   **Mitigation.** Keep classic FIR shapers on their existing impulse-train
   representation (they don't share `struct smoother`). Only
   `INPUT_SMOOTHERS` sees the piecewise change.

2. **Direct-quintic changes break external consumers of `trapq`.**
   motion_report, klipper-sim, Mainsail/Fluidd deserializers.
   Adding `move.kind` changes the on-wire schema.

   **Mitigation.** Version the motion_report payload. klipper-sim
   update tracked as follow-up (non-blocking). UI tools degrade
   gracefully when they see `kind = 1` — worst case they skip
   quintic moves in visualization, which is a regression but not a
   correctness issue.

3. **Step-gen query cost rises ~10× on quintic moves.** Combined
   effect of (a) fused kernel width `T_fused = 3 · T_sm` tripling
   sample count, (b) piecewise kernel dispatch (≤ 6 pieces +
   degree-5 Horner per sample, ~2-3 ns each), and (c) position
   polynomial is degree 10 on accel/decel phases (11 FMAs per
   sample vs current 3). Linear moves much cheaper (~near-zero
   overhead if compiler optimizes zero-coefficient branches). On
   a Trident SoC already running sysload 1.2-1.5, this could push
   past the ceiling. **Profile before shipping.** Fallback
   options: (a) degree-5 approximation of quintic(s(t)) in
   accel/decel with ~0.5% position error, (b) linear-move
   fast-path branch so only quintic moves pay the full cost.

   **Mitigation.** Profile on Trident's SoC before shipping. Fused
   kernel is a single polynomial evaluation per sample; the
   integration loop is already tight. Worst case: cache `k_fused`
   evaluations at a fixed sub-stepper grid.

4. **B-spline variants cluster in A_axis (~3600-4000).** Users on
   old `smooth_zv` (A_axis 5700) will see ~40% reduction in max
   accel, visible as slower straight-line cruising.

   **Mitigation.** Document the migration carefully. The corner
   fidelity win offsets this for corner-heavy prints; long straight
   moves are the loss case. User-level workaround: raise
   `target_smoothing` (currently defaults 0.12 mm) to 0.18-0.20 to
   recover the A_axis budget at a visual-quality cost.

5. **Inverse saturation at very sharp corners.** Typical `G ≈ 2.0`
   at conservative passband; at a tight corner `κ_peak = 0.05 mm⁻¹`:
   `v_sat = sqrt(5000 / (2.0 · 0.05)) ≈ 224 mm/s` (vs ~316 mm/s
   without inverse). At the more aggressive `pb_max = 0.5·f_sh` with
   `G = 2.84`: `v_sat ≈ 188 mm/s`. Tighter corners under aggressive
   passband settings may be speed-limited in ways that are
   surprising compared to Plan 4.

   **Mitigation.** The cap is honest (it prevents ringing, which is
   the worse failure mode). Document the scaling. Users can widen
   `corner_deviation` to raise the effective radius and recover
   speed.

6. **Polar and rotary_delta kinematics unimplemented for quintic.**
   D2 only touches Cartesian, CoreXY, delta. Users of polar printers
   or rotary_delta will hit the `assert m->kind == MOVE_LINEAR` and
   fail cleanly.

   **Mitigation.** Add a friendly error at `[printer]` load time:
   "Magnum Opus Plan 5 does not yet support {kinematics}. Pin to a
   pre-Plan-5 branch or contribute quintic support." Track as a
   follow-up plan.

7. **Lookahead-extension plus extruder PA stacking.** PA contributes
   its own ~40 ms to `kin_flush_delay`. Plan 5 adds `T_h/2` on top.
   On tight M400 / SET_PRESSURE_ADVANCE / E-stop timing, this extra
   delay stacks.

   **Mitigation.** Documented in D5 audit. No code change required;
   the existing `max()` reduction handles the stacking correctly.

8. **`target_smoothing = 0` sentinel must survive D1.** Existing
   rule: `target_smoothing ≤ 0` disables the shaper cap
   (`_extract_shapers` returns `[]`). New family must preserve this.

   **Mitigation.** Regression test: `target_smoothing = 0` under
   any `bs*` still disables the cap.

## Successor (planned)

Plan 5 closes the Magnum Opus three-pillar architecture. Remaining
optional follow-ups:

- **Plan 6a — EtherCAT / CANopen-402 backend.** Substantial project;
  architecture enabled by Plan 5 (parametric trajectories + fused
  kernel map cleanly onto position-command PDO streams).
- **Plan 6b — Adaptive `bs*` variant selection.** Per-move choice
  of shaper variant based on local curvature (sharp corners use
  `bs1` for speed, long smooth features use `bs5` for fidelity).
  Depends on sane HW results first.
- **Plan 6c — Shape-pluggable strategy interface.** Extract a
  `BlendShape` ABC so alternative primitives (clothoid, PH quintic)
  can be swapped in without rewriting `blendplanner`.

## Literature anchors

- **Besset & Béarée (2017)**, "FIR filter-based online
  jerk-constrained trajectory generation", *Control Engineering
  Practice* **66**:169-180. B-spline chain forward filter,
  closed-form pseudo-inverse. Primary anchor for D1/D3.
- **Biagiotti & Melchiorri (2008)**, *Trajectory Planning for
  Automatic Machines and Robots*, Springer. ISBN 978-3-540-85628-3.
  §5.5 (B-spline trajectories), §5.8 (inversion of smoothing
  filters), §5.8.2 (ZPETC). Secondary anchor for D1/D3; anchor for
  L¹-L∞ bound in D4.
- **Unser, Aldroubi & Eden (1993)**, "B-spline signal processing:
  Part I — Theory", *IEEE Trans. Signal Processing* **41**(2):821-833.
  Cardinal B-spline spectral form `sinc^{m+1}`, second-moment
  closed form used in D1 A_axis derivation.
- **Curry & Schoenberg (1966)**, "On Pólya frequency functions IV:
  The fundamental spline functions and their limits", *J. Analyse
  Mathématique* **17**:71-107. Closed-form piecewise polynomial
  coefficients via divided differences. Theorem 2 is the basis for
  the per-piece polynomial form in D1.

**Removed from prior spec revisions:** references to Wang-Altintas
2022 CIRP Annals and Altintas-Ever-Hanley-Erkorkmaz 2023 CIRP-JMST
as anchors for D4 saturation feedback. An adversarial review round
(`plan5-derivations/REVIEW_2026-04-22.md` §2) could not locate these
papers; one author name appears fabricated. The saturation-feedback
derivation in `saturation_feedback.md` stands on the
Biagiotti-Melchiorri §5.8 L¹-L∞ bound alone, which is standard
real-analysis. No CIRP-specific anchor is required.

## Key design decisions (summary)

- **Retire the polyline, replace with direct-quintic.** Primary
  architecture shift of Plan 5.
- **Replace the smooth-IS family, don't augment it.** Fork-as-gate
  policy; retaining un-invertible kernels as options would require
  plumbing two parallel paths.
- **B-spline chain over other invertible families.** Literature-backed
  (Besset-Béarée 2017), single-parameter design space, fits existing
  `struct smoother` after piecewise extension.
- **Fused forward+inverse kernel at query time.** Single convolution
  per query, not two. Precomputed at shaper-config.
- **Feedforward applies to all axes (XY + E).** Earlier "XY only"
  scoping was incorrect; `kin_extruder.c` runs the same shaper
  convolution on the E stepper's commanded position, so it needs
  the same inverse for PA synchronization.
- **Saturation cap is pointwise-in-s, no iteration.** Per-s
  replacement of `a_max` with `a_max / G_worst(s)` where
  `G_worst(s) = max_axes G_axis · (|proj_t(s)|+|proj_n(s)|)` is the
  sum-of-absolute-projections L¹ bound per axis. Prior spec
  revisions used `√2·|proj|·G` (over-conservative) or `|proj_n|·G`
  (unsafe at tangent-aligned corners). Derivation in
  `per_axis_saturation_derivation.md`. `A_axis` flows the same
  correction into `v_jerk` via `compute_shaper_bounds`.
- **All variants invertible.** The `bs1..bs5` family is uniform —
  no "partial invertibility" like the old smooth-IS. Users don't
  need to know which variant is feedforward-compatible because they
  all are.
- **Per-s v(s) instead of single mid-curve velocity.** D7 turns the
  quintic trapq entry into a velocity-varying primitive, absorbing
  Plan 3's extruder cap directly into `v_cap_fn(s)`. Closes the
  Magnum Opus design loop — no cross-cutting `cap_move` pass.
- **Compose at emit time, not at query time.** quintic(s(t)) is
  degree 10 in t during accel/decel; composing per phase in Python
  once at `_emit_blend` gives the C integrator a simple per-phase
  polynomial and avoids fragile degree-10-with-composition math
  in the step-gen inner loop.
- **Upstream junction cap (Option Z).** The blend's true minimum
  `v_cap_min = min_s v_cap(s)` is fed to the lookahead's
  junction-velocity decision upstream. No retract machinery — the
  existing forward+backward lookahead naturally produces
  compatible cruise velocities.
