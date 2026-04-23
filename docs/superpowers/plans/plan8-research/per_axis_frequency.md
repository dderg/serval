# Plan 8 research — per-axis frequency polynomial layout

**Research gap:** spec §6.3, Phase 0 Task #120.
**Question:** when `shaper_freq_x != shaper_freq_y`, the two axes have different kernel widths and different natural phase boundaries after baking. How should `struct move`'s polynomial payload represent that?

## 1. Current struct layout (baseline)

Per `klippy/chelper/trapq.h:33-60`:

```c
#define MOVE_QUINTIC_POLY_COEFFS 11

struct move_quintic_phase {
    double t_end;                         // move-local phase end
    struct coord c[MOVE_QUINTIC_POLY_COEFFS];   // per-axis coeff, xyz packed
};

struct move {
    ...
    union {
        struct { ... } lin;                     // MOVE_LINEAR
        struct {                                // MOVE_QUINTIC_POLY_T
            double arc_length;
            struct move_quintic_phase accel, cruise, decel;
            double v_cap_min;
        } quintic;
    } u;
    ...
};
```

Invariants baked into the representation:

- **Exactly 3 phases** per move (accel / cruise / decel). Zero-length phases are allowed.
- **`t_end` is shared across all three axes.** Each `move_quintic_phase` has one `t_end` and the x/y/z coefficients sit in the same `c[k]` array.
- **Phase-local time.** Each phase's polynomial is evaluated in `delta_t = move_time − prev_phase.t_end`, so all three axes pivot on the same phase boundaries (`trapq.c:45-60`).

This shared-`t_end` invariant is read by every quintic consumer:

- `trapq.c:45-60` `quintic_pick_phase` — branch tree on `accel.t_end`, `cruise.t_end`.
- `trapq.c:91-110` `move_get_coord` — uses picked phase.
- `integrate.c:155-172` `move_axis_phase_polynomial` — inlined phase-pick, emits `phase_start` and `phase_end` to the smoother integrator.
- `itersolve.c:150-163` `check_active` — scans `c[1..10]` across the three phases.
- `kin_extruder.c:52-65` `pa_move_integrate` — iterates over the same three phases.
- `trapq.c:260-286` `trapq_append_quintic` — emit side packs `{t_accel_end, t_decel_start, move_t}` into the three `t_end` slots.

Ten sites total touch the shared partition.

## 2. The new constraint (Plan 8)

After baking a FIR kernel of N impulses at delays `{0, d_1, …, d_{N−1}}`, the baked polynomial per axis acquires breakpoints at every `t_phase_boundary ± d_i`. Two axes with different `f_sh` produce different impulse delay sets, so the natural per-axis piecewise-polynomial partitions differ.

Concrete example from §6.3: `shaper_freq_x = 50 Hz`, `shaper_freq_y = 120 Hz`.
- MZV at 50 Hz → half-period 10 ms → three impulses at `t ∈ {0, 5 ms, 10 ms}`.
- MZV at 120 Hz → half-period ~4.17 ms → impulses at `t ∈ {0, 2.08 ms, 4.17 ms}`.

The two axes' piecewise partitions now differ in both placement and count.

## 3. Candidate A — unified finer partition, pad wider-kernel axis

Keep the shared-`t_end` invariant. Form the union of every phase-boundary set across all axes, then split each per-axis polynomial into sub-phases aligned to that union. Wider-kernel axis (coarser natural partition) carries the same polynomial across several consecutive sub-phases.

### Struct delta

`move_quintic_phase` is unchanged in layout. `struct move.u.quintic` grows from `{accel, cruise, decel}` to a variable-length `phases[]` array. Best fit in C:

```c
struct move_quintic_ext {
    double arc_length;
    double v_cap_min;
    uint8_t num_phases;                  // ≤ N_MAX (see §3.2 below)
    struct move_quintic_phase phases[N_MAX];
};
```

Or, given that phase count is now data-dependent, a small-vector pattern with `N_MAX` = 12 phases keeps `struct move` a fixed size and stays cache-friendly. At 8 bytes for `t_end` + 11·3·8 = 272 bytes coefficients per phase, 12 phases = 3.36 KB per move. Trapq moves are allocated individually on the heap (`move_alloc`), so this doesn't cascade into a batched allocation problem.

### Worst-case inflation

Per move, per axis: a FIR kernel of N impulses applied to a 3-phase motion polynomial produces at most `3 · (2·N − 1) = 3·(2N−1)` pieces from that axis alone. `ZVD` (N = 3) gives 15 pieces per axis, MZV (N = 3) gives 15, EI3 (N = 4) gives 21. smooth-IS and bs kernels are single-polynomial per phase → 3 pieces per axis.

At 50 Hz vs 150 Hz (3× kernel-width ratio, the mismatch the spec calls out as worst case) the union of two axes' breakpoint sets has no interior alignment — every Y breakpoint falls strictly between two X breakpoints. So

$$
N_\text{union} \le N_x + N_y - 2
$$

(the −2 accounts for shared move-start and move-end). For FIR×FIR: `ZVD/ZVD = 15 + 15 − 2 = 28 sub-phases`. For FIR×smooth: `ZVD/smooth_zv = 15 + 3 − 2 = 16`. For smooth×smooth: `3 + 3 − 2 = 4`.

Realistic worst-case ceiling with both axes MZV/ZVD: **~28 sub-phases per move**.

### Cost model on step-gen

- `quintic_pick_phase` cost is O(log N) with a binary-search replacement of the current two-branch ladder (trivial: the `t_end` array is sorted). For N = 28 that's ~5 compares per pick; vs ~2 compares today. Evaluator body unchanged — still 11-term Horner.
- `check_active` (`itersolve.c:155-163`) becomes O(N_phases · 10 · 3 = 840 compares) worst-case vs 90 today. Runs once per move at step-gen entry; negligible.
- `integrate_move` / `move_axis_phase_polynomial` — uses the picked phase's polynomial. Same O(log N) pick cost.

No step-rate-level cost explosion. Horner is still 11 mul+add per axis per step.

### Padding / projection math (this is the load-bearing derivation)

Given the wider-kernel axis (say X, f_sh = 50 Hz) has its natural polynomial `P_x(τ)` defined on a sub-interval `[τ_a, τ_b]` in phase-local time. The finer partition splits that interval into sub-sub-intervals `[τ_a, τ_1], [τ_1, τ_2], …, [τ_m, τ_b]`. For each sub-sub-interval `[τ_{k−1}, τ_k]` we need the polynomial in its own phase-local time `δ = t − τ_{k−1}`.

Let the source polynomial be

$$
P_x(τ) = \sum_{j=0}^{10} a_j \, (τ − τ_a)^j
$$

(source phase-local at its own start `τ_a`). For a sub-sub-interval starting at `τ_{k−1} ≥ τ_a`, define `Δ = τ_{k−1} − τ_a ≥ 0`. Substituting `τ = τ_{k−1} + δ = τ_a + (Δ + δ)`:

$$
P_x(τ) = \sum_{j=0}^{10} a_j \, (Δ + δ)^j
     = \sum_{j=0}^{10} a_j \sum_{i=0}^{j} \binom{j}{i} Δ^{j-i} δ^i
$$

Collecting by power of `δ`:

$$
b_i = \sum_{j=i}^{10} a_j \binom{j}{i} Δ^{j-i}, \quad i = 0 \dots 10
$$

This is the **standard Horner/Pascal shift of polynomial basis**, done once per sub-sub-interval at emit time. Cost: `O(d^2) = 121` multiply-adds per axis per sub-sub-interval, done in Python at plan time. Completely dominated by the rest of the composer.

Numerical note: the shift uses only non-negative `Δ ≤ t_phase_duration ~ tens of ms`, and coefficients `a_j` are bounded by `|a_j| ≤ |position| · (1/Δ)^j` from the polynomial construction. No catastrophic cancellation paths.

## 4. Candidate B — per-axis move struct

Break the shared-`t_end` invariant. Each axis has its own `phases[]` array:

```c
struct move_quintic_axis { uint8_t num_phases; struct move_quintic_phase phases[N_MAX]; };
struct move_quintic { ... struct move_quintic_axis x, y, z; };
```

### Invasiveness — affected files

Every site that does a phase-pick has to do one per axis:

1. `klippy/chelper/trapq.h:33-60` — struct redesign; `struct coord c[]` per phase cannot stay (each axis owns its own `phases[]`). ~15 LOC.
2. `klippy/chelper/trapq.c:28-110` — `quintic_phase_eval` becomes scalar-axis; `quintic_pick_phase` returns a per-axis struct; `move_get_coord` fans out to three picks. ~40 LOC.
3. `klippy/chelper/trapq.c:245-286` — `trapq_append_quintic` wire signature explodes (from 99 doubles + 3 t_ends to 3×(N·(11+1)) = ~400 doubles). ~60 LOC including Python ffi layer.
4. `klippy/chelper/integrate.c:141-172` — `move_axis_phase_polynomial` picks the per-axis phase. Already per-axis at the API level; just swaps which array it reads. ~20 LOC.
5. `klippy/chelper/itersolve.c:141-164` — `check_active` per axis. ~20 LOC.
6. `klippy/chelper/kin_extruder.c:41-70` — `pa_move_integrate` picks X and Y separately. ~30 LOC.
7. `klippy/extras/blendmath.py` and the Plan-5 quintic composer — emit three coefficient buffers instead of one. ~50 LOC.
8. Python ffi wrapper in `chelper/__init__.py` for `trapq_append_quintic` signature. ~20 LOC.
9. `struct pull_move` and `trapq_extract_old` motion-report consumers — the motion-report format is already downgrade-to-linear in `pull_move`, so no wire-format break. ~0 LOC.
10. Tests and klipper-sim harness that manipulate quintic moves directly. ~50 LOC.

**Total: ~300 LOC across 8 source files, plus wire-format + ffi rework.** Z-axis gets the same treatment for consistency even though Z is unshaped — extra dead cost.

## 5. Candidate C — shared partition + per-axis phase mask

Form the union partition as in (A) but flag which sub-phases each axis is "really" active on. Saves no memory in the common case (the coefficients still need to be stored somewhere) and adds a mask-check branch into the hot evaluator.

Dismissed: provides none of (B)'s per-axis independence and all of (A)'s partition inflation plus extra branching. No tie-breaker under the "evaluator simplicity" priority.

## 6. Recommendation — Candidate A

Tiebreakers applied in order:

1. **Evaluator simplicity** — (A) keeps the single-phase-pick → Horner-eval structure. (B) forces three parallel phase-picks per position evaluation. (A) wins.
2. **Emit-side simplicity** — (A) needs a Pascal shift per padded sub-sub-interval, done in Python. (B) needs three coefficient buffers, three phase arrays, expanded ffi. (A) wins.
3. **Memory footprint** — (A) at `N_MAX = 12` phases is 3.36 KB/move. (B) at per-axis `N_MAX = 8` is 3×(8·(11·8+8)) = 2.3 KB/move excluding Z. Comparable; (A) slightly heavier, not load-bearing.

All three tiebreakers favor (A) or are neutral. Recommendation: **Candidate A, unified-finer-partition with per-sub-phase Pascal-shifted coefficients**.

Implementation notes:

- `N_MAX = 16` is a safe worst-case ceiling (ZVD/ZVD = 28 upper bound only applies if both axes are both FIR and maximally misaligned, which is possible in configuration; a higher ceiling is justified — `N_MAX = 32` costs 8 KB/move, still inside L1 for a printer). Pick 32 for safety; revisit once sim regression shows actual distributions.
- Replace the two-branch `quintic_pick_phase` ladder with a small linear scan (N ≤ 32) or binary search. Linear scan wins on branch predictor for the common case (N ≤ 4) typical for single-frequency-axis or smooth/bs moves.
- The `struct coord c[11]` xyz-interleaved layout is preserved. A padded sub-phase simply re-holds the same axis polynomial (after Pascal shift) for the non-splitting axis, and the genuinely-split axis holds its own polynomial.
- Add an `__attribute__((flatten))` hint or equivalent inline on the phase-pick helper if profiling shows a branch-predictor cliff at high N.

## 7. Padding worked example (concrete)

X at 50 Hz (kernel width 10 ms), Y at 120 Hz (kernel width ~4.17 ms). Move has accel phase ending at t = 20 ms.

- X natural breakpoints in accel: `{0, 5, 10, 15, 20}` (impulses projected inside the phase).
- Y natural breakpoints in accel: `{0, 2.08, 4.17, 6.25, 8.33, 10.42, 12.5, 14.58, 16.67, 18.75, 20}`.
- Union: 15 distinct breakpoints in the accel phase alone (with some X points coincident with Y points if aligned; in general, no coincidences).

For each sub-interval `[τ_{k−1}, τ_k]`:
- X reads its source polynomial (the MZV-baked degree-10 piece covering `[floor, ceil]` of its natural breakpoints) and applies the Pascal shift by `Δ = τ_{k−1} − τ_\text{source_start}`.
- Y does the same with its own source polynomial.

Emit-time cost: ~15 sub-intervals × 2 axes × 121 mul-adds = 3630 flops for the accel phase. Python-side, per move. Not a hotspot.

## 8. Summary

- Current layout (`trapq.h:35-56`) locks `t_end` across axes — 10 consumers depend on this.
- Padding to the union partition (Candidate A) preserves every invariant, inflates phase count to ≤ ~28 in the pathological FIR/FIR mismatch, costs nothing at step-gen hot-path beyond O(log N) phase pick.
- Per-axis structs (Candidate B) is ~300 LOC across 8 files, changes wire format and ffi.
- Pascal-shift math for padding is standard and numerically stable.
- Recommend Candidate A.

## References

- `klippy/chelper/trapq.h:33-56` — struct definition.
- `klippy/chelper/trapq.c:45-60` — `quintic_pick_phase`.
- `klippy/chelper/trapq.c:91-110` — `move_get_coord` dispatch.
- `klippy/chelper/trapq.c:251-286` — emit path.
- `klippy/chelper/integrate.c:141-172` — integrator phase-pick.
- `klippy/chelper/itersolve.c:141-164` — `check_active`.
- `klippy/chelper/kin_extruder.c:41-70` — PA phase-pick.
- `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md:§3, §6.3` — design context.
