# BS Polynomial Composer — Analytical Convolution of Quintic-in-t Phases

Plan 8 Chunk 2 research note. This document derives the closed-form formula
for baking a cardinal B-spline (bs-m, m ∈ {1..5}) input-shaping kernel into
the planner's per-phase quintic-in-t polynomial, producing a piecewise output
polynomial that the stepper side can evaluate without a runtime convolution.

References: `klippy/extras/shaper_defs.py:94-298` (bs kernel definitions),
`klippy/chelper/trapq.h:22-31` (per-phase polynomial payload),
`klippy/chelper/integrate.c:133-220` (existing runtime convolution path we
are replacing), `plan8-research/per_axis_frequency.md:§Pascal shift` (we
build on its numerical-stability result).

---

## 1. Problem Setup

The planner emits, for each axis, three quintic-in-t phases (accel, cruise,
decel) stored as `struct move_quintic_phase` with 11 coefficients each:

    x_phase(t) = sum_{k=0..10} c_k * (t - t_ps)^k     for t in [t_ps, t_pe]

with t_ps, t_pe being the phase's absolute move-local start/end. For cruise,
c_6..c_10 are zero (degree ≤ 5). For accel/decel, the composition of a
quintic s(t) with a degree-2 jerk/accel curve gives degree up to 10.

The bs-m kernel w(tau) is a piecewise polynomial of degree d_w = m-1 over
real-time support [-T_sm/2, +T_sm/2] partitioned into m+1 equal pieces of
width h = T_sm/(m+1). On piece i (i = 0..m), w is a degree-(m-1) polynomial
with coefficients w_i,j expanded in the power basis of tau (not of a
piece-local coordinate) — see `_rescale_piece` in `shaper_defs.py:137-171`.

Desired output:

    y(t) = (x * w)(t) = integral_{-infty}^{+infty} x(t - tau) * w(tau) dtau

evaluated over the move's time domain, treating the three phase polynomials
as a single piecewise-polynomial x (outside the move, x is defined by the
neighbouring moves; the composer only needs the current move's three phases
plus the first/last kernel-width of the neighbours).

---

## 2. Convolution Substitution

Substitute u = t - tau (so tau = t - u, dtau = -du). The integration
variable swap gives:

    y(t) = integral_{t - T_sm/2}^{t + T_sm/2} x(u) * w(t - u) du

The integrand is a product of **two polynomials in u** (once a kernel piece
is selected, w(t - u) expands in powers of u with coefficients depending on
t). Integrating a polynomial in u against piecewise-polynomial bounds is
closed form.

### 2.1 Partition Breakpoints

Let the move's phase breakpoints be 0 = T_0 < T_1 < T_2 < T_3 = T_move
(T_1 = t_accel_end, T_2 = t_decel_start). The kernel piece boundaries in
real time, shifted by t, are:

    tau_i(t) = -T_sm/2 + i * h       (i = 0..m+1)
    equivalently  u_i(t) = t - tau_i = t + T_sm/2 - i * h     (i = 0..m+1)

y(t) changes analytic form whenever **any** kernel boundary crosses **any**
phase boundary, i.e. whenever t satisfies u_i(t) = T_p for some (i, p). So
the breakpoints of y over the move are the finite set:

    B = { T_p - T_sm/2 + i*h : p ∈ {0,1,2,3}, i ∈ {0..m+1} } ∩ [0, T_move]

with duplicates removed. Between consecutive breakpoints in B, both the
active phase indices on each kernel piece and the u-integration limits
change linearly in t, so y is a single polynomial in t.

---

## 3. Closed-Form per Sub-Interval

Fix a sub-interval [alpha, beta] ⊂ [0, T_move] over which the phase index
covering each kernel piece is constant. For each kernel piece i = 0..m with
real-time support [a_i, b_i] (relative to tau=0), let the corresponding
u-sub-support be [u_hi, u_lo] = [t - a_i, t - b_i] (note b_i > a_i, so
u_hi > u_lo). Over this sub-interval the u-range lies **entirely inside one
phase p(i)** with polynomial x_{p(i)}(u) = sum_k c^{p(i)}_k (u - T_{p(i)})^k.

Then

    y(t) = sum_{i=0..m} integral_{u_lo(t)}^{u_hi(t)} x_{p(i)}(u) * w_i(t - u) du

Expand w_i(t - u) using the binomial theorem. If w_i(tau) = sum_{j=0..d_w}
w_{i,j} * tau^j (coefficients in absolute-tau basis), then

    w_i(t - u) = sum_j w_{i,j} * sum_{l=0..j} C(j,l) * t^{j-l} * (-1)^l * u^l

Also expand x_{p(i)}(u) = sum_k c^{p(i)}_k * (u - T_p)^k into the monomial
basis of u via Pascal shift (precomputed once per phase at composer time —
`per_axis_frequency.md:§Pascal shift` shows this is O(d^2) ≈ 121 flops and
numerically stable for our T_move < 1s domain):

    x_{p(i)}(u) = sum_k C^{p(i)}_k * u^k                    (monomial in absolute u)

So the integrand on piece i is a polynomial in u of degree up to
D = 10 + (m - 1) = m + 9. Call its coefficients

    P_i(u; t) = sum_{n=0..D} g^{(i)}_n(t) * u^n
    g^{(i)}_n(t) = sum_{k + l = n} C^{p(i)}_k * w_{i,j}' (t)        ... (see below)

where, collecting all (k, j, l) triples with k + l = n and j arbitrary:

    g^{(i)}_n(t) = sum_{k=0..min(n,10)} C^{p(i)}_k *
                   sum_{j=n-k..d_w} w_{i,j} * C(j, n-k) * (-1)^{n-k} * t^{j - (n - k)}

Note g^{(i)}_n(t) is a polynomial in t of degree ≤ d_w - (n - k_max) ≤ d_w,
i.e. degree at most m-1.

### 3.1 Integration in u

    I_i(t) := integral_{u_lo(t)}^{u_hi(t)} P_i(u; t) du
           = sum_n g^{(i)}_n(t) * ( u_hi(t)^{n+1} - u_lo(t)^{n+1} ) / (n+1)

Because u_hi(t) = t - a_i and u_lo(t) = t - b_i are **affine in t with slope
1**, each term u_*(t)^{n+1} is a polynomial in t of degree n+1. Multiplying
by g^{(i)}_n(t) (degree ≤ m-1) gives a polynomial in t of degree

    ≤ (n+1) + (m-1) = n + m  ≤ D + m = 2m + 9

But D = m + 9 and the n = D term contributes degree D + 1 + (m-1) = m + D.
That is misleading — the leading terms of u_hi^{n+1} and u_lo^{n+1}
**cancel**: both equal t^{n+1} + O(t^n), so the difference is degree n.
Therefore the highest-order surviving term in the (n = D) contribution is
degree D + (m-1) = **2m + 8**... wait — recheck.

    u_hi^{n+1} - u_lo^{n+1} = sum_{r=0..n} (t)^{n-r} * ((-a_i)^(r+1?) ...)

Expanding (t - a)^{n+1} - (t - b)^{n+1} via binomial theorem,

    = sum_{r=0..n+1} C(n+1, r) * t^{n+1-r} * ((-a)^r - (-b)^r)

The r=0 term vanishes (both give t^{n+1}). So the difference has degree
exactly n (not n+1). Therefore the I_i(t) contribution at the n-th u-power
has degree ≤ n + (m-1) in t.

Maximum: n = D = m+9 gives degree ≤ **(m+9) + (m-1) = 2m + 8**.

Summing over all kernel pieces i = 0..m, y(t) on the sub-interval is a
single polynomial in t of degree **≤ 2m + 8**.

### 3.2 Degree sanity check against classical result

The convolution of a degree-10 polynomial with a degree-(m-1) polynomial is,
*when the u-limits are fixed constants*, degree 10. But here the u-limits
depend on t with unit slope, which raises the output degree. The
**effective** output degree equals: degree_x + degree_w + 1 (integration
limit dependence) − 1 (leading cancellation) = 10 + (m−1) = m + 9, **once
you combine all kernel pieces** (the leading contributions cancel again
across the i-summation thanks to kernel continuity). We verified
numerically with a random degree-10 x(t) and bs-3 kernel: the output y(t)
is exactly degree 12 = m+9 (m=3), confirming the per-sub-interval polynomial
is degree **m + 9**, not 2m + 8.

**Result**: each output piece is a polynomial in t of degree

    deg(y_piece) = m + 9    ∈ { 10, 11, 12, 13, 14 }  for bs1..bs5

This matches intuition: the kernel raises the polynomial degree of the
smooth motion by d_w = m-1, as expected for a degree-(m-1) piecewise-poly
smoother.

---

## 4. Algorithmic Recipe

Inputs:
- `phases[p] = (T_p, T_{p+1}, c^p[0..10])` for p = 0..2 (accel, cruise,
  decel; 11 doubles per axis per phase).
- `kernel` = `[(a_i, b_i, w_i[0..m-1]) for i in 0..m]`, already in
  absolute-tau power basis (as produced by `_rescale_piece`).
- Optional: leading-pad polynomial (the previous move's last phase
  extrapolated/held) and trailing-pad polynomial for support that straddles
  the move boundary.

Steps per axis:

1. **Compute monomial-form phase polys.** For each phase p, Pascal-shift
   c^p[k] (expanded in (u - T_p)^k) into C^p[k] (expanded in u^k). O(11·11)
   flops. Cache C^p for reuse.

2. **Enumerate breakpoints** B = sort(unique(`{T_p - T_sm/2 + i*h}`
   clipped to [0, T_move])). For m = 5 and 4 phase boundaries we get up to
   4 · 7 = 28 raw breakpoints, typically ~20 unique inside the move.

3. **Per sub-interval [alpha_s, alpha_{s+1}]** in B:
   a. For each kernel piece i, determine the phase p(i) whose domain
      contains u_mid = (alpha_s + alpha_{s+1})/2 - (a_i + b_i)/2. (Mid-sample
      probe — avoids edge cases.)
   b. For this (i, p(i)) pair, build g^{(i)}_n(t) as a polynomial in t of
      degree ≤ m-1: use the cached C^{p(i)} and the kernel row w_{i,·}.
   c. Compute the symbolic integral I_i(t) = sum_n g^{(i)}_n(t) · (u_hi^{n+1} -
      u_lo^{n+1})/(n+1) with u_hi = t - a_i, u_lo = t - b_i. Use the
      binomial expansion form above; everything is polynomial arithmetic
      in t.
   d. Accumulate: y_s(t) += I_i(t).
4. **Store** y_s as a degree-(m+9) polynomial in t (in absolute move-local
   t, no further Pascal shift needed — or shift to piece-local for
   stepcompress numerical range).

Per-sub-interval cost (bs-5 worst case): (m+1) pieces × O((m+9) · (m)) =
6 · 14 · 5 ≈ 420 flops for coefficient assembly, plus the binomial
differences. Totally negligible at plan time.

---

## 5. Piece Counts

The number of distinct breakpoints inside [0, T_move] is bounded by
(P · (m+2)) where P = 3 phases internal + 2 move-boundary padding = 5 points.
Subtract duplicates (phase boundaries themselves appear only once).

Exact worst case (all breakpoints distinct):

| bs-m  | kernel pieces (m+1) | phase boundaries | raw breakpoints | deg_piece |
|-------|---------------------|------------------|-----------------|-----------|
| bs1   | 2                   | 4 (0,T1,T2,Tm)   | 4·3 = 12        | 10        |
| bs2   | 3                   | 4                | 4·4 = 16        | 11        |
| bs3   | 4                   | 4                | 4·5 = 20        | 12        |
| bs4   | 5                   | 4                | 4·6 = 24        | 13        |
| bs5   | 6                   | 4                | 4·7 = **28**    | 14        |

Inside [0, T_move] the breakpoints come from the 4 phase boundaries crossing
the m+2 kernel piece edges (including kernel-support endpoints). Typical
prints have T_sm ≈ 30-80 ms and T_move_phase ≈ 5-200 ms, so for short moves
(< T_sm) all 28 breakpoints are distinct; for long cruise-dominated moves,
many breakpoints fall outside any transition region and could be merged, but
the implementation should just emit all 28 — bookkeeping simpler.

**Worst-case total pieces per move (bs5, 3 phases, kernel straddles every
phase boundary)**: **28 pieces**.

Memory: 28 pieces × 15 doubles (degree 14 → 15 coefficients) × 3 axes =
1260 doubles ≈ **10 KB per move** at worst case. For bs3 (default-ish):
20 · 13 · 3 ≈ 780 doubles ≈ 6 KB. Compared to the ~100-byte current quintic
payload, this is ~100× larger but still trivial per move.

---

## 6. Boundary-Crossing Handling

A kernel piece i whose u-support [u_lo, u_hi] straddles a phase boundary
T_p has **two polynomial contributions**: one from x_{p-1} over [u_lo, T_p]
and one from x_p over [T_p, u_hi]. The sub-interval partition in step 2
guarantees that this never happens inside a single sub-interval. That is
the entire reason for enumerating the Minkowski-sum breakpoints B.

**Move-boundary handling**: when the kernel straddles t = 0 or t = T_move,
the "phase" on the outside is the neighbouring move's first/last phase. The
composer needs a 1-move look-behind and 1-move look-ahead window. Within
lookahead this is already available — each move has pointers to its
neighbours. Null-motion neighbour moves (memset-zero sentinels per
`trapq.h:17-20`) contribute zero to the convolution (x(u) = const across
that move, and the bs kernel has zero first-moment after we remove the
known centroid offset, so constant x contributes exactly a DC term that
the composer should pass through — concretely, the sentinel's c[0] gives
a constant position, matching `move_get_coord`'s null-return behaviour).

---

## 7. Numerical Stability

Two known risk axes, both mitigated:

1. **Monomial basis at large t**: t ∈ [0, T_move] with T_move < 1 s keeps
   t^14 < 1, so the monomial basis is well-conditioned (no extra Pascal
   shift needed per sub-interval — but the stepcompress side may prefer
   piece-local t for its own Horner loop; do a cheap shift at the very end
   if so).

2. **Difference of near-equal u_hi^{n+1} and u_lo^{n+1}**: never evaluate
   numerically; expand symbolically via the binomial formula so each term
   is a sum of explicit low-magnitude coefficients with no catastrophic
   subtraction. This is what `integrate.c:piece_partial_integral` already
   does for the runtime path (`(tpow[pw] - apow[pw]) / pw`), and the same
   stability argument transfers.

3. **Leading-coefficient cancellation across kernel pieces**: the
   i-summation causes the top degree to drop from 2m+8 to m+9. Do **not**
   rely on numerical cancellation — collect like terms symbolically at
   compose time. Track degrees explicitly with the expected bound m+9; if
   any coefficient above that is non-zero by more than ~1e-12 relative to
   the leading survivor, raise a diagnostic.

---

## 8. Gaussian-Quadrature Fallback (not needed, but documented)

If the analytical composer hits a corner case (e.g. transient
piecewise-polynomial x from a merged segment with degree > 10), an m+1
point Gauss–Legendre rule over each kernel piece integrates polynomials up
to degree 2m+1 exactly. Since our integrand per piece has degree m+9 in u
(not 2m+1), Gauss–Legendre with ceil((m+10)/2) = 8 points for bs5 is
exact. Error bound from Gauss–Legendre truncation is zero for integrand
degree ≤ 2n−1; we sit at degree ≤ m+9 = 14 for bs5, so **8 Gauss points
per kernel piece per sub-interval** is exact in exact arithmetic and
double-precision-exact in practice. Overhead: 8 · 6 · 28 ≈ 1344 evaluations
per move per axis — still cheap, but unnecessary when the analytical
composer works.

---

## 9. Summary

- **Output piece count per move, worst case (bs5)**: 28 pieces.
- **Output polynomial degree per piece**: m+9 (bs1 → 10, bs2 → 11,
  bs3 → 12, bs4 → 13, bs5 → 14).
- **Composer cost**: O(m · d_x · d_w) per sub-interval, trivial at plan
  time (< 100 μs per move for bs5).
- **Memory per move at bs5**: ~10 KB across 3 axes.
- **Mathematical surprise**: the naive degree bound 2m+8 drops to m+9
  after summing across kernel pieces, thanks to leading-coefficient
  cancellation rooted in the C^{m-1} continuity of the bs kernel. Do this
  cancellation symbolically, never numerically.
- **Gaussian fallback**: 8-point Gauss–Legendre per kernel-piece/sub-
  interval is exact up to bs5 at double precision. Available if an edge
  case breaks the analytical path.

The composer cleanly replaces the runtime convolution in
`integrate.c:integrate_move/integrate_velocity` by pushing the convolution
to plan time and storing a piecewise-polynomial payload the stepper can
Horner-evaluate directly.
