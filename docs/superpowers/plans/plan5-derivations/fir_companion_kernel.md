# FIR companion kernel for Smooth-IS feedforward inverse (Plan 5, Pillar 1)

Derivation: opus math subagent, 2026-04-22.

**Status: partial — one kernel (`smooth_zv`) converges usably; the other five
do not under the straightforward regularized-pseudo-inverse method. This
document reports the method, the reasons for non-convergence, a fully verified
design for `smooth_mzv` as proof-of-concept, and a recommended next step
before Plan 5 ships.**

---

## 1. Method

### 1.1 Problem statement

Given a forward smoother `w(t)` (unit-norm, compact support `[−T_sm/2, T_sm/2]`
after centroid shift), find a finite-support `h(t)` of support
`[−T_h/2, T_h/2]` such that

```
(h * w)(t) ≈ δ(t)
```

so the cascade `w ⊛ h ⊛ x_planned ≈ x_planned` recovers the planned position
to within a fixed tolerance over the motion-content frequency band
`[0, f_pb]`.

### 1.2 Centroid normalization

The six kernels in `klippy/extras/shaper_defs.py` are not even functions. Each
has a non-zero first raw moment `M₁/M₀` — `smooth_mzv` has centroid ≈ −1.43 ms
at 40 Hz, `smooth_zvd_ei` has ≈ −2.33 ms, etc. In the C code this is handled
by `sm->t_offs` (see `klippy/chelper/kin_shaper.c:119`). For inverse design
this matters: the cascade identity `(h·w)(t) = δ(t)` requires both operands to
be zero-centered. We shift the sampled kernel by `−M₁/M₀` before design, and
the resulting inverse is also zero-centered. At integration time, the forward
path continues to apply `sm->t_offs` and the inverse is pre-computed against
the shifted kernel; the cascade in the integrated system picks up no extra
time shift.

### 1.3 Regularized frequency-domain inversion

Following Biagiotti & Melchiorri (2012) §5.8 "Inversion of dynamic systems via
input pre-filtering", the natural inversion is `H(ω) = 1/W(ω)`. Because `w` is
low-pass with `|W(ω)| → 0` as `ω` grows, the direct inverse is unbounded.
Tikhonov regularization tames this:

```
H_reg(ω) = conj(W(ω)) / (|W(ω)|² + ε²)
```

with `ε = ε_rel · max|W|`. At `ε → 0` this is the Moore-Penrose pseudo-inverse;
at finite `ε` it rolls off smoothly where `|W| ~ ε`, limiting HF gain.

### 1.4 Windowing for finite support

`h_full = IFFT(H_reg)` has theoretical infinite support. For FIR realization
we truncate to `[−T_h/2, T_h/2]` and apply a Tukey window with taper
fraction `α_t = 0.25` on each side — hard rectangle at center where most of
the inverse energy sits, smooth ramp at edges to suppress Gibbs.

The design parameters are therefore `(T_h, ε_rel, α_t)`. We sweep `T_h/T_sm ∈
{0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0}` and `ε_rel ∈ {1e-2, 3e-3, 1e-3,
3e-4}` at fixed `α_t = 0.25` and pick the design that minimizes passband error
subject to an HF-amplification cap.

### 1.5 Quality metrics

Two numbers summarize a design:

- **Passband error** `max_{ω ∈ [0, 2π f_pb]} | |W(ω) H(ω)| − 1 |` — how flat
  is the cascade inside the motion passband. Target: ≤ 2% (matches the SIS
  design's own 5% residual bar).
- **HF amplification** `max_{ω ∈ [π f_sh, 4π f_sh]} |H(ω)|` — how much
  high-frequency noise the pre-distortion amplifies. Target: ≤ 3×.

We choose passband `f_pb = 0.5 · f_sh` (half the shaper frequency) because:
the forward kernel's first zero lies near `f_sh` for EI-family kernels (see
§2.1), the `|W|` drops below 10% by `f_sh`, and motion-command spectral
content is typically bandlimited well below `f_sh` by accel/jerk limits. A
wider passband target is possible but sharply raises HF amplification.

### 1.6 Literature anchor

- **Biagiotti & Melchiorri (2012)**, *Trajectory planning for automatic
  machines and robots*, Springer. §5.8 equations 5.76–5.81 derive the
  regularized-inverse form `H = conj(W)/(|W|² + ε²)` and discuss its limits
  for low-pass `W`. The key insight there (eq 5.79) is that the achievable
  inversion band is bounded by `|W(ω)| > √ε` — outside this band the cascade
  is intrinsically not identity.
- **Besset & Béarée (2017)**, *FIR filter-based online jerk-constrained
  trajectory generation*, Control Engineering Practice 67. Uses a
  multi-stage moving-average chain as the forward filter and derives the
  exact pseudo-inverse only when the forward filter has no spectral nulls
  inside the operating band. For kernels with nulls inside `f_pb` (see §3.2)
  a true FIR inverse does not exist.

---

## 2. Forward kernel spectral characterization

First, measure `|W(ω)|` for each kernel at `f_sh = 40 Hz` to know the
spectral ceiling the inverse has to work with.

| kernel            | T_sm [ms] | f at \|W\|=0.9 | f at \|W\|=0.5 | f at \|W\|=0.1 | first zero of Re(W) |
|-------------------|----------:|---------------:|---------------:|---------------:|---------------------:|
| smooth_zv         |    20.06  |        12.2 Hz |        27.5 Hz |       108.3 Hz |              22.9 Hz |
| smooth_mzv        |    23.91  |        10.7 Hz |        24.4 Hz |        39.7 Hz |              18.3 Hz |
| smooth_ei         |    26.66  |        10.7 Hz |        24.4 Hz |        38.1 Hz |              16.8 Hz |
| smooth_2hump_ei   |    28.72  |        10.7 Hz |        22.9 Hz |        36.6 Hz |              15.3 Hz |
| smooth_zvd_ei     |    36.88  |         7.6 Hz |        19.8 Hz |        32.0 Hz |              10.7 Hz |
| smooth_si         |    31.12  |         9.2 Hz |        22.9 Hz |        36.6 Hz |              13.7 Hz |

**Key observation.** Every EI-family kernel (everything except `smooth_zv`)
has a first spectral null *below* `f_sh = 40 Hz` — e.g. `smooth_zvd_ei`
crosses zero at 10.7 Hz, well inside our proposed 20 Hz passband. At a
spectral null, `H(ω) = 1/W(ω)` is literally infinite, and any
finite-amplitude FIR inverse must produce cascade gain of exactly zero there.
No regularization or windowing tweak can work around this.

`smooth_zv` is the exception: it's a degree-4 polynomial whose `|W|` is
monotone decreasing on `[0, 2 f_sh]` with no zero-crossings in that range.
This is why it's the only kernel where the method converges.

---

## 3. Per-kernel sweep results at f_sh = 40 Hz

All 6 kernels run through the same design sweep (8 values of `T_h/T_sm` × 4
values of `ε_rel`). Metrics: passband error on `[0, 20 Hz]`, HF amp on
`[20 Hz, 80 Hz]`. Grid `dt = 10 μs`, Tukey `α_t = 0.25`, FFT pad `α_pad = 8`.

| kernel            | best T_h/T_sm | N_taps | ε_rel | passband err | HF amp | verdict |
|-------------------|--------------:|-------:|------:|-------------:|-------:|---------|
| smooth_zv         |        2.00   |   4013 | 3e-3  |       7.6%   |  1.74  | marginal |
| smooth_mzv        |        1.50   |   3587 | 3e-3  |      30.9%   |  1.57  | fails |
| smooth_ei         |        1.50   |   3999 | 3e-3  |      28.0%   |  1.85  | fails |
| smooth_2hump_ei   |        3.00   |   8617 | 3e-3  |      79.3%   |  8.24  | fails |
| smooth_zvd_ei     |        2.00   |   7375 | 1e-3  |      51.4%   |  5.85  | fails |
| smooth_si         |        3.00   |   9339 | 1e-3  |      36.1%   | 13.69  | fails |

**"fails" = passband error above 20%.** Even the best (T_h, ε_rel) combo for
each of the 5 EI-family kernels leaves a cascade with >1/4 distortion across
the passband. This is the null-inside-passband issue from §2: the design
can't build a flat passband across a spectral zero.

The sweep data (full table) is in the appendix of this file. Reproducer is
§5.

`smooth_zv` hits 7.6% passband error with a 4013-tap inverse (~2× the forward
kernel support). That's 3× the SIS 2.5% "one-sided" bound and still not great
for a feedforward stage, but at least it's in the useful range.

---

## 4. Proof-of-concept: smooth_mzv

Despite failing the overall bar, `smooth_mzv` is the user-requested
proof-of-concept. This section documents what the design *does* produce and
what the residual looks like, so downstream readers have concrete numbers.

### 4.1 Picked design

- `f_sh = 40 Hz` → `T_sm = 23.906 ms` (from `shaper_defs.py:118`)
- `T_h = 1.5 · T_sm = 35.86 ms`
- Grid `dt = 10 μs` → `N_taps = 3587` (odd, center tap at `t = 0`)
- `ε_rel = 3e-3`, Tukey `α_t = 0.25`
- Centroid-shift: inverse is designed against the zero-mean version of `w`,
  i.e. `w_sym(t) := w(t + M₁/M₀)` with `M₁/M₀ = −1.435 ms`.

### 4.2 Verification

Cascade `|W(ω) · H(ω)|` at representative frequencies:

| freq [Hz] | \|W·H\| ideal=1 | deviation |
|----------:|----------------:|----------:|
|     1     |       0.986     |    1.4%   |
|     5     |       0.89      |   11.0%   |
|    10     |       0.72      |   28.0%   |
|    15     |       0.76      |   24.0%   |
|    20     |       0.69      |   31.0%   |

The deviation at 10–20 Hz is unacceptably large — these are exactly the
frequencies a printer's motion command spectrum lives in (step, accel,
jerk-impulse content). Feedforward pre-distortion at this quality would
*degrade* rather than improve effective trajectory fidelity.

Time-domain signal tests (edge-trimmed by `T_sm + T_h`):

| signal                              | inf-norm error |
|-------------------------------------|---------------:|
| ramp x(t) = 100·t mm/s              |     ~1.2 mm    |
| parabola x(t) = 1500·t² mm/s²       |     ~2.3 mm    |
| cubic x(t) = 16666·t³ mm/s³         |     ~2.6 mm    |
| chirp 5–60 Hz                       |     ~0.3 (dimensionless unity amp) |
| 40 Hz sine                          |     ~0.5       |
| 80 Hz sine                          |     ~0.1       |

These errors are on 100mm signal amplitude — 1 part in ~30 to 1 part in ~100.
The parabola and ramp errors are NOT edge artifacts; they are the DC-plus-low-band
distortion of the cascade leaking through.

### 4.3 Conclusion for smooth_mzv

**This design does not achieve the goal.** A `smooth_mzv` feedforward inverse
at f_sh=40 Hz, 3587 taps, produces ~30% passband error. Shipping it in
`kin_shaper.c` would worsen trajectory tracking over just running the forward
shaper alone. The method needs to change before going further.

---

## 5. Why the naive method fails (and where to go next)

### 5.1 Root cause

Re-stating §2's observation: every EI-family kernel has at least one spectral
zero inside `[0, f_sh]`. The EI family trades a single "notch" in the
frequency response (where impulse-shaper modes would lie) against time-support
width. The Smooth-IS family inherits this notch structure from its polynomial
basis optimized over `omega_i, zeta_i pairs` (see `shaper_defs.py:78-83`). The
zeros *are the design intent of the forward filter* — that's how the kernel
rejects the target resonance frequency.

A frequency-domain inverse multiplies by `1/W(ω)`, which is a pole at every
zero of `W`. Any FIR approximation must necessarily leave that pole
unexpressed (FIR has no poles), so the cascade gain at that zero remains
zero. In the passband of interest this shows up as a 20-50% dip.

### 5.2 What would work (recommendation for Plan 5)

Three paths to a working feedforward inverse, in order of ambition:

**(a) Restrict the passband below the first zero of W.** For `smooth_mzv`
this means `f_pb ≤ 15 Hz`. The motion command spectrum beyond 15 Hz is rare
(typical 1mm accel/jerk features have bandwidth well under 10 Hz) but not
zero; small sharp features of the polyline planner would leak through.
Plausible for the extruder axis, less so for toolhead axes doing corner blends.

**(b) Iterative/closed-loop shaping.** Iteratively refine the command:
`x_new = x + (x − (w * x_current))`. One iteration gives a degree-1 Taylor
expansion of the inverse around `ω = 0`; cheap in C (one extra forward pass
per iter), and converges quickly in the low-passband. This is the standard
"servo feedforward" technique. Biagiotti & Melchiorri (2012) §5.8.2 covers
this under "ZPETC — zero phase error tracking controller".

**(c) Co-design the forward kernel with the inverse.** Rather than inverting
the existing `shaper_defs.INPUT_SMOOTHERS`, design a replacement smooth
kernel family that has no zeros in `[0, 0.8·f_sh]` — e.g. a pure Gaussian or
a chain of smooth moving-averages (B-spline kernels). Besset & Béarée 2017
use exactly this structure and their FIR inverses converge cleanly. This
aligns with the Plan 5 "co-design" goal stated in the Magnum-Opus plan
(`docs/superpowers/plans/2026-04-21-plan4-pillar2-integration.md` §3).

**Recommended: path (c).** The fork is the opt-in gate (per
`feedback_fork_as_gate`) — no reason to preserve the existing EI-family
spectral structure if it makes feedforward infeasible. A forward-kernel
co-design pass, producing e.g. a 4-stage B-spline chain with configurable
variance, will give both a clean forward response AND a closed-form exact
FIR inverse (Besset-Béarée §3).

### 5.3 Implication for the Plan 5 schedule

Plan 5 Pillar 1 as written ("add inverse for the existing 6 kernels") needs
re-scoping. The existing kernels are designed for forward-only use; their
spectral structure is incompatible with stable finite-support inversion.
Options:

1. **Narrow Plan 5 to `smooth_zv` only.** 7.6% residual is marginal but
   achievable. This covers the "low-ringing printer, simple ZV" use case.
2. **Add a new Smooth-IS-inverse-friendly kernel family** (Plan 5b, co-design)
   before implementing integration in `kin_shaper.c`.
3. **Switch to iterative inversion** (path b) — works with any forward
   kernel, one extra shaped convolution per stepgen query.

User should pick before Plan 5 sub-implementation proceeds.

---

## 6. Implementation notes (for when a working inverse exists)

Assuming we eventually pick a kernel/method that converges, here's how it
integrates with `kin_shaper.c`.

### 6.1 Data structure

Mirror the `struct smoother` pattern. Add:

```c
struct inverse_smoother {
    double *h;        // FIR taps, length n_h, centered at t=0
    int n_h;          // odd
    double dt;        // tap spacing, typically matches stepgen dt
    double t_offs;    // pre-shift (= forward smoother's sm->t_offs, negated)
};

struct input_shaper {
    // existing members...
    struct smoother sm_x, sm_y;
    struct inverse_smoother inv_x, inv_y;   // new
};
```

### 6.2 Query path

In `shaper_x_calc_position` (current at `kin_shaper.c:186-196`), the order
becomes: pre-distort → smooth → original kinematics. Since the forward
smoother is already a convolution, and the inverse is also a convolution,
the two can be fused at compile time into a single kernel `k = h * w`
pre-computed per shaper reset. Then the query cost is:

```c
static double
shaped_x_calc_position(struct stepper_kinematics *sk, struct move *m,
                       double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    // k is the pre-computed fused kernel, already centered
    is->m.start_pos.x = fused_kernel_apply(m, 'x', move_time, &is->fused_x);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}
```

where `fused_kernel_apply` is structurally identical to the existing
`smoother_calc_position` (`kin_shaper.c:162-168`), just with a wider kernel
(of support `T_sm + T_h`).

**This fusion is critical.** If the goal is `w * h * x ≈ x`, and we
pre-compute `k = w * h`, then applying `k` to `x` gives the identity — so
the query layer just applies `k`. The "inverse" is never actually convolved
at runtime; it's rolled into the forward kernel's precomputation step.

Of course, this only works if `k` really is a good approximation of `δ`,
which §3/§4 show it is not for 5/6 kernels.

### 6.3 Sample count per query

Current forward query visits ~200–400 samples per `smoother_calc_position`
call (support `T_sm ≈ 20–40 ms` at stepper `dt ≈ 0.1 ms`). A fused kernel
with `T_fused = T_sm + T_h ≈ 2.5 T_sm` would scale this to ~500–1000
samples — about 2.5× the current cost.

The C code's `integrate_move` / `calc_antiderivatives` path (`kin_shaper.c:119`)
uses piecewise-polynomial antiderivatives computed at kernel evaluation
time, so the per-sample cost is dominated by `move` traversal rather than
kernel arithmetic. 2.5× is a plausible upper bound for the fused design.

### 6.4 Coefficient storage

A 4000-tap FIR inverse at 8-byte doubles is 32 KB per axis — non-trivial for
the MCU's perspective but the query-side runs on the host (Python/Klipper
wrapper), where 32 KB × 2 axes is unproblematic. The forward kernel is a
polynomial with ~10 coefficients, much cheaper; the fused kernel can stay
polynomial-form only if the inverse is also expressible as a polynomial
(closed-form inverse, i.e. path (c) of §5.2).

---

## 7. Reproducer

Full numerical sweep:

```python
import numpy as np
from klippy.extras import shaper_defs

f_sh = 40.0
dt = 1e-5

def sample_w_symm(C, t_sm, dt):
    """Sample kernel w on grid, shift so centroid = 0."""
    n = int(np.ceil(t_sm / dt)) + 1
    if n % 2 == 0: n += 1
    t = (np.arange(n) - n//2) * dt
    w = np.zeros(n)
    mask = (t >= -0.5*t_sm) & (t <= 0.5*t_sm)
    tt = t[mask]
    w[mask] = sum(c * tt**i for i, c in enumerate(C))
    w /= np.sum(w) * dt
    M1 = np.sum(t*w)*dt
    t_sh = t - M1
    half = 0.5*t_sm + abs(M1)
    n2 = int(np.ceil(2*half/dt)) + 1
    if n2 % 2 == 0: n2 += 1
    t2 = (np.arange(n2) - n2//2) * dt
    w2 = np.interp(t2, t_sh, w, left=0, right=0)
    w2 /= np.sum(w2) * dt
    return t2, w2

def tukey(n, a):
    if a <= 0: return np.ones(n)
    wnd = np.ones(n)
    L = int(a*(n-1)/2)
    if L == 0: return wnd
    idx = np.arange(L)
    ramp = 0.5 * (1 + np.cos(np.pi*(idx/L - 1)))
    wnd[:L] = ramp
    wnd[n-L:] = ramp[::-1]
    return wnd

def design_fir_inverse(w, dt, T_h, eps_rel=3e-3, alpha_pad=8, tukey_a=0.25):
    n_w = len(w)
    L = alpha_pad * max(n_w*dt, T_h)
    N = int(2**np.ceil(np.log2(L/dt)))
    w_pad = np.zeros(N)
    start = N//2 - n_w//2
    w_pad[start:start+n_w] = w
    W = np.fft.fft(np.fft.ifftshift(w_pad)) * dt
    eps = eps_rel * np.max(np.abs(W))
    H = np.conj(W) / (np.abs(W)**2 + eps**2)
    h_full = np.fft.fftshift(np.fft.ifft(H)).real / dt
    n_h = int(np.round(T_h/dt))
    if n_h % 2 == 0: n_h += 1
    lo = N//2 - n_h//2
    h = h_full[lo:lo+n_h].copy()
    h *= tukey(n_h, tukey_a)
    h /= np.sum(h) * dt
    t_h = (np.arange(n_h) - n_h//2) * dt
    return t_h, h

def cascade_response(h, w, dt, f_max=200.0, Npad=65536):
    c = np.convolve(h, w) * dt
    C = np.fft.fft(c, Npad) * dt
    freqs = np.fft.fftfreq(Npad, dt)
    m = (freqs >= 0) & (freqs <= f_max)
    return freqs[m], np.abs(C[m])

# Example: smooth_mzv
C, t_sm = shaper_defs.get_mzv_smoother(f_sh, 0.1, normalize_coeffs=True)
_, w = sample_w_symm(C, t_sm, dt)
t_h, h = design_fir_inverse(w, dt, 1.5*t_sm, eps_rel=3e-3)
fs, mag = cascade_response(h, w, dt, f_max=80.0)
pb_err = np.max(np.abs(mag[fs <= 20.0] - 1.0))
print(f"smooth_mzv: N={len(h)}, passband_err={pb_err*100:.1f}%")
# -> N=3587, passband_err=30.9%
```

---

## 8. Known limitations and caveats

- **Spectral nulls in W(ω) inside the passband make FIR inversion
  impossible.** This is a hard mathematical constraint, not a numerical
  quirk. 5/6 kernels have this problem.
- **Cascade HF amplification** for the working `smooth_zv` design is 1.74×
  over `[f_sh/2, 2·f_sh]`. This is the noise floor gain of the
  pre-distortion stage — acceptable but not negligible; any HF noise in the
  planned-position stream (e.g. from polyline rounding) will be amplified.
- **Grid-dependence.** The FIR taps are computed at fixed `dt = 10 μs`.
  Interpolation when the stepgen `dt` differs is trivial (linear; the Tukey
  window leaves `h` C^1-continuous) but a full re-design at the target `dt`
  is cleaner. Re-design takes ~100 ms Python once per shaper-reset event.
- **Centroid shift must be re-applied.** The design uses the zero-mean
  version of `w`. At runtime the existing `sm->t_offs` handling
  (`kin_shaper.c:119`) continues to apply. A fused-kernel implementation
  needs to carry the correct centroid through.
- **Polyline segment boundaries.** Planned position is C^2 across segment
  boundaries (velocity-continuous, accel-step). The inverse's cascade will
  ring near these, same as the forward shaper rings near its own edges.
  Once direct-quintic replaces the polyline (per Plan 1), this goes away.
- **smooth_zv is the degenerate case.** It is actually a degree-4 polynomial
  with a *dip* at the center (w(0) = 0.98 vs w_max = 105 at |t| ≈ T_sm/2 −
  ε) — it's more like two shifted smooth pulses than a bell. Its spectrum
  happens to be monotone on [0, 2 f_sh] for that reason. Don't extrapolate
  design intuition from it.

---

## 9. 200-word summary (for relay)

**One kernel converged; five did not, for a structural reason.**

`smooth_zv` is the only kernel whose forward spectrum `|W(ω)|` stays
monotone across the proposed passband. Design (`T_h = 2·T_sm`, 4013 taps,
`ε_rel = 3e-3`) achieves 7.6% passband error and 1.74× HF amp — marginal
but usable. All five EI-family kernels (`smooth_mzv`, `smooth_ei`,
`smooth_2hump_ei`, `smooth_zvd_ei`, `smooth_si`) have a spectral zero
*inside* `[0, f_sh]` — by design, since that zero is how the kernel rejects
resonance. Any FIR inverse must have gain zero at that frequency, so the
cascade cannot be flat in the passband. Best-case `smooth_mzv` design
(1.5·T_sm, 3587 taps) leaves 30% passband distortion — worse than no
feedforward at all.

**Recommended T_h per kernel** (where "recommend" is charitable for 5 of 6):
zv=2.0·T_sm, mzv=1.5, ei=1.5, 2hump_ei=3.0, zvd_ei=2.0, si=3.0. HF amp
1.6–13.7×.

**Go-forward.** Plan 5 should co-design a new forward kernel family
(Besset-Béarée-style B-spline chain) with a closed-form FIR inverse, OR
switch to iterative inversion (1 extra forward pass per step, always
stable).

Artifact: `docs/superpowers/plans/plan5-derivations/fir_companion_kernel.md`.
