# XDIRECT buzz — phase-locked update scheduling

Load-bearing math and control flow for the generator in `buzz_xdirect.rs`. The
companion exists because the kernel cites the scheme by name; this holds the
formulas. Symbols reuse `buzz_gen`: ω (`omega`), μ (`mu`, chirp rate), A
(`amplitude_mm`), `lut_step` (PHASE_LUT microstep in mm).

## Carrier phase and instantaneous frequency

```
φ(t)      = ω·t + ½·μ·t²              // buzz_gen::phase(p, t)
ω_inst(t) = ω + μ·t                    // buzz_gen::omega_inst
f_inst(t) = ω_inst(t) / 2π            // Hz
A_eff(t)  = A · ω / ω_inst(t)         // constant-peak-velocity taper; == A for a tone
```

## The scheduling rule (replaces the displacement grid)

Updates land on a uniform **carrier-phase** grid with spacing Δφ = 2π/N. The
k-th update time solves φ(t) = φ_k for φ_k = φ(t_cursor) + Δφ:

```
½·μ·t² + ω·t − φ_k = 0
        ⎧ (−ω + sqrt(ω² + 2·μ·φ_k)) / μ      μ ≠ 0   (positive root)
t_k  =  ⎨
        ⎩ φ_k / ω                            μ = 0   (tone)
offset_k  = round(position_rel(p, t_k) / lut_step)    // env + A_eff folded in
cycle_k   = cycle_at(p, t_k)                           // unchanged from today
```

Closed form — no forward scan, no `refine_extremum`, no `scan_dt` velocity
bound. Forward progress is structural: φ_k strictly increases by Δφ.

**Why this sweeps with the buzz.** dφ/dt = ω_inst, and each update advances φ by
2π/N, so updates per second = ω_inst / (2π/N) = N · f_inst(t). The update rate
rides the instantaneous frequency exactly; within one cycle (≈constant ω_inst)
the N updates are evenly spaced in time. Uniform spacing ⇒ ZOH spectral images
sit at (N±1)·f, (2N±1)·f … — harmonics of the tone, out of band — instead of
smearing into the audible band the way the non-uniform displacement grid does.

## Choosing N — exact rate, transitions only at turning points

The realized rate is `N·f_inst` *exactly* (CAP-2): N is an integer, so the
carrier period always divides evenly into the grid. `target_rate` (~motion sample
rate, just under the step-output re-arm cap) only bounds N from above; it is never
the realized rate. "Closest to 10 kHz" is the defect we are removing — 10 kHz does
not divide the period, which is what smeared the spectrum.

```
N_rate = round(target_rate / f_inst(t))           // upper bound from the budget
N_amp  = 4 · round(A_eff(t) / lut_step)            // path/cycle ≈ 4·A_eff; cap so
                                                    // the grid is never finer than
                                                    // ~LUT resolution
N      = max(4, min(N_rate, N_amp))
N      += (4 − N % 4) % 4                           // round UP to a multiple of 4
```

- `N % 4 == 0`, **with the phase grid anchored at a turning point**, puts every
  quarter-phase `φ = n·π/2` on a grid point: both extrema (k = N/4, 3N/4) and both
  zero crossings (k = 0, N/2). So `offset` at each extremum is the exact commanded
  peak — amplitude pinned with no special case (CAP-3) — and that alignment holds
  across any N change because both old and new N are ÷4 sharing the turning point.
- `N_amp` caps the count when `A_eff` tapers near the top of a sweep, so the grid
  stays near LUT resolution and the per-update offset change is ≤ ~2 LUT steps
  everywhere (CAP-4). Every grid point is emitted — no-op writes near an extremum
  are not skipped, since the zero-order-held waveform is identical either way, and
  emitting keeps the spacing uniform (CAP-2).

**When N may change (CAP-7).** Recompute N only **at a carrier turning point**
(velocity zero, φ = π/2 + nπ) — never mid-segment. The motor is momentarily
stationary there, so the change in Δφ introduces no spacing discontinuity while
the carrier is moving, and the shared turning point keeps the grid phase-coherent.
Concretely: hold N fixed across each half-cycle (turning point → next turning
point, N/2 updates at Δφ = 2π/N); at the turning point re-evaluate
`A_eff`/`f_inst`, pick the new N, and resume the grid anchored at that φ. This is
exact within every segment and continuous in φ across the seam.

## Net-zero close (CAP-6)

The trapezoidal envelope hits 0 at `total_seconds`, so `position_rel(total) = 0`
⇒ `offset = 0`. The last phase-grid point generally falls short of `total`, so
append one final update at exactly `total_seconds` with `offset_steps = 0`. The
axis parks on base. This single appended update is off the phase grid by design
and does not affect the tone.

## What is deleted

- `XdirectConfig::grid_steps`, `for_rate`'s `v_peak / (rate·lut)` inversion →
  replaced by `n_per_cycle` and the N-selection above.
- `scan_dt` and its `v_max = A·(ω_hi + 1/ramp)` traverse bound → no scan.
- The `refine_extremum` branch and the `at_end && offset != last` grid patch in
  `next_update` → extrema are grid points; only the net-zero append remains.
