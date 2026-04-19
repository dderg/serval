# Sub-spec 6g — Inverse-Shaper Pre-Compensation

**Date:** 2026-04-19
**Status:** pending — can start independently of 6d/6e/6f
**Depends on:** nothing architecturally (works on any planner-emitted trajectory). Benefits from 6d+6e (smoother commanded input = inverse filter produces smaller pre-distortion, less chance of actuator saturation)
**Novel content:** yes — Phase-0 research gap still open in 2026; no FDM firmware ships this; no CNC controller documentation describes convolutional-shaper inverses combined with corner blending

---

## What this sub-spec does

Add a post-planner filter that **pre-distorts the commanded trajectory** such that, after the input shaper convolves the commanded position into physical motion, **the physical motion matches the user's desired geometry exactly**.

Today:
```
gcode → planner → commanded(t) → [shaper] → physical(t) ≠ desired
```

With 6g:
```
gcode → planner → desired(t) → [h⁻¹] → commanded(t) → [shaper h] → physical(t) = desired
```

The inverse filter `h⁻¹` undoes the shaper's smoothing ahead of time. This is the final piece of the "ultimate quality" story: post-shaper physical error drops from ~0.3mm (today) to sub-50µm.

---

## The math

### Shaper as an FIR filter

For shaper type X (ZV, MZV, EI, ZVD) with impulses `A = [a₀, a₁, ..., aₙ]` at times `T = [t₀, t₁, ..., tₙ]`:

```
p_physical(t) = (1/ΣA) · Σᵢ aᵢ · p_commanded(t − tᵢ)
```

In z-transform (at sample interval dt, offsets kᵢ = tᵢ/dt):

```
H(z) = (1/ΣA) · Σᵢ aᵢ · z^(−kᵢ)
```

### Inverse filter

```
H⁻¹(z) = ΣA / (Σᵢ aᵢ · z^(−kᵢ))
```

This is an **IIR filter**. Its stability depends on the roots of the denominator polynomial all lying inside the unit circle (for causal stability) or outside (for anti-causal). For minimum-phase shapers, causal implementation is stable; otherwise we use anti-causal (preview-based) implementation, which our look-ahead architecture supports.

### Stability check per shaper type

Numerically verify for each of {ZV, MZV, EI, ZVD}: compute the roots of the characteristic polynomial
```
a₀ · z^kₙ + a₁ · z^(kₙ−k₁) + ... + aₙ = 0
```
at representative (freq, ζ, dt) combinations. Classify each shaper as:

- **Minimum-phase**: all roots inside unit circle → causal IIR stable
- **Maximum-phase**: all roots outside → anti-causal IIR stable
- **Mixed**: some roots inside, some outside → split into causal + anti-causal subfilters; implementable with look-ahead buffering

MZV (the most common shaper for Klipper/Kalico) has all-positive impulse amplitudes; typical FIR filters with positive coefficients are often minimum-phase, but must be verified.

---

## Verification plan (before any Kalico-side code)

### Phase 1: numerical stability (2 days)

`klipper-sim/examples/verify_inverse_shaper.py`:

1. For each shaper type in {ZV, MZV, EI, ZVD}:
   1. Generate `h` at representative params (60–200 Hz, ζ ∈ {0.05, 0.1, 0.15}, dt=100µs)
   2. Compute roots of the characteristic polynomial
   3. Classify as minimum-phase / maximum-phase / mixed
   4. If mixed: factor into causal and anti-causal components

2. For each classified shaper: implement the inverse and verify numerically:
   - Feed a synthetic input `p(t) = step, ramp, sinusoid, or arc trajectory`
   - Compute `p_commanded = h⁻¹ · p`
   - Compute `p_physical = h · p_commanded`
   - Assert `|p_physical − p| < 1µm` across the full trajectory

### Phase 2: real-trajectory validation (2 days)

Using existing simulator runs:

1. Take `klipper-sim` output CSV (commanded + shaped columns)
2. Apply `h⁻¹` to the commanded stream off-line
3. Re-convolve the pre-distorted commanded with `h`
4. Plot with `examples/plot_path_fidelity.py` — should show post-shaper (red) converging onto desired (grey) within tolerance

### Phase 3: actuator saturation check (1 day)

The inverse filter may demand commanded acceleration beyond `a_max` if the desired trajectory has sharp discontinuities. Measure:

1. For the `sharp_short.gcode` run (90° corners every 0.5mm, blend-arc cd=0.2, max_a=45k):
   - Apply inverse MZV to commanded (x, y)
   - Compute `|a_commanded|_max`
   - If > 45k, pre-compensation is saturating — either (a) need 6d first for smoother input, (b) reduce commanded to a feasible subset, (c) limit the inverse gain

2. Same for cube-slice run. Report peak-demand acceleration for both.

If phase 3 shows < 55k peak demand (within 25% margin of the 45k budget), we can ship 6g on top of current arc blender without 6d.

If peak demand exceeds ~70k, 6d becomes a prerequisite.

---

## Implementation (after verification passes)

### Location

Option A (recommended): a new module `klippy/shaper_inverse.py` that hooks into the step generation path in `klippy/extras/input_shaper.py`. The existing input_shaper already convolves commanded with shaper impulses; we add a pre-convolution step that applies the inverse.

Option B: implement in C alongside `input_shaper.c` for performance. Not needed for first ship — Python is fast enough for trajectory-level convolution.

### Config

```
[input_shaper]
shaper_type_x: mzv
shaper_freq_x: 180
damping_ratio_x: 0.1
enable_inverse_compensation: false   # opt-in until hardware-validated
```

**Flag exception:** per memory, the fork avoids runtime feature flags. 6g gets a **one-time exception** because inverse-shaper pre-compensation can destabilize mis-calibrated printers (wrong shaper params → inverse produces diverging commanded trajectory). Opt-in default is safer during the validation-across-user-base phase. Remove the flag once field-validated.

### Emission path

The inverse filter is applied to the position stream before it reaches `input_shaper_set_pos()`. For each axis independently:

```python
def pre_compensate_axis(pos_stream, h_inv):
    """Apply h_inv to the commanded position stream. pos_stream is
    a list of (t, x) samples at dt intervals."""
    # Implementation depends on minimum/maximum/mixed phase of the shaper.
    # For minimum-phase: direct IIR (causal).
    # For maximum-phase: time-reverse, apply causal IIR, time-reverse again.
    # For mixed: split into components, combine.
    ...
```

Boundary conditions at trajectory start and end:
- **Start**: pad the input stream with constant-hold at the first sample; IIR filter "warms up" over the shaper's window (~10ms). Initial samples will have transient artefacts that the planner can ignore (tool is at rest anyway).
- **End**: similar with constant-hold at the last sample.

---

## Test plan (implementation phase)

1. **Unit tests on the inverse filter itself:**
   - Identity: `h(h⁻¹(x)) = x` within 1µm on synthetic trajectories (step, ramp, sinusoid, arc)
   - Boundary handling: start/end artefacts smaller than 10µm after 1 shaper-window of warmup
   - Stability: no divergence over 10,000-sample trajectory

2. **Integration with `input_shaper`:**
   - Regression: existing input_shaper tests still green
   - End-to-end: `test_inverse_shaper_path_fidelity` feeds a G-code with a known corner; asserts post-shaper lateral deviation < 50µm (vs current ~300µm)

3. **Simulator smoke test:**
   - `klipper-sim` with `enable_inverse_compensation=true`: re-run `slice_24layers.gcode`, verify post-shaper deviation drops measurably (target: <50µm max across the full print)

4. **Hardware validation (after simulator passes):**
   - Print a test part on V0 or Trident with known sharp corners (caliper-measurable)
   - Compare dimensional accuracy with and without `enable_inverse_compensation`
   - Macro photo of corners before/after

---

## Open questions to resolve during design

1. **Which shapers are minimum-phase?** Need empirical check. Start with MZV (user's current config); handle others via the mixed-phase path if needed.
2. **Anti-causal implementation details.** For maximum-phase shapers we need to time-reverse the input, apply causal IIR, and time-reverse again. This requires the full trajectory in memory — fine for look-ahead-driven path planning (we have all moves queued) but costly in the hot path. Measure.
3. **Interaction with pressure advance.** PA is applied per-axis to the E coordinate; the inverse shaper is applied to (x, y). These should compose linearly, but verify.
4. **Per-axis saturation headroom.** After pre-compensation, peak per-axis accel may temporarily exceed `A_axis` (the shaper budget). This is not a correctness issue (by definition the post-shaper trajectory is under-budget) but could stress motor torque. Characterize and document.
5. **Failure mode if the user mis-calibrates the shaper.** If `shaper_freq` is wrong, the inverse filter pre-distorts for the wrong frequency; the shaper fails to cancel; the physical motion has amplified error. The `enable_inverse_compensation: false` default protects against this. A runtime sanity check (compare shaper_freq to resonance_tester's detected peak) would be a nice-to-have for the Kalico toolchain.

---

## Scope and exit criteria

**Estimated effort:** 3–5 weeks. Breakdown:
- 4 days: verification plan (Phase 1–3)
- 3 days: minimum-phase shaper implementation
- 3 days: mixed-phase / anti-causal handling
- 3 days: Kalico integration (`shaper_inverse.py` + `input_shaper.py` hook)
- 4 days: unit and integration tests
- 3 days: simulator end-to-end run on cube slice; tune; iterate
- 3 days: hardware test on V0 or Trident
- 2 days: documentation, config reference, commit stack

**Done when:**
- Verification plan phases 1–3 pass for MZV (minimum needed shaper for the user's config); others passing is a stretch goal
- Simulator post-shaper deviation drops to <50µm on the cube slice
- Hardware test prints show measurably better corner fidelity with the flag on vs off
- `enable_inverse_compensation: false` default preserves current behavior byte-for-byte (regression-safe)
- Documented in user-facing docs: when to enable, how to verify it's working, when to disable

---

## After this sub-spec

Remove the opt-in flag once confidence is built across multiple user configs (3–6 months of field use). The flag exception ends here; no long-term fork-as-gate violation.

If 6f was skipped (no ringing observed), the 6d+6e+6g stack is the complete ultimate-quality target. If 6f was triggered, this same inverse filter applies identically to clothoid output.
