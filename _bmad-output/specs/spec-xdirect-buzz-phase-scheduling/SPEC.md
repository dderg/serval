---
id: SPEC-xdirect-buzz-phase-scheduling
companions:
  - scheduling-algorithm.md
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only — consult them only if you need narrative rationale or prose color this contract intentionally omits.

# XDIRECT buzz: phase-locked update scheduling

## Why

A pain to solve. The phase-stepping (XDIRECT) input-shaper buzz screeches and pumps — audible aliasing plus a loud→quiet→loud sweep — so input-shaper calibration is unusable on phase-stepping machines (the Trident bench drives 4 phase motors). The XDIRECT path exists because the STEP/DIR exact-crossing buzz overruns step delivery at 256 microstepping; XDIRECT writes absolute coil positions and is late-tolerant and microstepping-independent. But the current generator schedules those writes on a **uniform displacement grid** (one update every `grid_steps` LUT microsteps of travel). Uniform in space is non-uniform in time: dense at the zero crossings, sparse at the peaks. The motor is a zero-order hold, so its sound is the pattern of snaps — and non-uniform snap timing folds the ZOH spectral images down into the audible band (the "aliasing") and modulates snap density at 2f within every cycle (the "loud→quiet→loud"). The fix is the originally-agreed scheme that was never implemented: schedule each update on the **carrier phase**, so the update rate is `N·f_inst(t)` and sweeps lock-step with the buzz frequency, uniform in time within each cycle.

The bar is not "good enough to hear as a tone" but "clean enough to measure." The accelerometer reads the machine's response to this commanded motion, so any sampling artefact in the commanded sinewave becomes a spurious feature in the resonance estimate and corrupts the input-shaper result. The commanded motion must therefore be the cleanest sine the update budget allows: an **exact** phase-locked rate (the buzz period always divides evenly into the update grid), extrema and zero crossings always on the grid, and no change in update spacing anywhere the carrier is moving.

## Capabilities

- id: CAP-1
  intent: A phase-stepping (XDIRECT) axis excited by RESONANCE_TESTER produces a clean excitation tone usable for input-shaper calibration.
  success: By ear on the bench at 256 microstepping with phase stepping enabled, a sweep is a smooth tone — no screech, no aliasing hash, no loud→quiet→loud pumping. Offline, the spectrum of the commanded coil-position waveform places its ZOH images only at `N·f` harmonics (out of band), not smeared into the audible band.

- id: CAP-2
  intent: The realized update rate is an exact integer multiple of the instantaneous buzz frequency (`N·f_inst`), uniform in phase — never a fixed rate that the carrier period fails to divide.
  success: Each carrier half-cycle contains exactly `N/2` updates at uniform `Δφ = 2π/N`; the realized rate equals `N·f_inst(t)` with zero remainder (the period always divides evenly into the grid). `target_rate` only bounds `N` from above; it never sets the realized rate. Within a segment, emitted timestamps are evenly spaced in time (max/min gap ratio = 1.0 at constant frequency).

- id: CAP-3
  intent: Carrier extrema and zero crossings are sample points for every value of `N`, including across an `N` change, so the commanded peak equals the requested amplitude exactly with no extremum-refinement code path.
  success: Every quarter-phase `φ = n·π/2` is a grid point (`N` divisible by 4, grid anchored at a turning point), throughout the sweep and across any `N` change. The emitted offset at each carrier extremum equals `round(commanded_peak / lut_step)` exactly. `next_update` contains no separate `refine_extremum` branch.

- id: CAP-4
  intent: The per-cycle update count adapts to the (tapering) amplitude so the grid is never finer than LUT resolution — redundant identical writes stay confined to the immediate neighbourhood of an extremum and are bounded.
  success: `N = max(4, min(round(target_rate / f_inst), 4·round(A_eff / lut_step)))`, rounded up to a multiple of 4. The per-update offset change is ≤ ~2 LUT steps everywhere (no large jumps at the zero crossings); consecutive equal offsets occur only within a few updates of an extremum, where the carrier is near-stationary, never across a moving section. Every grid point is emitted (uniform spacing per CAP-2); no-op writes are not skipped, since the held waveform — and thus the spectrum — is identical either way.

- id: CAP-7
  intent: When `N` changes during a sweep it changes only at a carrier turning point — where the motor is momentarily stationary — so update spacing never changes while the carrier is moving.
  success: Across a chirp, every change in `N` coincides with a velocity-zero (`φ = π/2 + nπ`); no pair of updates straddling a moving section of the carrier carries a different `Δφ`. The spacing discontinuity, when it occurs, sits exactly at a point where commanded velocity is zero.

- id: CAP-5
  intent: The emitted update stream depends only on the tone curve, the anchor, and the cycle-counter rate — never on microstep size or the motion ISR tick rate.
  success: Changing the microstep setting or `CONFIG_MOTION_SAMPLE_RATE_HZ` leaves the emitted `(t, offset_steps)` sequence unchanged. Each `cycle_abs` is derived solely via `cycle_at(t)`.

- id: CAP-6
  intent: The buzz returns the axis exactly to its parked base when the window closes (net-zero excitation).
  success: The final emitted update lands at `total_seconds` with `offset_steps == 0`; the axis parks on base with no residual offset.

## Constraints

- Update instants are scheduled on **carrier phase** (Δφ = 2π/N per update), never on displacement and never on a fixed wall-clock Δt. This is the load-bearing decision; it rules out the current displacement grid and any fixed-rate scheme not locked to the buzz frequency.
- The realized update rate is exactly `N·f_inst` (integer updates per cycle); the carrier period must always divide evenly into the update grid. `target_rate` is only an upper bound on `N`, never the realized rate. "Closest to a target rate" is a defect — the rate must be exact relative to the buzz frequency.
- `N` must be divisible by 4 and the phase grid anchored at a carrier turning point, so every quarter-phase (both extrema and both zero crossings) is a grid point — and stays one across any `N` change.
- `N` changes only at carrier turning points (velocity zero). Spacing must never change while the carrier is moving, because the goal is zero movement artefact in the commanded sinewave.
- Reuse the existing unified delivery (StepQueue → refill → TIM3) and the `cycle_at(t)` timing mapping; routing stays keyed on the axis `StepMode`. No second delivery path, no clone of the scheduler.
- Tie nothing to `CONFIG_MOTION_SAMPLE_RATE_HZ` or the ISR tick rate (explicit standing instruction).
- Fail loud: strict-monotonic time, bounded iteration, no silent clamping or padding (project rule).
- Comments are a failure of expression; unit tests live in a file separate from the code under test (project rules).
- Change is contained to `rust/runtime/src/buzz_xdirect.rs` (+ its `tests.rs`) and the single `XdirectConfig::for_rate` call site in `engine.rs`. `XdirectConfig` carries `n_per_cycle`, not `grid_steps`.

## Non-goals

- Not touching the STEP/DIR (Pulse-mode) buzz path — it works correctly at ≤32 microstepping and is selected by axis mode.
- Not moving any buzz motion generation to the host. The host orchestrates; the MCU generates and executes motion. (Explicitly against the architecture.)
- Not reintroducing host-driven exit/re-enter of phase stepping around the buzz; the MCU routes by axis mode.
- Not fixing the underlying late-step delivery starvation at shared `MOTION_NVIC_PRIO`; XDIRECT's absolute writes sidestep it and that is a separate concern.
- Not changing `accel_per_hz` semantics or the host-side `resonance_tester` sweep schedule.

## Success signal

On the Trident bench, phase stepping enabled at 256 microstepping, a RESONANCE_TESTER frequency sweep is audibly a clean, smooth tone the whole way through — no screech at onset, no aliasing buzz, no resolution-switching pump — and yields usable input-shaper parameters. The moment that confirms it: dderg runs the sweep, hears one continuous clean tone instead of the current screech-then-pump, and the computed shaper looks sane. The deeper bar: the resonance estimate computed from the accelerometer response shows no spurious peaks attributable to the excitation sampling — the only features are the machine's true resonances.

## Assumptions

- The "commanded peak" pinned at φ = π/2 + nπ (CAP-3) is the enveloped-sine value `env·A_eff` at that phase, i.e. the cycle's commanded carrier peak — not the true displacement extremum, which the envelope slope shifts slightly off φ = π/2 during the ramps. Commanding position (not chasing the velocity zero) makes that distinction immaterial.
- The forced net-zero close (CAP-6) is a single update appended at `total_seconds`; it lands during `env → 0`, so it is off the phase grid by design and carries no artefact cost.
