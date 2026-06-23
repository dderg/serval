---
id: SPEC-stepped-buzz-sweep
companions: []
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only — consult them only if you need narrative rationale or prose color this contract intentionally omits.

# Stepped-frequency buzz sweep

## Why

A pain to solve. The resonance buzz sweep is generated as one smooth analytic chirp (`mu ≠ 0`). On a STEP/DIR (pulse) axis the MCU must find the exact time of every microstep crossing by root-solving `position(t) = gridline`, and a chirp has no closed-form inverse, so every crossing falls back to a numerical scan + bisection — measured at ~37× the per-emission cost of a fixed tone. That starves the MCU foreground and freezes the buzz on pulse axes (the per-crossing solve is what pushes the foreground behind; once it falls ~8 s behind, the wrapping step deadline turns "behind" into multi-second stalls). The phase (XDIRECT) path is unaffected because it picks times and evaluates the sine instead of solving. Mainline never has this problem: it does not sweep smoothly — `VibrationPulseTestGenerator` plays a staircase of **fixed-frequency** half-oscillations, stepping the frequency one notch per half-cycle, each segment a plain constant-frequency move. Adopt the same shape here: build the sweep as a loop of single-frequency buzz tones. Every tone is `mu = 0`, so it stays on the cheap closed-form path on *both* phase and pulse axes — no scan, no freeze — and reuses the single-frequency buzz we already have.

**Resolved shape (loop location + continuity + scope).** The loop lives on the **MCU** as a **separate, pulse-only generator** — not the fixed-tone buzz with a frequency-stepping flag bolted on (that conditional is the awkward entanglement we are avoiding). It exists to keep the STEP/DIR path off the per-crossing scan, which is the only place that freezes. The **phase (XDIRECT) path is left exactly as it is**: it never root-solves `position = gridline` (it inverts the carrier phase analytically and evaluates the sine), so the continuous chirp is already cheap there and stays. The routing forks deliberately by axis mode: **phase → existing continuous chirp; pulse → new staircase**.

Within the pulse staircase: one command, the generator holds the carrier frequency constant and steps it at carrier **turning points** (velocity zero, `phi = pi/2 + k*pi`), exactly where mainline steps it. Stepping only at a turning point keeps the sweep seamless — the carrier phase stays continuous, velocity is ~zero across the step. A single trapezoidal envelope spans the whole sweep: frequency is held at `freq_start` through ramp-in and `freq_end` through ramp-out (both constant-frequency, handled by the existing closed-form tone solver), and steps across the flat top. Frequency increment per segment is `hz_per_sec * segment_seconds`, matching mainline's per-half-period staircase resolution. The wire command is unchanged (`freq_start`, `freq_end`, `amplitude`, `duration`, `ramp`); the pulse generator derives the staircase rate from the same `freq_start`/`freq_end`/`duration` that previously set `mu`, and solves every crossing at constant per-segment frequency.

## Capabilities

- id: CAP-1
  intent: A resonance sweep across a frequency band is produced as an ordered sequence of single-frequency buzz tones stepping from `freq_start` to `freq_end`.
  success: Every emitted segment is one constant frequency (`mu = 0`); the band `[freq_start, freq_end]` is covered end to end; no segment uses the chirp/exact-crossing scan path.

- id: CAP-2
  intent: The sweep runs on a STEP/DIR (pulse) axis without the per-crossing solver starving the foreground.
  success: A full sweep on a pulse axis at 32 microsteps completes with no freeze and no `ring_overflow` / `fg_freeze` / stall forensics; per-emission cost is the closed-form tone cost, not the scan cost.

- id: CAP-3
  intent: One sweep command routes each axis to the generator for its step mode — phase axes to the existing XDIRECT chirp, pulse axes to the new staircase — through the existing per-axis mode routing.
  success: A sweep on a phase axis is driven by the unchanged XDIRECT chirp and a sweep on a pulse axis by the new staircase; the computed input shaper is sane on both axis types.

- id: CAP-4
  intent: The pulse staircase is a separate generator, not a flag on the fixed-tone path and not a change to the XDIRECT path; it reuses the existing constant-frequency crossing solver for each segment and the ramp ends.
  success: The XDIRECT/phase generator and the fixed-tone (`freq_start == freq_end`) pulse path are untouched; the new code is the segment-sequencing generator that steps frequency at turning points and solves each segment at constant frequency (`mu = 0`).

- id: CAP-5
  intent: The frequency-step granularity is fine enough that the staircase resolves resonances for input-shaper calibration.
  success: The step size (frequency increment per segment) is parameterized; the default yields frequency resolution comparable to mainline's `VibrationPulseTestGenerator`, and a calibration run on a known machine returns the expected shaper.

## Constraints

- The pulse staircase is a separate generator. It does not modify the fixed-tone (`freq_start == freq_end`) pulse path and does not touch the XDIRECT/phase path.
- Each pulse-sweep segment is a single fixed frequency (`mu = 0`); the smooth analytic chirp is never used on the pulse path.
- Frequency steps only at carrier turning points (`phi = pi/2 + k*pi`, velocity zero), so the carrier phase is continuous and no segment boundary injects a position/velocity discontinuity beyond sub-microstep rounding.
- One trapezoidal envelope spans the whole sweep (a single ramp-in and ramp-out), not a per-segment fade. Frequency is held constant through each ramp end (so the existing constant-frequency solver covers them) and steps only across the flat top.
- The loop runs on the MCU: one `submit_resonance_buzz` command produces the whole staircase; the wire arguments are unchanged.
- Route by step mode through the existing per-axis routing: `StepMode::Phase` → unchanged XDIRECT chirp; `StepMode::Pulse` (sweep) → new staircase. A deliberate fork by axis type.
- Follow mainline's staircase shape: segment length ≈ a small integer number of half-periods at the segment frequency (default one half-period); frequency increment = `hz_per_sec * segment_seconds`.

## Non-goals

- Not changing the accelerometer capture or the input-shaper calibration math.
- Not changing the single-frequency (fixed-tone) buzz behaviour: a `freq_start == freq_end` command still plays exactly the constant tone it does today.
- Not changing the phase (XDIRECT) sweep: it keeps the continuous analytic chirp, which is already cheap there (no root-solve) and works. The staircase is pulse-only.
- Not removing the chirp (`mu`) machinery: the XDIRECT path still uses it. Only the pulse path stops constructing `mu != 0` curves.

## Success signal

On the Trident bench, a `RESONANCE_BUZZ_SWEEP` on a regular-stepping (pulse) axis at 32 microsteps runs the entire band as continuous buzz with no freeze and no foreground-starvation forensics — where today it stalls for 20–30 s at a time — and the same command on a phase axis is equally clean. The moment that confirms it: dderg runs the non-phase sweep, it plays straight through, and the computed shaper matches the phase-axis result.

## Assumptions

- A fine-enough frequency staircase is acceptable for shaper calibration; mainline's long-standing use of exactly this construction is the precedent.
- On the pulse path every segment runs the `mu = 0` closed-form crossing solver, so the pulse sweep never touches the exact-crossing scan (except the existing constant-frequency ramp handling at the two ends). The phase path keeps its chirp, which is cheap on XDIRECT by construction.

## Resolved decisions (2026-06-23, dderg)

- **Loop location → MCU.** One command; the generator steps frequency internally at segment boundaries. No host loop, no per-segment command latency.
- **Continuity → seamless via turning-point stepping.** Frequency (and the matching amplitude taper) changes only at a carrier turning point, where velocity is ~zero, so the step costs no movement artefact. One envelope spans the whole sweep. No gaps, no per-segment fade.
- **Per-segment duration → one half-period (mainline's resolution).** Step at every turning point; increment `hz_per_sec * segment_seconds`. A coarser "small integer number of half-periods" is permitted if it simplifies the generator without measurably coarsening the staircase.
- **Scope → pulse-only, separate generator.** Looping the fixed-tone buzz adds an awkward conditional; the sweep is its own function instead. Because it is separate, the working phase (XDIRECT) chirp is left untouched — the routing forks by axis mode (phase → chirp, pulse → staircase). The chirp machinery stays (XDIRECT uses it); only the pulse path stops building `mu != 0` curves.
