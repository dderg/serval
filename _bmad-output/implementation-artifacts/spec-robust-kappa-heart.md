---
title: 'Robust integrated-slope kappa_signal heart + conclusive/visible heart measurement (Part 2a)'
type: 'feature'
created: '2026-06-26'
status: 'in-development'
baseline_commit: 'f0a4b534357e57b2515eda89949148d5f7c959d5'
context:
  - '{project-root}/_bmad-output/project-context.md'
  - '{project-root}/_bmad-output/brainstorming/brainstorming-session-2026-06-26-004207.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-causal-unified-fitter.md'
  - '{project-root}/_bmad-output/implementation-artifacts/deferred-work.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The cutover (Part 1) reused the proven kernels and is byte-for-byte snapshot-faithful — so it delivered architecture but **zero toolpath improvement**. The `kappa_signal` heart is also not the robust GR#4 the brainstorming intended: `leg_kappa`/`grow_kappa_span` (`heart/kappa_signal.rs:88-112`) estimate curvature as a **per-leg central difference** θ′ — the maximally noise-amplifying estimator. On sub-mm jittered/dense slicer arcs it fragments one curve into many spans (measured: `A L A L A L`, 3 mid-curve stops in `heart_comparison`), exactly where the integrated turning signal was supposed to win. And the win is currently **invisible**: the snapshot harness never plumbs `heart`, so every baseline runs `position_greedy`.

**Approach:** Make `kappa_signal` the genuine GR#4 — segment the *integrated* turning signal θ(s) by **windowed least-squares slope** (slope = κ; variance falls with window length, so jitter integrates out), keeping the existing `cocircular`/`in_band_prefix` position check as the hard correctness gate. Then make the difference both **conclusive** (extend `heart_comparison` with noisy-dense-arc + sharp-tip-cluster cases and assert KS fits strictly fewer elements / lower κ-jump than PG) and **visible** (plumb `heart` through the snapshot harness; add an `arc_fit_kappa` case group whose baselines show the better toolpath).

## Boundaries & Constraints

**Always:**
- Robust estimator only: replace per-leg central-difference κ with a windowed **least-squares slope of θ(s)** over the candidate span. The κ estimate must use every vertex in the window, never just adjacent pairs. No Menger/circumradius/3-point curvature (same ban as Part 1).
- `cocircular`/`in_band_prefix` stays the hard gate: the slope fit only *proposes* a span; a proposed span is emitted only if it still passes the existing position-band check. In-band (≤δ measured on the final curve) and G2-by-construction guarantees from Part 1 are preserved.
- `KappaSignal` must be ≥ as good as `PositionGreedy` on clean inputs (identical element count on `circle`/`straight_to_curve`/`faceted_quarter_circle`) and strictly better on the new hard cases — fewer elements, no mid-curve stops, lower peak κ-jump.
- Keep `Heart::arc_spans(chain, tol, min_run, corner) -> Vec<(usize,usize)>` — the trait, the driver, and the kernels are untouched. This spec changes only the *inside* of the `kappa_signal` heart plus tests/harness.
- Single fixed-plane projection is fine (slicer output is planar per layer); keep `chain_plane`. The 3D multi-plane lift stays deferred.
- Fail loudly: a zero/degenerate span or non-finite slope → debug_assert + the existing reject path, never a silent 0.

**Ask First:**
- Accepting the new `arc_fit_kappa` baselines (human-gated via the web UI, as in Part 1 — never auto-overwrite).
- Flipping the *default* heart to `kappa_signal` — defer this to a measurement-gated follow-up; this spec leaves `PositionGreedy` the default and proves the case.

**Never:**
- Never touch the driver/kernels/biclothoid or the other heart — scope is `kappa_signal` internals + measurement.
- Never widen `δ` or relax the position gate to make spans merge — robustness must come from the estimator, not a looser tolerance.
- No comments; unit tests in `heart/kappa_signal/tests.rs`.

## I/O & Edge-Case Matrix

| Scenario | Input | Expected (kappa_signal) | Error |
|----------|-------|------------------------|-------|
| Clean dense circle | `circle.gcode` | identical element stream to `position_greedy` (1 arc + eases) | N/A |
| Noisy dense arc | `arc_fit_kappa/noisy_arc` | one cocircular arc span (no `A L A L`), in-band ≤δ | N/A |
| Sharp-tip cluster | `arc_fit_kappa/sharp_tip` | tip clamped to one biclothoid, flanks single arcs, no fragmentation | N/A |
| Real corner mid-arc | inflection (κ sign flip) | span breaks at the sign change; no merge across inflection | N/A |
| Single short leg / <min_run | tiny chain | no span (returns `[]`), driver falls back to lines/corner blend | N/A |
| Degenerate span (zero Δs) | coincident verts | debug_assert; rejected, not emitted as κ=0 | N/A |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/fitter/heart/kappa_signal.rs` — replace `leg_kappa`/`grow_kappa_span` with `windowed_slope` (incremental LS regression of θ on s) + a slope-consistency `grow_slope_span`; `arc_spans` flow otherwise unchanged (band → grow → `in_band_prefix`).
- `rust/geometry/src/fitter/heart/kappa_signal/tests.rs` — pin: clean == PG, noisy arc → 1 span, inflection splits, degenerate rejects.
- `rust/geometry/tests/heart_comparison.rs` — add `noisy_arc` (denser/larger jitter than the existing case) + `sharp_tip_cluster`; assert `KS.elements < PG.elements` and `KS.max_kappa_jump ≤ PG.max_kappa_jump` on both; keep the printed table.
- `snapshots/harness.py` — `run_case` passes `heart=` to `pipeline_snapshot`.
- `scripts/viz_pipeline.py` — `read_printer_config` also returns the heart (via `arc_fit_heart_from_config`); `run_case` and `pipeline_snapshot` callers thread it.
- `snapshots/cases/arc_fit_kappa/{printer.cfg,noisy_arc.gcode,sharp_tip.gcode}` — NEW group; `printer.cfg` sets `heart: kappa_signal`. Baselines human-accepted.

## Tasks & Acceptance

**Execution:**
- [ ] `heart/kappa_signal.rs` — add `windowed_slope(s, theta, start, end) -> (slope, max_resid)` via incremental Σs/Σθ/Σsθ/Σss; replace `grow_kappa_span` with `grow_slope_span` that extends while the candidate vertex's residual from the running LS line stays within an angular band τ_θ (tunable; start from the existing `KAPPA_REL_TOL` relative to the fitted slope) and the slope sign is stable; delete `leg_kappa`.
- [ ] `heart/kappa_signal.rs` — keep `arc_spans` structure; the only change is the span-proposal estimator; `in_band_prefix`/`cocircular` gate unchanged.
- [ ] `heart/kappa_signal/tests.rs` — clean-input parity with PG, noisy-arc single-span, inflection split, degenerate-span reject.
- [ ] `scripts/viz_pipeline.py` + `snapshots/harness.py` — plumb `heart` from `[arc_fit]` through `read_printer_config` → `run_case` → `pipeline_snapshot`.
- [ ] `snapshots/cases/arc_fit_kappa/` — new group (`heart: kappa_signal`) with `noisy_arc.gcode` + `sharp_tip.gcode`; generate, review, human-accept baselines.
- [ ] `rust/geometry/tests/heart_comparison.rs` — add the two hard cases + the `KS` strictly-better assertions; table still prints.

**Acceptance Criteria:**
- Given `circle`/`straight_to_curve`/`faceted_quarter_circle`, when fitted with `kappa_signal`, then the element stream is identical to `position_greedy` (no regression on clean input).
- Given `noisy_arc` and `sharp_tip_cluster` in `heart_comparison`, when fitted, then `kappa_signal.elements < position_greedy.elements` and `kappa_signal.max_kappa_jump ≤ position_greedy.max_kappa_jump`, both in-band ≤δ.
- Given `snapshots/run.py`, when an `arc_fit_kappa` case runs, then it is fitted with `kappa_signal` (heart plumbed end-to-end) and its baseline shows a single arc where the `position_greedy` baseline of the same geometry fragments.
- Given `./scripts/ci.sh quick` (and `py` — klippy/harness touched), when run, then fully green.

## Design Notes

Why the slope, not the derivative: θ(s)=Σ turn angles; on a constant-κ arc θ is linear in s with slope κ. The per-leg estimator `Δθ/Δs` is a finite derivative — it amplifies the per-leg jitter ε_k by 1/Δs. The least-squares slope over an N-vertex window has variance ∝ 1/(N·baseline²): jitter integrates out, the κ estimate sharpens as the span grows. The accumulated-turn random walk (θ residual grows ~√N) is *not* the gate here — the hard correctness gate stays the position-space `cocircular` check; the slope fit only decides where to *propose* a break, so a too-generous τ_θ degrades gracefully (cocircular rejects, `in_band_prefix` backs off) rather than emitting a bad arc. Split spans at inflections (slope sign change) before merging — never merge across a κ sign flip.

Visibility vs conclusiveness are two different artifacts: `heart_comparison` asserts the win deterministically with no human gate (the conclusive measurement); the `arc_fit_kappa` snapshot group makes the better toolpath a reviewable, checked-in diff (the visible win). Both are required — the user explicitly wants to *see* a better-fitted toolpath, not just read a metrics table.

## Verification

- `cd rust && cargo nextest run -p geometry` — green (new kappa_signal tests).
- `cd rust && cargo nextest run -p geometry -E 'test(heart_comparison)'` — KS strictly better on the two hard cases; table prints.
- `./scripts/ci.sh snapshot` then `snapshots/snapshot-tests.sh` — `arc_fit_kappa` baselines reviewed + human-accepted; existing `arc_fit` baselines stay EXACT.
- `./scripts/ci.sh quick` + `./scripts/ci.sh py` — fully green.

## Suggested Review Order

1. The estimator swap — `windowed_slope` + `grow_slope_span` replacing per-leg κ. `heart/kappa_signal.rs`
2. The conclusive measurement — new hard cases + strictly-better asserts. `heart_comparison.rs`
3. The visible win — heart plumbed through the harness + the `arc_fit_kappa` group. `harness.py`, `viz_pipeline.py`, `cases/arc_fit_kappa/`
