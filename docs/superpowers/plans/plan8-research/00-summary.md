# Plan 8 — Phase 0 Research Summary

**Date:** 2026-04-23
**Spec:** `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md`

## 6.1 FIR piecewise evaluator performance

**Verdict:** safe.

**Key decisions:**

- Piecewise evaluator is ~4.8× cheaper per step than today's `shaper_calc_position` for MZV; ~6.3× for a 4-impulse shaper.
- Existing bisection fallback in `itersolve_gen_steps_range` handles polynomial reversals at sharp corners correctly; no solver change needed.
- `check_oscillate` firing frequency estimate: ~1% of all steps on corner-heavy prints, ~1.5% added iterations globally — well inside solver budget.
- Mitigation on standby if real prints exceed threshold: composer-level fallback emits unshaped quintic for reversal-producing corners.
- **Scope correction surfaced:** this fork only retains `zv` and `mzv` as FIR shapers (`shaper_defs.py:281-285`). EI / EI3 / ZVD are not configurable. Spec will be updated below.

**Blocks implementation detail:** none — ready.

## 6.2 Non-linear PA Chebyshev piecewise fit

**Verdict:** degree-4 Chebyshev, adaptive 1–5 pieces per trapq phase, hard cap 8 pieces.

**Key decisions:**

- Acceptance threshold: reject if `max|ε_cheb| · nonlinear_offset > 1 µm` filament.
- Typical short-segment moves need only a single piece (error ~56 nm filament at NO=0.08 with degree-3; degree-4 gives ~10 nm).
- Full-accel ramps need adaptive splitting at `x=1.0` (tanh knee) and `x=2.5` (saturation) — 5 pieces worst case.
- Retractions: PA branch skipped entirely (zero fit error). z-hops: 0.4 nm filament error.
- Edge wart: raw Chebyshev doesn't pin `c(0)=0`; mitigate in composer via `fit(f(x) − f(0))`.

**Blocks implementation detail:** none — Chunk 3 design can proceed with these defaults.

## 6.3 Per-axis frequency polynomial layout

**Verdict:** Candidate A — unified finer time partition with padded wider-kernel axis via Pascal shift.

**Key decisions:**

- Replace fixed `{accel, cruise, decel}` trio with variable-length `phases[N_MAX]` carrying the union of per-axis breakpoints.
- Pascal shift formula for padded sub-phases: `b_i = Σ_{j=i..10} a_j · C(j,i) · Δ^(j-i)`. O(121) mul-adds per axis per sub-phase, Python-side at emit time.
- Worst-case phase count ~28 at 50 Hz/150 Hz axis mismatch with bs on both axes.
- Safe `N_MAX = 32` → 8 KB per move.
- Common case (same kernel on both axes): 4 phases.
- Phase-pick: linear scan at N≤4, binary search above.
- Layout `struct coord c[11]` xyz-interleaved preserved — no cache disruption.

**Blocks implementation detail:** Chunk 1 struct layout decision; informs `trapq.h` edits in Chunk 1.

## 6.4 Lookahead commit window

**Verdict:** current `LOOKAHEAD_FLUSH_TIME = 250 ms` is adequate. No change required.

**Key decisions:**

- Worst-case kernel support `S = 109 ms` (bs4 @ 23 Hz, bs5 @ 25 Hz — both at `min_freq`).
- Design-point S ≈ 45 ms (bs5 @ 60 Hz). MZV @ 60 Hz: 12.6 ms.
- `LOOKAHEAD_FLUSH_TIME` is a lazy-flush lower bound, not a ceiling; `BUFFER_TIME_HIGH = 2.0 s` means real commit horizon is seconds.
- Late-arrival / quiescent periods (M400, G4, idle): handled correctly — `_flush_lookahead` drains queue; boundary moves have zero-padded kernels (matches today's post-hoc shaper behavior).
- Homing / probing bypass via `drip_move` immediate flush (`toolhead.py:749-775`), NOT via `shape_disabled`. 250 ms threshold irrelevant on drip path.
- Revisit threshold: push to 400–500 ms only if a future shaper has `F_m > 4.6` or `min_freq < 20 Hz`.
- Flagged for Phase 1 measurement: corpus `min_move_t` p95/p99 not yet measured on real gcode.

**Blocks implementation detail:** none — no code change needed.

## 6.5 `shape_disabled` flag threading

**Verdict:** clean threading — add `int shape_disabled` parameter to `trapq_append` and `trapq_append_quintic`; three hardcoded `true` call-site edits plus one toolhead-level `_drip_mode` flag.

**Key decisions:**

- **Must-be-unshaped (3):** `force_move.py:103`, `manual_stepper.py:78`, `toolhead.py:482/496` stamped via `drip_move` entry.
- **Must-be-shaped (2):** `toolhead.py:482` (quintic) and `:496` (linear) for normal print path.
- **Conditional (1):** `extruder.py:772` — inherit parent XY move's flag; force `true` on pure-E moves (`axes_d[0]==0 && axes_d[1]==0`) since no XY kernel to cascade.
- Homing / probing: all flow through `HomingMove.homing_move → toolhead.drip_move`. Single stamp at drip_move entry covers every probe/homing path (verified across `probe*.py`, `homing.py`, `dockable_probe.py`, `eddy_current`, `load_cell`).
- `set_position` boundary: mark history-marker move with `shape_disabled=true` so FIR piecewise evaluator doesn't look back across it. Fallback: pad with kernel_support-worth of unshaped moves after `set_position`.
- `idex_modes.py:80` uses `set_position` only, no direct emit — covered.

**Blocks implementation detail:** none — Chunk 2 can proceed with this audit.

---

## Cross-cutting findings

1. **Fork scope is narrower than spec originally stated.** This Kalico fork retains only `zv` and `mzv` from the FIR family (`shaper_defs.py:281-285`) and has retired the smooth-IS family entirely in favor of bs (`shaper_defs.py:305-311` `RETIRED_SMOOTHER_MIGRATION`). The Plan 8 spec was written as if EI/EI3/ZVD and smooth-IS were live; they are not. **Spec corrected below.**

2. **Homing bypass doesn't need `shape_disabled` thanks to drip_move flush.** The `shape_disabled` flag is still required for `force_move` and `manual_stepper`, but the more invasive "route homing around the planner" concern from the spec's §3.6 list is partially absorbed by existing drip-mode plumbing.

3. **Pascal shift is the load-bearing math for per-axis kernel mismatch.** If a future refactor touches `trapq.c` polynomial machinery, the Pascal-shift inflation (121 flops per padded sub-phase) is the one non-obvious cost worth watching.

4. **Plan 8 scope remains intact.** No research gap uncovered an architectural blocker. All five gaps resolve to "design choice confirmed" or "numeric threshold set."

---

## Ready-to-implement status

- **Chunk 1 (Plan 6 fold):** ready. Informed by §6.3 (variable-length phases, N_MAX=32).
- **Chunk 2 (Bake XY shaper):** ready. Informed by §6.1 (piecewise evaluator), §6.5 (`shape_disabled` threading). FIR family narrowed to zv/mzv only.
- **Chunk 3 (Bake E + PA):** ready. Informed by §6.2 (Chebyshev defaults). Smooth-IS family retired; scope is FIR (zv/mzv) + bs (bs1–5).

## Next step

Write Chunk 1 implementation plan at `docs/superpowers/plans/2026-04-YY-plan8-chunk1-plan6-fold.md` informed by this summary.
