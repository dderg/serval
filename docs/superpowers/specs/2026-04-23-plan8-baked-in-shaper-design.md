# Plan 8 — Bake input shaping and pressure advance into the planner

**Date:** 2026-04-23
**Branch:** `magnum-opus`
**Status:** DESIGN. Implementation plan to be produced via `writing-plans` skill after this spec is approved.
**Supersedes:** Plan 6 draft (`2026-04-22-plan6-spline-native-draft.md` — folded in), Plan 7 draft (`2026-04-23-plan7-configurable-k-design.md` — shelved).

## 1. Goal

Move input shaping AND pressure advance from post-hoc step-gen stages into the planner. The planner emits per-axis polynomial moves whose coefficients already encode the target resonance-kernel shape and PA compensation. Post-hoc convolution stages retire.

**End state, one line:** `kin_shaper.c` is deleted. Most of `kin_extruder.c` is deleted. Every move — XY, E — is a polynomial the planner computed with resonance rejection and PA already inside it.

## 2. Scope

**In scope:**

- Fold Plan 6 (retire `MOVE_LINEAR`, every move is a quintic polynomial).
- Bake the input shaper into the planner for all three kernel families — FIR (MZV / ZV / EI / EI3 / ZVD), smooth-IS (smooth_zv / smooth_mzv / smooth_ei), and bs (bs1 … bs5).
- Bake pressure advance into the planner — planner emits the E polynomial with PA applied. Linear PA is exact; non-linear PA models (tanh, recipr) fit as piecewise polynomial.
- Retire Plan 5 Pillar 1 feedforward inverse (no shaper left to invert).
- Retire `target_smoothing` knob.

**Out of scope:**

- Extruder hardware limits (`max_extruder_accel`, `max_extruder_rpm`) — remain in Plan 3 / Pillar 3 as separate work.
- New shaper families or resonance models.
- Phase stepping (Duet RRF 3.6 style) — orthogonal.
- Wire-format / motion-report schema bumps beyond what Plan 5 already docketed.

## 3. Architecture

### 3.1 Today's pipeline

1. gcode → lookahead emits trapezoidal moves (`MOVE_LINEAR`) or Plan 5 quintic blend at corners.
2. Step generator consumes each move, evaluates position, convolves with the input shaper kernel (`kin_shaper.c`), emits steps.
3. Extruder step generator runs a parallel pipeline: evaluate XY velocity → convolve with PA kernel + the fused shaper kernel (cascade identity wiring at `input_shaper.py:602`) → emit extruder steps.

### 3.2 New pipeline

1. gcode → planner (lookahead + polynomial composer).
2. Planner emits per-move polynomial payloads. Each move contains:
   - XY polynomial (per axis) whose coefficients already encode the kernel shape at the current `f_sh`.
   - E polynomial pre-composed with PA.
3. Step generator reads the move, evaluates the polynomial at time `t`, emits a step. No convolution, no shaper dispatch, no PA kernel.

### 3.3 What "baked in" means per kernel family

- **Smooth-IS (smooth_zv, smooth_mzv, smooth_ei):** kernel is a continuous piecewise polynomial. Planner composes `kernel ⊛ motion_polynomial` analytically, producing one polynomial per phase. Step-gen: single polynomial eval per move. This is the genuinely trivial case.
- **bs (bs1 … bs5):** cardinal B-spline kernels. Same treatment as smooth-IS — analytical composition, one polynomial per phase.
- **FIR (MZV / ZV / EI / EI3 / ZVD):** impulse-train kernels of N impulses (N = 2 … 4). Planner emits a **piecewise polynomial** per move, with breakpoints at the impulse delay offsets. Step-gen: select the right piece for the current `t`, evaluate. Mathematically identical to the current `shaper_calc_position` sum-over-N-delayed-positions, but the summing is done once at planner-emit time rather than at every step.

### 3.4 What step-gen gets

- For smooth-IS and bs: single polynomial evaluator per move. Genuinely trivial.
- For FIR: piecewise polynomial evaluator (N pieces, select-the-right-one by t). Simpler than the current `shaper_calc_position` but not zero-cost.

In both cases the step generator loses all shaper-dispatch and PA-convolution code. The `kin_shaper.c` dispatch tree (`shaper_calc_position` vs `smoother_calc_position`) disappears entirely.

### 3.5 Extruder / PA data flow

- XY velocity polynomial is computed once per move.
- E polynomial shares that source velocity (cascade identity guaranteed by construction — no possible numerical divergence).
- Linear PA: E position derivative `dE/dt = k · dV_xy/dt + base_flow` composes exactly with the velocity polynomial.
- tanh / recipr PA: non-linear function of velocity. Fit as a Chebyshev piecewise polynomial per move; piece count set by research gap §6.2.

### 3.6 Homing / probing path

Planner's baking always applies — it's embedded in the polynomial composer. Homing and probing require bit-exact unshaped motion. Solution: a `shape_disabled` flag on `struct move`. The polynomial composer skips baking when this flag is set, emitting a degenerate linear-equivalent polynomial.

Bypass code paths that must set `shape_disabled`:

- `toolhead.drip_move` (homing feed)
- `extras/force_move.py` direct trapq emit
- `extras/manual_stepper.py`
- IDEX / dual-carriage handoff moves
- Any extruder-only move that shouldn't inherit XY shaping (TBD — see §6.5)

## 4. User-facing config

**Unchanged:**

- `[input_shaper]` block name (legacy-but-stable).
- `shaper_type_x / y = mzv | zv | ei | ei3 | zvd | smooth_zv | smooth_mzv | smooth_ei | bs1 | bs2 | bs3 | bs4 | bs5`. Same names; controls planner motion primitive family under the hood.
- `shaper_freq_x / y`, `damping_ratio_x / y`.
- `pressure_advance`, `pressure_advance_smooth_time`, `pressure_advance_model` — unchanged.

**Retired:**

- `target_smoothing` — no post-hoc smoothing to bound. Hard removal (config error, not silent ignore).

**Behavior preserved:**

- `shaper_type = ""` still works; means "planner emits unshaped quintic" (Plan 5 baseline behavior).
- `SHAPER_CALIBRATE` flow identical from user perspective.

## 5. Code retirement list

**Fully retired:**

- `klippy/chelper/kin_shaper.c` (~350 lines) + `kin_shaper.h`.
- `klippy/extras/extruder_smoother.py` (logic migrates to the planner-side polynomial composer).
- `klippy/chelper/bspline_inverse.py` (no shaper to invert).
- `input_shaper.py:463-607` fused-kernel wire-up.
- `shaper_calibrate.py` target-smoothing machinery (`:446-452`, `:604-620`).
- Plan 5 Pillar 1 feedforward inverse entirely.

**Mostly retired (thin stub remains):**

- `klippy/chelper/kin_extruder.c` — PA convolution loop and cascade wiring delete. The extruder stepper-generator stays (polynomial → steps).

**New code:**

- **Polynomial composer** (Python, `klippy/extras/blendmath.py` extension): `compose(move, kernel, pa_model) → (xy_poly, e_poly)`. Runs at planner-emit time.
- **Piecewise-polynomial evaluator** (C, step-gen helper): handles FIR baked moves with N breakpoints at impulse delays.
- **`shape_disabled` flag** on `struct move` + threading through Python bypass paths.
- **Extended lookahead commit window**: `LOOKAHEAD_FLUSH_TIME` covers `max(kernel_support) × max_moves_per_sec`. Today 250 ms; at 120 Hz f_sh the kernel support is ~8 ms, at 50 Hz it is ~50 ms. Both inside budget.

## 6. Research gaps (resolve before writing the implementation plan)

Each gap gets a subagent research pass. Findings feed into the implementation plan.

### 6.1 FIR piecewise evaluator performance

**Question:** does the per-step select-piece-then-evaluate cost stay within itersolve's secant-solver budget when FIR shaping is baked in and sharp-corner moves produce brief polynomial reversals?

**What to measure:** on the high-throughput sections of the regression corpus (Cowling short-segment gcode at aggressive accel), how many secant iterations per step occur for FIR-baked moves vs current shaper dispatch? Where does `check_oscillate` fire?

**Deliverable:** performance report with numbers; if bad, propose mitigation (e.g., restrict FIR baking to non-declined corners).

### 6.2 Non-linear PA as piecewise polynomial

**Question:** what's the worst-case error for Chebyshev-fit tanh / recipr PA across the full supported flow range, not just mid-range?

**What to measure:** max absolute error in µm of filament position between exact `tanh(v/v_lin)` and 2/3/5-piece Chebyshev fits, evaluated at representative flow profiles including ramp-up, steady-state, retract, and hop transitions.

**Deliverable:** error-vs-pieces curve; recommendation for default piece count; acceptance criterion for when to reject the fit.

### 6.3 Per-axis frequency handling

**Question:** how do we represent one move's polynomial when X and Y have different `f_sh` → different kernel widths → different natural phase boundaries?

**Candidates:**

- Pick the finer kernel's time partition, pad the other axis to match. Polynomial struct unchanged.
- Split into per-axis move structs. Invasive; breaks `move_quintic_phase`'s shared `t_end`.

**Deliverable:** decision with cost estimate; if padding, characterization of the polynomial-complexity inflation at worst-case axis mismatch (e.g., `shaper_freq_x = 50 Hz`, `shaper_freq_y = 120 Hz`).

### 6.4 Lookahead commit window

**Question:** exact minimum extension to `LOOKAHEAD_FLUSH_TIME` / `BUFFER_TIME_*` required to guarantee move N's polynomial has all kernel-support-worth of neighbors committed before emit.

**What to measure:** worst-case kernel support across all supported shapers and frequency ranges; min_move_t distribution on the regression corpus; safety margin against late-arrival gcode (e.g., M400 in the middle of a stream).

**Deliverable:** numeric bound, code change to `toolhead.py:134` if needed.

### 6.5 `shape_disabled` flag threading

**Question:** audit every code path that emits to the trapq. Which should set `shape_disabled = true`?

**What to check:**

- `drip_move`, `force_move`, `manual_stepper` — definitely yes.
- IDEX / dual-carriage handoff — yes.
- Extruder-only moves (no XY motion) — does shaping still make sense? Probably no, but confirm.
- `set_position` boundary interaction — kernel support from before a set_position must not bleed into after.

**Deliverable:** checklist of all emit sites with recommended `shape_disabled` value; test plan covering each.

## 7. Implementation chunks

Three internal chunks, one branch, no HW gates between them. Each chunk ships when sim regression corpus + unit tests pass.

### Chunk 1 — Plan 6 fold

Retire `MOVE_LINEAR`. Every move in the trapq is a quintic polynomial. Post-hoc shaper and PA still run unchanged. This is the foundation: the `struct move` layout is unified to quintic.

**Exit criteria:** sim regression corpus prints byte-identical steps to current Kalico behavior. Unit tests cover degenerate-quintic-from-linear equivalence.

### Chunk 2 — Bake XY shaper

Polynomial composer implements the XY-side baking for all three kernel families. Piecewise-polynomial evaluator added to step-gen. `kin_shaper.c` retires. `shape_disabled` flag threaded through bypass paths.

**Exit criteria:** sim regression corpus produces trajectories matching post-hoc-shaper output within numerical tolerance (epsilon TBD from subagent research). HW spot-check encouraged but not required.

### Chunk 3 — Bake E shaping + PA

E polynomial composer implements PA baking (linear + non-linear). Cascade identity wired through shared source velocity polynomial. `kin_extruder.c` convolution code retires.

**Exit criteria:** sim regression corpus E-motion matches post-hoc PA output within tolerance. Printed output at acceptance prints compares against reference.

## 8. Risks

1. **Regressions vs current shaper behavior.** Mitigation: sim regression corpus (Voron Cube, speedbench, Cowling) runs before each chunk ships.
2. **FIR reversal performance at sharp corners.** Mitigation: benchmark in Chunk 2; if bad, restrict FIR baking to non-declined corners or add a fallback path.
3. **Homing correctness with `shape_disabled` flag.** Mitigation: dedicated test covering every bypass path.

## 9. Testing strategy

- **Sim regression corpus** (primary gate): Voron Cube, speedbench, Cowling gcode through klipper-sim. Trajectory comparison pre/post per chunk.
- **Unit tests:** polynomial composer math, Chebyshev PA fit bounds, piecewise evaluator correctness, `shape_disabled` flag handling.
- **HW validation:** user-driven, not gated. Tests on Trident with bs and smooth-IS families whenever convenient.

## 10. Relationship to prior plans

| Plan | Status under Plan 8 |
|------|---------------------|
| Plan 1 (quintic revival) | preserved foundation |
| Plan 3 (extruder-first-class, Pillar 3) | preserved; extruder limits untouched |
| Plan 5 (direct-quintic + Pillar 1) | partially retired — feedforward inverse deletes; direct-quintic step-gen evolves |
| Plan 6 (spline-native unification draft) | folded into Plan 8 Chunk 1 |
| Plan 7 (configurable K) | shelved — baked-in planner makes the K-cap obsolete |

## 11. Not in scope / deferred

- Plan 6 research gap §7 (shared `calc_position` helper across kinematics) — orthogonal refactor, later work.
- Ethercat / real-time stepper path — separate future work; the baked-in polynomial format is a good input for it.
- Alternative resonance models (multiple notches, damping-varying kernels) — single-notch per axis for this plan.

---

**Next step:** user reviews this spec. On approval, invoke `writing-plans` skill to produce the implementation plan.
