# Plan 8 Phase 0 Task 5 — `shape_disabled` flag audit

**Date:** 2026-04-23
**Branch:** `magnum-opus`
**Spec reference:** `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md` §3.6, §6.5
**Scope:** enumerate every code path that emits to a trapq (direct or via lookahead) and classify whether the emitted `struct move` must carry `shape_disabled = true`.

## 1. Call-site enumeration

Every emission to a trapq goes through one of three C entry points:

- `trapq_append` — linear trapezoid, 3 sub-moves (accel / cruise / decel).
  (`klippy/chelper/trapq.c:196-243`)
- `trapq_append_quintic` — single quintic poly move, 3 phases packed.
  (`klippy/chelper/trapq.c:252-286`)
- `trapq_set_position` — writes a zero-length history marker only; not a
  motion-emitting path but relevant to kernel-support bleed-through.
  (`klippy/chelper/trapq.c:336-362`)

Every Python caller of `trapq_append` / `trapq_append_quintic`:

| # | Site (file:line)                              | What kind of move                              | Classifier         | `shape_disabled` | Reason |
|---|-----------------------------------------------|-----------------------------------------------|--------------------|------------------|--------|
| 1 | `klippy/toolhead.py:482` (`_process_moves`, quintic branch) | Planner-emitted quintic blend (`QuinticBlendMove`) from `CornerBlender` | Print/travel | `false` (default) | Normal lookahead output; the whole point of Plan 8. |
| 2 | `klippy/toolhead.py:496` (`_process_moves`, linear branch)  | Planner-emitted straight edge, pre/post blend or pure travel | Print/travel | `false` (default) | Same lookahead queue; shaping still wanted. |
| 3 | `klippy/kinematics/extruder.py:772` (`PrinterExtruder.move`) | Per-move extruder-side trapq_append driven by toolhead's `_process_moves` | Print (conditional for pure-E) | Inherit from XY parent move (see §2.3) | Coupled to the XY move time; PA baking presumes XY shape known. |
| 4 | `klippy/extras/force_move.py:103` (`ForceMove.manual_move`) | `FORCE_MOVE` / `STEPPER_BUZZ` diagnostic nudge on a single stepper | Diagnostic / bypass | **`true`** | Bypasses kinematics entirely; never part of a kernel-shaped trajectory; §3.6 of spec lists it explicitly. |
| 5 | `klippy/extras/manual_stepper.py:78` (`ManualStepper.do_move`) | Manual-stepper motion (MMU-style selector, etc.) | Diagnostic / bypass | **`true`** | Runs on its own private trapq; shaping has no physical meaning on an axis that isn't the shaped XY. §3.6. |
| 6 | `klippy/extras/manual_stepper.py:161` (`ManualStepper.drip_move`) | Homing a homed manual-stepper axis (calls `do_move` in drip) | Homing | **`true`** | Homing move — `trapq_append` again at line 78 via `do_move`; inherits #5's flag. |
| 7 | `klippy/extras/trad_rack.py:2391-2393` (TradRack-owned trapq) | TradRack's private `TradRackToolHead` feeds the same lookahead path | Print | `false` (default) | It's a full secondary toolhead; whether it shapes depends on whether it has an `[input_shaper]` tied to it (today — no). Practically: no shape, no baking; flag irrelevant. |

### Indirect emission paths (no direct `trapq_append` call, but produce a move that flows into one of the above)

| # | Site (file:line) | Flows into | Classifier | `shape_disabled` | Reason |
|---|------------------|------------|------------|------------------|--------|
| 8 | `klippy/toolhead.py:749-775` (`ToolHead.drip_move`) | #1/#2 via `lookahead.flush()` at 769 | Homing | **`true`** | Homing feed move. §3.6 explicit. Drip loop calls `self.move(newpos, speed)` → `lookahead.add_move` → `_process_moves` → `trapq_append*`. Confirmed flow through lookahead. |
| 9 | `klippy/extras/homing.py:140` (`HomingMove.homing_move` → `toolhead.drip_move`) | #8 | Homing | **`true`** (propagated through #8) | Main homing path for all endstop-based moves. Also used by `probe.py` via homing infrastructure (see #11). |
| 10 | `klippy/kinematics/idex_modes.py:80` (`DualCarriagesRail.toggle_active_dc_rail` → `toolhead.set_position`) | `trapq_set_position` — no motion emit | Boundary | n/a (set_position) | Carriage handoff; see §2.1 on set_position boundary treatment. |
| 11 | `klippy/extras/probe.py`, `klippy/extras/probe_eddy_current.py`, `klippy/extras/dockable_probe.py`, `klippy/extras/load_cell_probe.py` | #9 via `HomingMove.homing_move` | Probing | **`true`** (propagated through #9) | All probe implementations delegate the actual motion to `HomingMove`, which calls `drip_move`. No probe-side direct trapq access; `tmc*.py` likewise has no direct trapq emission (verified). |
| 12 | `klippy/extras/dockable_probe.py:428,438,442` (`toolhead.manual_move`) | #1/#2 via `toolhead.move` | Print (travel during probe) | `false` (default) | Non-homing positional moves before probing — these are normal travels that benefit from shaping; only the actual touch-off must be unshaped. |

### Count split

- **Must-be-unshaped:** 5 direct sites (#4, #5, #6, #8, #9) + 1 class-of-probes (#11) that inherits from #9. Concretely: **2 direct flag plants** (in `force_move.py` and `manual_stepper.py`) plus **1 planner-side plant** (drip_move path in `toolhead.py`) covers all six.
- **Must-be-shaped:** 2 sites (#1, #2) + #12 (manual_move travel) + #7 (trad_rack if ever shaped).
- **Conditional:** 1 site (#3, extruder). Resolution below.
- **Boundary (no motion emit):** 1 (#10).

## 2. Edge-case discussion

### 2.1 `set_position` boundary (`trapq_set_position`, trapq.c:336-362)

`trapq_set_position` today: flushes all pending moves via `trapq_finalize_moves(tq, NEVER_TIME, 0)`, then writes a single zero-length marker at `print_time` as `MOVE_LINEAR` (kind=0 via the `memset` in `move_alloc`).

Under Plan 8, kernel support of moves **before** a `set_position` must not bleed into the evaluation of moves **after** the set_position — otherwise a homed-then-repositioned axis could see phantom motion contributions from pre-homing geometry.

The bleed-through concern applies only to the **step-generator side** (piecewise evaluator consuming FIR pieces with delay offsets that reach back across the set_position boundary). The trapq itself is flushed cleanly.

**Proposed handling:**

1. The set_position marker move carries `shape_disabled = true`. When the step-gen piecewise evaluator reaches a marker, it treats preceding FIR impulse-delay offsets as zero-contribution (i.e., no cross-boundary lookback).
2. Equivalently: emit a `kernel_support` worth of zero-polynomial padding after a set_position, long enough that all impulse delays fall cleanly inside unshaped territory. Lookahead-side (§6.4 research) already has to extend the flush window to `max(kernel_support) * safety`; setting that window after a set_position costs nothing extra.
3. Implementation: `toolhead.set_position` at `klippy/toolhead.py:631-639` already calls `self.flush_step_generation()` before `trapq_set_position`. Add a post-call: `shape_disabled=True` on the next N moves emitted, where N = ceil(kernel_support / min_move_t). The planner polynomial composer picks this up from a small counter on `struct toolhead`.

Recommendation: option 1 (marker carries the flag) for Chunk 2; option 2 (pad moves) is belt-and-braces and can fold in if hardware shows any boundary artifacts.

### 2.2 `drip_move` path flow through lookahead

Verified at `klippy/toolhead.py:749-775`:

- `drip_move` calls `self.lookahead.flush()` at line 752 (clear state), then `self.move(newpos, speed)` at 762 (which calls `self.lookahead.add_move(move)` at 676), then `self.lookahead.flush()` at 769.
- The second flush invokes `_process_moves` which does call `trapq_append` / `trapq_append_quintic`.
- **So yes — drip_move moves reach the trapq via the lookahead queue's normal flush path, not a bypass emit.**

This is good news: a single planting point suffices. `drip_move` sets a `self._drip_in_progress = True` flag on toolhead (or stamps the specific `Move` object), and `_process_moves` propagates the flag onto the C-side `struct move` at the `trapq_append*` call. No changes needed to `lookahead.py` itself.

### 2.3 Extruder-only moves (pure E, zero XY velocity)

Current behavior (`klippy/kinematics/extruder.py:766-771`): if a move has no XY motion, `extr_r = [0.0, 0.0, axis_r]` and PA is **not applied** (`extr_r[:3]` are zero, so the kin_shaper cascade sees zero XY velocity → zero PA contribution).

Under Plan 8 with baked PA: an extruder-only move has no XY velocity polynomial to share, so there is no cascade XY kernel to invert/bake. The correct behavior is to emit a pure E polynomial **with `shape_disabled = true`** — the XY shaping has nothing to inherit from, and applying PA to a pure extrude move (retract / purge / filament-load) is an existing no-op.

**Rule:** if `move.axes_d[0] == 0 and move.axes_d[1] == 0` at the extruder.move entry point, set `shape_disabled = true` on the extruder-side trapq entry regardless of the XY flag. For moves with real XY content, inherit the XY move's flag (homing-drip extrudes unshaped, normal moves shape as usual).

## 3. Required code edits (file:line, nature of change)

Once `struct move` gains a `shape_disabled:1` bitfield (Plan 8 Chunk 2 work, spec §5), the Python bypass threading:

1. **`klippy/chelper/trapq.c:196` (`trapq_append` signature)** — add `int shape_disabled` as a new final parameter. Thread into the per-sub-move allocation (three places: accel, cruise, decel blocks) to set `m->shape_disabled = shape_disabled;` after each `move_alloc()`.
2. **`klippy/chelper/trapq.c:252` (`trapq_append_quintic` signature)** — same: new `int shape_disabled` parameter, set on the single allocated `struct move`.
3. **`klippy/chelper/__init__.py:120` and `:125`** — bump both cdef signatures to match.
4. **`klippy/toolhead.py:482` and `:496`** — propagate `getattr(move, "shape_disabled", False)` (default false) into both `trapq_append*` calls.
5. **`klippy/toolhead.py:749-775` (`drip_move`)** — set `move.shape_disabled = True` before `self.move(newpos, speed)` at line 762. Since `self.move` constructs a new `Move` at line 647, the clean approach is to set `self._drip_mode = True` at line 753 and have `Move.__init__` (in `toolhead.py` ~line 70) check it to set `self.shape_disabled`.
   - Also reset `self._drip_mode = False` in the finally-block after line 775.
6. **`klippy/extras/force_move.py:103`** — append a hard-coded `True` as the new final argument to `self.trapq_append(...)`.
7. **`klippy/extras/manual_stepper.py:78`** — same: append `True` as the new final argument.
8. **`klippy/kinematics/extruder.py:772`** — pass a computed flag: `bool(getattr(move, "shape_disabled", False) or (move.axes_d[0] == 0.0 and move.axes_d[1] == 0.0))`.
9. **`klippy/extras/trad_rack.py:2391-2393`** — no edit needed (default-false propagation from whatever lookahead feeds it).
10. **`klippy/toolhead.py:631-639` (`set_position`)** — depending on which option from §2.1 wins:
    - Option 1: add a `self._post_set_position_unshaped = N` counter; decrement on each `_process_moves` iteration. `Move.__init__` picks it up.
    - Option 2 requires the lookahead-window extension work tracked in research gap §6.4 and can land there.

Test scaffolding (not code edit, but belongs in the plan):

- Unit test exercising each `shape_disabled=True` site and asserting the emitted polynomial coefficients match the degenerate-linear-equivalent case.

## 4. Test plan — per must-be-unshaped site

Every test below should assert via the pull-moves / motion-report extraction API (`ffi_lib.trapq_extract_old`, `klippy/extras/motion_report.py:137`) that the emitted polynomial's coefficients satisfy:

- accel phase: `c[2] = 0.5 * accel`, `c[1] = start_v`, `c[0] = start_pos`, all higher-order coefficients (c[3]...c[10]) = 0.
- cruise phase: `c[1] = cruise_v`, `c[0] = cruise_start_pos`, all others = 0.
- decel phase: `c[2] = -0.5 * accel`, `c[1] = cruise_v`, all others = 0.

i.e., bit-exact match to what `trapq_append` would have produced from the same `(accel_t, cruise_t, decel_t, start_v, cruise_v, accel)` tuple. Call this the **degenerate-linear fingerprint**.

| Must-be-unshaped site | Test |
|-----------------------|------|
| #4 `force_move.manual_move` | `test_force_move_shape_disabled`: configure `[input_shaper] shaper_type_x=mzv shaper_freq_x=50`. Issue `FORCE_MOVE STEPPER=stepper_x DISTANCE=10 VELOCITY=20 ACCEL=100`. Extract the emitted trapq entry; assert degenerate-linear fingerprint. Assert the polynomial evaluation at the accel/decel boundaries matches the closed-form trapezoid to 1e-12 (no baked smoothing). |
| #5 `manual_stepper.do_move` | `test_manual_stepper_shape_disabled`: define a `[manual_stepper ms_test]`. Issue `MANUAL_STEPPER STEPPER=ms_test MOVE=5 SPEED=10 ACCEL=50`. Same fingerprint check. |
| #6 `manual_stepper.drip_move` | `test_manual_stepper_homing_shape_disabled`: on a `[manual_stepper]` with `endstop_pin`, issue `MANUAL_STEPPER STEPPER=ms_test MOVE=5 STOP_ON_ENDSTOP=1 SPEED=10 ACCEL=50`. Trigger the endstop immediately; assert fingerprint. |
| #8 `toolhead.drip_move` | `test_drip_move_shape_disabled`: configure shaping on X+Y. Trigger a `G28 X` homing move. Extract the trapq entries emitted during drip; assert each carries `shape_disabled=true` (extracted via pull_move); fingerprint match for each. Additionally assert that the extruder trapq for the same window is empty (no PA bake). |
| #9 homing flow (endstop-based) | Covered by #8 — the homing module hits the same `drip_move` path. No separate test necessary; but add `test_probe_homing_move_shape_disabled` as a sanity check exercising `PROBE` with a simulated touch-off. |
| §2.1 `set_position` boundary | `test_set_position_no_kernel_bleed`: issue a shaped move, then `SET_KINEMATIC_POSITION`, then a second shaped move starting from the new origin. Extract steps; assert the second move's step pattern matches a fresh-boot shaped-move pattern (no offset from carry-over impulse contributions of the first move). Tolerance: stepper-position delta ≤ 1 step between boundary case and fresh-boot case. |
| §2.3 pure-E move | `test_extrude_only_shape_disabled`: with shaping + nonzero `pressure_advance`, issue `G1 E5 F100` with XY held. Extract extruder trapq entry; assert fingerprint of pure linear E (no PA bake). Contrast with `G1 X10 E5 F1200` which **must** show PA baked (non-trivial E polynomial). |

All tests run in klipper-sim against the sim regression harness (`~/Developer/klipper-sim/`), which already knows how to extract pull_moves and compare polynomial coefficients (cf. 2026-04-20-plan1 validation infra).

## 5. Summary

- 5 direct trapq emit sites require `shape_disabled = true`; 1 (extruder) is conditional; 2 are the main print path (default false); 1 is the trad_rack sub-toolhead (default false, no shaping wired).
- Threading lives cleanly at the Python → C boundary: add one `int shape_disabled` parameter to `trapq_append` and `trapq_append_quintic`, default-false at the call sites that don't care.
- `drip_move` flows through the normal lookahead queue, so a toolhead-level flag (`self._drip_mode`) stamping freshly-constructed `Move` objects is the minimal-surface intervention — no lookahead.py changes required.
- `set_position` boundary: mark the history-marker move with `shape_disabled = true` and rely on the step-gen piecewise evaluator refusing to cross it for FIR delay lookback. If HW shows bleed artifacts, pad with unshaped moves for one `kernel_support` window after each set_position.
- Test plan: one sim-harness test per must-be-unshaped site; all assert the degenerate-linear polynomial fingerprint on the emitted trapq entry.
