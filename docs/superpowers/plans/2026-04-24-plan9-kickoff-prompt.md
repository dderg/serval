# Plan 9 kickoff — green-field motion pipeline rewrite

> **For the assistant:** this file is the entry point for a new session. Read it end-to-end before doing anything. The user explicitly handed you this prompt so you can start fresh with full context. Do not skim.

---

## What we are planning

A complete rewrite of the **motion pipeline** in the Kalico fork (`magnum-opus` branch). Specifically:

- `Move` data structure
- `LookAheadQueue` and the `set_junction` / `calc_junction` math
- The planner ↔ step-gen interface (today: trapq C list + itersolve consumer)
- The shaper / PA composer integration

What stays UNCHANGED:
- MCU firmware (stepcompress, step queue, hardware HAL — years of work, not motion-related)
- Sensor stack (probes, endstops, thermistors, ADCs, multi-MCU clock sync)
- gcode parser, dispatch, configuration, Moonraker
- Stepper driver protocols (TMC, CAN, etc.)
- Kinematics callbacks (cartesian / corexy / delta / polar / etc.)
- Community plugins + configs (best-effort, with documented breaking changes)

The end state: a single coherent motion pipeline with no legacy "trapezoidal contract" preserved. Spline-native, jerk-limited, shaper-baked-everywhere by default.

## Why now

The pattern of Plan 8 was: bake the shaper into the planner. Each implementation step uncovered another legacy contract that didn't fit the new architecture and had to be incrementally rewritten:

- Plan 8 Chunk 1: retired `MOVE_LINEAR` (legacy trapq tagged union).
- Chunk 2: bake XY shaper. Then we discovered the composer needed neighbor-awareness for boundary continuity. Then we missed shape-everywhere (only QuinticBlendMove was baking; single-segment moves emitted unshaped). Then we missed `__visible` on every new C function so LTO stripped them on Linux.
- Chunk 3: bake PA. Then we discovered tanh fits needed degree-6 not 4 for sharp corners.
- Now: jerk-limited motion would need a 2D lookahead rewrite because the trapezoidal `(accel_t, cruise_t, decel_t)` contract is hardcoded everywhere.

The user is fed up with this incremental pattern (correctly). The architectural cause: every fix has been minimum-change, preserving the next legacy contract until it bites. The right move is to commit to a clean motion-pipeline rewrite and stop carrying forward contracts we don't want.

## End-state vision

State-of-the-art FDM motion. Three concrete goals:

1. **Print fast.** Extract maximum throughput from the mechanical envelope — proper jerk-limited motion (true 3rd-order, S-curve acceleration, `max_jerk` user knob), shaper-baked everywhere, polynomial step-gen.

2. **Phase stepping ready.** Motion polynomial output at fine time resolution. Future MCU firmware work (out of scope for Plan 9) can consume that to compute phase currents in real-time. Plan 9's polynomial output should be SUITABLE for phase stepping consumption later.

3. **EtherCAT-servo-ready.** Same polynomial output is what an EtherCAT cyclic-position interface wants. No assumption that the consumer is a discrete-step microcontroller.

These end goals inform the design but are NOT in scope for Plan 9 — Plan 9 lands the host-side motion rewrite. Phase stepping and EtherCAT are future MCU/firmware projects on top of it.

## What is already landed (Plan 8 state)

Branch `magnum-opus`, tip `c56a3bd1` plus follow-ups. Read these to understand current state:

- `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md` — Plan 8 spec
- `docs/superpowers/plans/plan8-research/00-summary.md` — Phase 0 research summary
- `docs/superpowers/plans/plan8-research/bs_polynomial_composer.md` — bs convolution math
- `docs/superpowers/plans/2026-04-23-plan8-chunk1-plan6-fold.md` — Chunk 1 plan
- `docs/superpowers/plans/2026-04-23-plan8-chunk2-bake-xy-shaper.md` — Chunk 2 plan
- `docs/superpowers/plans/2026-04-23-plan8-chunk3-bake-e-and-pa.md` — Chunk 3 plan

Already in the codebase:
- `klippy/chelper/trapq.h` — flattened `struct move` with `phases[MOVE_MAX_PIECES=32]` and 15-coeff polynomial slots, X/Y/Z/E axes
- `klippy/chelper/bs_compose.c`, `fir_compose.c`, `smooth_compose.c` — kernel-shape composers
- `klippy/chelper/linear_pa_compose.c`, `nonlinear_pa_compose.c`, `cheb_fit.c` — PA bakers
- `klippy/blendplanner.py`, `klippy/blendquintic.py` — planner-side polynomial composition + corner blending
- `kin_shaper.c` deleted; `kin_extruder.c` slimmed to step-gen wrapper
- `shape_disabled` flag on `struct move` for homing / force / manual stepper bypass

Known regression (this is what surfaced the green-field discussion):
- **Shaper is only baked on `QuinticBlendMove` (corner blends).** Single-segment moves emit raw degenerate trapezoidal motion via `append_trapezoid_as_quintic`. At high velocity (z_tilt at 1000 mm/s), the unshaped jerk-step at accel/cruise transitions causes mechanical resonance and stepper slip on Trident. **This is the proximate driver for Plan 9.**

Other known caveats:
- Long-cruise numerical precision in `smooth_compose` (~0.5 mm error at >0.4s phase-local time, due to monomial-basis cross-cancellation). Not operationally critical TODAY because long cruises bypass the composer — but Plan 9 routes everything through the composer, so this MUST be addressed (Bernstein or centered-Chebyshev basis).
- Sharp-corner non-linear PA fit residual exceeds 1 µm filament budget on extreme cases (still 0.04 µm on typical, 1.36 µm on full 0–12.5×v_lin tanh accel). Worth quantifying further but not a blocker.
- bs5 PA composition output can exceed degree 14, truncated to fit the 15-coeff slot. Plan 9 may want to bump `MOVE_QUINTIC_POLY_COEFFS` or use a different basis.

## Open architectural decisions for Plan 9

These are the questions the brainstorming should resolve:

### 1. Motion profile shape

Today: 3-phase trapezoidal `(accel_t, cruise_t, decel_t)` with constant accel per phase. Discontinuous jerk at phase boundaries. The shaper smooths it (when it's applied).

Plan 9: spline-native — what shape?

- **Option A**: 7-phase jerk-limited S-curve (`jerk-up, accel, jerk-down, cruise, jerk-down, decel, jerk-up`). True 3rd-order. Industry-standard CNC profile.
- **Option B**: Direct polynomial output, no fixed phase decomposition. Planner emits a single piecewise polynomial per move; the "phases" emerge from constraint switches (jerk hits limit → switch).
- **Option C**: Quintic Bezier-style end-to-end blend per move, similar to the corner blending today but applied to every move.

### 2. Boundary conditions between moves

Today: lookahead matches velocity at junctions (`v_junction`). Acceleration is implicitly zero at the start/end of every move.

Plan 9 with jerk-limited motion needs:

- **Option A**: match velocity AND acceleration at junctions (true C² continuity). Lookahead becomes 2D — pick `(v, a)` pairs that satisfy jerk budget on both sides. Harder math, smoother result.
- **Option B**: match velocity only, accept jerk discontinuity at junctions. The shaper baking smooths it. Closer to standard practice in jerk-limited planners.

### 3. Move data structure

Today: `struct move` with phase polynomials (Plan 8 Chunk 2 layout — `phases[32]`, 15 coeffs, 4 axes).

Plan 9: keep this layout (already polynomial-native, accommodates higher-order spline output) or redesign?

- **Option A**: keep struct, just rename "trapq" → "motion_queue" everywhere. Data structure is fine.
- **Option B**: redesign with Bernstein or centered-Chebyshev basis to fix the long-cruise numerical precision issue and reduce coefficient cancellation.
- **Option C**: per-axis polynomial decomposition (today X/Y/Z/E share `t_end`s — limits per-axis kernel mismatch handling).

### 4. Shape-everywhere mechanism

How does the planner ensure every move (single-segment OR blended) gets the shaper baked?

- **Option A**: route every move through `CornerBlender` even if there's no corner. Single-move emit becomes a "blend with itself".
- **Option B**: separate `ShaperBaker` component that runs on every move's polynomial regardless of source.
- **Option C**: have the planner's polynomial output emit shape-baked by construction (no separate baking step).

### 5. User config

What new knobs, what changes for the user?

- **New**: `max_jerk` (mm/s³), per-axis or global. Replaces SCV / square_corner_velocity (already retired).
- **Retained semantics**: `shaper_type`, `shaper_freq_x/y`, `damping_ratio_x/y`, `pressure_advance`, `pressure_advance_model`.
- **Retired (consider)**: anything that assumes the trapezoidal contract. `pressure_advance_smooth_time` is already vestigial.

### 6. Motion-only vs full green-field

Critical scope question:

- **Option A (motion-pipeline-only):** rewrite host-side motion components, keep Klipper's MCU/sensor/gcode/everything-else. Plan 9's deliverable is a swap-in motion module.
- **Option B (full green-field):** fork Klipper itself, rewrite freely without compatibility constraints. Loses years of MCU/sensor/community work for no motion benefit.

Strong recommendation: **(A)**. The user has previously aligned on this — Magnum Opus IS the motion-only rewrite. Confirm this is still the intent.

## User preferences and constraints (memory)

Read these MEMORY files at session start:
- `~/.claude/projects/-Users-daniladergachev-Developer-kalico/memory/MEMORY.md` (and the files it indexes)

Highlights you must honor:
- Plain English, no Greek letters in conversation (formulas in specs are fine)
- Short chunks, one decision per question (user has ADHD)
- Math derivations via subagent (opus model, never haiku)
- Motion / planner design choices via subagent research
- Execute approved plans straight through (don't ask between tasks)
- Subagent model selection: opus default for implementers + reviewers + architecture; sonnet only for 1-2-file mechanical
- NO Co-Authored-By trailers in commits
- Backdating rule: lifted as of 2026-04-23 evening; commits use actual system time
- Deploy via commit + git pull on Trident; never scp / sed in place on the printer (third-party plugin patches are an exception)
- Fork is the gate — no runtime feature flags; replace cleanly

## Process expectations for this session

1. Use the `brainstorming` skill from the start.
2. Acknowledge the iterative-rewrite frustration; commit to scoping Plan 9 as one cohesive rewrite, not another series of incremental fixes.
3. Ask the open decisions above one at a time. Multiple-choice preferred. Lead with your recommendation and reasoning.
4. Spawn opus subagents for any math or architecture research questions (e.g., "what's the optimal jerk-limited profile for a given (start_v, end_v, max_v, max_a, max_j)?").
5. Once the design is settled, write the spec to `docs/superpowers/specs/2026-04-XX-plan9-greenfield-motion-design.md` and ask the user to review.
6. After spec approval, transition to `writing-plans` skill.
7. Implementation will likely be subagent-driven-development with multiple chunks (similar to Plan 8 Chunks 1-3 cadence). Estimate 4-8 weeks of subagent work end-to-end.

## What success looks like

Plan 9 is done when:

- The planner natively outputs jerk-limited spline polynomial motion. No `(accel_t, cruise_t, decel_t)` decomposition anywhere.
- Every move (regardless of source: single-segment, blended, force_move, manual stepper, homing) is shape-baked except where `shape_disabled` is set.
- `max_jerk` works as a real user knob.
- Long-cruise numerical precision is solved (Bernstein/Chebyshev basis or equivalent).
- z_tilt at 1000 mm/s on Trident does NOT skip steps.
- Voron Cube test gcode prints cleanly at speeds bounded by torque, not by trapezoidal-contract artifacts.
- The C/Python split is clean: planner emits polynomial, step-gen consumes polynomial, no in-between contract.
- The motion pipeline is ONE coherent component, ready to extend toward phase stepping and EtherCAT later.

## First action for the assistant

Greet the user, acknowledge you read this prompt end-to-end, and ask the first decision question (likely from the open-decisions list above — start with §6 confirmation, then §1 motion profile shape).

Stay grounded: this is a real rewrite of significant scope, but the building blocks from Plan 8 dramatically reduce the work compared to starting from zero.
