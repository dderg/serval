# Plan 9 — Green-field motion pipeline design

**Date:** 2026-04-24
**Branch:** `magnum-opus`
**Status:** Draft for user review

---

## TL;DR

Plan 9 is a green-field rewrite of the motion pipeline in the Kalico fork of Klipper. Scope is full fork: host planner, host↔MCU protocol, and MCU step-generation firmware are all rewritten together. Goals: state-of-the-art FDM motion (jerk-limited, shape-baked everywhere), phase-stepping-ready, EtherCAT-ready.

**Seven decisions that define the rewrite:**

1. **Polynomial-native planner.** Each move is a piecewise polynomial; phases (accel / cruise / decel / jerk segments) emerge from constraint switches, not from a hardcoded 7-phase template.
2. **Polynomial-on-MCU protocol (γ).** Host ships polynomial coefficients; MCU Newton-solves for next-step time. `stepcompress` retired on motion MCUs. Follows RepRapFirmware 3.6 / Marlin FT-Motion architecture.
3. **Coast-through programmed junctions (a=0).** Acceleration returns to zero at every junction that isn't wrapped in a corner blend. Blended corners carry nonzero acceleration across. No 2D lookahead.
4. **Shape-baked by construction.** Planner emits shape-baked polynomials directly — no separate baking step, no un-baked intermediate state, no way to accidentally skip shaping.
5. **Per-axis `max_jerk`** (mm/s³) with a global fallback. New first-class user knob.
6. **No Rust.** Python orchestration + C hot paths, like today. Motion wins are algorithmic.
7. **Motion MCUs must be Cortex-M4F or better.** F103 and AVR are dropped from motion; they can still run as sensor-only MCUs.

Additional technical decisions (not user-facing):
- Monomial polynomial basis per-segment with normalized `t ∈ [0, 1]`. Cruise segments are kept linear (not fit into degree-5 polynomials).
- Move merging preprocess: collinear adjacent moves are merged into single segments before polynomial generation.
- Synchronized-stepper bundling: steppers that always move in lockstep (e.g., 3 Z motors) evaluate one polynomial with multiple step-pulse comparators.
- CoreXY and other kinematic-coupling corrections are explicit in the planner's per-axis constraint handling.

---

## Context

### Why now

Plan 8 baked input-shaping into the planner at the polynomial level. Each chunk uncovered the next legacy contract that didn't fit the new architecture: `MOVE_LINEAR` tagged union, neighbor-aware boundary continuity, shape-everywhere (single-segment moves emitted unshaped), LTO `__visible` export, degree-6 Chebyshev for PA corner fit. Each fix was minimum-change, preserving the next legacy contract.

The proximate driver is a hardware regression: on Trident at 1000 mm/s during `z_tilt`, single-segment moves emit raw degenerate trapezoidal motion and the unshaped jerk-step at acceleration transitions causes stepper slip. The root cause is structural — shape-baking is conditional on corner-blending being active.

True jerk-limited motion would need a lookahead rewrite because the trapezoidal `(accel_t, cruise_t, decel_t)` contract is hardcoded across the planner, step-gen, and kinematics. Rather than patch around it again, we commit to one cohesive rewrite.

### Prior work (Plan 8 foundation)

Useful infrastructure already in the tree:
- Polynomial `struct move` with `phases[MOVE_MAX_PIECES=32]` and 15-coeff slots for X/Y/Z/E (`klippy/chelper/trapq.h`)
- Kernel composers: `bs_compose.c`, `fir_compose.c`, `smooth_compose.c`
- PA bakers: `linear_pa_compose.c`, `nonlinear_pa_compose.c`, `cheb_fit.c`
- Quintic corner primitive: `blendplanner.py`, `blendquintic.py`
- `shape_disabled` flag on `struct move` for homing / force / manual stepper bypass
- `damping_ratio` is still plumbed through shaper config

Retired before Plan 9: `kin_shaper.c`, SCV, `square_corner_velocity`, `MOVE_LINEAR` tagged union.

---

## Goals

Plan 9 is done when:

1. Planner natively outputs jerk-limited, shape-baked piecewise polynomial motion for every move (single-segment, blended, `force_move`, manual stepper, homing — except where `shape_disabled` is set).
2. No `(accel_t, cruise_t, decel_t)` tuple anywhere in host or MCU code.
3. `max_jerk` is a real user knob, per-axis with a global fallback.
4. Long-cruise numerical precision is solved (cruise emitted as linear segments, not high-degree polynomials).
5. `z_tilt` at 1000 mm/s on Trident does NOT skip steps.
6. Voron Cube test g-code prints cleanly at speeds bounded by stepper torque, not by trapezoidal-contract artifacts.
7. Host→MCU protocol is polynomial coefficients; MCU evaluates locally.
8. One polynomial representation flows through the whole pipeline — no secondary "sampled step list" contract in between.
9. The design leaves phase stepping and EtherCAT cyclic-position as clean follow-on projects that consume the same polynomial stream.

## Non-goals

Explicitly out of scope:
- Gcode parser, Moonraker interface, configuration system
- Sensor stack: probes, endstops, thermistors, ADCs, multi-MCU clock sync
- Stepper driver protocols: TMC SPI/UART, CAN-bus framing
- Kinematics callbacks: cartesian / corexy / delta / polar / etc. (these are consumers of the planner; their interfaces stay stable)
- F103 / AVR motion support (dropped — sensor-only still works)
- Phase stepping firmware (follow-on project; Plan 9 makes it tractable)
- EtherCAT master implementation (follow-on project; Plan 9 makes it tractable)
- Rust adoption (deferred)

---

## Scope

**Full Klipper fork.** MCU firmware is on the table for the motion pipeline. Host↔MCU protocol is rewritten. Step-generation firmware is rewritten for motion MCUs (M4F+).

**Motion MCUs**: Cortex-M4F or better. F446, F407, F405, G431, G474, H743, H723, SAMD51. No fixed-point path. No weak-MCU path.

**Non-motion MCUs** (sensors, expansion): unchanged. Protocol stays compatible. F103, AVR, SAMD21, RP2040, etc. can still serve as temp / endstop / adc boards. They cannot drive steppers.

Weak MCUs (M0/M0+/M3, no FPU) are not a supported motion target. A fork committed to state-of-the-art FDM motion does not carry a legacy path that forces design compromises onto modern hardware.

**Breaking for community:** users running main motion on SKR Mini E3 (F103), RAMPS-on-AVR, or other pre-M4F boards will need to upgrade hardware. Kalico fork already signals "performance-focused"; this is consistent with that direction.

---

## Magnum-opus pillars elevated in Plan 9

Plan 9 does not start from zero. The magnum-opus saga (Plans 1-8) established three pillars that Plan 9 carries forward as first-class design principles, not just implementation details:

- **Pillar 1 — Shape-baked by construction** (supersedes the original "feedforward inverse-shaper" framing from `docs/Magnum_Opus_Design.md`; `blendshaper_inverse.py` was never needed). The planner emits polynomials that are already shape-baked — see architectural decision #4.
- **Pillar 2 — Smooth corner primitive** (evolved from clothoid through arc-blend to quintic blend). The `CornerBlender` + quintic blend primitive is the only place nonzero acceleration crosses junctions — see architectural decision #3.
- **Pillar 3 — Extruder as first-class kinematic citizen** (Plan 3). The extruder has its own acceleration/RPM budgets, and the pressure-advance model's derivatives `f'(v)`, `f''(v)` feed the profile generator's constraint set — not just the E-polynomial composer. See architectural decision #5.

## Architectural decisions

### 1. Motion profile shape: polynomial-native (Q2 = A)

Each move is a piecewise polynomial in time. The planner does NOT decompose moves into a fixed 7-phase S-curve template. Phases emerge where a constraint hits its limit (velocity cap, acceleration cap, jerk cap, kinematic limit).

**Segment types:**
- **Polynomial segment**: degree ≤ 5 in the monomial basis on normalized `t ∈ [0, 1]`. Used for acceleration phases, deceleration phases, jerk ramps, shape-baked blend primitives.
- **Linear (cruise) segment**: `p(t) = p0 + v·t`. Used for constant-velocity cruise. Explicitly NOT fit into a degree-5 polynomial — this avoids the monomial-basis cancellation issue observed in Plan 8 chunk 2 at cruise phase-local times > 0.4s.

The existing `struct move` with `phases[MOVE_MAX_PIECES=32]` already accommodates this. Each phase carries a type tag (polynomial or linear) and the appropriate coefficients.

### 2. Planner→MCU interface: polynomial-on-MCU (Q3 = γ)

**Wire format:** host ships polynomial segments to the MCU. Each segment: per-axis coefficients (up to 6 coefficients for degree-5, or 2 for linear cruise), segment duration, start-of-segment absolute time.

**MCU step generation:** on each timer tick in the step scheduler, evaluate position polynomial for each stepper, compare to next-step target, fire pulse if crossed. **Optimization: Newton-solve for step time directly** rather than per-tick polling — 6-10× CPU savings. This is the RepRapFirmware 3.6 phase-stepping architecture, proven in production on Cortex-M7.

**Synchronized-stepper bundling:** steppers that are constrained to move identically (e.g., 3 Z motors in `z_tilt` control, dual-Y setups, gantry steppers) share one polynomial evaluator. Each gets its own step-pulse comparator. For the user's F4-based Z board with 3 Z motors, this drops CPU to ~5%.

**Bandwidth:** ~120-240 bytes per segment (SP or DP float) covers all axes. At typical print rates (~50 moves/s): 7.5 KB/s. USB-FS has 1 MB/s effective; CAN-FD has 500 KB/s. Luxurious headroom.

**Queue depth:** MCU buffers ~200 ms worth of segments (matching current `buffer_time_high` semantics). ~10 segments × 240 bytes = 2.4 KB RAM. F4 has 128+ KB RAM; H7 has 560 KB.

**`stepcompress` retired on motion MCUs.** It remains in the codebase as reference but is not compiled into motion-MCU firmware.

### 3. Junction boundary conditions: coast at programmed, C² at blends (Q4 = C)

At any junction between two moves where there is NOT a corner-blend primitive wrapping the transition, acceleration must return to zero on both sides. The planner guarantees this by emitting a jerk-down-to-zero ramp at the end of move A and a jerk-up-from-zero ramp at the start of move B.

At blended corners (quintic corner primitive from Plan 8), the blend absorbs the acceleration transition — velocity and acceleration are C² across the blended region.

Consequence: programmed junctions between non-collinear moves at high velocity are physically slow (the toolhead must coast through the junction). This is correct — a sharp 90° at 500 mm/s cannot be safely smoothed by the planner without the user declaring it as a blend candidate via cornering tolerance.

**Collinear move merging** as a preprocess: inherited from `blendprepass.py` / `CollinearCollapser`. Adjacent moves are merged when all four gates pass:
1. **Speed equality** — target speeds within epsilon (so merging doesn't hide a deliberate speed change).
2. **Flow equality** — extrusion-per-mm within epsilon (variable-flow Arachne segments must NOT be merged, or flow discontinuities leak into the merged polynomial).
3. **Perpendicular deviation** — endpoint lies within epsilon of the extended ray of the previous move (true collinearity, not just near-parallel).
4. **Projection bounds** — merged segment length remains within per-axis sanity bounds.

This is more than "direction dot-product > 1 − ε." Flow changes and retract reversals break merges; a naive re-implementation will fragment real slicer output.

**Half-segment rule for blend consumption** (from `2026-04-17-planner-integration-design.md`): the radius of any inserted corner blend is capped at `R_mid = 0.5 × min(L_prev, L_next) / tan(θ/2)`, so adjacent blends cannot overlap regardless of lookahead pass order. This is a correctness invariant, not an optimization — without it the planner depends on pass direction (a bug LinuxCNC fixed decades ago).

### 4. Shape-everywhere by construction (Q5 = C)

No move leaves the planner un-shaped unless its `shape_disabled` flag is set. There is no "baking step" that can be skipped, no conditional code path — the planner's polynomial output is shape-baked as an invariant.

**How:** the planner's per-axis polynomial emitter is composed of two layers:
1. **Inner trajectory generator**: produces un-shaped polynomial coefficients from move geometry + kinematic constraints.
2. **Outer shape composer**: convolves the per-axis shaper kernel (bs / fir / smooth / zv / ei / etc.) with the inner polynomial, producing the shape-baked output polynomial.

These layers are not optional. For any move that disables shaping (`shape_disabled`), the outer layer is a no-op pass-through — but the code path is identical.

**`shape_disabled` bypass audit** (inherited as an explicit Phase A deliverable). `shape_disabled` is the *single* bypass for shape-baking. Every emit site that sets it must be audited once and documented. Known sites to audit:
- `drip_move` (homing approach)
- `force_move` (manual stepper movement)
- `manual_stepper` extras
- IDEX toolhead handoff
- `set_position` immediate repositioning
- Pure-E-only moves when no XY shaper context is active

Any future bypass site that needs to skip shaping must go through `shape_disabled` — no other mechanism is permitted. This is the contract that makes "shape-baked by construction" a real invariant rather than a wish.

**Extruder PA composer** is structurally parallel: the outer layer is pressure-advance composition rather than input-shaping composition. Commits `939f9cd1` (smooth_compose), `49dc1846` (smooth-IS kernel), `c56a3bd1` (nonlinear_pa_compose with degree-6 Chebyshev) are the building blocks.

### 5. User-facing config (Q6)

**New:**
- `max_jerk` (mm/s³) — global fallback
- `max_jerk_x`, `max_jerk_y`, `max_jerk_z`, `max_jerk_e` — per-axis overrides
- Derivation guidance in docs: typical belt-driven CoreXY `max_jerk` is 100k-500k mm/s³ depending on mechanical stiffness; provide a calibration macro

**Extruder as first-class kinematic citizen** (Pillar 3 — from Plan 3). The extruder is a peer of XY/Z in the constraint set:
- `max_extruder_accel` (mm/s² on filament) — already in tree; stays first-class
- `max_extruder_rpm` — stays first-class
- Pressure-advance model derivatives `f'(v)` and `f''(v)` feed the profile generator's constraint set — not just the E-polynomial composer. When the PA model would drive the extruder past `max_extruder_accel` at a phase transition, the profile generator **reduces XY acceleration to keep the coupled extruder motion within budget.** The binding constraint is typically peak-at `min(v_prev, v_next)`.

**Retained:**
- `max_velocity`, `max_accel`
- `shaper_type`, `shaper_freq_x/y`, `damping_ratio_x/y` (already in tree; confirmed alive)
- `pressure_advance`, `pressure_advance_model`
- `target_smoothing` — role unchanged (per-axis runtime velocity cap from shaper bandwidth; `ts=0` sentinel disables the cap)
- `max_extruder_accel`, `max_extruder_rpm` — first-class, not vestigial

**Retired (breaking):**
- `pressure_advance_smooth_time` — vestigial with new PA composer. The old `K_h = (15/8) / smooth_time` peak-factor derivation (Plan 3) is superseded; the new composer derives peak factors directly from `pressure_advance_model`.
- `minimum_cruise_ratio` and the deprecated `max_accel_to_decel` alias — trapezoidal-contract artifacts. Jerk-limited profiles have no "cruise ratio" knob; the profile shape is determined by `max_accel` and `max_jerk`.
- `square_corner_velocity` / `junction_deviation` — already retired pre-Plan-9; Plan 9 does not revive them.
- Feedforward inverse-shaper (original MO Pillar 1 / `blendshaper_inverse.py`) — never landed, superseded by shape-baked-by-construction. Anyone reading `docs/Magnum_Opus_Design.md` and expecting a `blendshaper_inverse.py` should know: it's not coming. Shape-baked planner output *is* the feedforward pre-distortion.
- Shaper-aware and velocity-aware suppression rules (blend-arc era) — superseded by shape-everywhere; every move is shape-baked, so suppression-based corner-speed adjustments are no longer needed.
- Anything else downstream of the trapezoidal contract surfaces during implementation.

**Kinematic scaling:** `max_accel` and `max_jerk` in user config are **toolhead-space** (Cartesian). The planner applies kinematic-coupling scaling per-axis (CoreXY `1/sqrt(2)` for pure-motor moves, `sqrt(2)` for pure-diagonal moves, etc.) when computing per-stepper constraints. This is explicit in the spec because it's the single thing most likely to ship broken.

### 6. Language choice (Q7 = A)

Python for orchestration (reactor, gcode dispatch, config, toolhead controller, lookahead queue management). C for hot paths (polynomial composition, shape baking, MCU step generation). No Rust.

### 7. MCU hardware floor (Q8 = A)

Motion MCUs: Cortex-M4F or better. Enforced at build time — motion firmware does not compile for F103 / AVR / M0 (non-plus) / pure Cortex-M3 without FPU.

Sensor MCUs: unchanged. F103 can still serve as a toolhead temperature / endstop / accelerometer board.

---

## Architecture

### Component map

```
┌──────────────────────────────────────────────────────────────────┐
│                         HOST (Python + C)                        │
│                                                                  │
│  GCODE → Parser → Dispatch → ToolheadController                  │
│                                     │                            │
│                                     ▼                            │
│                              LookAheadQueue                      │
│                          (velocity matching,                     │
│                           collinear merging,                     │
│                           blend detection)                       │
│                                     │                            │
│                                     ▼                            │
│                        JerkLimitedProfileGen (C)                 │
│                        per-axis polynomial emit                  │
│                        with kinematic-coupling scaling           │
│                                     │                            │
│                                     ▼                            │
│                         ShapeComposer (C)                        │
│                       input-shaping + PA baking                  │
│                   (invariant: output is shape-baked)             │
│                                     │                            │
│                                     ▼                            │
│                      MotionSegmentSerializer                     │
│                    (poly coeffs + duration + t_start,            │
│                        per-axis, with kin routing)               │
└────────────────────────────────────┬─────────────────────────────┘
                                     │
                       USB-FS / CAN-FD / CAN / Ethernet
                                     │
┌────────────────────────────────────┴─────────────────────────────┐
│                         MCU (C, M4F+)                            │
│                                                                  │
│                      MotionSegmentReceiver                       │
│                              │                                   │
│                              ▼                                   │
│                         SegmentQueue                             │
│                    (ring buffer, ~200ms deep)                    │
│                              │                                   │
│                              ▼                                   │
│                       PolynomialEvaluator                        │
│                  (per-stepper or bundled-stepper;                │
│                  Newton-solve for next step time)                │
│                              │                                   │
│                              ▼                                   │
│                        StepPulseScheduler                        │
│                     (hardware timer + GPIO)                      │
└──────────────────────────────────────────────────────────────────┘
```

### Host components

**`ToolheadController` (Python)**: gcode dispatch, move submission to lookahead, user-facing commands. Largely unchanged interface.

**`LookAheadQueue` (Python)**: velocity matching at non-blended junctions (coast-through), collinear move merging preprocess, corner-blend detection and application, feeds moves to the profile generator. Replaces today's lookahead.

**`JerkLimitedProfileGen` (C, `klippy/chelper/jerk_profile.c`)**: per-move polynomial emitter. Input: move geometry, `v_start`, `v_end`, and a `KinematicLimits` bundle carrying:
- per-axis `max_accel` and `max_jerk` (toolhead-space)
- kinematic coupling matrix (CoreXY / cartesian / delta / polar)
- extruder caps: `max_extruder_accel`, `max_extruder_rpm`
- pressure-advance model callable with `f'(v)`, `f''(v)`
- shaper bounds (residual ringing frequency, damping ratio) for per-segment velocity caps

Output: piecewise polynomial segments with per-axis coefficients. Produces polynomial and linear (cruise) segments as appropriate. The `KinematicLimits` dataclass pattern (from Plan 1 / Plan 3) is the single carrier for all constraint sources — per-axis jerk, extruder caps, shaper bounds, kinematic coupling all travel together rather than being passed as parallel arguments.

**`ShapeComposer` (C, existing `bs_compose.c` / `fir_compose.c` / `smooth_compose.c` / `linear_pa_compose.c` / `nonlinear_pa_compose.c`)**: convolves shaper kernels with un-shaped per-axis polynomials. Plan 9 extends existing composers to cover all move types uniformly (no `append_trapezoid_as_quintic` shortcut).

**`MotionSegmentSerializer` (C + Python wrapper)**: serializes shaped polynomial segments for transmission to the MCU. Handles kinematic routing (which segment goes to which MCU based on which steppers it drives) and per-MCU clock synchronization.

### MCU components

**`MotionSegmentReceiver` (C, MCU)**: deserialize polynomial segments from host, validate, enqueue.

**`SegmentQueue` (C, MCU)**: ring buffer of active + pending polynomial segments. Evicted when all steppers on that MCU have finished the segment.

**`PolynomialEvaluator` (C, MCU)**: per-stepper or per-bundle polynomial evaluator. Uses Newton-Raphson from a warm initial guess (previous step time + interval estimate) to solve `p(t) = next_step_pos` for `t`. Float on M4F+; double on M7.

**`StepPulseScheduler` (C, MCU)**: hardware timer fires at computed step time; GPIO pulse; advance target.

### Host↔MCU wire protocol

**Segment descriptor (per motion segment, per MCU):**
```
segment_id:         u32
t_start_mcu_clock:  u32   (MCU clock ticks, synchronized upstream)
duration:           u32   (MCU clock ticks)
segment_type:       u8    (POLYNOMIAL | LINEAR)
axis_count:         u8    (how many axes follow)
flags:              u16   (shape_disabled, synchronized, ...)

per-axis (axis_count times):
  stepper_oid:      u8
  coeff_count:      u8    (2 for LINEAR, up to 6 for POLYNOMIAL)
  coeffs:           f32[coeff_count]  (monomial, normalized t)
```

Typical segment: 4-axis polynomial = 16 bytes header + 4×(2+6×4) = 16 + 104 = **120 bytes**.
Small segment: 2-axis linear cruise = 16 bytes header + 2×(2+2×4) = 16 + 20 = **36 bytes**.

**Synchronization:** segment times are in MCU clock domain. Existing multi-MCU clock-sync infrastructure (`clocksync.c`) is unchanged; this is a sensor-layer service that Plan 9 consumes.

---

## Data flow

1. Slicer emits g-code → `gcode_parser`.
2. `ToolheadController` receives move → translates to planner-space → submits to `LookAheadQueue`.
3. `LookAheadQueue` performs:
   - Collinear move merging (preprocess)
   - Velocity matching at non-blended junctions (1D pass, left-to-right + right-to-left)
   - Corner blend detection and insertion
   - `a=0` enforcement at non-blended junctions
4. Moves flush in FIFO order to `JerkLimitedProfileGen`.
5. `JerkLimitedProfileGen` emits piecewise polynomial + linear segments satisfying `max_velocity`, `max_accel`, `max_jerk` per axis (with kinematic coupling).
6. `ShapeComposer` convolves shaper kernels; output is shape-baked polynomial. PA composer adds extruder correction.
7. `MotionSegmentSerializer` routes segments to target MCUs; transmits over USB / CAN / Ethernet.
8. MCU enqueues segments.
9. At each step boundary, `PolynomialEvaluator` Newton-solves for the next step time; `StepPulseScheduler` fires the pulse.

---

## Error handling

**Host-side failures:**
- Profile generation fails (infeasible constraints): reject move at lookahead time, surface to user via gcode response. Planner never emits an infeasible segment.
- Shape composer divergence: guard with numerical sanity checks (max coefficient magnitude, max predicted velocity/accel within segment). Fall back to more conservative profile on assertion failure.
- Segment serialization queue backpressure: host pauses move submission when MCU queue is full (existing `buffer_time_high` logic).

**MCU-side failures:**
- Segment queue underrun: toolhead stops cleanly at last known position. Host is expected to stay ahead; underrun is logged as an error.
- Newton-solve non-convergence: fall back to bisection on the remaining segment interval. Log, but do not crash.
- Segment time desync (MCU detects segment's `t_start` is in the past): reject and request resync.

**Homing / force_move / manual stepper:**
- These emit polynomial segments with `shape_disabled` flag. `ShapeComposer` passes them through. Velocity/accel limits may be overridden per existing semantics.

---

## Testing strategy

### Unit tests (host, Python)
- `JerkLimitedProfileGen`: boundary condition tests (v_start=0, v_start=v_max, zero-distance, infeasible velocity → rejected).
- Collinear move merging: adjacent colinear moves merge; non-collinear don't; epsilon boundary.
- `LookAheadQueue`: velocity matching correctness, a=0 enforcement, blend insertion.
- Kinematic-coupling scaling: CoreXY 45° move → per-motor accel = toolhead_accel / sqrt(2).

### Unit tests (C, `klippy/chelper/tests/`)
- Polynomial evaluation at endpoints matches analytical velocity/acceleration.
- Shape composer convolution output is shape-equivalent to reference Python implementation.
- Cruise segment emitted as linear (not fit to degree-5 polynomial).

### Simulator tests (`klipper-sim`)
- Replay Voron Cube gcode, verify no acceleration spikes above `max_accel` per axis.
- Replay octagon and sharp-short test cases, verify velocity profiles match spec.
- Replay `z_tilt` at 1000 mm/s, verify no unshaped-move stepper slip.
- Long-cruise precision: 1m cruise at 400 mm/s, verify position error < 1 µm end-to-end.

### MCU simulation
- `PolynomialEvaluator` on host-emulated MCU timer loop: compare step times to analytically derived values.
- Newton-solve convergence tests: 100k random polynomials within spec, verify all converge within 4 iterations.

### Hardware validation
- Trident (H723 AB, F4 Z): `z_tilt` at 1000 mm/s succeeds without slip.
- Voron Cube print at current ringing-bound accel: print quality equivalent or better than Plan 8 state.
- Direct comparison print: Plan 8 tip vs Plan 9, same gcode, same speeds, same model.
- Stress: 500 mm/s infill with dense slicer output; verify MCU queue never underruns.

---

## Implementation phases

**Phase A — Host planner rewrite (green-field)**
- A1: New `JerkLimitedProfileGen` C module with analytical jerk-limited polynomial emit for single-move segments (start_v, end_v, max_v, max_a, max_j). Unit-tested against derivation.
- A2: New `LookAheadQueue` Python module with velocity matching + collinear merging + blend detection + a=0 enforcement at junctions.
- A3: `ShapeComposer` integration: existing composers apply to every polynomial segment uniformly (remove `append_trapezoid_as_quintic` shortcut).
- A4: Per-axis kinematic-coupling scaling in profile generator. CoreXY correctness verified in simulation.
- A5: Extruder polynomial path through nonlinear PA composer for every move. PA derivatives `f'(v)`, `f''(v)` feed the profile generator's constraint set, bounding XY acceleration at phase transitions.
- A6: `shape_disabled` bypass audit. Exhaustive audit of every trapq emit site (drip_move, force_move, manual_stepper, IDEX handoff, set_position, pure-E) to confirm `shape_disabled` is the single mechanism for skipping shape-baking. Document each site. Add lint rule / test that rejects any new un-shaped emit path that doesn't set `shape_disabled`.

**Phase B — Host↔MCU protocol**
- B1: New wire-format segment descriptors. Retire stepcompress commands on motion MCUs.
- B2: `MotionSegmentSerializer` host-side. Per-MCU routing. Clock-domain conversion.
- B3: `MotionSegmentReceiver` MCU-side. Ring buffer. Basic validation.

**Phase C — MCU polynomial step generation**
- C1: `PolynomialEvaluator` with Newton-solve step-time computation. Float on M4F, double on M7.
- C2: `StepPulseScheduler` integrated with existing `sched.c` timer infrastructure.
- C3: Synchronized-stepper bundling (Z-motor lockstep optimization).
- C4: Segment queue underrun / stall / resync handling.

**Phase D — Integration + validation**
- D1: End-to-end host→MCU on bench. Single-axis move works.
- D2: Multi-axis CoreXY moves work. Kinematic coupling correct.
- D3: z_tilt stress test passes on Trident.
- D4: Voron Cube print equivalent quality to Plan 8.
- D5: Breaking-changes migration doc for the community.

**Phase E — Cleanup + retirement**
- E1: Retire stepcompress from motion MCU builds.
- E2: Retire trapezoidal-contract code paths.
- E3: Update all config docs with `max_jerk` and migration notes.
- E4: Drop F103 / AVR from motion MCU build matrix.

Each phase lands as one or more chunks (PRs). Estimated 4-8 weeks total for subagent-driven implementation.

---

## Open technical details

Things that need to be resolved during implementation rather than now:

1. **Polynomial degree ceiling.** `MOVE_QUINTIC_POLY_COEFFS` is 15; bs5 PA composition can exceed degree 14. Plan 9 may need to bump this, OR use a per-segment degree that truncates gracefully.
2. **Damping ratio calibration workflow.** Now that `damping_ratio` is a real first-class parameter, document the calibration path (shaper_calibrate output → config).
3. **Max_jerk defaults.** Ship sensible per-kinematics defaults (CoreXY, cartesian, delta) derived from typical belt / rail dynamics.
4. **`buffer_time_high` semantics in polynomial world.** Current math is step-time-based; revise for segment-based world.
5. **Multi-axis constraint coupling in profile generator.** When a move is velocity-limited on X but accel-limited on Y, what's the optimal polynomial shape? This is the time-optimal-trajectory-along-path problem — will need a dedicated derivation subagent during Phase A.

---

## References

**Plan 8 prior work:**
- `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md`
- `docs/superpowers/plans/2026-04-23-plan8-chunk1-plan6-fold.md`
- `docs/superpowers/plans/2026-04-23-plan8-chunk2-bake-xy-shaper.md`
- `docs/superpowers/plans/2026-04-23-plan8-chunk3-bake-e-and-pa.md`

**Plan 9 kickoff:**
- `docs/superpowers/plans/2026-04-24-plan9-kickoff-prompt.md`

**External / industrial prior art:**
- RepRapFirmware 3.6 third-order motion / phase stepping — https://docs.duet3d.com/User_manual/RepRapFirmware/Third_order_motion
- Marlin FT-Motion — https://marlinfw.org/docs/features/ft_motion.html
- Prunt (31-phase polynomial motion) — https://prunt3d.com/docs/features/
- Biagiotti & Melchiorri, *Trajectory Planning for Automatic Machines and Robots* (Springer, 2008)
- Singhose, "Command shaping for flexible systems: A review of the first 50 years" (Int. J. Precision Eng. Manuf., 2009)
- Klipper benchmarks — https://www.klipper3d.org/Benchmarks.html

---

## Out-of-scope follow-on projects

After Plan 9 lands, these become tractable:
- **Phase stepping** on H723 AB: consume polynomial stream at fine time resolution, compute phase currents. Separate firmware project.
- **EtherCAT cyclic position** on a servo-capable master: consume polynomial stream, emit CiA-402 CSP setpoints at 1-8 kHz.
- **Rust rewrite** of `klippy/chelper/*.c`: language modernization, unrelated to motion quality.
- **Velocity-dependent accel limits** from stepper torque curve (existing parking-lot idea; now tractable because the polynomial path has per-segment accel visibility).

Weak-MCU support is **not** a deferred follow-on project. It is not coming back.
