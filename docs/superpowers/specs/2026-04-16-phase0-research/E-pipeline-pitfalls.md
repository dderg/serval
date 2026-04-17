# E — Pipeline Pitfalls: Integration Risks of Native Arc Blending in Kalico

Scope: architectural/integration risks inside the Kalico motion pipeline if
we replace the zero-duration corner with a planner-emitted arc of
configurable radius. Grounded in a reading of the code.

---

## 1. Executive Summary — Five Biggest Risks

1. **`struct move` hard-codes a straight-line primitive.** Every trapq
   consumer (itersolve, every `kin_*.c`, `kin_shaper.c`, `kin_extruder.c`,
   `motion_report`, `tap_analysis`) reads `start_pos + axes_r * distance(t)`
   as a line. Adding a second "arc" move type is pervasive surgery, not a
   local change. `klippy/chelper/trapq.h:15-21`, `trapq.c:31-39`.
2. **Input shaping convolution breaks physical meaning on curved paths.**
   `calc_position` (`kin_shaper.c:91-103`) is a linear convolution on the
   commanded toolpath. `shift_pulses` (`kin_shaper.c:28-39`) is justified
   only because constant-velocity *linear* motion is an identity.
   Convolution across a curve produces a different curve, not a shaped
   version of the original.
3. **Nonlinear kinematics (`delta`, `deltesian`, `polar`, `rotary_delta`,
   `winch`) already nonlinearize position** — the secant-method iterator
   `itersolve_gen_steps_range` (`itersolve.c:28-128`) may see oscillation
   edge cases when a second trig nonlinearity is introduced by arcs.
4. **Lookahead semantics change.** `LookAheadQueue.flush`
   (`toolhead.py:164-226`) computes `reachable_start_v2` from
   `delta_v2 = 2·move_d·accel`. Inserting blend arcs couples geometric
   constraints (arc fit, radius shrinkage, neighbor `move_d` reduction)
   with the velocity backward pass — the one-pass pairwise `calc_junction`
   (`toolhead.py:79-122`) cannot express it.
5. **Pressure advance math is a 1-D integral and survives
   (`kin_extruder.c:54-112`), but the `instantaneous_corner_velocity` guard
   (`extruder.py:235, 328-332`) keys on `diff_r` at a discrete junction.
   Blended corners erase that junction**, silently bypassing a PA/jerk
   safety.

---

## 2. Per-Component Analysis

### 2.1 trapq — `klippy/chelper/trapq.{h,c}`

**Current model.** `struct move` (`trapq.h:15-21`) encodes a
trapezoidal-velocity straight segment: scalar
`start_v + half_accel*t` along a unit `axes_r`. `move_get_coord`
(`trapq.c:31-39`) reconstructs XYZ as `start_pos + axes_r*d`. Every
consumer trusts `|axes_r|=1` and linearity.

**What an arc primitive needs.** `(center, r_cos, r_sin, plane_u, plane_v,
theta_start, omega)` or equivalent. Crucially, arc-length parametrization
is still `start_v*t + half_accel*t²`, so the trapezoidal-velocity model
survives iff `move_get_coord` dispatches on move type.

**What breaks.**

- `trapq_append` (`trapq.c:118-164`) splits a logical move into up to three
  sub-moves sharing `axes_r`. An arc must either become a single
  variable-geometry primitive with internal trapezoidal speed, or split
  into three sub-arcs sharing a center. The flat C signature
  `(axes_r_x, axes_r_y, axes_r_z)` forks.
- `move_get_coord` is inlined hot-path code (`trapq.c:31-39`); every
  callsite (§2.3) reads its output shape directly. Making it polymorphic
  costs a function pointer per step iteration.
- `trapq_extract_old` / `struct pull_move` (`trapq.c:231-256`,
  `trapq.h:27-32`) flatten to seven scalars assuming line geometry.
  Consumers: `extras/motion_report.py:100-179`,
  `extras/load_cell/tap_analysis.py:26-49, 405-410`. Either extend the
  ABI or segment arcs into history (see §4).
- `check_active` in `itersolve.c:137-143` tests `m->axes_r.{x,y,z} != 0`.
  Needs type dispatch: an arc's "active axes" = arc plane ∪ helical axis.
- `trapq_set_position`, null-fill, sentinels (`trapq.c:75-116, 166-228`)
  are move-type-agnostic. Safe.

**Verdict: MEDIUM-HARD.** Data-structure rewrite is small; ripple through
consumers is the real cost.

### 2.2 Input shaping — `klippy/chelper/kin_shaper.c`

`calc_position` (`:91-103`) convolves shaper pulses with
`get_axis_position` (`:68-89`), which is hard-coded to
`axis_r.axis[i] * move_dist` — linear-only.

Two problems:

1. **Physical meaning on curves differs.** For a line, shaping convolves a
   1-D velocity profile with a low-pass impulse response. For a circular
   arc at constant speed, the X-axis velocity is sinusoidal; convolution
   scales/phase-shifts it but does not cancel resonance except at specific
   shaper/arc-frequency ratios. The `shift_pulses` identity (`:28-39`)
   assumes constant-velocity *linear* motion; on an arc, constant speed
   does not mean constant per-axis velocity, so the zero-sum shift
   introduces position error.
2. **The code rewrite is bounded** (swap the per-axis accessor for an
   arc-aware one), but validating resonance cancellation on hardware is
   not.

**Open question.** If blend arcs are always short (≤2–3 mm), the shaper's
pre/post-active window (`:193-209`) may already smear the arc into
neighbors such that the shaped output is close enough in practice. Needs
simulation.

**Verdict: HARD, research-required.**

### 2.3 Kinematics — `klippy/chelper/kin_*.c`, `klippy/kinematics/*.py`

Every C stepper kinematics reads `struct coord c = move_get_coord(m, t)`
and applies a function of `c`:

| File | `f(c)` | Difficulty |
|---|---|---|
| `kin_cartesian.c:15-33` | `c.x`, `c.y`, `c.z` | Easy |
| `kin_corexy.c:14-27` | `c.x ± c.y` | Easy |
| `kin_corexz.c:13-27` | `c.x ± c.z` | Easy |
| `kin_idex.c:23-34` | affine on `c` | Easy |
| `kin_shaper.c:119-157` | convolution (§2.2) | Hard |
| `kin_extruder.c:119-131` | `move_get_distance` only | Easy (§2.7) |
| `kin_delta.c:20-28` | `sqrt(arm²−Δx²−Δy²)+c.z` | Hard |
| `kin_deltesian.c:20-29` | `sqrt(arm²−Δx²)+c.z` | Medium |
| `kin_polar.c:14-34` | `sqrt(x²+y²)`, `atan2(y,x)` | Hard |
| `kin_winch.c:20-29` | Euclidean distance | Hard |
| `kin_rotary_delta.c:42-56` | rotated `c`, inverse-arm | Research |

For Cartesian-family: `c` is linear in arc parameter, so stepper position
becomes sinusoidal. The secant-method iterator handles general monotone-ish
functions via bracketing plus bisection fallback
(`itersolve.c:54-67`); it converges but may be slower when a seek window
straddles an inflection.

For delta/polar/winch/rotary_delta: stepper position is
`sqrt(quadratic of sines) + linear`. `SEEK_TIME_RESET = 100 µs`
(`itersolve.c:25`) and the exponential search (`:63`) were tuned assuming
smooth, monotone behavior in a few-µs neighborhood of the target —
locally still true, but the oscillation guard
(`itersolve.c:79-84, check_oscillate`) can mis-fire on arc mid-height
inflections. Worth fuzzing.

**Python `check_move`** (e.g. `cartesian.py:138-156`, `delta.py:165-207`,
`polar.py:115-131`, `limited_cartesian.py:133-151`,
`limited_corexz.py:84`) inspects only `axes_d`/`end_pos`. For delta,
`extreme_xy2 = max(end_xy2, start²)` (`delta.py:198-200`) **does not
contain an arc that bulges outward** between endpoints. Envelope checks
must add arc-bulge handling.

**Verdict.** Cartesian family: Easy. Nonlinear: Medium to Hard —
technically unchanged if `move_get_coord` returns the right `(x,y,z)`, but
timing performance, envelope checks, and iterator edge cases need
per-kinematic verification.

### 2.4 Step compression — `klippy/chelper/stepcompress.c`

`stepcompress` compresses step times into `{interval, count, add}` triples
approximating `interval += add` (`:7-15, 51-55`). Quality depends on the
second difference of step times being roughly constant over ~`count`
steps — i.e. locally quadratic (`QUADRATIC_DEV = 11`, `:102`). For
trapezoidal-velocity linear motion this is exact on each accel/cruise/decel
segment.

For an arc at speed `v`, radius `R`, one axis's position is
`R·sin(vt/R)` — second-difference oscillates with period `2πR/v`.
Compression efficiency drops when `count` approaches a fraction of that
period.

- R=1 mm, v=100 mm/s → period ~63 ms → thousands of steps available:
  fine.
- R=0.2 mm, v=200 mm/s → period ~6 ms → short compression runs, `add`
  changes frequently, MCU bandwidth up. **Not a correctness problem, a
  step-rate ceiling degradation.**

**Verdict: MEDIUM.** Compression works; throughput budget needs
validation.

### 2.5 Lookahead / `calc_junction` — `klippy/toolhead.py`

`Move.calc_junction` (`:79-122`) uses approximated centripetal velocity
`R_jd = sin_theta_d2 / (1 − sin_theta_d2)` × `junction_deviation·accel`,
where `junction_deviation` is derived from `square_corner_velocity`
(`:787-790`). The backward pass in `LookAheadQueue.flush` (`:164-226`) uses
`delta_v2 = 2·move_d·accel` (`:57`).

**What changes with native blend arcs:**

- **Arc has an exact centripetal limit:** `v² = accel·R`. Replaces the
  heuristic junction formula. Good.
- **Arc is a third queue entry** between the two straights; backward pass
  still works, but the arc's `move_d` (arc length) and `delta_v2` must be
  populated before the pass.
- **Geometric pathologies:**
  1. *Short-segment:* a 0.4 mm segment between two corners each wanting
     ≥0.3 mm blend radius can't fit both. Requires coupled
     radius-shrinkage with velocity — a forward geometric constraint
     propagated with backward velocity constraint. Klipper's one-pass
     `calc_junction` doesn't express this.
  2. *Cascading sharp corners:* straight between them shrinks; arcs may
     need to touch or merge.
  3. *Arc overlap into neighbors:* adjacent `move_d` shrinks → `delta_v2`
     must be recomputed before lookahead.
- `LookAheadQueue.add_move` (`:228-236`) calls `calc_junction` immediately
  on append. Today it's pairwise and history-free. Blend radius depends on
  the *next* unprocessed moves, so the decision must defer to `flush` or
  be revocable.

**Verdict: HARD.** This is likely the biggest rewrite on the Python side
— coupling a geometric solver to the existing velocity solver.

### 2.6 G2/G3 — `klippy/extras/gcode_arcs.py`

`ArcSupport` (`:27-199`) chops G2/G3 into G1 segments of
`mm_per_arc_segment` (default 1 mm, `:30`). Every artificial sub-corner
goes through `calc_junction`; centripetal limiting emerges from stacked
mini-chord angles.

With native arcs:

- **Cleanest:** bypass `planArc` entirely, pass `(I,J,direction)` as a
  single arc Move. Fall back to segmentation only if a blend arc at the
  endpoint overlaps the G2.
- **Tangent direction at G2 endpoint** (not chord direction) must be
  known to the planner when picking a blend radius there.

**Verdict: EASY-to-MEDIUM** given §2.1 and §2.5.

### 2.7 Extruder / pressure advance — `klippy/kinematics/extruder.py`,
`klippy/chelper/kin_extruder.c`

Extruder trapq is separate (`extruder.py:238-242, 334-363`). It uses
`axes_r.x` as 1-D extruder ratio and overloads `axes_r.y`/`axes_r.z` to
smuggle PA settings (`extruder.py:346-358`, `kin_extruder.c:62-74`).
`pa_range_integrate` (`kin_extruder.c:85-112`) is a pure function of time
and the 1-D trapezoidal profile; input is `move_get_distance` only
(`:127`). **XY geometry is never read.**

**Survives.** Commanded speed along an arc is trapezoidal in arc-length
time, so PA integrates correctly — nothing downstream notices.

**Breaks.** `PrinterExtruder.calc_junction` (`extruder.py:328-332`) uses
`diff_r = move.axes_r[3] − prev_move.axes_r[3]` to invoke
`instantaneous_corner_velocity`. With a blend arc, adjacent moves likely
share the same per-arc-length extruder ratio, so `diff_r≈0` and the guard
collapses. Real extruder-ratio discontinuities (retract→print, purge
transitions) slip through.

**Verdict: EASY** on PA math, **MEDIUM** for `calc_junction` semantics.

### 2.8 Endstops / homing / ancillary

- `extras/homing.py:100-202` uses `drip_move`; homing moves are always
  linear. No impact if we gate blending off for `drip_move`.
- `extras/manual_stepper.py:78-93`, `extras/force_move.py:78-120`,
  `extras/trad_rack.py:2393` call `trapq_append` with linear params on
  their own trapqs. Safe — they never emit arcs.
- `extras/motion_report.py:100-179` logs `(x_r, y_r, z_r)` and
  reconstructs position linearly — silently wrong on arcs unless
  `pull_move` is extended.
- `extras/load_cell/tap_analysis.py:26-49, 405-410` — same issue, same
  fix.

---

## 3. Ranked Risks (Descending)

1. **Input shaping on curved toolpaths (physical correctness).**
   `kin_shaper.c:91-103`. No first-principles answer exists for what
   convolving a shaper impulse response with an arc produces on machine
   resonance. Top candidate for "2× the scope we thought."
2. **Lookahead geometric-velocity coupling.** `toolhead.py:79-236`. The
   pairwise junction calc cannot express "radius here depends on two
   downstream moves' lengths and angles." Planner redesign, not a patch.
3. **trapq polymorphism ripple.** Two primitive types force every
   `calc_position_cb` callsite, `pull_move`, `motion_report`, and
   `tap_analysis` to fork. Large footprint, small per-edit.
4. **Nonlinear-kinematics iterator stability.** `itersolve.c:28-128`.
   Secant/bisection on arc-sinusoid-under-sqrt may hit oscillation-guard
   edge cases that never occur today.
5. **Extruder junction safety with blended corners.**
   `extruder.py:328-332`. `instantaneous_corner_velocity` is bypassed at
   blended junctions; real extruder-ratio jumps slip through.

---

## 4. "Cheap Path" — Fine-Segmented Linear Approximation

Discretize planner-chosen blend arcs into N straight sub-moves with a
guaranteed chord-error bound ε ≈ s²/(8R). ε=10 µm at R=0.5 mm ⇒
s≈0.2 mm.

**Benefits.**
- **Zero changes** to trapq, itersolve, every `kin_*.c`, stepcompress,
  `kin_shaper`, `kin_extruder`, `motion_report`, `tap_analysis`,
  `force_move`, `manual_stepper`. The entire C pipeline is unchanged.
- Input shaping continues to work on a polyline — the regime it was
  derived for. Shaping a dense polyline is a good approximation of
  shaping the underlying arc, and is similar to what
  `gcode_arcs.py:planArc` (`:160`) already produces today.
- Lookahead continues pairwise; sub-arc junctions have `cos_theta≈1`, so
  `calc_junction` returns ~`max_cruise_v2` and centripetal limiting
  emerges from the chord-angle math.
- `smooth_delta_v2` / `minimum_cruise_ratio` already prevent
  over-decelerating at every microscopic corner.

**Costs.**
- **Step generation throughput** rises with segmentation density. Small
  aggressive blends at speed ⇒ 50+ sub-moves per corner. At ~5000
  corners/s, that's ~250k trapq entries/s — likely fine for itersolve,
  pushes `LookAheadQueue` Python overhead.
- **Stepcompress efficiency** dips at each chord boundary (same regime as
  today's `gcode_arcs.py`).
- **MCU `queue_step` throughput** is the hard ceiling. Today's
  `mm_per_arc_segment = 1.0` default copes; finer per-blend segmentation
  lowers the effective ceiling. Needs measurement.
- **Does not deliver "mathematically smooth" arcs** — same chord
  polyline, just with corner selection moved from slicer to planner.

**Verdict: VIABLE as v1.** Delivers corner-blending semantics without
touching the C-side math. Native arcs become a throughput/quality win,
not a correctness dependency.

---

## 5. Open Questions

1. **Input shaping + arcs.** Does convolution preserve ringing
   cancellation on a radius-R arc? Closed-form or simulation study
   required.
2. **Iterator smoothness on nonlinear kinematics.** Does
   `itersolve_gen_steps_range` converge within `stepcompress.max_error`
   (`stepcompress.c:35, 259-265`) when `calc_position` is
   `sqrt(arm² − sinusoid² − sinusoid²)`?
3. **Planner ordering.** Can blend-radius selection be done per-triple
   without unbounded look-back? If not, `LookAheadQueue` semantics
   including `flush(lazy=True)` (`toolhead.py:164-166, 228-236`) change
   globally.
4. **Extruder corner velocity.** Should
   `instantaneous_corner_velocity` become an along-arc jerk limit or
   migrate into the planner's arc selection (short arc at extrusion
   ratio change)?
5. **`pull_move` ABI.** Third-party consumers (`motion_report`,
   `tap_analysis`) read `(start_x, …, x_r, y_r, z_r)`. Extending the
   struct breaks the CFFI typedef in `klippy/chelper/__init__.py:119`.
   Versioned struct? Parallel arc struct?
6. **Legacy `gcode_arcs.py` interaction.** If user has arc resolution
   set, does the planner re-segment pre-segmented arcs? Fast-path to
   re-merge?

---

## 6. References

- `klippy/chelper/trapq.h:6-53` — `struct coord`, `struct move`,
  `struct pull_move`, public API.
- `klippy/chelper/trapq.c:24-39` — `move_get_distance`,
  `move_get_coord` (linear assumption).
- `klippy/chelper/trapq.c:96-164` — `trapq_add_move`, `trapq_append`
  (accel/cruise/decel split).
- `klippy/chelper/trapq.c:231-256` — `trapq_extract_old` / `pull_move`
  ABI.
- `klippy/chelper/itersolve.c:28-128` — secant step generator.
- `klippy/chelper/itersolve.c:137-143` — `check_active`.
- `klippy/chelper/itersolve.h:6-26` — `stepper_kinematics` layout.
- `klippy/chelper/kin_cartesian.c:14-51`, `kin_corexy.c:13-40`,
  `kin_corexz.c:13-40` — Cartesian family.
- `klippy/chelper/kin_delta.c:20-41`, `kin_deltesian.c:20-41`,
  `kin_polar.c:14-59`, `kin_winch.c:20-42`,
  `kin_rotary_delta.c:42-73` — nonlinear kinematics.
- `klippy/chelper/kin_idex.c:23-82` — IDEX transform.
- `klippy/chelper/kin_shaper.c:28-39` (`shift_pulses`),
  `:68-103` (`get_axis_position*` / `calc_position`),
  `:119-157` (`shaper_{x,y,xy}_calc_position`).
- `klippy/chelper/kin_extruder.c:28-112` (PA integral),
  `:119-131` (`extruder_calc_position`).
- `klippy/chelper/stepcompress.c:31-55` (`stepcompress` /
  `step_move`), `:81-197` (`compress_bisect_add`).
- `klippy/toolhead.py:20-139` (`Move` + `calc_junction` +
  `set_junction`), `:146-236` (`LookAheadQueue`), `:453-494`
  (`_process_moves`), `:600-608` (`set_position`), `:660-704`
  (`drip_move`).
- `klippy/kinematics/cartesian.py:138-156`,
  `delta.py:165-207`, `polar.py:115-131`,
  `limited_cartesian.py:133-151`, `limited_corexz.py:84` —
  `check_move` bounds.
- `klippy/kinematics/extruder.py:235-363` — extruder moves + PA;
  `:328-332` `calc_junction`; `:334-363` `move`.
- `klippy/extras/gcode_arcs.py:27-199` — G2/G3 segmentation.
- `klippy/extras/homing.py:100-202` — `homing_move` / `drip_move`.
- `klippy/extras/motion_report.py:100-179`,
  `extras/load_cell/tap_analysis.py:26-49, 405-410` — `pull_move`
  consumers.
- `klippy/extras/force_move.py:78-120`, `manual_stepper.py:78-93`,
  `trad_rack.py:2393` — auxiliary arc-agnostic `trapq_append`
  callsites.
