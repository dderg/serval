# Plan 5 research — direct-quintic step generation

Status: research / design-options memo. Not yet approved. Written 2026-04-22
on branch `magnum-opus`.

## 1. Current architecture audit

### 1.1 Data flow (today, post-Plan 4)

```
 Python                                   C (chelper)
 ------                                   ----------
 ToolHead.move(newpos, speed)
     |
     v
 blendprepass                             kin_*.c (cartesian/corexy/delta/...)
     |    (per-axis motion primitives)       |  calc_position_cb(m, t)
     v                                        ^
 blendplanner.CornerBlender                   | sampled at ~SEEK_TIME granularity
     |   _emit_blend():                       |
     |      shape.polyline(chord_tol)         |
     |      -> [Vec3 ...] (4096 pts worst)    | wrapped by input_shaper's sk:
     |                                        |
     v                                        | shaper_{x,y,xy}_calc_position
 ToolHead.lookahead                            \
     |                                          shaper_calc_position (pulse conv)
     v                                       or smoother_calc_position
 _process_moves(moves)                          (weighted integral via
     |                                           integrate_move)
     v                                       |
 trapq_append(print_time, accel_t, cruise_t,   returns weighted x(t + offsets)
              decel_t, start_pos_xyz,
              axes_r_xyz, start_v,
              cruise_v, accel)
     |
     v
 struct move {
    print_time, move_t;
    start_v, half_accel;     <-- linear-v trapezoid
    start_pos, axes_r;       <-- unit direction
 }
     |
     v
 itersolve.itersolve_generate_steps(sk, flush_time)
     |
     v
 itersolve_gen_steps_range (secant-method root-find over calc_position_cb)
     |
     v
 stepcompress_append (step clock)
```

### 1.2 What trapq stores (trapq.h:15-21)

```c
struct move {
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;
    struct list_node node;
};
```

One primitive only: straight-line segment with linear velocity ramp.
`move_get_distance` is `(start_v + half_accel * t) * t` (trapq.c:24-28).
`move_get_coord` is `start_pos + axes_r * distance` (trapq.c:31-39). Unit
direction vector `axes_r` is constant per move — the trapq has no notion of
a curved primitive.

`trapq_append` (trapq.c:119-164) slices the S-curve into up to three separate
moves (accel, cruise, decel), each straight. `trapq_add_move` may also
insert zero-motion "null moves" to fill idle time (trapq.c:102-113). Nothing
else uses the queue.

### 1.3 Where the position/velocity query lives

`stepper_kinematics::calc_position_cb(sk, m, t)` (itersolve.h:12) is the one
and only query hook. Called from `itersolve_gen_steps_range` (itersolve.c:
72) inside the secant solver. Signature returns a scalar axis position;
velocity is never queried directly — the solver searches for time-at-target-
position, not position-at-target-time.

Per-kinematic implementations are trivial wrappers over `move_get_coord`:
- `kin_cartesian.c:15-33` — return `c.x`, `c.y`, or `c.z`
- `kin_corexy.c:13-27` — `c.x ± c.y`
- `kin_delta.c:20-28` — `sqrt(arm2 - dx^2 - dy^2) + c.z`

They all call `move_get_coord(m, t)` first, then apply a per-kinematics
formula to the Cartesian triple. This is the critical abstraction layer for
Plan 5: anything that widens `move_get_coord` propagates to all kinematics
for free.

### 1.4 kin_shaper.c — the wrapper contract

`struct input_shaper` wraps `orig_sk` (the real kinematic) and overrides
`calc_position_cb` (kin_shaper.c:176-196). Two flavours:

- Pulse shapers: `shaper_calc_position` (kin_shaper.c:86-98) walks the
  trapq list backward/forward via `list_prev_entry`/`list_next_entry` to
  sample `get_axis_position_across_moves(m, axis, t + τ_i)` and forms
  `Σ a_i * x(t + τ_i)`. Queries are scalar; each pulse calls
  `get_axis_position` which is `start_pos + axis_r * move_get_distance`.

- Smooth shapers: `smoother_calc_position` → `range_integrate` (kin_shaper.
  c:104-160) computes `∫ w(τ) x(t+τ) dτ` in closed form using precomputed
  antiderivatives on the smoother (integrate.c:18-63). This is where the
  trapq's linearity is load-bearing: `integrate_move` (integrate.c:51-63)
  folds `start_v + accel*t` into a polynomial product against the smoother
  antiderivatives `it0, it1, it2`. Any position function richer than
  `p₀ + r·(v₀t + ½at²)` breaks this closed form.

Once the shaper has computed the smeared axis position, it writes it into a
stub `is->m.start_pos` and calls the original kinematic with a neutered
move (`axes_r = 0`, `start_v = 0`, `half_accel = 0`, `move_t = 2*DUMMY_T`).
So the underlying kinematic only ever sees Cartesian points, not a move.

### 1.5 `step_generation_scan_time` / `kin_flush_delay`

`ToolHead.note_step_generation_scan_time(delay)` (toolhead.py:809-816)
maintains a max over all registered delays, stored as `kin_flush_delay`.
Drivers:
- `extras/input_shaper.py:627-635` — `input_shaper_get_step_gen_window`
  (kin_shaper.c:332-338) returns `max(pre_active, post_active)`, derived
  from the shaper's farthest pulse time / half-smoother-support (kin_shaper.
  c:267-293).
- `kinematics/extruder.py:406-443` — same for the extruder's PA smoother.

`_advance_flush_time` (toolhead.py:413-434) uses `kin_flush_delay` to gate
step generation: `sg_flush_want = min(flush_time + STEPCOMPRESS_FLUSH_TIME,
print_time - kin_flush_delay)`. This guarantees the trapq has `kin_flush_
delay` of lookahead past any time the solver might query. `itersolve` then
widens that into `gen_steps_pre_active` / `gen_steps_post_active` per sk
(itersolve.h:22, itersolve.c:159-208). For a quintic whose derivatives are
bounded within a blend of duration ≤ some T_blend, lookahead is
`kin_flush_delay + T_blend/2` — likely unchanged because T_blend is already
well under current `kin_flush_delay` (O(10 ms) shaper support vs O(ms)
blend).

### 1.6 Other trapq consumers

- `extras/motion_report.py:123-180` — `DumpTrapQ` reads via
  `trapq_extract_old` (trapq.c:231-256), exporting `pull_move` dumps for
  analysis and an in-memory "current position" query. **Assumes linear
  primitive** — reconstructs position as `start + (start_v + 0.5·accel·t)·t
  · direction` (motion_report.py:172-177).
- `extras/manual_stepper.py:32-96`, `extras/force_move.py:42-121`,
  `kinematics/extruder.py:644-763` — all call `trapq_append`; they consume
  and produce only linear primitives. Extruder also has `trapq_set_position`
  calls.
- `extras/trad_rack.py` — same pattern; auxiliary trapqs.
- Pressure advance (`kin_extruder.c`) reads the same trapq as the XY
  kinematics but through its own `calc_position_cb`. Uses both
  `shaper_calc_position` and smoother integration (kin_extruder.c:184-226),
  again both depending on the linear `move_get_distance` form.
- Homing: no direct reads; it drives `trapq_set_position` and normal moves.

### 1.7 How the quintic reaches trapq today

`CornerBlender._emit_blend` (blendplanner.py:166-252) calls
`shape.polyline(chord_tol)` (blendquintic.py:597-598) to get a `Vec3` list
via adaptive De Casteljau subdivision. Each consecutive pair becomes a
Python `Move`; the planner then emits them through the normal path to
`trapq_append`. Every polyline vertex is a C¹ kink: `axes_r` flips
direction, which makes κ a step function. Plan 5 exists to retire this.

## 2. Architecture options

Three concrete designs. Each is evaluated against the constraints:
- `calc_position_cb(sk, m, t) -> scalar` contract is intact (rewriting
  the itersolve solver is a different plan).
- Smooth-shaper closed-form integration still works (or we pay the cost
  to rewrite it).
- Linear moves from manual_stepper/force_move/extruder keep working.

### Option A — "Poisoned move": union tag inside `struct move`

Add a kind field; for blends, overload `start_pos / axes_r / start_v /
half_accel` to hold quintic metadata, or union in a coefficient blob.

```c
enum move_kind { MOVE_LINEAR = 0, MOVE_QUINTIC = 1 };

struct move {
    double print_time, move_t;
    double start_v, half_accel;       // still valid for LINEAR
    struct coord start_pos, axes_r;   // LINEAR: direction; QUINTIC: see below
    int kind;
    union {
        struct {
            // 6 x (x,y,z) control points in world coords
            double Q[6][3];
            // cached arc-length s(t) polynomial or Gauss-Legendre table
            double s_poly[N];
            double total_s;
            // v(s) -> v(t) conversion: store v(t) coefficients directly
            double v_poly[M];
        } quintic;
    } u;
    struct list_node node;
};
```

- **trapq changes**: `struct move` grows from ~7 doubles to ~7 + 21 (6 CPs)
  + 8-20 (s/v tables) = ~50 doubles. `trapq_append` stays; new
  `trapq_append_quintic(tq, print_time, Q[6][3], total_s, v_poly, ...)` is
  added.
- **Quintic coefficients**: shipped verbatim from Python. `QuinticShape`
  already has `Q`, `arc_length`, `_s_tab`, `_t_tab`, `v_cap_fn`. We'd build
  a `v(t)` polynomial from the planner's accel/cruise/decel fit along `s`
  — either ship `v(s)` sampled and interpolate, or keep the linear-v
  ramp in the move's `start_v/half_accel` fields (sliding those along s,
  not t) if the blend is short enough to treat as constant-v.
- **kin_*.c changes**: every `move_get_coord` call becomes
  `move_get_coord(m, t)` with branch:
  ```c
  if (m->kind == MOVE_QUINTIC) return quintic_eval(m, t);
  else                         return linear_eval(m, t);
  ```
  Since `move_get_coord` is `inline` in the header today (trapq.c:31-39 is
  declared inline in trapq.h via the `.c` file pattern — we'd need to
  move it to a non-inline function or expose a table). Actually it's
  `inline double move_get_coord` in the `.c`; every kin_*.c re-inlines
  trivially. We can keep that by putting the dispatch inside
  `move_get_coord` itself.
- **kin_shaper.c pulse path**: unchanged structurally (still sums scalar
  position samples).
- **kin_shaper.c smoother path**: **BREAKS** `integrate_move`
  (integrate.c:51-63). That routine assumes `x(t) = start_pos + axes_r ·
  (start_v·t + half_accel·t²)` and collapses the smoother convolution
  into `base·it0 - start_v·it1 + half_accel·it2`. A quintic position
  function is degree 5 in t; the convolution against the smoother
  (currently up to degree-12 polynomial in the smoother weight) still has a
  closed form, but needs a new `integrate_move_quintic` emitting, e.g.,
  `Σ c_i · it_i` for i = 0..5 — meaning we need 6 antiderivative columns
  rather than 3, both on the smoother side (`it0, it1, it2` → `it0..it5`)
  and on the move side (coefficients of x(t) as a polynomial).

  Net work: rewrite `smoother_antiderivatives` to carry 6 moments,
  rewrite `init_smoother`, rewrite `integrate_move` to be polynomial-degree
  parametric. Doable, but every use of the smoother (extruder PA + XY
  smooth shapers) rebuilds.

- **blendplanner.py handoff**: new method on `trapq` wrapper;
  `CornerBlender._emit_blend` emits a single quintic-trapq call plus the
  two truncated linear moves.
- **Linear back-compat**: pure-linear path is the `MOVE_LINEAR` branch,
  identical to today. One extra int field per struct; one predictable
  branch in `move_get_coord`.
- **Invasiveness**: trapq.{h,c} (struct grows), integrate.{h,c} (full
  rewrite of moment count), kin_shaper.c (trivial once integrate is
  rewritten), kin_extruder.c (pulled along via integrate.c), motion_report.c
  /.py (pull_move struct extension or a new extract_old variant), the
  Python FFI table (chelper/__init__.py:109-130). **~6 C files + 3 Py
  files**. Tests: any test that builds a `struct move` by hand breaks;
  simulator tools probably break too.

### Option B — "Split-queue": dedicated quintic queue beside trapq

Leave `trapq` strictly linear. Add a sibling `curveq` for quintic
primitives. The wrapper sk (like `input_shaper`) merges them at query time.

```c
struct curve_move {
    double print_time, move_t;
    double Q[6][3];
    double v_poly[M];
    double arc_length;
    struct list_node node;
};
struct curveq { struct list_head moves, history; };
```

- **trapq changes**: none.
- **Quintic coefficients**: stored in a parallel queue. `itersolve` needs
  to know which queue to read from at time t.
- **kin_*.c**: unchanged. The merge is done by a new wrapper sk, analogous
  to `input_shaper`, that looks up the right queue and forwards.
- **kin_shaper.c**: the shaper wraps the *merge* wrapper, not the kinematic
  directly, so it sees a unified `calc_position_cb(sk, m, t)` that may
  traverse either queue. The `struct move *m` argument becomes a problem:
  the shaper today uses `list_prev_entry(m, node)` to walk the trapq
  (kin_shaper.c:75-84, 134-158). If some of the "moves" are on `curveq`
  with different node offsets, the list walk breaks.
  
  Fix: unified intrusive list of a tagged superclass (pulls us back toward
  Option A) OR the merge wrapper pre-splices both lists at query time —
  costly.
- **Smoother integration**: same closed-form problem as Option A on the
  quintic segments. Same fix required.
- **blendplanner.py**: new `curveq_append_quintic` FFI entry.
- **Linear back-compat**: perfect.
- **Invasiveness**: new C module (curveq.{h,c}), new wrapper sk, new FFI
  entries, still have to rewrite integrate.c for smoother. Plus the
  list-walk problem makes shaper multi-move convolution awkward; either
  we give up on blends that span two trapq entries (blends are inside one
  move's window so maybe OK), or we bite the unified-list bullet.
  **~5 new C files + integrate.c rewrite**. Test disruption: simulator
  needs curveq awareness too.

### Option C — "Thin primitive, thick sampler": store only the polynomial, do generic polynomial integrate

Rather than discriminate quintic vs linear by tag, upgrade `struct move`
to **always** store a polynomial `x(t) = Σ c_i t^i` per axis, with degree
up to 5. Linear moves use degree ≤ 2 with higher coefficients zero.
Smoother integration becomes a generic polynomial·smoother convolution.

```c
#define MOVE_MAX_DEG 5
struct move {
    double print_time, move_t;
    // per-axis polynomial x(t) = Σ c[axis][i] * t^i, i=0..MOVE_MAX_DEG
    double c[3][MOVE_MAX_DEG + 1];
    struct list_node node;
};
```

- **trapq changes**: fundamental. `start_v`, `half_accel`, `start_pos`,
  `axes_r` all disappear; `move_get_distance` and `move_get_coord` become
  polynomial evaluations (Horner). Every `trapq_append` caller rebuilds
  the coefficient array from its accel/v/pos inputs (trivial: 6 coeffs
  for the trapezoid case, 3 non-zero).
- **Quintic coefficients**: trivial — convert the Bernstein basis of the
  Bezier to monomial basis once in Python or at `trapq_append_quintic`
  entry. `QuinticShape.Q` → monomial `c[axis][0..5]` is a fixed
  6x6 linear map. But t in `struct move` is time; the quintic is
  parameterised by its own s or its own internal parameter u. Two options:
  (a) re-parameterise the quintic by time via `u(t) = ∫ v(s)/|B'(u)| ...`
  — not polynomial in t, no closed form; 
  (b) restrict the blend to constant-parametric-speed: doesn't hold.
  
  **This is the fundamental problem with Option C**: t inside a move must
  be the same t the smoother integrates against. The quintic's natural
  parameter is arc length s, not time. Re-parameterising curve-by-time is
  not polynomial — it involves `ds/dt = v(s)` and an inverse integral.
  Only way to stay polynomial in t is to treat **the blend's v(t) as
  constant** (pure arc-length-linear through the curve at fixed speed).
  
  That's actually workable for the plan's first cut: Plan 4 already
  bounds `v` to a single `arc_cap_v` across the whole blend (blendplanner.
  py:210-213). If we enforce constant speed through the blend, `s(t) =
  v·(t - t_blend_start)` and the quintic's Bernstein → monomial map gives
  a clean degree-5 polynomial in t. Accel/decel through the blend
  becomes impossible though — the entry/exit must match the truncated-
  linear move speeds, requiring a more-than-5 degree polynomial or a
  separate v(s) profile.

- **kin_*.c**: all rewritten to do Horner over `c[axis]`. Cheap, uniform,
  eliminates the axes_r · distance indirection.
- **kin_shaper.c smoother**: rewrite `integrate_move` to handle degree 5
  polynomials. Antiderivative moments extend to `it0..it5`. Same core
  fix as Option A.
- **kin_shaper.c pulse**: trivial — still scalar sums, but each sample is
  now a Horner eval.
- **Linear back-compat**: every linear caller has to switch. `trapq_append`
  can stay as a compatibility shim that fills in `c[axis][0..2]`.
- **Invasiveness**: trapq.{h,c}, integrate.{h,c}, itersolve.c (no change,
  it's blind to move internals except through calc_position_cb), every
  `kin_*.c`, chelper FFI, motion_report.c/.py (pull_move struct is the
  external-facing linear quad — could be kept as a view). Python callers
  of `trapq_append` unchanged if shim stays; `trapq_extract_old` consumers
  either see a new degree-5 struct or we keep a linear-view for non-blend
  moves only (motion_report can dump blend moves differently).
  **~8 C files + shim work**. Test disruption: every unit test that reads
  `m->start_v` or `m->half_accel` breaks; consumer analysis needed.

### Option variations worth flagging (not fully elaborated)

- **Option A' — shared-polynomial fallback**: start with A, but when you
  have to rewrite `integrate.c` anyway, upgrade linear path to the same
  polynomial representation (i.e. A collapses into C). Worth considering
  as a staging order for a single design rather than as a separate
  option.
- **Option D — defer to solver rewrite**: replace itersolve's secant-
  method root finder with a solver that takes `x(s)`, `v(s)`, and
  generates steps in arc-length space. Massive scope; out of this memo.
  Note as a future direction only.

## 3. Recommendation

**Go with Option A (tagged union), with the explicit plan that the
smoother rewrite done in step 2 is forward-compatible with Option C's
general polynomial view.**

Rationale:

1. **Blast radius**: Option A keeps `trapq_append` and all linear
   consumers (force_move, manual_stepper, extruder, homing, trad_rack)
   untouched. That's worth a lot — those modules have no reason to
   learn about quintics, and keeping them out of the diff reduces
   hardware-regression risk.
2. **The smoother rewrite is unavoidable.** Every realistic option that
   preserves closed-form shaper integration requires extending
   `smoother_antiderivatives` from 3 moments to 6 moments and
   parametrising `integrate_move` by polynomial degree. Option A, B, and
   C all pay this cost. The bill is about the same regardless of which
   outer design we pick, so we shouldn't let it drive the choice.
3. **`calc_position_cb` contract stays scalar**, so every `kin_*.c`
   change is the same trivial dispatch (`move_get_coord` branches on
   `m->kind`). `kin_shaper.c` barely changes: pulse path is already
   scalar-sum, smoother path goes through the new polynomial
   `integrate_move`.
4. **Option B's list-walk hazard** (the shaper's `list_prev_entry`
   traversal across mixed queues) is real and ugly, and solving it by
   splicing both lists into a unified intrusive tagged list just
   recreates Option A with extra ceremony.
5. **Option C's re-parameterisation problem** is a hard blocker for
   anything richer than constant-speed-through-the-blend. We want Plan 5
   Pillar 1 (feedforward inverse shaper) to amplify HF content, which
   means we may want to vary speed through the blend to exploit
   shaper pre-/post-ringing budget. Keeping the quintic's natural
   arc-length parameterisation inside the tagged union (Option A) lets us
   feed `v(s)` independently of the curve geometry — cleaner math.

Concretely, the staged implementation order (to be laid out in the
actual plan doc, not here):

1. Extend `smoother_antiderivatives` to 6 moments; rewrite `init_smoother`
   / `integrate_move` / `integrate_velocity` to be polynomial-degree
   parametric up to degree 5. Hardware-validate that linear moves
   still match to numerical tolerance (regression gate).
2. Add `struct move::kind` + quintic union + `trapq_append_quintic` + a
   `move_get_coord` dispatch. Update every `kin_*.c`'s direct
   `move_get_coord` call sites to go through the dispatch (they already
   do via the inline). Hardware-validate that default (all-linear) path
   is byte-identical.
3. Update `blendplanner.py::_emit_blend` to emit a single quintic trapq
   entry plus truncated linear heads. Delete `shape.polyline` call.
   Hardware-validate corner-blending on a test rig.
4. Plan 5 Pillar 1 (feedforward inverse shaper) builds on top of the
   now-C²-continuous command stream.

### Items requiring another research round before Plan 5 starts

- **v(s) storage inside the quintic union**: do we ship a polynomial
  `v(t)`, or an arc-length table, or just a constant `v` per move?
  Impacts whether we can vary speed through the blend to exploit
  shaper pre-/post-ringing.
- **Coordinate system**: the planner builds Q in world XY; kin_delta /
  kin_corexy queries are in Cartesian. The union holds world-space
  control points, the `kin_*.c` dispatch does `quintic_eval` into a
  `struct coord`, then the per-kinematic formula. This works for
  Cartesian/CoreXY/delta; verify for polar (`kin_polar.c`) and
  rotary_delta before committing. Not checked in this memo.
- **motion_report / DumpTrapQ**: decide whether blended moves appear
  as linear (via a fit) or get a new schema. Affects the dump-trapq
  frontend and any offline analysis tools.
- **klipper-sim compatibility**: ~/Developer/klipper-sim/ simulates
  the current linear trapq. Needs a parallel update; won't be
  upstream until kin_shaper integrate.c is shippable.

## 4. Known risks and unknowns

- **Moment arithmetic numerical stability**: extending to `it5` means
  6th-power-of-time numerics on smoother support ~10 ms. `t^5 ≈
  1e-10` in SI; with smoother coefficients and normalisation, this
  should be fine but needs numerical validation (condition number
  of the basis conversion). Unverified.
- **Pressure advance consistency**: kin_extruder.c uses
  `integrate_velocity` (integrate.c:65-74) which assumes linear v.
  If the extruder is coupled to XY position via `axes_r` during a
  blend, PA's velocity model has to be recomputed on the quintic
  too. Section 3's staging does the smoother extension first, so
  this falls out. But PA's coupling to blend is itself a Plan 5
  open question.
- **Input-shaper pulse path across blend boundaries**: the pulse
  shaper samples `x(t + τ_i)` across multiple moves via list walk
  (kin_shaper.c:72-84). When one of those moves is a quintic, the
  scalar sample is well-defined (Horner over monomial basis once
  converted), but the walk must traverse tagged moves. Provided
  we route everything through `move_get_coord`, the walk code is
  agnostic. Verified in principle; not exercised in this memo.
- **`trapq_set_position` interaction with quintic history moves**:
  history replay (trapq.c:203-228) reconstructs pose from
  `pull_move` fields. Blend moves produce non-linear history;
  either we keep blend entries linear-ish in history (fitting) or
  extend `pull_move`. Open.
- **Test/harness disruption**: the simulator and any unit tests
  touching `struct move` directly need updating. Scope not
  measured here.
- **Invasiveness score (rough)**:
  - Option A: medium-low. ~6 C files, ~3 Py files, integrate.c is
    the biggest single change.
  - Option B: medium-high (curveq + list-walk hazard).
  - Option C: high (all kin_*.c + all linear trapq callers).

Nothing in this memo is hardware-validated. Every claim above is from
reading the tree as of 2026-04-22 on `magnum-opus`. Section 3's
"recommended" Option A path includes two hardware-validation gates
(steps 1 and 2) before blending changes.
