# REVIEW 3 — C integration audit of Plan 5 spec

**Date:** 2026-04-22
**Reviewer scope:** ground-truth every file:line claim in
`docs/superpowers/specs/2026-04-22-plan5-direct-quintic-pillar1-design.md`
against the actual source on branch `magnum-opus`. No theory, just C.

## Verdict: **ship with fixes** (major-ish)

The spec's tagged-union direction is sound and the "other 7 kin_*.c
files are fine" observation is mostly correct. But the spec undercounts
the direct-access footprint in two load-bearing files outside the "3
files" list (`itersolve.c`, `trapq.c`'s own bookkeeping paths) and
misunderstands the motion_report / klipper-sim external interface in
both directions (motion_report's Python side is safer than the spec
thinks; klipper-sim has no C-side deserializer at all). The FFI change
for `input_shaper_set_smoother_params` and the `_get_smoother_sigma2`
Python path are also understated. Fixable without re-architecting,
but D2's 6-8 day estimate is optimistic by ~30-50%.

## Files I actually grepped / read

- `klippy/chelper/trapq.h:1-53` — current `struct move`, `struct coord`,
  `struct pull_move`.
- `klippy/chelper/trapq.c:1-256` — move_alloc, move_get_coord (inline,
  :31-39), move_get_distance (inline, :24-28), trapq_add_move,
  trapq_append, trapq_finalize_moves (:168-199), trapq_set_position,
  **trapq_extract_old (:232-256)**.
- `klippy/chelper/integrate.h:1-30` — `smoother_antiderivatives` typedef
  (`:4-6`, 3 fields), `struct smoother` (`:8-13`, flat `c0/c1/c2[12]`).
- `klippy/chelper/integrate.c:1-113` — `calc_antiderivatives`
  (Horner-form, :23-39), `integrate_move` (:51-63), `integrate_velocity`
  (:65-74), `init_smoother` (:80-113).
- `klippy/chelper/kin_shaper.c:1-347` — `get_axis_position` (:63-70,
  direct `m->axes_r`, `m->start_pos`), `range_integrate` (:105-160),
  FFI `input_shaper_set_smoother_params` (:314-330),
  `shaper_note_generation_time` (:267-293).
- `klippy/chelper/kin_extruder.c:1-316` — `pa_move_integrate` (:38-49,
  direct `m->axes_r.x`/`.y`), `extruder_calc_position` (:184-226,
  direct `m->start_pos.axis[i]`, `m->axes_r.axis[i]`).
- `klippy/chelper/itersolve.c:1-281` — `itersolve_gen_steps_range`
  (:28-128), `check_active` (:136-143, **direct `m->axes_r.[xyz]`**),
  `itersolve_calc_position_from_coord` (:256-267, synthesizes a
  `struct move` via memset).
- `klippy/chelper/itersolve.h:1-41` — `sk_calc_callback` typedef.
- `klippy/chelper/__init__.py:93-198` — FFI cdef for trapq/itersolve/
  kin_extruder/kin_shaper.
- `klippy/extras/motion_report.py:100-200` — `DumpTrapQ.extract_trapq`
  uses `struct pull_move`, not `struct move`.
- `klippy/extras/input_shaper.py:400-465` — `update_stepper_kinematics`,
  `update_extruder_kinematics` FFI call sites.
- `klippy/extras/shaper_calibrate.py:422-449, 577-617` —
  `_get_smoother_sigma2`, `find_smoother_max_accel`.
- `klippy/extras/shaper_defs.py:208-221` — `INPUT_SHAPERS` (zv, mzv
  only), `INPUT_SMOOTHERS` (6 smooth_*).
- `~/Developer/klipper-sim/klipsim/*.py`, `README.md`, `STATUS.md` —
  confirmed no C-side trapq deserializer.

All grep patterns `m->(axes_r|start_v|half_accel|start_pos)` enumerated
in §V2 below.

## Verified / refuted claims

### V1. `struct move` layout (D2b) — mostly correct, two misses

Current layout (`trapq.h:15-21`):
```c
struct move {
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;
    struct list_node node;
};
```

The spec's proposed tagged union is compilable and reasonable. One
ABI concern: the spec keeps `print_time`, `move_t`, `start_pos` at the
same offsets at the top of the struct, which is compatible with
`list_prev_entry(m, node)` traversal and with `trapq.c:52`
(`tail_sentinel->print_time = NEVER_TIME`). Good.

**But `enum move_kind` between `move_t` and `start_pos` will shift
offsets.** The spec places `enum move_kind kind` on line 416 of the
struct definition. C `enum` default-promotes to `int` (4 bytes). Put
after the two `double`s, it'll get 4 bytes of padding either side,
so `start_pos.x` shifts 8 bytes later. That's fine AS LONG AS nothing
outside the struct computes `offsetof(struct move, start_pos)` — let
me check.

```
$ grep -rn "offsetof(struct move" klippy/chelper/
(no matches)
```

No `offsetof(struct move, ...)` outside includes. Safe.

However, **`sizeof(struct move)` matters**: `move_alloc()` uses
`malloc(sizeof(*m))` at `trapq.c:18` and the union bloats sizeof to
~840 B for quintic. Every linear null move (the `MAX_NULL_MOVE`
fill-gap moves at `trapq.c:103-113`) would also pay this cost. Set the
`kind = MOVE_LINEAR` explicitly on the null move and make sure the
zero-init (`memset(m, 0, sizeof(*m))`) leaves `kind = MOVE_LINEAR = 0`
— **that's a hard constraint: `MOVE_LINEAR` must be the zero value**
because `move_alloc()` memsets to zero and does NOT touch `kind`.

**Missed site 1: `trapq.c:91`** — `tail_sentinel->start_pos =
move_get_coord(m, m->move_t)`. Fine through `move_get_coord` with
dispatch. No fix needed as long as `move_get_coord` dispatches.

**Missed site 2: `trapq.c:183`** — `if (m->start_v || m->half_accel)`.
Used to detect non-null linear moves for history retention. With
tagged union this becomes an `m->u.lin.start_v || m->u.lin.half_accel`
for linear kind, always true for quintic. Needs a `move_is_null()`
helper dispatched on kind.

**Missed site 3: `trapq.c:244-251`** — `trapq_extract_old` reads
`m->start_v`, `m->half_accel`, `m->start_pos.x/y/z`, `m->axes_r.x/y/z`
directly to project onto `struct pull_move`. For quintic moves, this
projection is **not representable**: `pull_move` is a linear-only
schema. Options:
  - Sample the quintic at `pull_move` projection time (N samples per
    quintic — blows up the `max` parameter contract).
  - Add a second projection API `trapq_extract_old_v2` returning
    `struct pull_move_v2` with a kind tag.
  - Projection-by-approximation: emit one `pull_move` per phase with
    a linear fit. Drops curvature for motion_report but stays schema-
    compatible.

The spec's paragraph on motion_report (§D2c "motion_report schema")
assumes external consumers see a struct-move-shaped payload on the
wire. **They do not.** Python-side `motion_report.py:170-179` reads
flat `pull_move` fields. The schema on the wire is already flat and
trivially extensible. But the **C-side projection function has to
dispatch on kind**, which is a real code change in `trapq_extract_old`
missed by the spec.

### V2. "Only 3 C files need changes" — **refuted**

Spec claim: `trapq.c`, `kin_shaper.c`, `kin_extruder.c` are the only
files that directly access `m->axes_r` / `m->start_v` / `m->half_accel`.

Actual grep (`m->(axes_r|start_v|half_accel|start_pos)` under
`klippy/chelper/`):

| file | site | notes |
|---|---|---|
| `trapq.c:27` | `move_get_distance` | core dispatch target |
| `trapq.c:36-38` | `move_get_coord` | core dispatch target |
| `trapq.c:105,132-135,145-148,158-161,183,224-226,244-251` | writers + extract | multiple paths |
| `integrate.c:55-57, 69-71` | `integrate_move`, `integrate_velocity` | spec knew this (D2a) |
| `kin_shaper.c:66-67, 122, 130, 156, 221` | `get_axis_position`, `range_integrate`, `shaper_xy_calc_position` | spec knew this |
| `kin_extruder.c:44, 208` | `pa_move_integrate`, `extruder_calc_position` | spec knew this |
| **`itersolve.c:140-142`** | **`check_active`** | **MISSED by spec** |

`itersolve.c::check_active` at `:136-143`:
```c
static inline int
check_active(struct stepper_kinematics *sk, struct move *m)
{
    int af = sk->active_flags;
    return ((af & AF_X && m->axes_r.x != 0.)
            || (af & AF_Y && m->axes_r.y != 0.)
            || (af & AF_Z && m->axes_r.z != 0.));
}
```

This is called for **every move** by `itersolve_generate_steps`
(`:161, 225`) — the main step-gen loop. For a quintic move,
`m->u.lin.axes_r` is simply uninitialized garbage. **This will miscompile
silently and produce either spurious activity or false inactivity.**

Must add quintic dispatch: for `MOVE_QUINTIC_POLY_T`, derive axis
activity from the per-axis polynomial coefficients (non-zero if any
`c[k]` per axis is non-zero). Trivial conceptually, ~6 lines, but this
is a **step-gen inner-loop function** (inlined) — the kind branch
touches hot code.

`itersolve_calc_position_from_coord` at `itersolve.c:256-267`
synthesizes a `struct move` via `memset` with only `start_pos` set.
Under tagged union this **must** explicitly set `m.kind = MOVE_LINEAR`
(it will get zero from memset, so if `MOVE_LINEAR = 0` we're fine —
reinforces the "MOVE_LINEAR must be the zero enumerator" constraint
above). This is a real landmine — one wrong enum ordering later and
`itersolve_set_position` silently produces garbage.

**Python ctypes access:** none. Python decodes trapq moves only via
`pull_move` (a separate projection struct). `klippy/chelper/__init__.py`
does not cdef `struct move` itself — only `struct pull_move`,
`struct trapq`, `struct stepper_kinematics`. Safe.

**Count correction:** the spec's "3 files" is actually **5 files**
(`trapq.c`, `integrate.c`, `kin_shaper.c`, `kin_extruder.c`,
`itersolve.c`) plus the motion_report C-side projection
(`trapq_extract_old`) which lives in `trapq.c` already. Call it
**5 files with direct-access touch points** and **7 kin_*.c files
genuinely untouched**.

The spec's "zero changes to `kin_cartesian`, `kin_corexy`, …"
**is verified**: all of those call `move_get_coord` exclusively
(`kin_cartesian.c:18,25,32`; `kin_corexy.c:17,25`; `kin_corexz.c:17,25`;
`kin_delta.c:25`; `kin_deltesian.c:26`; `kin_idex.c:29`;
`kin_polar.c:18,26`; `kin_rotary_delta.c:47`; `kin_winch.c:25`).

### V3. 11-moment extension (D2a) — feasible, spec mislabels the struct

Spec says "`struct calc_antiderivatives` at `klippy/chelper/integrate.h:8-13`
has 3 fields." **The name is wrong.** Actual struct is
`smoother_antiderivatives` (a typedef, not a `struct`, at lines
`:4-6`), and it is the `struct smoother` that lives at lines `:8-13`.
Two different types. Fix the label.

The 3 fields are `it0, it1, it2` (`integrate.h:5`). Extending to 11
for `(m_0 … m_10)` is straightforward — it's just more parallel
accumulators in `calc_antiderivatives` (`integrate.c:23-39`).

**LOC estimate for `integrate.c` rewrite:**
- `calc_antiderivatives`: current 17 LOC. For 11 fields with the same
  Horner pattern: ~30-40 LOC (more accumulators), or better, a macro/
  loop. Call it 35 LOC.
- `integrate_move`: current 13 LOC, one `axis_r` scalar * (start_v,
  half_accel). For quintic with 11 per-axis coefficients, needs to
  evaluate `Σ_{k=0..10} c_k · s->m_k` where `c_k` is the axis-
  specific coefficient. Without `axes_r` scaling (the quintic stores
  absolute per-axis coefficients, not a scalar times `axes_r`), the
  inner becomes `m->u.quintic.accel.c[k].axis[axis-'x'] * s->m[k]`
  summed. ~40 LOC with phase dispatch.
- Phase dispatch: **this is the nontrivial part.** See below.

Phase dispatch at the integration window boundary: given sample-time
`move_time`, kernel half-support `hst`, the window `[move_time-hst,
move_time+hst]` may straddle `t_accel_end` or `t_decel_start`. The
correct algorithm:

```
t_lo = max(move_time - hst, 0)
t_hi = min(move_time + hst, move_t)
boundaries = [t_accel_end, t_decel_start] ∩ (t_lo, t_hi)
pieces = [t_lo, boundaries..., t_hi]
accumulate integral piece-wise, swapping c[] between accel/cruise/decel
```

For the kernel-polynomial side, `range_integrate` at
`kin_shaper.c:105-160` already handles "window spans move boundaries"
with `diff_antiderivatives`. Phase dispatch is the same pattern one
level deeper: split on phase boundaries, call `calc_antiderivatives`
at each split point, use `diff_antiderivatives` per piece. Tractable
but adds ~20-30 LOC and at most 2 extra `calc_antiderivatives` calls
per sample (once per phase boundary crossed).

Per-sample cost: spec says ~3.5× per-sample cost for quintic.
Realistically: `calc_antiderivatives` currently does 3*n multiplies
(n ≤ 12). For 11-moment version it's 11*n — that's **~3.7×**
`calc_antiderivatives`. Plus the phase dispatch potentially doubles
the number of `calc_antiderivatives` calls per sample. Combined with
D3's fused kernel (3× support width = 3× piece crossings), total
worst-case is closer to **~12× per-sample on quintic** vs linear
today. Spec's "~10×" estimate is in the right ballpark but the
breakdown undercounts the phase-dispatch extra `calc_antiderivatives`
calls.

**Other consumers of `smoother_antiderivatives`:**
```
$ grep -rn "smoother_antiderivatives" klippy/chelper/
integrate.c  — definitions and users
integrate.h  — typedef and function signatures
kin_shaper.c — range_integrate uses diff_antiderivatives
kin_extruder.c — same
```

No external Python code cdefs `smoother_antiderivatives`; it's an
internal C type. Extending it from 3 to 11 fields requires a rebuild
of `c_helper.so` but is otherwise ABI-internal. **Safe.**

### V4. Piecewise `struct smoother` (D1) — **FFI break is material**

Current `struct smoother` at `integrate.h:8-13`:
```c
struct smoother {
    double c0[12], c1[12], c2[12];
    double hst, t_offs;
    smoother_antiderivatives m_hst, p_hst, pm_diff;
    int n, symm;
};
```

Three precomputed coefficient arrays of degree ≤ 11 (size 12).
Spec's proposed `struct smoother_piece { double coeffs[6]; double
t_start, t_end; }` with up to 6 pieces is reasonable. With per-piece
precomputed `m_hst`-equivalent antiderivatives, the struct grows from
~400 B to maybe 1200-1500 B (6 pieces × (6 coeffs × 3 moments × 11
moment-count + 2 time bounds + precomputed antiderivatives)). Cache
footprint grows ~3-4×.

**FFI signature at `kin_shaper.c:314-330`:**
```c
int __visible
input_shaper_set_smoother_params(struct stepper_kinematics *sk, char axis
                                 , int n, double a[], double t_sm)
```

Called from Python at `input_shaper.py:421-430`:
```python
ffi_lib.input_shaper_set_smoother_params(
    sk, axis, self.n, self.coeffs, self.smooth_time
)
```

Where `self.coeffs` is a flat Python list of coefficients.

The spec's piecewise-form signature change
`input_shaper_set_smoother_params(sk, axis, n_pieces, piece_descriptors[], t_sm)`
is materially different ABI. Need to think about what `piece_descriptors`
looks like over FFI — a flat `double[]` with a known layout
(`[start_0, end_0, c0_0..c5_0, start_1, end_1, c0_1..c5_1, ...]`)
is simplest. cdef update required in `klippy/chelper/__init__.py:189-198`.

**Also affected (missed by spec): `extruder_set_smoothing_params`**
at `kin_extruder.c:285-297`, called from
`input_shaper.py:441-444`. Has the same `double a[]` signature and
needs the same piecewise extension. Spec mentions extruder smoothing
in D3 but the D1 §"FFI signature change" paragraph only calls out
`input_shaper_set_smoother_params`. Minor oversight.

**`ShaperCalibrate.find_smoother_max_accel` and `_get_smoother_sigma2`**
at `shaper_calibrate.py:422-449, 601-617`. Current algorithm:
```python
def _get_smoother_sigma2(self, smoother):
    C, t_sm = smoother   # C is a flat polynomial coefficient list
    hst = 0.5 * t_sm
    def raw_moment(k):
        s = 0.0
        for i, c in enumerate(C):
            if (i + k) % 2 == 0:
                s += c * 2.0 * hst ** (i + k + 1) / (i + k + 1)
        return s
    M0 = raw_moment(0); ts = raw_moment(1) / M0
    return raw_moment(2) / M0 - ts * ts
```

This assumes a single flat polynomial over `[-hst, hst]` with the
**parity trick** (odd powers vanish over the symmetric interval). For
piecewise kernels, the parity trick breaks **per-piece** but holds
for the full support. The fix is to sum `raw_moment` over pieces
with full numerical integration (no parity shortcut per piece). ~15
LOC change, careful because `_get_smoother_sigma2` is the closed-form
root of `find_smoother_max_accel`'s `A_crit = 2 * target / sigma^2`
(`shaper_calibrate.py:617`). Getting this wrong silently corrupts
A_axis caps.

Spec's claim "works against the new family via the same
polynomial-moment code path as before, modulo the piecewise extension"
is technically correct but understates: the "same code path" needs
per-piece iteration and the parity simplification is no longer valid
per-piece. Real LOC closer to 40 than 10. Not catastrophic.

### V5. Fused kernel `k_fused = h ⊛ w` — storage fits, degree explodes

For bs3: forward kernel `h` has 4 pieces of degree 3. Per
`new_shaper_family.md` (checked out of scope for this review, but
spec §D3 quotes it), the FIR inverse `w` is windowed with support
`T_h = 2·T_sm` and a cosine taper. Convolution:

- **Number of pieces:** convolution of two piecewise polynomials
  with `p` and `q` pieces produces **up to `p+q-1` pieces** (each
  interior breakpoint of either input contributes). For bs5 (6
  pieces) convolved with an FIR inverse of say 6 pieces, k_fused
  has **≤ 11 pieces**. This **does not fit** in the spec's "up to 6
  pieces" piecewise smoother.

- **Degree per piece:** conv of degree-m1 + degree-m2 poly → degree
  `m1 + m2 + 1`. For bs5 (degree 5) × FIR inverse (degree ≤ 5?),
  k_fused has degree ≤ **11** per piece. So the spec's `double
  coeffs[6]` (degree-5) **does not fit** — needs `double coeffs[12]`
  (degree-11).

This is a **real blocker**: the proposed struct doesn't hold the
k_fused the spec itself calls out as the storage target. Either:
  (a) Sample k_fused and refit at lower degree per piece (degrade to
      tractable storage, accept approximation error).
  (b) Grow the piecewise struct to ≤ 12 pieces × 12 coeffs = 144
      doubles per kernel.
  (c) Keep forward and inverse separate in C, compose at sample time
      (loses the "one convolution" D3 optimization).

Spec §D1 line 298: `struct smoother_piece { double coeffs[6]; … }`
and "up to 6 pieces." Spec §D3 lines 569-571: "for bs_m with m+1
pieces convolved with FIR inverse of similar piece count, the fused
has ≤ 2(m+1) pieces." For bs5 that's 12 pieces — already double the
spec's 6-piece cap. **Internal spec inconsistency.**

**Convolution-of-piecewise-polynomials at shaper-reset:** tractable
in Python via numpy or sympy, with care for piece boundaries. ~80-150
LOC of Python + thorough tests. Don't need C-side implementation at
config time. The real work is in the storage format + C evaluation
of a piecewise kernel with up to 12 pieces and degree-11 per piece.

### V6. `kin_extruder.c` extruder smoothing — spec verified, one nuance

Spec claim: `extruder_calc_position` applies `smoother_calc_position`
to XY axes. **Verified.** `kin_extruder.c:201-219`:

```c
for (i = 0; i < 3; ++i) {
    int axis = 'x' + i;
    const struct smoother* sm = &es->sm[i];
    ...
    if (num_pulses) {
        shaper_pa_range_integrate(m, axis, move_time, sp, sm, &pa_vel.axis[i]);
    } else {
        pa_range_integrate(m, axis, move_time, sm, &pa_vel.axis[i]);
    }
}
```

The extruder holds **its own** `struct smoother sm[3]` (one per XYZ
axis — line `:147`). So it is a **separate** smoother instance from
the XY input-shaper smoother. This is relevant for D3's "shared
`k_fused` across all axes" claim — actually there are TWO sets of
smoothers in play (input_shaper's `struct input_shaper::sm_x/sm_y`
at `kin_shaper.c:181` and extruder's `struct extruder_stepper::sm[3]`
at `kin_extruder.c:147`). Both must be updated.

`extruder_set_smoothing_params` at `:285-297` takes its own smoother
config via `init_smoother(n, a, t_sm, sm)`. So the fused-kernel pre-
computation must run once and then push identical piecewise coefs to
both input_shaper and extruder smoother slots.

**Extruder does NOT read XY's already-inverted position.** Line `:208`:
`e_pos.axis[i] = ... : m->start_pos.axis[i] + m->axes_r.axis[i] * move_dist`.
It reads raw `m->start_pos` + `m->axes_r * distance` — the planned
(unshaped) trajectory. Applies its own smoother on top. So yes, E-axis
PA is built on planned XY, not shaped XY. **Spec D3 claim verified:
XY inverse and E inverse need to be applied independently**.

### V7. Inlining discipline — `move_get_coord` is `inline` today

Actual: `trapq.c:31` — `inline struct coord move_get_coord(...)`.
Also declared extern in `trapq.h:36`. So it's both inline-in-
translation-unit and callable from other TUs. Other TUs calling it
pay a function-call cost today (kin_cartesian/corexy/delta/… all
include `trapq.h` and call `move_get_coord` — those are genuine calls,
not inlined).

**Call rate estimate.** `itersolve_gen_steps_range` at `itersolve.c:28-128`
runs a secant-method loop. Line `:72`: `guess.position =
calc_position_cb(sk, m, next_time);`. Loop iterates until bracket
converges to half-step. Typical ~3-10 iterations per step, so for a
100 mm/s move at 20 steps/mm = 2000 steps/s, that's 6000-20000
`calc_position_cb` calls/sec per active stepper. With N steppers
(XY + 2xZ + 4xE = ~7 steppers), ~100k calls/sec toolhead-wide.

Under quintic the per-call cost with fused kernel goes ~10×. At 100k
calls/sec × 10× = 1M equivalent calls/sec. On a BTT Pi/Trident SoC
that's ~50-80% of a core at current efficiency. This is **the risk
3 of the spec verbatim**, and it's real.

Spec's "benchmark before implementation" is the right call. Fallback
— a linear fast-path branch in `move_get_coord` preserving today's
3-FMA cost for `kind == MOVE_LINEAR` — is trivial to write and
essentially free for the linear path (one predictable branch).

### V8. klipper-sim — spec is **wrong about the nature of the change**

Spec §D2c lines 523-528: "The batch-sim harness at `~/Developer/
klipper-sim/` reads `trapq` state. D2b tagged-union change breaks its
deserializer."

**Refuted.** `~/Developer/klipper-sim/` exists, has 59 tests, but
does **not deserialize C trapq**. Grep for `trapq_extract_old`,
`cdef.*struct move`, `ffi.*trapq` under the repo: zero hits.

The actual architecture of klipper-sim:
- `klipsim/driver.py`, `sampler.py`, `toolhead_shim.py` — uses a
  pure-Python `Move` object (the Klipper Python planner's `Move`,
  imported from the kalico source tree via `--klipper-root`).
- `sampler.py:41-128` — samples by reading Move's Python attributes
  `start_v`, `cruise_v`, `axes_r`, etc. No C struct crossing.
- CSV writer (`csv_writer.py`) dumps Python Move fields directly.

**What actually breaks klipper-sim:** if `CornerBlender._emit_blend`
stops producing polyline Moves and starts producing quintic Moves,
the Python-side Move class needs quintic-aware attributes, and
`klipsim/sampler.py` needs to sample from the quintic polynomial
instead of the linear `start_v + a·t` form. That's a Python-side
update in klipper-sim, not a C deserializer update.

Implication: the spec's "klipper-sim deserializer update in the
same batch" deliverable is misnamed. It's a **Python-side simulator
update** tracking the `CornerBlender` emit format. Effort ~same or
maybe less (no CFFI work).

### V9. motion_report schema — Python side does NOT see `struct move`

Spec §D2c line 531-535: "motion_report emits trapq-move structures
via websocket; any external consumer (Mainsail, Fluidd, Moonraker)
that parses these will see a schema change."

**Partially refuted.** The wire format is already a flat tuple:
`motion_report.py:184-193`:
```python
d = [
    (m.print_time, m.move_t, m.start_v, m.accel,
     (m.start_x, m.start_y, m.start_z),
     (m.x_r, m.y_r, m.z_r))
    for m in data
]
```

Where `m` is a `struct pull_move`, NOT `struct move`. So:
- External websocket consumers see (print_time, move_t, start_v,
  accel, start_pos, direction) — a trapezoid flat tuple.
- For quintic moves, the C-side `trapq_extract_old` must project
  somehow into this flat schema — **cannot represent curvature
  honestly**. Either approximate with per-phase linear equivalents
  (3 `pull_move` entries per quintic) or extend the wire schema.

The spec's "emit a `version: 2` field" suggestion is right in spirit,
but the actual change is:
  1. Add a kind-dispatched C projection in `trapq_extract_old`.
  2. Either (a) multi-entry output per quintic (may overflow
     `max=128` buffer at `motion_report.py:127`) or (b) new
     `pull_move_v2` struct + new FFI entry point.
  3. Python side adds a `kind` column and consumers guard.

**This is more invasive than the spec's one-liner paragraph implies.**
Closer to 1-2 days of work, not an afternoon.

### V10. Effort reality-check

Spec D2: **6-8 days**. My breakdown:

| sub-task | spec implicit | real |
|---|---|---|
| struct move tagged union + dispatch | 1-2 d | 1.5-2 d |
| `integrate_move` 11-moment + phase dispatch | 2 d | 3-4 d (phase dispatch interacts with `range_integrate`'s existing split) |
| `kin_shaper.c` review + refactor | 1 d | 1-1.5 d (`is->m` synthetic move pattern at :221 needs care) |
| `kin_extruder.c` review + refactor | 1 d | 1-1.5 d (more direct access than spec knew) |
| **`itersolve.c` check_active + from_coord fix** | **0** (missed) | **0.5-1 d** |
| **`trapq.c:183, 244-256` kind dispatch** | **0** (missed) | **0.5 d** |
| blendplanner emit quintic | 0.5 d | 1 d |
| klipper-sim Python update (NOT deserializer) | 0.5 d | 0.5-1 d |
| regression: linear bit-identical | 0.5 d | 1 d (golden-dataset capture + diff harness) |
| regression: quintic round-trip | 0.5 d | 1 d |
| motion_report projection dispatch | (missing) | 0.5-1 d |

**Real total: 10-14 days.** Spec's 6-8 is ~40-60% under. Not
catastrophic — still a single-sprint deliverable — but if you staff
it for 8 days you will ship broken `itersolve::check_active` behavior
on quintic moves.

## Blockers found

### Critical

1. **`itersolve.c::check_active` dispatch missing.** Silent
   miscompile on quintic moves (step-gen will misjudge activity,
   skip or duplicate step generation). Spec §D2b line 474-487 lists
   dispatch points; `itersolve.c` is not on the list. ADD IT.
   Fix: 6 LOC plus a `move_has_axis_activity(m, axis)` helper in
   `trapq.c`.

2. **Fused-kernel piecewise storage insufficient.** Spec §D1
   `coeffs[6]` and "up to 6 pieces" vs spec §D3's own math showing
   k_fused needs ≤ 12 pieces at degree ≤ 11 for bs5 × FIR inverse.
   Internal contradiction. Must resolve before implementation.
   Fix: bump to 12 pieces × degree-11 coeffs, OR redesign to
   compose at sample time (losing the D3 optimization).

3. **`MOVE_LINEAR = 0` invariant not called out.** `move_alloc`
   memsets to zero; `itersolve_calc_position_from_coord` memsets
   to zero; `trapq_alloc`'s sentinels are zero'd. All of these
   implicitly create `MOVE_LINEAR` moves. If any future reorder
   of the enum puts `MOVE_LINEAR` at a nonzero value, silent
   corruption everywhere. Spec must pin this invariant.

### Important

4. **FFI signature break for `extruder_set_smoothing_params`**
   missed. Spec §D1 FFI paragraph only lists
   `input_shaper_set_smoother_params`. Extruder needs the same
   update (`kin_extruder.c:285-297`, cdef at
   `__init__.py:184-185`).

5. **`_get_smoother_sigma2` parity trick breaks per-piece.**
   `shaper_calibrate.py:422-441`. The closed-form
   `find_smoother_max_accel` (A_axis cap) depends on this
   function; silently wrong A_axis is a regression that doesn't
   fail any test but produces bad max-accel. Spec says "same
   polynomial-moment code path as before" — not quite true per-
   piece.

6. **`trapq_extract_old` projection for quintic moves.** Must
   decide the wire format: 3× `pull_move` entries per quintic, or
   a new `pull_move_v2` schema with kind tag. Spec D2c sketch is
   vague. External tool compatibility hinges on this.

7. **`trapq.c:183` null-move detection via `start_v || half_accel`.**
   Needs `move_is_null()` dispatched on kind. 3-line fix but easy
   to miss.

### Minor

8. **Spec mislabels `smoother_antiderivatives` as `struct
   calc_antiderivatives`.** `integrate.h:4-6` — it's a typedef, not
   a struct tag with that name.

9. **klipper-sim "deserializer" is misnomer.** No C-side binding.
   Update is Python-side (Move-class attributes) only.

10. **`sizeof(struct move)` null-move bloat.** Tagged union grows
    every allocated move to ~840 B including the null-fill moves at
    `trapq.c:103-113`. Consider an out-of-band `struct move_quintic_ext`
    pointer-to-big-thing if cache pressure becomes measurable.

## Implementation guidance

**Deliverable D1 (piecewise smoother):**
- `struct smoother_piece { double coeffs[12]; double t_start, t_end;
  smoother_antiderivatives m_start, m_end, m_diff; };` — 12 coeffs to
  handle the fused-kernel degree-11 case from V5.
- Cap the piece count at `N_PIECES_MAX = 12` (bs5 × FIR inverse upper
  bound). `struct smoother { struct smoother_piece pieces[N_PIECES_MAX];
  int n_pieces; double hst, t_offs; int symm; };` — loses the
  precomputed `p_hst`, `m_hst`, `pm_diff` shortcut per whole-kernel;
  must recompute per piece.
- FFI: flatten per-piece layout into a contiguous
  `double piece_buf[N_PIECES_MAX * (12 + 2)]`, passed as a single
  `double[]` with separate `int n_pieces` argument.
- Update *both* `input_shaper_set_smoother_params` (`kin_shaper.c:314`)
  and `extruder_set_smoothing_params` (`kin_extruder.c:285`).
- Update Python `_get_smoother_sigma2` to iterate pieces (no parity
  shortcut per piece; parity may still apply globally if the kernel
  is even-symmetric).

**Deliverable D2 (tagged union):**
- Place `enum move_kind kind` AFTER the two `double` fields (no
  padding) and BEFORE the union. `MOVE_LINEAR = 0` pinned.
- Add `move_has_axis_activity(m, axis)` helper in `trapq.c`; use in
  `itersolve.c::check_active`.
- Add `move_is_null(m)` helper in `trapq.c`; use in
  `trapq.c:183`.
- `trapq_extract_old`: for `MOVE_QUINTIC_POLY_T`, emit 3
  `pull_move` entries (one per phase) with linear-fit parameters.
  This preserves the `pull_move` schema at a modest accuracy loss
  for motion_report visualization.
- `move_get_coord` / `move_get_distance`: add fast path
  `if (likely(m->kind == MOVE_LINEAR)) { existing body }`. The
  compiler will hoist the common case. For quintic, evaluate the
  per-phase polynomial via Horner.
- `itersolve_calc_position_from_coord`: after `memset`, explicitly
  `m.kind = MOVE_LINEAR;` — defensive even though it's zero today.

**Deliverable D3 (fused kernel):**
- Python computes k_fused via numpy.polynomial per-piece
  convolution. Piece count up to 12, degree up to 11. Test the
  convolution routine against a 1D numerical reference (sample and
  integrate).
- C side just consumes the piecewise coefficients via the D1
  piecewise smoother struct. No C-side convolution code.
- Shared k_fused: input_shaper's `sm_x`/`sm_y` and extruder's
  `sm[0..2]` get the same piecewise data via separate FFI calls.
  No need to share the struct instance.

**Deliverable D5 (lookahead):**
- `shaper_note_generation_time` at `kin_shaper.c:267-293` already
  computes `pre_active = sm->hst + sm->t_offs` — under piecewise,
  `sm->hst` is still the global half-support (T_fused/2), so this
  math generalizes. Verify numerically.

**Benchmarks to run BEFORE landing D2:**
- Linear-only moves: compare `calc_position_cb` throughput with and
  without the `kind` branch in `move_get_coord`. Expect <5% overhead
  if branch-predicted. If >5%, consider a function-pointer table
  indexed by `kind`.
- Quintic-only moves: measure `calc_position_cb` with the 11-moment
  integrator + fused piecewise kernel. Target <15 us per call on
  Trident SoC. If above, look at kernel-piece binary search vs
  sequential (at 12 pieces, linear scan is probably fine).

**Regression harness:**
- Capture a golden `motion_report` trace from a 5-minute print on
  current `magnum-opus` with linear-only Moves. Bit-compare post-D2a.
- Generate 100 random quintic shapes, sample C-side vs numpy
  reference at 1000 points each. Tolerance 1e-9 mm.

**Files that definitely need edits (as a checklist):**
- `klippy/chelper/trapq.h` — struct move, enums, helpers
- `klippy/chelper/trapq.c` — dispatch in move_get_coord,
  move_get_distance, trapq_finalize_moves null-check, trapq_extract_old
- `klippy/chelper/integrate.h` — smoother_antiderivatives (3→11),
  struct smoother (piecewise)
- `klippy/chelper/integrate.c` — calc_antiderivatives,
  integrate_move, integrate_velocity, init_smoother
- `klippy/chelper/kin_shaper.c` — FFI signature, range_integrate
  (piece-aware), direct-access paths at :66-68, :221
- `klippy/chelper/kin_extruder.c` — FFI signature, direct-access
  at :44, :208, pa_range_integrate piece-aware
- `klippy/chelper/itersolve.c` — check_active dispatch,
  itersolve_calc_position_from_coord explicit kind init
- `klippy/chelper/__init__.py` — cdef updates for pull_move (maybe),
  smoother FFI
- `klippy/extras/motion_report.py` — maybe new kind column
- `klippy/extras/input_shaper.py` — piecewise FFI payload packing
- `klippy/extras/shaper_calibrate.py` — `_get_smoother_sigma2`
  piecewise
- `klippy/extras/shaper_defs.py` — replace INPUT_SMOOTHERS

Count: **12 files** with real edits for D2+D1+D3 combined. Spec says
"3 C files" — true for the tagged-union struct access pattern, but
hides the FFI, Python, and integrate.c scope. Be honest about this
in planning.
