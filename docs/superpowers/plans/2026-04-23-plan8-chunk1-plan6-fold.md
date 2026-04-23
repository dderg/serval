# Plan 8 Chunk 1 — Plan 6 Fold (every move is quintic) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Retire `MOVE_LINEAR` from the trapq. Every move in the queue becomes a `MOVE_QUINTIC_POLY_T` (eventually the only variant — the enum itself also retires). Straight-line moves are represented as **degenerate quintics** with only the first three polynomial coefficients non-zero. Post-hoc shaper and PA still run unchanged.

**Architecture:** Two-stage migration.
- **Stage A** rewrites `trapq_append` (linear entry point) to internally construct a degenerate-quintic coefficient buffer and dispatch through `trapq_append_quintic`. This retires the `MOVE_LINEAR` code path in all C evaluators while leaving every Python emit site unchanged. After Stage A, no `MOVE_LINEAR` structs exist in the running trapq.
- **Stage B** migrates each Python emit site from `trapq_append` to `trapq_append_quintic` (via a Python helper `linear_as_quintic_coeffs`), then deletes the `trapq_append` FFI binding and C function entirely.

**Tech Stack:** C (`klippy/chelper/*.c`, `*.h`), Python (`klippy/chelper/__init__.py`, `klippy/toolhead.py`, `klippy/kinematics/*.py`, `klippy/extras/*.py`), pytest under `test/`.

**Spec reference:** `docs/superpowers/specs/2026-04-23-plan8-baked-in-shaper-design.md` §7 Chunk 1.

**Research reference:** `docs/superpowers/plans/plan8-research/00-summary.md`. Per §6.3, the variable-length `phases[N_MAX=32]` struct upgrade is deferred to Chunk 2 (only needed when per-axis kernel-width mismatch matters — i.e., when baking happens). Chunk 1 keeps the fixed `{accel, cruise, decel}` 3-phase layout.

**Branch:** `magnum-opus` (work continues on this branch; no worktree split).

**Commit convention:** commits backdate to **2026-04-23 within 12:00–13:00 CEST** per project memory (Phase 0 already consumed 07:45–08:30).

---

## Prerequisites

- Phase 0 research committed (`docs/superpowers/plans/plan8-research/00-summary.md`).
- Plan 8 spec approved and current (§6 marked resolved).
- Clean working tree on `magnum-opus`.

## File Structure

### Files created (Chunk 1):

- `klippy/chelper/linear_quintic.h` — declares `build_linear_as_quintic_coeffs` C helper.
- `klippy/chelper/linear_quintic.c` — implements the helper.
- `klippy/chelper/linear_quintic.py` — Python wrapper exposing `linear_as_quintic_coeffs(...)` that returns a 99-double buffer.
- `test/test_linear_as_quintic.py` — tests for the helper (C + Python layers).

### Files modified:

- `klippy/chelper/trapq.h` — eventually drop `enum move_kind`, `lin` union, `MOVE_LINEAR`.
- `klippy/chelper/trapq.c` — rewrite `trapq_append` to call helper + dispatch through quintic path; later delete linear branches in `move_get_coord`, `move_get_distance`.
- `klippy/chelper/integrate.c` — delete MOVE_LINEAR branches in `move_axis_phase_polynomial` and `integrate_move`.
- `klippy/chelper/kin_extruder.c` — delete MOVE_LINEAR branch in `pa_move_integrate`.
- `klippy/chelper/__init__.py` — update `defs_trapq`; remove `trapq_append` declaration after Stage B.
- `klippy/toolhead.py` — convert emit site (line ~496) to quintic.
- `klippy/kinematics/extruder.py` — convert emit site (line ~772) to quintic.
- `klippy/extras/force_move.py` — convert emit site (line ~103) to quintic.
- `klippy/extras/manual_stepper.py` — convert emit site (line ~78) to quintic.
- `klippy/extras/trad_rack.py` — update FFI binding (line ~2392).
- `test/test_trapq_quintic.py` — update or retire linear-path assertions.
- `test/test_plan5_integration.py` — update `test_linear_move_integration` to verify degenerate-quintic produces same stepper output as old linear path.

### Files NOT modified (explicit):

- `klippy/chelper/kin_cartesian.c` / `kin_corexy.c` / `kin_corexz.c` / `kin_delta.c` / `kin_deltesian.c` / `kin_polar.c` / `kin_rotary_delta.c` / `kin_winch.c` / `kin_idex.c` / `kin_shaper.c`, and `hybrid_corexy.c` / `hybrid_corexz.c` — these consume `move_get_coord` and `move_get_distance` abstractions; no per-file edits needed.

---

## Stage A — Internal migration (emit sites unchanged)

### Task 1: Add `build_linear_as_quintic_coeffs` C helper with tests

**Files:**
- Create: `klippy/chelper/linear_quintic.h`
- Create: `klippy/chelper/linear_quintic.c`
- Create: `test/test_linear_as_quintic.py`
- Test: `test/test_linear_as_quintic.py`

**Rationale:** Build a single reusable helper that constructs a 99-double coefficient buffer representing a linear motion `x(t) = start_pos + axes_r * (start_v * t + half_accel * t²)` as a degenerate quintic where `c[0] = start_pos_axis`, `c[1] = axes_r_axis * start_v`, `c[2] = axes_r_axis * half_accel`, `c[3..10] = 0`. Buffer layout matches `trapq_append_quintic`'s existing `coeff_buf` (3 phases × 11 coeffs × 3 axes, xyz-interleaved per coefficient).

Per the Phase 0 `per_axis_frequency.md` research, today the layout is 99 doubles: `coeff_buf[phase][coeff][axis]` where phase ∈ {accel, cruise, decel}, coeff ∈ [0..10], axis ∈ {x,y,z}. See `trapq.c:251-286` for the existing unpack.

- [ ] **Step 1: Write the failing test**

Create `test/test_linear_as_quintic.py`:

```python
import pytest
from klippy.chelper import get_ffi
from klippy.chelper.linear_quintic import linear_as_quintic_coeffs


def test_degenerate_quintic_matches_linear_at_sample_times():
    # Linear motion: accel from v0=10 mm/s over accel_t=0.05s at a=200 mm/s^2,
    # cruise for 0.1s, decel over 0.05s. axes_r = (1,0,0) — pure X.
    ffi, lib = get_ffi()
    accel_t, cruise_t, decel_t = 0.05, 0.1, 0.05
    start_v, accel = 10.0, 200.0
    cruise_v = start_v + accel * accel_t
    start_pos = (0.0, 0.0, 0.0)
    axes_r = (1.0, 0.0, 0.0)

    coeffs = linear_as_quintic_coeffs(
        accel_t, cruise_t, decel_t,
        start_v, cruise_v, accel,
        axes_r, start_pos,
    )
    assert len(coeffs) == 99

    # Degenerate quintic: phase 0 (accel), c[0]=start_pos_x=0, c[1]=start_v,
    # c[2]=half_accel=100, c[3..10]=0.
    # Buffer index: phase * 33 + coeff * 3 + axis.
    assert coeffs[0 * 33 + 0 * 3 + 0] == pytest.approx(0.0)    # x0
    assert coeffs[0 * 33 + 1 * 3 + 0] == pytest.approx(10.0)   # v0
    assert coeffs[0 * 33 + 2 * 3 + 0] == pytest.approx(100.0)  # half_a
    for i in range(3, 11):
        assert coeffs[0 * 33 + i * 3 + 0] == 0.0


def test_degenerate_quintic_pure_cruise():
    # accel_t=0, cruise_t=0.1, decel_t=0: constant velocity segment only.
    coeffs = linear_as_quintic_coeffs(
        0.0, 0.1, 0.0,
        50.0, 50.0, 0.0,
        (1.0, 0.0, 0.0), (5.0, 0.0, 0.0),
    )
    # Cruise phase (phase 1): c[0]=5 (x0 at start of cruise), c[1]=50, c[2..10]=0.
    assert coeffs[1 * 33 + 0 * 3 + 0] == pytest.approx(5.0)
    assert coeffs[1 * 33 + 1 * 3 + 0] == pytest.approx(50.0)
    assert coeffs[1 * 33 + 2 * 3 + 0] == 0.0
```

- [ ] **Step 2: Run test to verify it fails**

```
pytest test/test_linear_as_quintic.py -v
```
Expected: FAIL — `ModuleNotFoundError: klippy.chelper.linear_quintic` or `ImportError`.

- [ ] **Step 3: Implement the C helper**

Create `klippy/chelper/linear_quintic.h`:

```c
#ifndef LINEAR_QUINTIC_H
#define LINEAR_QUINTIC_H

// Fill a 99-double coefficient buffer representing a linear
// accel/cruise/decel trapezoid as a degenerate quintic. Buffer layout:
// coeff_buf[phase * 33 + coeff * 3 + axis]. phase ∈ {0=accel, 1=cruise,
// 2=decel}, coeff ∈ [0..10], axis ∈ {0=x, 1=y, 2=z}. For degenerate
// quintic: c[0] = start_pos, c[1] = axes_r * v_start_of_phase, c[2] =
// axes_r * half_accel_of_phase, c[3..10] = 0.
void build_linear_as_quintic_coeffs(
    double accel_t, double cruise_t, double decel_t,
    double start_v, double cruise_v, double accel,
    double axes_r_x, double axes_r_y, double axes_r_z,
    double start_pos_x, double start_pos_y, double start_pos_z,
    double coeff_buf[99]);

#endif
```

Create `klippy/chelper/linear_quintic.c`:

```c
#include "linear_quintic.h"

static inline void
fill_phase(double *buf_phase, double v, double a,
           double pos_x, double pos_y, double pos_z,
           double rx, double ry, double rz)
{
    // c[0] = start_pos_axis
    buf_phase[0 * 3 + 0] = pos_x;
    buf_phase[0 * 3 + 1] = pos_y;
    buf_phase[0 * 3 + 2] = pos_z;
    // c[1] = axes_r * v
    buf_phase[1 * 3 + 0] = rx * v;
    buf_phase[1 * 3 + 1] = ry * v;
    buf_phase[1 * 3 + 2] = rz * v;
    // c[2] = axes_r * (a / 2)
    double half_a = 0.5 * a;
    buf_phase[2 * 3 + 0] = rx * half_a;
    buf_phase[2 * 3 + 1] = ry * half_a;
    buf_phase[2 * 3 + 2] = rz * half_a;
    // c[3..10] = 0
    for (int i = 3; i < 11; i++) {
        buf_phase[i * 3 + 0] = 0.0;
        buf_phase[i * 3 + 1] = 0.0;
        buf_phase[i * 3 + 2] = 0.0;
    }
}

void
build_linear_as_quintic_coeffs(
    double accel_t, double cruise_t, double decel_t,
    double start_v, double cruise_v, double accel,
    double axes_r_x, double axes_r_y, double axes_r_z,
    double start_pos_x, double start_pos_y, double start_pos_z,
    double coeff_buf[99])
{
    // Accel phase: start_v + accel * t, pos starts at (start_pos_*).
    fill_phase(&coeff_buf[0 * 33], start_v, accel,
               start_pos_x, start_pos_y, start_pos_z,
               axes_r_x, axes_r_y, axes_r_z);
    // Cruise phase: constant cruise_v, pos starts where accel ended.
    double pos_after_accel_x = start_pos_x + axes_r_x * (start_v * accel_t + 0.5 * accel * accel_t * accel_t);
    double pos_after_accel_y = start_pos_y + axes_r_y * (start_v * accel_t + 0.5 * accel * accel_t * accel_t);
    double pos_after_accel_z = start_pos_z + axes_r_z * (start_v * accel_t + 0.5 * accel * accel_t * accel_t);
    fill_phase(&coeff_buf[1 * 33], cruise_v, 0.0,
               pos_after_accel_x, pos_after_accel_y, pos_after_accel_z,
               axes_r_x, axes_r_y, axes_r_z);
    // Decel phase: starts at cruise_v, accel = -accel (deceleration).
    double pos_after_cruise_x = pos_after_accel_x + axes_r_x * cruise_v * cruise_t;
    double pos_after_cruise_y = pos_after_accel_y + axes_r_y * cruise_v * cruise_t;
    double pos_after_cruise_z = pos_after_accel_z + axes_r_z * cruise_v * cruise_t;
    fill_phase(&coeff_buf[2 * 33], cruise_v, -accel,
               pos_after_cruise_x, pos_after_cruise_y, pos_after_cruise_z,
               axes_r_x, axes_r_y, axes_r_z);
}
```

- [ ] **Step 4: Register the new C file in the build**

Modify `klippy/chelper/__init__.py`. Locate the source list near line 40 (look for `SRC_FILES` or equivalent — grep for `trapq.c` in `__init__.py` and find the adjacent source list). Add `'linear_quintic.c'` to the sources. Add the FFI declaration to `defs_trapq`:

```python
defs_trapq = """
    ...
    void trapq_append_quintic(...);
    // ADD THIS:
    void build_linear_as_quintic_coeffs(
        double accel_t, double cruise_t, double decel_t,
        double start_v, double cruise_v, double accel,
        double axes_r_x, double axes_r_y, double axes_r_z,
        double start_pos_x, double start_pos_y, double start_pos_z,
        double coeff_buf[99]);
    ...
"""
```

- [ ] **Step 5: Create Python wrapper**

Create `klippy/chelper/linear_quintic.py`:

```python
"""Python wrapper around build_linear_as_quintic_coeffs C helper."""
from klippy.chelper import get_ffi


def linear_as_quintic_coeffs(
    accel_t, cruise_t, decel_t,
    start_v, cruise_v, accel,
    axes_r, start_pos,
):
    """Return a 99-double list representing a linear accel/cruise/decel
    motion as a degenerate quintic coefficient buffer.

    axes_r, start_pos: 3-tuples (x, y, z)."""
    ffi, lib = get_ffi()
    buf = ffi.new("double[99]")
    lib.build_linear_as_quintic_coeffs(
        accel_t, cruise_t, decel_t,
        start_v, cruise_v, accel,
        axes_r[0], axes_r[1], axes_r[2],
        start_pos[0], start_pos[1], start_pos[2],
        buf,
    )
    return [buf[i] for i in range(99)]
```

- [ ] **Step 6: Run test to verify it passes**

```
pytest test/test_linear_as_quintic.py -v
```
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:05:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:05:00+02:00" \
  git add klippy/chelper/linear_quintic.h klippy/chelper/linear_quintic.c \
          klippy/chelper/linear_quintic.py klippy/chelper/__init__.py \
          test/test_linear_as_quintic.py && \
  GIT_AUTHOR_DATE="2026-04-23T12:05:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:05:00+02:00" \
  git commit -m "chunk1: linear→degenerate-quintic coeff helper"
```

---

### Task 2: Rewrite `trapq_append` to dispatch through quintic

**Files:**
- Modify: `klippy/chelper/trapq.c:195-243`
- Test: `test/test_trapq_append_routes_quintic.py` (new)

**Rationale:** `trapq_append` currently constructs three `MOVE_LINEAR` entries (accel, cruise, decel). We rewrite it to build a degenerate-quintic coefficient buffer via `build_linear_as_quintic_coeffs` and delegate to `trapq_append_quintic`. After this, every move reaching the trapq is `MOVE_QUINTIC_POLY_T`; `MOVE_LINEAR` structs cease to exist at runtime.

- [ ] **Step 1: Write the failing test**

Create `test/test_trapq_append_routes_quintic.py`:

```python
import pytest
from klippy.chelper import get_ffi


def test_trapq_append_produces_quintic_kind():
    ffi, lib = get_ffi()
    tq = lib.trapq_alloc()
    lib.trapq_append(
        tq, 0.0,         # print_time
        0.05, 0.1, 0.05, # accel_t, cruise_t, decel_t
        0.0, 0.0, 0.0,   # start_pos x,y,z
        1.0, 0.0, 0.0,   # axes_r x,y,z
        10.0, 20.0, 200.0,  # start_v, cruise_v, accel
    )
    # Walk the list and assert every move has kind=MOVE_QUINTIC_POLY_T (=1).
    move = lib.trapq_get_history_head(tq)  # or equivalent head accessor
    seen_moves = 0
    while move != ffi.NULL:
        assert move.kind == 1, f"expected MOVE_QUINTIC_POLY_T (1), got {move.kind}"
        seen_moves += 1
        move = move.node.next  # adjust per actual list macro
    assert seen_moves >= 1
    lib.trapq_free(tq)
```

(Note: the list walk needs to match the actual `struct trapq` accessor. If no head-accessor exists, add one in trapq.c or use `trapq_extract_old` in a targeted window. An alternative is to use an existing test fixture in `test_trapq_quintic.py` — inspect that file first and mirror its style.)

- [ ] **Step 2: Run test to verify it fails**

```
pytest test/test_trapq_append_routes_quintic.py -v
```
Expected: FAIL — moves have `kind == 0` (MOVE_LINEAR).

- [ ] **Step 3: Rewrite `trapq_append` in `klippy/chelper/trapq.c`**

Replace the body of `trapq_append` (currently lines 195-243) with:

```c
void __visible
trapq_append(struct trapq *tq, double print_time
             , double accel_t, double cruise_t, double decel_t
             , double start_pos_x, double start_pos_y, double start_pos_z
             , double axes_r_x, double axes_r_y, double axes_r_z
             , double start_v, double cruise_v, double accel)
{
    double coeff_buf[99];
    build_linear_as_quintic_coeffs(
        accel_t, cruise_t, decel_t,
        start_v, cruise_v, accel,
        axes_r_x, axes_r_y, axes_r_z,
        start_pos_x, start_pos_y, start_pos_z,
        coeff_buf);
    double move_t = accel_t + cruise_t + decel_t;
    // arc_length for a straight line = sum of per-phase displacements.
    double accel_d = start_v * accel_t + 0.5 * accel * accel_t * accel_t;
    double cruise_d = cruise_v * cruise_t;
    double decel_d = cruise_v * decel_t - 0.5 * accel * decel_t * decel_t;
    double total_d = accel_d + cruise_d + decel_d;
    double axes_r_mag = sqrt(axes_r_x * axes_r_x + axes_r_y * axes_r_y
                             + axes_r_z * axes_r_z);
    double arc_length = total_d * axes_r_mag;
    // v_cap_min is the minimum instantaneous velocity; for a trapezoid this
    // is min(start_v, cruise_v endpoints) — conservatively set to the
    // smaller of start_v and decel end velocity. Decel end v = cruise_v -
    // accel * decel_t.
    double decel_end_v = cruise_v - accel * decel_t;
    double v_cap_min = fmin(fmin(start_v, cruise_v), decel_end_v);
    if (v_cap_min < 0.0) v_cap_min = 0.0;
    trapq_append_quintic(
        tq, print_time,
        accel_t,                // t_accel_end
        accel_t + cruise_t,     // t_decel_start
        move_t, arc_length, v_cap_min,
        start_pos_x, start_pos_y, start_pos_z,
        coeff_buf);
}
```

Add `#include "linear_quintic.h"` and `#include <math.h>` (if not already) at the top of `trapq.c`.

- [ ] **Step 4: Rebuild and run test**

```
# Rebuild the C extension (cffi recompile).
rm -f klippy/chelper/c_helper.so*
pytest test/test_trapq_append_routes_quintic.py -v
```
Expected: PASS — all emitted moves have `kind == 1`.

- [ ] **Step 5: Run full regression suite**

```
pytest test/ -v
```
Expected: PASS for ALL existing tests. If `test_trapq_quintic.py` or `test_plan5_integration.py:577-609 test_linear_move_integration` assumes `kind == 0` after a `trapq_append` call, those tests need updating in Task 8. For now, just note which tests fail and confirm the failures are limited to `kind == 0` assertions.

- [ ] **Step 6: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:10:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:10:00+02:00" \
  git add klippy/chelper/trapq.c test/test_trapq_append_routes_quintic.py && \
  GIT_AUTHOR_DATE="2026-04-23T12:10:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:10:00+02:00" \
  git commit -m "chunk1: trapq_append dispatches through quintic path"
```

---

### Task 3: Delete `MOVE_LINEAR` branch from `move_get_coord`

**Files:**
- Modify: `klippy/chelper/trapq.c:92-110`

**Rationale:** After Task 2, no `MOVE_LINEAR` structs exist at runtime. The linear branch is dead code. Delete it.

- [ ] **Step 1: Write a confirmation test**

Update or create `test/test_move_get_coord_quintic_only.py`:

```python
from klippy.chelper import get_ffi


def test_move_get_coord_on_degenerate_quintic_matches_linear_formula():
    ffi, lib = get_ffi()
    tq = lib.trapq_alloc()
    lib.trapq_append(tq, 0.0,
                     0.05, 0.1, 0.05,
                     0.0, 0.0, 0.0,
                     1.0, 0.0, 0.0,
                     10.0, 20.0, 200.0)
    # At t=0.025 (mid-accel), linear formula says x = start_v*t + 0.5*a*t^2
    #                                           = 10 * 0.025 + 100 * 0.025^2
    #                                           = 0.25 + 0.0625 = 0.3125
    # Walk to the accel move and evaluate at move-local t=0.025.
    # (Use whichever accessor / test fixture exists in test_plan5_integration.)
    # ...assertion: coord.x == approx(0.3125) at that time.
    lib.trapq_free(tq)
```

(Use the pattern from `test_plan5_integration.py:577-609` for the list-walk.)

- [ ] **Step 2: Run test to verify it passes BEFORE edit**

```
pytest test/test_move_get_coord_quintic_only.py -v
```
Expected: PASS (the quintic path handles this correctly even with the linear branch in place).

- [ ] **Step 3: Delete the linear branch in `move_get_coord`**

In `klippy/chelper/trapq.c`, locate `move_get_coord` (lines 92-110 per the map):

```c
// BEFORE:
inline struct coord
move_get_coord(const struct move *m, double move_time)
{
    if (likely(m->kind == MOVE_LINEAR)) {
        double move_dist = (m->u.lin.start_v + m->u.lin.half_accel*move_time)
                           * move_time;
        return (struct coord) {
            .x = m->start_pos.x + m->u.lin.axes_r.x * move_dist,
            .y = m->start_pos.y + m->u.lin.axes_r.y * move_dist,
            .z = m->start_pos.z + m->u.lin.axes_r.z * move_dist };
    }
    // MOVE_QUINTIC_POLY_T
    double delta_t;
    const struct move_quintic_phase *ph = quintic_pick_phase(m, move_time, &delta_t);
    return quintic_phase_eval(ph, delta_t);
}

// AFTER:
inline struct coord
move_get_coord(const struct move *m, double move_time)
{
    double delta_t;
    const struct move_quintic_phase *ph = quintic_pick_phase(m, move_time, &delta_t);
    return quintic_phase_eval(ph, delta_t);
}
```

- [ ] **Step 4: Rebuild and verify tests still pass**

```
rm -f klippy/chelper/c_helper.so*
pytest test/test_move_get_coord_quintic_only.py test/test_plan5_integration.py -v
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:15:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:15:00+02:00" \
  git add klippy/chelper/trapq.c test/test_move_get_coord_quintic_only.py && \
  GIT_AUTHOR_DATE="2026-04-23T12:15:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:15:00+02:00" \
  git commit -m "chunk1: drop MOVE_LINEAR branch from move_get_coord"
```

---

### Task 4: Delete `MOVE_LINEAR` branch from `move_get_distance`

**Files:**
- Modify: `klippy/chelper/trapq.c:72-85`

- [ ] **Step 1: Confirmation test**

Verify `move_get_distance` on a degenerate-quintic trapezoidal move matches the closed-form linear total distance.

Add to `test/test_move_get_coord_quintic_only.py`:

```python
def test_move_get_distance_on_degenerate_quintic():
    ffi, lib = get_ffi()
    tq = lib.trapq_alloc()
    lib.trapq_append(tq, 0.0, 0.05, 0.1, 0.05,
                     0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                     10.0, 20.0, 200.0)
    # Walk to the accel move. At move-local t=0.05 (end of accel),
    # distance = 10*0.05 + 100*0.05^2 = 0.5 + 0.25 = 0.75 mm.
    # ...assertion: move_get_distance(accel_move, 0.05) ≈ 0.75.
    lib.trapq_free(tq)
```

- [ ] **Step 2: Run test (should pass with linear branch still in place)**

```
pytest test/test_move_get_coord_quintic_only.py -v
```
Expected: PASS.

- [ ] **Step 3: Delete the linear branch**

In `klippy/chelper/trapq.c:72-85`:

```c
// BEFORE:
inline double
move_get_distance(const struct move *m, double move_time)
{
    if (likely(m->kind == MOVE_LINEAR))
        return (m->u.lin.start_v + m->u.lin.half_accel * move_time) * move_time;
    // MOVE_QUINTIC_POLY_T — chord distance
    struct coord end = move_get_coord(m, move_time);
    double dx = end.x - m->start_pos.x, dy = end.y - m->start_pos.y,
           dz = end.z - m->start_pos.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// AFTER:
inline double
move_get_distance(const struct move *m, double move_time)
{
    struct coord end = move_get_coord(m, move_time);
    double dx = end.x - m->start_pos.x, dy = end.y - m->start_pos.y,
           dz = end.z - m->start_pos.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}
```

- [ ] **Step 4: Rebuild and verify**

```
rm -f klippy/chelper/c_helper.so*
pytest test/test_move_get_coord_quintic_only.py -v
```
Expected: PASS.

**Note on arc-length vs chord:** the Phase 0 spec flagged `move_get_distance` as returning chord (not arc length) for quintic moves. For degenerate quintics (straight lines), chord == arc length, so no regression. For curved quintics (corners), this is a latent issue addressed in Chunk 2. Leave the chord behavior unchanged here.

- [ ] **Step 5: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:20:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:20:00+02:00" \
  git add klippy/chelper/trapq.c test/test_move_get_coord_quintic_only.py && \
  GIT_AUTHOR_DATE="2026-04-23T12:20:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:20:00+02:00" \
  git commit -m "chunk1: drop MOVE_LINEAR branch from move_get_distance"
```

---

### Task 5: Delete `MOVE_LINEAR` branch from `move_axis_phase_polynomial`

**Files:**
- Modify: `klippy/chelper/integrate.c:142-172`

- [ ] **Step 1: Run full test suite to capture baseline**

```
pytest test/ -v 2>&1 | tee /tmp/baseline.log
```
Note which tests pass.

- [ ] **Step 2: Delete the linear branch**

In `klippy/chelper/integrate.c` around line 147:

```c
// BEFORE:
static inline void
move_axis_phase_polynomial(const struct move* m, int axis, double move_time,
                           double out_c[SMOOTHER_NUM_MOMENTS],
                           double* out_phase_start, double* out_phase_end)
{
    if (likely(m->kind == MOVE_LINEAR)) {
        double start_v = m->u.lin.start_v;
        double half_accel = m->u.lin.half_accel;
        double axes_r_a = ((axis == 0) ? m->u.lin.axes_r.x
                           : (axis == 1) ? m->u.lin.axes_r.y
                           : m->u.lin.axes_r.z);
        out_c[0] = ((axis == 0) ? m->start_pos.x
                    : (axis == 1) ? m->start_pos.y : m->start_pos.z);
        out_c[1] = axes_r_a * start_v;
        out_c[2] = axes_r_a * half_accel;
        for (int i = 3; i < SMOOTHER_NUM_MOMENTS; i++) out_c[i] = 0.0;
        *out_phase_start = 0.0;
        *out_phase_end = m->move_t;
        return;
    }
    // MOVE_QUINTIC_POLY_T: pick phase, copy c[] directly
    double delta_t;
    const struct move_quintic_phase *ph = quintic_pick_phase(m, move_time, &delta_t);
    double axis_c_offset = axis; // xyz-interleaved: c[k].x=[3k+0], y=[3k+1], z=[3k+2]
    for (int i = 0; i < SMOOTHER_NUM_MOMENTS; i++) {
        out_c[i] = ((double*)ph->c)[i * 3 + axis_c_offset];
    }
    // ... phase_start / phase_end computed from accel_t / decel_t offsets ...
    *out_phase_start = /* ... */;
    *out_phase_end = /* ... */;
}

// AFTER — retain only the quintic body:
static inline void
move_axis_phase_polynomial(const struct move* m, int axis, double move_time,
                           double out_c[SMOOTHER_NUM_MOMENTS],
                           double* out_phase_start, double* out_phase_end)
{
    double delta_t;
    const struct move_quintic_phase *ph = quintic_pick_phase(m, move_time, &delta_t);
    double axis_c_offset = axis;
    for (int i = 0; i < SMOOTHER_NUM_MOMENTS; i++) {
        out_c[i] = ((double*)ph->c)[i * 3 + axis_c_offset];
    }
    *out_phase_start = /* per existing logic, phase boundary */;
    *out_phase_end = /* per existing logic */;
}
```

**IMPORTANT:** read the actual body of `move_axis_phase_polynomial` at `integrate.c:142-172` before editing. The phase_start/phase_end computation depends on the move's `accel`, `decel_start` fields — preserve those exactly.

- [ ] **Step 3: Rebuild and re-run baseline**

```
rm -f klippy/chelper/c_helper.so*
pytest test/ -v 2>&1 | tee /tmp/after.log
diff /tmp/baseline.log /tmp/after.log
```
Expected: no test regressions.

- [ ] **Step 4: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:25:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:25:00+02:00" \
  git add klippy/chelper/integrate.c && \
  GIT_AUTHOR_DATE="2026-04-23T12:25:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:25:00+02:00" \
  git commit -m "chunk1: drop MOVE_LINEAR branch from move_axis_phase_polynomial"
```

---

### Task 6: Delete `MOVE_LINEAR` branch from `integrate_move`

**Files:**
- Modify: `klippy/chelper/integrate.c:178-261`

- [ ] **Step 1: Delete the linear fast-path**

In `integrate.c:178-261`, the `integrate_move` function currently has:

```c
static double
integrate_move(const struct move *m, int axis,
               double t_center, double t_half,
               const smoother_antiderivatives *ad)
{
    if (likely(m->kind == MOVE_LINEAR)) {
        // closed-form 3-moment integration...
        return /* linear fast-path result */;
    }
    // MOVE_QUINTIC_POLY_T: call move_axis_phase_polynomial, integrate against
    // 11-moment smoother via binomial expansion...
    // ... ~50 lines ...
}
```

Replace with just the quintic body. **Read the actual body at integrate.c:178-261 before editing — preserve the 11-moment integration exactly.**

- [ ] **Step 2: Rebuild and run integration tests**

```
rm -f klippy/chelper/c_helper.so*
pytest test/test_plan5_integration.py test/test_trapq_quintic.py -v
```
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:30:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:30:00+02:00" \
  git add klippy/chelper/integrate.c && \
  GIT_AUTHOR_DATE="2026-04-23T12:30:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:30:00+02:00" \
  git commit -m "chunk1: drop MOVE_LINEAR fast-path from integrate_move"
```

---

### Task 7: Delete `MOVE_LINEAR` branch from `pa_move_integrate`

**Files:**
- Modify: `klippy/chelper/kin_extruder.c:41-70`

- [ ] **Step 1: Delete the linear branch**

In `kin_extruder.c:41-70`, the `pa_move_integrate` function currently has:

```c
static inline void
pa_move_integrate(const struct move *m, int axis, double t0,
                  const smoother_antiderivatives *ad, double *pa_velocity_integral)
{
    int can_pressure_advance = 0;
    if (likely(m->kind == MOVE_LINEAR)) {
        can_pressure_advance = (m->u.lin.axes_r.x > 0 || m->u.lin.axes_r.y > 0);
    } else {
        // MOVE_QUINTIC_POLY_T — scan all 3 phases for non-zero c[1..10] on X/Y
        const struct move_quintic_phase *phases[] = {
            &m->u.quintic.accel, &m->u.quintic.cruise, &m->u.quintic.decel
        };
        for (int p = 0; p < 3; p++) {
            for (int k = 1; k < MOVE_QUINTIC_POLY_COEFFS; k++) {
                if (phases[p]->c[k].x != 0.0 || phases[p]->c[k].y != 0.0) {
                    can_pressure_advance = 1;
                    break;
                }
            }
            if (can_pressure_advance) break;
        }
    }
    if (!can_pressure_advance) { *pa_velocity_integral = 0.0; return; }
    integrate_velocity(m, axis, t0, ad, pa_velocity_integral);
}
```

Replace with the quintic scan only (the linear check becomes a subset of the quintic scan for degenerate-quintic moves, since `axes_r.x > 0` implies `c[1].x != 0` on the accel phase):

```c
static inline void
pa_move_integrate(const struct move *m, int axis, double t0,
                  const smoother_antiderivatives *ad, double *pa_velocity_integral)
{
    int can_pressure_advance = 0;
    const struct move_quintic_phase *phases[] = {
        &m->u.quintic.accel, &m->u.quintic.cruise, &m->u.quintic.decel
    };
    for (int p = 0; p < 3; p++) {
        for (int k = 1; k < MOVE_QUINTIC_POLY_COEFFS; k++) {
            if (phases[p]->c[k].x != 0.0 || phases[p]->c[k].y != 0.0) {
                can_pressure_advance = 1;
                break;
            }
        }
        if (can_pressure_advance) break;
    }
    if (!can_pressure_advance) { *pa_velocity_integral = 0.0; return; }
    integrate_velocity(m, axis, t0, ad, pa_velocity_integral);
}
```

- [ ] **Step 2: Rebuild and run extruder tests**

```
rm -f klippy/chelper/c_helper.so*
pytest test/ -k extruder -v
```
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:35:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:35:00+02:00" \
  git add klippy/chelper/kin_extruder.c && \
  GIT_AUTHOR_DATE="2026-04-23T12:35:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:35:00+02:00" \
  git commit -m "chunk1: drop MOVE_LINEAR branch from pa_move_integrate"
```

---

### Task 8: Remove `MOVE_LINEAR` enum + `lin` union from `struct move`

**Files:**
- Modify: `klippy/chelper/trapq.h:15-56`
- Modify: `klippy/chelper/trapq.c` (any remaining `m->kind` reads, `m->u.lin.*` reads)

**Rationale:** No code path reads `m->u.lin` or `m->kind == MOVE_LINEAR` anymore. Delete the union and the enum.

- [ ] **Step 1: Delete the enum and union**

In `klippy/chelper/trapq.h`:

```c
// BEFORE:
enum move_kind {
    MOVE_LINEAR = 0,
    MOVE_QUINTIC_POLY_T = 1,
};

struct move {
    double print_time, move_t;
    enum move_kind kind;
    struct coord start_pos;
    union {
        struct { double start_v, half_accel; struct coord axes_r; } lin;
        struct {
            double arc_length;
            struct move_quintic_phase accel, cruise, decel;
            double v_cap_min;
        } quintic;
    } u;
    struct list_node node;
};

// AFTER:
struct move {
    double print_time, move_t;
    struct coord start_pos;
    double arc_length;
    struct move_quintic_phase accel, cruise, decel;
    double v_cap_min;
    struct list_node node;
};
```

- [ ] **Step 2: Update all reads of `m->u.quintic.*` to `m->*`**

Grep for `u.quintic.` across `klippy/chelper/*.c`:

```bash
grep -rn 'u\.quintic\.' klippy/chelper/
```

Rewrite each occurrence. Example: `m->u.quintic.accel` → `m->accel`, `m->u.quintic.arc_length` → `m->arc_length`, etc.

- [ ] **Step 3: Update `trapq_append_quintic` to fill the flattened struct**

In `trapq.c`, the quintic unpack loop at lines 271-284 writes `m->u.quintic.accel.c[i]` etc. Rewrite to `m->accel.c[i]` etc.

- [ ] **Step 4: Update any test code that references `m.kind` or `m.u.quintic`**

Grep the test tree:

```bash
grep -rn 'u\.quintic\|\.kind' test/ klippy/
```

Fix each reference. Tests that assert `m.kind == 1` need to be deleted or replaced with an assertion that the move has quintic phase data (e.g., `m.accel.t_end >= 0`).

- [ ] **Step 5: Rebuild and full regression**

```
rm -f klippy/chelper/c_helper.so*
pytest test/ -v
```
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:40:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:40:00+02:00" \
  git add klippy/chelper/trapq.h klippy/chelper/trapq.c klippy/chelper/integrate.c \
          klippy/chelper/kin_extruder.c test/ && \
  GIT_AUTHOR_DATE="2026-04-23T12:40:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:40:00+02:00" \
  git commit -m "chunk1: delete MOVE_LINEAR enum and lin union from struct move"
```

---

### Task 9: Stage A closure — sim regression corpus verification

**Files:** test only; no code change.

**Rationale:** At the end of Stage A, the trapq runs entirely on quintic. Verify sim output is bit-identical to pre-Chunk-1.

- [ ] **Step 1: Run sim regression corpus**

Assuming `klipsim` integration exists (per recent memory `reference_klipper_sim.md`):

```bash
cd ../klipper-sim
pytest tests/ -v
# If there's a specific regression command for pre/post comparison:
# ./run_regression_corpus.sh --baseline <pre-chunk1-sha> --current HEAD
```

Expected: all tests pass. If klipsim has a trajectory-diff mode, diff against the pre-Chunk-1 commit; expect zero diff on all corpus gcode.

- [ ] **Step 2: Run the full klippy test suite on the fork**

```bash
cd /Users/daniladergachev/Developer/kalico
pytest test/ -v
```
Expected: PASS.

- [ ] **Step 3: Record results (no commit — this is a verification gate)**

If any regression appears, fix before proceeding. If clean, proceed to Stage B.

---

## Stage B — Retire `trapq_append` entry point

### Task 10: Migrate `klippy/toolhead.py:496` emit site to quintic

**Files:**
- Modify: `klippy/toolhead.py:~496`
- Test: `test/test_plan5_integration.py:577-609` already covers this path; extend if needed.

- [ ] **Step 1: Inspect the current emit site**

```
grep -n 'trapq_append' klippy/toolhead.py
```

Confirm the site at ~line 496 and its parameters.

- [ ] **Step 2: Replace the call**

```python
# BEFORE (toolhead.py ~496):
self.trapq_append(
    self.trapq, next_move_time,
    move.accel_t, move.cruise_t, move.decel_t,
    move.start_pos[0], move.start_pos[1], move.start_pos[2],
    move.axes_r[0], move.axes_r[1], move.axes_r[2],
    move.start_v, move.cruise_v, move.accel)

# AFTER:
from klippy.chelper.linear_quintic import linear_as_quintic_coeffs
# ... (import at module top)

coeffs = linear_as_quintic_coeffs(
    move.accel_t, move.cruise_t, move.decel_t,
    move.start_v, move.cruise_v, move.accel,
    move.axes_r, move.start_pos)
move_t = move.accel_t + move.cruise_t + move.decel_t
self.trapq_append_quintic(
    self.trapq, next_move_time,
    move.accel_t,                  # t_accel_end
    move.accel_t + move.cruise_t,  # t_decel_start
    move_t, move.move_d, move.min_cruise_v,  # arc_length, v_cap_min
    move.start_pos[0], move.start_pos[1], move.start_pos[2],
    coeffs)
```

(Parameter names may differ — verify `move.move_d`, `move.min_cruise_v` exist on the Move object in toolhead.py; if not, use the same computation as in `trapq_append` C-side: `arc_length = total_d * |axes_r|`, `v_cap_min = max(0, min(start_v, cruise_v, decel_end_v))`.)

Add `self.trapq_append_quintic = ffi_lib.trapq_append_quintic` to the FFI binding setup if not already present.

- [ ] **Step 3: Run regression**

```
pytest test/test_plan5_integration.py -v
pytest test/ -v
```
Expected: PASS. Byte-identical stepper output to pre-migration.

- [ ] **Step 4: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:45:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:45:00+02:00" \
  git add klippy/toolhead.py && \
  GIT_AUTHOR_DATE="2026-04-23T12:45:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:45:00+02:00" \
  git commit -m "chunk1: toolhead emits via trapq_append_quintic"
```

---

### Task 11: Migrate `klippy/kinematics/extruder.py:~772` emit site

**Files:**
- Modify: `klippy/kinematics/extruder.py:~772`

- [ ] **Step 1: Apply the same pattern as Task 10**

Locate the `trapq_append` call near line 772. Replace with a `trapq_append_quintic` call using `linear_as_quintic_coeffs`. The extruder move uses `extr_r` (a 3-tuple) as `axes_r`, and `extr_pos` (a 3-tuple) as `start_pos`.

- [ ] **Step 2: Run extruder tests**

```
pytest test/ -k extruder -v
```
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:50:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:50:00+02:00" \
  git add klippy/kinematics/extruder.py && \
  GIT_AUTHOR_DATE="2026-04-23T12:50:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:50:00+02:00" \
  git commit -m "chunk1: extruder emits via trapq_append_quintic"
```

---

### Task 12: Migrate `klippy/extras/force_move.py:~103` emit site

**Files:**
- Modify: `klippy/extras/force_move.py:~103`

- [ ] **Step 1: Apply the migration pattern**

Locate and convert the `trapq_append` call near line 103. `force_move`'s motion is typically a simple accel-cruise-decel trapezoid with known `axes_r`; reuse the linear_as_quintic_coeffs helper.

- [ ] **Step 2: Run tests**

```
pytest test/ -k force_move -v
```

- [ ] **Step 3: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:53:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:53:00+02:00" \
  git add klippy/extras/force_move.py && \
  GIT_AUTHOR_DATE="2026-04-23T12:53:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:53:00+02:00" \
  git commit -m "chunk1: force_move emits via trapq_append_quintic"
```

---

### Task 13: Migrate `klippy/extras/manual_stepper.py:~78` emit site

**Files:**
- Modify: `klippy/extras/manual_stepper.py:~78`

- [ ] **Step 1: Apply the migration pattern**

Same as Task 12.

- [ ] **Step 2: Run tests**

```
pytest test/ -k manual_stepper -v
```

- [ ] **Step 3: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:56:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:56:00+02:00" \
  git add klippy/extras/manual_stepper.py && \
  GIT_AUTHOR_DATE="2026-04-23T12:56:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:56:00+02:00" \
  git commit -m "chunk1: manual_stepper emits via trapq_append_quintic"
```

---

### Task 14: Migrate `klippy/extras/trad_rack.py` FFI binding

**Files:**
- Modify: `klippy/extras/trad_rack.py:~2392`

- [ ] **Step 1: Inspect the usage**

```
grep -n 'trapq_append' klippy/extras/trad_rack.py
```

If trad_rack only binds but never calls `trapq_append`, remove the binding. If it does call, migrate.

- [ ] **Step 2: Apply and test**

```
pytest test/ -k trad_rack -v
```

- [ ] **Step 3: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T12:59:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:59:00+02:00" \
  git add klippy/extras/trad_rack.py && \
  GIT_AUTHOR_DATE="2026-04-23T12:59:00+02:00" GIT_COMMITTER_DATE="2026-04-23T12:59:00+02:00" \
  git commit -m "chunk1: trad_rack migrated off trapq_append"
```

---

### Task 15: Delete `trapq_append` C function and FFI binding

**Files:**
- Modify: `klippy/chelper/trapq.c` — delete `trapq_append` function body
- Modify: `klippy/chelper/trapq.h` — delete `trapq_append` declaration
- Modify: `klippy/chelper/__init__.py` — remove `trapq_append` from `defs_trapq`

- [ ] **Step 1: Confirm no remaining callers**

```
grep -rn 'trapq_append\b' klippy/ test/ | grep -v '_quintic'
```
Expected: only the declaration + definition themselves remain. Everything else must be `trapq_append_quintic`.

- [ ] **Step 2: Delete the C function**

In `trapq.c`, delete the `trapq_append` function (was at lines 195-243 originally; now rewritten in Task 2 to dispatch through quintic — just delete it entirely).

In `trapq.h`, delete the `trapq_append` declaration.

- [ ] **Step 3: Remove FFI binding**

In `klippy/chelper/__init__.py`, remove the `void trapq_append(...)` line from `defs_trapq`.

- [ ] **Step 4: Rebuild and full regression**

```
rm -f klippy/chelper/c_helper.so*
pytest test/ -v
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
GIT_AUTHOR_DATE="2026-04-23T13:00:00+02:00" GIT_COMMITTER_DATE="2026-04-23T13:00:00+02:00" \
  git add klippy/chelper/trapq.c klippy/chelper/trapq.h klippy/chelper/__init__.py && \
  GIT_AUTHOR_DATE="2026-04-23T13:00:00+02:00" GIT_COMMITTER_DATE="2026-04-23T13:00:00+02:00" \
  git commit -m "chunk1: delete trapq_append — trapq_append_quintic is sole entry"
```

---

### Task 16: Final sim regression corpus verification

**Files:** no code change.

- [ ] **Step 1: Run sim regression corpus against Chunk 1 final head**

```bash
cd ../klipper-sim
pytest tests/ -v
```
Expected: all tests pass.

- [ ] **Step 2: Diff trajectories against pre-Chunk-1 baseline**

If klipsim offers pre/post trajectory comparison:

```bash
# Example — adapt to actual klipsim command:
./scripts/compare_trajectories.sh \
  --baseline <sha-before-chunk1> \
  --current HEAD \
  --corpus 'Voron_Cube_v7_ABS speedbench Cowling'
```

Expected: zero diff on all corpus gcode (bit-identical stepper output).

- [ ] **Step 3: Run klippy unit tests one more time**

```bash
cd /Users/daniladergachev/Developer/kalico
pytest test/ -v
```
Expected: PASS.

- [ ] **Step 4: Optional — HW test on Trident (user-driven, not gated)**

User prints a test model at typical settings. Expected: no perceptible change in print quality or step timing vs pre-Chunk-1.

---

## Exit criteria (all must be green)

- `klippy/chelper/trapq.h` has no `enum move_kind`, no `lin` union. `struct move` has flat quintic fields.
- `klippy/chelper/trapq.c`: no `trapq_append` function; `trapq_append_quintic` is the sole entry point.
- `klippy/chelper/integrate.c`: `move_axis_phase_polynomial` and `integrate_move` have no MOVE_LINEAR branches.
- `klippy/chelper/kin_extruder.c`: `pa_move_integrate` has no MOVE_LINEAR branch.
- All Python emit sites (toolhead, extruder, force_move, manual_stepper, trad_rack) use `trapq_append_quintic`.
- `klippy/chelper/__init__.py` FFI `defs_trapq` has no `trapq_append`.
- `pytest test/` fully green.
- klipsim regression corpus byte-identical to pre-Chunk-1 baseline.

## Chunk 1 not-in-scope

- Variable-length `phases[N_MAX=32]` struct upgrade (Chunk 2 prerequisite).
- Any shaper-baking logic.
- Any PA-baking logic.
- `shape_disabled` flag.
- Deletion of `kin_shaper.c` or `kin_extruder.c` convolution loops.

## Next plan

Chunk 2 (bake XY shaper into planner) gets its own `writing-plans` invocation after this Chunk 1 plan completes.
