# Plan 9 — Phase A2a — Jerk-profile → quintic-trapq emitter

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Add a C function `build_jerk_profile_as_quintic_coeffs` (and its trapq-append wrapper) that translates a 1-D `jerk_profile_result` (from Phase A1) into the existing multi-axis quintic-trapq slot layout (`phases[MOVE_MAX_PIECES=32]`, 15-coeff × 4-axis per phase). Unit-tested against direct polynomial evaluation.

**Architecture:** Mirror `linear_quintic.c : build_linear_as_quintic_coeffs` which does the same job for 3-phase trapezoidal moves. The new function takes the jerk-profile's up-to-7 segments (each with up to 4 low-degree monomial coefficients per the 1-D motion) and projects them onto the 3D move direction via `axes_r_{x,y,z}` ratios. Segment count can be 1 (pure cruise) up to 7 (full 7-phase); all fit comfortably inside `MOVE_MAX_PIECES=32`.

**Tech Stack:** C (existing chelper build); Python cffi wrapper; pytest.

**Reference docs:**
- Phase A1 completion: `docs/superpowers/plans/2026-04-24-plan9-phaseA1-completion.md`
- Phase A1 header (jerk_profile_result layout): `klippy/chelper/jerk_profile.h`
- Existing trapezoidal emitter (to mirror): `klippy/chelper/linear_quintic.c : build_linear_as_quintic_coeffs`
- Trapq slot layout: `klippy/chelper/trapq.h`, `trapq_append_quintic`
- Spec: `docs/superpowers/specs/2026-04-24-plan9-greenfield-motion-design.md`

**Commit policy:** per `feedback_plan9_autonomous_mode.md`, commit after each passing test. No `Co-Authored-By` trailer. Use `python3` for pytest invocations on this macOS setup.

---

## File structure

**New files:**
- `test/test_linear_as_jerk_profile.py` — unit tests for the new C emitter

**Modified files:**
- `klippy/chelper/linear_quintic.c` — add `build_jerk_profile_as_quintic_coeffs` (non-static, `__visible`)
- `klippy/chelper/linear_quintic.py` — add Python wrapper function `build_jerk_profile_as_quintic_coeffs`
- `klippy/chelper/__init__.py` — extend `defs_compose` or add `defs_linear_quintic_jerk` with the new function declaration

---

## Task 1: Scaffold emitter signature + failing test

**Files:**
- Modify: `klippy/chelper/linear_quintic.c` (add function stub)
- Modify: `klippy/chelper/linear_quintic.py` (add wrapper stub)
- Modify: `klippy/chelper/__init__.py` (add cffi decl)
- Create: `test/test_linear_as_jerk_profile.py`

### Step 1.1: Inspect the existing trapezoidal emitter for the pattern to mirror

Open `klippy/chelper/linear_quintic.c` and locate `build_linear_as_quintic_coeffs`. Note:
- Signature: takes scalar trapezoidal timings + axes_r ratios + start pos, writes a 180-element coeff buffer (`coeff_buf[180]` = 3 phases × 15 coeffs × 4 axes).
- Coefficient layout: for phase `p`, coeff slot `k` (0..14), axis `a` (0..3), index into flat buffer is `(p * 15 + k) * 4 + a`.
- Axes beyond the 3D move direction (axis 3 = E) get zero.
- For a 1-D-along-direction motion `p(t)` with move direction ratios `(rx, ry, rz)`, the X-axis polynomial is `rx * p(t)`, Y is `ry * p(t)`, Z is `rz * p(t)`. Start-of-segment position offsets are added to the constant term (c0) of each axis.

### Step 1.2: Write the failing test

`test/test_linear_as_jerk_profile.py`:

```python
"""Tests for klippy/chelper/linear_quintic.c::build_jerk_profile_as_quintic_coeffs.

Plan 9 Phase A2a — emitter that translates a 1-D jerk_profile_result into the
multi-axis quintic-trapq slot layout (phases × 15-coeff × 4-axis).
"""
from __future__ import annotations

import math

import pytest

from klippy.chelper import get_ffi, jerk_profile as jp
from klippy.chelper.linear_quintic import (
    build_jerk_profile_as_quintic_coeffs,
)


# ---- helpers --------------------------------------------------------------

def _eval_phase(coeff_buf, phase_idx, axis, t):
    """Horner-eval the 15-coeff polynomial for (phase, axis) at phase-local t."""
    # Coefficients stored ascending: c0, c1, c2, ..., c14
    acc = 0.0
    for k in range(14, -1, -1):
        c = coeff_buf[(phase_idx * 15 + k) * 4 + axis]
        acc = acc * t + c
    return acc


def _eval_phase_deriv(coeff_buf, phase_idx, axis, t, order):
    if order == 0:
        return _eval_phase(coeff_buf, phase_idx, axis, t)
    derived = []
    for k in range(1, 15):
        c = coeff_buf[(phase_idx * 15 + k) * 4 + axis]
        derived.append(c * k)
    # Evaluate derived poly of length 14.
    acc = 0.0
    for k in range(len(derived) - 1, -1, -1):
        acc = acc * t + derived[k]
    if order == 1:
        return acc
    # For order >= 2, recurse by building a fresh coeff_buf-like shape.
    # Simpler: do finite differences here since tests only use order in {0,1,2}.
    # (order=2 tests use explicit poly derivatives below.)
    raise NotImplementedError("order > 1 not needed in these tests")


def _make_single_axis_move():
    """Simple X-only move: v0=0, v1=0, v_peak=200, a_max=5000, j_max=100000, L=50."""
    prof = jp.compute_profile(0.0, 0.0, 200.0, 5000.0, 100000.0, 50.0)
    assert prof.status == jp.JP_OK
    return prof


# ---- tests ----------------------------------------------------------------

def test_emitter_populates_phase_count():
    """The emitter should populate n_phases phases, each with coeffs filled per axis."""
    prof = _make_single_axis_move()
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=(1.0, 0.0, 0.0),         # pure +X motion
        start_pos=(0.0, 0.0, 0.0),
    )
    # Number of phases equals nonzero-T segments in the profile.
    expected = sum(1 for s in prof.segments if s.T > 1e-12)
    assert n_phases == expected, f"n_phases {n_phases} != expected {expected}"
    assert len(phase_t_ends) == n_phases
    assert len(coeff_buf) == 32 * 15 * 4  # MOVE_MAX_PIECES * coeffs * axes


def test_emitter_reproduces_position_on_x_axis():
    """X-axis polynomial evaluated at each phase boundary matches jerk_profile position."""
    prof = _make_single_axis_move()
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=(1.0, 0.0, 0.0),
        start_pos=(0.0, 0.0, 0.0),
    )
    # Running phase-local evaluation: at end of each phase, x should equal the
    # jerk_profile's expected end-of-phase position (which itself builds from
    # segment 0 forward).
    running_x = 0.0
    seg_iter = iter(s for s in prof.segments if s.T > 1e-12)
    for phase_idx in range(n_phases):
        seg = next(seg_iter)
        # Evaluate X at phase-local t = seg.T; should equal running_x after
        # traversing this segment using the jerk_profile's coeffs.
        x_end = _eval_phase(coeff_buf, phase_idx, 0, seg.T)
        # Reference: seg's own polynomial coefficients give p(seg.T).
        seg_local_end = 0.0
        for k in range(len(seg.coeffs) - 1, -1, -1):
            seg_local_end = seg_local_end * seg.T + seg.coeffs[k]
        assert x_end == pytest.approx(seg_local_end, abs=1e-9, rel=1e-9)
        running_x = x_end


def test_emitter_projects_onto_3d_direction():
    """A 3D direction (rx, ry, rz) with |r|=1 produces per-axis polys = r_axis * p(t)."""
    prof = _make_single_axis_move()
    r = (0.6, 0.8, 0.0)  # unit vector in XY plane
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=r,
        start_pos=(0.0, 0.0, 0.0),
    )
    for phase_idx in range(n_phases):
        t = phase_t_ends[phase_idx] - (phase_t_ends[phase_idx - 1] if phase_idx else 0.0)
        # Evaluate mid-phase (t/2) on each axis; ratios should hold.
        tm = t * 0.5
        px = _eval_phase(coeff_buf, phase_idx, 0, tm)
        py = _eval_phase(coeff_buf, phase_idx, 1, tm)
        pz = _eval_phase(coeff_buf, phase_idx, 2, tm)
        # px / rx must equal py / ry (both = p(tm) in 1-D).
        # Avoid division; use cross-check.
        assert px * r[1] == pytest.approx(py * r[0], abs=1e-9, rel=1e-9)
        # pz must be zero since rz == 0.
        assert pz == pytest.approx(0.0, abs=1e-12)


def test_emitter_applies_start_position_offset():
    """Nonzero start_pos shifts each axis's c0 on phase 0."""
    prof = _make_single_axis_move()
    start_pos = (10.0, 20.0, -5.0)
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=(1.0, 0.0, 0.0),
        start_pos=start_pos,
    )
    # At t=0 on phase 0, each axis should equal start_pos[axis].
    assert _eval_phase(coeff_buf, 0, 0, 0.0) == pytest.approx(start_pos[0], abs=1e-12)
    assert _eval_phase(coeff_buf, 0, 1, 0.0) == pytest.approx(start_pos[1], abs=1e-12)
    assert _eval_phase(coeff_buf, 0, 2, 0.0) == pytest.approx(start_pos[2], abs=1e-12)


def test_emitter_rejects_bad_profile():
    """A profile with status != JP_OK should raise."""
    bad = jp.compute_profile(0.0, 0.0, 0.0, 5000.0, 100000.0, 10.0)  # v_peak=0 → JP_BAD_INPUT
    assert bad.status == jp.JP_BAD_INPUT
    with pytest.raises(ValueError):
        build_jerk_profile_as_quintic_coeffs(
            profile=bad, axes_r=(1.0, 0.0, 0.0), start_pos=(0.0, 0.0, 0.0))
```

### Step 1.3: Run and verify failure

Run: `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/test_linear_as_jerk_profile.py -v`
Expected: FAIL on all 5 cases with `ImportError: cannot import name 'build_jerk_profile_as_quintic_coeffs'` (the Python wrapper doesn't exist yet).

### Step 1.4: Add stub to `klippy/chelper/linear_quintic.c`

Append after `build_linear_as_quintic_coeffs`:

```c
/* Translate a 1-D jerk_profile_result into the multi-axis quintic-trapq slot
 * layout. Writes up to MOVE_MAX_PIECES phases into coeff_buf[MOVE_MAX_PIECES*15*4];
 * unused phases are left untouched (caller is expected to zero the buffer).
 *
 * Returns the number of phases emitted, or -1 if the profile is not JP_OK.
 *
 * axes_r: direction ratios (rx, ry, rz). For a unit-norm move vector, |(rx,ry,rz)|=1.
 * start_pos: absolute start position (axis-E / axis 3 is set to 0 by caller).
 * phase_t_ends_out: absolute (cumulative) phase end times, length must be
 * MOVE_MAX_PIECES.
 *
 * Plan 9 Phase A2a. Mirrors build_linear_as_quintic_coeffs for jerk profiles.
 */
__visible int
build_jerk_profile_as_quintic_coeffs(
    const struct jerk_profile_result *prof,
    double rx, double ry, double rz,
    double start_pos_x, double start_pos_y, double start_pos_z,
    double *phase_t_ends_out,
    double *coeff_buf /* [MOVE_MAX_PIECES * 15 * 4] */)
{
    (void)prof; (void)rx; (void)ry; (void)rz;
    (void)start_pos_x; (void)start_pos_y; (void)start_pos_z;
    (void)phase_t_ends_out; (void)coeff_buf;
    return -1;
}
```

Add both `#include "jerk_profile.h"` (for `struct jerk_profile_result` + `JP_OK`) and `#include "trapq.h"` (for `MOVE_MAX_PIECES`) at the top of `linear_quintic.c` if not already present. The file currently includes only `compiler.h` and `linear_quintic.h`.

### Step 1.5: Add cffi declaration in `klippy/chelper/__init__.py`

Append to the `defs_jerk_profile` block (added in Phase A1, contains `struct jerk_profile_result`). cdef declaration order requires the function signature appear after the struct it references; placing it inside `defs_jerk_profile` guarantees that locality.

Append this function declaration to the end of the `defs_jerk_profile` string in `klippy/chelper/__init__.py`:

```c
    int build_jerk_profile_as_quintic_coeffs(
        const struct jerk_profile_result *prof,
        double rx, double ry, double rz,
        double start_pos_x, double start_pos_y, double start_pos_z,
        double *phase_t_ends_out,
        double *coeff_buf);
```

### Step 1.6: Add Python wrapper in `klippy/chelper/linear_quintic.py`

Append:

```python
MOVE_MAX_PIECES = 32
QUINTIC_SLOT_COEFFS = 15
QUINTIC_AXES = 4


def build_jerk_profile_as_quintic_coeffs(profile, axes_r, start_pos):
    """Translate a jerk_profile.Profile into the quintic-trapq slot layout.

    Parameters
    ----------
    profile : klippy.chelper.jerk_profile.Profile
        Result of jerk_profile.compute_profile(); must have status == JP_OK.
    axes_r : tuple of 3 floats
        Move direction ratios (rx, ry, rz). For a unit-norm vector |r| == 1.
    start_pos : tuple of 3 floats
        Start position (sx, sy, sz). Axis E (index 3) is always 0 here.

    Returns
    -------
    (n_phases, phase_t_ends, coeff_buf)
        n_phases: int in [1, 7].
        phase_t_ends: list of n_phases absolute (cumulative) phase end times.
        coeff_buf: list of MOVE_MAX_PIECES * QUINTIC_SLOT_COEFFS * QUINTIC_AXES
            doubles, ready to feed to trapq_append_quintic. Unused phases are
            zero-filled.

    Raises
    ------
    ValueError: if profile.status != JP_OK or axes_r / start_pos are wrong shape.
    """
    from klippy.chelper import jerk_profile as jp_mod  # avoid circular-import footgun
    if profile.status != jp_mod.JP_OK:
        raise ValueError(f"profile status {profile.status} is not JP_OK")
    if len(axes_r) != 3 or len(start_pos) != 3:
        raise ValueError("axes_r and start_pos must be 3-tuples")
    ffi, lib = get_ffi()
    # Build a jerk_profile_result C struct mirroring the profile.
    result_c = ffi.new("struct jerk_profile_result *")
    result_c.status = profile.status
    result_c.n_segments = len(profile.segments)
    result_c.a_acc = profile.a_acc
    result_c.a_dec = profile.a_dec
    result_c.v_hat = profile.v_hat
    for i, seg in enumerate(profile.segments):
        type_int = {"J+": 1, "A+": 2, "J-": 3, "C": 4,
                    "J-d": 5, "A-": 6, "J+d": 7}.get(seg.type, 0)
        result_c.segments[i].type = type_int
        result_c.segments[i].T = seg.T
        for k in range(6):
            result_c.segments[i].coeffs[k] = seg.coeffs[k]
        result_c.segments[i].p0 = seg.p0
        result_c.segments[i].v0 = seg.v0
        result_c.segments[i].a0 = seg.a0
        result_c.segments[i].j = seg.j
    phase_t_ends = ffi.new(f"double[{MOVE_MAX_PIECES}]")
    coeff_buf = ffi.new(
        f"double[{MOVE_MAX_PIECES * QUINTIC_SLOT_COEFFS * QUINTIC_AXES}]")
    # Zero-fill (cffi ffi.new should already zero, but be explicit).
    rx, ry, rz = axes_r
    sx, sy, sz = start_pos
    n_phases = lib.build_jerk_profile_as_quintic_coeffs(
        result_c, rx, ry, rz, sx, sy, sz, phase_t_ends, coeff_buf)
    if n_phases < 0:
        raise RuntimeError("build_jerk_profile_as_quintic_coeffs failed")
    return (n_phases,
            [phase_t_ends[i] for i in range(n_phases)],
            [coeff_buf[i] for i in
             range(MOVE_MAX_PIECES * QUINTIC_SLOT_COEFFS * QUINTIC_AXES)])


# Ensure get_ffi is available at module scope (existing pattern in this file).
from klippy.chelper import get_ffi  # noqa: E402
```

`get_ffi` is already imported at line 2 of `linear_quintic.py` (existing chelper pattern). **Remove the bottom-of-file `from klippy.chelper import get_ffi` instruction** — it's redundant.

### Step 1.7: Run test — expect 1 PASS, 4 ERROR

Run: `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/test_linear_as_jerk_profile.py -v`
Expected result breakdown:
- `test_emitter_rejects_bad_profile` → **PASS** (the wrapper's profile-status check raises `ValueError` before touching C).
- Other 4 tests → **ERROR** (unhandled `RuntimeError` from wrapper when C stub returns `-1`).

This is the desired red state. Do NOT widen test tolerances or swallow exceptions to "fix" the ERRORs — they are by-design placeholders that turn green in Task 2.

Commit the stub + infrastructure:

```bash
git add klippy/chelper/linear_quintic.c klippy/chelper/linear_quintic.py klippy/chelper/__init__.py test/test_linear_as_jerk_profile.py
git commit -m "plan9-A2a: scaffold jerk_profile → quintic emitter"
```

---

## Task 2: Implement the emitter

**Files:**
- Modify: `klippy/chelper/linear_quintic.c` (replace stub body)

### Step 2.1: Replace stub body with real implementation

Replace the stub body of `build_jerk_profile_as_quintic_coeffs` in `klippy/chelper/linear_quintic.c`:

```c
__visible int
build_jerk_profile_as_quintic_coeffs(
    const struct jerk_profile_result *prof,
    double rx, double ry, double rz,
    double start_pos_x, double start_pos_y, double start_pos_z,
    double *phase_t_ends_out,
    double *coeff_buf)
{
    if (prof == NULL || prof->status != JP_OK)
        return -1;
    /* Zero coeff_buf (caller may not have zeroed). */
    for (int i = 0; i < MOVE_MAX_PIECES * 15 * 4; i++)
        coeff_buf[i] = 0.0;
    /* Note: seg->coeffs[0] is absolute-in-1D (set by build_accel_side in
     * jerk_profile.c, which threads *p_cursor as an absolute scalar starting
     * at 0.0 set by jerk_profile_compute). So axis-wise c0 is simply
     * start_pos_<axis> + r_<axis> * seg->coeffs[0]. No per-phase running
     * offset needed. */
    double cum_t = 0.0;
    int out_phase = 0;
    for (int s = 0; s < prof->n_segments; s++) {
        const struct jerk_profile_segment *seg = &prof->segments[s];
        if (seg->T <= 1e-12)
            continue;       /* Skip zero-duration segments. */
        if (out_phase >= MOVE_MAX_PIECES)
            return -1;      /* Too many phases to fit. */
        /* Per-axis polynomial coefficients: ax_c[k] = axis_ratio * seg.coeffs[k]. */
        double *phase_base = coeff_buf + out_phase * 15 * 4;
        for (int k = 0; k < 6; k++) {
            double c_1d = seg->coeffs[k];
            phase_base[k * 4 + 0] = rx * c_1d;
            phase_base[k * 4 + 1] = ry * c_1d;
            phase_base[k * 4 + 2] = rz * c_1d;
            phase_base[k * 4 + 3] = 0.0; /* Axis E not handled here — A5 scope. */
        }
        /* Override c0 with absolute start_pos + axis-ratio * 1-D segment start. */
        phase_base[0 * 4 + 0] = start_pos_x + rx * seg->coeffs[0];
        phase_base[0 * 4 + 1] = start_pos_y + ry * seg->coeffs[0];
        phase_base[0 * 4 + 2] = start_pos_z + rz * seg->coeffs[0];
        cum_t += seg->T;
        phase_t_ends_out[out_phase] = cum_t;
        out_phase++;
    }
    return out_phase;
}
```

**C² continuity note:** per Phase A1's `build_accel_side`, each segment's `coeffs[0]` is the absolute 1-D position at segment start, threaded by `*p_cursor`. So at the end of phase N, position is `start_pos + rx * (coeffs[0] of phase N) + rx * eval(phase N poly at t=T)`; the next phase's start position `start_pos + rx * (coeffs[0] of phase N+1)` = `start_pos + rx * end-position-of-phase-N`. Continuity is inherent — no manual boundary patching needed.

### Step 2.2: Run tests

Run: `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/test_linear_as_jerk_profile.py -v`
Expected: all 5 tests PASS.

If any test fails:
- Check the coefficient indexing: `coeff_buf[(phase * 15 + k) * 4 + axis]` is the flat-index formula. Mismatch here gives garbage on every axis.
- Check sign convention: `seg->coeffs[k]` is ascending-order (c0 + c1*t + c2*t² + c3*t³). The test helpers use the same convention.
- Check that `phase_t_ends_out` holds *cumulative* times (sum of segment durations), not per-phase durations. The test at §Step 1.2 references `phase_t_ends[phase_idx] - (phase_t_ends[phase_idx - 1] if phase_idx else 0.0)` to get per-phase duration, which only works if phase_t_ends is cumulative.

### Step 2.3: Commit

```bash
git add klippy/chelper/linear_quintic.c
git commit -m "plan9-A2a: implement jerk_profile → quintic emitter"
```

---

## Task 3: Integration test — round-trip through trapq_append_quintic

Feed the emitter output to `trapq_append_quintic` and verify the resulting trapq move is position-consistent with the original `jerk_profile_compute` output evaluated at sample times.

**Files:**
- Modify: `test/test_linear_as_jerk_profile.py`

### Step 3.1: Write the round-trip test

Klipper does not currently export a Python `TrapQ` class wrapper, so the round-trip here is: sample positions from `coeff_buf` at key times and verify they match the jerk_profile's own 1-D polynomial evaluation + start_pos offset. (The true end-to-end trapq integration is covered by Phase A2d's `test_plan9_integration.py`.)

Append to `test/test_linear_as_jerk_profile.py`:

```python
def test_roundtrip_eval_matches_profile_sum():
    """Sample positions from coeff_buf at key times; must match jerk_profile's
    own polynomial evaluation + start_pos offset."""
    prof = jp.compute_profile(0.0, 0.0, 200.0, 5000.0, 100000.0, 50.0)
    n_phases, phase_t_ends, coeff_buf = build_jerk_profile_as_quintic_coeffs(
        profile=prof,
        axes_r=(1.0, 0.0, 0.0),
        start_pos=(100.0, 0.0, 0.0),
    )
    segs_nonzero = [s for s in prof.segments if s.T > 1e-12]
    for phase_idx, seg in enumerate(segs_nonzero):
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            local_t = frac * seg.T
            # Direct eval on the segment's 1-D polynomial.
            p_1d = 0.0
            for c in reversed(seg.coeffs):
                p_1d = p_1d * local_t + c
            x_expected = 100.0 + p_1d
            x_from_buf = _eval_phase(coeff_buf, phase_idx, 0, local_t)
            assert x_from_buf == pytest.approx(x_expected, abs=1e-9, rel=1e-9)
```

### Step 3.2: Run test

Run: `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/test_linear_as_jerk_profile.py -v`
Expected: 6/6 PASS (5 from Task 1 + 1 round-trip).

### Step 3.3: Commit

```bash
git add test/test_linear_as_jerk_profile.py
git commit -m "plan9-A2a: round-trip emitter validation test"
```

---

## Task 4: Completion marker

Verify the full suite still passes (jerk_profile + linear_as_jerk_profile + any other impacted tests).

- [ ] **Step 4.1: Run both test files**

Run: `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/test_jerk_profile.py test/test_linear_as_jerk_profile.py -v`
Expected: all passed. `test_jerk_profile.py` has 56 tests as of Phase A1 completion (verify via `pytest --collect-only` if uncertain); `test_linear_as_jerk_profile.py` has 6 (Tasks 1+3 combined: 5 basic + 1 roundtrip). Combined total is 62 unless A1 tests were extended elsewhere.

- [ ] **Step 4.2: Write brief completion report**

Create `docs/superpowers/plans/2026-04-24-plan9-phaseA2a-completion.md`:

```markdown
# Plan 9 Phase A2a — completion report

**Status:** COMPLETE
**Date:** 2026-04-24
**Commits:**
- `<sha>` — plan9-A2a: scaffold jerk_profile → quintic emitter
- `<sha>` — plan9-A2a: implement jerk_profile → quintic emitter
- `<sha>` — plan9-A2a: round-trip emitter validation test

## What shipped

- `klippy/chelper/linear_quintic.c` — new `build_jerk_profile_as_quintic_coeffs` C function (non-static, `__visible`).
- `klippy/chelper/linear_quintic.py` — new Python wrapper `build_jerk_profile_as_quintic_coeffs` with Profile→C-struct marshaling.
- `klippy/chelper/__init__.py` — extended `defs_jerk_profile` cdef with new function declaration.
- `test/test_linear_as_jerk_profile.py` — 6 tests: phase-count, X-axis position fidelity, 3D direction projection, start-position offset, bad-profile rejection, and 1-D round-trip eval.

## Validation

- 6/6 new tests pass.
- Jerk-profile suite (56 tests) still passes.

## Next — A2b

Jerk-aware reachable-velocity math derivation + Python `reachable_v2` function. Replaces the `delta_v2 = 2*move_d*accel` constant-accel approximation in Move.__init__, LookAheadQueue.flush reverse pass, and Move.calc_junction.
```

- [ ] **Step 4.3: Commit completion report**

```bash
git add docs/superpowers/plans/2026-04-24-plan9-phaseA2a-completion.md
git commit -m "plan9-A2a: completion report"
```

---

## References

- Spec: `docs/superpowers/specs/2026-04-24-plan9-greenfield-motion-design.md`
- Phase A1 completion: `docs/superpowers/plans/2026-04-24-plan9-phaseA1-completion.md`
- Phase A1 derivation: `docs/superpowers/plans/2026-04-24-plan9-phaseA1-derivation.md`
- Trapq slot layout: `klippy/chelper/trapq.h` (MOVE_MAX_PIECES=32, 15 coeffs × 4 axes per phase)
- Trapezoidal emitter reference: `klippy/chelper/linear_quintic.c::build_linear_as_quintic_coeffs`
