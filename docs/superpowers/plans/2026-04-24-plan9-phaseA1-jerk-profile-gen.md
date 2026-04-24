# Plan 9 — Phase A1 — Jerk-limited polynomial profile generator

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a C module `jerk_profile` that, given a 1-D move spec (`v0, v1, v_peak, a_max, j_max, L`), emits a piecewise polynomial in time describing the time-optimal jerk-limited motion. Output matches a pre-verified Python reference implementation to within 1e-9 relative error across a full degeneracy sweep.

**Architecture:** Mirror the Python reference (`docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py`) in C. Factor into three layers: (1) `accel_side_timings` primitive computing triangular-or-trapezoidal timing for a one-sided velocity change, (2) `find_v_hat` Newton-Raphson for reduced peak velocity when cruise collapses, (3) top-level `jerk_profile_compute` that sequences accel-group + optional cruise + decel-group and emits per-phase polynomial coefficients. Use `fp64` throughout (fp32 showed unacceptable error in derivation verification).

**Tech Stack:** C (compiled via cffi wrapper in `klippy/chelper/__init__.py`), Python for tests (pytest + numpy), reference impl at `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py`.

**Reference docs:**
- Derivation: `docs/superpowers/plans/2026-04-24-plan9-phaseA1-derivation.md`
- Reference impl: `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py`
- Plan 9 spec: `docs/superpowers/specs/2026-04-24-plan9-greenfield-motion-design.md`

**Commit policy for this plan:** user memory `feedback_plan9_autonomous_mode.md` explicitly overrides the work-hour-commit rule for Plan 9 autonomous execution. Commit after each passing test as specified in each task; no batching required. **No Co-Authored-By trailers** on any commit (per `feedback_no_coauthor_trailer.md`).

---

## File structure

**New files:**
- `klippy/chelper/jerk_profile.c` — C implementation
- `klippy/chelper/jerk_profile.h` — public C header
- `klippy/chelper/jerk_profile.py` — thin cffi wrapper
- `test/test_jerk_profile.py` — parity tests against Python reference

**Modified files:**
- `klippy/chelper/__init__.py` — add `jerk_profile.c` to `SOURCE_FILES`, add cffi `defs_jerk_profile` declarations

**Reference (read-only):**
- `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py` — oracle for tests

---

## Task 1: Scaffolding — headers, stub C, Python wrapper, failing test

**Files:**
- Create: `klippy/chelper/jerk_profile.h`
- Create: `klippy/chelper/jerk_profile.c`
- Create: `klippy/chelper/jerk_profile.py`
- Modify: `klippy/chelper/__init__.py` (add source file + cffi defs)
- Create: `test/test_jerk_profile.py`

- [ ] **Step 1.1: Write the failing test** that imports the wrapper and fails because the C function is a stub.

`test/test_jerk_profile.py`:

```python
"""Parity tests for klippy/chelper/jerk_profile.c against the Python
reference at docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py.

Plan 9 Phase A1 — jerk-limited polynomial profile generator.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

from klippy.chelper import jerk_profile as jp


def _load_reference():
    ref_path = (
        Path(__file__).resolve().parents[1]
        / "docs"
        / "superpowers"
        / "plans"
        / "plan9-derivations"
        / "jerk_profile_ref.py"
    )
    spec = importlib.util.spec_from_file_location("jerk_profile_ref", ref_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


REF = _load_reference()


def test_module_importable():
    """Sanity — wrapper and C symbols load cleanly."""
    assert hasattr(jp, "compute_profile")
```

- [ ] **Step 1.2: Run test and verify it fails**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'klippy.chelper.jerk_profile'`

- [ ] **Step 1.3: Create the C header**

`klippy/chelper/jerk_profile.h`:

```c
/* Plan 9 Phase A1: jerk-limited polynomial profile generator.
 *
 * Given a 1-D single-move spec (v0, v1, v_peak, a_max, j_max, L), produce a
 * piecewise polynomial description of the time-optimal jerk-limited motion.
 *
 * Output layout: up to 7 segments, each with a duration T and ascending-order
 * polynomial coefficients c0..c5 (so p(t) = c0 + c1*t + c2*t^2 + ... + c5*t^5).
 * Degree per segment: jerk phase = 3, const-accel phase = 2, cruise = 1.
 * Coefficients above the polynomial degree are set to 0.0 for safety.
 */
#ifndef JERK_PROFILE_H
#define JERK_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

#define JERK_PROFILE_MAX_SEGMENTS 7
#define JERK_PROFILE_MAX_COEFFS 6

/* Segment type tags, matching the reference implementation. */
enum jerk_profile_seg_type {
    JP_SEG_NONE = 0,
    JP_SEG_JERK_UP_ACC   = 1,   /* 'J+':  accel rising 0 -> a_acc         */
    JP_SEG_CONST_ACC     = 2,   /* 'A+':  constant accel at a_acc         */
    JP_SEG_JERK_DOWN_ACC = 3,   /* 'J-':  accel falling a_acc -> 0        */
    JP_SEG_CRUISE        = 4,   /* 'C':   constant velocity v_hat         */
    JP_SEG_JERK_DOWN_DEC = 5,   /* 'J-d': accel falling 0 -> -a_dec       */
    JP_SEG_CONST_DEC     = 6,   /* 'A-':  constant decel at -a_dec        */
    JP_SEG_JERK_UP_DEC   = 7,   /* 'J+d': accel rising -a_dec -> 0        */
};

/* Result status codes. */
enum jerk_profile_status {
    JP_OK           = 0,
    JP_INFEASIBLE   = 1,   /* L < d_floor: cannot achieve v1 from v0 within limits */
    JP_BAD_INPUT    = 2,   /* NaN / negative / nonsense inputs                      */
};

struct jerk_profile_segment {
    int type;                                  /* enum jerk_profile_seg_type */
    double T;                                  /* segment duration (s)        */
    double coeffs[JERK_PROFILE_MAX_COEFFS];    /* ascending: c0, c1, ..., c5  */
    /* Diagnostic state at segment start (not required for replay but handy). */
    double p0;
    double v0;
    double a0;
    double j;
};

struct jerk_profile_result {
    int status;                                /* enum jerk_profile_status    */
    int n_segments;
    struct jerk_profile_segment segments[JERK_PROFILE_MAX_SEGMENTS];
    /* Diagnostics. */
    double a_acc;
    double a_dec;
    double v_hat;
};

/* Main entry point. Inputs must be: v0 >= 0, v1 >= 0, v_peak >= max(v0, v1),
 * a_max > 0, j_max > 0, L > 0. Returns JP_OK on success, error code otherwise.
 */
int jerk_profile_compute(
    double v0,
    double v1,
    double v_peak,
    double a_max,
    double j_max,
    double L,
    struct jerk_profile_result *out);

/* Sub-primitives exposed for testing. */
void jerk_profile_accel_side_timings(
    double v_start,
    double v_end,
    double a_max,
    double j_max,
    double *out_t_j,
    double *out_t_a,
    double *out_a_peak,
    double *out_dist);

double jerk_profile_find_v_hat(
    double v0,
    double v1,
    double v_peak,
    double a_max,
    double j_max,
    double L);

#ifdef __cplusplus
}
#endif

#endif /* JERK_PROFILE_H */
```

- [ ] **Step 1.4: Create the stub C implementation**

`klippy/chelper/jerk_profile.c`:

```c
/* Plan 9 Phase A1: jerk-limited polynomial profile generator.
 *
 * Implementation mirrors docs/superpowers/plans/plan9-derivations/
 * jerk_profile_ref.py (the pre-verified Python reference).
 */
#include <math.h>
#include <string.h>

#include "compiler.h" // __visible
#include "jerk_profile.h"

static const double JP_EPS = 1e-12;

__visible void
jerk_profile_accel_side_timings(double v_start, double v_end,
                                double a_max, double j_max,
                                double *out_t_j, double *out_t_a,
                                double *out_a_peak, double *out_dist)
{
    (void)v_start; (void)v_end; (void)a_max; (void)j_max;
    *out_t_j = 0.0;
    *out_t_a = 0.0;
    *out_a_peak = 0.0;
    *out_dist = 0.0;
}

__visible double
jerk_profile_find_v_hat(double v0, double v1, double v_peak,
                        double a_max, double j_max, double L)
{
    (void)v0; (void)v1; (void)a_max; (void)j_max; (void)L;
    return v_peak;
}

__visible int
jerk_profile_compute(double v0, double v1, double v_peak,
                     double a_max, double j_max, double L,
                     struct jerk_profile_result *out)
{
    (void)v0; (void)v1; (void)v_peak;
    (void)a_max; (void)j_max; (void)L;
    memset(out, 0, sizeof(*out));
    out->status = JP_BAD_INPUT;
    return JP_BAD_INPUT;
}
```

- [ ] **Step 1.5: Create the Python wrapper**

`klippy/chelper/jerk_profile.py`:

```python
"""Python wrapper around klippy/chelper/jerk_profile.c.

Plan 9 Phase A1: jerk-limited polynomial profile generator.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from klippy.chelper import get_ffi

JP_MAX_SEGMENTS = 7
JP_MAX_COEFFS = 6

# Match enum jerk_profile_seg_type in jerk_profile.h.
SEG_TYPE_NAMES = {
    1: "J+",
    2: "A+",
    3: "J-",
    4: "C",
    5: "J-d",
    6: "A-",
    7: "J+d",
}

JP_OK = 0
JP_INFEASIBLE = 1
JP_BAD_INPUT = 2


@dataclass
class Segment:
    type: str
    T: float
    coeffs: List[float] = field(default_factory=list)
    p0: float = 0.0
    v0: float = 0.0
    a0: float = 0.0
    j: float = 0.0


@dataclass
class Profile:
    status: int
    segments: List[Segment] = field(default_factory=list)
    a_acc: float = 0.0
    a_dec: float = 0.0
    v_hat: float = 0.0


def accel_side_timings(v_start: float, v_end: float, a_max: float, j_max: float):
    """Call the C accel_side_timings primitive. Returns (t_j, t_a, a_peak, dist)."""
    ffi, lib = get_ffi()
    out_t_j = ffi.new("double[1]")
    out_t_a = ffi.new("double[1]")
    out_a_peak = ffi.new("double[1]")
    out_dist = ffi.new("double[1]")
    lib.jerk_profile_accel_side_timings(
        v_start, v_end, a_max, j_max,
        out_t_j, out_t_a, out_a_peak, out_dist)
    return out_t_j[0], out_t_a[0], out_a_peak[0], out_dist[0]


def find_v_hat(v0: float, v1: float, v_peak: float,
               a_max: float, j_max: float, L: float) -> float:
    """Call the C find_v_hat Newton-Raphson for reduced peak velocity."""
    _, lib = get_ffi()
    return lib.jerk_profile_find_v_hat(v0, v1, v_peak, a_max, j_max, L)


def compute_profile(v0: float, v1: float, v_peak: float,
                    a_max: float, j_max: float, L: float) -> Profile:
    """Compute the full jerk-limited profile. Returns a Profile dataclass."""
    ffi, lib = get_ffi()
    result = ffi.new("struct jerk_profile_result *")
    status = lib.jerk_profile_compute(v0, v1, v_peak, a_max, j_max, L, result)
    prof = Profile(status=status,
                   a_acc=result.a_acc,
                   a_dec=result.a_dec,
                   v_hat=result.v_hat)
    for i in range(result.n_segments):
        c_seg = result.segments[i]
        seg = Segment(
            type=SEG_TYPE_NAMES.get(c_seg.type, "?"),
            T=c_seg.T,
            coeffs=[c_seg.coeffs[k] for k in range(JP_MAX_COEFFS)],
            p0=c_seg.p0, v0=c_seg.v0, a0=c_seg.a0, j=c_seg.j,
        )
        prof.segments.append(seg)
    return prof
```

- [ ] **Step 1.6: Register module in chelper `__init__.py`**

Four edits in `klippy/chelper/__init__.py`:

**1.** Add `"jerk_profile.c"` to the `SOURCE_FILES` list (around line 22). Insert it after `"nonlinear_pa_compose.c"` for topical grouping:

```python
SOURCE_FILES = [
    ...,
    "nonlinear_pa_compose.c",
    "jerk_profile.c",
    ...,
]
```

**2.** Add `"jerk_profile.h"` to the `OTHER_FILES` list (around line 51), next to the other `.h` files in the list.

**3.** Declare a new top-level `defs_jerk_profile` string. Place it **immediately after the `defs_compose` block** (after line 246, before the first `defs_kin_*` block). No struct-level enum declarations needed — the struct fields are plain `int`, and the wrapper hard-codes the enum values:

```python
defs_jerk_profile = """
    struct jerk_profile_segment {
        int type;
        double T;
        double coeffs[6];
        double p0;
        double v0;
        double a0;
        double j;
    };
    struct jerk_profile_result {
        int status;
        int n_segments;
        struct jerk_profile_segment segments[7];
        double a_acc;
        double a_dec;
        double v_hat;
    };
    int jerk_profile_compute(double v0, double v1, double v_peak,
        double a_max, double j_max, double L,
        struct jerk_profile_result *out);
    void jerk_profile_accel_side_timings(double v_start, double v_end,
        double a_max, double j_max,
        double *out_t_j, double *out_t_a,
        double *out_a_peak, double *out_dist);
    double jerk_profile_find_v_hat(double v0, double v1, double v_peak,
        double a_max, double j_max, double L);
"""
```

**4.** Register in the `defs_all` list (around line 367). This is a Python list that the cffi wrapper iterates to call `FFI_main.cdef(d)` per block. Add `defs_jerk_profile` alongside `defs_compose`:

```python
defs_all = [
    ...,
    defs_compose,
    defs_jerk_profile,
    ...,
]
```

- [ ] **Step 1.7: Run test and verify it now passes**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py::test_module_importable -v`
Expected: PASS. The C library builds (gcc invoked by `get_ffi`), the cffi binding resolves all declared symbols, the wrapper imports. If gcc errors: fix compile errors before proceeding.

- [ ] **Step 1.8: Commit scaffolding**

```bash
git add klippy/chelper/jerk_profile.h klippy/chelper/jerk_profile.c klippy/chelper/jerk_profile.py klippy/chelper/__init__.py test/test_jerk_profile.py
git commit -m "plan9-A1: scaffold jerk_profile module"
```

---

## Task 2: Implement `accel_side_timings`

Mirror `accel_side_timings` from the Python reference (lines 47-83 of `jerk_profile_ref.py`). Handles both triangular (accel never reaches `a_max`) and trapezoidal (accel hits `a_max` and holds) cases for a one-sided velocity change.

**Files:**
- Modify: `klippy/chelper/jerk_profile.c` (replace stub)
- Modify: `test/test_jerk_profile.py` (add parity test)

- [ ] **Step 2.1: Write the failing parity test**

Append to `test/test_jerk_profile.py`:

```python
import pytest


_ACCEL_CASES = [
    # (v_start, v_end, a_max, j_max, description)
    (0.0,   100.0, 5000.0, 100000.0, "zero to 100"),
    (100.0, 0.0,   5000.0, 100000.0, "100 to zero (decel)"),
    (0.0,   500.0, 5000.0, 100000.0, "zero to 500 (trapezoidal)"),
    (0.0,   50.0,  5000.0, 100000.0, "zero to 50 (triangular, small dv)"),
    (200.0, 200.0, 5000.0, 100000.0, "no change (dv == 0)"),
    (300.0, 100.0, 3000.0, 50000.0,  "decel, different limits"),
    (0.0,   250.0, 2500.0, 25000.0,  "exactly at trap/tri boundary"),
]


@pytest.mark.parametrize(
    "v_start,v_end,a_max,j_max,desc", _ACCEL_CASES,
    ids=[c[4] for c in _ACCEL_CASES])
def test_accel_side_timings_matches_reference(v_start, v_end, a_max, j_max, desc):
    t_j_c, t_a_c, a_p_c, d_c = jp.accel_side_timings(v_start, v_end, a_max, j_max)
    t_j_r, t_a_r, a_p_r, d_r = REF.accel_side_timings(v_start, v_end, a_max, j_max)
    # All four returned quantities must match to 1e-12 (same math on same CPU fp64).
    assert t_j_c == pytest.approx(t_j_r, abs=1e-12), f"t_j mismatch ({desc})"
    assert t_a_c == pytest.approx(t_a_r, abs=1e-12), f"t_a mismatch ({desc})"
    assert a_p_c == pytest.approx(a_p_r, abs=1e-12), f"a_peak mismatch ({desc})"
    assert d_c   == pytest.approx(d_r,   abs=1e-9),  f"dist mismatch ({desc})"
```

- [ ] **Step 2.2: Run and verify failure**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py::test_accel_side_timings_matches_reference -v`
Expected: FAIL — stub returns all zeros, reference returns real values.

- [ ] **Step 2.3: Implement `jerk_profile_accel_side_timings`**

In `klippy/chelper/jerk_profile.c`, replace the stub body of `jerk_profile_accel_side_timings` with:

```c
__visible void
jerk_profile_accel_side_timings(double v_start, double v_end,
                                double a_max, double j_max,
                                double *out_t_j, double *out_t_a,
                                double *out_a_peak, double *out_dist)
{
    double dv = v_end - v_start;
    if (dv < 0.0)
        dv = -dv;
    if (dv < JP_EPS) {
        *out_t_j = 0.0;
        *out_t_a = 0.0;
        *out_a_peak = 0.0;
        *out_dist = 0.0;
        return;
    }
    double dv_tri = (a_max * a_max) / j_max;
    double t_j, t_a, a_p;
    if (dv >= dv_tri) {
        t_j = a_max / j_max;
        t_a = (dv - dv_tri) / a_max;
        a_p = a_max;
    } else {
        a_p = sqrt(j_max * dv);
        t_j = a_p / j_max;
        t_a = 0.0;
    }
    double T = 2.0 * t_j + t_a;
    double d = 0.5 * (v_start + v_end) * T;
    *out_t_j = t_j;
    *out_t_a = t_a;
    *out_a_peak = a_p;
    *out_dist = d;
}
```

- [ ] **Step 2.4: Run and verify pass**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py::test_accel_side_timings_matches_reference -v`
Expected: PASS on all 7 parametrized cases.

- [ ] **Step 2.5: Commit**

```bash
git add klippy/chelper/jerk_profile.c test/test_jerk_profile.py
git commit -m "plan9-A1: implement accel_side_timings"
```

---

## Task 3: Implement `find_v_hat` (reduced peak velocity via Newton/bisection)

Python reference lines 95-135. Finds `v_hat` in `[max(v0,v1), v_peak]` such that the total accel-distance + decel-distance equals `L`. Use bisection since the function is monotonic and bracketed; Newton-Raphson is faster but bisection is simpler and robust. Derivation §Part 3 justifies bisection as adequate.

**Files:**
- Modify: `klippy/chelper/jerk_profile.c`
- Modify: `test/test_jerk_profile.py`

- [ ] **Step 3.1: Write the failing parity test**

Append to `test/test_jerk_profile.py`:

```python
# Cases where cruise collapses — find_v_hat must return something < v_peak.
_V_HAT_CASES = [
    # (v0, v1, v_peak, a_max, j_max, L, desc)
    (0.0, 0.0, 500.0, 5000.0, 100000.0, 10.0,  "short symmetric"),
    (0.0, 100.0, 500.0, 5000.0, 100000.0, 15.0, "short asymmetric"),
    (50.0, 150.0, 500.0, 3000.0, 50000.0, 20.0, "both endpoints nonzero"),
    (200.0, 200.0, 500.0, 5000.0, 100000.0, 8.0, "endpoints equal, nonzero"),
]


@pytest.mark.parametrize(
    "v0,v1,v_peak,a_max,j_max,L,desc", _V_HAT_CASES,
    ids=[c[6] for c in _V_HAT_CASES])
def test_find_v_hat_matches_reference(v0, v1, v_peak, a_max, j_max, L, desc):
    v_hat_c = jp.find_v_hat(v0, v1, v_peak, a_max, j_max, L)
    # Reference's find_v_hat has signature (v0, v1, a_max, j_max, L) — it does
    # NOT take v_peak (brackets by doubling from max(v0,v1)). The C uses v_peak
    # as v_hi instead. Both converge to the same root.
    v_hat_r = REF.find_v_hat(v0, v1, a_max, j_max, L)
    assert v_hat_c == pytest.approx(v_hat_r, rel=1e-9, abs=1e-9), \
        f"v_hat mismatch ({desc}): C={v_hat_c}, ref={v_hat_r}"
    # Sanity: v_hat must be in [max(v0,v1), v_peak].
    assert v_hat_c >= max(v0, v1) - 1e-9
    assert v_hat_c <= v_peak + 1e-9
```

- [ ] **Step 3.2: Run and verify failure**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py::test_find_v_hat_matches_reference -v`
Expected: FAIL — stub returns `v_peak` unchanged.

- [ ] **Step 3.3: Implement `jerk_profile_find_v_hat`**

Replace the stub body in `klippy/chelper/jerk_profile.c`:

```c
/* Distance covered by a one-sided accel group (v_start -> v_end), no jerk
 * limit care here — we just compute (v_start+v_end)/2 * T where T is the
 * group's total duration under (a_max, j_max). Reused inside find_v_hat. */
static double
accel_side_distance(double v_start, double v_end, double a_max, double j_max)
{
    double t_j, t_a, a_p, d;
    jerk_profile_accel_side_timings(v_start, v_end, a_max, j_max,
                                    &t_j, &t_a, &a_p, &d);
    return d;
}

__visible double
jerk_profile_find_v_hat(double v0, double v1, double v_peak,
                        double a_max, double j_max, double L)
{
    double v_lo = (v0 > v1) ? v0 : v1;
    double v_hi = v_peak;
    /* If full-peak is already feasible (caller mis-used us), return v_peak. */
    double d_full = accel_side_distance(v0, v_peak, a_max, j_max)
                  + accel_side_distance(v_peak, v1, a_max, j_max);
    if (d_full <= L + JP_EPS)
        return v_peak;
    /* Target: residual(v_hat) = d_acc(v0 -> v_hat) + d_dec(v_hat -> v1) - L.
     * Monotonically increasing in v_hat over [v_lo, v_hi]. Bisect. */
    for (int iter = 0; iter < 80; iter++) {
        double v_mid = 0.5 * (v_lo + v_hi);
        double d_mid = accel_side_distance(v0, v_mid, a_max, j_max)
                     + accel_side_distance(v_mid, v1, a_max, j_max);
        if (d_mid > L)
            v_hi = v_mid;
        else
            v_lo = v_mid;
        if ((v_hi - v_lo) < 1e-12 * (v_hi + 1.0))
            break;
    }
    return 0.5 * (v_lo + v_hi);
}
```

- [ ] **Step 3.4: Run and verify pass**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py::test_find_v_hat_matches_reference -v`
Expected: PASS on all 4 parametrized cases.

- [ ] **Step 3.5: Commit**

```bash
git add klippy/chelper/jerk_profile.c test/test_jerk_profile.py
git commit -m "plan9-A1: implement find_v_hat bisection"
```

---

## Task 4: Implement polynomial-coefficient builders for accel-side segments

For a full accel side `(v_start -> v_end)` with timings from `accel_side_timings`, emit up to three segments: `JP_SEG_JERK_UP_ACC` (degree 3), optionally `JP_SEG_CONST_ACC` (degree 2), and `JP_SEG_JERK_DOWN_ACC` (degree 3). Each segment carries polynomial coefficients such that `p(t)` is continuous across boundaries and begins at position 0, velocity `v_start`, accel 0 at segment 1 entry.

Reference impl: **the accel/decel/cruise segment emission is split across three nested closures inside `compute_profile`** in `jerk_profile_ref.py`: `emit_jerk_phase` / `emit_const_accel_phase` / `emit_cruise` (roughly lines 225-257), invoked in a 7-call sequence at lines ~260-268. There is NO single `build_accel_side` function in the reference — the factoring is a C-side convenience introduced in this plan. The reference's `poly_const_jerk` returns a 4-tuple `(c0, c1, c2, c3)`; the C struct's `coeffs[6]` must zero-pad the high-degree slots.

**Files:**
- Modify: `klippy/chelper/jerk_profile.c`
- Modify: `test/test_jerk_profile.py`

- [ ] **Step 4.1: Re-read the reference implementation section for accel-side segment emission**

Open `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py` and locate the function that builds segments for one side (accel or decel). Note the exact coefficient formulas for each segment type. The math is:

- **JP_SEG_JERK_UP_ACC** (constant jerk `+j_max`, duration `t_j`, entry `(p0, v_start, 0)`):
  - `p(t) = p0 + v_start*t + 0 + (j/6)*t^3`
  - coeffs: `[p0, v_start, 0, j_max/6, 0, 0]`
- **JP_SEG_CONST_ACC** (constant accel `a_peak`, duration `t_a`, entry `(p1, v1, a_peak)`):
  - `p(t) = p1 + v1*t + (a_peak/2)*t^2`
  - coeffs: `[p1, v1, a_peak/2, 0, 0, 0]`
- **JP_SEG_JERK_DOWN_ACC** (constant jerk `-j_max`, duration `t_j`, entry `(p2, v2, a_peak)`):
  - `p(t) = p2 + v2*t + (a_peak/2)*t^2 + (-j/6)*t^3`
  - coeffs: `[p2, v2, a_peak/2, -j_max/6, 0, 0]`

Verify these against the reference file before writing code. If reference uses `+j_max` versus `-j_max` signs differently, match the reference exactly.

- [ ] **Step 4.2: Add a helper test that exercises per-segment polynomial continuity**

Append to `test/test_jerk_profile.py`:

```python
def _eval_poly(coeffs, t):
    """Horner evaluation on ascending-order coefficients."""
    acc = 0.0
    for c in reversed(coeffs):
        acc = acc * t + c
    return acc


def _eval_poly_deriv(coeffs, t, order):
    """order-th derivative at t. order in {0,1,2}."""
    if order == 0:
        return _eval_poly(coeffs, t)
    # d/dt of ascending-order [c0, c1, c2, c3, c4, c5] -> [c1, 2c2, 3c3, 4c4, 5c5]
    derived = [coeffs[k+1] * (k+1) for k in range(len(coeffs)-1)]
    return _eval_poly_deriv(derived + [0.0], t, order - 1)


# A few fully-specified cases where we know the full 7-phase profile exists.
_COMPUTE_CASES_FULL_7 = [
    # (v0, v1, v_peak, a_max, j_max, L, desc)
    (0.0, 0.0, 200.0, 5000.0, 100000.0, 100.0, "symmetric zero-to-zero, long"),
    (0.0, 100.0, 200.0, 5000.0, 100000.0, 80.0, "asym, non-cruise collapse"),
]


@pytest.mark.parametrize(
    "v0,v1,v_peak,a_max,j_max,L,desc", _COMPUTE_CASES_FULL_7,
    ids=[c[6] for c in _COMPUTE_CASES_FULL_7])
def test_profile_c2_continuity(v0, v1, v_peak, a_max, j_max, L, desc):
    prof = jp.compute_profile(v0, v1, v_peak, a_max, j_max, L)
    assert prof.status == jp.JP_OK, f"status {prof.status} ({desc})"
    # C0, C1, C2 continuity across every interior segment boundary.
    for i in range(len(prof.segments) - 1):
        s = prof.segments[i]
        t = prof.segments[i + 1]
        p_end = _eval_poly_deriv(s.coeffs, s.T, 0)
        v_end = _eval_poly_deriv(s.coeffs, s.T, 1)
        a_end = _eval_poly_deriv(s.coeffs, s.T, 2)
        p_next = _eval_poly_deriv(t.coeffs, 0.0, 0)
        v_next = _eval_poly_deriv(t.coeffs, 0.0, 1)
        a_next = _eval_poly_deriv(t.coeffs, 0.0, 2)
        assert p_end == pytest.approx(p_next, abs=1e-9), f"p jump at seg {i}"
        assert v_end == pytest.approx(v_next, abs=1e-9), f"v jump at seg {i}"
        assert a_end == pytest.approx(a_next, abs=1e-9), f"a jump at seg {i}"
    # End conditions: v(T_last) == v1, a(T_last) == 0.
    last = prof.segments[-1]
    assert _eval_poly_deriv(last.coeffs, last.T, 1) == pytest.approx(v1, abs=1e-9)
    assert _eval_poly_deriv(last.coeffs, last.T, 2) == pytest.approx(0.0, abs=1e-9)
    # Sum of durations * .. well, end position must equal L.
    assert _eval_poly_deriv(last.coeffs, last.T, 0) == pytest.approx(L, abs=1e-9)
```

- [ ] **Step 4.3: Run and verify failure**

Expected: FAIL — `compute_profile` still returns JP_BAD_INPUT. This test becomes the acceptance gate for Task 5.

- [ ] **Step 4.4: Add a static helper `build_accel_side` in `jerk_profile.c`**

Below `accel_side_distance`, add (note: positions are built on an incremental `p0` cursor, not globals):

```c
/* Append up to three segments (J+, A+, J-) describing the one-sided speed
 * change v_start -> v_end under (a_max, j_max). Segment 1 starts at state
 * (p0, v_start, 0). On return, *p_cursor, *v_cursor, *a_cursor are updated
 * to the state at the *end* of the last emitted segment. n_segments is
 * incremented. Returns the accel peak (a_p). If dv == 0, emits nothing and
 * returns 0.
 */
static double
build_accel_side(double v_start, double v_end, double a_max, double j_max,
                 struct jerk_profile_segment *segs, int *n_segments,
                 double *p_cursor, double *v_cursor, double *a_cursor)
{
    double t_j, t_a, a_p, dist;
    jerk_profile_accel_side_timings(v_start, v_end, a_max, j_max,
                                    &t_j, &t_a, &a_p, &dist);
    if (t_j < JP_EPS && t_a < JP_EPS)
        return 0.0;
    double sign = (v_end >= v_start) ? +1.0 : -1.0;
    double j = sign * j_max;
    /* Segment 1: J+ (jerk-up, accel rising 0 -> sign*a_p). */
    struct jerk_profile_segment *s = &segs[(*n_segments)++];
    s->type = (sign > 0) ? JP_SEG_JERK_UP_ACC : JP_SEG_JERK_DOWN_DEC;
    s->T = t_j;
    s->coeffs[0] = *p_cursor;
    s->coeffs[1] = *v_cursor;
    s->coeffs[2] = 0.0;
    s->coeffs[3] = j / 6.0;
    s->coeffs[4] = 0.0;
    s->coeffs[5] = 0.0;
    s->p0 = *p_cursor; s->v0 = *v_cursor; s->a0 = 0.0; s->j = j;
    /* Advance cursor to end of segment 1. */
    double p1 = *p_cursor + *v_cursor * t_j + (j / 6.0) * t_j * t_j * t_j;
    double v1s = *v_cursor + 0.5 * j * t_j * t_j;
    double a1 = j * t_j;    /* == sign * a_p */
    *p_cursor = p1; *v_cursor = v1s; *a_cursor = a1;
    /* Segment 2: A+ (const-accel) only if t_a > 0. */
    if (t_a > JP_EPS) {
        s = &segs[(*n_segments)++];
        s->type = (sign > 0) ? JP_SEG_CONST_ACC : JP_SEG_CONST_DEC;
        s->T = t_a;
        s->coeffs[0] = *p_cursor;
        s->coeffs[1] = *v_cursor;
        s->coeffs[2] = 0.5 * a1;
        s->coeffs[3] = 0.0;
        s->coeffs[4] = 0.0;
        s->coeffs[5] = 0.0;
        s->p0 = *p_cursor; s->v0 = *v_cursor; s->a0 = a1; s->j = 0.0;
        double p2 = *p_cursor + *v_cursor * t_a + 0.5 * a1 * t_a * t_a;
        double v2 = *v_cursor + a1 * t_a;
        *p_cursor = p2; *v_cursor = v2;
    }
    /* Segment 3: J- (jerk-down, accel falling sign*a_p -> 0). */
    s = &segs[(*n_segments)++];
    s->type = (sign > 0) ? JP_SEG_JERK_DOWN_ACC : JP_SEG_JERK_UP_DEC;
    s->T = t_j;
    s->coeffs[0] = *p_cursor;
    s->coeffs[1] = *v_cursor;
    s->coeffs[2] = 0.5 * a1;
    s->coeffs[3] = -j / 6.0;
    s->coeffs[4] = 0.0;
    s->coeffs[5] = 0.0;
    s->p0 = *p_cursor; s->v0 = *v_cursor; s->a0 = a1; s->j = -j;
    double p3 = *p_cursor + *v_cursor * t_j + 0.5 * a1 * t_j * t_j
              + (-j / 6.0) * t_j * t_j * t_j;
    double v3 = *v_cursor + a1 * t_j + 0.5 * (-j) * t_j * t_j;
    /* a should return to 0. */
    *p_cursor = p3; *v_cursor = v3; *a_cursor = 0.0;
    return a_p;
}
```

- [ ] **Step 4.5: Run the C2-continuity test**

After Task 5 is done (which wires `build_accel_side` into `jerk_profile_compute`), this test should pass. For now it's expected to still fail — leave the expectation.

- [ ] **Step 4.6: Commit (helper only — compute still stub)**

```bash
git add klippy/chelper/jerk_profile.c
git commit -m "plan9-A1: add build_accel_side helper"
```

---

## Task 5: Implement top-level `jerk_profile_compute`

Stitches: accel-side (v0 -> v_hat) + cruise (v_hat constant over remaining distance) + decel-side (v_hat -> v1). When cruise-collapse triggers, `v_hat < v_peak` found via `find_v_hat`.

**Files:**
- Modify: `klippy/chelper/jerk_profile.c`
- Modify: `test/test_jerk_profile.py`

- [ ] **Step 5.1: Re-read derivation Part 2 (degeneracy dispatch)**

Open `docs/superpowers/plans/2026-04-24-plan9-phaseA1-derivation.md` §Part 2 and confirm the dispatch logic:
1. Compute `d_floor` = minimum distance feasible under (v0, v1, a_max, j_max). If `L < d_floor`, return `JP_INFEASIBLE`.
2. Compute `d_full_peak` = distance if we hit `v_peak` on both sides. If `L >= d_full_peak`, cruise phase is non-zero; `v_hat = v_peak`, cruise duration = `(L - d_full_peak) / v_peak`.
3. Else, `v_hat < v_peak`; use `find_v_hat` to solve for it; no cruise segment.

- [ ] **Step 5.2: Implement `jerk_profile_compute`**

Replace the stub body in `klippy/chelper/jerk_profile.c`:

```c
__visible int
jerk_profile_compute(double v0, double v1, double v_peak,
                     double a_max, double j_max, double L,
                     struct jerk_profile_result *out)
{
    memset(out, 0, sizeof(*out));
    /* Input validation. */
    if (!(v0 >= 0.0) || !(v1 >= 0.0) || !(v_peak > 0.0)
        || !(a_max > 0.0) || !(j_max > 0.0) || !(L > 0.0)
        || v0 > v_peak + JP_EPS || v1 > v_peak + JP_EPS) {
        out->status = JP_BAD_INPUT;
        return JP_BAD_INPUT;
    }
    /* Feasibility: d_floor = accel(v0 -> max(v0,v1)) + accel(max(v0,v1) -> v1)
     * (one side ramps up, the other down, by a trivial min-distance path). */
    double v_mid = (v0 > v1) ? v0 : v1;
    double d_floor = accel_side_distance(v0, v_mid, a_max, j_max)
                   + accel_side_distance(v_mid, v1, a_max, j_max);
    if (L + JP_EPS < d_floor) {
        out->status = JP_INFEASIBLE;
        out->v_hat = v_mid;
        return JP_INFEASIBLE;
    }
    /* Does full-peak fit? */
    double d_full = accel_side_distance(v0, v_peak, a_max, j_max)
                  + accel_side_distance(v_peak, v1, a_max, j_max);
    double v_hat;
    int have_cruise = 0;
    double cruise_T = 0.0;
    if (L + JP_EPS >= d_full) {
        v_hat = v_peak;
        cruise_T = (L - d_full) / v_peak;
        have_cruise = (cruise_T > JP_EPS);
    } else {
        v_hat = jerk_profile_find_v_hat(v0, v1, v_peak, a_max, j_max, L);
    }
    out->v_hat = v_hat;
    /* Build accel side (v0 -> v_hat). */
    double p_cur = 0.0, v_cur = v0, a_cur = 0.0;
    double a_acc = build_accel_side(v0, v_hat, a_max, j_max,
                                    out->segments, &out->n_segments,
                                    &p_cur, &v_cur, &a_cur);
    out->a_acc = a_acc;
    /* Cruise (if any). */
    if (have_cruise) {
        struct jerk_profile_segment *s = &out->segments[out->n_segments++];
        s->type = JP_SEG_CRUISE;
        s->T = cruise_T;
        s->coeffs[0] = p_cur;
        s->coeffs[1] = v_cur;
        s->coeffs[2] = 0.0;
        s->coeffs[3] = 0.0;
        s->coeffs[4] = 0.0;
        s->coeffs[5] = 0.0;
        s->p0 = p_cur; s->v0 = v_cur; s->a0 = 0.0; s->j = 0.0;
        p_cur += v_cur * cruise_T;
        /* v, a unchanged. */
    }
    /* Build decel side (v_hat -> v1). */
    double a_dec = build_accel_side(v_hat, v1, a_max, j_max,
                                    out->segments, &out->n_segments,
                                    &p_cur, &v_cur, &a_cur);
    out->a_dec = a_dec;
    out->status = JP_OK;
    return JP_OK;
}
```

- [ ] **Step 5.3: Run the C2-continuity test**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py::test_profile_c2_continuity -v`
Expected: PASS on both parametrized cases.

- [ ] **Step 5.4: Commit**

```bash
git add klippy/chelper/jerk_profile.c
git commit -m "plan9-A1: implement jerk_profile_compute dispatch"
```

---

## Task 6: Full parity sweep against Python reference

Run the 36-case sweep from the derivation (v0 in {0, 50, 200}, v1 in {0, 50, 200}, L in {1, 10, 100, 1000}, fixed `a_max=5000`, `j_max=100000`, `v_peak=500`). For every feasible case, the C output must match the Python reference.

**Files:**
- Modify: `test/test_jerk_profile.py`

- [ ] **Step 6.1: Write the sweep test**

Append to `test/test_jerk_profile.py`:

```python
import itertools

_SWEEP_V0 = [0.0, 50.0, 200.0]
_SWEEP_V1 = [0.0, 50.0, 200.0]
_SWEEP_L  = [1.0, 10.0, 100.0, 1000.0]
_SWEEP = list(itertools.product(_SWEEP_V0, _SWEEP_V1, _SWEEP_L))


@pytest.mark.parametrize("v0,v1,L", _SWEEP, ids=[f"v0={v0},v1={v1},L={L}"
                                                  for v0, v1, L in _SWEEP])
def test_sweep_parity_vs_reference(v0, v1, L):
    V_PEAK = 500.0
    A_MAX = 5000.0
    J_MAX = 100000.0
    # Reference — may return a Profile with feasible=False.
    ref = REF.compute_profile(v0, v1, V_PEAK, A_MAX, J_MAX, L)
    c = jp.compute_profile(v0, v1, V_PEAK, A_MAX, J_MAX, L)
    # Infeasibility must agree.
    if not ref.feasible:
        assert c.status == jp.JP_INFEASIBLE, (
            f"C reported feasible where reference said infeasible: "
            f"v0={v0} v1={v1} L={L}")
        return
    assert c.status == jp.JP_OK, (
        f"C infeasible where reference was feasible: v0={v0} v1={v1} L={L}")
    # Segment count matches.
    assert len(c.segments) == len(ref.segments), (
        f"seg count mismatch ({len(c.segments)} vs {len(ref.segments)}): "
        f"v0={v0} v1={v1} L={L}")
    # Durations and coeffs match to 1e-9.
    for i, (cs, rs) in enumerate(zip(c.segments, ref.segments)):
        assert cs.type == rs.type, f"type[{i}] differs v0={v0} v1={v1} L={L}"
        assert cs.T == pytest.approx(rs.T, abs=1e-9, rel=1e-9), \
            f"T[{i}] differs"
        for k, (cc, rc) in enumerate(zip(cs.coeffs, rs.coeffs)):
            assert cc == pytest.approx(rc, abs=1e-9, rel=1e-9), \
                f"coeff[{i}][{k}] differs"
```

- [ ] **Step 6.2: Run the sweep**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py::test_sweep_parity_vs_reference -v`
Expected: PASS on all 36 cases (some will be marked infeasible; those must match reference's infeasibility).

- [ ] **Step 6.3: If any case fails, investigate**

Likely culprits:
- Segment-type enum mapping (the reference uses string tags; wrapper must map C enums back to strings identically)
- Sign convention in `build_accel_side` when `v_end < v_start`
- Floating-point ordering in `find_v_hat` vs reference — widen tolerance slightly (1e-8) only if numerics are the root cause, NOT to hide a logic bug

Inspect the failing case by printing both profiles side-by-side:

```python
# In a one-off debug session:
from klippy.chelper import jerk_profile as jp
prof_c = jp.compute_profile(0, 0, 500, 5000, 100000, 10)
prof_r = REF.compute_profile(0, 0, 500, 5000, 100000, 10)
for i, (cs, rs) in enumerate(zip(prof_c.segments, prof_r.segments)):
    print(f"seg {i}: C T={cs.T:.9f} type={cs.type} vs REF T={rs.T:.9f} type={rs.type}")
```

Fix and rerun until all 36 cases pass.

- [ ] **Step 6.4: Commit passing sweep**

```bash
git add test/test_jerk_profile.py
git commit -m "plan9-A1: 36-case parity sweep vs reference"
```

---

## Task 7: Edge-case and robustness tests

Additional scenarios not covered by the sweep but critical for planner integration.

**Files:**
- Modify: `test/test_jerk_profile.py`

- [ ] **Step 7.1: Add tests for bad input and numerical edge cases**

Append to `test/test_jerk_profile.py`:

```python
def test_rejects_zero_v_peak():
    prof = jp.compute_profile(0.0, 0.0, 0.0, 5000.0, 100000.0, 10.0)
    assert prof.status == jp.JP_BAD_INPUT


def test_rejects_negative_distance():
    prof = jp.compute_profile(0.0, 0.0, 500.0, 5000.0, 100000.0, -10.0)
    assert prof.status == jp.JP_BAD_INPUT


def test_rejects_v_above_peak():
    prof = jp.compute_profile(600.0, 0.0, 500.0, 5000.0, 100000.0, 10.0)
    assert prof.status == jp.JP_BAD_INPUT


def test_very_long_cruise_precision():
    """10 meter cruise at 400 mm/s — end position must be exactly 10000 mm."""
    prof = jp.compute_profile(0.0, 0.0, 400.0, 5000.0, 100000.0, 10000.0)
    assert prof.status == jp.JP_OK
    # Integrate all segments to get total distance.
    last = prof.segments[-1]
    # last.coeffs[0] is p at t=0 of last segment; evaluate at T for total.
    total_p = _eval_poly_deriv(last.coeffs, last.T, 0)
    assert total_p == pytest.approx(10000.0, abs=1e-6)


def test_pure_cruise_when_no_dv_required():
    """v0 == v1 == v_peak, long L — should produce exactly one linear cruise segment."""
    prof = jp.compute_profile(500.0, 500.0, 500.0, 5000.0, 100000.0, 100.0)
    assert prof.status == jp.JP_OK
    # All accel/decel segments have zero duration; only cruise has T > 0.
    nonzero = [s for s in prof.segments if s.T > 1e-12]
    assert len(nonzero) == 1 and nonzero[0].type == "C"


def test_infeasible_returns_status():
    """Tiny L with nonzero endpoint speed mismatch — physically impossible."""
    prof = jp.compute_profile(200.0, 0.0, 500.0, 5000.0, 100000.0, 0.1)
    assert prof.status == jp.JP_INFEASIBLE
```

- [ ] **Step 7.2: Run edge-case tests**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py -v`
Expected: ALL PASS (sweep + continuity + edge cases).

- [ ] **Step 7.3: Commit**

```bash
git add test/test_jerk_profile.py
git commit -m "plan9-A1: edge-case + robustness tests"
```

---

## Task 8: Documentation + handoff

**Files:**
- Create: `docs/superpowers/plans/2026-04-24-plan9-phaseA1-completion.md`

- [ ] **Step 8.1: Run the full test suite**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest test/test_jerk_profile.py -v --tb=short`
Expected: all tests pass. Paste the summary into the completion doc.

- [ ] **Step 8.2: Write the completion doc**

`docs/superpowers/plans/2026-04-24-plan9-phaseA1-completion.md`:

```markdown
# Plan 9 Phase A1 — completion report

**Status:** COMPLETE
**Date:** <YYYY-MM-DD of completion>
**Commits:** <list of commit hashes from this phase>

## What shipped

- `klippy/chelper/jerk_profile.c` — C implementation of jerk-limited polynomial profile generator
- `klippy/chelper/jerk_profile.h` — public header
- `klippy/chelper/jerk_profile.py` — cffi Python wrapper
- `test/test_jerk_profile.py` — <N> tests, all passing
- Registered in `klippy/chelper/__init__.py`

## Validation

- 36-case parity sweep vs Python reference (`docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py`): all cases match to 1e-9.
- C² continuity verified at every segment boundary to 1e-9.
- Feasibility detection agrees with reference on all infeasible cases.
- Long-cruise precision: 10 m cruise ends at exactly 10000 mm.

## Known limits (Phase A1 scope)

- Single-move only. Lookahead is NOT integrated — that's Phase A2.
- No kinematic-coupling scaling yet (CoreXY etc.). That's Phase A4.
- Extruder is not in scope; `v0, v1` are toolhead-space scalars. Extruder integration is Phase A5.

## Next — Phase A2

Wire `jerk_profile_compute` into a new `LookAheadQueue` that performs velocity matching + collinear merging + blend detection, with `a=0` enforced at non-blended junctions.
```

- [ ] **Step 8.3: Commit**

```bash
git add docs/superpowers/plans/2026-04-24-plan9-phaseA1-completion.md
git commit -m "plan9-A1: completion report"
```

---

## Self-review checklist (run before handoff)

- [ ] Every task step has concrete code OR concrete commands OR both.
- [ ] No "TBD", "TODO", "add appropriate X" placeholders.
- [ ] All API signatures in later tasks match what Task 1 declares in the header.
- [ ] All function names used in Python wrapper match those in the cffi declarations match those in the C implementation.
- [ ] Every test has a clear expected-outcome statement.
- [ ] All commits happen after a passing test.
- [ ] File paths are absolute-from-repo-root throughout.

---

## References

- Spec: `docs/superpowers/specs/2026-04-24-plan9-greenfield-motion-design.md`
- Derivation: `docs/superpowers/plans/2026-04-24-plan9-phaseA1-derivation.md`
- Python reference: `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py`
- Biagiotti & Melchiorri, *Trajectory Planning for Automatic Machines and Robots*, Springer 2008
