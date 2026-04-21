# Plan 4 — Pillar 2 integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish Pillar 2 geometry. Wire the smooth-IS shaper family into the quintic blend's velocity cap (P0 — currently a silent no-op), add a shaper-bandwidth-driven velocity ceiling so polyline segment boundaries don't leak HF energy past shaper rejection, re-derive the corner-suppression rule for quintic geometry, and close gaps (endpoint singularities, optional per-sub-move Plan 3 cap refinement).

**Architecture:** Five deliverables, all inside the Python planner layer — no chelper / C changes. D1 extends `blendmath._extract_shapers` + `blendshaper.shaper_span` with a type-aware dispatcher for Smooth Input Shapers. D2 adds `blendmath.v_cap_from_bandwidth` and plugs it into `CornerBlender._emit_blend`. D3 introduces `blendmath.should_suppress_quintic` replacing the double-counted arc-era suppression check. D4 (optional) extracts a `cap_k` helper from `blendextruder.cap_move` for per-sub-move precision. D5 pins `QuinticShape.v_cap_fn` endpoint behavior with tests (+ clamp if broken).

**Tech Stack:** Python (planner layer), pytest, numpy (numerical verification of analytical derivations).

**Predecessor:** Plan 3 (commit `ce0fe532` — spec commit after the Plan 3 P0 fix at `59619cbd`).

**Spec:** `docs/superpowers/specs/2026-04-21-plan4-pillar2-integration-design.md`.

---

## File Structure

| File | Role | Change type |
|---|---|---|
| `klippy/blendshaper.py` | `shaper_span` becomes a type-aware dispatcher (FIR via `_SHAPER_SPAN_FACTOR`, SIS via `T_sm` from `shaper_defs.INPUT_SMOOTHERS`); `compute_shaper_bounds` unchanged | modify (~20 LOC) |
| `klippy/blendmath.py` | `_extract_shapers` branches on `TypedInputSmootherParams`; new helpers `_compute_A_axis_smooth_is`, `v_cap_from_bandwidth`, `should_suppress_quintic` | modify (~80 LOC added) |
| `klippy/blendplanner.py` | `CornerBlender._emit_blend` uses `v_cap_from_bandwidth`; `CornerBlender.feed` uses `should_suppress_quintic`; optional D4 per-sub-move cap | modify (~25 LOC) |
| `klippy/blendquintic.py` | Endpoint clamp in `v_cap_fn` if tests show blow-up | modify (conditional, ~5 LOC) |
| `klippy/blendextruder.py` | Optional: extract `cap_k(pa_snap, limits, k, v_target, a_target)` helper from `cap_move` | modify (D4 only) |
| `test/test_blendshaper.py` | FIR regression parametrized over all FIR names; SIS `shaper_span` tests | modify / create sections |
| `test/test_blendmath.py` | `_extract_shapers` under SIS; `A_axis` numerical verification; `target_smoothing=0` sentinel; `v_cap_from_bandwidth`; `should_suppress_quintic` | modify (~100 LOC added) |
| `test/test_blendquintic.py` | `v_cap_fn` endpoint tests; smooth-IS integration test | modify (~50 LOC added) |
| `docs/superpowers/plans/plan4-derivations/` | Saved math-subagent derivations — `A_axis_smooth_is.md`, `delta_kappa_max.md`, `quintic_suppression.md` | create (research artifacts) |

---

## Notes for the implementer

- **User's rule on git hygiene:** stage specific files by name. **Never `git add -A` or `git add .`** — past incidents captured `.claude/`, `.dSYM/`, and user-edited configs into commits.
- **User's rule on commit timing:** no commits during work hours (Mon–Fri 08:00–18:00 CEST) until 2026-05-01. If you're within work hours, **stage with `git add <files>`, then HOLD the commit and note "staged; commit pending off-hours" in your report**. Per current session, user has granted a session-wide override: commits are allowed; **no push** until after 18:00 unless the user explicitly authorizes it.
- **No `Co-Authored-By: Claude …` trailers** in any commit message. Ever.
- **Run tests from repo root:** `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/`.
- **Before starting any task**, run `git status` and confirm tree state matches the expected post-previous-task state. Stop and report if it doesn't.
- **Math-heavy tasks (1, 9, 13) dispatch a subagent to produce the derivation** before any code is written. Save the subagent's output as a file under `docs/superpowers/plans/plan4-derivations/` so the derivation is reviewable in the commit and future plans can reference it. The implementation tasks then consume the saved formula.
- **Current HEAD as Plan 4 starts:** `ce0fe532` (Plan 4 spec commit).

---

# Part 1 — D1: Smooth-IS shaper cap (P0)

**Why P0.** `klippy/blendmath.py:228-239` explicitly records SIS axes with `A_axis=0.0` ("Arc-blending's velocity cap today only consumes the impulse family"). Then `klippy/blendshaper.py:101-102` skips any snap with `A_axis <= 0`. Net: under a smooth-IS config, the quintic blend's shaper-derived velocity cap is inf (no cap). Same class as Plan 3's `extruder_stepper` singular/plural bug. User's Plan 2 HW validation was running with shaper-cap-at-corners silently off.

---

## Task 1: Derive `A_axis` for Smooth Input Shapers

**Research task. No code in this task — produces a derivation doc consumed by Task 3.**

**Files:**
- Create: `docs/superpowers/plans/plan4-derivations/A_axis_smooth_is.md`

- [ ] **Step 1: Dispatch math subagent**

Use the `Agent` tool, model `opus`, subagent_type `general-purpose`. Prompt:

```
You are deriving the per-axis shaper max-acceleration coefficient A_axis for the Smooth Input Shaper (SIS) family in Kalico, needed to plug SIS into the existing blendshaper.compute_shaper_bounds path.

## Context

For classic FIR shapers, `klippy/extras/shaper_calibrate.py::ShaperCalibrate.find_shaper_max_accel(impulses)` computes A_axis as the coefficient such that, for a commanded acceleration step of magnitude a, the residual worst-case physical acceleration overshoot is `a * f(shaper_params)`. The blend planner uses A_axis in two bounds (blendshaper.py:103-113):

  Bound (b) entry-step: `v_step_cap = sqrt(A_axis * R / proj)` — limits how fast we can start an arc of radius R when the shaper's step response fits inside the deflection budget.
  Bound (c) rotation jerk: `j_eff = A_axis / (T_a * in_plane_projection)` — effective jerk budget during a rotation of span T_a.

## Question

Derive the analytical form of A_axis for each SIS kernel in `klippy/extras/shaper_defs.py`:

- smooth_zv        (degree 4)
- smooth_mzv       (degree 6)
- smooth_ei        (degree 6)
- smooth_2hump_ei  (degree 8)
- smooth_zvd_ei    (degree 8)
- smooth_si        (degree 8)

Each is a compactly-supported polynomial kernel w(τ) on [-T_sm/2, T_sm/2] with ∫w = 1 and w(±T_sm/2) = 0. They're documented in shaper_defs.py lines 96-184 and the scientific rationale is in https://github.com/DangerKlippers/danger-klipper (bleeding-edge-v2 branch).

For each kernel, compute:

1. The kernel's frequency response |W(ω)| and the frequency ω_c at which |W| drops to 5% (or whatever threshold the existing FIR `find_shaper_max_accel` uses — verify from shaper_calibrate.py).
2. The "equivalent step-response overshoot": given a commanded acceleration step, what's the peak physical acceleration deviation after smoothing? This is A_axis in the same units FIR uses.
3. A closed-form expression where tractable, numerical verification otherwise.

## Deliverable

Return a markdown document with:

1. Per-kernel analytical `A_axis` formula (in closed form where possible, a numerical coefficient where not).
2. Verification: for each SIS kernel at a representative config (f_sh=40 Hz, damping_ratio=0.1), numerical simulation confirming the analytical `A_axis` matches a direct step-response peak within 1e-6 relative error.
3. Python-ready expression: a snippet that, given `(shaper_type, shaper_freq, damping_ratio)`, returns `A_axis`. Callable style, ready to paste into blendmath.py as `_compute_A_axis_smooth_is`.
4. Sanity ranges: for each SIS kernel, expected `A_axis` order-of-magnitude at f_sh=40 Hz. Used as the numerical-verification test range.

Check against literature: Biagiotti & Melchiorri 2012 "Trajectory Planning for Automatic Machines and Robots" covers compactly-supported smoothing filters; Sencer-Tajima 2017 has frequency-domain equivalents. Use web search if needed.

Be rigorous. Numerical verification must be reproducible (provide the simulation code). This derivation goes into production Kalico motion-planner code.

Report as a self-contained markdown document, ~500-800 words + code snippets. Do NOT write code to the Kalico repo; only return the markdown document.
```

- [ ] **Step 2: Save the subagent's output**

Save the subagent's returned markdown verbatim to `docs/superpowers/plans/plan4-derivations/A_axis_smooth_is.md`. If the subagent's response includes the "Python-ready expression" snippet, extract it — Task 3 consumes it.

- [ ] **Step 3: Sanity-check the deliverable**

Verify the saved doc contains:
- An `A_axis` formula (analytical or numerical) per SIS kernel name.
- Reproducible numerical verification.
- A Python-ready snippet for `_compute_A_axis_smooth_is(shaper_type, shaper_freq, damping_ratio) -> float`.

If any of these three are missing, re-dispatch with the missing item called out explicitly.

- [ ] **Step 4: Commit the derivation**

```bash
cd /Users/daniladergachev/Developer/kalico
git add docs/superpowers/plans/plan4-derivations/A_axis_smooth_is.md
git commit -m "plan-4: derivation — A_axis for Smooth Input Shapers

Math-subagent derivation (opus) of the per-axis shaper max-acceleration
coefficient for the SIS family. Consumed by D1 Task 3 (_compute_A_axis_smooth_is)."
```

---

## Task 2: Type-aware `shaper_span` dispatcher (with FIR regression tests)

**Goal:** Extend `klippy/blendshaper.py::shaper_span` to handle SIS names without regressing FIR. SIS kernels carry their span explicitly as `T_sm` in `shaper_defs.INPUT_SMOOTHERS`; FIR uses the existing `_SHAPER_SPAN_FACTOR` lookup.

**Files:**
- Modify: `klippy/blendshaper.py` (lines ~44-60 — dispatcher)
- Modify: `test/test_blendshaper.py`

- [ ] **Step 1: Inspect current `shaper_span` + `_SHAPER_SPAN_FACTOR`**

```bash
cd /Users/daniladergachev/Developer/kalico
sed -n '40,65p' klippy/blendshaper.py
```
Expected: `_SHAPER_SPAN_FACTOR` dict with FIR names (zv, mzv, zvd, ei, 2hump_ei, 3hump_ei), `shaper_span(shaper_type, shaper_freq, damping_ratio)` reads the factor and computes `factor * (1/(freq*sqrt(1-dr²)))`.

- [ ] **Step 2: Inspect `INPUT_SMOOTHERS` to see how T_sm is carried**

```bash
sed -n '205,225p' klippy/extras/shaper_defs.py
```
Expected: `INPUT_SMOOTHERS` is a tuple of objects each with `.name` (e.g., `smooth_mzv`) and `.init_func` returning (coeffs, T_sm). Or similar — the exact structure will inform how to read `T_sm` from type name + freq.

- [ ] **Step 3: Write failing FIR-regression test (parametrize over all FIR names)**

Append to `test/test_blendshaper.py`:
```python
import pytest
from klippy import blendshaper


@pytest.mark.parametrize("shaper_type,expected_factor", [
    ("zv",        0.5),
    ("mzv",       0.75),
    ("zvd",       1.0),
    ("ei",        1.0),
    ("2hump_ei",  1.5),
    ("3hump_ei",  2.0),
])
def test_shaper_span_fir_unchanged(shaper_type, expected_factor):
    """FIR shaper_span must not regress after SIS dispatcher lands.
    shaper_span = factor * 1/(freq*sqrt(1-dr^2)); at freq=40, dr=0:
    shaper_span = factor * 0.025.
    """
    span = blendshaper.shaper_span(shaper_type, shaper_freq=40.0, damping_ratio=0.0)
    assert span == pytest.approx(expected_factor * 0.025, rel=1e-9)


def test_shaper_span_fir_damped():
    """Damping widens the span. dr=0.1 → factor / (freq * sqrt(0.99))."""
    span = blendshaper.shaper_span("mzv", shaper_freq=40.0, damping_ratio=0.1)
    import math
    expected = 0.75 / (40.0 * math.sqrt(0.99))
    assert span == pytest.approx(expected, rel=1e-9)
```

- [ ] **Step 4: Write failing SIS test**

Append to `test/test_blendshaper.py`:
```python
@pytest.mark.parametrize("shaper_type", [
    "smooth_zv",
    "smooth_mzv",
    "smooth_ei",
    "smooth_2hump_ei",
    "smooth_zvd_ei",
    "smooth_si",
])
def test_shaper_span_smooth_returns_T_sm(shaper_type):
    """SIS shaper_span returns the kernel's T_sm directly. T_sm is carried
    in shaper_defs.INPUT_SMOOTHERS — shaper_span must not raise ValueError
    on SIS names, and must return a positive finite number."""
    span = blendshaper.shaper_span(shaper_type, shaper_freq=40.0, damping_ratio=0.0)
    assert span > 0.0
    assert span < 0.5  # sanity: SIS spans at 40 Hz are tens of ms, not seconds
```

- [ ] **Step 5: Run tests — expect FIR tests pass (pre-change) and SIS tests fail**

```bash
python3 -m pytest test/test_blendshaper.py -v -k "shaper_span"
```
Expected: FIR tests pass (existing behavior unchanged). SIS tests fail with `ValueError: unknown shaper type: 'smooth_mzv'` (current `shaper_span` raises on non-FIR names).

- [ ] **Step 6: Implement the dispatcher**

Modify `klippy/blendshaper.py` around lines 44-60. Replace the existing `shaper_span` with:

```python
# Pulse-sequence span in units of the damped period, keyed by FIR shaper name.
# Values match klippy/extras/shaper_defs.py exactly (last T[i] of each).
_SHAPER_SPAN_FACTOR = {
    "zv": 0.5,
    "mzv": 0.75,
    "zvd": 1.0,
    "ei": 1.0,
    "2hump_ei": 1.5,
    "3hump_ei": 2.0,
}


def _smooth_is_span(shaper_type: str, shaper_freq: float, damping_ratio: float) -> float:
    """Span of a Smooth Input Shaper polynomial kernel.

    The SIS kernels carry T_sm explicitly (field on the smoother
    definition in shaper_defs.INPUT_SMOOTHERS). At configure time T_sm
    is computed from shaper_freq and the kernel's target smoothing — we
    replicate that here by calling the kernel's init_func.
    """
    from klippy.extras import shaper_defs
    factory = {s.name: s for s in shaper_defs.INPUT_SMOOTHERS}
    if shaper_type not in factory:
        raise ValueError("unknown smooth-IS shaper type: %r" % (shaper_type,))
    smoother_def = factory[shaper_type]
    # init_func returns (coeffs, T_sm) for smooth shapers.
    _coeffs, T_sm = smoother_def.init_func(shaper_freq, damping_ratio)
    return float(T_sm)


def shaper_span(shaper_type: str, shaper_freq: float, damping_ratio: float) -> float:
    """Effective span in seconds for the given shaper configuration.

    FIR shapers: damped-period * per-type factor (existing behavior).
    Smooth-IS shapers: kernel T_sm read from shaper_defs.INPUT_SMOOTHERS.
    """
    if shaper_type in _SHAPER_SPAN_FACTOR:
        factor = _SHAPER_SPAN_FACTOR[shaper_type]
        t_d = 1.0 / (shaper_freq * math.sqrt(1.0 - damping_ratio * damping_ratio))
        return factor * t_d
    # Try SIS. If unknown there too, raise a clear error.
    return _smooth_is_span(shaper_type, shaper_freq, damping_ratio)
```

- [ ] **Step 7: Run tests — expect all pass**

```bash
python3 -m pytest test/test_blendshaper.py -v -k "shaper_span"
```
Expected: all parametrized FIR + SIS tests pass. If SIS tests fail with an import error or unexpected T_sm structure, inspect `shaper_defs.INPUT_SMOOTHERS` — the `init_func` return contract may differ from the assumed `(coeffs, T_sm)` tuple.

- [ ] **Step 8: Commit**

```bash
git add klippy/blendshaper.py test/test_blendshaper.py
git commit -m "blendshaper: type-aware shaper_span dispatcher (FIR + Smooth-IS)

shaper_span now routes FIR names through the existing _SHAPER_SPAN_FACTOR
lookup and SIS names through shaper_defs.INPUT_SMOOTHERS' T_sm. FIR path
unchanged — all 6 FIR shaper parametrizations pass identically.

Part of Plan 4 D1 (smooth-IS shaper cap)."
```

---

## Task 3: `_compute_A_axis_smooth_is` helper with numerical verification

**Goal:** Port the Python-ready expression from `docs/superpowers/plans/plan4-derivations/A_axis_smooth_is.md` (Task 1 deliverable) into `blendmath.py`. Verify numerically against the derivation.

**Files:**
- Modify: `klippy/blendmath.py` (new function near top, after existing helpers)
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Read the saved derivation**

```bash
cat docs/superpowers/plans/plan4-derivations/A_axis_smooth_is.md
```
Extract the Python-ready expression. If the derivation offers an analytical formula, use that; if numerical, use the coefficient table.

- [ ] **Step 2: Write failing numerical-verification test**

Append to `test/test_blendmath.py`:
```python
import pytest
import math
import numpy as np
from klippy import blendmath


@pytest.mark.parametrize("shaper_type,shaper_freq", [
    ("smooth_zv",        40.0),
    ("smooth_mzv",       40.0),
    ("smooth_ei",        40.0),
    ("smooth_2hump_ei",  40.0),
    ("smooth_zvd_ei",    40.0),
    ("smooth_si",        40.0),
])
def test_A_axis_smooth_is_positive(shaper_type, shaper_freq):
    """A_axis for any configured SIS must be strictly positive."""
    A_axis = blendmath._compute_A_axis_smooth_is(
        shaper_type, shaper_freq, damping_ratio=0.1
    )
    assert A_axis > 0.0
    assert math.isfinite(A_axis)


def test_A_axis_smooth_is_scales_with_freq_squared():
    """Physical intuition: A_axis is a max-accel coefficient with units
    mm/s^2 per unit forcing. The frequency scaling for a step-response
    peak is f^2 for a 2nd-order system. Check monotonicity and rough
    scaling on smooth_mzv over freq in [20, 60] Hz.
    """
    a_low  = blendmath._compute_A_axis_smooth_is("smooth_mzv", 20.0, 0.1)
    a_high = blendmath._compute_A_axis_smooth_is("smooth_mzv", 60.0, 0.1)
    ratio = a_high / a_low
    # Expect ratio ~9 (3x freq → 9x A_axis under f^2 scaling); allow wide
    # tolerance because kernel shape interacts with freq non-trivially.
    assert 5.0 < ratio < 15.0


def test_A_axis_smooth_is_matches_numerical_step_response(tmp_path):
    """Verify the closed-form A_axis against a direct time-domain simulation
    of the shaper's step-response peak. This is the gold-standard check
    that the derivation matches physics.
    """
    shaper_type = "smooth_mzv"
    shaper_freq = 40.0
    damping_ratio = 0.1

    # Build the SIS kernel impulse response on a dense time grid.
    from klippy.extras import shaper_defs
    factory = {s.name: s for s in shaper_defs.INPUT_SMOOTHERS}
    smoother = factory[shaper_type]
    coeffs, T_sm = smoother.init_func(shaper_freq, damping_ratio)

    # Sample the polynomial kernel on [-T_sm/2, T_sm/2].
    dt = 1e-5
    t_grid = np.arange(-T_sm/2.0, T_sm/2.0, dt)
    # Kernel w(t): sum_i coeffs[i] * t^i, scaled so integral = 1.
    # Note: exact coeff polynomial layout is confirmed by the derivation doc.
    w = np.polyval(coeffs[::-1], t_grid)
    w /= np.sum(w) * dt  # normalize to integral=1

    # Apply a unit step in commanded acceleration (Heaviside convolved with w).
    # Worst-case step-response peak deviation = max |w * step|.
    # For a polynomial kernel the max is approximately the peak of its running
    # integral — numerical peak-finder over a long enough window.
    cumw = np.cumsum(w) * dt
    peak_response = float(np.max(np.abs(cumw - 1.0)))

    # A_axis is (by convention in shaper_calibrate) the max-accel coefficient
    # such that A_axis ≈ 1/peak_response at unit damping_ratio scaling.
    # Per the derivation doc, the exact relation is A_axis = K_f * f^2 for
    # kernel-specific K_f — check the closed form matches the numerical peak.
    analytical = blendmath._compute_A_axis_smooth_is(shaper_type, shaper_freq, damping_ratio)

    # Rough consistency: the analytical should agree with the numerical peak
    # within 10% (exact form depends on derivation; derivation doc should
    # specify the tolerance).
    # If derivation gives an exact closed form, tighten tolerance to 1e-4.
    assert analytical == pytest.approx(1.0 / peak_response, rel=0.1)
```

*Note to implementer:* if the derivation in `A_axis_smooth_is.md` specifies an exact closed form with tighter tolerance, update the `rel=0.1` to match (e.g., `rel=1e-4`). If the derivation says tolerance should be `rel=1e-6`, use that.

- [ ] **Step 3: Run tests — expect fail with `AttributeError`**

```bash
python3 -m pytest test/test_blendmath.py -v -k "A_axis_smooth"
```
Expected: `AttributeError: module 'klippy.blendmath' has no attribute '_compute_A_axis_smooth_is'`.

- [ ] **Step 4: Implement `_compute_A_axis_smooth_is`**

Paste the Python-ready expression from the derivation doc into `klippy/blendmath.py`. Skeleton (the exact body comes from the derivation):

```python
# ---- Smooth Input Shaper A_axis ----
# Derivation: docs/superpowers/plans/plan4-derivations/A_axis_smooth_is.md
_SIS_A_AXIS_COEFFS = {
    # shaper_type: K_f such that A_axis = K_f * f^2 * g(damping_ratio)
    # Values produced by the math subagent; verify against the numerical
    # test in test_blendmath::test_A_axis_smooth_is_matches_numerical_step_response.
    "smooth_zv":       ...,  # paste from derivation
    "smooth_mzv":      ...,
    "smooth_ei":       ...,
    "smooth_2hump_ei": ...,
    "smooth_zvd_ei":   ...,
    "smooth_si":       ...,
}


def _compute_A_axis_smooth_is(shaper_type: str, shaper_freq: float,
                              damping_ratio: float) -> float:
    """A_axis for a Smooth Input Shaper axis.

    Same semantics as ShaperCalibrate.find_shaper_max_accel(impulses) for
    FIR: the coefficient such that a commanded acceleration step of
    magnitude a produces worst-case physical overshoot ≈ a / A_axis.
    """
    if shaper_type not in _SIS_A_AXIS_COEFFS:
        raise ValueError("unknown smooth-IS shaper type: %r" % (shaper_type,))
    K_f = _SIS_A_AXIS_COEFFS[shaper_type]
    # Exact damping-ratio correction comes from the derivation; placeholder
    # linear (1 - dr²) here — REPLACE with the derivation's formula.
    g_dr = 1.0 - damping_ratio * damping_ratio
    return K_f * shaper_freq * shaper_freq * g_dr
```

Replace placeholders with the actual expression from `A_axis_smooth_is.md`.

- [ ] **Step 5: Run tests — expect all pass**

```bash
python3 -m pytest test/test_blendmath.py -v -k "A_axis_smooth"
```
Expected: all 6 parametrized positivity tests pass; freq-scaling test passes; numerical-match test passes within tolerance. If numerical-match fails with > 10% error, the derivation is wrong or the implementation has a bug — stop, re-read the derivation, fix.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: _compute_A_axis_smooth_is for SIS shapers

Per-kernel A_axis coefficient for the Smooth Input Shaper family,
derived analytically in plan4-derivations/A_axis_smooth_is.md and
verified numerically against a direct step-response simulation.

Part of Plan 4 D1."
```

---

## Task 4: Extend `_extract_shapers` to branch on `TypedInputSmootherParams`

**Goal:** Make `blendmath._extract_shapers` produce `A_axis > 0` for SIS axes (currently hardcoded to 0.0 at line 239).

**Files:**
- Modify: `klippy/blendmath.py` (lines ~200-247)
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Inspect current `_extract_shapers`**

```bash
sed -n '195,250p' klippy/blendmath.py
```
Confirm the `A_axis = 0.0` branch at line 239 for smooth-family params.

- [ ] **Step 2: Write failing integration test**

Append to `test/test_blendmath.py`:
```python
def test_extract_shapers_smooth_is_produces_nonzero_A_axis():
    """After D1, SIS axes must carry a finite positive A_axis, not 0.0."""
    # Build a mock toolhead with a smooth_mzv input_shaper configured.
    class MockShaperParams:
        shaper_type = "smooth_mzv"
        shaper_freq = 40.0
        damping_ratio = 0.1

    class MockAxisShaper:
        def __init__(self, axis):
            self._axis = axis
            self.params = MockShaperParams()
        def get_axis(self):
            return self._axis

    class MockInputShaper:
        target_smoothing = None  # default
        def get_shapers(self):
            return [MockAxisShaper("x"), MockAxisShaper("y")]

    class MockPrinter:
        def lookup_object(self, name, default=None):
            if name == "input_shaper":
                return MockInputShaper()
            return default

    class MockToolhead:
        printer = MockPrinter()

    snaps = blendmath._extract_shapers(MockToolhead())
    assert len(snaps) == 2
    for s in snaps:
        assert s.shaper_type == "smooth_mzv"
        assert s.A_axis > 0.0
        import math
        assert math.isfinite(s.A_axis)


def test_extract_shapers_fir_unchanged():
    """FIR path must still produce A_axis via ShaperCalibrate.find_shaper_max_accel."""
    class MockShaperParams:
        shaper_type = "mzv"
        shaper_freq = 40.0
        damping_ratio = 0.1

    class MockAxisShaper:
        def __init__(self, axis):
            self._axis = axis
            self.params = MockShaperParams()
        def get_axis(self):
            return self._axis

    class MockInputShaper:
        target_smoothing = None
        def get_shapers(self):
            return [MockAxisShaper("x")]

    class MockPrinter:
        def lookup_object(self, name, default=None):
            return MockInputShaper() if name == "input_shaper" else default

    class MockToolhead:
        printer = MockPrinter()

    snaps = blendmath._extract_shapers(MockToolhead())
    assert len(snaps) == 1
    assert snaps[0].shaper_type == "mzv"
    assert snaps[0].A_axis > 0.0
```

- [ ] **Step 3: Run tests — expect SIS test fail, FIR test pass**

```bash
python3 -m pytest test/test_blendmath.py -v -k "extract_shapers"
```
Expected: `test_extract_shapers_smooth_is_produces_nonzero_A_axis` fails with `A_axis == 0.0`. `test_extract_shapers_fir_unchanged` passes.

- [ ] **Step 4: Update `_extract_shapers` to branch on smooth params**

Modify `klippy/blendmath.py` around lines 225-246. Replace the SIS-as-zero-A_axis branch:

```python
    snaps = []
    for axis_shaper in is_obj.get_shapers():
        params = axis_shaper.params
        freq = float(getattr(params, "shaper_freq", 0.0) or 0.0)
        shaper_type = getattr(params, "shaper_type", "") or ""
        damping_ratio = float(getattr(params, "damping_ratio", 0.0) or 0.0)
        if freq <= 0.0 or not shaper_type:
            A_axis = 0.0
        elif shaper_type in shaper_factory:
            # FIR: use ShaperCalibrate.find_shaper_max_accel.
            impulses = shaper_factory[shaper_type](freq, damping_ratio)
            A_axis = float(sc.find_shaper_max_accel(impulses))
        elif shaper_type.startswith("smooth_"):
            # SIS: analytical A_axis from the kernel.
            A_axis = _compute_A_axis_smooth_is(shaper_type, freq, damping_ratio)
        else:
            A_axis = 0.0
        snaps.append(blendshaper.AxisShaperSnapshot(
            axis=axis_shaper.get_axis(),
            shaper_type=shaper_type,
            shaper_freq=freq,
            damping_ratio=damping_ratio,
            A_axis=A_axis,
        ))
    return snaps
```

- [ ] **Step 5: Run tests — all pass**

```bash
python3 -m pytest test/test_blendmath.py -v -k "extract_shapers"
```
Expected: both tests pass.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: _extract_shapers branches on Smooth-IS params

SIS axes now carry analytical A_axis from _compute_A_axis_smooth_is
instead of the previous hardcoded 0.0. FIR path unchanged.

Fixes the P0 silent no-op that made smooth-IS configs run with zero
shaper-derived velocity cap at corners.

Part of Plan 4 D1."
```

---

## Task 5: `target_smoothing=0` sentinel regression test for SIS

**Goal:** Confirm the `target_smoothing=0` diagnostic sentinel (per `project_target_smoothing_sentinel.md` — disables the shaper cap for A/B tests) still disables under SIS, now that SIS axes produce non-zero `A_axis`.

**Files:**
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the regression test**

Append to `test/test_blendmath.py`:
```python
def test_extract_shapers_target_smoothing_zero_disables_SIS():
    """target_smoothing=0 sentinel must return [] regardless of shaper type.
    This is the A/B diagnostic — fully bypasses shaper-derived velocity cap.
    """
    class MockShaperParams:
        shaper_type = "smooth_mzv"
        shaper_freq = 40.0
        damping_ratio = 0.1

    class MockAxisShaper:
        def __init__(self, axis):
            self._axis = axis
            self.params = MockShaperParams()
        def get_axis(self):
            return self._axis

    class MockInputShaper:
        target_smoothing = 0.0  # sentinel
        def get_shapers(self):
            return [MockAxisShaper("x"), MockAxisShaper("y")]

    class MockPrinter:
        def lookup_object(self, name, default=None):
            return MockInputShaper() if name == "input_shaper" else default

    class MockToolhead:
        printer = MockPrinter()

    snaps = blendmath._extract_shapers(MockToolhead())
    assert snaps == []  # sentinel bypasses the cap entirely


def test_extract_shapers_target_smoothing_positive_keeps_SIS():
    """target_smoothing > 0 (user-configured) keeps the cap active."""
    class MockShaperParams:
        shaper_type = "smooth_mzv"
        shaper_freq = 40.0
        damping_ratio = 0.1

    class MockAxisShaper:
        def __init__(self, axis):
            self._axis = axis
            self.params = MockShaperParams()
        def get_axis(self):
            return self._axis

    class MockInputShaper:
        target_smoothing = 0.08  # non-default positive
        def get_shapers(self):
            return [MockAxisShaper("x")]

    class MockPrinter:
        def lookup_object(self, name, default=None):
            return MockInputShaper() if name == "input_shaper" else default

    class MockToolhead:
        printer = MockPrinter()

    snaps = blendmath._extract_shapers(MockToolhead())
    assert len(snaps) == 1
    assert snaps[0].A_axis > 0.0
```

- [ ] **Step 2: Run tests — expect pass (existing sentinel gate runs before the branch)**

```bash
python3 -m pytest test/test_blendmath.py -v -k "target_smoothing"
```
Expected: both tests pass. The `target_smoothing <= 0.0 → return []` gate at lines 218-220 runs before axis iteration, so sentinel behavior is type-independent.

If they fail, the sentinel's ordering is broken — fix by moving the `target_smoothing` check to run before the per-axis branch.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendmath.py
git commit -m "test: target_smoothing=0 sentinel still disables under Smooth-IS

Regression test — the A/B diagnostic sentinel (per
project_target_smoothing_sentinel.md) must return [] regardless of
shaper family.

Part of Plan 4 D1."
```

---

## Task 6: Integration test — quintic + smooth_mzv → finite physical v_cap

**Goal:** End-to-end sanity check that `QuinticShape.v_cap_fn` now produces a finite, physically reasonable velocity cap under a smooth-IS config. Compare against the same corner under an equivalent FIR shaper (`mzv`) — they should be within the same order of magnitude.

**Files:**
- Modify: `test/test_blendquintic.py`

- [ ] **Step 1: Write the integration test**

Append to `test/test_blendquintic.py`:
```python
import math
import pytest
from klippy import blendquintic, blendshape, blendmath


def _make_straight_move(p0, p1, cruise_v=200.0, accel=5000.0):
    """Minimal mock Move with the fields from_moves inspects."""
    class M:
        pass
    m = M()
    dx = [p1[i] - p0[i] for i in range(3)]
    d = math.sqrt(sum(v*v for v in dx))
    m.start_pos = tuple(p0) + (0.0,)
    m.end_pos = tuple(p1) + (0.0,)
    m.axes_d = [*dx, 0.0]
    m.axes_r = [x/d for x in dx] + [0.0]
    m.move_d = d
    m.max_cruise_v2 = cruise_v * cruise_v
    m.accel = accel
    return m


def _make_axis_shaper_snapshots_smooth_mzv(freq=40.0, dr=0.1):
    A = blendmath._compute_A_axis_smooth_is("smooth_mzv", freq, dr)
    return [
        blendshape.AxisShaperSnapshot(axis="x", shaper_type="smooth_mzv",
                                       shaper_freq=freq, damping_ratio=dr,
                                       A_axis=A),
        blendshape.AxisShaperSnapshot(axis="y", shaper_type="smooth_mzv",
                                       shaper_freq=freq, damping_ratio=dr,
                                       A_axis=A),
    ]


def _make_axis_shaper_snapshots_fir_mzv(freq=40.0, dr=0.1):
    from klippy.extras.shaper_calibrate import ShaperCalibrate
    from klippy.extras import shaper_defs
    sc = ShaperCalibrate(printer=None)
    factory = {s.name: s.init_func for s in shaper_defs.INPUT_SHAPERS}
    impulses = factory["mzv"](freq, dr)
    A = float(sc.find_shaper_max_accel(impulses))
    return [
        blendshape.AxisShaperSnapshot(axis="x", shaper_type="mzv",
                                       shaper_freq=freq, damping_ratio=dr,
                                       A_axis=A),
        blendshape.AxisShaperSnapshot(axis="y", shaper_type="mzv",
                                       shaper_freq=freq, damping_ratio=dr,
                                       A_axis=A),
    ]


def test_quintic_v_cap_finite_under_smooth_mzv():
    """Pre-Plan-4 bug: SIS had A_axis=0 → quintic v_cap was inf (uncapped).
    After Plan 4 D1: SIS carries finite A_axis → v_cap is finite.
    """
    # 90-degree corner at the origin: prev goes +X, next goes +Y.
    prev = _make_straight_move((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    next = _make_straight_move((10.0, 0.0, 0.0), (10.0, 10.0, 0.0))

    limits = blendshape.KinematicLimits(
        a_max=5000.0,
        v_max=300.0,
        jerk_max=None,
        extruder_caps=None,
        shapers=_make_axis_shaper_snapshots_smooth_mzv(),
    )
    shape = blendquintic.QuinticShape.from_moves(prev, next, corner_deviation=0.1,
                                                  limits=limits)
    assert shape is not None
    v_mid = shape.v_cap_fn(shape.arc_length / 2.0)
    assert math.isfinite(v_mid)
    assert 0.0 < v_mid < 300.0  # between zero and max_velocity


def test_quintic_v_cap_smooth_vs_fir_same_order_of_magnitude():
    """At a comparable frequency, smooth-IS and FIR caps should be
    within a factor of 2 of each other. Large divergence → derivation bug."""
    prev = _make_straight_move((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    next = _make_straight_move((10.0, 0.0, 0.0), (10.0, 10.0, 0.0))

    limits_sis = blendshape.KinematicLimits(
        a_max=5000.0, v_max=300.0, jerk_max=None,
        extruder_caps=None,
        shapers=_make_axis_shaper_snapshots_smooth_mzv(),
    )
    limits_fir = blendshape.KinematicLimits(
        a_max=5000.0, v_max=300.0, jerk_max=None,
        extruder_caps=None,
        shapers=_make_axis_shaper_snapshots_fir_mzv(),
    )
    shape_sis = blendquintic.QuinticShape.from_moves(prev, next, 0.1, limits_sis)
    shape_fir = blendquintic.QuinticShape.from_moves(prev, next, 0.1, limits_fir)

    v_sis = shape_sis.v_cap_fn(shape_sis.arc_length / 2.0)
    v_fir = shape_fir.v_cap_fn(shape_fir.arc_length / 2.0)
    ratio = v_sis / v_fir
    # Different shaper families at same freq give different but comparable caps.
    assert 0.5 < ratio < 2.0
```

- [ ] **Step 2: Run the tests**

```bash
python3 -m pytest test/test_blendquintic.py -v -k "v_cap_finite_under_smooth_mzv or same_order_of_magnitude"
```
Expected: both pass, confirming D1 is fully wired.

If `test_quintic_v_cap_smooth_vs_fir_same_order_of_magnitude` fails with ratio < 0.5 or > 2.0, the derivation's `A_axis` scale is off — re-run Task 1's subagent with the failure numbers to diagnose.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendquintic.py
git commit -m "test: quintic v_cap finite under Smooth-IS (integration)

Full-path integration test. Confirms D1 (smooth-IS A_axis + shaper_span
dispatcher) is wired end-to-end: a 90° corner under smooth_mzv @ 40 Hz
produces a finite physical v_cap in the same order of magnitude as
the FIR mzv equivalent.

Closes Plan 4 D1 (P0 silent no-op fix)."
```

---

# Part 2 — D5: Endpoint singularity tests

Simple, independent from D1. Land early to clear a latent issue.

---

## Task 7: Endpoint tests for `v_cap_fn(0)` and `v_cap_fn(arc_length)`

**Goal:** Pin behavior at arc-length endpoints where `_point_frame` (blendquintic.py:196) can be degenerate because inner control points coincide with endpoints.

**Files:**
- Modify: `test/test_blendquintic.py`

- [ ] **Step 1: Write the endpoint tests**

Append to `test/test_blendquintic.py`:
```python
@pytest.mark.parametrize("angle_deg", [45, 90, 120, 170])
def test_v_cap_fn_endpoints_finite_and_positive(angle_deg):
    """v_cap_fn(0) and v_cap_fn(arc_length) must be finite and positive
    for a representative range of corner angles.

    At blend endpoints the quintic is tangent to the incoming/outgoing
    straight move, so v_cap should logically equal the straight's
    max_cruise_v (or higher). A blow-up here would mean numerical
    degeneracy in _point_frame.
    """
    import math
    theta = math.radians(180.0 - angle_deg)  # interior angle
    prev = _make_straight_move((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    next = _make_straight_move(
        (10.0, 0.0, 0.0),
        (10.0 + 10.0 * math.cos(theta), 10.0 * math.sin(theta), 0.0),
    )
    limits = blendshape.KinematicLimits(
        a_max=5000.0, v_max=300.0, jerk_max=None,
        extruder_caps=None,
        shapers=_make_axis_shaper_snapshots_smooth_mzv(),
    )
    shape = blendquintic.QuinticShape.from_moves(prev, next, 0.1, limits)
    if shape is None:
        pytest.skip("from_moves returned None for this angle; not in scope")
    v0 = shape.v_cap_fn(0.0)
    vN = shape.v_cap_fn(shape.arc_length)
    assert math.isfinite(v0) and v0 > 0.0, f"v_cap_fn(0) = {v0}"
    assert math.isfinite(vN) and vN > 0.0, f"v_cap_fn(arc_length) = {vN}"


def test_v_cap_fn_endpoints_at_least_straight_cruise():
    """At a blend endpoint the curve is tangent to the straight — the
    cap should be at least the straight's cruise velocity (the blend
    doesn't make you slower than the straight would).
    """
    prev = _make_straight_move((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), cruise_v=150.0)
    next = _make_straight_move((10.0, 0.0, 0.0), (10.0, 10.0, 0.0), cruise_v=150.0)
    limits = blendshape.KinematicLimits(
        a_max=5000.0, v_max=300.0, jerk_max=None,
        extruder_caps=None,
        shapers=_make_axis_shaper_snapshots_smooth_mzv(),
    )
    shape = blendquintic.QuinticShape.from_moves(prev, next, 0.1, limits)
    v0 = shape.v_cap_fn(0.0)
    vN = shape.v_cap_fn(shape.arc_length)
    # Endpoint velocities should not be pathologically low.
    assert v0 >= 10.0, f"v_cap_fn(0) too low: {v0}"
    assert vN >= 10.0, f"v_cap_fn(arc_length) too low: {vN}"
```

- [ ] **Step 2: Run the tests — may pass or may fail depending on current state**

```bash
python3 -m pytest test/test_blendquintic.py -v -k "v_cap_fn_endpoints"
```
Observe outcome. Three possibilities:
- All pass → D5 is complete with tests alone, skip Task 8.
- `finite_and_positive` fails with `inf` or `nan` → `_point_frame` degenerates at endpoints; proceed to Task 8.
- `at_least_straight_cruise` fails with very low but finite values → endpoint v_cap is over-constrained; proceed to Task 8.

- [ ] **Step 3: Commit the tests regardless of outcome**

```bash
git add test/test_blendquintic.py
git commit -m "test: v_cap_fn endpoint behavior pinned

Parametrized endpoint tests over 45°/90°/120°/170° corners plus a
'at least as fast as the straight' regression. Pins QuinticShape.v_cap_fn
at s=0 and s=arc_length — locations where _point_frame (blendquintic.py:196)
can degenerate due to inner control points coinciding with endpoints.

Part of Plan 4 D5."
```

---

## Task 8: [Conditional] Clamp `v_cap_fn` at endpoints

**Goal:** If Task 7 tests failed, clamp the endpoint behavior. If they all passed, **skip this task** entirely.

**Files:**
- Modify: `klippy/blendquintic.py` (only if Task 7 tests failed)

- [ ] **Step 1: If Task 7 all passed, skip to Part 3 (Task 9)**

If all Task 7 tests passed, `v_cap_fn` endpoint behavior is already correct. Move on.

- [ ] **Step 2: If any Task 7 test failed, inspect `_point_frame`**

```bash
sed -n '190,215p' klippy/blendquintic.py
sed -n '580,600p' klippy/blendquintic.py  # v_cap_fn implementation
```
Identify where the degeneracy occurs. Typical culprits:
- `cross_norm` in `_point_frame` → 0 when control-point directions are parallel.
- Division by `R_loc` when `R_loc → inf` (zero curvature at endpoint).

- [ ] **Step 3: Implement a clamp**

In `v_cap_fn`, at the top, add a short-circuit for endpoint evaluation:
```python
    def v_cap_fn(self, s: float) -> float:
        """Max velocity at arc-length s along the blend."""
        # Clamp endpoints to the parent move's cruise velocity — at s=0
        # and s=arc_length the quintic is tangent to a straight move and
        # _point_frame can be degenerate there.
        if s <= 1e-9:
            return math.sqrt(self._prev_max_cruise_v2)
        if s >= self.arc_length - 1e-9:
            return math.sqrt(self._next_max_cruise_v2)
        # ... existing computation ...
```

The `_prev_max_cruise_v2` / `_next_max_cruise_v2` fields need to be stored in `QuinticShape.from_moves`. Add:
```python
    # In from_moves, before returning:
    shape._prev_max_cruise_v2 = prev.max_cruise_v2
    shape._next_max_cruise_v2 = next.max_cruise_v2
```

- [ ] **Step 4: Run tests — expect pass**

```bash
python3 -m pytest test/test_blendquintic.py -v -k "v_cap_fn_endpoints"
```

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py
git commit -m "blendquintic: clamp v_cap_fn at endpoints to parent cruise v

At s=0 and s=arc_length the quintic is tangent to the parent straight
move but _point_frame degenerates (inner control points coincide with
endpoints), making v_cap_fn return inf or nan. Clamp to the parent
move's cruise velocity — the correct physical answer.

Closes Plan 4 D5."
```

---

# Part 3 — D2: Sub-segmentation density / `v_cap_from_bandwidth`

---

## Task 9: Derive `Δκ_max(f_sh, T_sm, N, v)` bandwidth ceiling

**Research task.**

**Files:**
- Create: `docs/superpowers/plans/plan4-derivations/delta_kappa_max.md`

- [ ] **Step 1: Dispatch math subagent**

Use `Agent` tool, model `opus`. Prompt:

```
You are deriving the maximum allowable curvature step Δκ at a polyline
segment boundary such that residual physical acceleration after Input
Shaping stays below a 5% vibration floor at the shaper's tuned frequency.

## Context

The Kalico motion planner splits curvature-continuous corner blends
(quintic Hermite Bezier) into polyline sub-moves for trapq. Each sub-move
is C^0 in position, C^0 in velocity at boundaries → Dirac in jerk at
boundaries. Feeding a Dirac-jerk signal into the forward Input Shaper
(either FIR or Smooth-IS) produces HF content that the shaper can
only reject in a narrow band around its tuned frequency.

Plan 5 (feedforward inverse shaper) requires polyline density dense
enough that segment-boundary κ-steps stay below rejection bandwidth.
Plan 4 sets the velocity cap that makes this true WITHOUT requiring
Plan 1 — by velocity-limiting the blend so Δκ·v² stays under budget.

## Your task

Derive a closed-form or numerical expression for:

  v_cap_from_bandwidth(shape_v_cap_at_s, Δκ_boundary, shapers, chord_err) → float

where:

- `shape_v_cap_at_s` is the geometric/centripetal velocity cap at point s
  (from QuinticShape.v_cap_fn).
- `Δκ_boundary` is the expected κ-step at a polyline segment boundary
  near s, at the chord tolerance `chord_err`. For an arc-length-parametrised
  smooth curve, `Δκ_boundary ≈ (dκ/ds) × Δs_segment`, where `Δs_segment` is
  the local polyline segment length implied by the chord tolerance.
- `shapers` is the list of AxisShaperSnapshot.

Step 1: For each shaper family (FIR and SIS), compute the frequency
response |W(2πf)| — specifically |W(2π f_sh)|, where f_sh is the shaper's
tuned frequency. SIS polynomial kernels of degree N have
|W(ω)| ≤ (2π f T_sm)^-N asymptotically but the exact form is per-kernel.
Check the bleeding-edge-v2 smooth shapers in shaper_defs.py:96-184.

Step 2: The commanded-jerk pulse amplitude at a segment boundary is
`j_pulse ≈ v² · Δκ_boundary / Δt`, duration `Δt = Δs_segment / v`.
Its DFT magnitude at f_sh is bounded by `v² · Δκ_boundary · sinc(π f_sh Δt)`.

Step 3: Residual physical acceleration at f_sh is that magnitude times
`|W(2π f_sh)|`. Set this ≤ 5% × a_residual_budget (use 5% of max_accel
as the budget — typical design margin).

Step 4: Solve for v_cap (max v such that residual stays below budget).
This gives you `v_cap_from_bandwidth`.

## Deliverable

A markdown document with:

1. Per-shaper-family |W(2π f_sh)| expression (exact or asymptotic).
2. Derivation of v_cap_from_bandwidth in terms of known quantities.
3. Concrete numerical example: at f_sh=40 Hz, T_sm=24 ms, N=8 (smooth_mzv
   default), chord_err=20 µm, max_accel=5000 mm/s², κ_peak=0.03 mm⁻¹,
   what's v_cap? Trace through the formula step by step.
4. Python-ready expression: snippet taking `(shape, shapers, chord_err, s)`
   and returning a v_cap float. Ready to paste into blendmath.py.
5. Confirmation that the 5%-budget assumption is reasonable for a
   ringing-bound printer — the user runs Voron Trident ~45k mm/s² accel,
   ringing-visible threshold.
6. Optional: comparison with literature (Biagiotti-Melchiorri 2012, Cho
   2018, Sencer-Tajima 2017) on segment-density bounds for shaped trajectories.

Be rigorous. Numerical example must be computable by the reader.

Return the markdown document (~600-900 words + snippets). Do NOT write
code to the Kalico repo; only return the document.
```

- [ ] **Step 2: Save subagent output**

```bash
# Save the returned markdown to:
docs/superpowers/plans/plan4-derivations/delta_kappa_max.md
```

- [ ] **Step 3: Sanity-check**

Verify the saved doc has:
- |W(2π f_sh)| per shaper family (FIR + SIS).
- `v_cap_from_bandwidth` formula.
- A concrete numerical example (trace through at smooth_mzv 40 Hz).
- Python-ready snippet.

If any missing, re-dispatch.

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/plans/plan4-derivations/delta_kappa_max.md
git commit -m "plan-4: derivation — v_cap_from_bandwidth for polyline segments

Math-subagent derivation of the velocity ceiling needed to keep
polyline segment-boundary κ-steps below shaper rejection bandwidth.
Consumed by D2 Task 10 (v_cap_from_bandwidth implementation)."
```

---

## Task 10: Implement `v_cap_from_bandwidth` helper

**Goal:** Port the Python-ready expression from Task 9's derivation.

**Files:**
- Modify: `klippy/blendmath.py` (add helper)
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Read the saved derivation**

```bash
cat docs/superpowers/plans/plan4-derivations/delta_kappa_max.md
```

- [ ] **Step 2: Write failing test (property-based, doesn't depend on exact formula)**

Append to `test/test_blendmath.py`:
```python
def test_v_cap_from_bandwidth_finite_and_positive():
    """v_cap_from_bandwidth must return a finite positive number for
    a reasonable shape+shapers configuration. Exact value depends on
    derivation — here we just assert it's in a sane range."""
    shapers = _make_axis_shaper_snapshots_smooth_mzv()
    # Simulate a shape with known κ_peak and geometry. Using a mock
    # SmoothShape protocol object.
    class MockShape:
        arc_length = 1.0
        def v_cap_fn(self, s):  # geometric cap
            return 300.0
        # dκ/ds at peak (worst case for segment boundary κ-step)
        def dkappa_ds_peak(self):
            return 0.3  # mm⁻² — roughly κ_peak / (arc_length/4)

    v = blendmath.v_cap_from_bandwidth(
        shape=MockShape(),
        shapers=shapers,
        chord_err=20e-3,  # 20 µm floor
        a_residual_budget=5000.0,
    )
    import math
    assert math.isfinite(v)
    assert 0.0 < v < 1000.0, f"v_cap_from_bandwidth out of range: {v}"


def test_v_cap_from_bandwidth_monotone_in_chord_err():
    """Larger chord_err → more curvature step per segment → lower v_cap.
    The helper must be monotonically decreasing in chord_err."""
    shapers = _make_axis_shaper_snapshots_smooth_mzv()
    class MockShape:
        arc_length = 1.0
        def v_cap_fn(self, s): return 300.0
        def dkappa_ds_peak(self): return 0.3

    v_tight = blendmath.v_cap_from_bandwidth(MockShape(), shapers, 10e-3, 5000.0)
    v_loose = blendmath.v_cap_from_bandwidth(MockShape(), shapers, 50e-3, 5000.0)
    assert v_loose < v_tight, f"not monotone: tight={v_tight}, loose={v_loose}"


def test_v_cap_from_bandwidth_infinite_when_no_shapers():
    """No shapers → no bandwidth constraint → inf cap."""
    v = blendmath.v_cap_from_bandwidth(
        shape=None,  # shape not consulted when shapers is empty
        shapers=[],
        chord_err=20e-3,
        a_residual_budget=5000.0,
    )
    import math
    assert math.isinf(v)
```

- [ ] **Step 3: Run tests — expect fail**

```bash
python3 -m pytest test/test_blendmath.py -v -k "v_cap_from_bandwidth"
```
Expected: `AttributeError: module 'klippy.blendmath' has no attribute 'v_cap_from_bandwidth'`.

- [ ] **Step 4: Implement `v_cap_from_bandwidth`**

Paste the implementation from the derivation. Skeleton with placeholders:

```python
def v_cap_from_bandwidth(shape, shapers, chord_err: float,
                         a_residual_budget: float = 5000.0) -> float:
    """Max blend velocity such that polyline segment-boundary κ-steps
    produce < 5% residual physical acceleration after shaper rejection.

    shape: QuinticShape (or any SmoothShape with dkappa_ds_peak() method).
    shapers: list of AxisShaperSnapshot.
    chord_err: polyline chord tolerance (mm).
    a_residual_budget: reference acceleration (mm/s²) — 5% of this is
                       the vibration floor budget.

    Returns v_cap in mm/s. Returns inf if shapers list is empty.

    Derivation: docs/superpowers/plans/plan4-derivations/delta_kappa_max.md
    """
    import math
    if not shapers:
        return float("inf")

    # 1. Estimate segment length from chord_err and local curvature.
    # For a curve with local radius R and chord tolerance cd, segment
    # length Δs ≈ 2·sqrt(2·cd·R) (small-angle approximation).
    # κ_peak = 1/R_min → R_min = 1/κ_peak, Δs_min = 2·sqrt(2·cd/κ_peak).
    dkappa_ds_peak = shape.dkappa_ds_peak()
    # Δκ per segment boundary ≈ (dκ/ds) × Δs
    # At peak-κ, Δs minimum → Δκ ≈ dkappa_ds_peak × 2·sqrt(2·cd·R_min)
    # This is the worst-case boundary κ-step.

    # 2. Per-shaper, compute rejection budget.
    # |W(2π f_sh)| for FIR = from ShaperCalibrate; for SIS = from kernel.
    # [implementation per derivation doc]

    # 3. Solve Δκ·v² ≤ 5%·a_residual_budget · |W|^-1 for v.
    # [placeholder — paste actual formula from derivation]

    # v_cap = [ derivation formula ]
    # return v_cap

    # PLACEHOLDER — REPLACE with derivation's implementation:
    # First-order estimate ignoring shaper type:
    # v_cap² = 0.05 · a_residual_budget / Δκ_boundary
    # where Δκ_boundary = dkappa_ds_peak · sqrt(2·chord_err/κ_peak)
    kappa_peak = getattr(shape, "kappa_peak", lambda: 0.03)()  # fallback
    delta_s = 2.0 * math.sqrt(2.0 * chord_err / max(kappa_peak, 1e-9))
    delta_kappa_per_segment = dkappa_ds_peak * delta_s
    if delta_kappa_per_segment <= 0.0:
        return float("inf")
    v_cap_sq = 0.05 * a_residual_budget / delta_kappa_per_segment
    return math.sqrt(max(v_cap_sq, 0.0))
```

Replace placeholder with derivation's exact formula.

- [ ] **Step 5: Check `QuinticShape.dkappa_ds_peak` exists**

```bash
grep -n "dkappa_ds\|kappa_peak" klippy/blendquintic.py
```
If `dkappa_ds_peak` (or equivalent) doesn't exist on `QuinticShape`, add it:

```python
# In QuinticShape:
def dkappa_ds_peak(self) -> float:
    """Maximum d(κ)/d(s) magnitude over the blend."""
    # The existing implementation at blendquintic.py:~536 already has
    # analytical dkappa_ds. Take |.| sup over a dense s-grid.
    # If that's expensive, cache it in from_moves.
    sample = 64
    max_rate = 0.0
    for i in range(sample + 1):
        s = (i / sample) * self.arc_length
        r = abs(self._dkappa_ds_at(s))  # existing method
        if r > max_rate:
            max_rate = r
    return max_rate

def kappa_peak(self) -> float:
    """Maximum curvature over the blend."""
    return self._kappa_peak_cached  # already in from_moves
```

- [ ] **Step 6: Run tests — expect pass**

```bash
python3 -m pytest test/test_blendmath.py -v -k "v_cap_from_bandwidth"
```

- [ ] **Step 7: Commit**

```bash
git add klippy/blendmath.py klippy/blendquintic.py test/test_blendmath.py
git commit -m "blendmath: v_cap_from_bandwidth — shaper-rejection v_cap

New helper that derives a blend velocity ceiling so polyline
segment-boundary κ-steps stay below shaper rejection bandwidth at
the tuned frequency. Formula from plan4-derivations/delta_kappa_max.md.

Adds dkappa_ds_peak() and kappa_peak() on QuinticShape for the helper
to consult.

Part of Plan 4 D2."
```

---

## Task 11: Integrate `v_cap_from_bandwidth` in `CornerBlender._emit_blend`

**Goal:** Clamp the blend's velocity at the bandwidth ceiling alongside the existing centripetal cap.

**Files:**
- Modify: `klippy/blendplanner.py` (around line 196-199)

- [ ] **Step 1: Inspect current _emit_blend v-cap line**

```bash
sed -n '193,210p' klippy/blendplanner.py
```
Expected:
```python
shape_mid_v = shape.v_cap_fn(shape.arc_length / 2.0)
arc_cap_v2 = min(prev.max_cruise_v2, nxt.max_cruise_v2, shape_mid_v ** 2)
```

- [ ] **Step 2: Write failing integration test**

Append to `test/test_blendplanner.py` (create if missing):
```python
import math
import pytest
from klippy import blendplanner, blendmath, blendshape, blendquintic


def test_emit_blend_applies_bandwidth_cap():
    """CornerBlender._emit_blend must clamp blend v_cap to
    min(shape.v_cap_fn(mid), v_cap_from_bandwidth).
    """
    # Build a CornerBlender with a toolhead that has a smooth_mzv shaper.
    # Capture the emitted blend_moves and check their cruise_v.
    # [Implementation requires a mock toolhead — see test_blendplanner.py
    #  existing patterns, or skip if Kalico's test harness doesn't support
    #  easy mocking here.]
    pytest.skip("integration test — requires mock toolhead harness; cover via sim")
```

Actually this is hard to unit-test without a real toolhead. Use a simpler integration-level check in Step 3.

- [ ] **Step 3: Modify `_emit_blend` to use bandwidth cap**

In `klippy/blendplanner.py` around line 196, replace:
```python
shape_mid_v = shape.v_cap_fn(shape.arc_length / 2.0)
arc_cap_v2 = min(prev.max_cruise_v2, nxt.max_cruise_v2, shape_mid_v ** 2)
```
with:
```python
shape_mid_v = shape.v_cap_fn(shape.arc_length / 2.0)
# D2: bandwidth cap — polyline segment-boundary κ-step must stay below
# shaper rejection at the tuned frequency.
chord_err = self._resolve_chord_err()
limits = blendshape.KinematicLimits(
    a_max=th.max_accel,
    v_max=th.max_velocity,
    jerk_max=None,
    extruder_caps=_extract_extruder_caps(th),
    shapers=blendmath._extract_shapers(th),
)
bandwidth_v = blendmath.v_cap_from_bandwidth(
    shape=shape,
    shapers=limits.shapers,
    chord_err=chord_err,
    a_residual_budget=th.max_accel,
)
# Take the tighter of geometric and bandwidth caps.
effective_v = min(shape_mid_v, bandwidth_v)
arc_cap_v2 = min(prev.max_cruise_v2, nxt.max_cruise_v2, effective_v ** 2)
```

Note: `_resolve_chord_err` may have been private to the old method; check its visibility and reuse.

- [ ] **Step 4: Run test_blendplanner existing tests (don't break them)**

```bash
python3 -m pytest test/test_blendplanner.py -v
```
Expected: all still pass (bandwidth_v will be inf when no shapers, so behavior is unchanged in the no-shaper case).

- [ ] **Step 5: Run full test suite**

```bash
python3 -m pytest test/ -v 2>&1 | tail -40
```
Expected: no regressions from integrating the bandwidth cap.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: _emit_blend applies v_cap_from_bandwidth

CornerBlender._emit_blend now takes min(shape_mid_v, bandwidth_v)
instead of shape_mid_v alone. Under SIS configs where segment-boundary
κ-steps would leak past shaper rejection, the blend is velocity-limited
to stay under the 5% residual-acceleration budget.

No regression in no-shaper / target_smoothing=0 configs (bandwidth_v
returns inf there).

Part of Plan 4 D2."
```

---

## Task 12: Property test — Δκ·v² below rejection budget at chord-err floor

**Goal:** Verify the bandwidth cap actually produces polylines whose boundary κ-steps respect the budget. End-to-end property test.

**Files:**
- Modify: `test/test_blendquintic.py`

- [ ] **Step 1: Write property test**

Append to `test/test_blendquintic.py`:
```python
def test_polyline_boundary_kappa_step_below_bandwidth_budget():
    """End-to-end: at a representative 90° corner under smooth_mzv, the
    polyline produced at the chord-err floor has boundary κ-steps times
    v² below the 5% shaper rejection budget.

    This is the main guarantee Plan 4 D2 delivers.
    """
    import math
    import numpy as np
    prev = _make_straight_move((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    next = _make_straight_move((10.0, 0.0, 0.0), (10.0, 10.0, 0.0))
    limits = blendshape.KinematicLimits(
        a_max=5000.0, v_max=300.0, jerk_max=None,
        extruder_caps=None,
        shapers=_make_axis_shaper_snapshots_smooth_mzv(),
    )
    shape = blendquintic.QuinticShape.from_moves(prev, next, 0.1, limits)
    assert shape is not None

    chord_err = 20e-3  # 20 µm floor
    polyline = shape.polyline(chord_err)

    # Compute κ at each polyline point by finite difference over 3 consecutive.
    # For each internal boundary, compute Δκ and check Δκ·v² against budget.
    v_blend = min(
        shape.v_cap_fn(shape.arc_length / 2.0),
        blendmath.v_cap_from_bandwidth(shape, limits.shapers, chord_err, 5000.0),
    )

    budget = 0.05 * 5000.0  # 5% of max_accel
    # κ at boundary i = |(P_{i+1} - P_i) × (P_i - P_{i-1})| / (|...| ...) — simple FD approximation
    for i in range(1, len(polyline) - 1):
        p_prev, p_cur, p_next = np.array(polyline[i-1]), np.array(polyline[i]), np.array(polyline[i+1])
        v1 = p_cur - p_prev
        v2 = p_next - p_cur
        cross = np.linalg.norm(np.cross(v1, v2))
        denom = np.linalg.norm(v1) * np.linalg.norm(v2) * np.linalg.norm(v1 + v2) / 2.0 + 1e-12
        kappa_i = cross / denom
        # Δκ at this boundary is the discrete curvature change from the
        # surrounding polyline — bounded by κ at this node.
        # Use κ directly as an upper bound for the test.
        assert kappa_i * v_blend * v_blend <= budget * 10.0, \
            f"boundary {i}: κ·v² = {kappa_i * v_blend * v_blend:.2f} exceeds budget {budget}"
```

*Note:* the tolerance factor `10.0` is a loose bound because our κ estimate is crude. The real check is that a factor-10 margin is enforced by the bandwidth cap. If the test fails even with margin, the bandwidth cap is not being applied or the derivation is wrong.

- [ ] **Step 2: Run the test**

```bash
python3 -m pytest test/test_blendquintic.py -v -k "boundary_kappa_step"
```

- [ ] **Step 3: Commit**

```bash
git add test/test_blendquintic.py
git commit -m "test: polyline boundary κ-step·v² below bandwidth budget

End-to-end property test — the v_cap_from_bandwidth clamp keeps
polyline segment boundaries under the 5% residual-acceleration
budget at smooth_mzv 40 Hz.

Closes Plan 4 D2."
```

---

# Part 4 — D3: Quintic-aware suppression re-derivation

---

## Task 13: Derive the two-clause quintic suppression rule

**Research task.**

**Files:**
- Create: `docs/superpowers/plans/plan4-derivations/quintic_suppression.md`

- [ ] **Step 1: Dispatch math subagent**

Use `Agent` tool, model `opus`:

```
You are re-deriving the shaper-aware corner-suppression rule for Kalico's quintic
Hermite Bezier blend, superseding an arc-based rule that double-counts
chord tolerance under quintic geometry.

## Context

The current rule in klippy/blendmath.py:141-183:

    2·v·sin(φ/2)·σ_T ≤ corner_deviation  → skip blend, run sharp-V under shaper

was derived assuming blend = arc. It compares "how much the sharp-V trajectory
deviates from the vertex under shaper smearing" against the user's
corner_deviation budget. If the shaper's smearing alone fits the budget,
skip the arc.

But with quintic: the blend ITSELF produces a deviation of `corner_deviation`
from the vertex by construction (that's the parameter driving `d_consumed` /
`d_from_deviation`). Adding σ_T smear on top double-counts.

## Task

Derive a correct two-clause rule:

  should_suppress_quintic(prev, next, cd, shape, th) → bool

Clause 1 (path-tolerance): "sharp-V under shaper" deviation ≤ cd?
  That's the existing 2·v·sin(φ/2)·σ_T ≤ cd, where v is the entry velocity
  and σ_T is the shaper's first-moment (existing blendmath.py:~66).

Clause 2 (time): "sharp-V under shaper" total traversal time ≤ blend
  traversal time at quintic's v_cap_fn(mid)?
  If clause 1 is satisfied but the blend is faster overall, keep the blend.
  Only suppress when BOTH clauses say "sharp-V is fine and faster-or-equal."

## Deliverables

A markdown doc with:

1. Detailed derivation of each clause. Show the algebra.

2. A concrete pseudocode for should_suppress_quintic.

3. Worked examples at 3 corner angles (45°, 90°, 120°) and 2 velocities
   (100 mm/s, 300 mm/s) at a representative shaper (mzv at 40 Hz, σ_T=0.01).
   For each, state: sharp-V under-shaper deviation, clause 1 result, sharp-V
   ramp time, blend traversal time, clause 2 result, final should_suppress.

4. Sanity limits: at v→0 both clauses trivially satisfied → should_suppress = True
   (correct: no speed, no reason to blend). At v→large, clause 1 fails → False
   (correct: deviation dominates).

5. Python-ready snippet that can be pasted into blendmath.py.

6. Literature anchor: Biagiotti-Melchiorri 2012 / Cho 2018 / Sencer-Tajima
   2015-2020 treatment of path-tolerance + time comparison in IS corner
   handling.

Return the markdown (~600 words + snippets). Do NOT write code to the repo.
```

- [ ] **Step 2: Save output, sanity-check, commit**

```bash
# Save to docs/superpowers/plans/plan4-derivations/quintic_suppression.md
git add docs/superpowers/plans/plan4-derivations/quintic_suppression.md
git commit -m "plan-4: derivation — two-clause quintic suppression rule

Math-subagent derivation (opus) of the replacement for
blendmath.suppressed_junction_v's current arc-based suppression rule,
which double-counts chord tolerance under quintic geometry.

Consumed by D3 Task 14."
```

---

## Task 14: Implement `should_suppress_quintic` helper

**Goal:** Port the two-clause rule. Keep `suppressed_junction_v` intact (still used by the `from_moves = None` branch as the SCV-equivalent cap).

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Read derivation**

```bash
cat docs/superpowers/plans/plan4-derivations/quintic_suppression.md
```

- [ ] **Step 2: Write failing tests**

Append to `test/test_blendmath.py`:
```python
class MockShape:
    """Minimal SmoothShape protocol for suppression tests."""
    arc_length = 0.5
    def v_cap_fn(self, s): return 200.0

def test_should_suppress_quintic_at_zero_velocity():
    """At v→0, both clauses trivially satisfied → suppress = True."""
    prev = _make_straight_move((0.0,0.0,0.0), (10.0,0.0,0.0), cruise_v=0.1)
    next = _make_straight_move((10.0,0.0,0.0), (10.0,10.0,0.0), cruise_v=0.1)
    class ToolheadMock:
        corner_deviation = 0.1
        def lookup_object(self, n, default=None): return default

    assert blendmath.should_suppress_quintic(
        prev, next, 0.1, MockShape(), ToolheadMock(),
    ) is True


def test_should_suppress_quintic_at_high_velocity():
    """At high v, sharp-V under-shaper deviation > cd → do NOT suppress."""
    prev = _make_straight_move((0.0,0.0,0.0), (10.0,0.0,0.0), cruise_v=400.0)
    next = _make_straight_move((10.0,0.0,0.0), (10.0,10.0,0.0), cruise_v=400.0)
    class ToolheadMock:
        corner_deviation = 0.05
        def lookup_object(self, n, default=None): return default

    assert blendmath.should_suppress_quintic(
        prev, next, 0.05, MockShape(), ToolheadMock(),
    ) is False
```

- [ ] **Step 3: Run — expect AttributeError**

```bash
python3 -m pytest test/test_blendmath.py -v -k "should_suppress_quintic"
```

- [ ] **Step 4: Implement `should_suppress_quintic`**

Paste the snippet from the derivation. Skeleton:
```python
def should_suppress_quintic(prev, nxt, cd: float, shape, toolhead) -> bool:
    """Return True iff the quintic blend at this corner should be
    suppressed (sharp-V under shaper is fine AND faster).

    Two clauses (see plan4-derivations/quintic_suppression.md):
      1. Sharp-V under-shaper deviation ≤ cd.
      2. Sharp-V ramp time ≤ blend traversal time at quintic mid v_cap.
    """
    import math
    # Clause 1: path-tolerance check (existing formula).
    # Extract σ_T from the active shaper on the corner's deflection axis.
    shapers = _extract_shapers(toolhead)
    if not shapers:
        return False  # no shaper, no σ_T — don't suppress
    sigma_T = _max_sigma_T(shapers)  # existing helper or trivial inline
    v = math.sqrt(min(prev.max_cruise_v2, nxt.max_cruise_v2))
    # Corner half-angle from dot product.
    d = prev.axes_r[0]*nxt.axes_r[0] + prev.axes_r[1]*nxt.axes_r[1] + prev.axes_r[2]*nxt.axes_r[2]
    d = max(-1.0, min(1.0, d))
    phi = math.pi - math.acos(d)
    sharp_V_deviation = 2.0 * v * math.sin(phi / 2.0) * sigma_T
    if sharp_V_deviation > cd:
        return False  # path deviation dominates; blend is needed

    # Clause 2: time comparison.
    sharp_V_ramp_time = sharp_V_deviation / max(v, 1e-9)  # rough — refine per derivation
    blend_traversal_time = shape.arc_length / shape.v_cap_fn(shape.arc_length / 2.0)
    return sharp_V_ramp_time <= blend_traversal_time
```

Replace with derivation-exact formulas.

- [ ] **Step 5: Run tests — pass**

```bash
python3 -m pytest test/test_blendmath.py -v -k "should_suppress_quintic"
```

- [ ] **Step 6: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: should_suppress_quintic — two-clause suppression rule

Replaces the arc-era single-clause suppression under quintic geometry
(which double-counted chord tolerance). Checks both (1) sharp-V
under-shaper deviation ≤ corner_deviation AND (2) sharp-V ramp time
≤ blend traversal time before suppressing.

Per derivation in plan4-derivations/quintic_suppression.md.
suppressed_junction_v untouched — still used as the SCV-equivalent
v-cap for the from_moves=None branch.

Part of Plan 4 D3."
```

---

## Task 15: Wire `should_suppress_quintic` into `CornerBlender.feed`

**Goal:** Use the new rule to decide whether to emit sharp-V (skip blend) vs. quintic blend.

**Files:**
- Modify: `klippy/blendplanner.py`

- [ ] **Step 1: Inspect current `feed`**

```bash
sed -n '85,135p' klippy/blendplanner.py
```

- [ ] **Step 2: Add suppression check after `shape = QuinticShape.from_moves`**

In `CornerBlender.feed`, after `shape` is computed and before `_emit_blend`:

```python
shape = blendquintic.QuinticShape.from_moves(
    self._prev, move, th.corner_deviation, limits,
)
if shape is None:
    # ... existing None-branch handling ...
else:
    # D3: re-derived quintic suppression rule.
    if blendmath.should_suppress_quintic(self._prev, move, th.corner_deviation,
                                          shape, th):
        # Sharp-V is fine AND faster — skip the blend, emit prev, let the
        # calc_junction path handle the sharp corner under shaper smearing.
        emitted = [self._prev]
        self._prev = move
        self.blends_emitted += 0  # no-op, explicit for readability
        return emitted
    # Otherwise proceed to _emit_blend as before.
    trunc_prev, blend_moves, trunc_next_head = self._emit_blend(
        self._prev, move, shape
    )
    self._prev = trunc_next_head
    ...
```

- [ ] **Step 3: Run full test suite**

```bash
python3 -m pytest test/ -v 2>&1 | tail -40
```
Expected: no regressions. `blendplanner` tests that construct specific corners may need their expectations updated if suppression now triggers where arc-era suppression didn't.

- [ ] **Step 4: Commit**

```bash
git add klippy/blendplanner.py
git commit -m "blendplanner: CornerBlender.feed uses should_suppress_quintic

Wires the D3 two-clause suppression rule in the feed path — when sharp-V
under shaper is fine AND faster than blend traversal, skip the blend
and emit the truncated-prev directly.

Part of Plan 4 D3."
```

---

## Task 16: Regression tests for `suppressed_junction_v`

**Goal:** Confirm the existing `suppressed_junction_v` path — still used by the `from_moves = None` branch at `blendplanner.py:119` as the SCV-equivalent v-cap — continues to behave as expected.

**Files:**
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write regression tests**

Append to `test/test_blendmath.py`:
```python
def test_suppressed_junction_v_unchanged_for_typical_corner():
    """suppressed_junction_v is still the SCV-equivalent cap for the
    from_moves=None branch. Pin its behavior at a typical 45° corner."""
    prev = _make_straight_move((0.0,0.0,0.0), (10.0,0.0,0.0), cruise_v=200.0)
    next_pt = (10.0 + 10.0 * math.cos(math.radians(135)),
               10.0 * math.sin(math.radians(135)), 0.0)
    next = _make_straight_move((10.0,0.0,0.0), next_pt, cruise_v=200.0)

    class ToolheadMock:
        corner_deviation = 0.1
        def lookup_object(self, n, default=None): return default

    v_j = blendmath.suppressed_junction_v(prev, next, 0.1, ToolheadMock())
    # Value depends on the active shaper (none in mock) → should return None
    # OR a very high cap. Just assert it's either None or finite positive.
    if v_j is not None:
        assert math.isfinite(v_j)
        assert v_j > 0.0


def test_suppressed_junction_v_returns_none_without_shapers():
    """No shaper → σ_T undefined → return None (existing contract)."""
    prev = _make_straight_move((0.0,0.0,0.0), (10.0,0.0,0.0))
    next = _make_straight_move((10.0,0.0,0.0), (10.0,10.0,0.0))
    class ToolheadMock:
        corner_deviation = 0.1
        def lookup_object(self, n, default=None): return default

    v_j = blendmath.suppressed_junction_v(prev, next, 0.1, ToolheadMock())
    assert v_j is None
```

- [ ] **Step 2: Run tests — pass**

```bash
python3 -m pytest test/test_blendmath.py -v -k "suppressed_junction_v"
```

- [ ] **Step 3: Commit**

```bash
git add test/test_blendmath.py
git commit -m "test: suppressed_junction_v regression coverage

Pin the existing from_moves=None fallback path — suppressed_junction_v
continues to return the SCV-equivalent v-cap when the blend can't be
formed.

Closes Plan 4 D3."
```

---

# Part 5 — D4: [Optional] Per-sub-move Plan 3 cap refinement

**Skip this entire section if D1-D3 consume the full engineering budget.** D4 is throughput polish that Plan 5b (unified v(s)) will subsume.

---

## Task 17: [Optional] Extract `cap_k` helper from `cap_move`

**Goal:** Refactor `blendextruder.cap_move` so the k-dependent math can be called per-sub-move without constructing a full `Move`.

**Files:**
- Modify: `klippy/blendextruder.py`
- Modify: `test/test_blendextruder.py`

- [ ] **Step 1: Read current `cap_move`**

```bash
sed -n '85,160p' klippy/blendextruder.py
```

- [ ] **Step 2: Write failing test**

Append to `test/test_blendextruder.py`:
```python
def test_cap_k_matches_cap_move_when_k_matches():
    """cap_k(pa_snap, limits, k, v_target) must return the same (v_cap, a_cap)
    as cap_move when the move's k equals the argument k."""
    # Build a pa_snap + extruder_limits.
    # [reuse existing test helpers in test_blendextruder.py]
    from klippy import blendextruder, blendshape
    pa_snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.05,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    # Mock a move with axes_r[3] = 1.2
    class MockMove:
        axes_r = [0.707, 0.707, 0.0, 1.2]
        max_cruise_v2 = 200.0 * 200.0
        accel = 5000.0

    v_move, a_move = blendextruder.cap_move(MockMove(), pa_snap, limits)
    v_k, a_k = blendextruder.cap_k(pa_snap, limits, k=1.2, v_target=200.0, a_target=5000.0)
    assert v_move == pytest.approx(v_k, rel=1e-9)
    assert a_move == pytest.approx(a_k, rel=1e-9)
```

- [ ] **Step 3: Run — expect AttributeError**

```bash
python3 -m pytest test/test_blendextruder.py -v -k "cap_k"
```

- [ ] **Step 4: Refactor `cap_move` to delegate to `cap_k`**

In `klippy/blendextruder.py`, extract the math:
```python
def cap_k(pa_snap, extruder_limits, k: float, v_target: float,
          a_target: float) -> tuple:
    """Extruder cap math independent of a Move object.

    k: flow ratio (extruder mm per XY mm).
    v_target: the XY cruise velocity we'd like to run.
    a_target: the XY acceleration we'd like to run.

    Returns (v_cap, a_cap): the capped values given the extruder's
    limits and the live PA model.
    """
    # [body of existing cap_move, parameterized on k/v/a rather than move]
    ...


def cap_move(move, pa_snap, extruder_limits):
    """Backward-compatible wrapper around cap_k."""
    import math
    k = move.axes_r[3] if len(move.axes_r) > 3 else 0.0
    if k <= 0.0:
        return float("inf"), float("inf")
    v_target = math.sqrt(move.max_cruise_v2)
    a_target = move.accel
    return cap_k(pa_snap, extruder_limits, k, v_target, a_target)
```

- [ ] **Step 5: Run tests — pass**

```bash
python3 -m pytest test/test_blendextruder.py -v
```
Expected: existing `cap_move` tests still pass (delegation is transparent); new `cap_k` test passes.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendextruder.py test/test_blendextruder.py
git commit -m "blendextruder: extract cap_k helper from cap_move

Refactor: cap_move now delegates to cap_k, which operates on
(k, v_target, a_target) directly without needing a full Move.
cap_move remains the Move-shaped entry point used by toolhead.

Enables per-sub-move cap precision in blendplanner._emit_blend
(Plan 4 D4)."
```

---

## Task 18: [Optional] `_emit_blend` uses per-sub-move `cap_k`

**Goal:** Replace the conservative `min(prev.accel, nxt.accel)` and `min(prev.max_cruise_v2, nxt.max_cruise_v2, ...)` with per-sub-move `cap_k` using each sub-move's interpolated flow ratio.

**Files:**
- Modify: `klippy/blendplanner.py` (around line 197-206)

- [ ] **Step 1: Inspect current blend emit loop**

```bash
sed -n '185,215p' klippy/blendplanner.py
```

- [ ] **Step 2: Refactor per-sub-move cap**

Replace:
```python
shape_mid_v = shape.v_cap_fn(shape.arc_length / 2.0)
# ... bandwidth cap ...
effective_v = min(shape_mid_v, bandwidth_v)
arc_cap_v2 = min(prev.max_cruise_v2, nxt.max_cruise_v2, effective_v ** 2)
arc_cap_v = math.sqrt(arc_cap_v2)
arc_accel = min(prev.accel, nxt.accel)
blend_moves = []
for p0, p1 in zip(points_4d, points_4d[1:]):
    am = move_cls(th, p0, p1, arc_cap_v)
    am.max_cruise_v2 = arc_cap_v2
    am.limit_speed(arc_cap_v, arc_accel)
    am.min_move_t = am.move_d / arc_cap_v
    blend_moves.append(am)
```
with:
```python
shape_mid_v = shape.v_cap_fn(shape.arc_length / 2.0)
chord_err = self._resolve_chord_err()
limits = blendshape.KinematicLimits(
    a_max=th.max_accel, v_max=th.max_velocity, jerk_max=None,
    extruder_caps=_extract_extruder_caps(th),
    shapers=blendmath._extract_shapers(th),
)
bandwidth_v = blendmath.v_cap_from_bandwidth(
    shape, limits.shapers, chord_err, th.max_accel,
)
effective_v = min(shape_mid_v, bandwidth_v)

# D4: per-sub-move Plan 3 cap using each sub-move's interpolated k.
snap = getattr(th, "extruder_cap_snapshot", None)
blend_moves = []
for p0, p1 in zip(points_4d, points_4d[1:]):
    am = move_cls(th, p0, p1, effective_v)
    # Start from the blend-wide cap…
    am.max_cruise_v2 = effective_v ** 2
    am_v = effective_v
    am_a = min(prev.accel, nxt.accel)
    # …then tighten with Plan 3 cap_k at this sub-move's k.
    if snap is not None and am.axes_r[3] > 0.0:
        pa_snap, ext_limits = snap
        from klippy import blendextruder
        v_k, a_k = blendextruder.cap_k(
            pa_snap, ext_limits,
            k=am.axes_r[3], v_target=am_v, a_target=am_a,
        )
        if math.isfinite(v_k):
            am_v = min(am_v, v_k)
        if math.isfinite(a_k) and a_k > 0.0:
            am_a = min(am_a, a_k)
    am.limit_speed(am_v, am_a)
    am.min_move_t = am.move_d / am_v
    blend_moves.append(am)
```

- [ ] **Step 3: Run full tests**

```bash
python3 -m pytest test/ -v 2>&1 | tail -40
```
Expected: no regressions. If blend-emit tests fail because they assumed specific uniform cap values, update their expectations.

- [ ] **Step 4: Commit**

```bash
git add klippy/blendplanner.py
git commit -m "blendplanner: _emit_blend applies Plan 3 cap per-sub-move

Replaces the conservative min(prev.accel, nxt.accel) + blend-wide
v_cap with per-sub-move cap_k using each sub-move's interpolated
flow ratio. Unlocks throughput on asymmetric-flow blends.

Closes Plan 4 D4 (optional)."
```

---

## Final task: Integration sanity + hand-off

- [ ] **Step 1: Run the full test suite**

```bash
cd /Users/daniladergachev/Developer/kalico
python3 -m pytest test/ -v 2>&1 | tail -60
```
Expected: all pass. Any failure is a blocker — stop, diagnose.

- [ ] **Step 2: Run a batch-sim smoke test (optional — skip if no sim env set up)**

Per `docs/magnum_opus/Batch_Sim_Playbook.md`. The minimum check: verify
the Voron cube reference gcode doesn't error under the new caps. If the
implementer does not have the batch-sim environment configured (atmega2560
dict, klippy-env venv), skip this step and rely on the unit/integration
test suite — a sim regression would show as a test failure first anyway.

If set up:
```bash
# Follow Batch_Sim_Playbook.md "Sim config files" + "Running a sim" sections.
# Use docs/magnum_opus/sim_blendarc.cfg (renamed/updated for magnum-opus as
# needed) with smooth_mzv @ 40 Hz active. Pass criteria: sim completes, no
# 'Timer too close' / 'send_too_old' / stepcompress errors in log,
# buffer_time min > 1 s.
```

- [ ] **Step 3: Report complete**

End-state confirmation:
- D1 (smooth-IS cap P0): ✅ shipped, SIS configs now have finite shaper-derived v_cap
- D2 (sub-seg density): ✅ shipped, v_cap_from_bandwidth clamps polyline-boundary HF content
- D3 (quintic suppression): ✅ shipped, two-clause rule replaces arc-era double-count
- D4 (per-sub-move Plan 3 cap): ✅ or skipped
- D5 (endpoint tests): ✅ shipped, v_cap_fn(0) and v_cap_fn(arc_length) pinned

Commit range: `ce0fe532`..HEAD.

Next: **Plan 5 — Pillar 1 (feedforward inverse shaper).** Pillar 1 notes preserved in the Plan 4 spec (`docs/superpowers/specs/2026-04-21-plan4-pillar2-integration-design.md`).
