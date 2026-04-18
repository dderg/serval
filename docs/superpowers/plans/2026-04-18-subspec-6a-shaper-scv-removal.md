# Sub-spec 6a — Shaper-Calibration SCV Removal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Delete the vestigial `scv` (square corner velocity) parameter from the shaper-calibration math chain (`_get_shaper_smoothing`, `find_shaper_max_accel`, `fit_shaper`, `find_best_shaper`) and from every caller (`resonance_tester.py`, `blendmath.py`, `scripts/calibrate_shaper.py`, tests).

**Architecture:** Pure deletion + signature cleanup. Drop the `offset_90` (sharp-90°-corner) term from `_get_shaper_smoothing`; the arc-blending pipeline makes sharp corners impossible, so the term models a non-event. `offset_180` (U-turn cusp) is retained unchanged — still correctly models Kalico's `v_end=0` blender-decline path. Self-consistency: at the blender's shaper-derived arc cap (v² ≤ A_axis·R), centripetal accel ≤ A_axis, so arc-smoothing residual at the bound equals U-turn residual at the same accel. No replacement `offset_arc` term needed.

**Tech Stack:** Python 3, pytest, Kalico fork (Klipper-lineage planner).

**Spec:** `docs/superpowers/specs/2026-04-18-subspec-6a-shaper-scv-removal-design.md`

---

## File Map

| File | Responsibility |
|---|---|
| `klippy/extras/shaper_calibrate.py` | Drop `scv` param from `_get_shaper_smoothing`, `find_shaper_max_accel`, `fit_shaper`, `find_best_shaper`; drop `offset_90` block |
| `klippy/extras/resonance_tester.py` | Drop hardcoded `scv = 5.0` bridge from sub-spec #5; drop `scv=` kwarg from `find_best_shaper` call; drop unused `toolhead_info` line |
| `klippy/blendmath.py` | Drop `scv=0.0` kwarg from `find_shaper_max_accel` call in `_extract_shapers` |
| `scripts/calibrate_shaper.py` | Drop `--scv` / `--square_corner_velocity` CLI option; drop `scv` from `calibrate_shaper` function signature; drop `options.scv` passthrough |
| `test/test_blendshaper.py` | Drop `scv=0.0` kwarg from `_zv_A` helper's `find_shaper_max_accel` call |
| `test/test_shaper_calibrate.py` | **NEW** — numeric regression pin + TDD tests for sig changes + offset_180-only verification |

---

## Task 1: Pre-flight — create `test/test_shaper_calibrate.py` with baseline regression pin

**Files:**
- Create: `test/test_shaper_calibrate.py`

This task writes a regression test pinning the current `find_shaper_max_accel` behavior (old code, `scv=5.0` default). The test will continue to pass through Tasks 2–5 (with the expected value updated once); any unexpected drift signals a math error introduced by a later task.

- [ ] **Step 1: Create the new test file with baseline regression pin**

Create `test/test_shaper_calibrate.py`:

```python
# test/test_shaper_calibrate.py
"""Tests for shaper_calibrate post sub-spec 6a (SCV removal).

The canonical reference case: ZV shaper at 50 Hz with damping_ratio=0.1.
Closed-form for offset_180-only smoothing target (0.12 mm):
    T_d = 1 / (f * sqrt(1 - zeta**2)) = 1 / (50 * sqrt(0.99)) ≈ 0.020101 s
    T_1 = 0.5 * T_d ≈ 0.010050 s              (ZV pulse span)
    ts  = 0.5 * T_1 ≈ 0.005025 s              (shaper-centroid shift)
    sigma2 = (T_1 - ts)**2 = ts**2 ≈ 2.525e-5
    A = 0.24 / sigma2 ≈ 9505 mm/s**2          (accel where offset_180 = 0.12)

Task 1 pins the OLD (pre-6a) value from the current implementation.
Tasks 2 and 3 tighten this pin to the post-change closed form.
"""
import math

import pytest

from klippy.extras import shaper_calibrate, shaper_defs


def _zv_50hz():
    """Canonical reference shaper for all regression pins in this file."""
    return shaper_defs.get_zv_shaper(shaper_freq=50.0, damping_ratio=0.1)


def test_find_shaper_max_accel_baseline_preflight():
    """Baseline regression pin — locks current (pre-6a) behavior.

    Task 1 expects the OLD value (with scv=5.0 default) from the current
    implementation. Tasks 2–3 replace this assertion with the closed-form
    offset_180-only value.
    """
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    shaper = _zv_50hz()
    max_accel = sc.find_shaper_max_accel(shaper, scv=5.0)
    # Old code: max(offset_90(scv=5), offset_180) ≤ 0.12 mm. At the
    # bisection's upper end offset_180 slightly dominates, so the drift
    # from the pure offset_180 answer (~9505) is small but nonzero.
    # Pin to a ±3% band around the expected pre-6a value.
    assert 9000.0 <= max_accel <= 9800.0
```

- [ ] **Step 2: Run the baseline test to verify it passes on current code**

Run: `python3 -m pytest test/test_shaper_calibrate.py::test_find_shaper_max_accel_baseline_preflight -v`
Expected: PASS. (If FAIL, the current `find_shaper_max_accel` is already off-spec — STOP and investigate before changing anything.)

- [ ] **Step 3: Commit**

```bash
git add test/test_shaper_calibrate.py
git commit -m "subspec-6a: pre-flight baseline regression pin for find_shaper_max_accel"
```

---

## Task 2: Drop `scv` + `offset_90` from `_get_shaper_smoothing`

**Files:**
- Modify: `klippy/extras/shaper_calibrate.py` (`_get_shaper_smoothing` def at line 240; internal callers at lines 302 and 367)
- Test: `test/test_shaper_calibrate.py`

This task changes the math: the function drops the `offset_90` branch and the `scv` parameter. Two internal callers (`fit_shaper` line 302, `find_shaper_max_accel` line 367) stop passing `scv` to `_get_shaper_smoothing`. `find_shaper_max_accel` still accepts `scv` as a parameter — Task 3 removes that.

- [ ] **Step 1: Add the failing test for offset_180-only behavior**

Append to `test/test_shaper_calibrate.py`:

```python
def test_get_shaper_smoothing_returns_offset_180_only_closed_form():
    """After 6a, _get_shaper_smoothing returns exactly offset_180:
        (accel / 2) * sigma2_T
    where sigma2_T = sum_i A_i (T_i - ts)**2 / sum_i A_i.

    For ZV @ 50Hz, damping 0.1: sigma2 ≈ 2.525e-5 s**2.
    At accel=10000 mm/s**2: offset_180 ≈ 0.1262 mm.
    """
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    A, T = _zv_50hz()
    D = sum(A)
    ts = sum(A_i * T_i for A_i, T_i in zip(A, T)) / D
    sigma2 = sum(A_i * (T_i - ts) ** 2 for A_i, T_i in zip(A, T)) / D
    accel = 10000.0
    expected = 0.5 * accel * sigma2
    actual = sc._get_shaper_smoothing(_zv_50hz(), accel=accel)
    assert actual == pytest.approx(expected, rel=1e-9)


def test_get_shaper_smoothing_drops_offset_90_at_low_accel():
    """At low accel + nonzero scv the OLD code's offset_90 term dominated,
    returning a larger number than pure offset_180. After 6a the function
    has no way to see scv, so at the same accel the returned value equals
    offset_180(accel). Picks accel=1000 where offset_90 (old scv=5.0)
    would have been ~0.027 mm vs offset_180 = 0.0126 mm.
    """
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    A, T = _zv_50hz()
    D = sum(A)
    ts = sum(A_i * T_i for A_i, T_i in zip(A, T)) / D
    sigma2 = sum(A_i * (T_i - ts) ** 2 for A_i, T_i in zip(A, T)) / D
    accel = 1000.0
    expected_offset_180 = 0.5 * accel * sigma2   # ≈ 0.01262 mm
    actual = sc._get_shaper_smoothing(_zv_50hz(), accel=accel)
    assert actual == pytest.approx(expected_offset_180, rel=1e-9)
    # Sanity: confirm we are in the regime where the OLD offset_90 would
    # have been strictly larger than offset_180.
    old_offset_90_rough = math.sqrt(2.0) * 0.5 * (5.0 + 0.5 * accel * (T[1] - ts)) * (T[1] - ts) / D
    assert old_offset_90_rough > expected_offset_180 * 1.5
```

- [ ] **Step 2: Run the new tests — they MUST fail on current code**

Run: `python3 -m pytest test/test_shaper_calibrate.py -v -k "offset_180_only or drops_offset_90_at_low"`
Expected:
- `test_get_shaper_smoothing_returns_offset_180_only_closed_form`: Could either PASS (if offset_180 dominates at accel=10000 with scv=5) or FAIL (if offset_90 wins). Numerically: at accel=10000 scv=5, offset_90 ≈ 0.1071 vs offset_180 ≈ 0.1262 → offset_180 wins, so test PASSES on old code. This test is a post-change regression pin, not a TDD fail-first. Keep it.
- `test_get_shaper_smoothing_drops_offset_90_at_low_accel`: FAIL on old code — at accel=1000 with default scv=5.0, `max(offset_90, offset_180) ≈ 0.02669` but the assertion expects `0.01262`. This is the TDD fail-first test.

- [ ] **Step 3: Modify `_get_shaper_smoothing` in `klippy/extras/shaper_calibrate.py`**

Replace lines 240–260 with:

```python
    def _get_shaper_smoothing(self, shaper, accel=5000):
        half_accel = accel * 0.5
        A, T = shaper
        inv_D = 1.0 / sum(A)
        n = len(T)
        # Shaper centroid shift — subtracting ts leaves only shaper
        # distortion residual, per Singer & Seering 1990.
        ts = sum([A[i] * T[i] for i in range(n)]) * inv_D

        # offset_180: shaper residual at the cusp of a 180° velocity
        # reversal, x(t) = (a/2)(t - ts)**2. Models U-turn overshoot
        # at Kalico's blender-declined corners (next_junction_v2 = 0).
        offset_180 = 0.0
        for i in range(n):
            offset_180 += A[i] * half_accel * (T[i] - ts) ** 2
        return offset_180 * inv_D
```

- [ ] **Step 4: Fix internal caller at `fit_shaper` line 302**

In `klippy/extras/shaper_calibrate.py`, change:

```python
            shaper_smoothing = self._get_shaper_smoothing(shaper, scv=scv)
```

to:

```python
            shaper_smoothing = self._get_shaper_smoothing(shaper)
```

- [ ] **Step 5: Fix internal caller at `find_shaper_max_accel` line 367**

In `klippy/extras/shaper_calibrate.py`, change:

```python
    def find_shaper_max_accel(self, shaper, scv):
        # Just some empirically chosen value which produces good projections
        # for max_accel without much smoothing
        TARGET_SMOOTHING = 0.12
        max_accel = self._bisect(
            lambda test_accel: (
                self._get_shaper_smoothing(shaper, test_accel, scv)
                <= TARGET_SMOOTHING
            )
        )
        return max_accel
```

to:

```python
    def find_shaper_max_accel(self, shaper, scv=None):  # scv kept for Task 3
        # Just some empirically chosen value which produces good projections
        # for max_accel without much smoothing
        TARGET_SMOOTHING = 0.12
        max_accel = self._bisect(
            lambda test_accel: (
                self._get_shaper_smoothing(shaper, test_accel)
                <= TARGET_SMOOTHING
            )
        )
        return max_accel
```

(The `scv=None` default keeps existing callers — `blendmath.py:291`, `test_blendshaper.py:339`, `fit_shaper` line 314, scripts, the baseline pin in Task 1 — working until Task 3 removes the parameter entirely.)

- [ ] **Step 6: Run the new tests — they MUST pass now**

Run: `python3 -m pytest test/test_shaper_calibrate.py -v -k "offset_180_only or drops_offset_90_at_low"`
Expected: both PASS.

- [ ] **Step 7: Run the baseline regression pin — expect small drift**

Run: `python3 -m pytest test/test_shaper_calibrate.py::test_find_shaper_max_accel_baseline_preflight -v`
Expected: PASS. The baseline window [9000, 9800] should still hold — dropping `offset_90` makes the bisection slightly looser (closer to the pure closed form 9505).

- [ ] **Step 8: Run the existing shaper/blend test suite to ensure no regressions**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendplanner.py test/test_blendprepass.py test/test_blendshaper.py test/test_shaper_calibrate.py -q`
Expected: all pass. Note: the `test/test_blendshaper.py:339` helper `_zv_A` calls `find_shaper_max_accel(shaper, scv=0.0)` — still works because of the `scv=None` default we left in place.

- [ ] **Step 9: Commit**

```bash
git add klippy/extras/shaper_calibrate.py test/test_shaper_calibrate.py
git commit -m "subspec-6a: drop offset_90 and scv from _get_shaper_smoothing"
```

---

## Task 3: Drop `scv` param from `find_shaper_max_accel`

**Files:**
- Modify: `klippy/extras/shaper_calibrate.py` (`find_shaper_max_accel` def + `fit_shaper` line 314 call)
- Modify: `klippy/blendmath.py:291` (one `scv=0.0` kwarg)
- Modify: `test/test_blendshaper.py:339` (one `scv=0.0` kwarg)
- Modify: `test/test_shaper_calibrate.py` (update baseline pin, add signature regression test)

With `_get_shaper_smoothing` done, we can cleanly remove `scv` from `find_shaper_max_accel`. Four callers need to drop the arg: internal (fit_shaper line 314), `blendmath.py:291`, `test/test_blendshaper.py:339`, and the Task-1 baseline pin.

- [ ] **Step 1: Add the signature regression test**

Append to `test/test_shaper_calibrate.py`:

```python
def test_find_shaper_max_accel_signature_rejects_scv_positional():
    """After 6a Task 3, find_shaper_max_accel does not accept the
    legacy positional scv arg. Locks the signature."""
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    with pytest.raises(TypeError):
        sc.find_shaper_max_accel(_zv_50hz(), 5.0)  # old positional scv


def test_find_shaper_max_accel_signature_rejects_scv_kwarg():
    """Same, but via kwarg."""
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    with pytest.raises(TypeError, match="scv"):
        sc.find_shaper_max_accel(_zv_50hz(), scv=0.0)
```

- [ ] **Step 2: Replace the baseline pin with the closed-form pin**

In `test/test_shaper_calibrate.py`, change `test_find_shaper_max_accel_baseline_preflight` to:

```python
def test_find_shaper_max_accel_matches_offset_180_closed_form():
    """After 6a Tasks 2-3: find_shaper_max_accel bisects offset_180 only.
    Closed form: A = 0.24 / sigma2_T where sigma2_T = (T_d / 4)**2
    for a symmetric ZV shaper.
    For ZV @ 50Hz, damping=0.1: A ≈ 9505 mm/s**2. Assert in [9000, 10000]."""
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    shaper = _zv_50hz()
    max_accel = sc.find_shaper_max_accel(shaper)
    assert 9000.0 <= max_accel <= 10000.0
```

(Rename the function — the baseline-preflight one is now gone. This is the post-6a regression pin.)

- [ ] **Step 3: Run the new tests — they MUST fail**

Run: `python3 -m pytest test/test_shaper_calibrate.py -v`
Expected:
- `test_find_shaper_max_accel_signature_rejects_scv_positional`: FAIL (old sig still accepts positional scv; `scv=None` default silently swallows it)
- `test_find_shaper_max_accel_signature_rejects_scv_kwarg`: FAIL (same)
- `test_find_shaper_max_accel_matches_offset_180_closed_form`: PASS (no sig call change needed; already works after Task 2)

- [ ] **Step 4: Remove `scv` param from `find_shaper_max_accel`**

In `klippy/extras/shaper_calibrate.py`, change:

```python
    def find_shaper_max_accel(self, shaper, scv=None):  # scv kept for Task 3
```

to:

```python
    def find_shaper_max_accel(self, shaper):
```

- [ ] **Step 5: Update internal caller at `fit_shaper` line 314**

In `klippy/extras/shaper_calibrate.py`, change:

```python
            max_accel = self.find_shaper_max_accel(shaper, scv)
```

to:

```python
            max_accel = self.find_shaper_max_accel(shaper)
```

- [ ] **Step 6: Update `blendmath.py:291`**

In `klippy/blendmath.py`, change:

```python
            A_axis = float(sc.find_shaper_max_accel(impulses, scv=0.0))
```

to:

```python
            A_axis = float(sc.find_shaper_max_accel(impulses))
```

- [ ] **Step 7: Update `test/test_blendshaper.py:339`**

In `test/test_blendshaper.py`, change the `_zv_A` helper:

```python
def _zv_A(f, zeta=0.1):
    from klippy.extras.shaper_calibrate import ShaperCalibrate
    from klippy.extras import shaper_defs
    sc = ShaperCalibrate(printer=None)
    shaper = shaper_defs.get_zv_shaper(f, zeta)
    return sc.find_shaper_max_accel(shaper, scv=0.0)
```

to:

```python
def _zv_A(f, zeta=0.1):
    from klippy.extras.shaper_calibrate import ShaperCalibrate
    from klippy.extras import shaper_defs
    sc = ShaperCalibrate(printer=None)
    shaper = shaper_defs.get_zv_shaper(f, zeta)
    return sc.find_shaper_max_accel(shaper)
```

- [ ] **Step 8: Run the new signature tests — they MUST pass**

Run: `python3 -m pytest test/test_shaper_calibrate.py -v`
Expected: all pass.

- [ ] **Step 9: Run the full shaper/blend test suite**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendplanner.py test/test_blendprepass.py test/test_blendshaper.py test/test_shaper_calibrate.py -q`
Expected: all pass. `fit_shaper` still accepts `scv` positional (Task 4 removes it) — its internal line-302 and line-314 calls to the two sub-functions no longer forward it, but the outer signature is unchanged, so any caller that passes `scv` still works (it is now ignored internally).

- [ ] **Step 10: Commit**

```bash
git add klippy/extras/shaper_calibrate.py klippy/blendmath.py test/test_blendshaper.py test/test_shaper_calibrate.py
git commit -m "subspec-6a: drop scv param from find_shaper_max_accel"
```

---

## Task 4: Drop `scv` from `fit_shaper` and `find_best_shaper` + external callers

**Files:**
- Modify: `klippy/extras/shaper_calibrate.py` (`fit_shaper` def at line 262; `find_best_shaper` def at line 373; tuple at line 398)
- Modify: `klippy/extras/resonance_tester.py` (lines 569–580)
- Modify: `scripts/calibrate_shaper.py` (lines 70–81, 94–104 — the `find_best_shaper` call site)
- Modify: `test/test_shaper_calibrate.py` (add signature tests for the two top-level APIs)

This removes `scv` from the two public-ish APIs. External callers are `resonance_tester.py` (calls `find_best_shaper` with `scv=scv`) and `scripts/calibrate_shaper.py` (both calls `find_best_shaper` AND has its own `scv` parameter threaded through). The script's `--scv` CLI option is removed in Task 5.

- [ ] **Step 1: Add signature regression tests**

Append to `test/test_shaper_calibrate.py`:

```python
def test_find_best_shaper_signature_rejects_scv_kwarg():
    """After 6a Task 4, find_best_shaper does not accept scv."""
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    with pytest.raises(TypeError, match="scv"):
        sc.find_best_shaper(calibration_data=None, scv=5.0)


def test_fit_shaper_signature_rejects_scv_positional():
    """After 6a Task 4, fit_shaper's scv positional arg is gone."""
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    # Call with enough positionals to reach the old `scv` slot (5th).
    # A plain TypeError is expected (signature mismatch) before any
    # method logic runs, so the other args can be any sentinels.
    with pytest.raises(TypeError):
        sc.fit_shaper(None, None, None, None, 5.0, None, None, None)
```

- [ ] **Step 2: Run the new tests — they MUST fail**

Run: `python3 -m pytest test/test_shaper_calibrate.py -v -k "find_best_shaper_signature or fit_shaper_signature"`
Expected: both FAIL. The old sigs still have `scv`; both accept the arguments.

- [ ] **Step 3: Remove `scv` from `fit_shaper` signature**

In `klippy/extras/shaper_calibrate.py`, change (lines 262–272):

```python
    def fit_shaper(
        self,
        shaper_cfg,
        calibration_data,
        shaper_freqs,
        damping_ratio,
        scv,
        max_smoothing,
        test_damping_ratios,
        max_freq,
    ):
```

to:

```python
    def fit_shaper(
        self,
        shaper_cfg,
        calibration_data,
        shaper_freqs,
        damping_ratio,
        max_smoothing,
        test_damping_ratios,
        max_freq,
    ):
```

- [ ] **Step 4: Remove `scv` from `find_best_shaper` signature**

In `klippy/extras/shaper_calibrate.py`, change (lines 373–384):

```python
    def find_best_shaper(
        self,
        calibration_data,
        shapers=None,
        damping_ratio=None,
        scv=None,
        shaper_freqs=None,
        max_smoothing=None,
        test_damping_ratios=None,
        max_freq=None,
        logger=None,
    ):
```

to:

```python
    def find_best_shaper(
        self,
        calibration_data,
        shapers=None,
        damping_ratio=None,
        shaper_freqs=None,
        max_smoothing=None,
        test_damping_ratios=None,
        max_freq=None,
        logger=None,
    ):
```

- [ ] **Step 5: Remove `scv` from `find_best_shaper`'s internal `fit_shaper` call**

In `klippy/extras/shaper_calibrate.py`, change the tuple passed to `background_process_exec` (lines 391–403):

```python
            shaper = self.background_process_exec(
                self.fit_shaper,
                (
                    shaper_cfg,
                    calibration_data,
                    shaper_freqs,
                    damping_ratio,
                    scv,
                    max_smoothing,
                    test_damping_ratios,
                    max_freq,
                ),
            )
```

to:

```python
            shaper = self.background_process_exec(
                self.fit_shaper,
                (
                    shaper_cfg,
                    calibration_data,
                    shaper_freqs,
                    damping_ratio,
                    max_smoothing,
                    test_damping_ratios,
                    max_freq,
                ),
            )
```

- [ ] **Step 6: Update `resonance_tester.py` caller**

In `klippy/extras/resonance_tester.py`, replace the block around lines 569–580:

```python
            calibration_data[axis].normalize_to_frequencies()
            systime = self.printer.get_reactor().monotonic()
            # Sub-spec #6 will replace with shaper-tuning-aware corner-error
            # budget. Hardcoded 5.0 preserves historical default.
            scv = 5.0
            max_freq = self._get_max_calibration_freq()
            best_shaper, all_shapers = helper.find_best_shaper(
                calibration_data[axis],
                max_smoothing=max_smoothing,
                scv=scv,
                max_freq=max_freq,
                logger=gcmd.respond_info,
            )
```

with:

```python
            calibration_data[axis].normalize_to_frequencies()
            max_freq = self._get_max_calibration_freq()
            best_shaper, all_shapers = helper.find_best_shaper(
                calibration_data[axis],
                max_smoothing=max_smoothing,
                max_freq=max_freq,
                logger=gcmd.respond_info,
            )
```

(The `systime` line becomes unused once the `scv = 5.0` sub-spec #5 bridge is gone — remove it. The `toolhead_info` read from sub-spec #5 was already removed in that spec. Double-check with `grep -n 'systime\|toolhead_info' klippy/extras/resonance_tester.py` — if neither is referenced elsewhere in this function, clean up is complete.)

Verify with:

```bash
grep -n "systime\|toolhead_info" klippy/extras/resonance_tester.py
```

Expected: no hits in the `cmd_SHAPER_CALIBRATE` function body (line 530–610 range) besides what you just removed. If other hits exist outside this block, leave them alone.

- [ ] **Step 7: Update `scripts/calibrate_shaper.py`'s `find_best_shaper` call**

In `scripts/calibrate_shaper.py`, change (lines 94–104):

```python
    shaper, all_shapers = helper.find_best_shaper(
        calibration_data,
        shapers=shapers,
        damping_ratio=damping_ratio,
        scv=scv,
        shaper_freqs=shaper_freqs,
        max_smoothing=max_smoothing,
        test_damping_ratios=test_damping_ratios,
        max_freq=max_freq,
        logger=print,
    )
```

to:

```python
    shaper, all_shapers = helper.find_best_shaper(
        calibration_data,
        shapers=shapers,
        damping_ratio=damping_ratio,
        shaper_freqs=shaper_freqs,
        max_smoothing=max_smoothing,
        test_damping_ratios=test_damping_ratios,
        max_freq=max_freq,
        logger=print,
    )
```

Also drop `scv,` from the `calibrate_shaper` function signature (lines 70–81):

```python
def calibrate_shaper(
    datas,
    csv_output,
    *,
    shapers,
    damping_ratio,
    scv,
    shaper_freqs,
    max_smoothing,
    test_damping_ratios,
    max_freq,
):
```

becomes:

```python
def calibrate_shaper(
    datas,
    csv_output,
    *,
    shapers,
    damping_ratio,
    shaper_freqs,
    max_smoothing,
    test_damping_ratios,
    max_freq,
):
```

Note: the call-site at line ~352 (`scv=options.scv`) will now error — Task 5 removes that in the same commit as the CLI option. Do NOT run the script yet.

- [ ] **Step 8: Run the signature tests — they MUST pass now**

Run: `python3 -m pytest test/test_shaper_calibrate.py -v`
Expected: all pass.

- [ ] **Step 9: Run the full shaper/blend test suite**

Run: `python3 -m pytest test/test_blendmath.py test/test_blendplanner.py test/test_blendprepass.py test/test_blendshaper.py test/test_shaper_calibrate.py -q`
Expected: all pass. `resonance_tester.py` is not exercised by these tests directly (it requires the klipper runtime), but it will be grep-checked in Task 6.

- [ ] **Step 10: Commit**

```bash
git add klippy/extras/shaper_calibrate.py klippy/extras/resonance_tester.py scripts/calibrate_shaper.py test/test_shaper_calibrate.py
git commit -m "subspec-6a: drop scv from fit_shaper, find_best_shaper, and external callers"
```

---

## Task 5: Remove `--scv` CLI option from `scripts/calibrate_shaper.py`

**Files:**
- Modify: `scripts/calibrate_shaper.py` (option definition at lines 245–252; invocation at line 352)

Purely mechanical. Removes the user-facing `--scv` / `--square_corner_velocity` flag and its usage point. Without Task 5 the script throws a TypeError at runtime because Task 4 dropped `scv` from the `calibrate_shaper` function signature but the invocation still passes `scv=options.scv`.

- [ ] **Step 1: Remove the `--scv` option definition**

In `scripts/calibrate_shaper.py` around lines 245–252, delete:

```python
    opts.add_option(
        "--scv",
        "--square_corner_velocity",
        type="float",
        dest="scv",
        default=5.0,
        help="square corner velocity",
    )
```

Delete the entire `opts.add_option("--scv", ...)` block.

- [ ] **Step 2: Remove `scv=options.scv` from the invocation**

In `scripts/calibrate_shaper.py` around lines 346–357, change:

```python
    # Calibrate shaper and generate outputs
    selected_shaper, shapers, calibration_data = calibrate_shaper(
        datas,
        options.csv,
        shapers=shapers,
        damping_ratio=options.damping_ratio,
        scv=options.scv,
        shaper_freqs=shaper_freqs,
        max_smoothing=options.max_smoothing,
        test_damping_ratios=test_damping_ratios,
        max_freq=max_freq,
    )
```

to:

```python
    # Calibrate shaper and generate outputs
    selected_shaper, shapers, calibration_data = calibrate_shaper(
        datas,
        options.csv,
        shapers=shapers,
        damping_ratio=options.damping_ratio,
        shaper_freqs=shaper_freqs,
        max_smoothing=options.max_smoothing,
        test_damping_ratios=test_damping_ratios,
        max_freq=max_freq,
    )
```

- [ ] **Step 3: Verify the script still imports cleanly**

Run:

```bash
python3 -c "import sys; sys.path.insert(0, '.'); import scripts.calibrate_shaper"
```

Expected: no error (syntax check only; full runtime requires numpy/matplotlib/recorded accel data, which is fine to skip).

If the import fails because `scripts` is not a package, fall back to:

```bash
python3 -c "exec(open('scripts/calibrate_shaper.py').read().split('if __name__')[0])"
```

Expected: no error during module-top-level execution.

- [ ] **Step 4: Verify no `scv` references remain in the script**

Run:

```bash
grep -n "scv\|square_corner_velocity" scripts/calibrate_shaper.py
```

Expected: no matches.

- [ ] **Step 5: Commit**

```bash
git add scripts/calibrate_shaper.py
git commit -m "subspec-6a: remove --scv CLI option from calibrate_shaper.py"
```

---

## Task 6: Final sweep + verification

**Files:** none modified — verification only.

- [ ] **Step 1: Run the full blend/shaper test suite**

Run:

```bash
python3 -m pytest test/test_blendmath.py test/test_blendplanner.py test/test_blendprepass.py test/test_blendshaper.py test/test_shaper_calibrate.py -q
```

Expected: all tests pass. Capture the pass count.

- [ ] **Step 2: Run the full repo pytest (ignoring config parsing which has known pre-existing failures)**

Run:

```bash
python3 -m pytest test/ -q --ignore=test/test_configs 2>&1 | tail -30
```

Expected: the same pass count as before sub-spec 6a began, plus the new tests added in this sub-spec. Pre-existing unrelated failures (typically ~84 in `test_configs` and a handful of missing-module skips) must not increase.

- [ ] **Step 3: Grep for any remaining `scv` references in shaper-related code**

Run:

```bash
grep -nE "\bscv\b|square_corner_velocity" klippy/extras/shaper_calibrate.py klippy/extras/resonance_tester.py klippy/blendmath.py scripts/calibrate_shaper.py test/test_shaper_calibrate.py test/test_blendshaper.py
```

Expected: **zero matches**. If any match appears, investigate and clean it up as part of this task (append the cleanup as an additional step, commit under 6a).

Note: `scv` may still appear in `klippy/extras/trad_rack.py` (out-of-scope per sub-spec #5) and in markdown docs (out-of-scope per sub-spec #7). Those are not listed above; leave them.

- [ ] **Step 4: Grep for `square_corner_velocity` anywhere in the planner code**

Run:

```bash
grep -nrE "square_corner_velocity" klippy/ --include="*.py"
```

Expected matches:
- `klippy/extras/trad_rack.py` lines ~2360–2364 (out-of-scope per sub-spec #5; deferred to a future sub-spec)
- No other hits.

If a hit appears in `klippy/toolhead.py`, `klippy/blend*.py`, `klippy/extras/resonance_tester.py`, `klippy/extras/telemetry.py`, or anywhere else in the shaper chain, it is a regression from #5 or this sub-spec — investigate and fix.

- [ ] **Step 5: Verify the spec's "In Scope" coverage**

Open `docs/superpowers/specs/2026-04-18-subspec-6a-shaper-scv-removal-design.md` and confirm each item maps to a completed task:

| Spec item | Task |
|---|---|
| `_get_shaper_smoothing` — drop scv + offset_90 | Task 2 |
| `find_shaper_max_accel` — drop scv | Task 3 |
| `fit_shaper` — drop scv | Task 4 |
| `find_best_shaper` — drop scv | Task 4 |
| `resonance_tester.py` — drop scv=5.0 bridge + kwarg + toolhead_info | Task 4 |
| `blendmath.py:291` — drop scv=0.0 kwarg | Task 3 |
| `scripts/calibrate_shaper.py` — drop scv threading + --scv CLI | Tasks 4 + 5 |
| `test/test_blendshaper.py` — drop scv=0.0 kwarg | Task 3 |
| New tests (closed-form offset_180, sig rejects scv × 3) | Tasks 1, 2, 3, 4 |

All items accounted for. If anything is missing, fix inline and append a commit.

- [ ] **Step 6: Write a one-paragraph summary for the PR / merge commit**

Capture for the final report:
- Commits added: 6 (Tasks 1–5 each produce one commit; Task 6 produces zero unless cleanup is needed).
- Files touched: 6 (`shaper_calibrate.py`, `resonance_tester.py`, `blendmath.py`, `calibrate_shaper.py`, `test_blendshaper.py`, new `test_shaper_calibrate.py`).
- Tests added: 6 (1 closed-form match, 5 signature-rejects-scv tests across 3 APIs).
- Behavioral change: `SHAPER_CALIBRATE` max_accel recommendations may shift by ≤5% looser (dropping 5 mm/s scv contribution). Runtime arc-blend A_axis is bit-identical (blendmath was already calling with `scv=0.0`).
- Deferred to future work: Z-axis blender audit (sub-spec 6b), analytical ghosting-aware A_axis (later sub-spec), docs + example configs (sub-spec 7), Klippain Shake&Tune upstream fix (external).

---

## End State

- `scv` and `square_corner_velocity` appear nowhere in `klippy/extras/shaper_calibrate.py`, `klippy/extras/resonance_tester.py`, `klippy/blendmath.py`, `scripts/calibrate_shaper.py`, or any `test/test_blendshaper.py` / `test/test_shaper_calibrate.py` reference.
- `klippy/extras/trad_rack.py` retains its own `square_corner_velocity` (out-of-scope per sub-spec #5).
- `SHAPER_CALIBRATE` gcode produces recommendations derived from `offset_180`-only bisection. Numeric drift from pre-6a is ≤5% and direction is **looser** (higher recommended max_accel).
- `scripts/calibrate_shaper.py --scv` CLI flag is gone. Users of the offline script see an argparse error on that flag.
- Klippain Shake&Tune breaks (TypeError on `scv=` kwarg). Upstream fix required; no in-fork shim (fork-as-gate).
- Blend-arc runtime `A_axis` via `blendmath._extract_shapers` is numerically identical to pre-6a (sub-spec #3 already used `scv=0.0`, which is equivalent to no-scv under the new math).
