# Plan 2 — Smooth-shapers merge + HP-stepcompress port Implementation Plan

**Execution result (2026-04-21):** Phase A (Tasks 1–10) landed cleanly on `magnum-opus`. **Phase B (Tasks 11–13) aborted** — HP-stepcompress cherry-pick from `upstream/bleeding-edge-v2` conflicts with `f26c79c7` (step-dir pin timing fix, already on magnum-opus via Kalico `v2026.04.00`); no upstream branch has both. HP-stepcompress will be revisited in its own plan with dedicated conflict-resolution attention. Sequel plans (3–6) proceed from `496365b2` (the merge commit) regardless.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring two upstream-derived improvements into `magnum-opus`: the sibling `smooth-shapers` branch (polynomial smooth shapers, non-linear PA, extruder-IS sync, recent calibration work), and HP-stepcompress from `upstream/bleeding-edge-v2` (2nd-order step-timing encoding).

**Architecture:** Two phases on `magnum-opus`. Phase A = smooth-shapers merge, with shape-agnostic helpers pre-ported to magnum-opus first, then the merge itself with file-by-file conflict resolution. Phase B = two direct cherry-picks of HP-stepcompress commits from upstream, preserving authorship. No algorithm changes — this is integration/landing work.

**Tech Stack:** Python (`klippy/*`), C (`klippy/chelper/*`, `src/*`), pytest, git merge/cherry-pick, Kconfig.

**Predecessor:** Plan 1 (quintic revival + shape-pluggable primitive) shipped on magnum-opus.

**Spec:** `docs/superpowers/specs/2026-04-21-plan2-smooth-shapers-merge-plus-hpstepcompress-design.md`.

---

## File Structure

Plan 2 touches a clearly bounded set of files. Phase A is python-only + tests; Phase B is chelper C + MCU C + Python plumbing.

**Phase A files:**

| File | Role in Plan 2 |
|---|---|
| `klippy/blendmath.py` | Host `_scv_equivalent_junction_v`, `suppressed_junction_v` (ported shape-agnostic helpers) + `_extract_shapers` changes for `get_axis()` / smooth-param tolerance |
| `klippy/blendplanner.py` | Wire `suppressed_junction_v` into the `if shape is None:` branch |
| `test/test_blendmath.py` | Tests for the ported helpers + regression tests for the real AxisInputShaper / AxisInputSmoother API |
| `test/test_blendplanner.py` | Test fake mirrors real `AxisInputShaper` API via `get_axis()` |
| `klippy/blendquintic.py` | No edits; read-only reference for `REVERSAL_EPS` threshold alignment |

**Phase B files (modified by cherry-picks — straight ports, no manual edits):**

| File | Role in Plan 2 |
|---|---|
| `klippy/chelper/stepcompress.c` | Host side: existing stepcompress gets HP hook |
| `klippy/chelper/stepcompress.h` | Shared headers |
| `klippy/chelper/stepcompress_hp.c` | New file (621 lines) — the HP encoder |
| `klippy/chelper/__init__.py` | Build system update |
| `klippy/stepper.py` | Python plumbing for HP protocol selection |
| `src/stepper.c` | MCU side: HP decoder |
| `src/Kconfig`, `src/avr/Kconfig` | Opt-in Kconfig option |

---

## Notes for the implementer

- **User's rule on git hygiene**: stage specific files by name. **Do not use `git add -A` or `git add .`** — past incidents captured untracked `.claude/`, `.dSYM/`, and user-edited config files into commits.
- **User's rule on commit timing**: the "no commits during work hours" rule is active until 2026-05-01. On work days (Mon–Fri) between 08:00–18:00 CEST, finish the task, leave everything staged, and hold the commit for off-hours. If the current time is off-hours, commit normally. Each task has a Commit step that works either way (commit now if off-hours; stage and note for later if in work hours).
- **No Co-Authored-By trailers**. No `Co-Authored-By: Claude …` lines in any commit message.
- **Run tests from the repo root**: `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/` (the `klippy/` package imports need this working directory).
- **Before starting any task**, run `git status` and confirm the working tree matches the expected post-previous-task state. Stop and report if it doesn't.

---

## Task 1: Pre-merge survey

**Goal:** Confirm the merge base and conflict set match what the spec predicts.

**Files:**
- Read-only: git state, `klippy/blendquintic.py`, `klippy/blendmath.py`

- [ ] **Step 1: Fetch latest on both branches**

Run:
```bash
cd /Users/daniladergachev/Developer/kalico
git fetch origin
git status
```
Expected: on branch `magnum-opus`, working tree clean (ignoring untracked `.claude/`, `.dSYM/`, `test/configs/hostsimulator.config.old`).

- [ ] **Step 2: Confirm commits coming from smooth-shapers**

Run:
```bash
git log --oneline magnum-opus..smooth-shapers | wc -l
git log --oneline magnum-opus..smooth-shapers | head -40
```
Expected: ~30 commits. Recent tip should be `8d58010a shaper_calibrate: drop sub-sweep bins from vibration scoring` (or newer if the user has added more).

- [ ] **Step 3: Preview merge conflicts**

Run:
```bash
git merge-tree --write-tree magnum-opus smooth-shapers 2>&1 | tail -20
```
Expected: conflicts in exactly these four files:
- `klippy/blendmath.py`
- `klippy/blendplanner.py`
- `test/test_blendmath.py`
- `test/test_blendplanner.py`

**Stop and report** if any other file conflicts — the plan assumes these four only.

- [ ] **Step 4: Record `REVERSAL_EPS` and `COLLINEAR_EPS` from magnum-opus for Task 4 consistency check**

Run:
```bash
grep -n 'REVERSAL_EPS\|COLLINEAR_EPS' klippy/blendquintic.py
grep -n 'REVERSAL_EPS\|COLLINEAR_EPS' klippy/blendmath.py
```
Expected:
- `klippy/blendquintic.py:25:COLLINEAR_EPS = 1e-6`
- `klippy/blendquintic.py:26:REVERSAL_EPS = 1e-6`
- No match in `klippy/blendmath.py` (magnum-opus deleted it; Task 2 adds it back).

Note the values; Task 2 adds `COLLINEAR_EPS = 1e-6` to `blendmath.py`.

No commit for this task — it's read-only verification.

---

## Task 2: Pre-merge port — add `_scv_equivalent_junction_v` + `suppressed_junction_v` to `blendmath.py`

**Goal:** Land the shape-agnostic helpers on magnum-opus as a clean addition, with tests, before the merge. When the merge runs, smooth-shapers' version of these helpers will be "added in both" — trivially resolved.

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing tests**

Add these tests to `test/test_blendmath.py`. Append at the end of the file, after the last existing test. Use the existing test-fake patterns in the file; if `_FakeToolheadWithShapers` / `_FakeInputShaper` / `_FakeAxisInputShaper` are not present in the magnum-opus version, add minimal equivalents inline.

```python
# --- Task 2: suppressed_junction_v + _scv_equivalent_junction_v ---

def test_scv_equivalent_junction_v_collinear_returns_inf():
    """Collinear corner (sin_half=0) → no cap derivable → +inf."""
    v = blendmath._scv_equivalent_junction_v(
        cos_half=1.0, sin_half=0.0,
        corner_deviation=0.1, sigma_T_max=0.015, a_max=50000.0,
    )
    assert math.isinf(v)


def test_scv_equivalent_junction_v_reversal_returns_near_zero():
    """Near-reversal (cos_half≈0) → R_scv≈0 → v_j≈0."""
    v = blendmath._scv_equivalent_junction_v(
        cos_half=1e-5, sin_half=1.0,
        corner_deviation=0.1, sigma_T_max=0.015, a_max=50000.0,
    )
    assert v >= 0.0 and v < 1.0  # sub-1 mm/s


def test_scv_equivalent_junction_v_right_angle_is_finite():
    """90° corner (cos_half = sin_half = 1/sqrt(2)) → finite positive cap."""
    import math as _m
    h = _m.sqrt(2.0) / 2.0
    v = blendmath._scv_equivalent_junction_v(
        cos_half=h, sin_half=h,
        corner_deviation=0.1, sigma_T_max=0.015, a_max=50000.0,
    )
    assert math.isfinite(v) and v > 0.0


def test_scv_equivalent_junction_v_zero_sigma_returns_inf():
    """sigma_T_max=0 → no cap derivable → +inf."""
    v = blendmath._scv_equivalent_junction_v(
        cos_half=0.7, sin_half=0.7,
        corner_deviation=0.1, sigma_T_max=0.0, a_max=50000.0,
    )
    assert math.isinf(v)


def test_suppressed_junction_v_none_without_shaper():
    """Toolhead with no input_shaper → no cap derivable → return None."""
    class _TH:
        printer = None
    prev = _FakeMove(axes_r=(1.0, 0.0, 0.0), move_d=10.0, accel=50000.0)
    nxt  = _FakeMove(axes_r=(0.0, 1.0, 0.0), move_d=10.0, accel=50000.0)
    assert blendmath.suppressed_junction_v(prev, nxt, 0.1, _TH()) is None


def test_suppressed_junction_v_collinear_returns_none():
    """Collinear (sin_half < COLLINEAR_EPS) → None (no cap needed)."""
    prev = _FakeMove(axes_r=(1.0, 0.0, 0.0), move_d=10.0, accel=50000.0)
    nxt  = _FakeMove(axes_r=(1.0, 0.0, 0.0), move_d=10.0, accel=50000.0)
    th = _FakeToolheadWithShapers(_FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 50.0),
        _FakeAxisInputShaper("y", "zv", 50.0),
    ]))
    assert blendmath.suppressed_junction_v(prev, nxt, 0.1, th) is None


def test_suppressed_junction_v_right_angle_returns_finite():
    """90° corner with shaper loaded → finite positive cap."""
    prev = _FakeMove(axes_r=(1.0, 0.0, 0.0), move_d=10.0, accel=50000.0)
    nxt  = _FakeMove(axes_r=(0.0, 1.0, 0.0), move_d=10.0, accel=50000.0)
    th = _FakeToolheadWithShapers(_FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 50.0),
        _FakeAxisInputShaper("y", "zv", 50.0),
    ]))
    v = blendmath.suppressed_junction_v(prev, nxt, 0.1, th)
    assert v is not None and math.isfinite(v) and v > 0.0
```

Add `_FakeMove` near the top of the test file (after imports) if it's not already there:
```python
class _FakeMove:
    def __init__(self, axes_r, move_d, accel):
        self.axes_r = axes_r
        self.move_d = move_d
        self.accel = accel
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
python3 -m pytest test/test_blendmath.py -k 'scv_equivalent_junction_v or suppressed_junction_v' -v
```
Expected: all 7 new tests FAIL with `AttributeError: module 'blendmath' has no attribute '_scv_equivalent_junction_v'` (or similar for `suppressed_junction_v`).

- [ ] **Step 3: Add the helpers to `klippy/blendmath.py`**

Insert at the module level. Place after `_sigma_T_max_from_toolhead` and before `_extract_shapers`. Add `COLLINEAR_EPS` constant near the top (after `Vec3 = Tuple[float, float, float]`).

Top of file, add the constant:
```python
# Sine-of-half-angle below which we treat a junction as collinear.
# Matches blendquintic.COLLINEAR_EPS; kept local to avoid an import.
COLLINEAR_EPS = 1e-6
```

After `_sigma_T_max_from_toolhead`, add:
```python
def _scv_equivalent_junction_v(
    cos_half: float,
    sin_half: float,
    corner_deviation: float,
    sigma_T_max: float,
    a_max: float,
) -> float:
    """Klipper junction-deviation velocity cap equivalent to mainline SCV
    at a shaper with RMS impulse spread sigma_T_max, evaluated at a corner
    with the given half-angle geometry.

    Derivation:
      - SCV-equivalent at 90° matching corner_deviation under shaper smear:
            v_scv90 = cd / (sqrt(2) * sigma_T)
      - Klipper's JD formula (jd = SCV^2 * (sqrt(2) - 1) / a_max):
            jd_eq = v_scv90^2 * (sqrt(2) - 1) / a_max
      - Per-corner radius and velocity:
            R_scv = jd_eq * cos(theta/2) / (1 - cos(theta/2))
            v_j   = sqrt(R_scv * a_max)

    Returns +inf for collinear (no cap needed) or when any input is
    non-positive (no cap derivable).
    """
    one_minus_cos = 1.0 - cos_half
    if sin_half <= COLLINEAR_EPS or one_minus_cos <= 1e-12:
        return float("inf")
    if sigma_T_max <= 0.0 or corner_deviation <= 0.0 or a_max <= 0.0:
        return float("inf")
    v_scv90 = corner_deviation / (math.sqrt(2.0) * sigma_T_max)
    jd_eq = v_scv90 * v_scv90 * (math.sqrt(2.0) - 1.0) / a_max
    R_scv = jd_eq * cos_half / one_minus_cos
    return math.sqrt(R_scv * a_max)


def suppressed_junction_v(
    prev_move,
    next_move,
    corner_deviation: float,
    toolhead,
) -> Optional[float]:
    """SCV-equivalent junction velocity to apply when the corner-blender
    returns no shape at a non-collinear corner.

    Companion to shape builders: when a blend is suppressed (shape is None
    at a real corner, e.g. because adjacent segments are too short for
    the blend to fit, or the corner geometry falls outside the primitive's
    supported range), the fork's `calc_junction` has no JD cap of its own
    — so without this cap the toolhead would enter sharp corners at full
    commanded velocity, causing step skipping.

    Shape-agnostic: depends only on the two move vectors + the toolhead's
    shaper σ_T spread + corner_deviation + a_max. No blend-shape state.

    Returns:
        None  — truly collinear junction (no cap needed), or no shaper
                 loaded (no cap derivable; mainline-Kalico calc_junction
                 quarter-tan cap still applies as a lax safety net).
        float — velocity cap to pass to prev.limit_next_junction_speed().
    """
    if toolhead is None:
        return None
    prev_dir: Vec3 = (
        prev_move.axes_r[0], prev_move.axes_r[1], prev_move.axes_r[2],
    )
    next_dir: Vec3 = (
        next_move.axes_r[0], next_move.axes_r[1], next_move.axes_r[2],
    )
    dp = max(-1.0, min(1.0, vdot(prev_dir, next_dir)))
    cos_half = math.sqrt(max(0.0, (1.0 + dp) * 0.5))
    sin_half = math.sqrt(max(0.0, (1.0 - dp) * 0.5))
    if sin_half < COLLINEAR_EPS:
        return None
    sigma_T = _sigma_T_max_from_toolhead(toolhead)
    if sigma_T <= 0.0:
        return None
    a_max = min(prev_move.accel, next_move.accel)
    v_j = _scv_equivalent_junction_v(
        cos_half, sin_half, corner_deviation, sigma_T, a_max,
    )
    if not math.isfinite(v_j):
        return None
    return v_j
```

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
python3 -m pytest test/test_blendmath.py -k 'scv_equivalent_junction_v or suppressed_junction_v' -v
```
Expected: all 7 new tests PASS.

- [ ] **Step 5: Run full blendmath test suite to verify no regressions**

Run:
```bash
python3 -m pytest test/test_blendmath.py -v
```
Expected: all tests pass (existing magnum-opus tests + 7 new).

- [ ] **Step 6: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: port suppressed_junction_v from smooth-shapers

Pre-merge port of the SCV-equivalent junction-velocity helpers. Both
helpers are pure move-vector + shaper sigma_T math — shape-agnostic —
so they transfer from the deleted arc codepath to the quintic world
verbatim. Wired in by the next commit.

See docs/superpowers/specs/2026-04-21-plan2-smooth-shapers-merge-plus-hpstepcompress-design.md"
```

If work hours: stage only (`git add`), skip `git commit` and note in handoff.

---

## Task 3: Pre-merge wire — call `suppressed_junction_v` in `blendplanner.py`

**Goal:** Use the helper from Task 2 in the `if shape is None:` branch of `CornerBlender.feed`. Closes the skipped-steps bug on magnum-opus in theory (see Plan 2 spec for the investigation).

**Files:**
- Modify: `klippy/blendplanner.py:57-91`
- Modify: `test/test_blendplanner.py`

- [ ] **Step 1: Write the failing tests**

Add these tests to `test/test_blendplanner.py`. Append at the end of the file, using the existing test fixtures (`_FakeToolhead`, `_FakeMove`, etc.). If the existing file exposes a simpler builder for "short-segment infeasible-blend" setup, use that; otherwise construct the scenario inline.

```python
# --- Task 3: suppressed-corner junction cap ---

def test_feed_suppressed_corner_caps_junction_velocity():
    """Real corner, segments too short for blend: from_moves returns None.
    Planner must apply suppressed_junction_v cap via
    limit_next_junction_speed so the toolhead doesn't hit the corner at
    full cruise (skipped-steps scenario).
    """
    # Two short perpendicular moves — too short to blend at cd=0.1
    # (blend needs d > 0.3 mm on each side at this angle).
    th = _make_toolhead_with_zv_shapers(freq_x=50.0, freq_y=50.0,
                                         max_accel=50000.0,
                                         corner_deviation=0.1)
    prev = _make_move(th, start=(0, 0, 0, 0), end=(0.2, 0, 0, 0),
                      cruise_v=300.0)
    nxt  = _make_move(th, start=(0.2, 0, 0, 0), end=(0.2, 0.2, 0, 0),
                      cruise_v=300.0)
    cb = blendplanner.CornerBlender(th, move_cls=_FakeMove)
    _ = cb.feed(prev)
    emitted = cb.feed(nxt)
    # prev was emitted; its next-junction speed should have been capped.
    assert len(emitted) == 1 and emitted[0] is prev
    # limit_next_junction_speed should have been called with a finite cap.
    assert prev.next_junction_v_capped_to is not None
    assert math.isfinite(prev.next_junction_v_capped_to)
    assert prev.next_junction_v_capped_to > 0.0


def test_feed_suppressed_corner_no_shaper_falls_back_to_reversal_stop():
    """If no shaper is loaded, suppressed_junction_v returns None.
    Planner still hard-stops on near-reversals (dp <= -0.5) as safety.
    """
    th = _make_toolhead_without_shapers(max_accel=50000.0,
                                         corner_deviation=0.1)
    prev = _make_move(th, start=(0, 0, 0, 0), end=(0.2, 0, 0, 0),
                      cruise_v=300.0)
    # 150° reversal-ish
    import math as _m
    a = _m.radians(180.0 - 30.0)
    nxt  = _make_move(th, start=(0.2, 0, 0, 0),
                      end=(0.2 + _m.cos(a)*0.2, _m.sin(a)*0.2, 0, 0),
                      cruise_v=300.0)
    cb = blendplanner.CornerBlender(th, move_cls=_FakeMove)
    _ = cb.feed(prev)
    _ = cb.feed(nxt)
    assert prev.next_junction_v_capped_to == 0.0
```

If `_make_toolhead_with_zv_shapers`, `_make_toolhead_without_shapers`, `_make_move`, or a mechanism to record `next_junction_v_capped_to` aren't present in the current test file, add minimal helpers. Study the existing patterns in `test_blendplanner.py` (from Plan 1) before writing — match their naming/structure.

If `_FakeMove` doesn't already expose a `limit_next_junction_speed` that records the last value, add it:
```python
class _FakeMove:
    def __init__(self, ...):
        ...
        self.next_junction_v_capped_to = None
    def limit_next_junction_speed(self, v):
        self.next_junction_v_capped_to = v
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
python3 -m pytest test/test_blendplanner.py -k 'suppressed_corner' -v
```
Expected: both tests FAIL — on current magnum-opus, `suppressed_junction_v` is not called in the `if shape is None:` branch, so `next_junction_v_capped_to` remains `None` for the right-angle case.

- [ ] **Step 3: Rewrite the `if shape is None:` branch in `blendplanner.py`**

Edit `klippy/blendplanner.py`. Replace the existing `if shape is None:` block (roughly lines 76–91) with:

```python
        if shape is None:
            # from_moves returns None for:
            #   (a) collinear corners — no cap needed;
            #   (b) near-reversals — from_moves caught this via REVERSAL_EPS;
            #   (c) moves too short to accommodate the blend — need a
            #       fallback velocity cap, because fork calc_junction has
            #       no JD-based cap of its own (centripetal quarter-tan
            #       alone is empirically insufficient at high accel).
            # suppressed_junction_v derives an SCV-equivalent cap from
            # the active shaper's sigma_T; shape-agnostic.
            v_j = blendmath.suppressed_junction_v(
                self._prev, move, th.corner_deviation, th
            )
            if v_j is not None and math.isfinite(v_j):
                self._prev.limit_next_junction_speed(v_j)
            else:
                # No shaper loaded (or v_j undefined). Fall back to the
                # near-reversal hard-stop heuristic so the toolhead
                # doesn't round pi-radian reversals at cruise velocity.
                dp = sum(
                    self._prev.axes_r[i] * move.axes_r[i] for i in range(3)
                )
                if dp <= -0.5:
                    self._prev.limit_next_junction_speed(0.0)
            emitted = [self._prev]
            self._prev = move
            return emitted
```

The suppressed_junction_v branch replaces the previous `dp <= -0.5` special-case because suppressed_junction_v itself returns near-zero for near-reversals (cos_half ≈ 0 → R_scv ≈ 0 → v_j ≈ 0). The dp<=-0.5 branch is retained only as a no-shaper safety net.

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
python3 -m pytest test/test_blendplanner.py -k 'suppressed_corner' -v
```
Expected: both tests PASS.

- [ ] **Step 5: Run full blendplanner + blendmath suite**

Run:
```bash
python3 -m pytest test/test_blendplanner.py test/test_blendmath.py -v
```
Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: cap junction velocity when QuinticShape.from_moves returns None

When the quintic corner blender suppresses a blend at a real corner
(segments too short), the fork's calc_junction has no JD cap — the
centripetal quarter-tan alone is empirically insufficient at high
accel. Call suppressed_junction_v (shape-agnostic, shaper-sigma_T-
derived) and apply via limit_next_junction_speed. The no-shaper
safety path keeps the pi-radian reversal hard-stop heuristic.

See docs/superpowers/specs/2026-04-21-plan2-smooth-shapers-merge-plus-hpstepcompress-design.md"
```

If work hours: stage only and note.

---

## Task 4: Pre-merge threshold consistency check

**Goal:** Confirm `QuinticShape`'s `REVERSAL_EPS` behavior is consistent with `blendplanner`'s no-shaper fallback `dp <= -0.5`. No new cases should fall through uncapped.

**Files:** Read-only — `klippy/blendquintic.py`, `klippy/blendplanner.py`. New test.

- [ ] **Step 1: Write a narrow-wedge regression test**

Add to `test/test_blendplanner.py`:
```python
def test_feed_near_reversal_without_shaper_forces_stop():
    """At theta ≈ pi (near-reversal), from_moves returns None via the
    REVERSAL_EPS guard. Without a shaper the dp <= -0.5 fallback fires
    and stops the toolhead. Regression: narrow wedge of angles where
    both from_moves and the blendplanner fallback could otherwise
    disagree.
    """
    import math as _m
    th = _make_toolhead_without_shapers(max_accel=50000.0,
                                         corner_deviation=0.1)
    prev = _make_move(th, start=(0, 0, 0, 0), end=(1.0, 0, 0, 0),
                      cruise_v=300.0)
    # theta = pi - 1e-7 radians → from_moves returns None via REVERSAL_EPS
    theta = _m.pi - 1e-7
    nxt = _make_move(th, start=(1.0, 0, 0, 0),
                     end=(1.0 + _m.cos(theta)*1.0, _m.sin(theta)*1.0, 0, 0),
                     cruise_v=300.0)
    cb = blendplanner.CornerBlender(th, move_cls=_FakeMove)
    _ = cb.feed(prev)
    _ = cb.feed(nxt)
    assert prev.next_junction_v_capped_to == 0.0


def test_feed_narrow_reversal_wedge_with_shaper_caps_finite():
    """At theta just under the REVERSAL_EPS threshold with a shaper loaded,
    suppressed_junction_v is called and returns a very small (near-zero
    but finite positive) cap. Confirms no gap between REVERSAL_EPS and
    the `dp <= -0.5` fallback when a shaper is present.
    """
    import math as _m
    th = _make_toolhead_with_zv_shapers(freq_x=50.0, freq_y=50.0,
                                         max_accel=50000.0,
                                         corner_deviation=0.1)
    prev = _make_move(th, start=(0, 0, 0, 0), end=(1.0, 0, 0, 0),
                      cruise_v=300.0)
    theta = _m.pi - 1e-5
    nxt = _make_move(th, start=(1.0, 0, 0, 0),
                     end=(1.0 + _m.cos(theta)*1.0, _m.sin(theta)*1.0, 0, 0),
                     cruise_v=300.0)
    cb = blendplanner.CornerBlender(th, move_cls=_FakeMove)
    _ = cb.feed(prev)
    _ = cb.feed(nxt)
    # near-reversal via from_moves REVERSAL_EPS; suppressed_junction_v
    # returns a tiny positive cap.
    v = prev.next_junction_v_capped_to
    assert v is not None and math.isfinite(v) and 0.0 <= v < 10.0
```

- [ ] **Step 2: Run tests**

```bash
python3 -m pytest test/test_blendplanner.py -k 'narrow_reversal or near_reversal_without' -v
```
Expected: both pass with Task 3's implementation in place. If either fails, review Task 3's implementation.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "test/blendplanner: guard REVERSAL_EPS vs no-shaper dp fallback consistency

Regression guards around the narrow angle wedge where QuinticShape's
REVERSAL_EPS and blendplanner's dp<=-0.5 no-shaper fallback meet.
Confirms no uncapped case leaks through either branch."
```

---

## Task 5: Run the smooth-shapers merge

**Goal:** Merge `smooth-shapers` into `magnum-opus` with the helpers already in place; resolve conflicts per strategy.

**Files:** git merge operation; conflict resolution on 4 files (done in Tasks 6–8).

- [ ] **Step 1: Confirm clean working tree**

```bash
git status
```
Expected: on `magnum-opus`, nothing to commit (modulo the .dSYM/.claude ignored set).

- [ ] **Step 2: Start the merge (expect conflicts)**

```bash
git merge smooth-shapers --no-edit
```
Expected: `Automatic merge failed; fix conflicts and then commit the result.` Git reports conflicts in:
- `klippy/blendmath.py`
- `klippy/blendplanner.py`
- `test/test_blendmath.py`
- `test/test_blendplanner.py`

If any additional file conflicts, **stop and report** — spec predicted exactly these four.

- [ ] **Step 3: Run `git status`**

```bash
git status
```
Expected: `both modified` or `both added` listings for the 4 files.

Proceed to Task 6.

---

## Task 6: Resolve `klippy/blendmath.py` conflict

**Goal:** Finalize the merged `blendmath.py` so it has: magnum-opus's post-arc-deletion base, **plus** Task 2's ported helpers (already present on our side), **plus** `f1ec651d`'s `get_axis()` + smooth-param tolerance changes applied to `_extract_shapers` and `_sigma_T_max_from_toolhead`.

**Files:**
- Modify: `klippy/blendmath.py` (resolve conflict)

- [ ] **Step 1: Inspect the conflict**

```bash
grep -n '<<<<<<<\|=======\|>>>>>>>' klippy/blendmath.py | head -30
```
Note the conflict regions.

- [ ] **Step 2: Apply the resolution strategy**

For each conflict hunk, apply these rules (in priority order):

1. **Arc primitives (BlendArc, blend_geometry, segment_arc, blend_from_moves, _rotate)** — smooth-shapers kept them; magnum-opus deleted them. **Take magnum-opus side** (delete). These are gone from magnum-opus intentionally (Plan 1, commit `6d8e7fe6`).
2. **`_scv_equivalent_junction_v`, `suppressed_junction_v`** — smooth-shapers added them inside the arc code region; Task 2 added them to magnum-opus at module level. **Take magnum-opus side** (Task 2's version). Discard smooth-shapers' duplicate.
3. **`_extract_shapers` / `_sigma_T_max_from_toolhead`** — smooth-shapers' `f1ec651d` changes MUST be applied. The changes are:
   - `_sigma_T_max_from_toolhead`: replace direct attribute reads with `getattr(…, default)`:
     ```python
     freq = float(getattr(params, "shaper_freq", 0.0) or 0.0)
     stype = getattr(params, "shaper_type", "") or ""
     damp = float(getattr(params, "damping_ratio", 0.0) or 0.0)
     ```
   - `_extract_shapers`: same `getattr` pattern for the three params AND change `axis=axis_shaper.axis` to `axis=axis_shaper.get_axis()`.
4. **All other utilities (`vdot`, `vcross`, `vnorm`, …, `interpolate_extruder`)** — if they match, auto-resolve. If they differ, take whichever is richer (smooth-shapers' version likely has small improvements).

After resolving, re-inspect:
```bash
grep -n '<<<<<<<\|=======\|>>>>>>>' klippy/blendmath.py
```
Expected: no output (all markers removed).

- [ ] **Step 3: Verify the resolved file compiles**

```bash
python3 -c "from klippy import blendmath; print('ok')"
```
Expected: `ok`. If `ImportError` or `SyntaxError`, re-inspect the file.

- [ ] **Step 4: Run blendmath tests**

```bash
python3 -m pytest test/test_blendmath.py -v
```
Some tests may still fail at this point (test file has its own conflicts unresolved; tackled in Task 7). Report the count but do not commit yet.

Do not commit — we finish the merge in Task 10.

---

## Task 7: Resolve `test/test_blendmath.py` conflict

**Goal:** Finalize the merged test file with: magnum-opus's quintic-era tests, plus Task 2's `suppressed_junction_v` tests (already on our side), plus `f1ec651d`'s `get_axis()`-aware test fakes + regression tests.

**Files:**
- Modify: `test/test_blendmath.py` (resolve conflict)

- [ ] **Step 1: Inspect conflict regions**

```bash
grep -n '<<<<<<<\|=======\|>>>>>>>' test/test_blendmath.py | head -40
```

- [ ] **Step 2: Apply resolution**

1. **Arc test bodies** (`test_blendarc_*`, `test_blend_geometry_*`, etc.) — **take magnum-opus side** (deleted).
2. **`_FakeAxisInputShaper` class** — **take smooth-shapers side**. Must expose `get_axis()` and `get_type()` methods and use `_axis` private attribute (not public `.axis`). Reference diff (smooth-shapers `f1ec651d`):
   ```python
   class _FakeAxisInputShaper:
       """Mirrors the API of klippy.extras.input_shaper.AxisInputShaper.

       The real class exposes axis access via ``get_axis()``, not a direct
       ``.axis`` attribute — regression: test/test_blendmath.py used to
       expose ``.axis`` directly and masked a blendmath bug on real hardware.
       """

       def __init__(self, axis, shaper_type, freq, damping_ratio=0.1):
           self._axis = axis
           self._type = shaper_type
           self._freq = freq
           self._damping = damping_ratio

       def get_axis(self):
           return self._axis

       def get_type(self):
           return self._type

       class _Params:
           def __init__(self, outer):
               self.axis = outer._axis
               self.shaper_type = outer._type
               self.shaper_freq = outer._freq
               self.damping_ratio = outer._damping
       # (keep the rest of _Params wiring from whichever side has it)
   ```
   Check the full class structure from smooth-shapers:
   ```bash
   git show smooth-shapers:test/test_blendmath.py | grep -A 40 'class _FakeAxisInputShaper'
   ```
3. **Two new regression tests** from `f1ec651d`:
   - `test_extract_shapers_uses_real_axis_input_shaper_api`
   - `test_extract_shapers_smooth_family_axis_has_zero_A`
   
   Both **take smooth-shapers side**. Run:
   ```bash
   git show smooth-shapers:test/test_blendmath.py | sed -n '/test_extract_shapers_uses_real_axis_input_shaper_api/,/^$/p' | head -30
   git show smooth-shapers:test/test_blendmath.py | sed -n '/test_extract_shapers_smooth_family_axis_has_zero_A/,/^$/p' | head -20
   ```
   Copy both verbatim into the resolved file.
4. **Existing tests that used `.axis` directly on the fake** — update to use `.get_axis()`. Grep for `_FakeAxisInputShaper` callsites and any `.axis` reads on those instances.
5. **`suppressed_junction_v` / `_scv_equivalent_junction_v` tests** — Task 2 already put these on magnum-opus side. Smooth-shapers also has them. **Take either** (they should be identical content; if not, manually reconcile).

After resolving:
```bash
grep -n '<<<<<<<\|=======\|>>>>>>>' test/test_blendmath.py
```
Expected: no output.

- [ ] **Step 3: Run tests**

```bash
python3 -m pytest test/test_blendmath.py -v
```
Expected: all pass. If failures cluster around `AttributeError: ... no attribute 'axis'`, missed a `.axis` → `.get_axis()` conversion.

Do not commit — finish merge in Task 10.

---

## Task 8: Resolve `klippy/blendplanner.py` + `test/test_blendplanner.py` conflicts

**Goal:** Finalize both files: magnum-opus's quintic-era planner wiring, Task 3's `suppressed_junction_v` wire-in (already on our side), plus `f1ec651d`'s test-fake API update on the test side.

**Files:**
- Modify: `klippy/blendplanner.py` (resolve conflict)
- Modify: `test/test_blendplanner.py` (resolve conflict)

- [ ] **Step 1: Inspect `klippy/blendplanner.py`**

```bash
grep -n '<<<<<<<\|=======\|>>>>>>>' klippy/blendplanner.py
```

- [ ] **Step 2: Resolve `klippy/blendplanner.py`**

The only substantive change in smooth-shapers was `04943583`'s wire-in of `suppressed_junction_v`. Task 3 already put the equivalent wire-in on magnum-opus (adapted for `QuinticShape.from_moves` instead of `blend_from_moves`). **Take magnum-opus side** on the `if shape is None:` region.

Other hunks (if any) — if smooth-shapers has whitespace/comment tweaks, prefer the magnum-opus substance (quintic-era signatures) but adopt any meaningful clarifications.

```bash
grep -n '<<<<<<<\|=======\|>>>>>>>' klippy/blendplanner.py
```
Expected: no output.

- [ ] **Step 3: Resolve `test/test_blendplanner.py`**

```bash
grep -n '<<<<<<<\|=======\|>>>>>>>' test/test_blendplanner.py | head -30
```

Apply:
1. **`_FakeAxisIS` class** — **take smooth-shapers side** (the `f1ec651d` version with `get_axis()` + `get_type()` methods and `_axis` private). Reference:
   ```bash
   git show smooth-shapers:test/test_blendplanner.py | grep -A 15 'class _FakeAxisIS'
   ```
2. **Tests that used `.axis` directly on `_FakeAxisIS`** — update to `.get_axis()`.
3. **Tests from Tasks 3 & 4** — already present on magnum-opus side. Keep.
4. **smooth-shapers' new `suppressed_junction_v` regression tests** (from `04943583`) — if they exist and differ from Task 3's, adopt whichever is more thorough; likely keep magnum-opus's since they target QuinticShape, but port the smooth-shapers' testing ideas where useful. When in doubt, keep both.

```bash
grep -n '<<<<<<<\|=======\|>>>>>>>' test/test_blendplanner.py
```
Expected: no output.

- [ ] **Step 4: Run the affected test modules**

```bash
python3 -m pytest test/test_blendplanner.py test/test_blendmath.py -v
```
Expected: all pass.

Do not commit — finish in Task 10.

---

## Task 9: Add interaction check — smooth-shaper × quintic `v_cap_fn`

**Goal:** Verify `QuinticShape.v_cap_fn` behaves sanely when one or both axes are smooth-family shapers (i.e. `A_axis = 0.0`). Expected behavior: the shaper term drops out of the v_cap min, and the function returns a finite positive bound from a_max / v_max alone.

**Files:**
- Modify: `test/test_blendquintic.py`

- [ ] **Step 1: Write the interaction test**

Add to `test/test_blendquintic.py`:
```python
def test_v_cap_fn_degrades_gracefully_with_smooth_shaper_axis():
    """When a smooth-family axis is passed in, _extract_shapers records
    A_axis=0.0 (see test_blendmath.py::test_extract_shapers_smooth_family_axis_has_zero_A).
    QuinticShape.v_cap_fn must not crash or return zero from that — the
    shaper term should drop out, leaving a_max / v_max bounds intact.
    """
    from klippy import blendshape, blendshaper
    # Craft a KinematicLimits with one impulse axis (A_axis > 0) and one
    # smooth axis (A_axis = 0).
    shapers = [
        blendshaper.AxisShaperSnapshot(
            axis="x", shaper_type="zv", shaper_freq=50.0,
            damping_ratio=0.1, A_axis=30000.0,
        ),
        blendshaper.AxisShaperSnapshot(
            axis="y", shaper_type="smooth_mzv", shaper_freq=0.0,
            damping_ratio=0.0, A_axis=0.0,
        ),
    ]
    limits = blendshape.KinematicLimits(
        a_max=50000.0, v_max=600.0, jerk_max=None,
        extruder_caps=None, shapers=shapers,
    )
    # Right-angle corner to exercise both axes.
    prev = _FakeMove(axes_r=(1.0, 0.0, 0.0), move_d=5.0,
                     accel=50000.0, end_pos=(5.0, 0.0, 0.0, 0.0))
    nxt  = _FakeMove(axes_r=(0.0, 1.0, 0.0), move_d=5.0,
                     accel=50000.0, start_pos=(5.0, 0.0, 0.0, 0.0))
    shape = blendquintic.QuinticShape.from_moves(
        prev, nxt, corner_deviation=0.2, limits=limits,
    )
    assert shape is not None
    v_mid = shape.v_cap_fn(shape.arc_length / 2.0)
    assert math.isfinite(v_mid)
    assert v_mid > 0.0
    # Sanity: without shaper involvement for y, the cap should be no
    # tighter than a_max-derived centripetal * v_max bound; in particular
    # it must not collapse to 0.
    assert v_mid >= 50.0  # extremely lax lower bound
```

If `_FakeMove` in `test_blendquintic.py` doesn't have `start_pos`/`end_pos`, add fields compatible with the existing test patterns.

- [ ] **Step 2: Run the test**

```bash
python3 -m pytest test/test_blendquintic.py -k 'degrades_gracefully' -v
```
**Two possible outcomes:**

- **PASS**: the smooth-shaper × quintic interaction is already graceful. Proceed to Step 3.
- **FAIL**: `v_cap_fn` returned 0 or raised an exception. Investigate — likely a division-by-zero or a `min(…)` consuming `A_axis=0.0`. **Do not fix in Plan 2**; instead:
  1. Mark the test with `@pytest.mark.xfail(reason="Smooth-shaper × quintic v_cap_fn degradation — follow-up, see Plan 2 spec §Phase A open sub-questions")`.
  2. File the behavior as a known issue at the top of `klippy/blendquintic.py` with a `# TODO(plan-2-followup): …` comment referencing the spec.
  3. Proceed. This is explicitly out of Plan 2's scope (spec §Out of scope).

- [ ] **Step 3: Stage the test**

The merge is still in progress (started in Task 5; finalized in Task 10). Stage this test so it rolls into the merge commit:

```bash
git add test/test_blendquintic.py
# Also stage klippy/blendquintic.py if a TODO comment was added:
# git add klippy/blendquintic.py
```

Do NOT commit here — Task 10 makes the merge commit and captures this test alongside the conflict resolutions.

---

## Task 10: Finalize the merge

**Goal:** Run the full test suite on the resolved tree, then commit the merge.

**Files:** none directly; just the commit.

- [ ] **Step 1: Confirm no conflict markers anywhere**

```bash
grep -rn '<<<<<<<\|=======\|>>>>>>>' klippy/ test/ --include='*.py' --include='*.md' | head
```
Expected: no output.

- [ ] **Step 2: Run the full host-side test suite**

```bash
python3 -m pytest test/ -v 2>&1 | tail -40
```
Expected: all tests pass. Current magnum-opus has 355; smooth-shapers will add ~30–80 more (mostly input_shaper / shaper_calibrate / extruder tests). If anything fails:
1. Read the failure.
2. If it's an `AttributeError` about `.axis`, fix the remaining test fake miss.
3. If it's an extruder or shaper test, inspect whether the resolution dropped a needed import or symbol.
4. If uncertain, **stop and report**.

- [ ] **Step 3: Stage the merge**

```bash
git add klippy/blendmath.py klippy/blendplanner.py test/test_blendmath.py test/test_blendplanner.py test/test_blendquintic.py
# Add klippy/blendquintic.py too if a TODO comment was inserted in Task 9.
```

Do NOT use `git add -A` or `git add .`. Only the files modified in this merge.

- [ ] **Step 4: Inspect the staged merge**

```bash
git status
git diff --cached --stat | tail -20
```
Expected: 5 files staged (6 if `blendquintic.py` got a TODO). No stray `.claude/`, `.dSYM/`, `hostsimulator.config` entries.

- [ ] **Step 5: Commit the merge**

```bash
git commit -m "merge: bring smooth-shapers into magnum-opus

Integrates the smooth-shapers branch into magnum-opus: polynomial
smooth input shapers (zv/mzv impulse retained), non-linear PA, extruder
sync with IS, shaper-calibrate improvements, and two local fixes
(f1ec651d get_axis + smoother-param tolerance; 04943583
suppressed_junction_v cap — both already ported to magnum-opus in
pre-merge commits and verified against the quintic codepath).

Conflict resolutions (4 files):
- klippy/blendmath.py: keep magnum-opus arc deletions; re-apply
  f1ec651d getattr/get_axis changes to _extract_shapers and
  _sigma_T_max_from_toolhead; suppressed_junction_v helpers already
  pre-ported.
- klippy/blendplanner.py: keep magnum-opus quintic-era wiring; the
  suppressed_junction_v wire-in was already pre-applied.
- test/test_blendmath.py: adopt smooth-shapers test fakes
  (get_axis/get_type methods, no raw .axis); port f1ec651d real-shaper
  regression tests (TypedInputShaperParams / TypedInputSmootherParams).
- test/test_blendplanner.py: mirror test-fake updates.

Interaction check added (test_blendquintic.py): smooth-shaper × quintic
v_cap_fn degrades gracefully when A_axis=0.0 is present. REVERSAL_EPS
vs blendplanner dp<=-0.5 wedge confirmed consistent.

See docs/superpowers/specs/2026-04-21-plan2-smooth-shapers-merge-plus-hpstepcompress-design.md"
```

If work hours: stage only and note. The merge stays active until committed.

Phase A complete.

---

## Task 11: Cherry-pick HP-stepcompress core

**Goal:** Apply `9c49716e` (stepper: New optional high precision stepping protocol) from `upstream/bleeding-edge-v2`.

**Files touched by the cherry-pick:**
- `klippy/chelper/stepcompress.c` (modified)
- `klippy/chelper/stepcompress.h` (modified)
- `klippy/chelper/stepcompress_hp.c` (new, 621 lines)
- `klippy/chelper/__init__.py` (modified)
- `klippy/stepper.py` (modified)
- `src/stepper.c` (modified)

- [ ] **Step 1: Confirm working tree clean on magnum-opus**

```bash
git status
```
Expected: on `magnum-opus`, nothing staged. If Task 10's merge commit wasn't made (work-hours hold), **stop here** and do Phase B in the next off-hours session instead — Phase B commits on top of Phase A's merge.

- [ ] **Step 2: Cherry-pick the commit**

```bash
git cherry-pick 9c49716e
```
Expected: clean apply. If conflicts, **stop and report** — spec predicts a clean cherry-pick based on common-ancestor state.

If the cherry-pick succeeds, git will create the commit automatically. If not (e.g., the commit is empty or already applied), investigate before continuing.

- [ ] **Step 3: Verify the commit**

```bash
git show --stat HEAD | head -15
```
Expected: shows 6 files, +903 lines, subject `stepper: New optional high precision stepping protocol`.

- [ ] **Step 4: Build chelper on host**

```bash
cd klippy/chelper && make && cd ../..
```
Expected: clean build. Warnings are OK; errors are not.

- [ ] **Step 5: Run the Python test suite**

```bash
python3 -m pytest test/ 2>&1 | tail -10
```
Expected: same pass count as after Task 10. HP-stepcompress is Kconfig-gated in the MCU firmware but the host build + Python test suite should be unaffected (new code is compiled but not executed by default).

No additional commit — the cherry-pick IS the commit.

---

## Task 12: Cherry-pick HP-stepcompress Kconfig opt-in

**Goal:** Apply `b2854f71` (stepper: Optionally enable new stepcompress protocol in MCU firmware).

**Files touched:**
- `klippy/stepper.py` (modified)
- `src/Kconfig` (+6 lines)
- `src/avr/Kconfig` (+8 lines)
- `src/stepper.c` (modified)

- [ ] **Step 1: Cherry-pick**

```bash
git cherry-pick b2854f71
```
Expected: clean apply.

- [ ] **Step 2: Verify**

```bash
git show --stat HEAD | head -15
```
Expected: 4 files changed, +94/-60 lines, subject `stepper: Optionally enable new stepcompress protocol in MCU firmware`.

- [ ] **Step 3: Rebuild chelper**

```bash
cd klippy/chelper && make && cd ../..
```
Expected: clean.

---

## Task 13: Full test + build pass

**Goal:** Confirm both phases landed clean; no test regressions, chelper builds, Kconfig option exists.

**Files:** none (verification only).

- [ ] **Step 1: Full pytest suite**

```bash
python3 -m pytest test/ 2>&1 | tail -20
```
Expected: all pass. Record the count for future reference.

- [ ] **Step 2: Kconfig sanity**

```bash
grep -n 'STEPPER_HIGH_PRECISION\|HIGH_PRECISION_PROTOCOL\|PROTOCOL' src/Kconfig src/avr/Kconfig | head -10
```
Expected: at least one match — the new opt-in option (exact symbol name depends on upstream; look for something like `CONFIG_HIGH_PRECISION_STEP_PROTOCOL` or `CONFIG_WANT_HIGH_PRECISION_STEPPING`). Record the symbol name for the commit message.

- [ ] **Step 3: MCU firmware build spot-check (optional, user-requested)**

If the user wants an MCU build validated in this session:
```bash
# The user's MCU is typically stm32 on Trident.
cd ~/Developer/kalico  # already here
KCONFIG_CONFIG=.config-stm32 make menuconfig  # select stm32; select the new HP option
make KCONFIG_CONFIG=.config-stm32
```
Expected: firmware binary built. This is the user's job to validate on the printer — plan's success criterion (spec §Success criteria #4) only requires the host-side Python + chelper to pass.

**Skip this step if the user has not asked for it** — MCU builds are their workflow.

- [ ] **Step 4: Final git log check**

```bash
git log --oneline magnum-opus~10..HEAD | head -15
```
Expected (off-hours path): shows Task 2 commit, Task 3 commit, Task 4 commit, Task 10 merge commit, Task 11 cherry-pick, Task 12 cherry-pick. That's 6 commits on top of the pre-Plan-2 head.

(Work-hours path: commits still staged/held; verify git status confirms the state.)

- [ ] **Step 5: Summary report**

Report back:
1. How many tests pass vs pre-Plan-2 (355 was the Plan-1 final count).
2. What the Kconfig option is named.
3. Whether any Task 9 TODO was filed (smooth-shaper × quintic v_cap_fn interaction).
4. Any deviations from the plan.

Plan 2 complete.

---

## Open items that do NOT block Plan 2 completion

These were flagged in the spec as out of scope but may surface during implementation:

1. **Smooth-shaper-aware in-blend `v_cap_fn`**: if Task 9 reveals the degradation isn't graceful, a follow-up plan adds a smooth-family cap computation. Pillar 2 (unified `v(s)`) territory.
2. **Non-linear PA integration with `blendshape.ExtruderLimits`**: smooth-shapers brings non-linear PA as Python code; Plan 3 wires it into magnum-opus's extruder-first-class framework.
3. **MCU firmware HW validation**: user runs their own HW tests on Trident / V0.
