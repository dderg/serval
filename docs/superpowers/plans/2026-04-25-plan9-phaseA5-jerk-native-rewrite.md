# Plan 9 Phase A5 — Jerk-native Lookahead Rewrite

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the trapezoidal-era LookAheadQueue reverse pass with a jerk-native reachability cascade, delete the smoothed pass and its supporting Move attributes, and close the bed_mesh "Jerk profile infeasible" crash by making the reverse-pass feasibility check agree with `set_junction`'s feasibility check.

**Architecture:** Today's `LookAheadQueue.flush` is patches-on-trapezoidal — only the accel-side reachability (`reachable_v_from_v_end`) was replaced; the cruise-cap formula `(start_v² + reachable_start_v²) * 0.5`, the `peak_cruise_v2` averaging, the separate smoothed pass (backed by the deprecated `max_accel_to_decel` config), `smooth_delta_v2`, `max_smoothed_v2`, and the forward `delta_v2` cap in `calc_junction` all remain. Under jerk motion these trapezoidal caps are too loose — they let `set_junction` receive a `(start_v, cruise_v, end_v, L)` tuple that `jerk_profile.compute_profile` rejects as infeasible, which is exactly the bed_mesh crash. A5 makes the reverse pass jerk-feasible by construction: cruise_v is capped to `reachable_v_end(end_v, a_max, j_max, L_decel) ∩ reachable_v_end(start_v, a_max, j_max, L_accel)` using the accel-distance / decel-distance split that actually fits the move, the smoothed pass and `max_accel_to_decel` are deleted, `min_cruise_ratio` as a config knob is retired, `delta_v2` is retired from the forward cap in favour of a jerk-aware forward reachability check, and `smooth_delta_v2` / `max_smoothed_v2` are excised from every consumer. Pure-E moves remain trapezoidal (no XY path, no shaper bake) but their reverse-pass math is simplified to use the same jerk-aware reachability primitive with `j_max=max_jerk` from the toolhead — one control flow for both kinematic and pure-E moves. Centripetal in `calc_junction` stays in its A2c form (decoupled from `delta_v2`) — it is a geometric cap, not a trapezoidal artifact, and is still correct under jerk.

**Tech Stack:** Python 3 (planner), existing `klippy/jerk_math.py` (closed-form jerk-aware reachable-velocity), `klippy.chelper.jerk_profile.compute_profile` (feasibility oracle), pytest.

---

## Scope

**In scope:**
- `klippy/toolhead.py` — `Move.__init__`, `Move.calc_junction`, `Move.set_junction`, `Move.limit_speed`, `LookAheadQueue.flush`, `ToolHead.__init__` (config retirement), `ToolHead.max_accel_to_decel` (deletion).
- `klippy/blendplanner.py` — `QuinticBlendMove.__init__`, `QuinticBlendMove.calc_junction`, `QuinticBlendMove.finalize_shape`, `QuinticBlendMove.limit_speed`, `_copy_caller_state`.
- Test harness: migrate every stub toolhead in `test/` that sets `max_accel_to_decel` / `smooth_delta_v2` / `max_smoothed_v2`.
- Removal of the deprecated `max_accel_to_decel` / `minimum_cruise_ratio` / `MINIMUM_CRUISE_RATIO` / `ACCEL_TO_DECEL` config/gcode knobs.
- `klippy/extras/trad_rack.py` — one residual `max_accel_to_decel` deprecate call to remove.

**Out of scope (do not address):**
- Phase B (host↔MCU protocol). A5 is planner-only.
- `shape_disabled` bypass audit (Plan 9 A6 deferred).
- Plan 9 A4 (cross-blend-boundary continuity) — tracked by `docs/superpowers/plans/2026-04-24-plan9-phaseA4-cross-boundary-shape-bake.md`. A5 does not depend on A4, and A4 does not depend on A5 — they are independent.
- Per-axis `max_jerk` (spec decision #5). The toolhead-scope `max_jerk` stays the single jerk knob; per-axis is a future phase.
- Extruder PA-derivative feedback into the profile generator (spec A5 meaning — conflict in naming; the user's A5 here is the lookahead rewrite).
- Homing / `drip_move` / `force_move` jerk-awareness. These go through `Move` as-is today; the reverse pass never runs against a homing move (drip path short-circuits).

---

## File structure

No new files. All edits are to existing modules.

**Modified files and responsibilities:**
- `klippy/toolhead.py` — `Move` shrinks (no `smooth_delta_v2`, no `max_smoothed_v2`, no `delta_v2`); `LookAheadQueue.flush` loses its smoothed pass, its cruise-cap averaging, and its `(start_v² + reachable_start_v²) * 0.5` trapezoidal cruise cap. `calc_junction` loses the `delta_v2` forward cap in favour of a jerk-aware forward reachability call. The `max_accel_to_decel` property is deleted; the config deprecation branch is deleted.
- `klippy/blendplanner.py` — `QuinticBlendMove` loses `smooth_delta_v2`, `max_smoothed_v2`, `delta_v2`; `calc_junction` loses the `delta_v2` forward cap; `_copy_caller_state` is trimmed accordingly.
- `klippy/extras/trad_rack.py` — remove the `max_accel_to_decel` deprecation branch (it chains to `toolhead.max_accel_to_decel` which is gone).
- All `test/test_*.py` stubs — drop `max_accel_to_decel` / `smooth_delta_v2` / `max_smoothed_v2` plumbing.

---

## Key context for the implementing engineer

### The bed_mesh bug this plan fixes

During `bed_mesh_calibrate`, a short probe move arrives at the reverse pass with `start_v=374.7`, `cruise_v=469.8`, `end_v=469.8`, `move_d=1.143 mm`, `accel=70000 mm/s²`, `j_max=500000 mm/s³`. Today's flush gives it the green light via the trapezoidal cruise cap `(start_v² + reachable_start_v²) * 0.5 = mean of the accel-side bounds`. Then `set_junction` calls `jerk_profile.compute_profile` which rejects the tuple because under j=500k the 374.7 → 469.8 ramp needs `~11.65 mm`, not 1.14. The trapezoidal cap computes "0.57 mm is enough" because `dv² / (2·a) = 95² / 140000 ≈ 0.064 → L = 0.57` — off by 20×.

**Verification:**
```
$ python3 -c "from klippy import jerk_math; import math
v_start = 374.7; v_end = 469.8; a = 70000.0; j = 500000.0
lo, hi = 0.0, 1000.0
for _ in range(100):
    mid = (lo + hi) / 2.0
    if jerk_math.reachable_v_end(v_start, a, j, mid) < v_end: lo = mid
    else: hi = mid
print(f'jerk-aware L = {hi:.3f} mm')
print(f'trapezoid L = {(v_end**2 - v_start**2) / (2*a):.3f} mm')"
jerk-aware L = 11.647 mm
trapezoid L = 0.574 mm
```

The bug is closed iff the reverse pass's chosen `cruise_v` is clipped to a value such that `jerk_profile.compute_profile(start_v, end_v, cruise_v, L, a_max, j_max)` never returns a non-OK status. This plan does that by construction.

### Today's lookahead contract (what's being replaced)

`LookAheadQueue.flush` (in `klippy/toolhead.py`, lines 419-593) does two interleaved reverse passes:

1. **"Real" reverse pass** using `move.accel` / `move.j_max`:
   - `reachable_start_v² = reachable_v_from_v_end(sqrt(next_end_v²))` (line ~463-466). **Jerk-aware.**
   - `cruise_v² = min((start_v² + reachable_start_v²) * 0.5, max_cruise_v², peak_cruise_v²)` (line ~507-511). **Trapezoidal: this is the bug.**
   - `peak_cruise_v² = min(max_cruise_v², (smoothed_v² + reachable_smoothed_v²) * 0.5)` (line ~492-495). **Trapezoidal.**
   - `set_junction(min(start_v², cruise_v²), cruise_v², min(next_end_v², cruise_v²))` — triggers feasibility check.

2. **Smoothed pass** using `move.toolhead.max_accel_to_decel`:
   - `reachable_smoothed_v² = reachable_v_end(..., a_max=max_accel_to_decel, j_max=move.j_max)` (line ~473-479). **Half-jerk-aware, half-legacy.**
   - Decides whether this move can accelerate via `smoothed_v² + move.smooth_delta_v2 > next_smoothed_v²` (line ~484). **Trapezoidal.**
   - Drives `delayed[]` queue and `peak_cruise_v²` propagation.

The smoothed pass exists in Klipper-era code because trapezoidal motion is snap-limited and a gentler "smoothed acceleration" pass gave better cornering decisions. Under jerk motion, smoothness is built into the jerk profile itself — this entire second pass is redundant.

`Move.calc_junction` (lines 229-275) computes `max_start_v² = min(..., prev_move.max_start_v² + prev_move.delta_v²)`. The `delta_v² = 2 * move_d * accel` term is the constant-accel forward reachability — with `a=70k` and `move_d=40 mm` this gives `delta_v² = 5.6M`, sqrt ≈ 2366 mm/s, which is a huge over-estimate for any physically realizable motion. Forward reachability is still useful — we just want the jerk-aware version: `reachable_v_end(prev_start_v, a_max, j_max, prev_move.move_d)`. The centripetal cap (lines 261-270, written in A2c from the physical `0.5 * L * accel * tan(θ/2)` form directly) stays — it's geometric, not trapezoidal.

### Jerk-aware reverse-pass math (the new primitive)

Given a move `(start_v, end_v, L, a_max, j_max)`, the feasibility question is: what's the largest `cruise_v` such that (i) an accel ramp from `start_v` to `cruise_v` fits in some `L_accel`, and (ii) a decel ramp from `cruise_v` to `end_v` fits in `L - L_accel`? Under the jerk profile we have two monotonic constraints:

- `cruise_v ≤ reachable_v_end(start_v, a_max, j_max, L_accel)`
- `cruise_v ≤ reachable_v_end(end_v, a_max, j_max, L - L_accel)` (decel, by time-reversal symmetry)

Both sides of `L_accel` increase `cruise_v` monotonically up to the point where the two ramps meet. The largest reachable `cruise_v` given `(start_v, end_v, L, a_max, j_max)` is the max over `L_accel ∈ [0, L]` of `min(ramp_from_start(L_accel), ramp_from_end(L - L_accel))`. Because `ramp_from_start` is monotonically increasing in `L_accel` and `ramp_from_end` is monotonically decreasing, the max is the crossover point.

**Closed-form / bisection decision.** `reachable_v_end` is monotonic and continuous in `L`, so `L_accel*` solving `ramp_from_start(L_accel) = ramp_from_end(L - L_accel)` is a 1D root-find. 25 iterations of bisection converge to `1e-8` mm precision in single-digit microseconds — negligible compared to the rest of the flush pass. We use bisection rather than a closed form because `reachable_v_end` itself is a two-regime (triangular/trapezoidal in jerk) piecewise function and chasing a closed solution across its regimes is brittle.

**Short-circuit cases:**
- If `reachable_v_end(start_v, a_max, j_max, L) <= end_v`: the move cannot even reach `end_v` from `start_v` — the reverse pass upstream of this move must lower its demanded end_v. This case is handled by the reachable-from-end clamp we already propagate backwards.
- If `ramp_from_start(L) >= max_cruise_v` AND `ramp_from_end(L) >= max_cruise_v`: the move is trivially at-cruise-capable — skip bisection and return `max_cruise_v`.

**New helper:** `jerk_math.max_reachable_cruise_v(v_start, v_end, a_max, j_max, L, v_cruise_cap)` — the single primitive the new reverse pass calls. Returns the largest `cruise_v ≤ v_cruise_cap` feasible under the given constraints.

---

## Task decomposition

Six tasks, two of which are test migration (mechanical), the rest are design-and-implement. Model recommendations per task — `opus` for design judgement, `sonnet` for mechanical work.

- **Task 1** — Jerk-aware max cruise primitive (`opus`)
- **Task 2** — Reverse pass rewrite: call Task 1, delete trapezoidal cruise cap, delete smoothed pass (`opus`)
- **Task 3** — Forward pass rewrite: `calc_junction` uses jerk-aware forward reachability, delete `delta_v2` / `smooth_delta_v2` (`opus`)
- **Task 4** — Retire `max_accel_to_decel` / `minimum_cruise_ratio` config + gcode surface (`sonnet`)
- **Task 5** — Test migration: drop `max_accel_to_decel` / `smooth_delta_v2` / `max_smoothed_v2` stubs (`sonnet`)
- **Task 6** — bed_mesh regression test + dogfood flush run through the original crash inputs (`opus`)

---

### Task 1: Jerk-aware max cruise primitive

**Model:** opus

**Files:**
- Modify: `klippy/jerk_math.py` — add `max_reachable_cruise_v`
- Test: `test/test_jerk_math.py` — add regression cases covering triangular, trapezoidal, bisection crossover, and the bed_mesh numbers

- [ ] **Step 1: Write the failing unit tests**

Create `test/test_jerk_math.py` if it does not already exist; otherwise append. This file must not import anything from `klippy/toolhead.py` — `jerk_math` is a pure-math leaf.

```python
# test/test_jerk_math.py (additions)
"""Phase A5 — max_reachable_cruise_v primitive.

The inverse of reachable_v_end: given start_v and end_v at either end of
a segment of length L under (a_max, j_max), compute the largest cruise_v
such that the move is jerk-feasible.
"""
import math
import pytest
from klippy import jerk_math


def test_max_cruise_v_trivial_at_cap_when_long():
    # Long segment: starting and ending at low v, cap at 500 mm/s.
    # 100 mm is plenty to reach 500 mm/s under a=5000, j=1e5.
    v = jerk_math.max_reachable_cruise_v(
        v_start=100.0, v_end=100.0, a_max=5000.0, j_max=100000.0,
        L=100.0, v_cruise_cap=500.0,
    )
    assert v == pytest.approx(500.0, rel=1e-9)


def test_max_cruise_v_equals_endpoints_when_no_distance():
    # No distance: cruise_v collapses to the tighter of the two endpoints.
    v = jerk_math.max_reachable_cruise_v(
        v_start=200.0, v_end=300.0, a_max=5000.0, j_max=100000.0,
        L=0.0, v_cruise_cap=1e9,
    )
    # With L=0 no ramp is possible; the only feasible cruise is
    # min(v_start, v_end) (or, equivalently, the bisection collapses).
    assert v == pytest.approx(200.0, rel=1e-9)


def test_max_cruise_v_symmetric_triangular():
    # Start and end equal, short L — answer is the triangular peak.
    # v_peak = v0 + u^2 where u = (L * sqrt(j)) ^ (1/3) from rest,
    # but with v_start=v_end=100 and L=10, the accel side consumes L_acc,
    # decel side consumes L-L_acc, both with same v_start/v_end branch;
    # crossover is at L/2 by symmetry.
    L = 10.0
    v = jerk_math.max_reachable_cruise_v(
        v_start=100.0, v_end=100.0, a_max=5000.0, j_max=100000.0,
        L=L, v_cruise_cap=1e9,
    )
    expected = jerk_math.reachable_v_end(
        v_start=100.0, a_max=5000.0, j_max=100000.0, L=L * 0.5,
    )
    assert v == pytest.approx(expected, rel=1e-6)


def test_max_cruise_v_bed_mesh_crash_inputs():
    # The exact numbers from the bed_mesh crash. start_v=374.7, end_v=469.8,
    # L=1.143, a=70000, j=500000. Under the trapezoidal cap
    # (sqrt(2*a*L) and cousins) this let infeasible cruise_v through;
    # max_reachable_cruise_v MUST return something feasible for set_junction.
    v = jerk_math.max_reachable_cruise_v(
        v_start=374.7, v_end=469.8, a_max=70000.0, j_max=500000.0,
        L=1.143, v_cruise_cap=469.8,
    )
    # Feasibility: reachable_v_end(v_start, a, j, L_accel) >= v must hold
    # for some 0 <= L_accel <= L, and reachable_v_end(v_end, a, j, L - L_accel) >= v.
    # The bisection finds the crossover; we just check the value is no
    # greater than either endpoint's reach-from-L.
    assert v <= jerk_math.reachable_v_end(374.7, 70000.0, 500000.0, 1.143) + 1e-6
    assert v <= jerk_math.reachable_v_end(469.8, 70000.0, 500000.0, 1.143) + 1e-6
    # And the cruise_v returned must be such that v_start itself is reachable
    # from v (reverse direction): that is, v <= v_start or the decel fits.
    # For this input L is far too short to reach 469.8 from 374.7 — the
    # answer must clip below 469.8.
    assert v < 469.8


def test_max_cruise_v_bed_mesh_roundtrip_through_jerk_profile():
    # The acceptance test: the returned cruise_v MUST be feasible under
    # jerk_profile.compute_profile. This is the regression gate for the
    # bed_mesh crash.
    from klippy.chelper import jerk_profile as jp_mod
    v = jerk_math.max_reachable_cruise_v(
        v_start=374.7, v_end=469.8, a_max=70000.0, j_max=500000.0,
        L=1.143, v_cruise_cap=469.8,
    )
    # end_v cannot exceed cruise_v — cap it.
    end_v = min(469.8, v)
    start_v = min(374.7, v)
    prof = jp_mod.compute_profile(
        v0=start_v, v1=end_v, v_peak=v,
        a_max=70000.0, j_max=500000.0, L=1.143,
    )
    assert prof.status == jp_mod.JP_OK, (
        f"Jerk profile rejected A5 cruise_v={v:.6f} start_v={start_v:.6f} "
        f"end_v={end_v:.6f} L=1.143 (status={prof.status})"
    )


def test_max_cruise_v_obeys_cap():
    v = jerk_math.max_reachable_cruise_v(
        v_start=0.0, v_end=0.0, a_max=5000.0, j_max=100000.0,
        L=100.0, v_cruise_cap=250.0,
    )
    assert v == pytest.approx(250.0, rel=1e-9)


def test_max_cruise_v_rejects_non_finite():
    with pytest.raises(ValueError):
        jerk_math.max_reachable_cruise_v(
            v_start=float("nan"), v_end=0.0, a_max=1.0, j_max=1.0,
            L=1.0, v_cruise_cap=1.0,
        )
```

- [ ] **Step 2: Run the new tests and confirm they fail**

Run: `pytest test/test_jerk_math.py -v -k max_cruise`
Expected: ALL new tests FAIL with `AttributeError: module 'klippy.jerk_math' has no attribute 'max_reachable_cruise_v'`.

- [ ] **Step 3: Implement the primitive**

Append to `klippy/jerk_math.py`:

```python
def max_reachable_cruise_v(
    v_start: float, v_end: float, a_max: float, j_max: float,
    L: float, v_cruise_cap: float,
) -> float:
    """Largest cruise_v <= v_cruise_cap such that a jerk-limited accel
    ramp from v_start to cruise_v followed by a decel ramp from cruise_v
    to v_end fits within total distance L under (a_max, j_max).

    This is the A5 jerk-native replacement for the trapezoidal cruise cap
    ((v_start**2 + reachable_v_end_from_v_start**2) * 0.5) that Klipper's
    reverse pass used. Under that cap, a short move could be assigned a
    cruise_v that jerk_profile.compute_profile then rejected as infeasible.

    Short-circuits:
      * L == 0 returns min(v_start, v_end, v_cruise_cap).
      * If reachable_v_end from both endpoints at full L >= v_cruise_cap,
        returns v_cruise_cap (the move is at-cap-capable).
      * If v_cruise_cap <= min(v_start, v_end), returns v_cruise_cap
        (no acceleration needed on either side).

    Otherwise: bisect on L_accel in [0, L], solving
      reachable_v_end(v_start, a_max, j_max, L_accel)
        == reachable_v_end(v_end, a_max, j_max, L - L_accel).
    Both sides are monotonic and continuous; bisection is robust across
    the triangular/trapezoidal regime boundaries.
    """
    if not all(math.isfinite(x) for x in
               (v_start, v_end, a_max, j_max, L, v_cruise_cap)):
        raise ValueError(
            "max_reachable_cruise_v requires finite inputs; got "
            f"v_start={v_start!r}, v_end={v_end!r}, a_max={a_max!r}, "
            f"j_max={j_max!r}, L={L!r}, v_cruise_cap={v_cruise_cap!r}"
        )
    if v_start < 0.0 or v_end < 0.0:
        raise ValueError("v_start and v_end must be >= 0")
    if a_max <= 0.0 or j_max <= 0.0:
        raise ValueError("a_max and j_max must be > 0")
    if L < 0.0:
        raise ValueError("L must be >= 0")
    if v_cruise_cap <= 0.0:
        return 0.0
    if L == 0.0:
        return min(v_start, v_end, v_cruise_cap)
    # Short-circuit: cap is at or below both endpoints -- no acceleration
    # needed on either side to cruise at the cap.
    if v_cruise_cap <= min(v_start, v_end):
        return v_cruise_cap
    # Short-circuit: both ends can reach the cap in the full L — take it.
    reach_start_full = reachable_v_end(v_start, a_max, j_max, L)
    reach_end_full = reachable_v_end(v_end, a_max, j_max, L)
    if reach_start_full >= v_cruise_cap and reach_end_full >= v_cruise_cap:
        return v_cruise_cap
    # Bisection: find L_accel in [0, L] where ramp_from_start(L_accel) ==
    # ramp_from_end(L - L_accel). Monotonicity: ramp_from_start is
    # increasing in L_accel, ramp_from_end is decreasing.
    lo, hi = 0.0, L
    for _ in range(60):  # 2^-60 L is machine-epsilon territory.
        mid = (lo + hi) * 0.5
        v_from_start = reachable_v_end(v_start, a_max, j_max, mid)
        v_from_end = reachable_v_end(v_end, a_max, j_max, L - mid)
        if v_from_start < v_from_end:
            lo = mid
        else:
            hi = mid
    mid = (lo + hi) * 0.5
    crossover_v = min(
        reachable_v_end(v_start, a_max, j_max, mid),
        reachable_v_end(v_end, a_max, j_max, L - mid),
    )
    return min(crossover_v, v_cruise_cap)
```

- [ ] **Step 4: Run tests; confirm they pass**

Run: `pytest test/test_jerk_math.py -v -k max_cruise`
Expected: ALL 7 new tests PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/jerk_math.py test/test_jerk_math.py
git commit -m "plan9-A5 T1: jerk-aware max_reachable_cruise_v primitive"
```

---

### Task 2: Rewrite LookAheadQueue.flush reverse pass

**Model:** opus

**Files:**
- Modify: `klippy/toolhead.py` — `Move.__init__` (lines ~58-62), `Move.limit_speed` (lines ~73-80), `LookAheadQueue.flush` (lines ~419-593)
- Test: `test/test_toolhead_jerk_wiring.py`, `test/test_toolhead_jerk_integration.py` — new reverse-pass tests

This is the biggest step. The new reverse pass has one concept: propagate `end_v²` backward and, at each move, compute `cruise_v` via `jerk_math.max_reachable_cruise_v`, then call `set_junction`. No smoothed pass, no `peak_cruise_v²` averaging, no `delayed[]` queue.

- [ ] **Step 1: Write failing tests for the new reverse-pass contract**

Append to `test/test_toolhead_jerk_integration.py`:

```python
# Phase A5 — reverse pass is jerk-feasible by construction.

def test_reverse_pass_closes_bed_mesh_crash():
    """The original bed_mesh crash inputs, fed through Move +
    LookAheadQueue, must NOT raise 'Jerk profile infeasible'.
    """
    from klippy.toolhead import Move, LookAheadQueue

    class _Stub(_FakeToolhead):
        def __init__(self, **kw):
            super().__init__(**kw)
            self._captured = []
        def _process_moves(self, moves):
            self._captured.extend(moves)

    th = _Stub(max_accel=70000.0, max_jerk=500000.0, max_velocity=600.0)
    la = LookAheadQueue(th)
    # Recreate the crash pattern: a pre-probe cruise move feeding a
    # short 1.143 mm probe hop at 469.8 mm/s that lands at 469.8 mm/s
    # (probe drop into a subsequent move of equal speed).
    m_a = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=469.8)
    m_b = Move(th, (50, 0, 0, 0), (51.143, 0, 0, 0), speed=469.8)
    m_c = Move(th, (51.143, 0, 0, 0), (200, 0, 0, 0), speed=469.8)
    la.queue.extend([m_a, m_b, m_c])
    m_b.calc_junction(m_a)
    m_c.calc_junction(m_b)
    la.flush(lazy=False)
    # If the plan is correct, flush did not raise. The move's chosen
    # cruise_v must be feasible under jerk_profile.compute_profile
    # (implicitly checked by set_junction — if it wasn't, we'd have raised).
    for m in (m_a, m_b, m_c):
        assert hasattr(m, "jerk_profile")


def test_reverse_pass_no_smoothed_fields_on_move():
    """After A5, Move must not carry smoothed-pass state.

    The smoothed pass is dead — its backing fields should be gone so
    future code cannot accidentally read stale values.
    """
    from klippy.toolhead import Move
    th = _FakeToolhead()
    m = Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    assert not hasattr(m, "smooth_delta_v2"), \
        "A5 must remove smooth_delta_v2 from Move"
    assert not hasattr(m, "max_smoothed_v2"), \
        "A5 must remove max_smoothed_v2 from Move"


def test_reverse_pass_uses_max_reachable_cruise_v():
    """For a short move between two high-velocity moves, the chosen
    cruise_v must equal max_reachable_cruise_v(start_v, end_v, a, j, L).

    This is the structural assertion: the trapezoidal cruise cap is
    gone and the jerk-aware primitive is in its place.
    """
    from klippy import jerk_math
    from klippy.toolhead import Move, LookAheadQueue

    class _Stub(_FakeToolhead):
        def __init__(self, **kw):
            super().__init__(**kw)
            self._captured = []
        def _process_moves(self, moves):
            self._captured.extend(moves)

    th = _Stub(max_accel=5000.0, max_jerk=100000.0, max_velocity=500.0)
    la = LookAheadQueue(th)
    # Long flank, tiny middle, long flank — middle move's cruise_v is
    # constrained by jerk reachability from 500 mm/s ends across 2 mm.
    m_a = Move(th, (0, 0, 0, 0), (100, 0, 0, 0), speed=500.0)
    m_b = Move(th, (100, 0, 0, 0), (102, 0, 0, 0), speed=500.0)
    m_c = Move(th, (102, 0, 0, 0), (200, 0, 0, 0), speed=500.0)
    la.queue.extend([m_a, m_b, m_c])
    m_b.calc_junction(m_a)
    m_c.calc_junction(m_b)
    la.flush(lazy=False)
    # m_b's cruise_v should match the analytic jerk-aware cap.
    expected = jerk_math.max_reachable_cruise_v(
        v_start=m_b.start_v, v_end=m_b.end_v,
        a_max=m_b.accel, j_max=m_b.j_max,
        L=m_b.move_d, v_cruise_cap=500.0,
    )
    assert m_b.cruise_v == pytest.approx(expected, rel=1e-6)
```

- [ ] **Step 2: Run the new tests to confirm they fail**

Run: `pytest test/test_toolhead_jerk_integration.py -v -k "bed_mesh_crash or no_smoothed_fields or max_reachable_cruise_v"`
Expected: all three FAIL. The bed_mesh one fails with `command_error: Jerk profile infeasible`; the field-deletion tests fail because the fields still exist.

- [ ] **Step 3: Edit `Move.__init__` to drop the smoothed fields**

In `klippy/toolhead.py`, replace the block at lines ~55-63:

```python
        # Junction speeds are tracked in velocity squared.  max_start_v2
        # is set by calc_junction and tightened by the reverse pass
        # before set_junction is invoked.
        self.max_start_v2 = 0.0
        self.max_cruise_v2 = velocity**2
        self.next_junction_v2 = 999999999.9
```

The old `delta_v2`, `max_smoothed_v2`, `smooth_delta_v2` lines are deleted.

- [ ] **Step 4: Edit `Move.limit_speed` to drop delta_v2 / smooth_delta_v2 updates**

In `klippy/toolhead.py`, replace `limit_speed`:

```python
    def limit_speed(self, speed, accel):
        # Plan 9 A5: max cruise + accel are the only caller-mutable
        # kinematic limits that cross into the reverse pass; delta_v2
        # and smooth_delta_v2 (trapezoidal-era forward/smoothed caps)
        # are retired.
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)
```

- [ ] **Step 5: Edit `Move.calc_junction` to drop the delta_v2 forward cap and the smoothed propagation**

In `klippy/toolhead.py`, replace `calc_junction`. The centripetal cap (A2c form) stays because it's geometric, not trapezoidal. The forward reachability cap is swapped for the jerk-aware version.

```python
    def calc_junction(self, prev_move):
        # Plan 9 A5: forward junction cap is now jerk-aware. The old
        # prev_move.max_start_v2 + prev_move.delta_v2 term was the
        # constant-accel forward reachability; under jerk-limited motion
        # the correct bound is
        #   reachable_v_end(prev_start_v, a_max, j_max, prev.move_d).
        # Centripetal cap (A2c form) is kept — it is a geometric cap on
        # the corner radius, not a trapezoidal artifact.
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        extruder_v2 = self.toolhead.extruder.calc_junction(prev_move, self)
        prev_start_v = (math.sqrt(prev_move.max_start_v2)
                        if prev_move.max_start_v2 > 0.0 else 0.0)
        prev_forward_reach = jerk_math.reachable_v_end(
            v_start=prev_start_v,
            a_max=prev_move.accel, j_max=prev_move.j_max,
            L=prev_move.move_d,
        )
        max_start_v2 = min(
            extruder_v2,
            self.max_cruise_v2,
            prev_move.max_cruise_v2,
            prev_move.next_junction_v2,
            prev_forward_reach * prev_forward_reach,
        )
        axes_r = self.axes_r
        prev_axes_r = prev_move.axes_r
        junction_cos_theta = -(
            axes_r[0] * prev_axes_r[0]
            + axes_r[1] * prev_axes_r[1]
            + axes_r[2] * prev_axes_r[2]
        )
        sin_theta_d2 = math.sqrt(max(0.5 * (1.0 - junction_cos_theta), 0.0))
        cos_theta_d2 = math.sqrt(max(0.5 * (1.0 + junction_cos_theta), 0.0))
        if cos_theta_d2 > 0.0:
            tan_theta_d2 = sin_theta_d2 / cos_theta_d2
            move_centripetal_v2 = 0.5 * self.move_d * self.accel * tan_theta_d2
            pmove_centripetal_v2 = (
                0.5 * prev_move.move_d * prev_move.accel * tan_theta_d2
            )
            max_start_v2 = min(
                max_start_v2, move_centripetal_v2, pmove_centripetal_v2,
            )
        self.max_start_v2 = max_start_v2
```

The old `max_smoothed_v2` assignment is deleted.

- [ ] **Step 6: Rewrite `LookAheadQueue.flush`**

In `klippy/toolhead.py`, replace the body of `flush` (roughly lines 419-593). The shape-bake deferred-last pass is kept verbatim — it's the A3 contract, not a trapezoidal artifact. What changes is the reverse pass kinematics.

```python
    def flush(self, lazy=False):
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        update_flush_count = lazy
        queue = self.queue
        flush_count = len(queue)
        # Plan 9 A5 — jerk-native reverse pass. Propagate end_v^2 backwards;
        # at each move, clip cruise_v to the jerk-aware
        # max_reachable_cruise_v cap. No smoothed pass, no peak_cruise_v2
        # averaging, no delayed[] queue.
        #
        # QuinticBlendMove (from CornerBlender) carries a TOPP-baked
        # immutable profile — its set_junction is a no-op and its
        # max_start_v2 becomes the next_end_v2 for the move upstream.
        next_end_v2 = 0.0
        for i in range(flush_count - 1, -1, -1):
            move = queue[i]
            if not isinstance(move, Move):
                # QBM: immutable; just feed its v_in back as end_v^2.
                if update_flush_count and move.max_cruise_v2:
                    flush_count = i
                    update_flush_count = False
                next_end_v2 = move.max_start_v2
                continue
            # Clamp demanded end_v to max_cruise_v so jerk profile isn't
            # asked to cruise above cap.
            end_v2 = min(next_end_v2, move.max_cruise_v2)
            start_v2 = move.max_start_v2
            start_v = math.sqrt(start_v2) if start_v2 > 0.0 else 0.0
            end_v = math.sqrt(end_v2) if end_v2 > 0.0 else 0.0
            cruise_v_cap = math.sqrt(move.max_cruise_v2)
            cruise_v = jerk_math.max_reachable_cruise_v(
                v_start=start_v, v_end=end_v,
                a_max=move.accel, j_max=move.j_max,
                L=move.move_d, v_cruise_cap=cruise_v_cap,
            )
            cruise_v2 = cruise_v * cruise_v
            # Tighten the demanded end_v further if the chosen cruise_v
            # cannot sustain it.
            end_v2 = min(end_v2, cruise_v2)
            # Tighten the demanded start_v similarly so set_junction sees
            # a jerk-feasible (start, cruise, end, L) tuple.
            start_v2 = min(start_v2, cruise_v2)
            if update_flush_count and cruise_v2:
                flush_count = i
                update_flush_count = False
            if not update_flush_count and i < flush_count:
                move.set_junction(start_v2, cruise_v2, end_v2)
            # Update the upstream propagation. The backwards-reachability
            # bound for the move FEEDING this one's start_v is
            # reachable_v_end(end_v, a, j, L) — by time-reversal symmetry,
            # the largest v_start such that a jerk-limited accel group
            # ends at end_v in L. This is what Move.reachable_v_from_v_end
            # wraps — we call jerk_math directly here for clarity.
            reach = jerk_math.reachable_v_end(
                v_start=start_v, a_max=move.accel, j_max=move.j_max,
                L=move.move_d,
            )
            next_end_v2 = min(start_v2, reach * reach)

        # Plan 9 A3 — drain the pending-last move when the queue is empty
        # and the caller requested a full drain. Preserved verbatim from
        # pre-A5 code; trapezoidal cleanup only touched the reverse pass.
        if (not lazy) and self._pending_last is not None and not flush_count:
            pending_move, prev_unshaped, prev_start = self._pending_last
            self._finalize_with_neighbours(
                pending_move, prev_move=None, next_move=None,
                prev_override=(prev_unshaped, prev_start),
            )
            self.toolhead._process_moves([pending_move])
            self._pending_last = None
        if update_flush_count or not flush_count:
            return

        # Plan 9 A3 — deferred-last shape-bake pass. Unchanged from pre-A5.
        batch_to_emit = []
        if self._pending_last is not None:
            pending_move, prev_unshaped, prev_start = self._pending_last
            self._finalize_with_neighbours(
                pending_move, prev_move=None, next_move=queue[0],
                prev_override=(prev_unshaped, prev_start),
            )
            batch_to_emit.append(pending_move)
            self._pending_last = None
        for i in range(flush_count - 1):
            move = queue[i]
            prev_move = batch_to_emit[-1] if batch_to_emit else None
            self._finalize_with_neighbours(move, prev_move, queue[i + 1])
            batch_to_emit.append(move)
        last = queue[flush_count - 1]
        prev = batch_to_emit[-1] if batch_to_emit else None
        if lazy and _is_shape_bake_target(last):
            if prev is not None and _is_shape_bake_target(prev):
                prev_unshaped = prev._unshaped_payload
                prev_start = tuple(prev.start_pos[:3])
            else:
                prev_unshaped = None
                prev_start = None
            self._pending_last = (last, prev_unshaped, prev_start)
        else:
            self._finalize_with_neighbours(last, prev, next_move=None)
            batch_to_emit.append(last)
        if batch_to_emit:
            self.toolhead._process_moves(batch_to_emit)
        del queue[:flush_count]
```

- [ ] **Step 7: Run the new tests to confirm they pass**

Run: `pytest test/test_toolhead_jerk_integration.py -v -k "bed_mesh_crash or no_smoothed_fields or max_reachable_cruise_v"`
Expected: all three PASS.

- [ ] **Step 8: Run the entire jerk test suite to catch regressions**

Run: `pytest test/test_toolhead_jerk_wiring.py test/test_toolhead_jerk_integration.py -v`
Expected: all 20+ tests PASS. Some may fail because they reference the retired fields (`smooth_delta_v2`, `max_smoothed_v2`) — Task 5 will migrate test stubs. For this task it is acceptable if tests that only exercise the retired fields fail; do NOT add back-compat shims to make them pass. Record the failing test names in a scratch note for Task 5.

- [ ] **Step 9: Commit**

```bash
git add klippy/toolhead.py test/test_toolhead_jerk_integration.py
git commit -m "plan9-A5 T2: jerk-native reverse pass; delete smoothed pass"
```

---

### Task 3: QuinticBlendMove parity with the new contract

**Model:** opus

**Files:**
- Modify: `klippy/blendplanner.py` — `QuinticBlendMove.__init__` (lines ~388-389), `QuinticBlendMove.finalize_shape` (lines ~537-538), `QuinticBlendMove.limit_speed` (lines ~555-561), `QuinticBlendMove.calc_junction` (lines ~568-588), `_copy_caller_state` (lines ~314-338)
- Test: `test/test_blendplanner.py`, `test/test_blendprepass.py` — drop assertions on retired fields

QuinticBlendMove mirrors Move's attribute contract because LookAheadQueue treats them the same via the reverse pass. Under the new contract, `delta_v2`, `smooth_delta_v2`, `max_smoothed_v2` are gone. `QBM.calc_junction` ran the legacy forward cap — it now runs jerk-aware forward reachability.

- [ ] **Step 1: Write failing test for QBM parity**

Append to `test/test_blendplanner.py`:

```python
def test_quintic_blend_move_no_smoothed_fields():
    """A5 — QuinticBlendMove must not carry smoothed-pass state."""
    from klippy.blendplanner import QuinticBlendMove
    # Minimal construction — we just need the attribute surface.
    # Detailed construction is out of scope; if QBM is only
    # constructible via CornerBlender, skip this test.
    th = _FakeToolhead()
    shape_stub = _make_min_shape_stub()  # existing test helper
    qbm = QuinticBlendMove(
        toolhead=th, shape=shape_stub,
        start_pos_4d=(0, 0, 0, 0), end_pos_4d=(1, 0, 0, 0),
        v_in=100.0, v_out=100.0, cruise_v=200.0,
        s_accel_end=0.4, s_decel_start=0.6,
        a_max=5000.0, v_cap_min=200.0,
    )
    assert not hasattr(qbm, "smooth_delta_v2")
    assert not hasattr(qbm, "max_smoothed_v2")
    assert not hasattr(qbm, "delta_v2")
```

If the existing test-support code has no `_make_min_shape_stub`, use a skip marker and rely on the integration test in Task 6. Check the existing helpers in `test/test_blendplanner.py` first.

- [ ] **Step 2: Run the new test**

Run: `pytest test/test_blendplanner.py -v -k no_smoothed_fields`
Expected: FAIL — the attributes still exist.

- [ ] **Step 3: Edit `QuinticBlendMove.__init__`**

In `klippy/blendplanner.py`, delete line 389:

```python
        self.max_smoothed_v2 = v_in * v_in
```

- [ ] **Step 4: Edit `QuinticBlendMove.finalize_shape`**

In `klippy/blendplanner.py`, delete lines 537-538:

```python
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = self.delta_v2
```

- [ ] **Step 5: Edit `QuinticBlendMove.limit_speed`**

In `klippy/blendplanner.py`, replace with the slim version (delete the `delta_v2` / `smooth_delta_v2` tail):

```python
    def limit_speed(self, speed, accel):
        v2 = speed * speed
        if v2 < self.max_cruise_v2:
            self.max_cruise_v2 = v2
        self.accel = min(self.accel, accel)
```

- [ ] **Step 6: Edit `QuinticBlendMove.calc_junction`**

In `klippy/blendplanner.py`, replace with the jerk-aware forward cap variant. Note that QBM doesn't have `j_max` directly — it inherits the toolhead's via `self.toolhead.max_jerk`.

```python
    def calc_junction(self, prev_move):
        # Plan 9 A5: same treatment as Move.calc_junction — jerk-aware
        # forward reachability replaces delta_v2; smoothed pass is gone.
        # The blend's v_in is pointwise-safe via TOPP + v_cap_fn (D7
        # Option Z); we still run the upstream cascade so the reverse
        # pass can tighten max_start_v2 via prev_move caps.
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        prev_start_v = (math.sqrt(prev_move.max_start_v2)
                        if prev_move.max_start_v2 > 0.0 else 0.0)
        prev_j_max = getattr(prev_move, "j_max", self.toolhead.max_jerk)
        prev_forward_reach = jerk_math.reachable_v_end(
            v_start=prev_start_v,
            a_max=prev_move.accel, j_max=prev_j_max,
            L=prev_move.move_d,
        )
        max_start_v2 = min(
            self.max_start_v2,
            self.max_cruise_v2,
            prev_move.max_cruise_v2,
            prev_move.next_junction_v2,
            prev_forward_reach * prev_forward_reach,
        )
        self.max_start_v2 = max_start_v2
```

Add `from . import jerk_math` at the top of `blendplanner.py` if not present.

- [ ] **Step 7: Edit `_copy_caller_state`**

In `klippy/blendplanner.py` (lines 314-338), delete the `delta_v2` / `smooth_delta_v2` preservation logic:

```python
def _copy_caller_state(src, dst):
    """Transfer caller-mutable Move state from src to the truncated dst.

    Pins caller-intent fields verbatim (timing_callbacks, next_junction_v2,
    max_cruise_v2, accel) so that M204 / SET_VELOCITY_LIMIT
    / register_lookahead_callback mutations applied upstream to src survive
    the emit-time construction of dst.

    The accel pin is a direct assignment (not via dst.limit_speed) because
    limit_speed takes min(self.accel, accel); if an intervening M204 had
    lowered toolhead.max_accel between src construction and emit,
    Move.__init__'s snapshot of the new (lower) value would win over
    src.accel. min_move_t is recomputed from dst's move_d + max_cruise_v2.
    """
    dst.timing_callbacks = list(src.timing_callbacks)
    dst.next_junction_v2 = src.next_junction_v2
    dst.max_cruise_v2 = src.max_cruise_v2
    dst.accel = src.accel
    dst.min_move_t = dst.move_d / math.sqrt(dst.max_cruise_v2)
```

- [ ] **Step 8: Run the QBM test**

Run: `pytest test/test_blendplanner.py -v -k no_smoothed_fields`
Expected: PASS.

- [ ] **Step 9: Run the broader blendplanner test suite**

Run: `pytest test/test_blendplanner.py test/test_blendprepass.py -v`
Expected: some failures in tests that reference retired fields — these will be fixed in Task 5. Do NOT add back-compat attributes.

- [ ] **Step 10: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "plan9-A5 T3: QuinticBlendMove parity with jerk-native contract"
```

---

### Task 4: Retire `max_accel_to_decel` / `minimum_cruise_ratio` config

**Model:** sonnet

**Files:**
- Modify: `klippy/toolhead.py` — `ToolHead.__init__` (lines ~644-676), `ToolHead.max_accel_to_decel` property (lines ~1267-1271), `cmd_SET_VELOCITY_LIMIT` (lines ~1284-1381), `cmd_RESET_VELOCITY_LIMIT`, `get_status` (lines ~1207-1225).
- Modify: `klippy/extras/trad_rack.py` — the single `max_accel_to_decel` deprecate call (lines ~2351-2358).
- Test: `test/test_toolhead_jerk_integration.py` — ensure config parsing still works without the retired keys.

`minimum_cruise_ratio` is the Kalico-era user surface for `max_accel_to_decel`. Under jerk motion, the cruise-fraction knob has no meaningful physical counterpart — `max_accel` and `max_jerk` determine the profile shape entirely. Delete both.

- [ ] **Step 1: Write the failing test**

Append to `test/test_toolhead_jerk_integration.py`:

```python
def test_toolhead_has_no_max_accel_to_decel(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/toolhead_jerk"
    ],
):
    """A5: max_accel_to_decel is retired. The ToolHead must not expose
    it as a property, and the config deprecation path must be gone."""
    start_args = {"config_file": str(config_root / "printer.cfg")}
    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        from klippy.toolhead import ToolHead
        # The property must be gone.
        assert not hasattr(ToolHead, "max_accel_to_decel")


def test_toolhead_has_no_min_cruise_ratio():
    """A5: minimum_cruise_ratio is retired."""
    from klippy.toolhead import ToolHead
    # No class-level descriptor; instance attribute must not be set
    # by __init__. Probe via a minimal toolhead construction.
    th = _FakeToolhead()
    assert not hasattr(th, "min_cruise_ratio"), (
        "_FakeToolhead is wrong if it sets min_cruise_ratio; real "
        "ToolHead must not set it either"
    )
```

- [ ] **Step 2: Run the failing tests**

Run: `pytest test/test_toolhead_jerk_integration.py -v -k "no_max_accel_to_decel or no_min_cruise_ratio"`
Expected: FAIL — both attributes still exist.

- [ ] **Step 3: Edit `ToolHead.__init__`**

In `klippy/toolhead.py` around lines 644-676, replace the `min_cruise_ratio` / `max_accel_to_decel` block with just the core velocity/accel/jerk configuration:

```python
        # Velocity and acceleration control
        self.max_velocity = config.getfloat("max_velocity", above=0.0)
        self.max_accel = config.getfloat("max_accel", above=0.0)
        self.max_jerk = config.getfloat("max_jerk", 100000.0, above=0.0)
        self.corner_deviation = config.getfloat("corner_deviation", above=0.0)
        # Plan 9 A5: square_corner_velocity, minimum_cruise_ratio, and
        # max_accel_to_decel are all retired. Under jerk-limited motion the
        # profile shape is determined by (max_accel, max_jerk); there is no
        # "cruise fraction" knob. square_corner_velocity is still parsed
        # below as a deprecation warning path inherited from the arc-blending
        # cut-over (pre-A5) and is preserved so users carrying stale configs
        # still get a helpful message rather than a config.error.
        scv_legacy = config.getfloat(
            "square_corner_velocity", None, minval=0.0
        )
        if scv_legacy is not None:
            config.deprecate("square_corner_velocity")
            logging.warning(
                "config option [printer] square_corner_velocity is obsolete; "
                "the jerk-limited planner ignores it. Remove it from your "
                "config to silence this warning."
            )
        # Similar hard-fail messages for the retired A5 knobs: if a user
        # carries these from a pre-A5 config, refuse to start with a
        # direct message rather than silently ignoring them.
        for retired in ("max_accel_to_decel", "minimum_cruise_ratio"):
            if config.get(retired, None) is not None:
                raise config.error(
                    "config option [printer] %s is retired in Plan 9 A5; "
                    "jerk-limited motion has no cruise-fraction knob. "
                    "Remove the option. Tune max_accel and max_jerk instead."
                    % retired
                )
        self.orig_cfg = {}
        self.orig_cfg["max_velocity"] = self.max_velocity
        self.orig_cfg["max_accel"] = self.max_accel
        self.orig_cfg["max_jerk"] = self.max_jerk
        self.orig_cfg["corner_deviation"] = self.corner_deviation
```

- [ ] **Step 4: Delete the `max_accel_to_decel` property**

In `klippy/toolhead.py` lines ~1267-1271, delete:

```python
    @property
    def max_accel_to_decel(self):
        # Derived live from min_cruise_ratio rather than cached, so M204 /
        # SET_VELOCITY_LIMIT mutations are visible without an explicit recompute.
        return self.max_accel * (1.0 - self.min_cruise_ratio)
```

- [ ] **Step 5: Edit `cmd_SET_VELOCITY_LIMIT`**

In `klippy/toolhead.py` around lines 1284-1381, delete all mentions of `min_cruise_ratio`, `MINIMUM_CRUISE_RATIO`, `ACCEL_TO_DECEL`, `req_accel_to_decel`. Keep `VELOCITY`, `ACCEL`, `JERK`, `CORNER_DEVIATION`. The `SQUARE_CORNER_VELOCITY` parse-and-ignore path stays (same rationale as the config version).

Replace the command body with:

```python
    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        max_velocity = gcmd.get_float("VELOCITY", None, above=0.0)
        max_accel = gcmd.get_float("ACCEL", None, above=0.0)
        max_jerk = gcmd.get_float("JERK", None, above=0.0)
        # Parsed but discarded; the jerk-limited planner ignores SCV.
        square_corner_velocity = gcmd.get_float(
            "SQUARE_CORNER_VELOCITY", None, minval=0.0
        )
        # Plan 9 A5: MINIMUM_CRUISE_RATIO and ACCEL_TO_DECEL are retired.
        # Reject them loudly so users notice rather than silently losing
        # their tuning.
        for retired in ("MINIMUM_CRUISE_RATIO", "ACCEL_TO_DECEL"):
            if gcmd.get_float(retired, None) is not None:
                raise gcmd.error(
                    "%s is retired in Plan 9 A5; tune ACCEL and JERK instead."
                    % retired
                )
        corner_deviation = gcmd.get_float(
            "CORNER_DEVIATION", None, above=0.0
        )
        if max_velocity is not None:
            self.max_velocity = max_velocity
        if max_accel is not None:
            self.max_accel = max_accel
        if max_jerk is not None:
            self.max_jerk = max_jerk
        if corner_deviation is not None:
            self.corner_deviation = corner_deviation
        msg = [
            "max_velocity: %.6f" % self.max_velocity,
            "max_accel: %.6f" % self.max_accel,
            "max_jerk: %.6f" % self.max_jerk,
        ]
        if hasattr(self.kin, "max_x_velocity"):
            max_x_velocity = gcmd.get_float("X_VELOCITY", None)
            if max_x_velocity is not None:
                self.kin.max_x_velocity = max_x_velocity
            msg.append("max_x_velocity: %.6f" % self.kin.max_x_velocity)
        if hasattr(self.kin, "max_x_accel"):
            max_x_accel = gcmd.get_float("X_ACCEL", None)
            if max_x_accel is not None:
                self.kin.max_x_accel = max_x_accel
            msg.append("max_x_accel: %.6f" % self.kin.max_x_accel)
        if hasattr(self.kin, "max_y_velocity"):
            max_y_velocity = gcmd.get_float("Y_VELOCITY", None)
            if max_y_velocity is not None:
                self.kin.max_y_velocity = max_y_velocity
            msg.append("max_y_velocity: %.6f" % self.kin.max_y_velocity)
        if hasattr(self.kin, "max_y_accel"):
            max_y_accel = gcmd.get_float("Y_ACCEL", None)
            if max_y_accel is not None:
                self.kin.max_y_accel = max_y_accel
            msg.append("max_y_accel: %.6f" % self.kin.max_y_accel)
        if hasattr(self.kin, "max_z_velocity"):
            max_z_velocity = gcmd.get_float("Z_VELOCITY", None, above=0.0)
            if max_z_velocity is not None:
                self.kin.max_z_velocity = max_z_velocity
            msg.append("max_z_velocity: %.6f" % self.kin.max_z_velocity)
        if hasattr(self.kin, "max_z_accel"):
            max_z_accel = gcmd.get_float("Z_ACCEL", None, above=0.0)
            if max_z_accel is not None:
                self.kin.max_z_accel = max_z_accel
            msg.append("max_z_accel: %.6f" % self.kin.max_z_accel)
        msg.append("corner_deviation: %.6f" % self.corner_deviation)
        if get_danger_options().log_velocity_limit_changes:
            self.printer.set_rollover_info(
                "toolhead", "toolhead: %s" % (" ".join(msg),)
            )
            if (max_velocity is None and max_accel is None
                    and max_jerk is None and square_corner_velocity is None
                    and corner_deviation is None):
                gcmd.respond_info("\n".join(msg), log=False)
```

- [ ] **Step 6: Edit `cmd_RESET_VELOCITY_LIMIT` and `get_status`**

In `get_status` (lines ~1207-1225), delete the `minimum_cruise_ratio` entry from the returned dict.

In `cmd_RESET_VELOCITY_LIMIT`, remove any `min_cruise_ratio` reset. The body should reset only `max_velocity`, `max_accel`, `max_jerk`, `corner_deviation`, and per-axis kin limits — drop everything else that references the retired knobs.

- [ ] **Step 7: Edit `klippy/extras/trad_rack.py`**

In `klippy/extras/trad_rack.py` around lines 2351-2358, delete the `max_accel_to_decel` parse/deprecate branch. `trad_rack` carries its own local planner instance of these knobs; if the removal triggers further cleanup that's out of A5 scope — just remove the deprecate call and let the code compile.

Before editing, check what trad_rack actually does with the parsed value. If it stores it for its own use rather than delegating to the toolhead, leave the parse but replace `config.deprecate` with a `config.error` that matches the toolhead's rejection message. This is almost certainly a leftover from when trad_rack mirrored the toolhead's config surface and the ratio feeds nothing in its own planner.

- [ ] **Step 8: Run the tests**

Run: `pytest test/test_toolhead_jerk_integration.py -v -k "no_max_accel_to_decel or no_min_cruise_ratio"`
Expected: PASS.

Run: `pytest test/ -v --tb=short 2>&1 | head -80` to catch collateral damage.
Expected: tests that still reference retired fields will fail — Task 5 migrates them.

- [ ] **Step 9: Commit**

```bash
git add klippy/toolhead.py klippy/extras/trad_rack.py test/test_toolhead_jerk_integration.py
git commit -m "plan9-A5 T4: retire max_accel_to_decel and minimum_cruise_ratio"
```

---

### Task 5: Test migration — drop stubs for retired fields

**Model:** sonnet

**Files:**
- Modify: `test/test_toolhead_jerk_wiring.py` — `_FakeToolhead`, `_StubToolhead`
- Modify: `test/test_toolhead_jerk_integration.py` — `_FakeToolhead`, `_StubToolhead` variants
- Modify: `test/test_blendplanner.py`, `test/test_blendprepass.py`, `test/test_plan5_integration.py`, `test/test_chunk3_pa_integration.py`, `test/test_toolhead_shape_bake.py`, `test/test_toolhead_shape_bake_pipeline.py` — drop `max_accel_to_decel` / `smooth_delta_v2` / `max_smoothed_v2` lines from stubs and assertions

Mechanical edit — `grep -rn "max_accel_to_decel\|smooth_delta_v2\|max_smoothed_v2" test/` and delete every occurrence unless it's part of a deliberate test that A5 is removing this stuff.

- [ ] **Step 1: Enumerate current offending lines**

Run: `grep -rn "max_accel_to_decel\|smooth_delta_v2\|max_smoothed_v2" /Users/daniladergachev/Developer/kalico/test/`
Expected: the list from the "key context" section — ~25 lines across 8 files.

- [ ] **Step 2: For each file, delete the offending lines + any now-dead assertions that referenced them**

Worked example for `test/test_toolhead_jerk_wiring.py`:

```python
# Delete this line in _FakeToolhead.__init__:
-        self.max_accel_to_decel = kw.get("max_accel_to_decel", 5000.0)
```

Worked example for `test/test_blendprepass.py` around line 645:

```python
# Delete the smoothed-pass assertion — A5 has no smoothed pass.
-    m1.max_smoothed_v2 = 6789.0
...
-    assert merged.max_smoothed_v2 == 0.0
```

If a test case's *purpose* was to verify smoothed-pass behaviour (e.g. test names like `test_smoothed_cap_obeyed_...`), delete the entire test. Do not leave vestigial assertions like `assert True` or bodies that no longer exercise anything.

For `test/test_plan5_integration.py`, `test/test_chunk3_pa_integration.py`, `test/test_blendprepass.py`, `test/test_blendplanner.py`: the fake Move implementations in those test modules populate `max_smoothed_v2` / `smooth_delta_v2` directly. Delete those lines from the fake `Move.__init__` bodies.

- [ ] **Step 3: Run the full test suite**

Run: `pytest test/ --tb=short 2>&1 | tail -40`
Expected: all tests PASS. If any fail due to missing attribute or missing test stub, trace and fix in the same pass.

- [ ] **Step 4: Commit**

```bash
git add test/
git commit -m "plan9-A5 T5: migrate test stubs off retired smoothed/max_accel_to_decel"
```

---

### Task 6: bed_mesh regression test + end-to-end dogfood

**Model:** opus

**Files:**
- Test: `test/test_bed_mesh_regression.py` — NEW file; black-box replay of the original crash

This test is not a unit test of any one function — it drives a full real ToolHead + LookAheadQueue + Move pipeline through the exact numeric inputs that crashed on Trident during `bed_mesh_calibrate`. If A5 is correct, it finishes without raising. If any of Tasks 1-4 regressed (e.g. trapezoidal cap leaked back via a subclass, or `max_reachable_cruise_v` returns an infeasible value in some regime), this test catches it.

- [ ] **Step 1: Write the regression test**

Create `test/test_bed_mesh_regression.py`:

```python
# test/test_bed_mesh_regression.py
"""Plan 9 A5 — bed_mesh crash regression.

Replays the exact numeric inputs that crashed on Trident:
  start_v=374.7, cruise_v=469.8, end_v=469.8, move_d=1.143 mm,
  accel=70000, j_max=500000.

Under the trapezoidal cruise cap this passed the reverse pass and was
rejected by jerk_profile.compute_profile in set_junction. Under A5 the
reverse pass clips cruise_v via max_reachable_cruise_v and no crash
occurs.
"""
from __future__ import annotations

import math
import pytest


class _StubToolhead:
    """Minimal toolhead surface matching the A5 contract.

    Plan 9 A5 intentionally does NOT expose max_accel_to_decel,
    min_cruise_ratio, or any smoothed-pass attributes. This stub is a
    regression guard — a new contributor who re-adds the retired fields
    to a stub will see construction succeed but downstream lookahead
    still works; the bed_mesh tuple still passes.
    """

    def __init__(self, **kw):
        self.max_velocity = kw.get("max_velocity", 600.0)
        self.max_accel = kw.get("max_accel", 70000.0)
        self.max_jerk = kw.get("max_jerk", 500000.0)
        self.extruder_cap_snapshot = None
        self.shapers_snapshot = []
        class _K:
            def check_move(self, m): pass
        class _E:
            def check_move(self, m): pass
            def calc_junction(self, *_a): return 1e18
        self.kin = _K()
        self.extruder = _E()
        self.captured = []

    def _process_moves(self, moves):
        self.captured.extend(moves)


def _build_queue(th):
    from klippy.toolhead import LookAheadQueue
    return LookAheadQueue(th)


def test_bed_mesh_crash_tuple_replayed_through_full_lookahead():
    """Original crash pattern. Must not raise."""
    from klippy.toolhead import Move
    th = _StubToolhead()
    la = _build_queue(th)
    # A pre-probe cruise move, the 1.143 mm probe hop, and a following
    # move that holds the speed. All collinear so calc_junction gives
    # cos_theta=1 (no centripetal cap).
    m_a = Move(th, (0, 0, 0, 0), (40, 0, 0, 0), speed=469.8)
    m_b = Move(th, (40, 0, 0, 0), (41.143, 0, 0, 0), speed=469.8)
    m_c = Move(th, (41.143, 0, 0, 0), (80, 0, 0, 0), speed=469.8)
    la.queue.extend([m_a, m_b, m_c])
    m_b.calc_junction(m_a)
    m_c.calc_junction(m_b)
    # The critical call — today this raises "Jerk profile infeasible".
    # Post-A5, it must complete without error.
    la.flush(lazy=False)
    for m in (m_a, m_b, m_c):
        assert hasattr(m, "jerk_profile"), (
            "%r did not receive a jerk_profile — reverse pass bailed" % m
        )
        assert m.jerk_profile.status == 0, (
            "%r jerk profile status = %d (expected JP_OK=0)"
            % (m, m.jerk_profile.status)
        )


def test_bed_mesh_crash_tuple_returns_feasible_cruise_v():
    """A5 clip: m_b's cruise_v must be < 469.8 because a 1.143 mm hop
    at j=500k / a=70k cannot sustain a 374.7 -> 469.8 ramp there.

    We verify via the reverse-pass contract directly: after flush,
    m_b.cruise_v matches max_reachable_cruise_v from the actual
    upstream-tightened start_v.
    """
    from klippy.toolhead import Move
    from klippy import jerk_math
    th = _StubToolhead()
    la = _build_queue(th)
    m_a = Move(th, (0, 0, 0, 0), (40, 0, 0, 0), speed=469.8)
    m_b = Move(th, (40, 0, 0, 0), (41.143, 0, 0, 0), speed=469.8)
    m_c = Move(th, (41.143, 0, 0, 0), (80, 0, 0, 0), speed=469.8)
    la.queue.extend([m_a, m_b, m_c])
    m_b.calc_junction(m_a)
    m_c.calc_junction(m_b)
    la.flush(lazy=False)
    # Sanity: start_v must not exceed end_v's reach, cruise_v must be
    # feasible given both endpoints.
    assert m_b.cruise_v < 469.8 or (
        m_b.start_v < 374.7
    ), (
        "A5 must either tighten start_v or cap cruise_v on infeasible "
        "short hop; got start_v=%.3f cruise_v=%.3f end_v=%.3f move_d=%.6f"
        % (m_b.start_v, m_b.cruise_v, m_b.end_v, m_b.move_d)
    )
    # max_reachable_cruise_v must agree with what the reverse pass chose.
    expected = jerk_math.max_reachable_cruise_v(
        v_start=m_b.start_v, v_end=m_b.end_v,
        a_max=m_b.accel, j_max=m_b.j_max,
        L=m_b.move_d, v_cruise_cap=469.8,
    )
    assert m_b.cruise_v == pytest.approx(expected, rel=1e-6)


def test_bed_mesh_crash_pattern_batched_flush_matches_drain():
    """Same tuple, flushed lazily then drained. Both paths must succeed
    and agree on the chosen cruise_v — the lazy flush's shape-bake
    deferral must not leak trapezoidal assumptions back in."""
    from klippy.toolhead import Move
    th1 = _StubToolhead()
    th2 = _StubToolhead()
    for th, lazy in ((th1, True), (th2, False)):
        la = _build_queue(th)
        m_a = Move(th, (0, 0, 0, 0), (40, 0, 0, 0), speed=469.8)
        m_b = Move(th, (40, 0, 0, 0), (41.143, 0, 0, 0), speed=469.8)
        m_c = Move(th, (41.143, 0, 0, 0), (80, 0, 0, 0), speed=469.8)
        la.queue.extend([m_a, m_b, m_c])
        m_b.calc_junction(m_a)
        m_c.calc_junction(m_b)
        la.flush(lazy=lazy)
        if lazy:
            la.flush(lazy=False)  # drain
        # Both must have succeeded; the chosen cruise_v may differ
        # negligibly due to the lazy flush's floating-point path but
        # must be within 1 ppm of each other.
    v1 = m_b.cruise_v  # last iter; fine — drain path
    # Compare to the drain-only run.
    la2 = _build_queue(th2)
    m_a2 = Move(th2, (0, 0, 0, 0), (40, 0, 0, 0), speed=469.8)
    m_b2 = Move(th2, (40, 0, 0, 0), (41.143, 0, 0, 0), speed=469.8)
    m_c2 = Move(th2, (41.143, 0, 0, 0), (80, 0, 0, 0), speed=469.8)
    la2.queue.extend([m_a2, m_b2, m_c2])
    m_b2.calc_junction(m_a2)
    m_c2.calc_junction(m_b2)
    la2.flush(lazy=False)
    assert m_b.cruise_v == pytest.approx(m_b2.cruise_v, rel=1e-9)
```

- [ ] **Step 2: Run the regression test against today's (pre-A5) code**

Before Tasks 1-5 land, this test SHOULD fail with `command_error: Jerk profile infeasible`. Confirm this locally by temporarily checking out pre-A5 HEAD or by running the first test on a branch stash. This is a sanity check that the test captures the right failure mode — skip this step if Tasks 1-5 are already merged.

- [ ] **Step 3: Run the regression test against post-A5 code**

Run: `pytest test/test_bed_mesh_regression.py -v`
Expected: all three tests PASS.

- [ ] **Step 4: Run the full A5 test suite end-to-end**

Run: `pytest test/test_jerk_math.py test/test_toolhead_jerk_wiring.py test/test_toolhead_jerk_integration.py test/test_blendplanner.py test/test_blendprepass.py test/test_toolhead_shape_bake.py test/test_toolhead_shape_bake_pipeline.py test/test_bed_mesh_regression.py -v`
Expected: all PASS.

- [ ] **Step 5: Run the complete test suite to catch any other regressions**

Run: `pytest test/ --tb=short 2>&1 | tail -50`
Expected: all PASS or any failures are unrelated pre-existing flakiness that also fails on `main`. Document any outliers in the commit.

- [ ] **Step 6: Commit**

```bash
git add test/test_bed_mesh_regression.py
git commit -m "plan9-A5 T6: bed_mesh crash regression test"
```

---

## Self-review

**Spec coverage:**
- Reverse-pass cruise cap replacement — Task 1 + Task 2.
- Smoothed pass removal — Task 2.
- `max_accel_to_decel` / `min_cruise_ratio` retirement — Task 4.
- `delta_v2` retirement / jerk-aware forward cap — Task 2 (Move) + Task 3 (QBM).
- `smooth_delta_v2` / `max_smoothed_v2` excision — Task 2 (Move) + Task 3 (QBM) + Task 5 (tests).
- Pure-E path — unchanged (the reverse pass's new form works on any Move with `j_max` and `accel`; pure-E moves have both). Explicitly noted in Task 2's code comment.
- bed_mesh bug closure — Task 6.
- Centripetal cap under jerk — explicitly kept in A2c form (geometric, not trapezoidal); no changes needed.
- QuinticBlendMove parity — Task 3.
- Test migration strategy — Task 5 + Task 6.
- Public API preservation — `toolhead.move` / `flush_step_generation` / `drip_move` are untouched. The only visible surface change is the deletion of `max_accel_to_decel` config/gcode keys, which is called out in Task 4 as a hard-fail-on-config.

**Placeholder scan:** no "TBD" / "fill in later" / "similar to Task N" / bare-assertion-without-body remain. Every code block is concrete. Step titles describe what and each step carries the code or command.

**Type consistency:** `max_reachable_cruise_v` signature is fixed in Task 1 and consumed identically in Task 2, Task 3, and Task 6. `jerk_math.reachable_v_end` signature unchanged. The 9-tuple `quintic_trapq_payload` contract is untouched.

**Bite-sized check:** every task has 5-10 steps; each step is writable/verifiable in 2-5 min of subagent work. Tasks 2 and 3 have larger code blocks because replacing a ~180-line function is one atomic edit.

**Bed_mesh bug closed?** Yes. The primary acceptance criterion is Task 6's `test_bed_mesh_crash_tuple_replayed_through_full_lookahead` — it runs the exact tuple through the real `Move` + `LookAheadQueue` machinery. Under Task 2's new reverse pass, `max_reachable_cruise_v` returns a `cruise_v < 469.8` for the (374.7, 469.8, 1.143, 70k, 500k) inputs, so `set_junction` receives a feasible tuple and `jerk_profile.compute_profile` returns `JP_OK`.

**Open questions deferred to execution:**
- Whether `trad_rack.py`'s `max_accel_to_decel` parse serves any local purpose beyond mirroring the toolhead (Task 4 Step 7 asks the implementing subagent to check). If it does, the cleanup scope grows beyond A5; split that into A5-followup.
- Whether the bisection-count (60 iters) in `max_reachable_cruise_v` is excessive — 30 is probably enough for `1e-4` mm precision. Leave at 60 for safety; optimize only if flame-graph shows it.
- Centripetal cap under jerk motion: Task context note claims the geometric form `0.5 * L * a * tan(θ/2)` is still correct. A research subagent could verify by comparing to a derivation where `a` is replaced by the jerk-limited mean acceleration on the blend arc. If the geometric form is NOT correct under jerk, this spawns a follow-up phase — it does NOT break A5's bed_mesh-bug closure.
