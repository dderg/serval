# Plan 2 — Smooth-shapers merge + HP-stepcompress port

**Status:** Phase A landed 2026-04-21 (commit `496365b2`). **Phase B deferred** — HP-stepcompress cherry-pick conflicts with `f26c79c7` (`stepper: ensure minimum time between step and dir pin changes`, Kalico `v2026.04.00`), which already exists on magnum-opus. No upstream branch integrates both changes; resolution requires manual reconciliation of two `stepper_load_next()` refactors and is out of scope for this plan. Revisit HP-stepcompress in a dedicated follow-up plan.
**Branch target:** `magnum-opus`
**Predecessor:** Plan 1 (quintic revival + shape-pluggable primitive) — merged
**Successor (planned):** Plan 3 — non-linear PA integration

## Goal

Bring two upstream-derived improvements into `magnum-opus`:

1. Merge the sibling `smooth-shapers` branch (polynomial smooth shapers, non-linear PA, extruder-IS sync, recent shaper-calibration work).
2. Cherry-pick HP-stepcompress from `upstream/bleeding-edge-v2` (2nd-order step-timing encoding; shrinks MCU message count, tightens `min_move_time` ceiling).

## Why now

- The `smooth-shapers` branch has been HW-validated by the user on the sibling stack; prints are clean. Keeping it out of magnum-opus forces every later plan (3/4/5/6) to also carry the merge burden. Merging once, early, gives the remaining plans a clean baseline.
- HP-stepcompress is independent of the motion-planner layer but amplifies it: smoother trajectories (quintic corners, eventual inverse-shaper-precompensated paths) are exactly what HP's 2nd-order encoding compresses well. Landing it before plans 3/4/5/6 means every later HW validation can quantify HP's contribution.
- Both integrations are upstream-sourced; neither is a design problem. Plan 2 is a **landing plan**, not an algorithms plan.

## Architecture

Two phases, in order, both on the `magnum-opus` branch:

**Phase A — Smooth-shapers merge** (local branch → local branch).
**Phase B — HP-stepcompress cherry-picks** (upstream/bleeding-edge-v2 → magnum-opus).

Phase A lands first because Phase B only touches the chelper + stepcompress + stepper.py layers — orthogonal to the input-shaper/extruder layer Phase A lands. If Phase A gets tangled (unlikely but possible), Phase B can still proceed on its own.

---

## Phase A — Smooth-shapers merge

### What's coming in

The `smooth-shapers` branch is ~30 commits ahead of `magnum-opus`. Contents:

- **Smooth (polynomial) input shapers** (upstream bleeding-edge-v2 port): `smooth_zv`, `smooth_mzv`, `smooth_ei`, `smooth_2hump_ei`, `smooth_zvd_ei`, `smooth_si`.
- **Impulse shaper pruning**: kept `zv` and `mzv`; removed `zvd`, `ei`, `2hump_ei`, `3hump_ei` (replaced by smooth variants).
- **Non-linear Pressure Advance** (upstream): extruder PA moves from linear smear to non-linear; prereq for Plan 3.
- **Extruder sync with input shaping** (upstream): extruder position coupled to the shaper filter.
- **Extruder-specific smoothers** (upstream).
- **Shaper-calibrate improvements**: family-aware `target_smoothing` cap; raised `MAX_FREQ` 275→300 Hz; raised `MAX_SHAPER_FREQ` 215→250 Hz; vibration scoring revert to peak-ratio threshold; drop sub-sweep bins from vibration scoring.
- **Local bug fix `f1ec651d`**: `_extract_shapers` uses `get_axis()` method (was `.axis` attribute) and tolerates smooth-family shaper params (no `shaper_freq`, no `damping_ratio`). Fixed a TEST_RESONANCES crash.
- **Local bug fix `04943583`**: `suppressed_junction_v` — SCV-equivalent velocity cap applied at corners where the blender suppressed the blend, derived from shaper σ_T. Fixed observed skipped steps on Trident at 50k.

### Merge conflicts

Four files conflict — all in blend-arc code that Plan 1 on magnum-opus rewrote:

| File | Magnum-opus state | Smooth-shapers changes |
|---|---|---|
| `klippy/blendmath.py` | Arc primitives deleted; only vector ops + `_sigma_T_max_from_toolhead` + `_extract_shapers` + `interpolate_extruder` remain | `f1ec651d` rewrote `_extract_shapers`; `04943583` added `_scv_equivalent_junction_v` + `suppressed_junction_v` to arc math (which magnum-opus deleted) |
| `klippy/blendplanner.py` | Rewired to `QuinticShape.from_moves`; `_emit_arc` → `_emit_blend` | `04943583` wired `suppressed_junction_v` into the old `blend_from_moves`-returns-None path |
| `test/test_blendmath.py` | Arc tests removed; quintic tests present | `f1ec651d` added test-fakes updates + real-shaper regression tests; `04943583` added suppressed-junction-v regression tests |
| `test/test_blendplanner.py` | Rewired to use QuinticShape fakes | Mirror updates |

### Conflict resolution strategy

**Take magnum-opus's side on arc deletions.** Plan 1 deleted the arc primitive intentionally. The 04943583 commit put `_scv_equivalent_junction_v` and `suppressed_junction_v` inside the deleted arc code, but those helpers are **pure move-vector + shaper-σ_T math** — shape-agnostic — so they belong in the surviving `blendmath.py`, not in deleted arc code.

**Re-apply `f1ec651d`'s substance** to magnum-opus's `blendmath._extract_shapers`:

- Route axis lookup through `axis_shaper.get_axis()` (was direct `.axis` attribute).
- Use `getattr(shaper, 'shaper_freq', 0.0)` / `getattr(shaper, 'damping_ratio', 0.0)` so smooth-family shapers don't crash.
- For smooth-family axes, record `A_axis=0.0` (the legacy impulse-σ_T velocity cap doesn't consume polynomial shapers).
- Update test fakes in `test_blendmath.py` + `test_blendplanner.py` to expose `get_axis()` + `get_type()` methods (mirroring real `AxisInputShaper` / `AxisInputSmoother`) instead of raw attributes.
- Port the two real-shaper regression tests from `smooth-shapers:test/test_blendmath.py` (the `TypedInputShaperParams` / `TypedInputSmootherParams` end-to-end tests).

**Port `_scv_equivalent_junction_v` + `suppressed_junction_v`** from `git show 04943583:klippy/blendmath.py` (roughly lines 300–377 in the old file layout). Both are pure helpers with no blend-shape dependency; transfer verbatim into magnum-opus's `blendmath.py`. Port the matching tests from `smooth-shapers:test/test_blendmath.py` and `smooth-shapers:test/test_blendplanner.py`.

**Wire `suppressed_junction_v`** into `klippy/blendplanner.py:76-91` inside the existing `if shape is None:` branch:

```python
if shape is None:
    dp = sum(self._prev.axes_r[i] * move.axes_r[i] for i in range(3))
    if dp <= -0.5:
        # Near-reversal: force full stop.
        self._prev.limit_next_junction_speed(0.0)
    else:
        # Blend suppressed at a real corner: apply SCV-equivalent cap
        # derived from shaper σ_T so we don't hit the corner at full cruise.
        v_j = blendmath.suppressed_junction_v(
            self._prev, move, th.corner_deviation, th
        )
        if v_j is not None and math.isfinite(v_j):
            self._prev.limit_next_junction_speed(v_j)
    emitted = [self._prev]
    self._prev = move
    return emitted
```

### Interaction check — smooth shapers × quintic v_cap

On blend-arc, the runtime velocity cap consumed **impulse** shaper σ_T to derive per-axis `A_axis`. On magnum-opus, `blendquintic.QuinticShape.v_cap_fn(s)` reads the same per-axis budget via `blendshape.KinematicLimits.shapers`. When a smooth-shaper axis is passed in, `_extract_shapers` (per the resolution above) records `A_axis = 0.0`.

Open question: does `v_cap_fn` behave sanely with `A_axis = 0.0` on one or both axes? Expected: `A_axis = 0` ⇒ the shaper term drops out of the min, and `v_cap` falls back to the non-shaper bounds (centripetal × a_max, v_max). Verify with a targeted test exercising a smooth-shaper axis end-to-end through `QuinticShape.from_moves → v_cap_fn`. If the result is finite and non-zero, move on. If it returns 0 or crashes, file as a known issue in the spec (and a TODO in the code) — the smooth-shaper-aware in-blend cap is **out of Plan 2 scope**; it's a follow-up that pillar 2 (unified v(s)) will revisit.

### Reversal-threshold consistency check

`QuinticShape.from_moves` uses `REVERSAL_EPS` (check current value in `klippy/blendquintic.py`) to detect reversals and return None. `blendplanner` uses `dp <= -0.5` (120°) to force a stop. If they drift, a narrow angle band near 120° could fall through both branches uncapped.

Resolution: during merge, confirm they agree. If not, adjust `blendplanner`'s threshold to match `QuinticShape.REVERSAL_EPS` (or expose it as a module-level constant on `blendquintic` and import from there).

### Tests

- Full pytest suite green (`python3 -m pytest test/` — current magnum-opus passes 355; smooth-shapers adds an unknown delta, likely 30–80 more).
- All blend* tests (blendmath, blendplanner, blendquintic, blendshape, blendprepass, blendshaper).
- All input_shaper + shaper_calibrate tests.
- All extruder tests.
- New regression test (if interaction check above reveals anything worth guarding).

### Commit strategy

Single merge commit for the full integration. Conflict-resolution lives in that merge commit. `git commit -m "merge: bring smooth-shapers into magnum-opus"` with a body summarizing the 4 conflict resolutions and the two open-questions outcomes.

---

## Phase B — HP-stepcompress port

### What it does

The upstream commit `9c49716e` (Dmitry Butyugin, via Rogerio Goncalves) replaces the old step-timing encoding:

```
next_step = step + interval + add·i·(i-1)/2          // legacy
```

with a 2nd-order encoding:

```
next_step = step + (interval + rounding) >> shift     // HP
interval += add
add += add2
```

The `add2` second-difference term lets the compressor represent curves with a ~3rd-order polynomial approximation instead of piecewise-linear intervals. Result: fewer MCU messages per unit trajectory, tighter `min_move_time` ceiling, cleaner match to smooth-trajectory planners (magnum-opus's quintic corners benefit directly).

### Port approach

Direct cherry-pick of two commits from `upstream/bleeding-edge-v2`:

| Commit | Role | Lines |
|---|---|---|
| `9c49716e` | Core HP impl: new `stepcompress_hp.c`, extensions to `stepcompress.{c,h}`, `stepper.py`, `src/stepper.c`, `chelper/__init__.py` | +903 |
| `b2854f71` | Kconfig opt-in; `src/stepper.c` refinements | +94 / -60 |

Preserve authorship (Dmitry / Rogerio) for future upstream-patch traceability.

### Files touched

```
klippy/chelper/stepcompress.c          (modified)
klippy/chelper/stepcompress.h          (modified)
klippy/chelper/stepcompress_hp.c       (new, 621 lines)
klippy/chelper/__init__.py             (minor)
klippy/stepper.py                      (modified)
src/stepper.c                          (modified)
src/Kconfig                            (+6 lines)
src/avr/Kconfig                        (+8 lines)
```

### Opt-in semantics

`b2854f71` gates HP behind a Kconfig option (`CONFIG_STEPPER_HIGH_PRECISION_PROTOCOL` or the upstream name — verify during cherry-pick). Off by default. Users opt in by rebuilding MCU firmware with the option selected. Existing firmware builds keep the legacy protocol.

### Expected merge cleanliness

Magnum-opus's prereq state on the 6 target files matches bleeding-edge-v2's pre-HP state (common ancestor `d7f6348a` on `stepcompress.c`). No intervening local commits touch these files. Both cherry-picks expected to apply without conflict; verify with a dry-run before committing.

### Tests

- chelper rebuild clean on host (`make` in `klippy/chelper/`).
- Python test suite green.
- MCU firmware build on at least one target (e.g. stm32) with the new Kconfig option enabled — verifies `src/stepper.c` + Kconfig wiring. **Deferred to user** (requires an MCU toolchain; automation is out of scope).
- HW verification **deferred to user** (not a landing criterion).

### Commit strategy

Two cherry-pick commits, matching upstream. Do not squash — keeps blame/bisect clean across upstream.

---

## Risks & mitigations

| Risk | Mitigation |
|---|---|
| Phase A merge conflicts harder than expected | Phase A laid out file-by-file; if resolution tangles beyond the 4 expected files, abort, re-scope as a dedicated "pre-plan-2 merge" spec, then reduce Plan 2 to Phase B only. |
| Smooth-shaper × quintic v_cap interaction surprise | Interaction check above. If the smooth axis returns `A_axis=0`, verify `v_cap_fn` degrades gracefully (shaper term drops out). If crash, file as follow-up and proceed with a TODO. |
| HP-stepcompress regression | Opt-in only (off by default). Existing setups keep legacy protocol. |
| `suppressed_junction_v` port has a bug | Port the smooth-shapers regression tests alongside the helper; they guard the behavior. |
| `REVERSAL_EPS` and 120° threshold drift | Consistency check during merge; align thresholds via a shared constant. |

---

## Out of scope

- Any algorithmic change to HP-stepcompress (straight port).
- Any re-derivation of the suppressed-junction-v math (verbatim port).
- Smooth-shaper-aware in-blend v_cap (pillar 2 territory; may surface as a follow-up TODO depending on interaction check).
- Non-linear PA integration with magnum-opus's extruder-first-class framework (Plan 3).
- Hardware validation (user runs separately).

---

## Success criteria

1. `git status` clean on magnum-opus.
2. Full pytest suite green (current magnum-opus 355 + smooth-shapers delta).
3. `klippy/chelper/` C builds cleanly on host.
4. At least one MCU firmware target builds with the new Kconfig option enabled.
5. No behavior regression on existing magnum-opus quintic tests.
6. No behavior regression on smooth-shapers' input-shaper / shaper-calibrate / extruder tests.
7. The two open questions (smooth-shaper × quintic cap, reversal-threshold consistency) resolved or explicitly deferred with a TODO.

---

## Next plan

**Plan 3** — Non-linear PA integration with magnum-opus's extruder-as-first-class-constraint framework (pillar 3 prereq). Smooth-shapers brings non-linear PA as Python code; Plan 3 wires it into the magnum-opus extruder kinematics + `blendshape.ExtruderLimits`.
