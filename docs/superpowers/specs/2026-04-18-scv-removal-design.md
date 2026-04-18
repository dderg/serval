# SCV / Junction Deviation Removal — Design Spec

**Date:** 2026-04-18
**Branch:** `blend-arc` (continues sub-spec #4 work)
**Sub-spec:** #5 of the blend-arc roadmap
**Status:** APPROVED — ready for implementation plan

---

## Goal

Delete the `square_corner_velocity` (SCV) / `junction_deviation` (JD) corner-velocity machinery from the Kalico fork's planner. Sub-spec #4 added a real arc-blending stage (`CornerBlender`) that produces tangent corner arcs with their own centripetal velocity cap; the legacy JD term in `Move.calc_junction` is now redundant for the primary kin↔kin path and inferior for the residual fallback paths.

This is a **pure code-deletion pass**: no new behavior, no new knobs, no migration shims. The fork-as-gate philosophy applies — users opting into Kalico-blend-arc accept the removal.

## Architecture

```
Before:
  Move.calc_junction → min(JD-cap, centripetal-cap, …)
  Corner velocity governed by JD as a fallback "chord-tolerance" virtual-arc cap.

After:
  CornerBlender intercepts every kin↔kin corner, produces a tangent arc with
    v² ≤ 0.866·a·R cap.
  Move.calc_junction → min(centripetal-cap, …) only.
  At blender-tangent junctions, the centripetal block is naturally skipped via
    the existing `cos_theta_d2 > 0` guard.
  At blender-rejected corners (degenerate R, U-turn), CornerBlender already
    forces `next_junction_v2 = 0` via `limit_next_junction_speed(0.0)`.
```

## Verification (cross-checked by three independent research subagents)

**Math agent** — verified `calc_junction`'s angle convention (`θ = π − α`), confirmed centripetal-only behavior at U-turn (→ 0, full stop) and tangent (→ ∞, no constraint, guard skips), and confirmed JD's geometric meaning (corner-cutoff distance along bisector, not chord error). Flagged that on long moves at moderate angles, JD can be 130×–240× tighter than centripetal — but in our pipeline those corners are caught by the blender before they reach `calc_junction`.

**Industrial-CNC agent** — verified LinuxCNC `tp.c`/`blendmath.c`, Siemens G642/G645, Fanuc AICC all use centripetal-on-arc with **no JD term**. JD is exclusive to the Grbl/Marlin/Klipper lineage *because they have no arc blending*. Verdict: GO. Required ordering condition (naive-CAM prepass shipped before JD deletion) is already satisfied — sub-spec #2 shipped `CollinearCollapser` in `klippy/blendprepass.py`.

**Klipper-history agent** — JD has been in Klipper since the 2015 initial commit, lifted from Grbl 2011. Kalico/Danger-Klipper never replaced it. Our `blend-arc` branch is the first Klipper-lineage planner-level arc blend. Concrete failure mode call-out: blender-rejected corners must funnel back to `calc_junction` *or* be force-stopped; current `_emit_arc` already does the latter via `limit_next_junction_speed(0.0)`.

**External-API agent** — surveyed Moonraker, Mainsail, Fluidd, KlipperScreen, OctoPrint plugin, Klippain Shake&Tune. Recommendation: drop the status field in same release as the config knob. Documented impact below.

## In Scope

### `klippy/toolhead.py`

**`Move.__init__`** — delete `self.junction_deviation = toolhead.junction_deviation`. The `Move.junction_deviation` field is removed entirely.

**`Move.calc_junction`** — delete the JD half:

```python
# BEFORE
if one_minus_sin_theta_d2 > 0.0 and cos_theta_d2 > 0.0:
    R_jd = sin_theta_d2 / one_minus_sin_theta_d2
    move_jd_v2 = R_jd * self.junction_deviation * self.accel
    pmove_jd_v2 = R_jd * prev_move.junction_deviation * prev_move.accel
    quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2
    move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2
    pmove_centripetal_v2 = prev_move.delta_v2 * quarter_tan_theta_d2
    max_start_v2 = min(
        max_start_v2, move_jd_v2, pmove_jd_v2,
        move_centripetal_v2, pmove_centripetal_v2,
    )

# AFTER
if cos_theta_d2 > 0.0:
    quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2
    move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2
    pmove_centripetal_v2 = prev_move.delta_v2 * quarter_tan_theta_d2
    max_start_v2 = min(
        max_start_v2, move_centripetal_v2, pmove_centripetal_v2,
    )
```

The `one_minus_sin_theta_d2 > 0.0` guard becomes redundant once JD is gone (it only existed to prevent `sin/(1-sin)` division-by-zero); the centripetal expression has no such pole, and the existing `cos_theta_d2 > 0.0` guard prevents its own division-by-zero at full tangency.

**`ToolHead.__init__`** — replace SCV/JD setup:

```python
# BEFORE
self.square_corner_velocity = config.getfloat(
    "square_corner_velocity", 5.0, minval=0.0
)
self.orig_cfg["square_corner_velocity"] = self.square_corner_velocity
self.junction_deviation = self.max_accel_to_decel = 0
self._calc_junction_deviation()

# AFTER
scv_legacy = config.getfloat("square_corner_velocity", None, minval=0.0)
if scv_legacy is not None:
    config.deprecate("square_corner_velocity")
    logging.warning(
        "config option [printer] square_corner_velocity is obsolete; "
        "the new arc-blending planner ignores it. Remove it from your "
        "config to silence this warning."
    )
# self.square_corner_velocity, self.junction_deviation, and the
# _calc_junction_deviation method are all gone. self.max_accel_to_decel
# is now a @property derived from min_cruise_ratio.
```

**`ToolHead.max_accel_to_decel`** — convert from field to `@property`:

```python
@property
def max_accel_to_decel(self):
    return self.max_accel * (1.0 - self.min_cruise_ratio)
```

This eliminates six recompute call sites (`__init__`, `cmd_M204`, `set_accel`, `reset_accel`, `cmd_SET_VELOCITY_LIMIT`, `cmd_RESET_VELOCITY_LIMIT`) — the property is computed on every read. `Move.__init__`'s read at line 59 (`smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel`) works unchanged.

**`ToolHead._calc_junction_deviation`** — deleted entirely.

**`cmd_M204`, `set_accel`, `reset_accel`** — drop the `_calc_junction_deviation()` call. Property handles it.

**`cmd_SET_VELOCITY_LIMIT`** — accepts `SQUARE_CORNER_VELOCITY=N` but discards it (silent no-op so slicer-emitted commands don't error):

```python
square_corner_velocity = gcmd.get_float(
    "SQUARE_CORNER_VELOCITY", None, minval=0.0
)  # parsed, ignored — kept as local for the all-None guard below
```

Drop the SCV mutation (`if square_corner_velocity is not None: self.square_corner_velocity = ...`), drop the `_calc_junction_deviation()` call, drop `"square_corner_velocity: ..."` from msg list. **Keep** the `square_corner_velocity is None` clause in the all-None guard at line 903 — preserves the existing "args mean no-dump" intent so `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=N` doesn't spam the current-status dump.

**`cmd_RESET_VELOCITY_LIMIT`** — drop the SCV restore, drop `_calc_junction_deviation()` call, drop `"square_corner_velocity: ..."` from msg list. Drop `corner_deviation` restore? No — `corner_deviation` is the new arc-blending parameter from sub-spec #4 and stays.

**`get_status`** — drop the `"square_corner_velocity": self.square_corner_velocity` entry.

**`orig_cfg`** — drop the `"square_corner_velocity"` key.

### `klippy/blendplanner.py`

**`_copy_caller_state` (line 32)** — delete `dst.junction_deviation = src.junction_deviation`. Update docstring to drop `junction_deviation` from the pinned-field list.

**`_emit_arc` (lines 150, 155)** — delete:

```python
arc_jd = min(prev.junction_deviation, nxt.junction_deviation)  # delete
am.junction_deviation = arc_jd                                   # delete
```

The arc moves' velocity cap is already enforced by `am.max_cruise_v2 = arc_cap_v2` (line 154), where `arc_cap_v2` was derived from the arc's centripetal limit `v² ≤ 0.866·a·R`. The JD pin was redundant defense against `calc_junction` clamping arc moves further; with JD gone, calc_junction at the polyline tangent junctions skips entirely via `cos_theta_d2 > 0` guard.

### `klippy/extras/telemetry.py`

**Line 172** — drop the `"square_corner_velocity"` entry from the `[printer]` config-key inventory list. One line, mechanical.

### `klippy/extras/resonance_tester.py`

**Line 572** — replace the toolhead read with a hardcoded fallback:

```python
# BEFORE
toolhead_info = toolhead.get_status(systime)
scv = toolhead_info["square_corner_velocity"]

# AFTER
toolhead_info = toolhead.get_status(systime)
# TODO sub-spec #6: replace with shaper-tuning-aware corner-error budget
scv = 5.0
```

The `scv` value is consumed by `helper.find_best_shaper(scv=scv, ...)` in `klippy/extras/shaper_calibrate.py` as a corner-error velocity budget for shaper smoothing. This is conceptually distinct from the planner's JD usage and belongs in sub-spec #6 (Shake&Tune rework) for proper handling. The hardcoded `5.0` preserves the historical default until #6 lands.

## Out of Scope (deferred)

- **`klippy/extras/trad_rack.py`** — has its own `square_corner_velocity` config and `_calc_junction_deviation` method on `TradRackToolHead`. Self-contained MMU secondary motion planner; doesn't share `Move` state with the main planner. Untouched in #5.
- **All docs** (`Config_Reference.md`, `Config_Changes.md`, `Resonance_Compensation.md`, `Measuring_Resonances.md`, `Status_Reference.md`) — deferred to sub-spec #7 (docs + example configs + parameter naming).
- **7 example `config/printer-*.cfg` files** with `square_corner_velocity = N` lines — deferred to sub-spec #7. They will trigger the new deprecation warning at startup until #7 strips the lines, which is the intended UX (loud signal that the example needs updating).
- **Root `JUNCTION_DEVIATION_ANALYSIS.md`** — legacy analysis doc, deferred to #7.
- **`scripts/calibrate_shaper.py`** — deferred to #7 (docs + scripts pass).
- **Shape-aware shaper-tuning corner-error budget** — sub-spec #6.

## Known Downstream Impact

| Consumer | Impact | Symptom |
|---|---|---|
| Moonraker | NO_IMPACT | Forwards toolhead status generically; field just disappears |
| OctoPrint Klipper plugin | NO_IMPACT | No SCV references |
| Mainsail | DEGRADES | Machine > Limits panel SCV slider falls back to `8`; SET_VELOCITY_LIMIT button becomes silent no-op |
| Fluidd | DEGRADES | Printer Limits widget SCV slider goes blank/NaN (no `?? 8` fallback on the live value path) |
| KlipperScreen | DEGRADES | Limits screen SCV slider non-functional; gcode error toast on slider change once SET_VELOCITY_LIMIT no-ops are silent |
| Klippain Shake&Tune | HARD CRASH | Three commands index `dict[...]` directly; `KeyError` on first invocation. **Requires upstream `.get(..., 5.0)` fix.** |
| jschuh klipper-macros | DEGRADES | `m205` macro's SCV emit becomes silent no-op (not a crash) |

Recommendation: file an upstream issue against Klippain Shake&Tune requesting `.get('square_corner_velocity', 5.0)` fallback. No same-fork shim — fork-as-gate.

## Tests

### New tests (in `test/test_toolhead.py` and `test/test_blendplanner.py`)

1. **`test_blender_decline_zero_radius_forces_stop`** (in `test_blendplanner.py`) — construct kin↔kin pair where the second move is shorter than `0.5 * cot(θ/2)` so blender produces R=0. Assert `prev.next_junction_v2 == 0.0` after `feed()`.

2. **`test_blender_decline_uturn_forces_stop`** (already exists, verify it still asserts `next_junction_v2 == 0.0`).

3. **`test_calc_junction_skips_at_tangent`** (in `test_toolhead.py`) — build two collinear `Move`s, call `m2.calc_junction(m1)`, assert `m2.max_start_v2 == m1.max_start_v2 + m1.delta_v2` (the pre-block `min` value, no centripetal applied because `cos_theta_d2 == 0` skips block).

4. **`test_calc_junction_centripetal_at_90deg`** — build two perpendicular `Move`s of length 10mm with accel=1000, assert `m2.max_start_v2 ≈ 0.25 * 2 * 10 * 1000 = 5000` (i.e. `0.5 * d * a`).

5. **`test_max_accel_to_decel_property`** (in `test_toolhead.py`) — set `toolhead.min_cruise_ratio = 0.7`, assert `toolhead.max_accel_to_decel == toolhead.max_accel * 0.3` immediately, no recompute call needed.

6. **`test_scv_config_deprecation_warning`** — instantiate `ToolHead` from a config containing `square_corner_velocity = 5`. Assert: (a) `logging.warning` was called once with a message containing "square_corner_velocity is obsolete", (b) `config.deprecate("square_corner_velocity")` was called, (c) no `square_corner_velocity` attribute exists on the toolhead.

7. **`test_scv_gcode_silent_noop`** — call `cmd_SET_VELOCITY_LIMIT` with `SQUARE_CORNER_VELOCITY=10`. Assert: (a) no exception raised, (b) no `square_corner_velocity` attribute mutation, (c) the response message does NOT contain "square_corner_velocity".

8. **`test_status_excludes_scv`** — call `toolhead.get_status(systime)`, assert `"square_corner_velocity" not in result`.

### Existing tests to update

- **`test/test_blendplanner.py::test_copy_caller_state_pins_fields`** — the test asserts `dst.junction_deviation == src.junction_deviation`. Drop that assertion. Drop `junction_deviation` from the source-Move fixture.
- **`test/test_blendplanner.py::test_emit_arc_pins_arc_jd`** (if it exists, find via grep) — delete the test.
- **`test/test_blendprepass.py`** and **`test/test_blendmath.py`** — search for `junction_deviation` / `square_corner_velocity` references; delete or update.
- **Any `test/test_toolhead.py` test** that calls `_calc_junction_deviation` or asserts on `junction_deviation` field — delete or rewrite.

## Risk Surface

**The single residual risk:** kin↔kin moves that bypass the `BlendPipelineLookAheadQueue` (i.e., direct calls to the inner `LookAheadQueue.add_move`) lose the JD constraint and rely solely on centripetal. Two known bypass paths:

1. **Drip mode** (chip-resync flow) — uses pre-existing trapq state, no new corners introduced. Low risk.
2. **Z-only move sandwiched between XY moves** (layer change) — currently goes *through* the blender, which produces a 3D arc connecting XY → Z → XY. Probably benign at retract speed but worth flagging for sub-spec #6 audit (Z-plane filtering).

Neither is a deletion blocker. Centripetal-only is *stricter* than JD on short segments and the failure mode is "slower cornering," not "mechanical overload" (verified by industrial-CNC agent, point 3 of its risk assessment).

## Implementation Notes

- All changes land on the `blend-arc` branch on top of sub-spec #4's commits.
- No new files. Pure deletion + one `@property` conversion + one config-deprecation block + two test files extended.
- Estimated diff: ~150 LOC removed, ~30 LOC added (mostly tests + deprecation warning).
- Estimated commits: 6–8 small TDD commits (one per logical deletion + tests).

## Open Items for Sub-Spec #6 (referenced here for traceability)

- Replace `resonance_tester.py`'s hardcoded `scv = 5.0` with a shaper-tuning-aware corner-error budget.
- Audit Z-only move handling in CornerBlender (XY → Z layer-change blending).
- Rework `find_shaper_max_accel` and Klippain Shake&Tune integration.

## Open Items for Sub-Spec #7

- Strip `square_corner_velocity` lines from 7 example `config/printer-*.cfg` files.
- Update `docs/Config_Reference.md`, `docs/Config_Changes.md`, `docs/Resonance_Compensation.md`, `docs/Measuring_Resonances.md`, `docs/Status_Reference.md`.
- Delete root `JUNCTION_DEVIATION_ANALYSIS.md`.
- Audit `scripts/calibrate_shaper.py` for SCV references.
