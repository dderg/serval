---
title: 'Apply homing current to all coupled active rails, not just the homed lane'
type: 'bugfix'
created: '2026-06-28'
status: 'done'
baseline_commit: '3f8da608a19da747d6bec3baa78a808b7f2d74fa'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** On CoreXY, `_home_axis` switches `home_current` onto only the homed lane's rail (`homing.py:437/486/493` pass a single `rail`), while the enable group already spans every moving lane via `kin.active_rails(...)` (`homing.py:422`). The kinematically-coupled partner lane therefore drives the homing move at `run_current` — confirmed on the Trident bench: homing X switched `motor_a`/`motor_a1` to home current but left `motor_b`/`motor_b1` at run current. Result: homing violence tracks `run_current` and `home_current` appears ineffective.

**Approach:** Scope homing-current switching to the same rail set as the enable group. Compute `active_rails` once in `_home_axis` and have `_set_homing_current` iterate the current helpers across all of those rails (deduped), keeping the single max-dwell behavior. Cartesian/single-lane homing is unchanged because `active_rails` returns exactly one rail there.

## Boundaries & Constraints

**Always:** Homing-current scope must equal the enable-group scope — both derive from the one `kin.active_rails(*homing_deltas)` result. Pre-homing set and post-homing restore (success and error-unwind) must cover the identical rail set. Preserve current behavior: skip `None` helpers (motors without a TMC driver), dwell once for the slowest helper, restore on error unwind. Dedupe helpers by object identity so a stepper shared between lanes is switched at most once.

**Ask First:** If reusing `active_rails` would change behavior for any non-CoreXY kinematic (i.e. if it returns more rails than the homed lane for cartesian/single-axis homes).

**Never:** Do not change `tmc.py`/`tmc*.py` current-helper logic, the `set_current_for_homing` guard, or `home_current` config/defaults. Do not alter `active_rails` itself or the CoreXY coupling rule. Do not add per-driver config. No silent recovery — keep failures loud.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| CoreXY home X | active_rails = [X-lane, Y-lane]; both have TMC helpers | every helper across both lanes gets `set_current_for_homing(pre_homing=True)`; one dwell = max returned | N/A |
| Cartesian home X | active_rails = [X-lane] only | identical to today: only X-lane helpers switched | N/A |
| Restore after home | same rail set as pre-homing | every helper restored (`pre_homing=False`); on error-unwind too | restore failure logged, original exception re-raised (unchanged) |
| Stepper without TMC | a helper in the set is `None` | that entry skipped; others still switched | N/A |
| Shared helper across two rails | same helper object in two rails | `set_current_for_homing` invoked once for it | N/A |
| No change needed | all helpers return 0.0 dwell | no `toolhead.dwell` call | N/A |

</frozen-after-approval>

## Code Map

- `klippy/extras/homing.py` -- `_home_axis` (~line 391) computes `active_rails` for the enable group at 422 but passes single `rail` to `_set_homing_current` at 437/486/493; `_set_homing_current` (~495) iterates one rail's helpers. Both change here.
- `klippy/extras/tmc.py:870` -- `set_current_for_homing` (the guarded set/restore); unchanged, called per helper.
- `klippy/motion_kinematics.py:226` -- `active_rails(dx,dy,dz)`; CoreXY couples X+Y (228-229). Source of truth, reused as-is.
- `test/test_homing_current.py` -- unit tests for `_set_homing_current`; signature changes from one rail to a rail set — update call sites and add coupled/dedupe cases.

## Tasks & Acceptance

**Execution:**
- [x] `klippy/extras/homing.py` -- In `_home_axis`, bind `active_rails = list(kin.active_rails(*homing_deltas))` once, build `homing_names` from it, and pass `active_rails` to all three `_set_homing_current` calls. Change `_set_homing_current(self, toolhead, rails, pre_homing)` to loop over `rails`, collect each rail's `get_tmc_current_helpers()`, skip `None`, dedupe by `id(helper)`, take `max` dwell across all, dwell once. -- Makes home current cover every moving lane.
- [x] `test/test_homing_current.py` -- Update the three existing tests to pass a rail *list*. Add: (a) two coupled rails → all helpers across both called once, dwell = max; (b) a helper object shared between two rails is called exactly once. -- Locks the coupled-lane contract and dedupe.

**Acceptance Criteria:**
- Given a CoreXY printer homing X with distinct `run_current` and `home_current` on all four XY motors, when `_home_axis` runs, then every active-lane TMC helper (both belts) receives `set_current_for_homing(pre_homing=True)` and all are restored after.
- Given a cartesian/single-lane home, when `_home_axis` runs, then exactly the homed lane's helpers are switched — no behavior change from today.
- Given any home that raises mid-sequence, when the error unwinds, then the same rail set is restored to run current (existing error-unwind path preserved).

## Verification

**Commands:**
- `./scripts/ci.sh py` -- expected: green, including `test/test_homing_current.py`.
- `pytest test/test_homing_current.py -n0 -v` -- expected: existing + new coupled/dedupe tests pass.
- `./scripts/ci.sh ruff` -- expected: clean (touches `klippy/`).

**Manual checks (bench, optional):**
- On Trident, set distinct currents on all four XY motors, home X, and confirm `klippy.log` now shows `set_actual_current` for `motor_b`/`motor_b1` at their home values (not just `motor_a`/`motor_a1`), then restored to run after.

## Suggested Review Order

**Scope decision (the fix)**

- Entry point: bind the moving-lane set once, mirroring the enable group.
  [`homing.py:421`](../../klippy/extras/homing.py#L421)

- Pre-homing set now spans every active lane (was the single homed rail).
  [`homing.py:438`](../../klippy/extras/homing.py#L438)

**Application across rails**

- `_set_homing_current` iterates all rails, skips `None`, dedupes by helper identity, one max dwell.
  [`homing.py:498`](../../klippy/extras/homing.py#L498)

- Identity dedupe guard — a stepper shared between lanes is switched at most once.
  [`homing.py:504`](../../klippy/extras/homing.py#L504)

**Tests (supporting)**

- Coupled two-rail case: all helpers switched, dwell = max.
  [`test_homing_current.py:73`](../../test/test_homing_current.py#L73)

- Shared-helper dedupe: switched exactly once.
  [`test_homing_current.py:89`](../../test/test_homing_current.py#L89)
