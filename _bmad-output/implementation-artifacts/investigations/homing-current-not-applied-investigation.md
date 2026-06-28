# Investigation: Homing current appears not to be applied (violence tracks run_current)

## Hand-off Brief

1. **What happened.** On CoreXY, sensorless homing applies `home_current` only to the **homed lane's** rail; the kinematically-coupled **partner lane keeps `run_current`** during the homing move. Because both lanes drive the toolhead into the endstop, run_current dominates the homing "violence" and home_current applies only partially — read by the user as "home_current does nothing."
2. **Where the case stands.** **Root cause Confirmed** (code + bench logs). `home_rails` enables all `active_rails` (both coupled lanes) but `_set_homing_current` is passed only the single homed `rail` (homing.py:422 vs 437). Bench log: X-lane motors `motor_a`→0.5 / `motor_a1`→1.0 switched; partner-lane `motor_b`/`motor_b1` enabled and moving but never current-switched (stay at 0.8).
3. **What's needed next.** Fix: switch home current on every `kin.active_rails(*homing_deltas)`, not just the homed `rail` (mirror the enable-group at homing.py:422), for both the pre-homing set and the post-homing restore.

## Case Info

| Field            | Value                                                                 |
| ---------------- | --------------------------------------------------------------------- |
| Ticket           | N/A                                                                   |
| Date opened      | 2026-06-28                                                            |
| Status           | Active                                                                |
| System           | improved-kalico fork, branch `homing-current` (HEAD 3f8da608a)        |
| Evidence sources | Source code (klippy/extras/tmc.py, tmc2130.py, homing.py, stepper.py, rail.py); user report |

## Problem Statement

User report (hypothesis, not yet fact): "We are not applying homing current, because the violence of sensorless homing changes with my `run_current`, regardless of `homing_current`."

Note: the configured option is spelled **`home_current`** (tmc.py:815), not `homing_current`. Treating the user's wording as colloquial for the home-current feature.

## Evidence Inventory

| Source                         | Status    | Notes                                                                 |
| ------------------------------ | --------- | --------------------------------------------------------------------- |
| klippy/extras/homing.py        | Available | `_set_homing_current` apply path, lines 437/486/493/495               |
| klippy/extras/tmc.py           | Available | `BaseTMCCurrentHelper`, `set_current_for_homing`, `arm()`, config read |
| klippy/extras/tmc2130.py       | Available | `apply_current` for 2208/2209/2130 (writes IHOLD_IRUN from actual_current) |
| klippy/stepper.py, rail.py     | Available | `AxisRail.get_tmc_current_helpers`, helper wiring via set_tmc_current_helper |
| User's printer.cfg             | **Missing** | Where `home_current` is declared; section/name alignment              |
| klippy.log / structured logs   | **Missing** | `needs_home_current_change` / `set_actual_current` during a home run   |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | User config: is `home_current` set, and under `[tmc2209 X]` (not `[motor]`/`[axis]`)? | High | Open | Discriminates H1 |
| 2 | Logs: do `tmc X: needs_home_current_change True/False` + `set_actual_current` lines appear during the home? | High | Open | Discriminates H1 vs H2 |
| 3 | Confirm rail's PrinterStepper objects carry a non-None helper (name alignment tmc↔motor↔rail) | Medium | Open | Tests H2 |

## Confirmed Findings

### Finding 1: Homing only changes current when home ≠ run

`homing.py:495 _set_homing_current` → `tmc.py:870 set_current_for_homing`. The pre-homing branch acts **only if** `needs_home_current_change()` is true, i.e. `actual_current != req_home_current` (tmc.py:840-843). If they're equal, it returns 0.0 and writes nothing — the motor homes at whatever current is already loaded (run current).

### Finding 2: `home_current` defaults to `run_current`

`tmc.py:815-820`: `config_home_current = config.getfloat("home_current", self.config_run_current, …)`. With no explicit `home_current`, `req_home_current == req_run_current`, so Finding 1's guard is always false and the home is performed at run current. Changing `run_current` then changes homing behavior; "home_current" has nothing to change. **This alone reproduces the reported symptom.**

### Finding 3: StallGuard arming does not clobber the current

`tmc.py:649-689 arm()` writes SGTHRS, GCONF, TCOOLTHRS, TPWMTHRS, thigh — but **not** IHOLD_IRUN. A correctly-set home current survives arming. Rules out "arm() overwrites home current."

### Finding 4: apply path writes the right value

`tmc2130.py:258-267 apply_current` derives irun from `self.actual_current` (set to `req_home_current` during homing) and writes IHOLD_IRUN at print_time. The mechanism is correct when invoked with a differing value.

### Finding 5: Enable ordering is correct

`homing.py:424` enables steppers (inline `_do_enable_engine` → `_init_registers`, run current) **before** `homing.py:437` sets home current — so the home-current write is last and wins. Rules out "enable re-init overwrites home current."

## Confirmed root cause

### Finding 6 (ROOT CAUSE): CoreXY partner lane homes at run_current

**Evidence (code):** `homing.py:419-424` builds the enable group from `kin.active_rails(*homing_deltas)` — on CoreXY `active_rails` couples X+Y (`motion_kinematics.py:228-229`), so homing X enables BOTH lanes. But `homing.py:437` calls `_set_homing_current(toolhead, rail, …)` with only `rail = kin._axis_rails().get(axis)` (the single homed lane, homing.py:402). The post-homing restore (homing.py:486/493) has the same single-rail scope.

**Evidence (bench, klippy.log.2026-06-28_17-51-42:3403-3461):** during an X sensorless home —
`tmc motor_a: set_actual_current 0.5`, `tmc motor_a1: set_actual_current 1.0` (X-lane home currents applied), then both restored to `0.8`. `motor_b`/`motor_b1` are logged "enabled" and physically move (CoreXY) but **never** appear in any `needs_*_current_change` / `set_actual_current` line — they run the entire homing move at `run_current` 0.8.

**Mechanism:** On CoreXY both belts/lanes contribute the force that drives the toolhead into the endstop. With only the homed lane switched to home current and the partner lane left at run current, the homing "violence" is dominated by `run_current` (changing it is felt), while `home_current` changes only the homed lane (a partial effect the user reads as negligible — amplified here because the homed-lane motors split 0.5/1.0 around the 0.8 run value).

## Hypothesized Paths (resolved)

### Hypothesis 1: `home_current` equals `run_current` in effect — **REFUTED**

**Resolution:** Config sets distinct `home_current` (`${constants.homing_current}`=1.0, `homing_current_secondary`=0.5) on the `[tmc5160 …]` sections, and the bench log shows `needs_home_current_change True` with `set_actual_current 0.5/1.0`. Home current is read and applied to the homed lane.

### Hypothesis 2: Current helper not wired onto the homed rail's stepper — **REFUTED**

**Resolution:** Bench log shows `motor_a`/`motor_a1` helpers firing `set_actual_current` during the home. Helpers are wired and invoked for the homed lane.

### Hypothesis 3: Home current applied but dominated by SGTHRS — **REFUTED/superseded**

**Resolution:** Superseded by Finding 6. The dominance is from the partner lane running at run_current, not from SGTHRS.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| User printer config | Settles H1 (is home_current set, and on the right section?) | Read the printer.cfg `[tmc2209 …]`/`[motor …]`/`[axis …]` sections |
| Homing-run logs | Settles H1 vs H2 | `query-logs` skill (or klippy.log) for `needs_home_current_change` / `set_actual_current` around a sensorless home |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Apply origin | `klippy/extras/tmc.py:870 set_current_for_homing` (guarded by `needs_home_current_change`, tmc.py:840) |
| Trigger | `klippy/extras/homing.py:437/495 _set_homing_current` during `home_rails` |
| Condition | `req_home_current == req_run_current` (default coupling, tmc.py:815) **or** rail stepper helper is None (tmc.py:456 wiring) → write skipped |
| Related files | tmc2130.py (apply_current), stepper.py/rail.py (helper wiring), homing.py (orchestration) |

## Conclusion

**Confidence:** High (root cause Confirmed by both code and bench logs; deterministic).

`home_current` *is* applied — but only to the homed lane's rail. On CoreXY the homing move is driven by both coupled lanes, and `_set_homing_current` (homing.py:437/486/493) is scoped to the single `rail` while the enable group (homing.py:422) correctly spans `active_rails`. The partner lane therefore homes at `run_current`, which is why homing violence tracks `run_current` and `home_current` appears ineffective. The same scope bug would also leave the partner lane at the *reduced* home current after a fault if it were in the set — but since it's never in the set, the restore is simply moot for it.

## Recommended Next Steps

### Fix direction
Scope homing-current switching to the same rail set as the enable group. In `home_rails`, compute the active rails once (it already does, homing.py:422) and have `_set_homing_current` iterate **all** of them (their `get_tmc_current_helpers()`), for both `pre_homing=True` (line 437) and the restore (lines 486/493). Concretely: pass the `active_rails` list (or the homed axis's coupled set) into `_set_homing_current` instead of the single `rail`, and dwell on the max returned dwell across all helpers.

Caveat to verify in the fix: a motor shared between lanes must not be double-set; dedupe helpers (the homed lane and partner lane could in principle share a stepper object). Mirror the restore set exactly to the set used pre-homing so no motor is left at home current.

Hand off to `bmad-quick-dev` for the homing.py change; this is a contained edit in `_home_axis`/`_set_homing_current`.

### Verification Plan
On the Trident bench: set distinct `run_current` and `home_current` on all four XY motors, run an X sensorless home, and confirm `set_actual_current` lines now appear for `motor_b`/`motor_b1` (partner lane) at their home values — not just `motor_a`/`motor_a1`. Then vary `home_current` and confirm homing violence now tracks it; vary `run_current` and confirm it no longer affects the homing move.

## Reproduction (Confirmed)

Bench, klippy.log.2026-06-28_17-51-42 (commit 28f29f2bc): X sensorless home. `motor_a`/`motor_a1` switched to home current (0.5/1.0) and restored to 0.8; `motor_b`/`motor_b1` enabled, moved, and stayed at run current 0.8 for the whole move.

## Side Findings

- The user's wording `homing_current` does not match the actual option `home_current` (tmc.py:815). If they literally put `homing_current` in the config, Kalico's unused-option validation would normally reject it at startup — worth confirming the exact key in their config.
