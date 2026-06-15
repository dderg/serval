# Kinematics owns its spatial axes and coupling (no hardcoded x/y/z in the host kinematics layer)

Status: approved design, pre-implementation.
Conformance fix to `2026-06-12-follower-axes-and-limits-design.md` §1 ("the
module assumes no letters; roles bind to declared axis names explicitly").
Builds on the `[motor]`/`[kinematics]`/`[axis]` schema shipped on `e-follows-xy`.

## Problem

The kinematics generalized to role bindings (`[kinematics] axis_x: <name>`,
`KinematicsModule::from_tag` per type), but the **host kinematics layer still
hardcodes the literal spatial names**:

- `motion.py:14` `_SPATIAL_AXIS_NAMES = ("x","y","z")` — a second, independent
  source of truth for "what is spatial," used to *require* `[axis x/y/z]`
  (`:465`) and to *classify followers* (`:482`).
- `motion_kinematics.py` keys the **corexy coupling** and lane labels on the
  literal strings `"x"/"y"/"z"` (`:193` `zip("xyz", …)`, `:196` `coupled["x"]
  = coupled["y"] = …`, `:200`, `:266`). The corexy-specific logic is not
  self-contained; it lives as string-keyed branches in shared code.

The Rust engine is already generic (`VectorNurbs<f64,3>` + index-keyed
followers); only the host duplicates and hardcodes the names.

## Decision / scope

- **The G-code surface keeps X/Y/Z** — `G28` homes X/Y/Z and `gcode.py`'s
  `Coord("x","y","z","e")` are the G-code coordinate standard, the same in every
  slicer and macro. `homing.py` and `gcode.py` are **out of scope** and stay as
  they are. Spatial axes remain addressed as X/Y/Z at the G-code boundary.
- **The kinematics layer becomes the single source of truth.** `motion.py` asks
  the kinematics for the spatial axis set instead of holding its own constant;
  each kinematics type owns its coupling, expressed by **lane index**, not by
  literal axis name.

This is internal architecture cleanup — behavior-preserving — not a feature to
rename spatial axes.

## Changes

1. **`motion.py`** — delete `_SPATIAL_AXIS_NAMES`.
   - The required-axis check (`:465`) becomes: every axis the kinematics
     *claims* must have an `[axis <name>]` section.
   - Follower classification (`:482`) becomes: a follower is a declared axis the
     kinematics does **not** claim (and that carries `motors:`).
   - Ordering: `_read_axes` runs before `_load_kinematics`, so the claimed set
     must be available early — add a lightweight `motion_kinematics.claimed_axes(config)`
     that reads the `[kinematics]` `type` + role bindings (`axis_x/axis_y/axis_z`)
     without building rails, or reorder so kinematics loads first. Plan decides.

2. **`motion_kinematics.py`** — the corexy coupling and active-rail map key on
   **lane index (0/1/2)**, not `"x"/"y"/"z"`. The coupling becomes owned by the
   kinematics type (corexy couples lanes 0 and 1), so each type is
   self-contained rather than a string-keyed branch in shared code. A
   lane→char mapping needed purely for the C itersolve alloc
   (`cartesian_stepper_alloc`) may stay a fixed lane convention (lane 0 → 'x'),
   since that is an internal C-side label, not an axis-name assumption — confirm
   in the plan which `"xyz"[lane_idx]` sites are name-semantics vs C-label.

## Related (already shipped — context, not changed here)

How the extruder knows its axis is already in place (commit `6fbcf7cf4`):

```ini
[extruder]              axis: e                      ; extruder → which axis it drives
[axis e]               follows: x, y, z   motors: extruder_motor
[motor extruder_motor] drive: stepper  ...
```

`PrinterExtruder` reads `axis:` (mandatory) and validates it is a declared
*follower* (has `follows:`). This change is consistent with it: once
`_SPATIAL_AXIS_NAMES` is gone, `e` is a follower precisely because it is
*declared but not claimed by kinematics* — which is what `[extruder] axis: e`
points at. No change to the extruder here.

## Invariants

- **Behavior-preserving.** Existing `x/y/z` cartesian and corexy configs produce
  bit-identical output. The corexy coupling and XY phase-handover group form
  exactly as before (lanes 0/1 coupled).
- Fail loud: the claimed-axis check keeps its clear load-time error.

## Out of scope (deferred / not this work)

- Arbitrary spatial axis **names** at the G-code level (spatial stays X/Y/Z).
- Named / multi-kinematics ("motion channels") — parked in
  `docs/rewrite/future-motion-channels-multi-kinematics.md`.
- The `follows:` shorthand discussion — keep `[axis e] follows: x, y, z`.

## Tests

- Regression: `x/y/z` cartesian boot + corexy boot (mcu-sim self-test +
  phase-stepping test) — coupling/phase-handover still forms.
- Unit: the spatial set is sourced from the kinematics bindings (not a host
  constant); follower classification is "declared but not claimed."
- `./scripts/ci.sh py` + `./scripts/ci.sh ruff` green.

## Gates

`./scripts/ci.sh quick` + `./scripts/ci.sh py` green; mcu-sim cartesian +
corexy + phase-stepping boots PASS.
