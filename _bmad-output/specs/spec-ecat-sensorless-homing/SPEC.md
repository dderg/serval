---
id: SPEC-ecat-sensorless-homing
companions:
  - design.md
  - drive-reference.md
sources:
  - "/Users/daniladergachev/Library/Mobile Documents/com~apple~CloudDocs/A6-EC_series_servo_drive_manual.pdf"
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only — consult them only if you need narrative rationale or prose color this contract intentionally omits.

# EtherCAT Sensorless Homing

## Why

An opportunity to capture, plus a gap to close. EtherCAT servo axes (Inovance A6-EC class, CiA 402) are already wired through this fork — `ethercat-rt` drives them in CSP from the planner's coordinated drip, and the homing scaffolding (`SetDriveLimits`/`RestoreDriveLimits`/`SeedServoHome`, `homing_following_error`/`homing_max_torque` config, `ec_rt_get_following_error`, seed-home mode switching) is already in place. But the CSP drip still relies on a **separate MCU's endstop as the trigger** to end a homing move — the servo drive itself contributes nothing to detecting home. What is missing is making the drive the trigger: nothing in the real-time edge converts a *missed track* (the carriage hits a mechanical stop and the drive can no longer follow the streamed position) into a homing event. Without it, a sensorless servo axis either faults the drive (Er47.x excessive position deviation) or cannot home at all without an external endstop pin. The fix models the servo as a **virtual endstop on the host side, exactly like a probe** (`z_virtual_endstop`): the same remote-trigger arm path, pointed at the `ethercat_node` engine instead of a separate MCU. Closing this lets servo machines home with no endstop wiring, uniformly across cartesian, CoreXY, and AWD (coupled dual-motor) printers. It matters now because the surrounding pieces already landed; this is the keystone, not a greenfield build.

## Capabilities

- id: CAP-1
  intent: An EtherCAT servo axis configured for sensorless homing homes by driving into its mechanical hard stop, with no physical endstop pin, and is marked homed at its configured `position_endstop`.
  success: On the bench, `G28` on a sensorless-configured axis drives to the stop, the drive does not raise Er47.0/Er47.1, the axis ends marked homed, and back-off leaves the carriage off the stop. Demonstrable on a real machine and reproducible in the simulator.

- id: CAP-2
  intent: A "sensorless trigger armed" lifecycle relaxes the drive's following-error fault window and caps drive torque for the duration of the homing move, then restores the run-time limits afterward — including on abort or error.
  success: While armed, the drive tolerates a fully stalled track (deviation pinned at the stop) without faulting. After the move ends by any path (trip, timeout, abort, error), the drive's following-error window and max-torque read back the configured run-time values (`following_error`/`max_torque`), never the relaxed homing values.

- id: CAP-3
  intent: A sensorless servo axis is modeled host-side as a virtual endstop (like a probe's `z_virtual_endstop`): arming it via the existing remote-trigger path points the trigger at the `ethercat_node` engine rather than a separate MCU, and `ethercat-rt` — staying in CSP — detects contact by **actual drive torque** (`6077h`) crossing a gentle threshold and stops motion locally in-cycle, instead of letting the drive fault or stall silently.
  success: Arming the axis allocates a provider `endstop_id` through the existing `allocate_provider_id` / `arm_remote_trigger` path; when actual torque crosses the armed threshold, `ethercat-rt` freezes all targets on the loop in the same DC cycle (local stop) and emits one `EndstopTrip{endstop_id, trip_clock}` whose `trip_clock` falls inside the active homing window, which the engine's existing `dispatch_endstop_trip` consumes for position reconstruction. No separate-MCU endstop is required. A move that reaches `max_travel` with no trip fails loudly — no advance, no silent success.

- id: CAP-4
  intent: Sensorless homing works for cartesian (one motor per axis), CoreXY (axis = two coupled A/B motors), and AWD (two coupled motors per axis), driven by the planner's coordinated drip so coupled motors never run independent profiles; coupled drives share one EtherCAT DC loop so whichever detects the missed-track first stops all motion on that loop.
  success: Homing succeeds on each kinematics on the bench and/or in simulation. For a coupled axis, the first drive to detect ends the move for the whole loop and the reconstructed cartesian trip position is correct under inverse kinematics.

- id: CAP-5
  intent: After the trip and back-off, the drive's internal position frame is latched so the axis coordinate matches the configured endstop position.
  success: Post-home, the drive's `position_actual` maps to the configured endstop coordinate, and repeated `G28` lands within the axis's homing repeatability tolerance across N consecutive runs.

## Constraints

- The planner keeps ownership of the trajectory during homing: motion is the existing coordinated CSP drip + `home_drip` cohort. The drive's built-in autonomous Homing Mode (HM, methods −1/−2) must NOT be used as the coordinated-axis mechanism — it cannot coordinate the two coupled motors of CoreXY/AWD. (See design.md for why, and for its only admissible niche.)
- The drive must not enter Er47.0/Er47.1 (excessive position deviation) during an armed homing move. Achieved by raising the following-error fault window (6065h) and/or timeout (6066h) while armed; the relaxed window must be larger than the trip threshold so detection happens before the drive faults.
- Run-time drive protection must always be restored. Leaving the relaxed following-error window or homing torque cap active after homing would run prints with neutered deviation protection — treat restore as a safety invariant, enforced even on the abort/error path.
- The trigger stays in CSP and trips on **actual drive torque** (`6077h`), not on position-deviation distance. Position-deviation tripping is inherently violent (it presses a threshold-distance past the stop before firing); torque tripping sets contact force directly. Drive torque is also capped (6072h / 60E0h / 60E1h) while homing, low enough not to damage mechanics or the stop, high enough to be detectable.
- Homing contact must be gentle — this is a real requirement, not a nicety (current pin homing audibly presses the toolhead into the stop while the servo fights to hold an unreachable position). Gentleness is met by two bounds: peak contact force is bounded by the homing torque cap, and press duration is bounded to ~one DC cycle by stopping motion **locally in `ethercat-rt`**, mirroring the regular MCU's existing in-cycle endstop local-stop, rather than waiting on a host round-trip.
- Coupled motors of one axis share a single EtherCAT DC loop (one `ethercat-rt`), so the first drive to detect the missed-track stops all motion on that loop under one DC timebase. The design does not need cross-MCU clock reconciliation between coupled drives.
- The second-stage fine re-approach is left to the user (the existing `use_sensorless_homing` flag governs whether the slow re-home is skipped); the feature must honor that toggle rather than forcing one behavior.
- Fail loudly (project rule): a trip clock predating the homing window, an early/stale trip, a homing move that exhausts `max_travel` without a trip, or a drive that faults despite the relaxed window must raise a clear error — never pad, advance, or silently retry. The existing window/clock guards in `rust/motion-engine/src/homing.rs` are kept.
- The change lives at the EtherCAT real-time edge (`ethercat-rt` daemon + `motion-engine` bridge + thin host glue). Printer-MCU firmware (H7/F446 step/dir path) is not modified. The C/Rust boundary rules in `docs/rewrite/mcu-c-rust-boundary.md` apply to any new shared state.
- Reuse the existing trip transport: `EndstopTrip` message, `dispatch_endstop_trip`, `home_axis_start`/`home_axis_poll`, the drip cohort, and `SetDriveLimits`/`RestoreDriveLimits`/`SeedServoHome`. No parallel homing mechanism.

## Non-goals

- Stepper/TMC sensorless homing (stallguard) and any new printer-MCU (H7/F446) firmware homing path.
- Drive-autonomous Homing Mode (HM −1/−2/1/2/…) in any role — not as the coordinated mechanism and not as a single-motor fallback. The host-drip + virtual-endstop path is the only homing mechanism.
- Redesigning probing, bed mesh, or the second-stage fine re-home beyond what the existing `use_sensorless_homing` flag already toggles.
- A generalized multi-vendor CiA 402 abstraction. The mechanism (following-error trip in CSP) is generic CiA 402, but only the A6-EC class is in scope for validation.

## Success signal

A CoreXY (and, by the same path, a cartesian and an AWD dual-motor) servo printer with no physical endstop pins on its homed axes runs `G28`, drives each axis gently into its mechanical stop, and homes — the drive never faults, the home event comes from the missed-track trigger in the real-time edge, the axis frame is latched to the configured endstop position, and repeated homing is repeatable within tolerance. The operator wires no endstops and the behavior is indistinguishable from pin-based homing at the G-code level.

## Assumptions

- The `ethercat-rt` daemon (host-resident, SOEM-based, speaking `mcu_protocol`) is the correct "edge" to host the trigger. The user's "MCU only" preference is reconciled by treating this RT endpoint as the MCU-equivalent boundary; the EtherCAT drives are not behind the H7/F446 step/dir MCUs at all, so literal MCU-firmware-only is not applicable.
- The A6-EC drive in CSP exposes the objects the trigger relies on (60F4h position deviation, 6077h actual torque, 606Ch actual velocity, 6065h/6066h deviation window/timeout) via PDO/SDO as documented in drive-reference.md.
- Homing reuses the existing planner drip + `EndstopTrip` + `dispatch_endstop_trip` path rather than introducing a new transport.
- The servo virtual endstop reuses the existing provider/remote-trigger machinery (`allocate_provider_id`, `arm_remote_trigger`/`disarm_remote_trigger`), the same path probes use — no new `endstop_id` scheme. The only change is the trigger is armed on the `ethercat_node` engine and the trip is emitted by `ethercat-rt`.
- "AWD" is a per-axis coupled-motor mask (`awd_mask`), not a kinematics type; it composes with cartesian and CoreXY.
- The torque trip threshold and cap come from the existing `homing_max_torque` config; no new tuning knob is introduced.
