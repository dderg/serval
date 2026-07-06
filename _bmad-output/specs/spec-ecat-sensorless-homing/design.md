# Design — EtherCAT Sensorless Homing

Companion to `SPEC.md`. Holds the options analysis, the per-kinematics coordination argument, the arm/disarm lifecycle and state machine, and the existing-code-vs-gap inventory. Object-dictionary and homing-method detail it cites live in `drive-reference.md`.

## The two architectural options

### Option A — Drive-autonomous Homing Mode (CiA 402 HM)

Set `6060h = 6` (Homing), pick a homing method in `6098h`, toggle controlword bit 4, and let the drive run the entire homing profile by itself. The A6-EC's methods **−1 and −2** are true sensorless hard-stop homing: the drive drives toward a mechanical extreme, and declares home when *torque reaches the torque limit, velocity is near zero, and the state holds for a set time* (then optionally backs off to the nearest Z pulse). Method **35** = "current position is home" (used today only to seed/zero the frame, not to find a stop).

**Why it is rejected outright:** the drive runs an *independent* profile per axis. On CoreXY a single Cartesian axis is the sum of two motors (A and B) moving together; on AWD two motors are rigidly coupled to one axis. If each drive autonomously runs its own −1/−2 profile, the two coupled motors fight — there is no shared trajectory. HM also takes trajectory ownership away from the planner, conflicting with the project's "planner owns the trajectory" stance. Even on a genuinely single-motor axis it would diverge from the unified path, so it is excluded entirely — not even a single-motor fallback. The host-drip + virtual-endstop path is the only mechanism.

### Option B — Host drip + missed-track trigger in the RT edge  *(chosen)*

Keep the planner's coordinated CSP drip exactly as in normal motion. Before the homing move, **arm**: relax the drive's following-error fault window (`6065h`) and cap torque (`6072h`/`60E0h`/`60E1h`) so a stalled track does *not* fault the drive (no Er47.x). During the drip, the RT loop watches the live following error (`60F4h` / `ec_rt_get_following_error`) and, when it crosses the armed threshold (optionally gated by torque saturation at near-zero velocity), emits an `EndstopTrip` — reusing the exact path GPIO endstops already use. After the trip: **disarm** (restore run-time limits) and latch the home frame (`SeedServoHome`, method-35 / `607Ch`).

This is precisely the user's framing: *"keep the drip from the host, and when the sensorless trigger is armed we switch how we drive the servos so it doesn't crash when tracking is missed, but instead just triggers a function."* "Switch how we drive" = `SetDriveLimits`. "Doesn't crash" = relaxed `6065h`. "Triggers a function" = `EndstopTrip` → `dispatch_endstop_trip`.

**Plain-English version.** Normal driving: the servo is told "be exactly here, now," every cycle, and it complains loudly (faults) if it falls behind — because falling behind usually means something is broken. Homing flips that: we tell the servo "try to be here, but I've turned off your panic alarm and limited how hard you push." We then deliberately command it past the wall. It pushes gently, can't keep up, and the gap between *commanded* and *actual* grows. The moment that gap is big enough, we know the wall is there — that gap *is* the endstop. We stop, write down "this is home," put the panic alarm and full strength back, and back off.

## The servo as a virtual endstop (host side)

Today the servo CSP drip ends a homing move using a **separate MCU's endstop** as the trigger — armed through `RemoteMotionEndstop`, which calls `allocate_provider_id()` for a provider `endstop_id` (≥ 3) and `arm_remote_trigger(engine_mcu_handle, trsync_oid, endstop_id)` against that other engine MCU. Probes already use the same shape via a `z_virtual_endstop` pin (`klippy/extras/probe.py`).

This feature reuses that machinery unchanged and only swaps the trigger source: the sensorless servo axis exposes its own virtual endstop pin and arms the remote trigger **against the `ethercat_node` engine handle** instead of a separate MCU. `ethercat-rt`, watching following error in its DC loop, becomes the thing that emits `EndstopTrip{endstop_id, trip_clock}`. So there is no new id scheme and no change to `dispatch_endstop_trip` — the servo simply *is* a virtual endstop, like a probe is for Z. This resolves what was an open `endstop_id` question: the provider path already answers it.

## Why Option B is uniform across cartesian / CoreXY / AWD

The planner already streams a single coordinated trajectory to all motors of an axis (CoreXY couples A+B via `awd_mask = 0b0011`; AWD couples its pair the same way; cartesian is one motor). Hitting a hard stop stalls *whichever* drives carry that axis. The rule is **the first participating drive to detect the missed-track ends the move for the whole loop** — which is exactly what the existing drip-cohort + `dispatch_endstop_trip` Stop-broadcast already does for GPIO trips. So the same trigger semantics drop onto all three kinematics with no per-kinematics branching; inverse kinematics reconstructs the cartesian trip position from the motor frame as it already does.

Because all coupled drives sit on **one EtherCAT DC loop (one `ethercat-rt`)**, they share a single DC timebase: whoever trips stops all motion on that loop, and there is no cross-MCU clock skew to reconcile (this is what made the earlier "which drive's trip_clock wins" question moot). A coupled AWD pair homes by driving both motors symmetrically at low torque into the stop.

## Trigger design — stay in CSP, trip on torque, stop locally

Pin homing is violent for a structural reason, not a tuning one: CSP commands an *absolute position every cycle* and keeps commanding the toolhead *past* the wall, so the position loop winds torque up to the cap chasing an unreachable point and holds it there until the trip propagates and motion is yanked back. The harshness has two independent components — **peak force** (how hard the loop is allowed to wind up) and **press duration** (contact → motion actually stops). A good trigger attacks both.

Four orthogonal knobs were considered:

1. **Trigger ON** — position deviation (`60F4h`), actual torque (`6077h`), velocity collapse (`606Ch`→0), or the drive's deviation-alarm bit. Position deviation is inherently violent: it can't fire until the toolhead is already a threshold-*distance* past the wall, and that threshold can't be small or accel transients false-trip it. **Torque is the lever for "gentle"** — it sets contact force directly, in physical units, independent of distance.
2. **Approach MODE** — CSP (position), CSV (velocity), CST (torque). CSV/CST remove the position-integral windup entirely and bound force by construction, but they hand the A+B mixing for CoreXY back to us (CSP gets it free from the planner), and mode can't be flipped mid-move (SDO mailbox, ring-empty — see `seed_home.rs`). So they're a per-move choice, held as fallbacks.
3. **Stop LOCALITY** — the master writes every drive's target each DC cycle, so contact on one drive can freeze *all* targets on the loop in that same cycle, before any host round-trip. This collapses press-duration from host-round-trip latency to ~one DC cycle. (No peer-to-peer drive signaling is needed or available; "whoever trips stops all" is executed in the master.)
4. **Multi-touch** — fast coarse find, back off, slow gentle re-touch for the precise latch, like probe accuracy. Composes with the rest; this is where the user-toggled fine approach lives.

**Decision (chosen):** stay in **CSP** (keeps kinematics on the planner → CoreXY/AWD for free), trip on **actual torque** (`6077h`) at a low threshold with a low homing torque cap (peak-force bound), and **stop locally in `ethercat-rt` in-cycle** (press-duration bound). The regular MCU already does an in-cycle local stop on endstop trigger — `ethercat-rt` mirrors that same pattern rather than inventing a new one. The threshold and cap come from the existing `homing_max_torque` config; no new tuning knob. CSV (then CST) are documented fallbacks if CSP-with-torque-trigger still presses too hard, with the CoreXY-coordination cost noted.

**Plain-English version.** Don't measure *how far* it's been shoved past the wall — measure *how hard it's pushing* and stop at a gentle push (knob 1). Cap how hard it's ever allowed to push (knob 2/cap). Let the one controller in charge of all the motors freeze everyone the instant any one touches, instead of waiting for a message to go up and back (knob 3). And first feel for the wall fast, then touch it again slowly for the exact spot (knob 4).

## Arm / disarm lifecycle

```
G28 axis
  │
  ▼
[home_axis_start]  drain motion, build drip cohort
  │
  ▼
ARM ── SetDriveLimits{ following_error_counts = homing_following_error,
  │                    max_torque_tenth_pct   = homing_max_torque }
  │     (raise 6065h fault window above trip threshold; cap torque)
  ▼
DRIP ── planner home_drip cohort streams CSP toward the stop
  │        RT loop each cycle: read 6077h actual torque (+ optional 606Ch speed confirm)
  │
  ├── actual torque ≥ trip threshold ──────────► LOCAL STOP (in ethercat-rt, same DC cycle):
  │                                                freeze all targets on the loop
  │                                                        │
  │                                                        ▼
  │                                              emit EndstopTrip{endstop_id, trip_clock}
  │                                                        │
  │                                                        ▼
  │                                              dispatch_endstop_trip:
  │                                                reconstruct motor pos @ trip_clock,
  │                                                inverse-kin → cartesian trip pos
  │
  └── max_travel exhausted, no trip ───────────► FAIL LOUDLY (no advance, no retry)
  │
  ▼
DISARM ── RestoreDriveLimits  (run-time following_error + max_torque)   [ALWAYS, incl. abort/error]
  │
  ▼
LATCH ── back off stop, SeedServoHome{home_q16}  (method-35 / 607Ch → frame = position_endstop)
  │
  ▼
axis marked homed
```

The DISARM step is a safety invariant: it runs on every exit path, because a print that runs with the relaxed `6065h` window has lost its deviation protection.

## Existing code vs. the gap

Everything below already exists on the base branch — the feature is a keystone, not greenfield.

| Piece | Where | Status |
|---|---|---|
| EtherCAT master / CSP drip | `rust/ethercat-rt/` (SOEM), `klippy/extras/ethercat_node.py` | exists |
| Servo axis config incl. `homing_following_error` (2.5 mm), `homing_max_torque` (50%) | `klippy/extras/servo_axis.py` | exists |
| Claim passes homing + run-time following-error / torque to engine | `servo_axis.py` claim, `bridge.rs` `claim_ethercat_node` | exists |
| `SetDriveLimits` / `RestoreDriveLimits` / `SeedServoHome` / `SetTorque` messages + handlers | `rust/motion-engine/src/servo_torque.rs`, `rust/ethercat-rt/src/bin/ethercat-rt.rs` | exists |
| Op-mode switch (Homing↔CSP) + method-35 + home offset (`607Ch`) | `rust/ethercat-rt/src/seed_home.rs` | exists |
| Live following error read each cycle | `ec_rt_get_following_error`, DC loop telemetry/capture (`ethercat-rt.rs` ~894/955, `capture.rs`) | exists |
| Homing trip transport + reconstruction | `EndstopTrip` msg, `bridge.rs` `home_axis_start`/`home_axis_poll`/`dispatch_endstop_trip`, `motion-engine/src/homing.rs` | exists |
| Drip cohort (coupled multi-axis homing) | `bridge.rs`, `motion-engine/src/pump` drip path | exists |
| `awd_mask` coupling for CoreXY / AWD | `klippy/motion.py`, `configure_axes` | exists |
| Provider-id / remote-trigger machinery (probe path) | `motion_endstop.py` (`allocate_provider_id`, `arm_remote_trigger`), `probe.py` | exists — reused for the servo virtual endstop |
| In-cycle local endstop stop | regular MCU endstop path | exists — `ethercat-rt` mirrors the same pattern |
| **Armed torque trip + in-cycle local stop → `EndstopTrip` in the RT loop** | `ethercat-rt` DC loop | **GAP — the keystone** |
| **Arm/disarm tied to the homing window** (call `SetDriveLimits` before drip, `RestoreDriveLimits` after on all paths) | homing orchestration in `bridge.rs` / host `homing.py` | **GAP** |
| **Servo virtual-endstop wiring** (expose a virtual pin, arm the remote trigger on the `ethercat_node` engine, hand `ethercat-rt` the `endstop_id` to stamp) | `servo_axis.py`, `motion_endstop.py`, `ethercat_node.py` | **GAP — uses existing provider id; no new scheme** |
| **`trip_clock` stamping in the RT edge** (DC-cycle → engine clock) so reconstruction window-checks pass | `ethercat-rt` + `homing.rs` window guard | **GAP / verify** |

## Concrete build sketch (non-binding)

1. In the `ethercat-rt` DC loop, add an *armed* state (set by `SetDriveLimits` carrying, or paired with, a homing-arm flag + torque trip threshold + `endstop_id`). Each cycle while armed: if `actual_torque` (`6077h`) crosses the threshold (optionally confirmed by `velocity ≈ 0`), freeze all targets on the loop *in that cycle* (local stop, mirroring the regular MCU endstop path), stamp the current DC cycle's engine clock, and emit `EndstopTrip` once; latch so it fires at most once per arm.
2. Wire the homing orchestration to arm before the drip and `RestoreDriveLimits` after, on every exit path.
3. Expose the servo axis as a virtual endstop and arm it through the existing provider/remote-trigger path against the `ethercat_node` engine (reusing `allocate_provider_id` / `arm_remote_trigger`); pass the resulting `endstop_id` to `ethercat-rt` so it stamps trips with the right id.
4. Reuse `dispatch_endstop_trip` unchanged; verify the `homing.rs` clock-window guard accepts RT-stamped trip clocks.
5. After the trip + back-off, call `SeedServoHome` to set the frame to `position_endstop`.
6. Tests: unit-test the armed comparator + once-only latch; extend `test/test_servo_homing.py`; reproduce all three kinematics in the simulator (`mcu-sim`) before bench.
