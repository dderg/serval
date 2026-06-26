# Drive Reference — Inovance A6-EC (CiA 402)

Companion to `SPEC.md`. The subset of the A6-EC EtherCAT servo's object dictionary, operation modes, homing methods, and fault codes that the sensorless-homing design depends on. Distilled from `A6-EC_series_servo_drive_manual.pdf` (page numbers cited). Downstream reads this instead of the 279-page manual.

## Operation modes (`6060h` Modes of operation, display `6061h`)

| Value | Mode | Role here |
|---|---|---|
| 6 | Homing (HM) | drive-autonomous homing — see methods below; **not** the chosen mechanism (design.md Option A) |
| 8 | Cyclic Synchronous Position (CSP) | the normal drip mode; planner streams target position (`607Ah`) every DC cycle. Manual p.106–109 |
| 9 | Cyclic Synchronous Velocity (CSV) | not used |
| 10 | Cyclic Synchronous Torque (CST) | not used (a torque-mode probe is conceivable but out of scope). Manual p.112 |

In CSP the drive follows `607Ah` and the host owns accel/decel. Switching `6060h` mid-stream requires the motion ring empty (enforced by `ethercat-rt` SeedServoHome guard).

## Object dictionary — objects the trigger/lifecycle use

| Index | Sub | Name | Type | Use in this feature |
|---|---|---|---|---|
| 6040h | 00 | Controlword | U16 | enable / fault-reset / (HM only) bit 4 start |
| 6041h | 00 | Statusword | U16 | bit 3 = fault, bit 13 = "followed position error alarm" (CSP), bit 15 = homing complete (HM) |
| 6060h | 00 | Modes of operation | I8 | 8 = CSP (normal), 6 = HM |
| 6061h | 00 | Modes of operation display | I8 | confirm a mode switch took effect |
| 6064h | 00 | Position actual value | I32 | feedback; basis of homed frame after latch |
| 6065h | 00 | **Excessive position deviation threshold** | I32 | **the fault window.** Default 3145728 counts. Raised while armed so a stalled track does not fault (Er47.0). Restored on disarm |
| 6066h | 00 | Following error time out | U16 | companion to 6065h; may also be relaxed while armed |
| 6071h | 00 | Target torque | I16 | (torque-mode only; not used in CSP path) |
| 6072h | 00 | **Max torque** | U16 | 0–4000 (0.1% units). Cap while homing into the stop. Default 3000 |
| 60E0h | 00 | Positive torque limit | U16 | directional torque cap while homing |
| 60E1h | 00 | Negative torque limit | U16 | directional torque cap while homing |
| 6077h | 00 | Torque actual value | I16 | optional trigger discriminator (saturation = at the stop) |
| 606Ch | 00 | Velocity actual value | I32 | optional trigger discriminator (near-zero = at the stop) |
| 60F4h | 00 | **Following error (position deviation)** | I32 | **the trigger signal.** RT loop compares against the armed threshold. Exposed as `ec_rt_get_following_error()` |
| 607Ch | 00 | Home offset | I32 | written by method-35 latch so the frame = configured home |
| 6098h | 00 | Homing method | I8 | range −2..35 (HM only; see methods) |
| 6099h | 01/02 | Homing speeds | U32 | search / zero speeds (HM only) |
| 609Ah | 00 | Homing acceleration | U32 | HM only |

The relaxed window (6065h while armed) must exceed the trip threshold so the trigger fires *before* the drive's own deviation fault.

## Homing methods (`6098h`) — relevant subset

The drive defines methods −2..35 (manual p.74–78). The ones that matter:

- **−2** — drive forward into the mechanical extreme; declares the stop when *torque hits the limit + speed ≈ 0 + held for a set time*, then reverses to the nearest Z pulse. True sensorless hard-stop homing.
- **−1** — same as −2 but reverse direction.
- **35** — "current position is home." Used today (via `seed_home.rs`) only to *seed/zero the frame* after our own trip+back-off, **not** to find a stop.
- 1–34 — switch/limit-based methods (HSW, PL/NL limit switches, Z pulse). Not used; we have no such switches on a sensorless axis.

Methods −1/−2 prove the drive *can* sensorlessly find a stop on its own — but autonomously, per drive, which is why design.md rejects them as the coordinated mechanism for CoreXY/AWD.

## Deviation faults (the thing we must avoid while armed)

- **Er47.0 — Excessive position deviation** (code 0x470 / 0x8611, resettable). Raised when deviation exceeds `6065h`. Among listed causes: *"Motor locked-rotor occurs due to mechanical factors"* and *"the fault value is too small relative to operating conditions — increase 6065h."* That is exactly the homing condition; raising 6065h while armed is the documented avoidance. Manual p.190.
- **Er47.1 — Position deviation overflow** (code 0x471 / 0x8611, resettable). Manual p.191.

## DI signals present but unused here

The drive has DI-mappable Positive limit (P-OT), Negative limit (N-OT), and Home switch (HSW) inputs (manual p.108-ish DI table). A sensorless axis uses none of them — the whole point is no switch wiring.
