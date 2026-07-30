# Feature status

Honest per-feature status for Serval.

## Status at a glance

Three tiers, honestly applied:

| Tier | Meaning |
| --- | --- |
| **solid** | used daily on real hardware |
| **verified in sim** | passes the simulator tests; not recently exercised on real hardware |
| **exploratory** | in the pipeline, design still open |

| Feature | Status |
| --- | --- |
| Core pipeline (fitter → planner → lowerer → shaper) | **solid** |
| Corner-deviation clothoid blending | **solid** |
| Follower axes (extruder as a plain axis) | **solid** |
| `smooth_bell` smoothing | **solid** |
| Nonlinear pressure advance | **solid** |
| EtherCAT servo path (test bench) | **solid** |
| Sim + snapshot test infrastructure | **solid** |
| Structured logging / crash forensics | **solid** |
| Step/dir motor path | **solid** |
| Phase stepping | **verified in sim (2026-07)**; not recently exercised on real hardware |
| Explicit `max_jerk` as a first-class limit | **exploratory** — enforced, but whether it earns its keep next to smoothing kernels is open |
| Kinematics beyond cartesian / corexy | **exploratory** |
| Per-axis-group limit model | **exploratory** |
| CAN bus micro-controllers | **verified on the test bench (2026-07)**; never driven a real print |
| CAN-FD data path | **verified on the test bench (2026-07)**; never driven a real print |

### Drive types

| Drive type | Status | Notes |
| --- | --- | --- |
| Step/dir | **solid** | classic path |
| Phase stepping | **verified in sim (2026-07)**; not recently exercised on real hardware | opt in with `phase_stepping: 1` in the stepper's existing section. Switch-endstop homing on a phase-stepped axis is not covered by the sim tests — only sensorless |
| EtherCAT servo | **solid on the test bench** | industrial servo on X, steppers elsewhere |

### Host to micro-controller transports

| Transport | Status | Notes |
| --- | --- | --- |
| USB / serial | **solid** | the daily path |
| CAN bus (classic, 8-byte frames) | **verified on the test bench (2026-07)**; never driven a real print | `[mcu] canbus_uuid`; default framing |
| CAN-FD (64-byte frames) | **verified on the test bench (2026-07)**; never driven a real print | opt in with `CONFIG_CANBUS_DATA_FREQUENCY` in menuconfig; negotiated, falls back to classic |
| EtherCAT | **solid on the test bench** | servo drives, separate endpoint |

## Known limits

- **Jerk on corner blends: enforced, with one soft spot.** The jerk limit
  applies through clothoid blends, including the rotating acceleration's
  normal component. Where blend geometry makes the jerk budget infeasible
  outright, the planner follows the hard acceleration limit instead and
  jerk goes soft there (`rust/geometry/src/velocity/ride.rs`).
- **No per-axis XY limits.** Global limits plus Z-only caps, nothing
  finer.
- **Kinematics:** cartesian and corexy only.
- **Boards: STM32 F4, G0, H7, and F1 (partial).** `src/Kconfig` carries
  those four families plus a Linux-process MCU and the host simulator.
  AVR, LPC176x, RP2040, SAMD, HC32 and the STM32 F0/F2/F7/L4/G4 families
  are absent, and since the MCU executes the trajectory there is no
  host-side workaround for an unsupported chip.
- **Homing tolerates ~100 ms of host or transport stall, and no more.**
  Streamed moves run under `MAX_LEAD_SECS` = 2 s of lead, so ordinary
  host jitter is invisible. Homing does not: it drips, with
  `DRIP_WINDOW_SECS` = 100 ms of lead, and the serial `PushPieces` retry
  burst is deliberately sized at ~90 ms to fit inside that window. The
  slack left over is therefore small enough that one scheduler hiccup
  ends the move — the pump fails loudly with `piece in past at send`
  rather than padding the start time. Bench-observed on a 1 GB Pi 4:
  camera-streamer at 1296x972@30 produced a 101.7 ms send stall and
  aborted a homing move, while the same camera never disturbed a
  mainline-style step-queue host, which buffers far deeper. This is a
  property of the homing path, not of any one board.
- **The F103 runs, but does not home reliably.** Bench-tested 2026-07-30
  on an SKR Mini E3 v2.0 (STM32F103RCT6, 72 MHz, 48 KB SRAM) driving a
  CoreXY Voron 0: the firmware boots, klippy connects (181 commands), the
  motion engine binds all three lanes (`configure_axes ... kin=corexy
  present=0x7 steps_per_mm=[320, 320, 1280]`), streamed moves execute, and
  one sensorless `G28 X` completed against a TMC2209 StallGuard virtual
  endstop. Repeated homing aborts as above, at both 1 and 2 kHz sample
  rates and at both 40000 and 20000 TMC baud. [INFERENCE] the transport
  side of that budget is spent on klipper's bit-banged TMC UART, which
  drives one scheduler timer per bit (25 us at 40000 baud) at the same
  NVIC priority as the motion tick; measured read failures were ~9% with
  the tick running. Treat the F103 as a bring-up target, not a printing
  one.
- **F103 structural caveats.** TIM5 is required, so medium-density F103 is
  out. Every F1 timer is 16 bit, so the step-output deadline is chased in
  <=455 us hops instead of held in one 32-bit compare. Flash is 117 KB of
  256 KB; RAM is the tight one, leaving ~11 KB for klipper's C dynamic
  pool. The board option `!PA14` is mandatory on the SKR Mini E3 v2.0 —
  without it USB never enumerates and there is no way in.
- **Config is not mainline-compatible.** `[kinematics]`, `[motor]`,
  `[axis]`, `[post_processor]` replace the classic sections. This is
  intentional. Migration guide:
  [docs/Config_Migration.md](Config_Migration.md).
- **`mode_inverse` accel demand is unmodeled.** The planner does not fold
  its `k2·jerk` motor-accel demand into the accel limits, so at high jerk
  settings the motor command can exceed `max_accel`
  ([docs/rewrite/shaper.md](rewrite/shaper.md)).
- **CAN bus has not printed anything.** On a bench a toolhead
  micro-controller over CAN is exercised end to end: identify, config
  upload, thermistor and internal-sensor ADC reads, endstop reads,
  streamed motion, an extruder follower axis through its pressure-advance
  chain, coordinated moves whose machine axes live on a different
  micro-controller, a heater accepting a target and reporting back,
  restart and format-transition regressions, and a ten-minute
  continuous-motion soak with zero bus errors. What is still missing is a
  real print, a heater with an actual thermal load, and a physical endstop
  trip mid-move. Treat the first print as a bring-up, not a regression
  test.
- **A G0 toolhead is a one-axis micro-controller.** At the default 2 kHz
  sample rate an STM32G0B1 soaked a single streamed axis for ten minutes
  but faulted within a minute driving three. The failure reproduces over
  USB, so it bounds the micro-controller, not the transport
  ([docs/CANBUS.md](CANBUS.md)).
- **CAN framing is stream-chunked, not block-atomic.** Both ends split the
  byte stream into exact-fit frames, so an FD frame boundary does not
  align with a message block boundary. This halves fragmentation but does
  not deliver "one block, one frame", which is what would remove the
  message-reordering failure class outright.
- **Nothing enforces the CAN receive window.** The micro-controller
  advertises `RECEIVE_WINDOW` and the host does not honour it for the
  native frame channel; the FD receive buffer is sized for headroom
  instead. An overflow is a loud shutdown rather than a silent drop.
- **The USB-to-CAN bridge is classic only.** `src/generic/usb_canbus.c`
  has no CAN-FD support, so a bridge-mode board cannot carry the FD data
  path.
