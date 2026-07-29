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
