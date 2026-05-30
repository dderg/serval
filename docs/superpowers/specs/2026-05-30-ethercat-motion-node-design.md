# EtherCAT as a motion-node module — design

**Date:** 2026-05-30
**Branch:** `ethercat`
**Status:** design, pending implementation plan

## ⚠️ Mandatory reference — read before writing any EtherCAT-communicating code

**Drive manual:** `/Users/daniladergachev/Downloads/A6-EC_series_servo_drive_manual (1).pdf`

Whenever you write, modify, or debug *anything* that communicates over EtherCAT
with this drive — PDO mappings, SDO writes, CiA402 state-machine transitions, DC
sync / SYNC0 configuration, operating-mode selection, fault handling, units /
scaling, object-dictionary access — **read the relevant chapters of this manual
in full first.** Read entire chapters, not snippets: do not guess at object
indices, value semantics, sync-mode support, or timing requirements. The A6-EC's
DC handshake quirks (e.g. `1C32:01=2`, SYNC0-before-SAFE-OP, Er74.x faults) were
only solved by reading the manual carefully; the same discipline applies to every
new register or transition. When in doubt, the manual is authoritative — not
assumptions, not analogy to other CoE drives. (The PDF lives in the user's
`~/Downloads` — outside the repo, so it is not tracked by git; re-download from
the vendor if absent.)

**Page offset:** the printed page numbers are offset from the PDF page index by
~+2 (PDF page 5 prints "03"). So a TOC entry like "Chapter 8 — Communication
Description, p.156" is around **PDF page 158**; fault tables at printed p.171+
are around PDF page 173+. Read a couple pages wide to land on the right content.
Key sections: **Ch. 8 Communication (printed 156–168)** — EtherCAT specs, state
machine, **8.2.3 DC (p.162)**, **8.3 Process/Mailbox data (p.163+)**;
**Ch. 10 Troubleshooting (printed 171+)** — fault tables; **7.7 Torque
Feedforward (p.148)** for the corexy feedforward feature.

## Goal

Bring a STEPPERONLINE A6-EC EtherCAT servo into the kalico motion engine as a
first-class, clock-synced **motion node**, reusing the working CSP/DC bring-up
(`bench/ec_spin.c`) and the engine's existing shaped-Bézier piece stream. The
first observable success is jogging one EtherCAT-driven channel from Klipper,
with the other axes faked, through the *real* backend interface — not a bypass.

This is real code (the throwaway exception applied only to the `bench/` SOEM
spike). The shape must be the one we keep building on.

## Constraints and non-goals

- **No rewrite of the working STM32 motion path.** Promote it into a module
  only as a *low-risk extraction* of structure that already exists. If it ever
  turns into surgery, fall back to adding the EtherCAT node *alongside* the
  current path and migrate the STM32 path behind the trait as a later step.
- **Do not touch the endstop / homing query path.** It is being reworked on a
  separate branch. Servo position/feedback readback is implemented node-locally
  for now and converged with endstop queries later, *by the other branch's
  owner*, not here.
- **Axis-agnostic.** The EtherCAT module must not know or care which kinematic
  axis it drives. A node owns abstract output *channels*; axis→channel mapping
  lives in the kinematics/config layer and is out of scope for the module.
- **Legacy non-motion paths untouched.** Temperature, fans, GPIO, etc. keep
  running over the legacy Klipper MCU comms. The module boundary is the
  **motion plane only**.
- **Passthrough must be available on any axis**, not just Z.

## Key decision: EtherCAT master is a clock-synced kalico-native node (option B)

The sync requirement — "aligned with the steppers to the same level we get
between multiple stepper MCUs" — decides the architecture. Stepper MCUs stay
aligned by clock-syncing to the host (`ClockSyncEstimator`, periodic clock-sync
round-trips). If the EtherCAT master is *its own* clock-synced kalico-native
endpoint, it lines up with the steppers for free, by the same mechanism. This
also matches the documented intent (`docs/kalico-rewrite/mcu-c-rust-boundary.md`:
"an EtherCAT subordinate") and maximises reuse.

Rejected alternative (A): an in-process RT thread inside the `motion-bridge`
`.so`. It would need bespoke sync to the stepper MCUs, drags SOEM + `mlockall` +
`SCHED_FIFO` into klippy's process, and shares fate with the Python host. B keeps
the proven bench-grade RT hardening isolated in a small process and reuses the
protocol/transport/evaluator stack.

## Architecture

### The motion-node boundary

Today the engine has exactly one motion output: a dispatch closure built in
`rust/motion-bridge/src/bridge.rs::init_planner` (~L2137), handed to
`PlannerHandle::spawn` (`rust/motion-bridge/src/planner.rs:235`) as
`Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>`. The
closure body already iterates a **per-node context map** (`dispatch_ios`, keyed
by `mcu_id`) and calls `dispatch::build_push_params` to turn a `ShapedSegment`
into per-node curve loads + segment pushes.

The refactor promotes that existing structure into a trait (provisional):

```
trait MotionNode {
    fn caps(&self) -> NodeCaps;                 // channels owned, phase/servo flags
    fn dispatch_segment(&self, seg: &ShapedSegment) -> Result<(), DispatchError>;
    fn query_state(&self, channels: ChannelMask) -> Result<NodeState, NodeError>;
    // clock-sync participation handled via the shared ClockSyncEstimator path
}
```

- `StepperMcuNode` — the *current* closure body, lifted verbatim: `build_push_params`
  + `producer::load_curve` / `producer::push_segment` over the serial
  `KalicoHostIo`. No behavioural change.
- `EtherCatNode` — new. Same `build_push_params` + producer path, but over a
  **unix-socket** transport to the EtherCAT RT process.

The planner's dispatch closure becomes "for each node: `node.dispatch_segment(seg)`",
which is what it already does in spirit. Routing is per-channel: a node only
receives curves for the channels it owns.

### The EtherCAT RT process (`kalico-ethercat-rt`)

A standalone, RT-hardened binary (the bench's hardening: `SCHED_FIFO`,
`mlockall`, pinned isolated core, performance governor). It is, to the planner,
"just another node." Internally:

1. **kalico-native endpoint** — listens on a unix socket, decodes
   `LoadCurveCubic` / `PushSegment` frames (reuses `kalico-native-transport`
   decode + `kalico-protocol`), answers clock-sync round-trips against the host
   monotonic clock (near-zero offset since same host).
2. **Trajectory evaluator** — reuses `rust/runtime` built with `features =
   ["host"]`: a `CurvePool` + `eval_position_velocity` (the *same* Horner
   evaluator the STM32 runs), walked at the DC rate instead of 40 kHz.
3. **EtherCAT output** — the bench's SOEM/CSP/DC bring-up (the hard-won
   `1C32:01=2` + SYNC0-before-SAFE-OP ordering), reused via a thin Rust FFI
   wrapper around the C, or ported. Each DC tick: evaluate position(t) for the
   owned channel → scale mm→encoder counts → write `target_position` PDO.
   **Before touching any object index, PDO map, mode, or sync setting here, read
   the relevant manual chapters in full** (see the Mandatory reference above).
4. **Feedback** — reads `position_actual` / `torque_actual` / `following_error`
   from the TxPDO each cycle, exposed via `query_state` (node-local; not wired
   into endstop/homing).

### Channels, axis-agnosticism, passthrough-anywhere

- A node owns abstract channels (`channel 0..N`). `EtherCatNode` owns one for
  M1. The module never sees "X/Y/Z".
- Axis→channel assignment stays in the kinematics/config layer (unchanged here).
- **Passthrough generalised:** the per-axis shaper config gains a uniform
  "passthrough" (no-shaper) option for any axis (today only Z defaults off).
  For M1 the EtherCAT channel is passthrough, so the first trajectory is
  unshaped and trivially verifiable. *(Exact config-surface location to be
  pinned in the implementation plan; it must not special-case the EtherCAT
  channel — it is a general per-axis capability.)*

## How future features sit on this boundary (accommodate, don't build)

- **Torque feedforward (corexy inertia).** The EtherCAT node evaluates the same
  Bézier piece, so it can differentiate it analytically for acceleration *at the
  endpoint* — no protocol change. A corexy mass model inside the EtherCAT module
  turns A/B accelerations into per-servo CiA402 torque-offset (60B2h). The
  boundary holds because the curve already carries everything needed.
- **Pause → hand-move → resume.** Idle the servo (back-driveable), `query_state`
  returns *actual* position, reset commanded→actual before resuming. Falls out
  of node-local feedback.
- **Feedback calibration / auto-tune.** Same `query_state` (position/torque)
  correlated with toolhead accelerometer data. Same path.

## Milestones

- **M1 — one EtherCAT channel, jogged from Klipper, other axes faked.**
  `MotionNode` trait + `StepperMcuNode` extraction (if low-risk) + `EtherCatNode`
  + `kalico-ethercat-rt` process + unix-socket transport + passthrough-anywhere
  config + a fake/no-hardware mapping for the other axes. Jog → planner → socket
  → drive moves. No real stepper MCU required.
- **M2 — real second node.** Add the EBB36 stepper as a real `StepperMcuNode`
  (low tickrate acceptable) to validate two synced nodes of different types.
- **Future (out of scope):** torque feedforward + corexy mass model, hand-move
  resume, feedback calibration, converging servo feedback onto the unified
  endstop/state-query path (other branch).

## Reuse map

| Concern | Reused as-is | New |
| --- | --- | --- |
| Shaped piece stream | `trajectory::ShapedSegment`, planner | — |
| Per-node dispatch routing | `dispatch::build_push_params`, `dispatch_ios` loop | lift into `MotionNode` |
| Wire protocol / framing | `kalico-protocol`, `kalico-native-transport` | — |
| Curve load / segment push | `kalico-host-rt::producer` | — |
| Trajectory evaluation | `runtime` (`host` feature) `eval_position_velocity`, `CurvePool` | DC-rate driver loop |
| Clock sync | `ClockSyncEstimator` round-trips | host-side socket answerer (trivial) |
| EtherCAT/CSP/DC bring-up | `bench/ec_spin.c` logic | Rust FFI wrap / port |
| Transport | `Connection` trait | unix-socket `Connection` impl |

## Open questions / risks

1. **`KalicoHostIo` is serial-coupled.** It owns a serial port via the reactor
   thread. A unix-socket transport needs either a generalisation behind the
   `Connection`/`Transport` trait or a parallel socket-backed I/O owner. Must
   verify this stays a small addition and not a refactor of the serial owner.
2. **`StepperMcuNode` extraction must stay an extraction.** If lifting the
   closure into a trait perturbs the working serial dispatch, fall back to
   `EtherCatNode`-alongside per the constraints.
3. **Counts scaling / drive config** (131072 counts/rev, gear, direction,
   soft-limits, fault thresholds) needs a config surface on the EtherCAT node.
4. **mm↔counts and "home/zero"** for an axis-agnostic channel: M1 jogs relative
   from power-on actual position (no homing), avoiding the endstop path.
5. **Crate placement** — `kalico-ethercat-rt` as a new workspace crate; where the
   `MotionNode` trait lives (motion-bridge vs a small shared `motion-node` crate).
   To be decided in the plan.
