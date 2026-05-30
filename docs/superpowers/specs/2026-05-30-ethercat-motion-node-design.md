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

The refactor promotes that existing structure into a trait. **The seam is
chosen so the code most likely to regress does not move.** The closure's
per-MCU clock-base arithmetic (`bridge.rs:2260–2364` — the `schedule_state`
fresh/drained/continuous rebasing, with all its hard-won bench bug-fixes) is
*clock-domain-agnostic*: it only inlines two node-specific lookups —
`now_clock` (today `router.compute_ack_clock` + a 5 s block-wait for clock-sync
to converge) and `freq` (today `clock_freqs[mcu_id]`, fed by klippy's
`ClockSyncEstimator`). Those two lookups become the trait surface; the
arithmetic stays in the closure verbatim:

```
trait MotionNode: Send + Sync {
    fn now_clock(&self) -> Result<u64, DispatchError>;  // node clock-domain "now"
    fn clock_freq(&self) -> f64;                         // ticks/sec
    fn load_and_push(&self, plan: McuPushPlan) -> Result<(), DispatchError>;
    // query_state(channels) -> NodeState added later for feedback (out of M1)
}
```

- `StepperMcuNode` — `now_clock` = the existing `compute_ack_clock` + block-wait;
  `clock_freq` = the `clock_freqs` lookup; `load_and_push` = the slot-alloc +
  `producer::load_curve` + `dispatch_push_segment` inner loop (`bridge.rs:2465–2557`).
  All lifted verbatim over the serial `KalicoHostIo`. No behavioural change.
- `EtherCatNode` — new. `now_clock` = `monotonic_ns()` directly (no router, no
  block-wait: the clock is live and shared); `clock_freq` = `1e9`; `load_and_push`
  = the same producer path over a **unix-socket** `NativeCall` to the EtherCAT
  RT process.

The dispatch closure keeps `build_push_params` + the `schedule_state`
arithmetic, but reads `node.now_clock()` / `node.clock_freq()` where it used to
inline them, then calls `node.load_and_push(plan)`. Routing is per-channel: a
node only receives curves for the channels it owns.

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

## Findings from planning investigation (2026-05-30)

- **No clock-sync message exists in kalico-native.** Synchronization is carried
  by absolute `t_start`/`t_end` (u64) inside `PushSegment`, not by round-trips.
  Because the EtherCAT RT process runs on the *same host* as klippy, both share
  `CLOCK_MONOTONIC`; the sync requirement is met by a shared clock domain. The
  endpoint drives `runtime`'s evaluator with `now_cycles_u64 = monotonic_ns`,
  `cycles_per_second = 1e9`.
- **Receiver path confirmed:** `FrameSource<UnixStream>` → `Demuxer::feed_slice`
  → `decode_message_header` → `MessageKind::from_u16` → `Struct::decode(body)`.
  Frames are `0x55 | len:u16 | channel:u8 | payload | crc16` (CCITT); the
  per-message header is `kind:u16 | version:u8 | correlation_id:u32` (7 bytes).
- **Evaluator reuse confirmed:** `runtime` (default `host` feature, `f32`).
  Use `CurvePool::try_alloc_and_load(slot, &[WirePiece])` → `CurveHandle`,
  `lookup_active(handle)`, `eval_position_velocity(&piece, t_local_s)`. Output is
  **millimetres**; the endpoint applies a configurable `counts_per_mm` to reach
  encoder counts. The host SPSC queue is unnecessary — hold the active segment
  directly and walk pieces in the endpoint.

## Resolved decisions (2026-05-30, Plan 2 planning)

- **Transport: Approach B — lean native socket client, endpoint stays pure
  kalico-native.** The earlier "main risk" (below, struck) was based on a wrong
  reading. `KalicoHostIo` is *not* serial-only and the curve/segment traffic is
  already socket-portable: `producer::load_curve`/`push_segment` reach the wire
  only through `KalicoHostIo::kalico_call`, which emits **pure kalico-native
  `0x55` frames** byte-identical to what the Plan-1 endpoint already decodes.
  The *only* incompatibility is the construction handshake (`identify_handshake`
  speaks Klipper msgproto, not kalico-native). So instead of making the endpoint
  impersonate a Klipper MCU (Approach A), we: (1) hoist the one method the
  producers use, `kalico_call`, onto a small `NativeCall` trait — `KalicoHostIo`
  already has it; (2) genericize the two producer fns over `&impl NativeCall`
  (one-line signature change); (3) add `UnixNativeConn: NativeCall`, a lean
  socket client that frames a request, writes it, and awaits the
  correlation-matched response. The endpoint never learns Klipper msgproto. No
  `ClockSyncEstimator` round-trips for the EtherCAT node: same host ⇒ shared
  `CLOCK_MONOTONIC` ⇒ `clock_freq = 1e9`, `now_clock = monotonic_ns()`.

- **Crate placement.** `NativeCall` + `UnixNativeConn` live in `kalico-host-rt`
  (next to `kalico_call`); `MotionNode` + `StepperMcuNode` + `EtherCatNode` live
  in `motion-bridge`.

- **Scope split.** Plan 2 is **Rust-side only** and fully testable without
  klippy (mock `NativeCall`, loopback `UnixNativeConn`, and the real Plan-1
  endpoint as socket peer). Everything needing a live klippy or a `printer.cfg`
  — the axis→node mapping config surface, passthrough-anywhere wired into real
  config, the STM32/fake-stepper config that makes klippy validate, and the
  first real `G1` jog — is a **final integration step done with the user**, after
  the Rust side lands. The M1 jog needs no new command: `G1` already flows
  `gcode_move → toolhead.move → bridge.submit_move → classify → planner →
  ShapedSegment → dispatch`.

## Open questions / risks

1. ~~**`KalicoHostIo` is serial-only (confirmed).**~~ **Superseded by the
   Resolved Decisions above.** `KalicoHostIo` has `open_tcp`/`open_pipe`/
   `open_with_port`, and `kalico_call` already emits socket-portable
   kalico-native frames. Plan 2 adds `UnixNativeConn: NativeCall` rather than
   routing through `KalicoHostIo`; the producer fns become generic over
   `NativeCall`. This is a small, low-risk change, not the main risk.
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
