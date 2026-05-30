# EtherCAT Servo-Axis Integration — Design

**Status:** Design approved 2026-05-31. Scope = Part A (integration to `ready`, no motion).

## Goal

Let any motion axis be driven by an EtherCAT servo drive instead of a stepper, by
routing that axis's trajectory through the existing `EtherCatNode` to the
`kalico-ethercat-rt` endpoint that drives the hardware.

**End goal (eventual):** `SET_KINEMATIC_POSITION` then jog the axis back and forth
on the servo; the operator performs the jogging.

**This spec covers Part A only:** klippy reaches `ready` with the axis declared as
an EtherCAT-backed servo, the bridge instantiates an `EtherCatNode` +
`UnixNativeConn`, connects to the endpoint socket, and identify/configure
succeeds — with **no motion commanded**. A no-hardware stub server makes the whole
path testable with the drive powered down.

Motion (jogging the energized drive) is a separate, operator-supervised step, not
in this spec. The drive is physically powered off until the real-drive handshake.

## Background — what already exists

- **`bench/ec_spin.c` + `bench/libecrt`** — SOEM-based A6-EC (ANCTL AS715N) bring-up
  in CSP/DC. Hardware-verified 2026-05-30 (`wkc=3`, `AL=0x0000`, DC offset ±150 ns
  at 500 Hz). `libecrt.h` exposes `ec_rt_bringup`/`ec_rt_cycle`/
  `ec_rt_set_target_position`/`ec_rt_get_position_actual`/`ec_rt_get_statusword`/
  `ec_rt_get_following_error`/`ec_rt_get_error_code`/`ec_rt_disable`/`ec_rt_shutdown`.
- **`rust/kalico-ethercat-rt`** — the RT endpoint crate:
  - `server.rs` — `FrameServer` (`UnixListener`) kalico-native socket server.
  - `wire.rs` — decodes `Identify` / `LoadCurveCubic` / `PushSegment` /
    `ResetCurvePool`, frames responses.
  - `curves.rs` — cubic `CurveStore`, `eval_curve_at`, armed-segment sampling at
    `now_ns`.
  - `scale.rs` — `CountMap` (mm → encoder counts, relative to a captured origin).
  - `clock.rs` — `monotonic_ns` (CLOCK_MONOTONIC).
  - `ffi.rs` — `libecrt` bindings, gated behind the `hw` Cargo feature.
  - `bin/kalico-ethercat-rt.rs` — endpoint binary (`required-features = ["hw"]`):
    `kalico-ethercat-rt <ifname> [--socket PATH] [--cycle-us N] [--counts-per-mm F]
    [--rt-cpu N] [--rt-prio N] [--handle x|y|z|e]`. Brings up the drive, serves the
    socket, evaluates the cubic at `monotonic_ns`, scales mm→counts, feeds CSP.
  - `bin/ec-test-client.rs` — no-`hw` socket client.
- **`rust/motion-bridge` (Plan 2)** — `MotionNode` trait, `StepperMcuNode` (serial),
  `EtherCatNode` (same-host `UnixNativeConn`, shared `CLOCK_MONOTONIC`,
  `clock_freq=1e9`, `now_clock=monotonic_ns`). `EtherCatNode` is currently
  instantiated only in a unit test.
- **`rust/kalico-host-rt`** — `NativeCall` trait, `UnixNativeConn` (blocking same-host
  kalico-native socket client).
- **Bench** — Pi 3B; A6-EC on dedicated `eth0`; Linux-process stepper MCU
  (`klipper-mcu.service`) already running klippy at `ready`.

## The gap

1. **Bridge never creates an `EtherCatNode`.** The node-build loop
   (`bridge.rs:~2114`) inserts only `StepperMcuNode` into the `nodes:
   HashMap<u32, Arc<dyn MotionNode>>` map.
2. **Dispatch silently skips a non-serial node.** `bridge.rs:2230–2249`: an
   `EtherCatNode` would be in `nodes` but not in `dispatch_ios`, so the `io_weak`
   guard at line 2238 `continue`s past it. (The existing comment at 2230 prescribes
   the fix: make the `nodes` lookup primary; gate the `dispatch_ios`/`io` block,
   which feeds only stepper-only `runtime_seed_position`.)
3. **No klippy config surface** to declare an EtherCAT node or bind an axis to it.
4. **No drive-off test path** — the `hw` endpoint binary requires the drive on the
   bus (`ec_rt_bringup` → `-2` no slaves otherwise).

## Config surface (two-tier)

A **connection node** and a **motion device** are separate concerns:

```ini
[ethercat_node node_x]          # the connection — a capability node, NOT an MCU
socket: /tmp/kalico-ethercat.sock

[servo_x]                       # the X-axis motion device (replaces [stepper_x])
protocol: ethercat
node: node_x
rotation_distance: 40
position_min: 0
position_max: 200
# endstop / homing: future (drive DI). Part A uses SET_KINEMATIC_POSITION.
```

Rules — **generic over axis, never hardcoded to X**:

- For each axis `a in {x, y, z}`: if `[servo_<a>]` exists → build a servo device for
  that axis; else if `[stepper_<a>]` exists → build a stepper rail (today's path).
- **Error** if both `[servo_<a>]` and `[stepper_<a>]` are present for the same axis.
- `protocol:` is the device-type dispatch key (only `ethercat` is implemented;
  the field exists so other fieldbuses can be added without a new section type).
- `node:` references an `[ethercat_node <name>]`. Multiple servos MAY reference one
  node (multi-axis drive / shared bus) or separate nodes. n-servo capable.
- The bench config (X servo, Y/Z steppers) is one instantiation of the rule, not the
  contract.

The endpoint's `--handle`/`--counts-per-mm`/`--ifname` are configured on the
**endpoint service** side (a systemd unit, analogous to `klipper-mcu.service`), not
in klippy. klippy/bridge needs only the **socket path** (`[ethercat_node].socket`)
and the **axis→node binding** (`[servo_<a>].node`). The two must agree (endpoint
`--handle x` ↔ X bound; `--counts-per-mm 3276.8` ↔ `rotation_distance 40` against
131072 counts/rev); agreement is an operator/config-consistency concern, not
enforced across the process boundary in Part A.

## Klippy components

- **`[ethercat_node <name>]` config object** (new, e.g. `klippy/ethercat_node.py`)
  — owns the socket path and is the durable home for future DI/temp/status
  sub-config. It is **not** a `PrinterMCU`; it is a lightweight node object the
  bridge enumerates as a connection of `kind = EtherCat`.
- **`ServoRail`** (new, e.g. `klippy/servo_axis.py`) — the servo motion device,
  a **distinct class from `stepper.PrinterRail`**. It implements only the minimal
  axis-device contract the toolhead's axis slot needs:
  - `get_name`, `position_min`/`position_max`, range;
  - `get_steppers() → []` (no MCU steppers);
  - a `protocol` + `node` binding;
  - **no** `setup_itersolve`, no trapq, no step compression, no microsteps, no
    step/dir/enable pins.
  Because Part A uses `SET_KINEMATIC_POSITION` and does not home, `ServoRail` does
  not need `calc_position`/endstop wired (those are the future DI-homing
  capability). It must satisfy the toolhead's setup calls without a stepper.
- **`MotionToolhead._register_axis` branch** (`motion_toolhead.py:~139`) — generic:
  `if config.has_section("servo_"+axis): ServoRail(...) elif has_section("stepper_"+axis): PrinterRail(...)`.
- **`configure_axes` branch** (`motion_toolhead.py:~950`) — a stepper axis
  contributes `(steps_per_mm, oid, step_mode)` bound to an MCU; a **servo axis**
  contributes an **axis-slot → node `mcu_id` binding** (no oid, no `step_modes`).
  This is where "axis a's trajectory routes to node N" is declared to the bridge.

## Bridge components

- **Parse `[ethercat_node]` → node descriptor** `(name, socket, kind=EtherCat)`;
  assign each an `mcu_id` in the **same keyspace** as serial MCUs (dispatch keys on
  `mcu_id`).
- **Node-build branch** (`bridge.rs:~2114`): serial → `KalicoHostIo` +
  `StepperMcuNode` (existing); ethercat → `UnixNativeConn::connect(socket)` +
  `EtherCatNode`, inserted into `nodes`. Attach = connect socket + one
  `kalico_call(Identify)` to confirm the endpoint is alive (endpoint `wire.rs`
  answers `Identify`).
- **Dispatch restructure** (`bridge.rs:2230–2249`): make the `nodes` lookup
  primary; gate the `dispatch_ios`/`io_weak` block so an EtherCAT plan is **not**
  `continue`d at line 2238. The `io` is used solely for stepper-only
  `runtime_seed_position`; that path becomes conditional on the node being a stepper
  node. This is the restructure the existing comment prescribes.
- **Axis→node plan routing** — `mcu_plans` are already built per-MCU upstream from
  the `configure_axes` bindings. Binding axis `a` to node `N`'s `mcu_id` makes the
  planner emit `a`'s curve in the plan for `N`; dispatch then calls
  `node.load_and_push(plan)` on the `EtherCatNode`, which frames kalico-native
  `LoadCurveCubic`/`PushSegment` over `UnixNativeConn` to the endpoint.

## Data + clock flow

```
toolhead plan (mm cubic curves, per axis)
  → mcu_plans  (X's curve in the plan whose mcu_id = node_x)
  → EtherCatNode::load_and_push(plan)
  → UnixNativeConn  (kalico-native LoadCurveCubic / PushSegment frames)
  → kalico-ethercat-rt endpoint
  → eval cubic at monotonic_ns → CountMap mm→counts → ec_rt_set_target_position (CSP)
```

**Clock:** both sides use `CLOCK_MONOTONIC` (`EtherCatNode` `clock_freq=1e9`,
`now_clock=monotonic_ns`; endpoint `clock.rs` same). Same host ⇒ **no clock-sync
handshake** (contrast the serial MCU's `compute_ack_clock` calibration). The fragile
`schedule_state` clock-base arithmetic in the dispatch is exercised with
`freq = 1e9`; `EtherCatNode::now_clock` returns `monotonic_ns` directly (no
block-wait on a widened MCU clock).

## No-hw stub server (drive-off testing)

Add a no-hardware mode to `kalico-ethercat-rt` (a `--no-hw` flag, or a sibling
`kalico-ethercat-rt-stub` binary built from the no-`hw` lib) that **binds the socket
and answers `Identify`/`LoadCurveCubic`/`PushSegment`/`ResetCurvePool` but never
calls `ec_rt_*`**. It reuses the existing `FrameServer` + `wire` + `curves` (all
already build without `hw`). This is what lets Part A reach `ready` and exercise the
full bridge→socket path with the drive powered down.

## Testing

- **Unit:**
  - `ServoRail` satisfies the axis-device contract (name, limits, empty steppers,
    node binding) without step/dir.
  - `[ethercat_node]` / `[servo_<a>]` parsing, including the "both `stepper_a` and
    `servo_a` present → config error" rule.
  - Bridge serial-vs-socket node-build branch + `Identify` round-trip (against a
    test socket peer).
  - Dispatch restructure: a plan keyed to an EtherCAT `mcu_id` is dispatched to the
    `EtherCatNode`, not skipped at the `dispatch_ios` guard.
- **Integration (drive-off):** klippy + no-hw stub server → klippy `ready` with the
  axis bound to its node; logs confirm the axis's `mcu_plan` routes to the node's
  `mcu_id`.
- **Real-drive handshake (operator-supervised, separate step):** replace the stub
  with the `hw` endpoint on `eth0`; confirm identify/configure against the
  energized-but-stationary drive. Operator powers the drive on request; no motion is
  commanded in Part A.

## Future capabilities (designed-for, not built in Part A)

The `[ethercat_node]` object is the home for these; each is a new node sub-capability
plus a kalico-native message and an endpoint PDO mapping:

- **Digital inputs → endstops/home:** map the drive's DI bits in the input PDO; carry
  input-state / trsync messages (mirroring Klipper's `endstop` surface) so an axis
  can home to a drive limit / home-to-block.
- **Drive/motor temperature → `temperature_sensor`:** surface drive temp via PDO/SDO.
- **CiA402 status → diagnostics:** statusword, following error, fault codes
  (`libecrt` already reads several) exposed as telemetry.

None are required for Part A; the abstraction must not preclude them (which is why
the node is first-class and the device is `protocol`-dispatched).

## Scope / non-goals (Part A)

- **In:** config surface; `ServoRail` + `[ethercat_node]` objects; toolhead +
  `configure_axes` branches; bridge node-build branch + dispatch restructure; no-hw
  stub server; drive-off integration to `ready`; operator-supervised real-drive
  handshake.
- **Out:** any commanded motion; homing; endstops; temperature; status telemetry;
  multi-axis-per-node endpoint handling (single `--handle` per endpoint for now).
- **Generality:** all code is generic over axis and supports n servos; the bench's
  single-X-servo config is an example, not a hardcoded assumption.

## Open implementation details (resolve in the plan)

- Exact `ServoRail` method surface the toolhead/kinematics touch during setup (audit
  every call the cartesian kinematics + `MotionToolhead` make on a rail, ensure
  `ServoRail` satisfies or is correctly bypassed in bridge mode).
- How the `mcu_id` for an `[ethercat_node]` is allocated and threaded from the klippy
  config object through `configure_axes` into the bridge's node map and `mcu_plans`.
- Whether the endpoint service is a hand-written systemd unit for the bench or a
  generated one; how its `--handle`/`--counts-per-mm` are kept consistent with the
  klippy config (documented procedure for Part A; possible validation later).
- Stub server packaging: `--no-hw` flag on the existing bin vs a separate bin
  target.
