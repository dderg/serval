# Position Reporting — Design

**Date:** 2026-06-14
**Branch:** `position-reporting` (base: `sota-motion`)
**Status:** Approved design, pre-implementation

## Problem

The host needs to report the toolhead's *actual* current position (and velocity) —
the value Mainsail shows in the bracketed `[…]` field, and the value diagnostics
like `GET_POSITION` should expose. Today this value is dead:

- `motion_report.get_status()` is a zero stub (`klippy/extras/motion_report.py:49`)
  — it always returns `live_position = (0,0,0,0)`, so Mainsail's bracketed value
  reads `[0.00]` for every axis.
- `GET_POSITION`'s lower rungs are stubbed/broken: `MCU_stepper.get_commanded_position()`
  returns hardcoded `0.0` (`klippy/stepper.py:158`), `get_mcu_position()` does not
  exist (so the command would `AttributeError`), and `calc_position()` therefore
  averages zeros.

The commanded surfaces already work and are **out of scope**: M114, `toolhead.position`,
and `gcode_move.gcode_position` all report the planner's intended position (the large,
correct number in Mainsail). We do not touch them.

We must support two motor types under one consistent interface:

- **EtherCAT servos** — read the *actual encoder* position (and native velocity). If
  an axis has multiple motors, use the first.
- **Steppers** (plain or phase-stepped) — there are no step counts in this engine; a
  stepper is just one kind of motor and the engine speaks millimeters. We do not report
  what the planner *requested* (dishonest — the MCU may have stopped early, e.g. on a
  homing halt). Instead we ask the MCU "where are you?" and it answers with the position
  its own executor actually drove to, in mm.

## Core principle

**Honesty over convenience.** The reported live position is sourced from the device that
actually executed the motion — the MCU executor for steppers, the drive encoder for
servos — never from the host's record of what the planner sent. The planner's commanded
position continues to exist for *planning continuity* (the next move is queued relative to
the last intended endpoint, not measured actual, to avoid accumulating drift), and remains
what M114 and the commanded status fields report. This change adds an honest *measured*
readout alongside it; it does not replace the commanded one.

## Chosen approach (Approach B)

One MCU-side primitive — "report your position + velocity now, in mm" — uniform across
motor types. The bridge dispatches per-motor to either a serial round-trip (stepper MCU)
or a local EtherCAT telemetry read, and presents one uniform interface to the host. Two
host-facing faces over the same underlying data:

- **non-blocking, cached** → feeds `motion_report` (Mainsail's polled bracketed value);
- **blocking, fresh** → feeds `GET_POSITION`.

Rejected alternatives:

- **Host-side motion-history eval** (use the bridge's existing `motion_state_at_clock()`
  at "now"): cheap and smooth, but reports the planner's record, not what executed, and
  never touches the servo encoder. Fails the honesty requirement for both motor types.
- **Hybrid** (MCU query only for the blocking `GET_POSITION`, host-eval for the always-on
  cached value): for EtherCAT the bracketed value would still show commanded, never the
  encoder — defeating the servo use case. Honesty must live in the always-on path.

## Components and data flow

### 1. MCU position primitive (serial steppers)

Add a request/response message pair to the kalico protocol, modeled on the existing
`QueryRuntimeCaps`/`RuntimeCapsResponse` round-trip (`src/mcu_transport_dispatch.c:273`,
host match-up in `rust/host-rt/src/host_io/mcu_transport.rs`):

- **`MessageKind::QueryMotorState` → `MessageKind::MotorStateResponse`**, defined in
  `rust/mcu-protocol/src/messages.rs` (the `MessageKind` + `Encode`/`Decode`/`Cursor`
  home).
- Response body: `count:u8`, then per motor `[motor_index:u8, pos_q16:i32, vel_q16:i32]`.
  - Position uses the existing Q16 encoding (`encode_q16`, `rust/motion-engine/src/dispatch.rs:107`;
    decode = `q16 as f32 / 65536.0`). 1 LSB ≈ 15.3 µm — adequate for display and diagnostics.
  - Velocity in mm/s, same Q16 encoding (range ±32k mm/s, fits).
- MCU handler reads, per configured slot, the executor's **last-tick** position and
  velocity from `engine.stepping_axes[i].p_prev` / `.v_prev` (`rust/runtime/src/stepping_state.rs:77`),
  which the ISR writes every tick (`engine.rs:452-453`; idle → velocity 0 at `:461`; seeded
  at `:712-713`). NOTE: `engine.tick_caches.{p_prev,v_prev}` is the *seed snapshot only*
  (written exclusively by `seed_position`) and is stale during motion — do not read it.
  Rationale: the per-`AxisState` value is at most one tick old (tens of µs — effectively
  "now"), and a command-context read of it avoids racing the ISR's traversal of the live
  piece ring. When idle it equals the settled endpoint.
- `motor_index` is the engine's per-MCU **slot** index (0..`MAX_AXES`=8), which is a
  **kinematic motor lane**, not a cartesian axis: the host applies the kinematics transform
  (`KinematicsModule::forward`, `rust/motion-engine/src/kinematics.rs:84`) cartesian→motor
  *before* streaming, so on CoreXY slot 0 = motor A, slot 1 = motor B. The reported value is
  therefore motor-space and must be converted back to cartesian on the host (see §4). The
  host maps slots via the binding established at `configure_axis`.

### 2. EtherCAT position + velocity path

`EcTelemetry.position_actual` (encoder counts, `rust/ethercat-rt/src/ffi.rs:13`) is
read every DC cycle in the RT endpoint (`rust/ethercat-rt/src/bin/ethercat-rt.rs:591`)
but never surfaced to the host. We add the surfacing and add native velocity:

- **Velocity (new):** map CiA-402 object **`606Ch` Velocity actual value** (RO, I32,
  TPDO-mappable per the A6-EC manual) into the drive's TPDO. This touches:
  - the C EtherCAT master's PDO map (adds the `606Ch` entry),
  - `EcTelemetry` (`ffi.rs:13`) — add `velocity_actual: i32`,
  - the capture buffer layout (`rust/ethercat-rt/src/capture.rs` — new offset
    alongside `OFF_POSITION_ACTUAL`),
  - a new FFI getter `ec_rt_get_velocity_actual()`.
  - **Unit:** `606Ch` is in the drive's velocity unit (rpm-based on A6-EC). Convert to
    mm/s as `(rpm / 60) × rotation_distance`. The exact unit (rpm vs counts/s vs 0.1 rpm)
    is verified against the live drive during implementation; the conversion is a single
    scalar either way.
- **Surfacing:** the bridge queries the endpoint over the existing endpoint↔bridge socket
  (request/response, symmetric with the serial path — there is no per-DC-cycle push channel
  today, and adding one would be wasteful). The endpoint replies with its latest DC-cycle
  telemetry; the bridge converts counts→mm (reverse of `CountMap::target_counts`,
  `rust/ethercat-rt/src/scale.rs`: `mm = origin_mm + (actual - origin_counts) / counts_per_mm`)
  and the drive velocity unit→mm/s.
- For EtherCAT, "blocking fresh" and "cached" collapse to nearly the same value: the DC loop
  runs at ~kHz, so the endpoint's latest sample is already sub-ms fresh.
- **PDO budget (A6-EC):** the variable `1A00h` TxPDO caps at 10 mappings on the A6-EC drive,
  and `1A00h` is the only configurable TxPDO on its sync manager. Adding `606Ch` (velocity
  actual) would have made 11 entries and the drive rejects the remap (`rc=-6`,
  `EC_RT_ERR_PDO_REMAP`). To stay at 10 we dropped `position_demand (6062h)`: the host already
  knows the commanded position, and `following_error (60F4h)` already carries demand−actual
  directly. `position_demand` is removed from the TxPDO, `in_t`/`ec_telemetry_t`/`EcTelemetry`,
  and the capture record.

### 3. Bridge: unified per-axis query + cache

The bridge hides the serial/EtherCAT split behind one interface:

- serial-stepper motor → `mcu_call(QueryMotorState)` to that motor's MCU;
- EtherCAT motor → read the latest pushed telemetry sample.

Two pyo3 methods over the same data:

- **`live_motor_positions()`** — non-blocking; returns the cached snapshot (per-motor
  pos_mm, vel_mm_s, host-stamp). Feeds `motion_report`.
- **`query_motor_positions()`** — blocking; forces a fresh `QueryMotorState` round-trip to
  each serial MCU (EtherCAT returns its latest sample). Feeds `GET_POSITION`.

A background pull loop refreshes the cache on a fixed cadence (see §6).

### 4. Host: assembly, motion_report, GET_POSITION

- **Bridge-side motor→cartesian transform (not host `calc_position`).** The MCU returns
  per-slot **motor-space** positions/velocities. The bridge — which already owns the
  kinematics matrices — applies `KinematicsModule::inverse(motors)→axes`
  (`rust/motion-engine/src/kinematics.rs:88`; CoreXY `motor_to_axis = [[0.5,0.5,0],[0.5,-0.5,0],[0,0,1]]`,
  identity for Cartesian) to the three spatial slots to recover cartesian X/Y/Z; the E slot
  passes through. The bridge selects the kinematics per MCU from `McuAxisConfig.kinematics`
  (`rust/motion-engine/src/dispatch.rs:21`). The host receives cartesian (pos, vel) per axis
  and does **not** call `klippy`'s `calc_position`. (The "if an axis has multiple motors, use
  the first one" rule is moot for steppers — redundant steppers, e.g. dual-Z, are bound to a
  single engine slot and share its trajectory — and applies only to EtherCAT servos, where
  each motor has an independent encoder; handled in the EtherCAT plan.)
- **`motion_report`** (`klippy/extras/motion_report.py`): `get_status` returns the cached
  snapshot assembled into `live_position` (Coord x,y,z,e), `live_velocity`, and
  `live_extruder_velocity`. This is the bracketed `[…]` value in Mainsail. The zero stub
  is replaced by a cache read (non-blocking — never does I/O in `get_status`).
- **`GET_POSITION`** (`klippy/extras/gcode_move.py:305`): the `mcu` / `stepper` rungs are
  filled with the per-slot **motor-space** values from the blocking query (replacing the
  `0.0` stub at `stepper.py:158` and the missing `get_mcu_position`); the `kinematic` rung
  shows the inverse-transformed **cartesian** values; `toolhead` / `gcode` rungs stay
  commanded. Divergence between the measured and commanded rungs is the diagnostic signal
  the command exists to surface.
- **Untouched:** M114, `toolhead.position`, `gcode_move.gcode_position` (commanded).

### 5. Velocity → cartesian

Per-slot motor-space velocities run through the **same** `KinematicsModule::inverse`
transform as positions (valid because the Cartesian/CoreXY maps are linear), giving
cartesian (vx, vy, vz):

- `live_velocity = ‖(vx, vy, vz)‖`
- `live_extruder_velocity` = the extruder motor's velocity.

Non-linear kinematics (e.g. delta) velocity is **out of scope** for this cut; the velocity
combination asserts/rejects rather than silently approximating.

### 6. Cadence and failure handling

- **Cadence:** background pull at a modest fixed rate while connected (target a few–10 Hz;
  tunable). No motion-gating in the first cut — simpler, and idle traffic is small.
- **Failure handling:**
  - **Blocking explicit query** (`query_motor_positions()`): a timeout or MCU error
    **raises loudly** (project default; fail-loud catches bugs).
  - **`GET_POSITION`** specifically **catches** the blocking-query failure and reports it
    in the command response (e.g. `ERR` for the affected motor / an error line) rather than
    raising. A diagnostic command surfacing an error to the console is loud enough; it must
    not take down the printer.
  - **Background display cache:** a dropped poll **does not abort the print**. It emits a
    structured log event (`event_log_emit`) and serves the last-known sample with its
    host-stamp (staleness is visible via the timestamp). This is a deliberate, scoped
    exception to fail-loud: a flickering Mainsail readout is not safety-critical, and
    killing a print over a missed status poll is worse than a stale number.

## Delivery (two plans)

This spec is implemented in two independently-testable plans sharing the same host surfaces:

1. **Stepper live position** — the wire primitive (`QueryMotorState`/`MotorStateResponse`),
   the MCU read path, the bridge cache + blocking/non-blocking query with the
   `KinematicsModule::inverse` transform, and the `motion_report` / `GET_POSITION` host
   wiring. Delivers honest live position+velocity for stepper machines end to end.
2. **EtherCAT servo surfacing** — PDO `606Ch` mapping, `EcTelemetry`/capture/scale
   additions, the endpoint query handler, and bridge integration so servo axes flow into the
   *same* cache/query/assembly built in plan 1, including the "first motor per axis" rule.

## Testing

- **Rust (runtime):** `QueryMotorState` handler returns per-motor last-tick (pos, vel);
  Q16 position/velocity round-trip encode/decode.
- **Rust (ethercat-rt):** counts→mm reverse conversion against `CountMap`; `606Ch`
  rpm→mm/s conversion; capture-layout offset for the new velocity field.
- **Rust (bridge):** cached vs blocking paths; first-motor selection per rail; per-axis
  serial/EtherCAT dispatch; background-cache failure → log + serve stale.
- **Host (python):** assembly for CoreXY and dual-Z (first-motor); `motion_report.get_status`
  returns non-zero live position/velocity from a populated cache; `GET_POSITION` rungs
  populated and `GET_POSITION` error path reports `ERR` without raising; M114 unchanged.
- **End-to-end:** via the mcu-sim simulator where feasible (live position non-zero
  during a move; settles to endpoint when idle).

## Out of scope

- Homing overshoot / endstop-trigger position latch — already works; not touched.
- Commanded surfaces: M114, `toolhead.position`, `gcode_move.gcode_position`.
- Non-linear-kinematics (delta) velocity.
- Multiple motors per *servo* axis (servo is one-node-per-axis today; the first-motor
  assembly already accommodates it when multi-motor servo axes land).

## Key file references

| Concern | Location |
|---|---|
| Zero stub to replace | `klippy/extras/motion_report.py:49` |
| GET_POSITION command | `klippy/extras/gcode_move.py:305` |
| Stepper `0.0` stub | `klippy/stepper.py:158` |
| Kinematics assembly | `klippy/motion_kinematics.py:217` (`calc_position`) |
| New protocol message | `rust/mcu-protocol/src/messages.rs` |
| Existing R/R model | `src/mcu_transport_dispatch.c:273`, `rust/host-rt/src/host_io/mcu_transport.rs` |
| Executor pos/vel cache | `rust/runtime/src/stepping_state.rs:125`, `rust/runtime/src/motion_core.rs:13` |
| Q16 codec | `rust/motion-engine/src/dispatch.rs:107` |
| EtherCAT telemetry struct | `rust/ethercat-rt/src/ffi.rs:13` |
| EtherCAT counts↔mm | `rust/ethercat-rt/src/scale.rs` |
| EtherCAT capture layout | `rust/ethercat-rt/src/capture.rs:25` |
| EtherCAT DC read site | `rust/ethercat-rt/src/bin/ethercat-rt.rs:591` |
