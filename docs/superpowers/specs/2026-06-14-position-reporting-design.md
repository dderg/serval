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
`QueryRuntimeCaps`/`RuntimeCapsResponse` round-trip (`src/kalico_dispatch.c:273`,
host match-up in `rust/kalico-host-rt/src/host_io/kalico_native.rs`):

- **`MessageKind::QueryMotorState` → `MessageKind::MotorStateResponse`**, defined in
  `rust/kalico-protocol/src/messages.rs` (the `MessageKind` + `Encode`/`Decode`/`Cursor`
  home).
- Response body: `count:u8`, then per motor `[motor_index:u8, pos_q16:i32, vel_q16:i32]`.
  - Position uses the existing Q16 encoding (`encode_q16`, `rust/motion-bridge/src/dispatch.rs:107`;
    decode = `q16 as f32 / 65536.0`). 1 LSB ≈ 15.3 µm — adequate for display and diagnostics.
  - Velocity in mm/s, same Q16 encoding (range ±32k mm/s, fits).
- MCU handler reads, per configured motor, the executor's **last-tick cached** position
  and velocity (`tick_caches.p_prev` / `tick_caches.v_prev`, `rust/runtime/src/stepping_state.rs:125`,
  refreshed every ISR tick via `get_position_and_velocity()` at `rust/runtime/src/motion_core.rs:13`).
  Rationale: the cache is at most one tick old (tens of µs — effectively "now") and a
  command-context read of it avoids racing the ISR's traversal of the live piece ring.
  When idle, the last-tick cache equals the settled endpoint.
- `motor_index` is the engine's per-MCU motor/axis index (0..`MAX_AXES`=8). The host maps
  `motor_index` → rail via the oid↔index binding established at `configure_axis`.

### 2. EtherCAT position + velocity path

`EcTelemetry.position_actual` (encoder counts, `rust/kalico-ethercat-rt/src/ffi.rs:13`) is
read every DC cycle in the RT endpoint (`rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs:591`)
but never surfaced to the host. We add the surfacing and add native velocity:

- **Velocity (new):** map CiA-402 object **`606Ch` Velocity actual value** (RO, I32,
  TPDO-mappable per the A6-EC manual) into the drive's TPDO. This touches:
  - the C EtherCAT master's PDO map (adds the `606Ch` entry),
  - `EcTelemetry` (`ffi.rs:13`) — add `velocity_actual: i32`,
  - the capture buffer layout (`rust/kalico-ethercat-rt/src/capture.rs` — new offset
    alongside `OFF_POSITION_ACTUAL`),
  - a new FFI getter `ec_rt_get_velocity_actual()`.
  - **Unit:** `606Ch` is in the drive's velocity unit (rpm-based on A6-EC). Convert to
    mm/s as `(rpm / 60) × rotation_distance`. The exact unit (rpm vs counts/s vs 0.1 rpm)
    is verified against the live drive during implementation; the conversion is a single
    scalar either way.
- **Surfacing:** the endpoint converts counts→mm (reverse of `CountMap::target_counts`,
  `rust/kalico-ethercat-rt/src/scale.rs`: `mm = origin_mm + (actual - origin_counts) / counts_per_mm`)
  and rpm→mm/s, then pushes the latest (pos_mm, vel_mm_s, host-stamp) sample to the bridge
  over the existing endpoint↔bridge socket. The bridge keeps the latest sample per node.
- For EtherCAT, "blocking fresh" and "cached" collapse to the same value: the DC loop runs
  at ~kHz, so the latest pushed sample is already sub-ms fresh; there is no per-request
  round-trip to force.

### 3. Bridge: unified per-axis query + cache

The bridge hides the serial/EtherCAT split behind one interface:

- serial-stepper motor → `kalico_call(QueryMotorState)` to that motor's MCU;
- EtherCAT motor → read the latest pushed telemetry sample.

Two pyo3 methods over the same data:

- **`live_motor_positions()`** — non-blocking; returns the cached snapshot (per-motor
  pos_mm, vel_mm_s, host-stamp). Feeds `motion_report`.
- **`query_motor_positions()`** — blocking; forces a fresh `QueryMotorState` round-trip to
  each serial MCU (EtherCAT returns its latest sample). Feeds `GET_POSITION`.

A background pull loop refreshes the cache on a fixed cadence (see §6).

### 4. Host: assembly, motion_report, GET_POSITION

- **First-motor-per-rail assembly.** Build the `stepper_positions` dict using only the
  **first motor of each rail**, then call `kin.calc_position(dict)` →
  cartesian X/Y/Z. Because the dict holds exactly one motor per rail, the existing
  rail-averaging in `calc_position` (`klippy/motion_kinematics.py:217`) degenerates to
  "first motor" with no change to `calc_position` semantics. CoreXY still receives both
  rail primaries (a, b) and mixes correctly; dual-Z receives only the first Z. This
  realizes "if an axis has multiple motors, use the first one."
- **`motion_report`** (`klippy/extras/motion_report.py`): `get_status` returns the cached
  snapshot assembled into `live_position` (Coord x,y,z,e), `live_velocity`, and
  `live_extruder_velocity`. This is the bracketed `[…]` value in Mainsail. The zero stub
  is replaced by a cache read (non-blocking — never does I/O in `get_status`).
- **`GET_POSITION`** (`klippy/extras/gcode_move.py:305`): the `mcu` / `stepper` /
  `kinematic` rungs are filled from the blocking `query_motor_positions()` (replacing the
  `0.0` stub at `stepper.py:158` and the missing `get_mcu_position`); `toolhead` / `gcode`
  rungs stay commanded. Divergence between the measured and commanded rungs is the
  diagnostic signal the command exists to surface.
- **Untouched:** M114, `toolhead.position`, `gcode_move.gcode_position` (commanded).

### 5. Velocity → cartesian

Per-motor velocities run through the **same** kinematic combination as positions (valid
because the Cartesian/CoreXY maps are linear), giving cartesian (vx, vy, vz):

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
    structured log event (`kalico_log_emit`) and serves the last-known sample with its
    host-stamp (staleness is visible via the timestamp). This is a deliberate, scoped
    exception to fail-loud: a flickering Mainsail readout is not safety-critical, and
    killing a print over a missed status poll is worse than a stale number.

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
- **End-to-end:** via the kalico-sim simulator where feasible (live position non-zero
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
| New protocol message | `rust/kalico-protocol/src/messages.rs` |
| Existing R/R model | `src/kalico_dispatch.c:273`, `rust/kalico-host-rt/src/host_io/kalico_native.rs` |
| Executor pos/vel cache | `rust/runtime/src/stepping_state.rs:125`, `rust/runtime/src/motion_core.rs:13` |
| Q16 codec | `rust/motion-bridge/src/dispatch.rs:107` |
| EtherCAT telemetry struct | `rust/kalico-ethercat-rt/src/ffi.rs:13` |
| EtherCAT counts↔mm | `rust/kalico-ethercat-rt/src/scale.rs` |
| EtherCAT capture layout | `rust/kalico-ethercat-rt/src/capture.rs:25` |
| EtherCAT DC read site | `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs:591` |
