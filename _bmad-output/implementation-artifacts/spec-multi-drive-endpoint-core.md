---
title: 'Multi-drive EtherCAT — endpoint N-slave core (Phase 1)'
type: 'feature'
created: '2026-06-26'
status: 'done'
baseline_commit: '7d4bc7d51c761f6826d8532915d0c3cce6d7bca8'
context: ['{project-root}/CLAUDE.md', '{project-root}/_bmad-output/project-context.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The `ethercat-rt` endpoint, its FFI, and the IgH C backend are hardwired to a single CiA-402 slave. One chain (one NIC = one IgH master = one endpoint process) can drive exactly one servo. This blocks running multiple drives on one chain.

**Approach:** Make one endpoint bring up and run N slaves on its chain in one DC domain, each addressed by a slave index threaded through the C backend, the FFI, and the endpoint binary. Drives are configured by repeated per-slave CLI params; motion is routed to each via the wire protocol's existing `PushPieces.axes[]` per-axis path. This is the foundation; the host config (`ethercat_chain_index`) and per-drive SDO/limits/torque/homing addressing are deferred Phases 2–3 (see deferred-work). N=1 must stay byte-identical to today.

## Boundaries & Constraints

**Always:**
- N=1 is a zero-behavior-change special case: single-drive bring-up, DC timing, motion, and telemetry identical to pre-change.
- One IgH master, one DC grid, one domain — all slaves' PDO entries packed into the single shared process image; one `ec_rt_cycle` per tick covers the whole domain.
- All slaves match the same `VENDOR_ID`/`PRODUCT_CODE`; bring-up fails loudly (clear error naming the slave index) if a configured position is absent or mismatched.
- Slave index = topological chain position (IgH `SLAVE_POS`); the endpoint receives the list of positions + per-slave mechanical params at startup.
- `EcTelemetry`/`ec_telemetry_t` layout + size assert unchanged — telemetry is read one slave per call (indexed).

**Ask First:**
- The per-slave param transport: this spec proposes repeated CLI groups (`--slave <pos> --counts-per-mm … --rotation-distance …`). Confirm before building the parser.
- Bench verification with 2+ real drives (manual, user-run — do not flash or issue gcode).
- Any change to single-drive DC timing or the `ec_rt_*` ABI beyond adding a slave-index arg.

**Never:**
- Do NOT change the host (klippy) or the motion-engine bridge here — no `claim_ethercat_node`/spawn/`servo_*` changes. That is Phase 2; keeping it out is what makes this CI-green standalone.
- Do NOT add a slot index to the single-drive-targeting wire messages (SdoRead/Write, SetTorque, SetDriveLimits, SeedServoHome, arm-sensorless) — they stay single-target (slot 0) here; per-slot addressing is Phase 2. The motion path (`PushPieces.axes[]`, `MotorStateResponse.motors[]`, `status_heartbeat retired_counts[]`) is already vector-shaped and IS used.
- Do NOT add station-alias or vendor:product binding (position only). Do NOT support multiple motors on one axis. Do NOT spread a chain across processes.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Two slaves configured | `--slave 1 …  --slave 2 …` | Both reach OP; `PushPieces` with two axes drives both rings; coordinated move runs both | N/A |
| Single slave (default) | one `--slave` group (or legacy single args) | Identical to today | N/A |
| Configured position absent | `--slave 3` but chain has 2 slaves | Bring-up fails before OP | `EC_RT_ERR_NO_SLAVES`-class error naming index 3; endpoint exits loudly |
| Wrong drive at position | slave at index 2 vendor/product mismatch | Bring-up fails | loud error naming slave index 2 |
| One slave faults | slave 2 raises drive error mid-run | All slaves parked; fault surfaced with its slave index | existing fault path, per-slave error code |
| QueryMotorState | N slaves configured | Response carries N `MotorSample`s (one slot each) | N/A |

</frozen-after-approval>

## Code Map

- `rust/ethercat-rt/csrc/libecrt_igh.c` -- the core single-slave assumption. Per-slave globals → arrays indexed by slot: `g_sc[]`, `g_al_req[]`, `g_tx[]`, `g_enabled[]`/`g_activated[]`, the input offsets (`i_error_code`,`i_statusword`,`i_position_actual`,`i_velocity_actual`,`i_torque_actual`,`i_following_error`,`i_tp_status`,`i_tp1_pos`,`i_tp2_pos`,`i_digital_inputs`) and output offsets (`o_controlword`,`o_target`,`o_touch_probe`,`o_phys_outputs`,`o_velocity_offset`,`o_torque_offset`). `SLAVE_POS`/`SLAVE_ALIAS` → per-slot values. `g_master`, `g_domain`/`g_pd`, DC grid (`g_cycle_ns`/`g_ts`) stay singular. `flush_outputs` + the per-slave `ec_rt_*` gain a slot index.
- `rust/ethercat-rt/csrc/libecrt.h` -- add `int slave_idx` to per-slave decls (cycle, enable, set/get_*, telemetry, read/write_limits, sdo_read/write, run_homing, park_cycle, al_status, disable); `bringup_preop` learns slave count + positions; bring-up/shutdown stay global.
- `rust/ethercat-rt/src/ffi.rs` -- mirror the header; keep `EcTelemetry` + 32-byte assert.
- `rust/ethercat-rt/src/curves.rs` -- `NUM_AXES = 1`/`EC_AXIS_IDX = 0` → runtime slave count; one `AxisRing` per slot.
- `rust/ethercat-rt/src/cli.rs` (new) -- `SlaveCfg` + `parse_slaves` for the repeated `--slave` CLI groups, plus `EC_RT_MAX_SLAVES` (mirrors the C `#define`). In the lib (not the hw-gated binary) so the parsing/validation is CI-unit-tested.
- `rust/ethercat-rt/src/bin/ethercat-rt.rs` -- calls `cli::parse_slaves`; the per-slot motion state becomes `rings: Vec<AxisRing>` + `cmaps: Vec<Option<CountMap>>`; bring-up loops slaves (positions + per-slave limits); main DC loop loops slots for setpoint/telemetry/dynamics (two passes: sample all slots, then `model.torque_ff(slot, &all_acc, &all_vel)` with the full coupled vectors), single `ec_rt_cycle`; `PushPieces.axes[i]` → `rings[i]`; enable/disable loop all slots; `QueryMotorState` populates every slot; `status_heartbeat` passes all rings' retired counts. The torque gate, buzz, seed-home, sensorless, and capture stay **node-level / slot-0** (the frozen "single-target messages stay slot 0").
- `rust/ethercat-rt/src/wire.rs` -- added `motor_state_response_frame_multi` (N `MotorSample`s) and `push_pieces_response_frame_multi` (one `AxisDiag` per pushed axis). `status_heartbeat_frame` already took a `retired_counts` slice. No single-target message-shape break (Phase 2).
- `rust/ethercat-rt/src/claim.rs` -- added `all_slaves_reply(n, ..)` for the N-slave claim handshake (identical to `single_slave_reply(1, ..)` at n=1).
- Unchanged (already per-axis or node-level by the frozen boundary): `torque.rs` (one `TorqueGate`), `scale.rs` (`CountMap` instantiated per slot in the binary), `capture.rs` (slot-0 single-drive), `dynamics.rs` (already takes an axis index), `sensorless.rs`/`buzz.rs`/`seed_home.rs` (slot 0), `claim.rs`/`clock.rs`/`server.rs` (slave-agnostic).

## Tasks & Acceptance

Ordered by dependency.

**Execution:**
- [x] `csrc/libecrt_igh.c` -- per-slave `slave_t` array + one shared domain registering all slaves' PDO entries; `bringup_preop` takes positions + count, configures each slave, fails loudly (`EC_RT_ERR_NO_SLAVES`/`EC_RT_ERR_TOO_MANY_SLAVES`) naming the offending slot; `bringup_finish` walks every slave to OP then parks each; cyclic + accessor `ec_rt_*` index by slot; per-slave enable/disable/homing hold the other slaves. C parses clean under `-fsyntax-only` (only Linux-platform-API symbols unavailable on the mac toolchain).
- [x] `csrc/libecrt.h` + `src/ffi.rs` -- threaded `slave: c_int` through per-slave `ec_rt_*`; `bringup_preop` takes `(positions, num_slaves)`; added `EC_RT_ERR_TOO_MANY_SLAVES`/`EC_RT_ERR_BAD_SLAVE_IDX` + `EC_RT_MAX_SLAVES`; kept `EcTelemetry` layout/assert.
- [x] `src/curves.rs` -- dropped `NUM_AXES`/`EC_AXIS_IDX`; `AxisRing::with_slot(slot)` stores its slot (used for fault attribution); `new()` delegates at slot 0.
- [x] `src/cli.rs` (new) + `src/cli/tests.rs` -- `parse_slaves` for repeated `--slave` groups (legacy single-drive fallback at position 0); rejects duplicate position, orphan flag, missing value, non-integer position, over-cap. 9 unit tests.
- [x] `src/bin/ethercat-rt.rs` -- `cli::parse_slaves`; `rings`/`cmaps` vecs; bring-up positions + per-slave limits; two-pass motion loop (sample all → coupled torque FF → stage per slot); `PushPieces.axes[i]`→`rings[i]` with out-of-range guard; enable/disable loop all slots; per-slot drive-error scan; `QueryMotorState`/heartbeat over all slots; WKC expected `3*num_slaves`. Gate/buzz/seed-home/sensorless/capture stay slot-0.
- [x] `src/wire.rs` + `src/claim.rs` -- `motor_state_response_frame_multi`, `push_pieces_response_frame_multi`, `all_slaves_reply`; stub binaries + integration tests de-`NUM_AXES`'d.
- [x] tests -- `cli/tests.rs` (config edge cases incl. duplicate/absent-handling at parse layer), `wire/tests.rs` (`push_pieces_response_multi_echoes_every_axis`, `motor_state_response_multi_carries_one_sample_per_slot`). Bring-up edge cases (absent/mismatched slave) are hardware paths verified on the Pi. Full suite: 177 pass.

**Acceptance Criteria:**
- Given one `--slave` group (or legacy single args), when the endpoint runs, then bring-up, DC timing, motion, and telemetry are identical to pre-change (N=1 regression).
- Given two `--slave` groups for positions 1 and 2 present on the chain, when `PushPieces` carries two axes, then both rings advance and both drives track their setpoints in one DC domain.
- Given a configured position with no matching slave, when bring-up runs, then it fails before OP with an error naming that slave index, and the endpoint exits non-zero.
- Given N slaves, when `QueryMotorState` is issued, then the response carries N `MotorSample`s, one per slot.
- Given `./scripts/ci.sh quick`, when run, then green (this phase touches no klippy — `py` not required).

## Design Notes

**Slot = position-list ordinal.** The endpoint is given an ordered list of slave positions at startup (one `--slave <pos>` group each, with that drive's `counts-per-mm`/`rotation-distance`/optional limits). Slot `i` (the endpoint's 0-based ring/array index, selected by `PushPieces.axes[].axis_idx`) drives the slave at `positions[i]`. The host (Phase 2) is responsible for ordering groups so slot order matches the axis order it sends in `PushPieces`; this phase just honors the order given.

**Shared domain, packed offsets.** Keep the static `rx_entries`/`tx_entries`/`syncs` PDO definition; register it per slave config into the one domain. `ecrt_domain_data` returns one buffer; each slot's `i_*`/`o_*` offsets index into its partition. Domain-size check becomes the sum over slots.

**Per-slave bring-up to OP.** Loop `ecrt_master_slave_config` + PDO/SDO config + `create_reg_request` per position in `bringup_preop`; in `bringup_finish` poll each slot's `.operational` and run each CiA-402 park independently; all must reach OP before success.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p ethercat-rt` -- expected: pass (default features, no C).
- `./scripts/ci.sh quick` -- expected: green.
- `cargo build -p ethercat-rt --features hw --bin ethercat-rt` on the Pi -- expected: compiles + links libethercat.

**Manual checks (Pi/bench, user-run — Ask First):**
- One drive, single `--slave` group (or legacy args): homes/moves exactly as before.
- Two drives at positions 1 & 2: both reach OP; a two-axis `PushPieces` (via `ec-test-client`) advances both rings and both drives track.

## Suggested Review Order

**The chain-layout contract (entry point)**

- Per-slave C state: `slave_t` array, one shared master/domain/DC grid — the model everything else builds on.
  [`libecrt_igh.c:49`](../../rust/ethercat-rt/csrc/libecrt_igh.c#L49)

- Bring-up takes `(positions, num_slaves)`, configures each slave, fails loudly naming the slot.
  [`libecrt_igh.c:318`](../../rust/ethercat-rt/csrc/libecrt_igh.c#L318)

- The FFI contract: `slave` index threaded through every per-slave `ec_rt_*`; `EcTelemetry` unchanged.
  [`libecrt.h:38`](../../rust/ethercat-rt/csrc/libecrt.h#L38)
  [`ffi.rs:28`](../../rust/ethercat-rt/src/ffi.rs#L28)

**Cyclic exchange (highest risk)**

- One DC cycle stages every slave's controlword over the shared domain; `flush_outputs` loops all slots.
  [`libecrt_igh.c:505`](../../rust/ethercat-rt/csrc/libecrt_igh.c#L505)
  [`libecrt_igh.c:158`](../../rust/ethercat-rt/csrc/libecrt_igh.c#L158)

- Two-pass motion loop: sample all slots first (coupled torque FF needs the full per-axis vectors), then stage.
  [`ethercat-rt.rs:882`](../../rust/ethercat-rt/src/bin/ethercat-rt.rs#L882)

- PushPieces routing: single-drive ignores the host's global axis_idx (N=1 regression-safety); multi uses it as the slot.
  [`ethercat-rt.rs:365`](../../rust/ethercat-rt/src/bin/ethercat-rt.rs#L365)

**Config & wire surface**

- CLI chain layout: repeated `--slave` groups, legacy single-drive fallback, fail-loud validation.
  [`cli.rs:35`](../../rust/ethercat-rt/src/cli.rs#L35)
  [`ethercat-rt.rs:122`](../../rust/ethercat-rt/src/bin/ethercat-rt.rs#L122)

- New vector-shaped frame builders (motion path only; single-target messages untouched).
  [`wire.rs:273`](../../rust/ethercat-rt/src/wire.rs#L273)
  [`wire.rs:369`](../../rust/ethercat-rt/src/wire.rs#L369)

- N-slave claim handshake (≡ single at n=1); per-slot AxisRing carries its slot for fault attribution.
  [`claim.rs:73`](../../rust/ethercat-rt/src/claim.rs#L73)
  [`curves.rs:55`](../../rust/ethercat-rt/src/curves.rs#L55)

**Tests (peripherals)**

- CLI parsing/validation edge cases (duplicate, orphan flag, over-cap, legacy fallback).
  [`cli/tests.rs:1`](../../rust/ethercat-rt/src/cli/tests.rs#L1)

- Multi-slot wire frames round-trip (per-axis echo, one sample per slot).
  [`wire/tests.rs:60`](../../rust/ethercat-rt/src/wire/tests.rs#L60)
