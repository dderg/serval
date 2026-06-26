---
title: 'Multi-drive EtherCAT — host config + per-drive claim (Phase 2)'
type: 'feature'
created: '2026-06-27'
status: 'done'
baseline_commit: '1ee86fa772a6f1533182a4c503e835e39891f5c8'
context: ['{project-root}/CLAUDE.md', '{project-root}/_bmad-output/project-context.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Phase 1 made the endpoint drive N slaves on one chain via per-`--slave` CLI, but the host still claims exactly one drive per `[ethercat_node]` and every per-drive op (SDO, torque, limits, seed-home, sensorless-arm) targets an implicit single drive (slot 0). There is no way to say which drive serves which motor on a shared chain, and the read-back paths (`retired_counts`, motor samples) attribute everything to axis 0.

**Approach:** Add a `[motor] drive:servo` field `ethercat_chain_index` (the topological slave position). The node gathers all its servo rails, sorts them by chain_index → slot order, spawns the endpoint with N `--slave` groups, and claims them as one mcu with an authoritative slot↔global-axis map. Thread an explicit `slot: u8` through every single-target wire message and the bridge/Python methods so each per-drive op addresses its drive. Fix the slot↔axis read-back so per-slot counts/samples land on the right global axis.

## Boundaries & Constraints

**Always:**
- Regression-safe at N=1: a single-drive node behaves identically (slot defaults to 0; one `--slave` group is equivalent to the legacy global args).
- One authoritative slot↔global-axis map, established at claim time (drives sorted by `ethercat_chain_index` ascending; slot i = i-th drive). Both PushPieces routing and per-slot read-back (`retired_counts`, `MotorStateResponse`) use it — no positional `enumerate()==global-axis` shims left.
- Fail loudly: duplicate or out-of-range `ethercat_chain_index` on one node, or chain_index with no matching slave, raises a config/claim error — do not silently renumber.
- Wire changes stay in the manual `Encode`/`Decode` style already used in `mcu-protocol/src/messages.rs`; keep the host and endpoint in lockstep (both rebuilt from one repo per the bench flow).

**Ask First:**
- Bench verification on the Pi: N=1 regression (X axis homes/moves unchanged) and a 2-drive chain (`ec-test-client` + a real 2-drive bench if available). The user runs this; do not flash or issue gcode.

**Never:**
- Do NOT implement per-drive sensorless trip routing or the multi-drive bring-up docs — that is Phase 3 (the `slot` on `ArmSensorlessEndstop` is wired here, but per-slave torque-trip detection stays single-detector).
- Do NOT add dual-motor-per-axis (one lane still references one motor; N lanes/axes share one node).
- Do NOT change the `--slave` CLI shape or `EC_RT_MAX_SLAVES` (8) from Phase 1.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Single drive | one `[motor]` on node, no/`=1` chain_index | one `--slave 1` group; slot 0; identical to Phase 1 | N/A |
| Two drives | two `[motor]` on node, chain_index 1 & 2 | two `--slave` groups; slot 0→axis(idx1), slot 1→axis(idx2); each SDO/torque targets its slot | N/A |
| Duplicate index | two motors both chain_index 1 | claim aborts | config/claim error naming both motors |
| Out-of-range index | chain_index 9 (> EC_RT_MAX_SLAVES) | claim aborts | config error |
| SERVO_PARAM multi | `SERVO=<name>` on a multi-drive node | resolves (node, slot) for that motor; SDO targets its slot | error if name unknown/ambiguous |
| Per-slot readback | endpoint sends `retired_counts`/motors in slot order | each lands on `cfg.axes[slot]` global axis | N/A |

</frozen-after-approval>

## Code Map

- `klippy/extras/servo_axis.py` -- `ServoRail.__init__` (~84) parse `ethercat_chain_index` (int, default 1, minval 1); add `get_chain_index()` (~172).
- `klippy/extras/ethercat_node.py` -- `_find_rail`→`_find_rails` (54), multi-drive `_claim` (70) sorts rails by chain_index, builds per-drive list, calls multi-drive claim; `_push_drive_params`(rail,slot) (129) + `_poll_drive_fault`(slot) (118); fault msg names the slave.
- `klippy/motion_engine.py` -- `claim_ethercat_node` (145) takes a per-drive list; `set_drive_limits`/`restore_drive_limits`/`arm_sensorless_endstop`/`set_torque`/`sdo_read`/`sdo_write`/`take_drive_fault` (173–216) gain `slot` (default 0).
- `klippy/extras/servo_param.py` -- `_resolve_node` (135) → resolve `(node, slot)`; `cmd_SERVO_PARAM` (159) passes slot to `sdo_read/write`.
- `klippy/extras/servo_capture.py` -- `_resolve_node` (32) resolve `(node, slot)`; lift the "multi-servo capture … not implemented" guard (44–48); pass slot.
- `klippy/motion_kinematics.py` -- `_build_servo_lane` (172) unchanged (one motor/lane) but confirm N lanes may share one node.
- `rust/motion-engine/src/bridge.rs` -- `spawn_ethercat_endpoint` (393) build N `--slave` groups; `claim_ethercat_node` (954) take per-drive list + record slot↔axis map; `set_drive_limits`/`restore_drive_limits`/`arm_sensorless_endstop`/`set_torque`/`sdo_read`/`sdo_write` (1016–1360) gain `slot`; `place_motor_response` (135) use `MotorSample.slot`→`cfg.axes[slot]`; `retired_counts` consumers (3011/3091) map slot→`cfg.axes[slot]`.
- `rust/motion-engine/src/{servo_sdo.rs,servo_torque.rs}` -- `send_sdo_read/write`, `send_set_torque`, `send_drive_limits`, `send_restore_drive_limits`, `send_arm_sensorless_endstop` gain `slot`.
- `rust/mcu-protocol/src/messages.rs` -- add `slot: u8` (+1 byte Encode/Decode) to `SetTorque` (375), `SdoRead` (422), `SdoWrite` (469), `SetDriveLimits` (687), `RestoreDriveLimits` (728), `SeedServoHome` (760), `ArmSensorlessEndstop` (643).
- `rust/ethercat-rt/src/mailbox.rs` -- add `slot` to `WriteLimits`, `SeedHomeSetup` (33–84).
- `rust/ethercat-rt/src/bin/ethercat-rt.rs` -- retire slot-0 shims: dispatch each request to `request.slot`; `write_limits` closure (~306), `RestoreDriveLimits` `run_limits[slot]` (~515), `SeedServoHome` `counts_per_mm[slot]` (~548), FFI `ec_rt_*` slot args; PushPieces routing consumes slot directly (drop `num_slaves==1?0:axis_idx`).

## Tasks & Acceptance

**Execution:**
- [x] `rust/mcu-protocol/src/messages.rs` -- add `slot: u8` to the 7 single-target messages + Encode/Decode -- per-drive addressing on the wire.
- [x] `rust/mcu-protocol/src/messages/tests.rs` (or sibling) -- round-trip encode/decode tests for the 7 messages with non-zero slot -- fail-loud wire coverage.
- [x] `rust/ethercat-rt/src/mailbox.rs` -- thread `slot` into `WriteLimits`/`SeedHomeSetup` requests.
- [x] `rust/ethercat-rt/src/bin/ethercat-rt.rs` -- retire all slot-0/`axis_idx==slot` shims; dispatch by `request.slot`; per-slot FFI calls -- one authoritative slot map endpoint-side.
- [x] `rust/motion-engine/src/{servo_sdo.rs,servo_torque.rs}` -- add `slot` to the message builders.
- [x] `rust/motion-engine/src/bridge.rs` -- `spawn`/`claim` per-drive list + slot↔axis map; thread `slot` through per-drive methods; fix `place_motor_response` + `retired_counts` to map slot→`cfg.axes[slot]`.
- [x] `klippy/motion_engine.py` -- mirror per-drive claim list + `slot` params (default 0).
- [x] `klippy/extras/servo_axis.py` -- `ethercat_chain_index` field + `get_chain_index()`.
- [x] `klippy/extras/ethercat_node.py` -- `_find_rails`, multi-drive claim (sorted, validated), per-slot param push + fault poll.
- [x] `klippy/extras/{servo_param.py,servo_capture.py}` -- `(node, slot)` resolution; lift the multi-servo-capture guard.
- [x] `test/` -- host test: a 2-drive node validates (distinct/range), claim builds the right `--slave` arg list and slot map; duplicate index raises.

**Acceptance Criteria:**
- Given two `[motor] drive:servo` on one node with `ethercat_chain_index` 1 and 2, when the node claims, then the endpoint is spawned with two `--slave` groups (positions 1,2) and slot 0/1 map to the two motors' global axes.
- Given two motors on one node with the same `ethercat_chain_index`, when claiming, then a config/claim error fires naming the conflict (no silent renumber).
- Given a single-drive node (no `ethercat_chain_index`), when claiming, then behavior is byte-identical to Phase 1 (slot 0, one slave at position 1).
- Given `SERVO_PARAM SERVO=<motor>` on a multi-drive node, when run, then the SDO targets that motor's slot only.
- Given the endpoint emits `retired_counts`/motor samples in slot order, when the bridge consumes them, then each value lands on `cfg.axes[slot]`, not on `enumerate()` index.
- Given `./scripts/ci.sh quick` and `./scripts/ci.sh py`, when run, then green.

## Spec Change Log

- **2026-06-27 — step-04 review patches (no loopback; all patch/defer/reject):**
  - Endpoint now fails loud with a clean `-309` response (instead of the C `abort()` that would kill all drives) on an out-of-range `slot` for `SetDriveLimits`, `ArmSensorlessEndstop`, `SdoRead`, `SdoWrite` — matching the existing `RestoreDriveLimits`/`SeedServoHome` guards. (`ethercat-rt.rs`)
  - `place_motor_response` (ethercat path) now indexes `cfg.axes[m.slot]` using the sample's own `slot` field rather than a positional `zip`.
  - `SERVO_CAPTURE` now fails loud on a node driving >1 servo (capture is slot-0-only in the endpoint; per-drive capture is Phase 3). (`servo_capture.py`, new `EtherCatNode.get_drive_count`)
  - Added `slot_for_axis` unit test (hit/miss); softened the `endpoint_args` N=1 doc-comment to "behaviorally identical" (parser is order-independent).
  - Deferred to Phase 3: single-armed-sensorless-drive limitation; per-drive capture. Rejected: heartbeat retired-counts map (two reviewers confirmed `cfg.axes` is sorted = slot order, so correct).

- **2026-06-27 — implementation refinements (investigation findings):**
  - `SetTorque` is node-global, not per-slot — the endpoint enables/disables every slave under one torque gate and `register_torque_enable` energizes the whole machine. `slot` was therefore NOT added to `SetTorque`; the other six messages keep it. (Adjusts the "7 messages" count to 6.)
  - **Position base:** `ethercat_chain_index` is 1-based (default 1, minval 1); endpoint slave position = `chain_index - 1` (IgH 0-based ring `SLAVE_POS`). Default single drive → position 0, byte-identical to Phase 1.
  - **N=1 spawns the legacy CLI form** (no `--slave`), identical to Phase 1; N>1 spawns `--slave <pos> --axis <global_axis> --counts-per-mm … --rotation-distance … [limits]` per drive in slot order.
  - **Authoritative slot↔axis map** established at claim: drives sorted by **global axis** ascending → slot i; this matches `cfg.axes` (also sorted), so `place_motor_response` stays correct and the retired-counts path is fixed to `cfg.axes[slot]`. Stored per-mcu as `ethercat_slot_axes`. The endpoint learns each slave's global axis via the new per-slave `--axis` flag and routes PushPieces by it (N>1); N=1 keeps the `slot 0` shim. The endpoint still echoes the global `axis_idx` in responses, so transit-diag matching is unchanged.
  - **Drive fault stays node-global** (`StatusHeartbeat.fault_code` is one value per node); `take_drive_fault(mcu_handle)` unchanged. Per-slave fault attribution is Phase 3. The Python shutdown message names the node.
  - Per-drive `velocity_ff`/`dynamics_profile`/`ff_torque_clamp` stay node-global (endpoint CLI has no per-slave FF); the claim validates they are identical across a node's drives and fails loudly otherwise.

## Design Notes

**Slot↔global-axis map (the single source of truth).** At claim, the node lists its rails and sorts them ascending by **global (lane) axis**; slot `i` = the i-th rail in that order, its slave position = `chain_index - 1`. This sort matches `cfg.axes` (built sorted at `init_planner`), so the slot↔axis correspondence is identical on both the claim side (`ethercat_slot_axes`) and the dispatch side (`cfg.axes`). The endpoint keys everything by slot (Phase 1 `rings[slot]`); it routes incoming PushPieces (tagged with the global `axis_idx`) through a `--axis`-built global→slot map for N>1, and the bridge maps endpoint→host `retired_counts`/`MotorStateResponse` (slot-ordered) back via `cfg.axes[slot]`. This retires the Phase-1 `enumerate()==global-axis` retired shim (the 2026-06-26 defer); the `num_slaves==1→slot 0` shim is kept for the N=1 path.

**Slot default = 0** on every new wire field / Python param keeps single-drive callers (and any not-yet-updated path) on the existing drive, so the change is incremental and N=1-safe.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p mcu-protocol -p ethercat-rt -p motion-engine` -- expected: pass (incl. new slot round-trip tests).
- `./scripts/ci.sh quick` -- expected: green (rust-test, clippy `-D warnings`, fmt, ruff, watchdog-canary).
- `./scripts/ci.sh py` -- expected: green (touches `klippy/`).
- `./scripts/ci.sh rust-ethercat-hw` -- expected: green (endpoint still compiles under `--features hw`).

**Manual checks (Pi/bench, user-run — Ask First):**
- N=1: X axis homes + moves unchanged.
- N=2: both drives enumerate, each motor responds to its own `SERVO_PARAM`/move; `retired_counts` attributed to the correct axes.

## Suggested Review Order

**The slot↔axis design (entry point)**

- The host claim establishes the authoritative map: drives sorted by global axis → slot order, then spawn N groups.
  [`bridge.rs:1020`](../../rust/motion-engine/src/bridge.rs#L1020)

- How the CLI args encode that order (N=1 legacy form vs N>1 `--slave`/`--axis`), position = chain_index-1.
  [`bridge.rs:435`](../../rust/motion-engine/src/bridge.rs#L435)

**Wire protocol (slot on the 6 single-target messages)**

- The `slot: u8` leading field + Encode/Decode (SdoWrite shown; SdoRead/SetDriveLimits/RestoreDriveLimits/SeedServoHome/ArmSensorlessEndstop alongside).
  [`messages.rs:472`](../../rust/mcu-protocol/src/messages.rs#L472)

**Endpoint: route + dispatch by slot (highest risk)**

- PushPieces routing: global `axis_idx` → slot via the per-slave `--axis` map (N>1); N=1 shim kept.
  [`ethercat-rt.rs:382`](../../rust/ethercat-rt/src/bin/ethercat-rt.rs#L382)

- Per-slave `--axis` parse added to the chain-layout CLI.
  [`cli.rs:14`](../../rust/ethercat-rt/src/cli.rs#L14)

- SDO bus is now slot-addressed end to end (trait → FFI); seed-home + mailbox carry slot.
  [`sdo.rs:17`](../../rust/ethercat-rt/src/sdo.rs#L17)

**Read-back mapping (closes the Phase-1 defer)**

- `retired_counts` map slot→`cfg.axes[slot]` (was `enumerate()==global-axis`).
  [`bridge.rs:3096`](../../rust/motion-engine/src/bridge.rs#L3096)

- Motor samples placed by their own `slot` field, not positional zip.
  [`bridge.rs:151`](../../rust/motion-engine/src/bridge.rs#L151)

- `finalize_homed_axis` resolves slot from the global axis via the stored map.
  [`bridge.rs:144`](../../rust/motion-engine/src/bridge.rs#L144)

**Host config + claim (user-facing)**

- New `ethercat_chain_index` motor field (1-based).
  [`servo_axis.py:86`](../../klippy/extras/servo_axis.py#L86)

- Node gathers all rails, validates (distinct/range/FF), builds the per-drive list, claims, pushes per-slot params.
  [`ethercat_node.py:112`](../../klippy/extras/ethercat_node.py#L112)

- Fail-loud chain validation.
  [`ethercat_node.py:80`](../../klippy/extras/ethercat_node.py#L80)

- PyO3 wrapper: per-drive claim list + slot params.
  [`motion_engine.py:145`](../../klippy/motion_engine.py#L145)

**Tests (peripherals)**

- Chain validation unit tests (duplicate/range/FF mismatch).
  [`test_ethercat_node.py:1`](../../test/test_ethercat_node.py#L1)

- Endpoint-arg + slot_for_axis unit tests.
  [`bridge.rs:4949`](../../rust/motion-engine/src/bridge.rs#L4949)
