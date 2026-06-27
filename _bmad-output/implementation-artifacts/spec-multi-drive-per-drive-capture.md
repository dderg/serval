---
title: 'Multi-drive EtherCAT — per-drive servo telemetry capture (Phase 3)'
type: 'feature'
created: '2026-06-27'
status: 'done'
baseline_commit: '80e11befca0e28e3ea3383192aced5b34d4e3949'
context: ['{project-root}/CLAUDE.md', '{project-root}/_bmad-output/project-context.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Servo telemetry capture is slot-0-only. The endpoint samples `ec_rt_get_telemetry(0)`, writes one `DriveSample` per record, and the `.scap` header carries one drive; the host rejects any node with >1 drive (the `(Phase 3)` guard in `servo_capture.py`). On a multi-drive chain this blocks `SERVO_CAPTURE` and everything built on it (`SERVO_FIT_DYNAMICS`, `SERVO_CALIBRATE_GAINS`, `SERVO_MEASURE_*`), and it cannot record the coupled motors of one logical move (CoreXY, shared belt).

**Approach:** Make capture record a **host-chosen list of drives**, time-aligned on the shared 1 kHz DC cycle. The host (which owns kinematics and the Phase 2 slot↔axis map) resolves which slots to capture and passes an explicit `(slot, name)` list in `StartCapture`; the endpoint samples each listed slot via `ec_rt_get_telemetry(slot)` and writes one `DriveSample` block per drive per record. The `.scap` format gains N drive blocks per record and an N-entry header `drives` array. N=1 stays byte-identical. Drop the host guard. Analysis tooling selects a drive by name.

## Boundaries & Constraints

**Always:**
- N=1 capture is **byte-identical** to today: `RECORD_SIZE == 9 + N·28` (37 at N=1), the header `drives` array is already 1-element, and drive-0 channel offsets are unchanged. Pin this with a same-bytes regression test.
- The **host owns slot selection**; the endpoint samples only the slots it is handed and never maps axis→motor.
- Capture rides the existing per-cycle "sample all slots" pass — no extra `ec_rt_get_telemetry` calls beyond the listed slots, and **no heap allocation on the DC thread**: size the per-record drive array inline to `EC_RT_MAX_SLAVES` (the `capture-io` thread does all file I/O, per the module's RT comment).
- Fail loudly: an out-of-range slot, an empty list, or a duplicate slot in `StartCapture` → reject before any file open with a clear capture error code, tested.
- Host resolution mirrors `SERVO_PARAM` (`servo_param.py:135` `_resolve_node` → `(node, slot)` via `node.get_slot_for_motor`). Per-drive `counts_per_mm` for the header comes from the endpoint's existing per-slot map, not the host.

**Ask First:**
- Any analysis-tooling drive-selection logic beyond "the named motor, defaulting to the first drive in the file" (e.g. auto-detecting the moving drive) — confirm before adding heuristics.

**Never:**
- Do NOT teach the endpoint axis→motor kinematics.
- Do NOT change the `--slave` CLI shape, `EC_RT_MAX_SLAVES` (8), or the `EcTelemetry`/`ec_telemetry_t` layout + size assert.
- Do NOT capture more than the host-listed slots ("whole-node always" was considered and rejected).
- Do NOT alter the 11 channel definitions or a drive block's internal byte layout — only repeat the 28-byte drive block N times.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Single-drive node | `SERVO_CAPTURE SERVO=motor_x`, 1-drive node | one-drive `.scap`; header + records byte-identical to pre-change | N/A |
| Multi-drive, one motor | `SERVO_CAPTURE SERVO=motor_y`, 2-drive node | capture starts (no guard); `.scap` holds only the resolved slot's telemetry | N/A |
| Coupled list (future CoreXY) | host builds `[(slotA,…),(slotB,…)]` | `.scap` has 2 time-aligned drive blocks per record; header lists both | N/A |
| Out-of-range slot | `StartCapture` lists slot ≥ num_slaves | reject; no file created | bad-list capture error code |
| Empty / duplicate list | `drives=[]` or repeated slot | reject; no file created | bad-list capture error code |
| Unknown SERVO | `SERVO=` names no servo motor | host command error; no wire message sent | `gcmd.error` |

</frozen-after-approval>

## Code Map

- `rust/ethercat-rt/src/capture.rs` -- `CaptureRecord.drive`→inline N-drive array; `CaptureConfig.drive_name`/`counts_per_mm`→per-drive `(slot,name,counts_per_mm)` list; `RECORD_SIZE`/`encode_record`/`header_json` multi-drive; add bad-list error const.
- `rust/ethercat-rt/src/bin/ethercat-rt.rs` -- `StartCapture` handler (~576) builds per-slot `CaptureConfig` from the message list (`counts_per_mm[slot]`, validate slots); DC-loop sampling (~1276) loops the listed slots calling `ec_rt_get_telemetry(slot)` instead of `(0)`.
- `rust/mcu-protocol/src/messages.rs:563` -- `StartCapture`: replace `drive_name: String` with `drives: Vec<CaptureDrive{ slot: u8, name: String }>`; mirror the `PushPieces.axes` Vec `Encode`/`Decode` (251/271).
- `rust/motion-engine/src/servo_capture.rs` -- `send_start_capture` takes the drive list.
- `rust/motion-engine/src/bridge.rs:1158` -- `start_servo_capture` signature gains the drive list (`stop_servo_capture` unchanged).
- `klippy/motion_engine.py` -- mirror `start_servo_capture(handle, path, utc, drives)` list param.
- `klippy/extras/servo_capture.py:32,74,87` -- `_resolve_node`→`(node, slot)` `SERVO_PARAM`-style; drop the `get_drive_count() > 1` guard; build `[(slot, motor_name)]`; pass to the engine.
- `scripts/servo_capture.py:246,309,356` -- read N drives from header; decode N blocks/record; select drive by name (`--drive`, default first).
- `scripts/servo_gain_report.py:130`, `scripts/servo_fit_dynamics.py` -- select the relevant drive by name from a multi-drive `.scap`.
- `docs/rewrite/ethercat-bench-bringup.md` (~220) -- document multi-drive capture + per-drive analysis.
- `rust/ethercat-rt/src/capture/tests.rs` -- multi-drive round-trip, N=1 byte-identical, bad-list rejection.

## Tasks & Acceptance

**Execution:**
- [x] `rust/mcu-protocol/src/messages.rs` -- replace `StartCapture.drive_name` with `drives: Vec<CaptureDrive{slot:u8,name:String}>`; add `Encode`/`Decode` modeled on `PushPieces.axes` -- per-drive addressing on the wire.
- [x] `rust/ethercat-rt/src/capture.rs` -- inline N-drive `CaptureRecord`/`CaptureConfig`; `RECORD_SIZE = 9 + N·28`; multi-drive `encode_record` + `header_json` (`drives` array of `{name,counts_per_mm}`); bad-list error const -- the format change.
- [x] `rust/ethercat-rt/src/bin/ethercat-rt.rs` -- start handler validates the slot list (range/dup/empty), fills `counts_per_mm[slot]`; DC-loop samples each listed slot -- replace hardcoded slot 0.
- [x] `rust/motion-engine/src/servo_capture.rs` + `bridge.rs` -- thread the drive list through `send_start_capture`/`start_servo_capture`.
- [x] `klippy/motion_engine.py` -- `start_servo_capture` list param.
- [x] `klippy/extras/servo_capture.py` -- resolve `(node, slot)`, drop the >1 guard, build the `[(slot,name)]` list, pass it down.
- [x] `scripts/servo_capture.py` + `servo_gain_report.py` + `servo_fit_dynamics.py` -- read/decode N drives; select a drive by name.
- [x] `docs/rewrite/ethercat-bench-bringup.md` -- multi-drive capture section.
- [x] `rust/ethercat-rt/src/capture/tests.rs` -- multi-drive round-trip + N=1 byte-identical + bad-list rejection (paired fail-loud tests).

**Acceptance Criteria:**
- Given a single-drive node, when `SERVO_CAPTURE` runs, then the `.scap` bytes (header + records) are identical to pre-change.
- Given a 2-drive node, when `SERVO_CAPTURE SERVO=motor_y`, then capture starts with no guard, the `.scap` holds the resolved slot's telemetry, and `SERVO_FIT_DYNAMICS AXIS=y` completes.
- Given `StartCapture` with an out-of-range, empty, or duplicate slot list, then the endpoint rejects it and writes no file.
- Given a multi-drive `.scap`, when `servo_gain_report.py`/`servo_fit_dynamics.py` run, then they select the correct drive by name.
- Given `./scripts/ci.sh quick` and `./scripts/ci.sh py`, then both green.

## Design Notes

Record layout, N drives (drive block = 28 bytes, unchanged internally):

```
[ cycle_index u64 | flags u8 | drive0 (28B) | drive1 (28B) | … | driveN-1 ]
   off 0            off 8       off 9          off 37
RECORD_SIZE = 9 + N*28      # N=1 → 37, byte-identical to today
```

Header gains a real per-drive array (already a 1-element array at N=1, so unchanged there); `channels[].offset` describe one drive block, and the reader indexes drive d at `9 + d*28 + channel.offset`:

```json
{"version":1, ..., "record_size":65,
 "drives":[{"name":"motor_a","counts_per_mm":1280.0},
           {"name":"motor_b","counts_per_mm":1280.0}],
 "channels":[{"name":"target_counts","dtype":"i32","offset":0}, ...]}
```

Wire `StartCapture` carries `drives: [{slot,name}]`; the endpoint fills each drive's `counts_per_mm` from its own per-slot map. Host fills the list from `SERVO=<motor>` (length 1 today); a future CoreXY axis expands to multiple slots with no format change.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p ethercat-rt -p mcu-protocol -p motion-engine` -- expected: green (multi-drive + N=1 byte-identical + bad-list tests pass).
- `./scripts/ci.sh quick` -- expected: green (ruff, rust-test, clippy -D warnings, fmt, watchdog-canary).
- `./scripts/ci.sh py` -- expected: green (touches `klippy/`).

**Manual checks:**
- On a 2-drive bench node: `SERVO_CAPTURE SERVO=motor_y` produces a `.scap` whose header lists the resolved drive; `SERVO_FIT_DYNAMICS AXIS=y END=180` completes and emits a dynamics profile.

## Suggested Review Order

**Design intent — start here**

- Host validates the slot list, gates `capture_slots` on an accepted start, samples per-slot
  [`ethercat-rt.rs:577`](../../rust/ethercat-rt/src/bin/ethercat-rt.rs#L577)

**Wire protocol (the foundation everything links against)**

- `CaptureDrive{slot,name}` + `StartCapture.drives` replace the single `drive_name`
  [`messages.rs:566`](../../rust/mcu-protocol/src/messages.rs#L566)
- Encode/Decode modeled on `PushPieces.axes`; rejects empty + duplicate slots
  [`messages.rs:578`](../../rust/mcu-protocol/src/messages.rs#L578)

**`.scap` format — the load-bearing decision**

- Inline `[DriveSample; MAX_DRIVES]` keeps `CaptureRecord` `Copy` (no DC-thread alloc)
  [`capture.rs:64`](../../rust/ethercat-rt/src/capture.rs#L64)
- `record_size = 9 + N*28` and N back-to-back blocks; N=1 stays byte-identical
  [`capture.rs:125`](../../rust/ethercat-rt/src/capture.rs#L125)
- Header emits the N-entry `drives` array
  [`capture.rs:162`](../../rust/ethercat-rt/src/capture.rs#L162)
- Fail-loud validation: list shape + the testable range helper
  [`capture.rs:141`](../../rust/ethercat-rt/src/capture.rs#L141)

**Endpoint sampling**

- DC loop samples each listed slot via `ec_rt_get_telemetry(slot)`
  [`ethercat-rt.rs:1300`](../../rust/ethercat-rt/src/bin/ethercat-rt.rs#L1300)
- `claimed()` gate so a rejected start can't change a running capture's stride
  [`ethercat-rt.rs:608`](../../rust/ethercat-rt/src/bin/ethercat-rt.rs#L608)

**Bridge + host**

- Bridge `start_servo_capture` threads the drive list to the endpoint
  [`bridge.rs:1158`](../../rust/motion-engine/src/bridge.rs#L1158)
- Host resolves `SERVO=<motor>` → `(node, slot)` (mirrors `SERVO_PARAM`); guard deleted
  [`servo_capture.py:26`](../../klippy/extras/servo_capture.py#L26)

**Analysis tooling**

- Reads N drives, selects by name, fails loudly on a drive-less header
  [`servo_capture.py:44`](../../scripts/servo_capture.py#L44)

**Docs**

- Multi-drive capture + per-drive analysis note
  [`ethercat-bench-bringup.md:252`](../../docs/rewrite/ethercat-bench-bringup.md#L252)

**Tests (peripherals)**

- N=1 byte-identical pin + range-helper unit test
  [`capture/tests.rs:99`](../../rust/ethercat-rt/src/capture/tests.rs#L99)
- N=2 end-to-end distinct blocks + rejected-start stride regression
  [`capture_lifecycle.rs:297`](../../rust/ethercat-rt/tests/capture_lifecycle.rs#L297)
- Motor→slot resolution (each motor distinct) + drive-less-header reject
  [`test_servo_capture_cmd.py:166`](../../test/test_servo_capture_cmd.py#L166)
