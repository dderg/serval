# Same-MCU Homing Self-Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an MCU brake its own motion the instant a locally-armed endstop trips, instead of waiting for the host's `Stop` round-trip.

**Architecture:** Extract the gating core of the host `Stop` handler into `handle_stop_inner` (everything except the host-facing `send_stop_response`). Call it from `endstop_trip_task` so a local trip freezes the curve evaluator immediately. The host's existing `broadcast_stop` is untouched; its later `Stop` to the already-braked MCU is a harmless idempotent re-gate. Overshoot/slide accounting is out of scope (handled by a separate mechanism), so the host's position-reconstruction path is left as-is.

**Tech Stack:** C firmware (`src/*.c`), Rust runtime FFI (`rust/kalico-c-api`), the kalico-sim full-mode simulator (real firmware + klippy in Docker).

**Reference spec:** [`docs/superpowers/specs/2026-06-14-same-mcu-homing-self-gate-design.md`](../specs/2026-06-14-same-mcu-homing-self-gate-design.md)

**Testing note:** `src/mcu_transport_dispatch.c` and `src/endstop.c` are MCU firmware glue with no standalone C unit harness in this repo. Verification is therefore: (a) the existing Rust idempotency test that already proves a repeated gate is a safe no-op (`rust/kalico-c-api/tests/piece_gate.rs::gate_is_idempotent_like_a_repeated_stop`), and (b) end-to-end homing in kalico-sim full mode, which compiles `src/*.c` into the simulated firmware and auto-triggers the endstop during `G28`. This matches the project's established testing model for firmware glue. **Docker is required** for the kalico-sim verification.

---

## File Structure

| File | Responsibility | Change |
|------|----------------|--------|
| `src/mcu_transport_dispatch.c` | Host-command dispatch on the MCU (incl. `Stop`). | Extract `handle_stop_inner`; `handle_stop` calls it then `send_stop_response`. |
| `src/mcu_transport_dispatch.h` | Public prototypes for dispatch glue. | Declare `handle_stop_inner` so `endstop.c` can call it. |
| `src/endstop.c` | Endstop arm/detect/report on the MCU. | `endstop_trip_task` brakes locally via `handle_stop_inner` before emitting the trip event. |
| `CLAUDE.md` | Project constraints. | Replace the "do not optimize same-MCU homing" note with the self-gate behavior. |

No host-side or wire-protocol files change.

---

## Task 1: Extract `handle_stop_inner` (behavior-preserving refactor)

This task does NOT change any behavior — it only splits `handle_stop` so the gating core can be reused. The regression guard is the existing Rust idempotency test, which must stay green.

**Files:**
- Modify: `src/mcu_transport_dispatch.h` (add prototype after line 27)
- Modify: `src/mcu_transport_dispatch.c:551-563` (the `handle_stop` function)
- Regression test (existing): `rust/kalico-c-api/tests/piece_gate.rs`

- [ ] **Step 1: Confirm the regression guard passes before the change**

Run:
```bash
cd rust && cargo nextest run -p kalico-c-api -E 'test(gate_is_idempotent_like_a_repeated_stop)'
```
Expected: PASS (1 test run). This is the property the refactor must preserve — gating an already-gated engine returns `KALICO_OK`.

- [ ] **Step 2: Declare `handle_stop_inner` in the header**

In `src/mcu_transport_dispatch.h`, add the prototype right after the `mcu_transport_emit_endstop_trip` declaration (after line 27):

```c
void mcu_transport_emit_endstop_trip(uint8_t endstop_id, uint64_t trip_clock);

// Gate the curve evaluator and report the discard clock, without sending a
// StopResponse. Shared by the host `Stop` handler and the local endstop
// self-gate (endstop.c). Returns the runtime rc; writes the gate clock to
// *discard_clock.
int32_t handle_stop_inner(uint64_t *discard_clock);
```

- [ ] **Step 3: Extract the function in `mcu_transport_dispatch.c`**

Replace the current `handle_stop` (lines 551-563):

```c
static void
handle_stop(uint32_t correlation_id)
{
    int32_t rc = KALICO_ERR_NOT_INIT;
    uint64_t discard_clock = 0;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        rc = kalico_runtime_gate_pieces(runtime_handle);
        discard_clock = kalico_runtime_now_ticks(runtime_handle);
        irq_restore(flag);
    }
    send_stop_response(correlation_id, rc, discard_clock);
}
```

with:

```c
int32_t
handle_stop_inner(uint64_t *discard_clock)
{
    int32_t rc = KALICO_ERR_NOT_INIT;
    *discard_clock = 0;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        rc = kalico_runtime_gate_pieces(runtime_handle);
        *discard_clock = kalico_runtime_now_ticks(runtime_handle);
        irq_restore(flag);
    }
    return rc;
}

static void
handle_stop(uint32_t correlation_id)
{
    uint64_t discard_clock;
    int32_t rc = handle_stop_inner(&discard_clock);
    send_stop_response(correlation_id, rc, discard_clock);
}
```

Note: `handle_stop_inner` is intentionally non-`static` (so `endstop.c` can call it). It keeps the exact `irq_save`/`irq_restore` critical section the original had — required because `gate_pieces` mutates the engine and the TIM5 ISR could otherwise preempt it.

- [ ] **Step 4: Verify the firmware still compiles (kalico-sim build)**

Run (Docker required):
```bash
bash tools/kalico-sim/run.sh --privileged --homing-test --mode full --timeout 120
```
Expected: the image builds (compiles `src/mcu_transport_dispatch.c`) and the beacon Z homing test reports PASS. A compile error in the extraction shows up here as a build failure.

- [ ] **Step 5: Re-run the Rust idempotency guard**

Run:
```bash
cd rust && cargo nextest run -p kalico-c-api -E 'test(gate)'
```
Expected: all `gate*` tests PASS (the refactor touched no Rust, so behavior is unchanged).

- [ ] **Step 6: Commit**

```bash
git add src/mcu_transport_dispatch.c src/mcu_transport_dispatch.h
git commit -m "refactor(mcu): extract handle_stop_inner from the Stop handler"
```

---

## Task 2: Self-gate on a local endstop trip

**Files:**
- Modify: `src/endstop.c:102-116` (the `endstop_trip_task` function)
- End-to-end test: kalico-sim full mode (`--sensorless-phase-test` = same-MCU G28 X; `--homing-test` = cross-MCU beacon Z regression)

- [ ] **Step 1: Add the local brake to `endstop_trip_task`**

In `src/endstop.c`, replace the current `endstop_trip_task` (lines 102-116):

```c
void
endstop_trip_task(void)
{
    if (!sched_check_wake(&endstop_trip_wake))
        return;
    uint8_t oid;
    struct endstop *e;
    foreach_oid(oid, e, command_config_endstop) {
        if (!e->trip_pending)
            continue;
        e->trip_pending = 0;
        mcu_transport_emit_endstop_trip(e->endstop_id, e->trip_clock);
    }
}
```

with (brake first, then emit — `discard_clock` out-param unused on this path):

```c
void
endstop_trip_task(void)
{
    if (!sched_check_wake(&endstop_trip_wake))
        return;
    uint64_t discard_clock;
    handle_stop_inner(&discard_clock);
    uint8_t oid;
    struct endstop *e;
    foreach_oid(oid, e, command_config_endstop) {
        if (!e->trip_pending)
            continue;
        e->trip_pending = 0;
        mcu_transport_emit_endstop_trip(e->endstop_id, e->trip_clock);
    }
}
```

`endstop.c` already includes `mcu_transport_dispatch.h` (line 7), so `handle_stop_inner` is in scope with no new include.

- [ ] **Step 2: Verify same-MCU homing end-to-end (the new behavior)**

Run (Docker required):
```bash
bash tools/kalico-sim/run.sh --privileged --sensorless-phase-test --mode full --timeout 180
```
Expected: PASS. This homes X with a TMC virtual endstop on the *same* MCU as the X stepper, so the trip now self-gates locally. A pass confirms the engine freezes cleanly on the trip and the host's subsequent `Stop` is a harmless re-gate (a broken double-stop would shut klippy down and fail the run).

- [ ] **Step 3: Verify cross-MCU homing is unregressed**

Run (Docker required):
```bash
bash tools/kalico-sim/run.sh --privileged --homing-test --mode full --timeout 180
```
Expected: PASS. Beacon Z homing is genuinely cross-MCU; the source MCU self-gates its own idle engine harmlessly and the sink MCU still stops via the host relay.

- [ ] **Step 4: Commit**

```bash
git add src/endstop.c
git commit -m "feat(mcu): self-gate motion on a local endstop trip"
```

---

## Task 3: Lift the CLAUDE.md constraint

**Files:**
- Modify: `CLAUDE.md:27-28` (the `# Homing` section)

- [ ] **Step 1: Replace the homing note**

In `CLAUDE.md`, replace lines 27-28:

```markdown
# Homing
- We deliberately do not optimize for same mcu endstop+motor homing. All homing works as if mcu with the endstop is a different one than the one that drives the motors. it makes testing easier at this stage of development.
```

with:

```markdown
# Homing
- Same-MCU endstop+motor homing is optimized: when a locally-armed endstop trips, the MCU brakes its own motion immediately — `endstop_trip_task` calls `handle_stop_inner` (the gating core of the `Stop` handler) before emitting the trip event, so the curve evaluator freezes without a host round-trip. The host's `broadcast_stop` still fans the stop out to any genuinely-remote participant MCUs, and its later `Stop` to the self-gated MCU is a harmless idempotent re-gate. Both paths converge on the same `gate_pieces` freeze.
```

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: same-MCU homing is now optimized via the self-gate"
```

---

## Task 4: Final gate — full CI + sim green

**Files:** none (verification only)

- [ ] **Step 1: Run the quick CI gate**

Run:
```bash
./scripts/ci.sh quick
```
Expected: green (ruff, Rust workspace tests, clippy `-D warnings`, `cargo fmt --check`, watchdog canary). No `klippy/` host code changed, so `./scripts/ci.sh py` is not required.

- [ ] **Step 2: Confirm both sim homing runs still pass from a clean build**

Run (Docker required):
```bash
bash tools/kalico-sim/run.sh --no-cache --privileged --sensorless-phase-test --mode full --timeout 180
bash tools/kalico-sim/run.sh --privileged --homing-test --mode full --timeout 180
```
Expected: both PASS.

- [ ] **Step 3: Final fmt check (last step before any PR)**

Run:
```bash
cd rust && cargo fmt --all --check
```
Expected: no output (clean).

---

## Self-Review

**Spec coverage:**
- §1 Extract `handle_stop_inner` + call on local trip → Tasks 1 & 2. ✓
- §2 Double stop just stops again (idempotent re-gate) → verified by the existing Rust idempotency guard (Task 1 Step 1/5) and the same-MCU sim run (Task 2 Step 2). ✓
- §3 Lift CLAUDE.md constraint → Task 3. ✓
- Spec "out of scope" (slide/overshoot accounting; no host change; no new FFI) → respected: no host or FFI files in the file structure. ✓

**Placeholder scan:** No TBD/TODO; every code step shows the full before/after; every command has an expected result. ✓

**Type/name consistency:** `handle_stop_inner(uint64_t *discard_clock) -> int32_t` is declared identically in the header (Task 1 Step 2), defined identically in `mcu_transport_dispatch.c` (Task 1 Step 3), and called identically in `endstop.c` (Task 2 Step 1) and `handle_stop` (Task 1 Step 3). ✓
