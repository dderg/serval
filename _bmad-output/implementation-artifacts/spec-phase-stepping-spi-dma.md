---
title: 'Phase-stepping SPI → DMA conversion (H723)'
type: 'bugfix'
created: '2026-06-29'
status: 'done'
baseline_commit: '016fbea3de2b5d1beb6da5947b5c0b09c3febb66'
context:
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/extruder-uart-phase-stepping-investigation.md'
  - '{project-root}/docs/rewrite/mcu-c-rust-boundary.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The phase-stepping coil-current write busy-waits SPI per byte (`phase_stepping_spi.c:159-181`, 50µs TX/RX + 100µs EOT) inside the 10kHz TIM5 motion ISR at top NVIC priority, monopolizing the CPU so the bit-bang extruder TMC-UART bit sampler misses its ~25µs cells → `Unable to read tmc uart 'extruder' register DRV_STATUS` → shutdown (confirmed 3× in logs).

**Approach:** Replace the in-ISR busy-wait with per-bus DMA. The motion tick stages coil datagrams into a per-bus double buffer and triggers once; a DMA transfer-complete IRQ walks the bus's motors (CS-sequenced), freeing the CPU during the transfer. General topology (N buses × M drivers) via a per-bus array; the global SPI lock is deleted in favor of per-bus ownership.

## Boundaries & Constraints

**Always:** One code path for all topologies — a bus is a serialized queue of `(CS-low → 5-byte XDIRECT → CS-high)` jobs; topology is only the motor→bus fan-out (chain length 1 = the separate-bus case). C owns DMA buffers/streams/placement; the FFI seam stays `extern "C"` + `#[repr(C)]` and one-directional (Rust stages + triggers, C owns completion, no C→Rust callback). Fail loudly on any deadline/transfer fault. On H7, `SCB_CleanDCache_by_Addr` the staged buffer before enabling the stream; F446 (no D-cache) pays nothing. No narration comments.

**Ask First:** Changing the motion-ISR or any NVIC priority. Using hardware-NSS instead of GPIO-CS sequencing. Any change to `klippy/extras/tmc5160.py`. A shared-bus airtime budget that cannot drain 4×5 bytes @8MHz inside the 100µs tick.

**Never:** Removing the `tmc5160.py` `stop_checks()/start_checks()` pair (that is WI-8, valid only after this lands — removing now is a safety regression; see case file). NVIC-priority swap, symptom-mask, GPDMA register set (H723 uses classic DMA1/2 + DMAMUX1), daisy-chain CS assumption (drivers have independent CS).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Normal tick | Bus idle, N motors staged | Commit swaps buffer, cleans cache, arms motor 0; TC IRQ walks CS-high(i)→CS-low(i+1)→re-arm; last motor releases bus | N/A |
| Overrun | Commit finds bus still BUSY from prior tick | Loud fault (`PHASE_DMA_OVERRUN`), no skip/pad | shutdown via structured fault |
| Transfer/FIFO error | TC ISR sees TEIF/FEIF | Sticky per-bus fault flag → raised at next commit | `PHASE_DMA_TEIF`/`PHASE_DMA_FEIF` |
| Single-motor bus | bus has 1 motor | chain length 1: arm → TC → release (same path) | N/A |
| EOT race | DMA TC fires before SPI shift completes | CS-high gated on `SPI_SR.EOT`/`BSY` before deassert | last-byte truncation avoided |

</frozen-after-approval>

## Code Map

- `src/stm32/phase_stepping_spi.c` -- per-bus struct + double buffer; `write_xdirect` becomes staging-only; new `commit`, DMA arm, TC CS-walk. Delete global `phase_spi_busy`/`skip_count` (`:11-12`).
- `src/stm32/phase_stepping_spi.h` -- FFI contract: keep `write_xdirect`; add `phase_stepping_commit_tick()` returning per-bus fault status.
- `src/stm32/stm32h7_spi.c` -- migrate foreground `spi_transfer` (`:200`) off the global lock to per-bus arbitration (phase-priority; foreground drains in the gap).
- `rust/runtime/src/dispatch_stepper.rs` -- `:549` call becomes staging; extern decl `:29-31`.
- `rust/runtime/src/engine.rs` -- `tick()` (`:280`/`:367`): call `phase_stepping_commit_tick()` once after all axes dispatched; raise fault on returned status.
- `rust/runtime/src/log_codes.rs` -- append `EVENT_RUNTIME_PHASE_DMA_*` (wire-stable, do not renumber).
- `rust/runtime/src/error.rs` + `fault_helpers.rs` -- `FaultCode::PhaseDma*` + raise helper.

## Tasks & Acceptance

**Execution:**
- [x] `src/stm32/phase_stepping_spi.c` -- replace global lock with `phase_bus_state[]` (busy, `txbuf[2][MAX_PHASE_MOTORS*5]` 32-B aligned, motor_cursor, active_half, stream, dmamux_req); `write_xdirect` writes datagram into active buffer slot only (no SPI).
- [x] `src/stm32/phase_stepping_spi.c` -- DMA1/DMAMUX1 TX setup mirroring `serial.c:140-160` (H7: `DMAMUX1_Channel0->CCR = 38` for spi1; CR DIR=01/MINC/byte/TCIE/TEIE/EN; `SPI_CFG1.TXDMAEN`); per-bus uses one stream. Exact regs in case-file "DMA register reference".
- [x] `src/stm32/phase_stepping_spi.c` -- `phase_stepping_commit_tick()`: per configured bus, if BUSY→set overrun status; else clean cache, swap buffer, arm motor 0. TC IRQ: gate CS-high on `SPI_SR.EOT`, advance cursor, arm next or release; latch TEIF/FEIF.
- [x] `src/stm32/stm32h7_spi.c` -- foreground `spi_transfer` enqueues on the owning bus and waits without busy-spinning the removed global lock; phase batch has priority.
- [x] `rust/runtime/src/dispatch_stepper.rs` -- keep per-motor `write_xdirect` call (now staging); no signature change.
- [x] `rust/runtime/src/engine.rs` -- call `phase_stepping_commit_tick()` once at tick end; on nonzero status raise the matching `FaultCode::PhaseDma*`.
- [x] `rust/runtime/src/log_codes.rs` + `error.rs` + `fault_helpers.rs` -- add the four fault codes + raise helper (structured `event_log_emit`, not `printf`).
- [x] `rust/runtime/src/dispatch_stepper/tests.rs` -- unit tests (see Verification / I/O matrix).

**Acceptance Criteria:**
- Given the bench topology (1 bus, 4 independent-CS motors @8MHz), when a phase tick stages 4 datagrams, then all 4 clock out via DMA with correct CS framing and the motion ISR returns without per-byte SPI spinning.
- Given a bus whose prior-tick transfer has not drained, when `commit_tick` runs, then it raises `PHASE_DMA_OVERRUN` and does not advance.
- Given independent buses, when both tick, then neither serializes on the other (no global lock); `phase_spi_skip_count` no longer exists.
- Given the unit suite, when `cargo nextest` runs, then packing golden-vector, topology map (1/1, 4/1, 2×2), and the per-bus FSM transition table (incl. OVERRUN edge, CS-high(i) before CS-low(i+1)) all pass.

## Design Notes

Seam (minimal Rust churn): `write_xdirect(motor_idx, coil_a, coil_b)` keeps its signature but only writes the 5-byte datagram `{0xAD, B_sign, B_lo, A_sign, A_lo}` into `phase_buses[bus_of(motor)].txbuf[active_half]` at the motor's slot. `engine.rs tick()` calls `phase_stepping_commit_tick()` ONCE after every axis is dispatched (not inside per-axis `dispatch_phase`). `commit_tick` returns a per-bus fault bitmask; Rust raises the structured fault so no Rust event call happens inside the C ISR. TC-ISR transfer errors set a sticky flag consumed at the next commit.

DMA detail, request IDs, stream-free confirmation, and the EOT trap are fully resolved in the case-file "DMA register reference — WI-3 unblock" subsection — implement against it.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p runtime` -- expected: new packing/topology/FSM tests pass.
- `./scripts/ci.sh rust-clippy` -- expected: clean (`-D warnings`).
- `./scripts/ci.sh rust-fmt` -- expected: clean.
- `./scripts/ci.sh rust-mcu-h7 && ./scripts/ci.sh rust-mcu-f4 && ./scripts/ci.sh rust-mcu-g0` -- expected: all three MCU targets build.

**Manual checks (bench/renode — NOT in CI, do not claim green from CI):**
- Renode: per-bus priority/ordering invariant; `phase_spi_skip_count` removal has no regressions.
- Trident bench: scope CS framing (each CS low only during its 5-byte window); soak 1 bus×4 under phase load + extruder DRV_STATUS poll → zero `tmcuart` failed reads, zero retries, zero shutdowns (inverse of the original repro); inject a stalled transfer → `PHASE_DMA_OVERRUN` fires (no silent skip); cache-coherency: remove the clean call → corrupted-byte test goes red.

## Review & Fix Log

A 3-reviewer adversarial pass (blind hunter / edge-case hunter / acceptance auditor, no shared context) ran against the diff; findings were verified against the repo and patched in place (no re-derivation — structure was sound). Fixed:
- **BLOCKER — TX buffer DMA-unreachable:** `phase_buses[]` was in DTCM (`0x20000000`), which DMA1/2 cannot read → placed in `.axi_bss` (AXI SRAM), H7-gated, budget-asserted + occupant guard updated.
- **BLOCKER — false OVERRUN at the tick boundary:** the equal-priority DMA TC ISR can be queued behind the motion tick, so a *completed* batch read `busy==1` → spurious shutdown. Commit now discriminates via hardware (last motor + DMA TCIF + SPI EOT, no error) and finalizes inline instead of faulting; clears the stale pending IRQ.
- **MAJOR ×5:** EOT-timeout now fails loud (was silent truncation); DMA transfer error aborts the walk; a 2nd bus's fault is no longer dropped; unbounded foreground-defer now faults after 2 ticks; `SPE` cleared before CFG writes.
- **MINOR:** unknown-fault-kind decode event corrected; frozen-boundary narration comment removed; DMAMUX request IDs named.

Rust gates green post-fix (nextest 351 + 21 gated phase tests, clippy, fmt, all 3 MCU-Rust targets). The C is **not** compiled by any gate — bench/renode verification remains required (see Verification). The BLOCKER-A inline-finalize + `NVIC_ClearPendingIRQ` path is the #1 bench-verification target.

## Suggested Review Order

**Rust ↔ C seam (start here — grasp the design)**

- Commit called once per tick, after all axes dispatched; raises any fault.
  [`engine.rs:391`](../../rust/runtime/src/engine.rs#L391)
- `write_xdirect` is now staging-only; new one-directional commit wrapper.
  [`dispatch_stepper.rs:42`](../../rust/runtime/src/dispatch_stepper.rs#L42)

**DMA orchestration (the heart — highest risk)**

- Commit: false-overrun discriminator (hardware-confirmed) + per-bus arm.
  [`phase_stepping_spi.c:442`](../../src/stm32/phase_stepping_spi.c#L442)
- TC-ISR CS-walk: EOT-gated CS-high, fail-loud on EOT-timeout/TEIF/FEIF.
  [`phase_stepping_spi.c:270`](../../src/stm32/phase_stepping_spi.c#L270)
- Per-motor DMA arm: stream + DMAMUX request wiring, SPE cleared before CFG.
  [`phase_stepping_spi.c:225`](../../src/stm32/phase_stepping_spi.c#L225)

**Memory placement (blocker fix)**

- TX double-buffer in `.axi_bss` (DMA-reachable AXI SRAM), budget-asserted.
  [`phase_stepping_spi.c:65`](../../src/stm32/phase_stepping_spi.c#L65)
- AXI SRAM occupant sum / overflow guard updated.
  [`runtime_storage.c:29`](../../src/runtime_storage.c#L29)

**Foreground arbitration (global lock removed)**

- Foreground TMC read claims the bus per-bus, no global spin-lock.
  [`stm32h7_spi.c:203`](../../src/stm32/stm32h7_spi.c#L203)

**Fail-loud plumbing**

- Structured fault raise + status decode (C→Rust, one-directional).
  [`fault_helpers.rs:228`](../../rust/runtime/src/fault_helpers.rs#L228)
- Wire-stable fault codes + events (append-only).
  [`error.rs:205`](../../rust/runtime/src/error.rs#L205)

**Tests (model-level; real C is bench-verified)**

- Packing golden vector, topology fan-out, per-bus FSM incl. overrun edge.
  [`tests.rs:652`](../../rust/runtime/src/dispatch_stepper/tests.rs#L652)
