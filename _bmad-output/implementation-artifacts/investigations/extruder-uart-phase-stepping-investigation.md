# Investigation: Extruder TMC-UART read fails while AB motors are in phase stepping

## Hand-off Brief

1. **What happened.** While the four AB drivers (`motor_a/a1/b/b1`) run in **phase stepping**, a host-side `DRV_STATUS` read of the **extruder** TMC driver fails (`Unable to read tmc uart 'extruder' register DRV_STATUS`) and shuts the printer down — confirmed in the structured logs at 2026-06-28 21:24:10 / 21:25:06 / 21:28:59, each immediately preceded by active phase mode.
2. **Where the case stands.** Root cause **Confirmed (High)**: the phase-stepping motion ISR and the bit-bang UART bit-sampler run at the **same NVIC priority (2)**, so a long phase-stepping XDIRECT SPI burst inside the motion ISR delays the 25 µs UART bit samples past their sample point, corrupting the frame; 5 host + 5 MCU retries exhaust → command error. The existing guard that suppresses TMC register polling in phase mode is scoped to the AB tmc5160 drivers only — the extruder is unguarded.
3. **What's needed next.** Decide the fix mechanism: suppress/serialize the extruder's `DRV_STATUS` echeck poll against phase-mode windows (cheap, matches existing precedent), and/or remove the priority equality / ISR-length hazard. Trivial-to-moderate; investigation stops at diagnosis.

## Case Info

| Field            | Value                                                                                  |
| ---------------- | -------------------------------------------------------------------------------------- |
| Ticket           | N/A                                                                                    |
| Date opened      | 2026-06-28                                                                              |
| Status           | Concluded (diagnosis) — fix direction set; implementation tracked WI-1…WI-7 (see 2026-06-29 follow-up) |
| System           | Trident bench (`dderg@trident.local` / 192.168.1.150), Octopus Pro **STM32H723** main + Octopus **STM32F446** bottom; branch `extruder-in-xy-phase` |
| Evidence sources | Source (`src/tmcuart.c`, `src/stm32/phase_stepping_spi.c`, `klippy/extras/tmc*.py`, `src/generic/motion_nvic_prio.h`); VictoriaLogs structured logs (host-py) on the bench |

## Problem Statement

User report (Hypothesis #1): "When I run my AB motors in phase stepping, when I try extruding it gives `Unable to read tmc uart 'extruder' register DRV_STATUS, please check`." Treated as a hypothesis; verified independently and **confirmed** by source + logs. Nuance: the failure correlates with **phase mode being active under load**, which is when extrusion happens — not with the extrusion command per se (see Deduction 2).

## Evidence Inventory

| Source                                   | Status    | Notes                                                                                                          |
| ---------------------------------------- | --------- | ------------------------------------------------------------------------------------------------------------ |
| Error origin (host)                      | Available | `klippy/extras/tmc_uart.py:288-290`; 5-retry read at `:284`; outer 3-retry at `klippy/extras/tmc.py:173`     |
| Bit-bang UART MCU impl                   | Available | `src/tmcuart.c:138-209` — software-timer per-bit RX/TX, `waketime += bit_time`, `SF_RESCHEDULE`              |
| NVIC priority definitions                | Available | `src/generic/motion_nvic_prio.h:11,13` — `MOTION_NVIC_PRIO 2`, `SCHED_NVIC_PRIO 2` (equal)                   |
| Motion/phase ISR priority               | Available | `src/stm32/runtime_tick_h7.c:93,256` (TIM5/TIM3 = MOTION_NVIC_PRIO); SysTick = SCHED_NVIC_PRIO `armcm_timer.c:141` |
| Phase-stepping SPI burst                 | Available | `src/stm32/phase_stepping_spi.c:143-195` busy-wait SPI XDIRECT; call chain via `rust/runtime/src/dispatch_stepper.rs:549` |
| Existing AB-driver guard                 | Available | `klippy/extras/tmc5160.py:604-611` `stop_checks()` — suppresses DRV_STATUS/GSTAT polling in phase mode, tmc5160 only |
| Prior SPI-corruption fix (related)       | Available | commit `095f87c5b` (2026-05-25), lock commits `03adf33c9` / `9364c2210` (2026-05-24)                          |
| VL failure events + phase correlation    | Available | host-py shutdown @ 21:24:10.094 / 21:25:06.005 / 21:28:59.307; phase enter (no exit) @ 21:24:01.17–.21        |
| Extruder driver type + MCU (config)      | **Partial** | Error string proves UART (bit-bang) driver. Same-MCU-as-AB is **Deduced**, not yet read from `printer.cfg` — bench went down before the read (see Missing Evidence) |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Confirm extruder TMC `[tmc220x extruder]` and its `uart_pin` MCU vs the AB tmc5160 SPI bus MCU in `printer.cfg` | Medium | Blocked | Bench down; one-line grep when it returns. Closes the only Deduced link. |
| 2 | Measure actual phase-stepping motion-ISR duration on H7 with 4 AB drivers (per-tick µs) | Low | Open | Quantifies the jitter margin vs the 25 µs bit period; not required to confirm root cause. |
| 3 | Confirm whether the extruder echeck poll, not a move-time read, is the specific failing call | Low | Open | py-trace shows continuous `tmcuart_send`; echeck poll is the likely caller. |

## Timeline of Events

| Time (2026-06-28 UTC) | Event | Source | Confidence |
| --------------------- | ----- | ------ | ---------- |
| 21:24:01.17–.21 | All four AB motors **enter** phase mode (`motor_a/a1/b/b1`), no `exit` logged afterward in window | VL `subsystem=phase_stepping` | Confirmed |
| 21:24:08.915 | SD-card print starts; XY moves + extruder moves begin (`newpos=[…, -0.8]`) | VL `virtual_sdcard` | Confirmed |
| 21:24:08.93–21:24:10.09 | ~1.2 s of host-py log silence (host blocked in UART retry loop) | VL (gap) | Deduced |
| 21:24:10.094 | `Transition to shutdown: Unable to read tmc uart 'extruder' register DRV_STATUS` | VL host-py | Confirmed |
| 21:25:06.005 / 21:28:59.307 | Same shutdown reproduces twice more, each after active/cycling phase mode | VL host-py | Confirmed |

## Confirmed Findings

### Finding 1: The extruder uses bit-bang single-wire UART; the read is a 5×-retry that errors on frame failure
**Evidence:** `klippy/extras/tmc_uart.py:218-223` (`reg_read` → `_decode_read`), `:196-216` (`_decode_read` returns `None` on length≠10 / start-stop / CRC mismatch), `:284-290` (5 retries then `command_error`). Outer retry wrapper `klippy/extras/tmc.py:173`.
**Detail:** The error fires only when 5 consecutive reads return `None`, i.e. 5 corrupted/empty frames in a row. The string `tmc uart 'extruder'` proves the extruder is a UART (tmc220x-class) driver, not SPI.

### Finding 2: The MCU samples each UART bit on a scheduler software timer, one bit_time apart
**Evidence:** `src/tmcuart.c:138-157` (`tmcuart_read_event`: `gpio_in_read`, `t->timer.waketime += t->bit_time; return SF_RESCHEDULE`), `:159-180` sync, `:199-209` TX toggle via `gpio_out_toggle_noirq`.
**Detail:** bit_time ≈ 25 µs (40 kbaud, `tmc_uart.py:86`). The actual sample instant is whenever the scheduler dispatches the due timer; if dispatch is delayed, the sample point shifts inside/past the bit cell.

### Finding 3: The bit-bang dispatcher and the phase-stepping motion ISR are at EQUAL NVIC priority
**Evidence:** `src/generic/motion_nvic_prio.h:11` `MOTION_NVIC_PRIO 2`, `:13` `SCHED_NVIC_PRIO 2`. Motion ISR: `src/stm32/runtime_tick_h7.c:93` (`TIM5_IRQn` = MOTION_NVIC_PRIO). Scheduler/SysTick dispatch (runs the UART bit timers): `src/generic/armcm_timer.c:141` (SysTick = SCHED_NVIC_PRIO).
**Detail:** Equal priority ⇒ no preemption between them; whichever is executing blocks the other until it returns.

### Finding 4: Phase stepping makes the motion ISR long (busy-wait SPI XDIRECT × 4 AB drivers per tick)
**Evidence:** `src/stm32/phase_stepping_spi.c:143-195` (per-byte busy-wait TX/RX, 50 µs/byte timeouts, 100 µs EOT) called from the motion tick via `rust/runtime/src/dispatch_stepper.rs:549`. Motion tick rate 10 kHz (`src/Kconfig:304-305`, `src/stm32/runtime_tick_h7.c:60`).
**Detail:** Each 10 kHz tick now performs an inline SPI coil-current write for each phase-mode driver — tens of µs per tick (×4 drivers) versus a few µs for ordinary step generation. This is the differential vs. normal stepping.

### Finding 5: The existing in-phase polling guard covers only the AB tmc5160 drivers, not the extruder
**Evidence:** `klippy/extras/tmc5160.py:604-611` — on phase-mode entry, `self._echeck_helper.stop_checks()` for the tmc5160 driver, with a comment that ISR SPI activity corrupts foreground register reads. The generic echeck/stop_checks machinery is `klippy/extras/tmc.py:136-138,229`.
**Detail:** The precedent for "stop TMC register polling while phase mode runs" already exists — but it is applied per-driver to the AB tmc5160s. The extruder's echeck poll keeps running.

**CORRECTION (2026-06-29, from adversarial review of WI-1):** This finding mischaracterized the tmc5160 echeck lifecycle. For a phase-stepping tmc5160, a post-enable callback is installed (`tmc5160.py:429-430`), and every enable/connect path arms `start_checks()` **only when no callback is present** (`tmc.py:433-434`, the early `return` at `:481` before `:482`, `:522-525`). So the AB phase drivers' error-checks are **not running during phase-mode printing** — the `stop_checks()` at `:611` is largely a no-op, and the `start_checks()` at `:693` (`exit_phase_mode`) is the *only* arm point, reached during sensorless homing (`arm()` → `exit_phase_mode` `tmc.py:652`; `disarm()` → `enter_phase_mode` re-stops `:712`). The pair is **not a mask over the extruder bug** — it avoids **false `drv_err`/`uv_cp` shutdowns on the AB drivers** from SPI reads corrupted by the busy-wait ISR (the `0x010a0023` garbage in the original comment), during the homing window. The extruder (`tmc2209`, UART) is a *separate* driver whose checks run normally and fail loudly without any change here.

## Deduced Conclusions

### Deduction 1: Frame corruption is caused by ISR-induced sample-point jitter (timing starvation)
**Based on:** Findings 2, 3, 4.
**Reasoning:** A UART bit cell is ~25 µs. The bit sampler can only run when the scheduler dispatches it, and the equal-priority phase ISR blocks dispatch for the ISR's duration (tens of µs, 10 kHz, ×4 drivers). When a sample is delayed by an appreciable fraction of (or beyond) 25 µs, the sampled level is wrong → start/stop or CRC check fails → `_decode_read` returns `None`. Five such failures in a row → the error.
**Conclusion:** The extruder read fails not from a wiring/driver fault but from MCU CPU-time contention created by phase stepping. This is the root cause.

### Deduction 2: The trigger is "phase mode active," not the extrusion command itself
**Based on:** Timeline (phase entered 21:24:01, stayed in phase mode; print/extrusion began 21:24:08.9; failure 21:24:10) + continuous `tmcuart_send` py-traces (the extruder DRV_STATUS echeck polls ~1/s regardless of extrusion).
**Reasoning:** The failing read is the periodic extruder `DRV_STATUS` echeck poll, which collides with an active phase-ISR window. Extrusion correlates because that is precisely when the AB motors are loaded and in phase mode.
**Conclusion:** "When I try extruding" is the observable proxy for "while phase mode is active under print load."

## Hypothesized Paths

### Hypothesis 1 (user's premise): Phase stepping AB motors breaks extruder UART reads
**Status:** Confirmed
**Resolution:** Verified by source (Findings 1-5) and tight temporal correlation in VL (phase mode active at each of the three shutdowns). Refined: the coupling is CPU-time/ISR-priority contention, not anything extruder-specific.

### Hypothesis 2: SPI-bus contention / register corruption (the 095f87c5b mechanism) is the extruder's cause
**Status:** Refuted (for the extruder)
**Theory:** The known ISR-vs-foreground SPI corruption fixed for the AB drivers also explains the extruder failure.
**Would refute:** The extruder is a UART bit-bang driver, not on the shared SPI bus.
**Resolution:** Refuted — that mechanism and its guard (`tmc5160.py:604-611`) apply to SPI tmc5160 drivers. The extruder's failure is timing starvation (Deduction 1), a sibling symptom of the same root contention but via a different path. The earlier fix is the precedent, not the coverage.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| `printer.cfg` extruder TMC stanza (driver type, `uart_pin` MCU) and AB tmc5160 SPI bus MCU | Confirms (vs. Deduces) that extruder UART + AB phase SPI share the H7 MCU — required for the contention to hold | When bench returns: `ssh trident grep -niE '^\[tmc.*extruder\]|uart_pin|^\[mcu' printer.cfg` |
| Measured phase-ISR duration per tick (4 drivers) | Quantifies jitter margin vs 25 µs bit period | MCU diag / scope; or instrument the tick |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Error origin | `klippy/extras/tmc_uart.py:288-290` (`_do_get_register`, 5-retry exhausted) |
| Trigger | Extruder periodic `DRV_STATUS` echeck poll (`klippy/extras/tmc.py:136-138`) issued while AB phase-stepping ISR is active |
| Condition | Equal NVIC priority (`motion_nvic_prio.h:11,13`) + long phase ISR (`phase_stepping_spi.c:143-195`, `dispatch_stepper.rs:549`) delays the scheduler-dispatched UART bit sampler (`tmcuart.c:138-157`) past the ~25 µs bit cell → frame CRC/framing failure |
| Related files | `klippy/extras/tmc5160.py:604-611` (existing AB-only guard), `src/stm32/runtime_tick_h7.c:93`, `src/generic/armcm_timer.c:141`, `rust/runtime/src/isr_phase.rs` |

## Conclusion

**Confidence:** High

Confirmed root cause: running the AB drivers in phase stepping turns the 10 kHz motion ISR into a long busy-wait SPI burst (4 inline TMC5160 XDIRECT writes per tick). Because that ISR and the bit-bang UART bit-sampler share NVIC priority 2, the ISR blocks the scheduler from dispatching the extruder's UART bit samples on time; the sample point drifts past the ~25 µs bit cell, every retry's frame fails CRC/framing, and after 5 retries the host raises `Unable to read tmc uart 'extruder' register DRV_STATUS` and shuts down. The class of bug is already known and guarded for the AB tmc5160 drivers (`tmc5160.py` `stop_checks` on phase entry); the extruder is simply outside that guard. The one remaining Deduced (not Confirmed) link is that the extruder UART and the AB SPI live on the same H7 MCU — true for standard Trident/Octopus-Pro wiring and consistent with the reproducible correlation, but not yet read from `printer.cfg` (bench went down).

## Recommended Next Steps

### Fix direction
Two non-exclusive mechanisms:
- **Suppress/serialize the extruder poll during phase windows (matches existing precedent).** Extend the `stop_checks()`-on-phase-entry pattern so the extruder's echeck `DRV_STATUS`/`GSTAT` polling is paused (or its reads serialized against phase-ISR-active windows) while any AB driver is in phase mode. Cheapest, lowest-risk, mirrors `tmc5160.py:604-611`. Note this masks the symptom for the *poll* path; a register read genuinely needed mid-phase would still be exposed.
- **Remove the timing hazard at the source.** Either give the bit-bang UART dispatch priority over the motion/phase ISR, or shorten/de-busy-wait the phase SPI path (the `phase_stepping_spi.c` comments already point at DMA-based SPI / "Phase 2" as the real arbitration fix). This addresses the root contention rather than the extruder poll alone, but is the larger change and must respect the load-bearing M7/M4 priority + same-tick invariants.

### Diagnostic
- When the bench returns, close Backlog #1 (confirm same-MCU) — the only Deduced link.
- Optional: instrument phase-ISR duration to quantify the jitter margin (Backlog #2).

## Reproduction Plan
1. Put the four AB drivers in phase stepping (as in the 21:24 session).
2. Start a print / issue moves that hold phase mode active while the extruder driver is enabled (echeck polling running).
3. Expected: within seconds, an extruder `DRV_STATUS` echeck poll coincides with a phase-ISR window → 5 failed reads → `Unable to read tmc uart 'extruder' register DRV_STATUS` shutdown. Observed three times on 2026-06-28 (21:24:10, 21:25:06, 21:28:59).

## Side Findings
- `feed_throttle_enter/exit` (`subsystem=motion`) fired in the same window (e.g. 21:24:06–07), consistent with the planner throttling on MCU backpressure while the phase ISR loads the H7 — corroborating Finding 4 (heavy motion-ISR load in phase mode). Evidence: VL, Deduced.
- Phase mode was observed entering/exiting per-move at sub-second cadence in the 21:28 cluster (`motor_a/a1/b/b1`), so the hazard window recurs continuously throughout a print, not just once. Evidence: VL `subsystem=phase_stepping`, Confirmed.

## Follow-up: 2026-06-29

### New Evidence (bench `printer.cfg` + `src/stm32/stm32h7_spi.c`)

- **Same-MCU link now CONFIRMED** (was Deduced — closes Backlog #1 / the Missing-Evidence gap). Extruder driver is `[tmc2209 extruder] uart_pin: PD3` (bit-bang UART); the four AB drivers are `[tmc5160 motor_a/b/a1/b1]` on `spi_bus: spi1`. PD3 and `spi1` are both on the main `[mcu]` (H723); `[mcu bottom]` is the F446. The contention is intra-MCU on the H723, as reasoned.
- **Deployed topology = 1 shared bus (spi1) × 4 drivers, independent CS** (motor_a PC7, motor_b PC6, motor_a1 PD11, motor_b1 PC4). Not daisy-chained. This is simultaneously the real bench config and the worst-case shared-bus case for per-tick airtime.
- **Bus bandwidth is not the constraint.** `fast_cfg` clocks spi1 at 8 MHz; 4 × 5 B = 20 B ≈ 20 µs airtime per 100 µs tick (~20% util). At the TMC default ~1 MHz it would be ~160 µs > tick — which is why `fast_cfg` exists. The binding constraint was always the CPU busy-wait, not the wire.
- **No existing SPI DMA.** `stm32h7_spi.c` is fully polled (`spi_transfer_locked:138-194`); foreground `spi_transfer` also busy-waits the same global `phase_spi_try_acquire()` lock (`:200`). The only DMA in the firmware is `serial.c` USART RX, and it is **STM32F4** IP (`DMA2_Stream2`, `DMA_SxCR_CHSEL`, `:82-150`), F4-gated. The H723 DMA IP is **classic DMA1/DMA2 + DMAMUX1** (`DMA_Stream_TypeDef`, `SxCR/SxNDTR/SxPAR/SxM0AR`), **not** GPDMA/`CxCR`. `serial.c` is a partial stream-config mirror; the only net-new piece is the DMAMUX1 request routing (`DMAMUX1_Channelx->CCR = SPIx_TX` request id).

### Updated Hypotheses

- Hypothesis 1 (user premise): remains **Confirmed**, and the same-MCU dependency it rested on is now Confirmed rather than Deduced.

### Decision / Design Direction (set by user)

- **Reframe:** the defect is not "the extruder poll collides." It is that the phase-stepping motion ISR **busy-polls the CPU** (`phase_stepping_spi.c:159-181`, per-byte 50 µs TX/RX + 100 µs EOT spin inside the 10 kHz TIM5 ISR). Remove the busy-wait and the extruder UART symptom disappears as a side effect.
- **No symptom mask.** The root fix is removing the busy-wait, not masking. **(Revised 2026-06-29:)** the original plan to delete the `tmc5160.py` `stop_checks()`/`start_checks()` pair as a standalone first step was **dropped** — adversarial review showed that pair is not a mask over the extruder bug; it prevents false AB-driver shutdowns from ISR-corrupted SPI reads during sensorless homing, and deleting it would leave AB error-checks permanently off (a safety regression) without helping surface the extruder bug. The pair becomes genuinely removable once the DMA fix eliminates the SPI-read corruption — so suppression removal is now the **final cleanup step of the DMA work**, not a precursor. See the WI-1 entry and the Finding 5 correction.
- **NVIC-priority swap rejected** — it inverts which side loses and demotes the hard-real-time phase write to rescue a housekeeping read.
- **Root fix = move phase-stepping SPI to DMA**, with a general topology model (N buses × M drivers), per the converged design below.

### Converged v1 design (party-mode roundtable: Winston/Amelia/Dr. Quinn/Murat)

- **Unifying abstraction (no topology special-casing):** a *bus* is a serialized queue of jobs; a *job* = `(assert CS → 5-byte transfer → deassert CS)`. "Topology" is only the `motor → bus` fan-out already modeled by `phase_stepping_register_bus` / `register_motor`. Build a per-bus array (`MAX_PHASE_BUSES`); the bench uses one slot. No `if (shared)` branch.
- **Global `phase_spi_busy` lock DELETED**, replaced by per-bus ownership (`IDLE → BUSY → completion`, advanced by each bus's own DMA TC IRQ). This also removes the false cross-bus serialization counted by `phase_spi_skip_count`.
- **CS sequenced by DMA completion, not the data stream:** one DMA TX transfer per motor; the TC IRQ gates CS-high on `SPI_SR.EOT/BSY` (DMA TC fires when the FIFO is fed, not when SPI finishes shifting — early CS-high truncates the last byte), advances the cursor, arms the next motor or releases the bus. Hardware-NSS rejected (CS is arbitrary GPIO). Linked-list/timer-DMA CS chaining parked as the zero-CPU asymptote.
- **Arbitration (shared bus):** hard phase-priority; foreground AB tmc5160 reads enqueue onto the same per-bus queue and drain in a post-batch gap (reads are not deadline-bound; phase writes are). Foreground `spi_transfer` must stop busy-waiting in the SAME change, or the starvation class is only narrowed.
- **H7 D-cache:** DMA TX buffers `SCB_CleanDCache_by_Addr` after staging / before enable (or a non-cacheable MPU region as fast-follow). C owns DMA placement (boundary invariant). F446 (no cache) must not pay for it.
- **Fail-loud (own `log_codes.rs` codes):** `PHASE_DMA_OVERRUN` (per-bus still BUSY at next tick = airtime/deadline miss → shutdown; replaces today's silent skip), `PHASE_DMA_TEIF`, `PHASE_DMA_FEIF`, `PHASE_DMA_UNDERRUN` (reserved for the v-next RX path). Overrun check at the top of the per-tick kick, before the buffer swap.
- **Seam stays one-directional:** Rust (`dispatch_stepper.rs:549`) stages coil values into a `#[repr(C)]` double-buffer + calls one `extern "C"` trigger; C owns swap/arm/walk/release/completion. No C→Rust callback; Rust reads the fail-loud code next tick.

### Work breakdown

- **WI-1 — DROPPED as a standalone task (2026-06-29).** Original plan: delete the `tmc5160.py` `stop_checks()`/`start_checks()` pair. Adversarial review (`bmad-review-adversarial-general`) returned a BLOCKER: that pair is the *only* thing arming AB-driver error-checks during sensorless homing (the enable path skips `start_checks()` for phase drivers via the post-enable callback, `tmc.py:433-434`/`:481`/`:522-525`), so deleting it disables overtemp/short/undervoltage detection on the AB drivers — a safety regression — and does nothing for the extruder bug (separate `tmc2209` UART driver, already fails loudly). Change was reverted; tree clean. **Re-scoped:** remove the pair as the final cleanup step of the DMA work (WI-8), valid only once SPI-read corruption is gone.
- **WI-8 (new, last):** after WI-2…WI-7 land and SPI-read corruption is eliminated, delete the now-unnecessary `tmc5160.py` `stop_checks()`/`start_checks()` phase pair; verify AB-driver error-checks stay armed through a homing round-trip.

### DMA register reference — WI-3 unblock (2026-06-29, from `lib/stm32h7/include/stm32h723xx.h`)

**IP confirmed: classic DMA1/DMA2 + DMAMUX1** (`DMA_Stream_TypeDef` = `CR, NDTR, PAR, M0AR, M1AR, FCR`). **Not** GPDMA. In-tree mirror = `src/stm32/serial.c:140-160` (RX stream), but that path is **F4-coded** (uses `DMA_SxCR_CHSEL`, which does not exist on H7) — so mirror the *stream-config idiom*, not the request-select line.

- **Stream ↔ DMAMUX mapping:** `DMAMUX1_Channel{N}` drives `DMA1_Stream{N}` for N=0–7, `DMA2_Stream{N-8}` for N=8–15. Pick one free stream **per bus** (the per-motor CS-walk re-uses that one stream across the bus's motors). Bench spi1 → assign e.g. `DMA1_Stream0`/`DMAMUX1_Channel0`. **CONFIRMED all DMA1/2 streams free on the H723 build:** the only DMA consumer (`serial.c` RX) is gated `#if … CONFIG_MACH_STM32F401` (`serial.c:83`), and no other `DMA[12]_Stream`/`DMAMUX1_Channel` use exists in `src/stm32/`. No conflict.
- **Request select (the H7-specific step):** `DMAMUX1_Channel{N}->CCR = <SPIx_TX request id>` (field `DMAMUX_CxCR_DMAREQ_ID`, bits 0–7). Request IDs are not in the in-repo CMSIS header; **`SPI1_TX=38` CONFIRMED** (RM0468 Table 121 + ST community corroboration — DMAMUX1 lines are sequential rx/tx pairs: SPI1=37/38, SPI2=39/40). Others: `SPI3_TX=62, SPI4_TX=84, SPI5_TX=86`. **SPI6_TX is on BDMA/DMAMUX2 (D3 domain), not DMA1/2/DMAMUX1** — general-topology caveat; a bus on SPI6 needs the BDMA path. Bench uses spi1 (id 38).
- **TX stream `CR`** (bit positions, confirmed): `DIR=0b01` (mem→periph, `DIR_Pos=6`), `MINC` (`Pos=10`), `PINC=0`, `PSIZE=00`+`MSIZE=00` (byte), `TCIE` (`Pos=4`), `TEIE` (`Pos=2`), `EN` (`Pos=0`). Optional `DBM` (`Pos=18`) + `M0AR`/`M1AR` = hardware double-buffer (`CT` `Pos=19` = active half) — could replace the manual A/B swap; weigh against the per-motor re-arm which already re-points `M0AR`.
- **FIFO:** `FCR.DMDIS` (`Pos=2`); direct mode is fine for a 5-byte burst (serial.c uses defaults).
- **Completion flags / clear:** status in `LISR` (streams 0–3) / `HISR` (4–7); clear via `LIFCR`/`HIFCR`. `TCIF` bit positions per stream: S0=5, S1=11, S2=21, S3=27 (same pattern repeats S4–S7 in the high regs); `TEIF` = TCIF−2, `FEIF` = group base. Clear-before-arm like `serial.c:143`.
- **SPI side:** set `SPI_CFG1.TXDMAEN` to route `TXDR` to DMA.
- **IRQ:** `DMA1_Stream0..6_IRQn = 11..17`, `Stream7 = 47`; `DMA2_Stream0/1 = 56/57` (…). Register the TC handler via `armcm_enable_irq(...)` as `serial.c:160`. **Completion-IRQ priority is a flagged risk** (Murat): set it so it does not relocate the starvation — do not place it where it can preempt the motion-critical path.
- **WI-4 EOT trap stands:** DMA `TCIF` fires when the FIFO is fed, not when SPI finishes shifting — gate CS-high on `SPI_SR.EOT`/`BSY` (mirror `stm32h7_spi.c:178-189`).

WI-3 is now spec-able down to register writes; the only external confirms are the DMAMUX request-ID table (RM0468) and the free-stream check on the H7 build.
- **WI-2:** per-bus struct in `phase_stepping_spi.c`; delete global `phase_spi_busy`/`skip`; double-buffered `txbuf[2][N*5]` (32-B aligned for cache-clean granularity); `motor_cursor` walk.
- **WI-3:** net-new H723 DMA1/2 + DMAMUX1 TX-stream setup (mirror `serial.c` stream config; add DMAMUX1 request routing for `SPIx_TX`). Requires RM0468 DMA/DMAMUX register read first.
- **WI-4:** TC IRQ CS-walk with EOT/BSY gating.
- **WI-5:** one-directional `extern "C"` stage+trigger seam.
- **WI-6:** fail-loud codes + cache-maintenance call site.
- **WI-7:** test split — Rust nextest (packing golden vector, topology map 1/1 · 4/1 · 2×2, per-bus FSM transition table incl. OVERRUN edge); bench/renode for DMA TC timing, OVERRUN deadline injection, cache coherency, EOT-vs-TC ordering.

### Verification / acceptance (Murat)

- **Canaries already in the structured logs (pass/fail):** `phase_spi_skip_count` → **0** after per-bus locking; extruder `tmcuart` retry-histogram → **0** (watch the tail under load, not the mean) with the suppression removed.
- **Soak matrix:** P0 = 1 bus × 4 (airtime saturation) **and** 2 bus × 2 (parallel streams + per-bus lock race); P1 = 4 bus × 1; P2 = asymmetric. Bench is the only place timing is real; renode for the priority/ordering invariant; mcu-sim for coil-current correctness only.
- **Highest new risk:** M7 cache/DMA coherency → *silently wrong coil currents* (worse than the loud shutdown; violates fail-loudly). Gate explicitly.

### Backlog Changes

- Backlog #1 (same-MCU confirmation) → **Done** (Confirmed above).
- New: WI-2…WI-7 (DMA conversion) + WI-8 (suppression cleanup, last) tracked as the implementation follow-on. Standalone WI-1 dropped (see above).

### Updated Conclusion

Root cause and fix direction are settled. The investigation is **Concluded**; remaining work is implementation (DMA conversion) tracked as WI-1…WI-7, not further diagnosis.
