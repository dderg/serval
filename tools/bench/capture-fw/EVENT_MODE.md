# Event mode: sub-µs step timestamps

Status: design only, nothing implemented. Count mode (`SVC1`, 40 kHz i8 deltas) stays as it is. Event mode is a second mode, chosen per run with the start command.

## Goal

Timestamp every STEP and DIR edge on all four channels at ≤100 ns resolution, keep the step count exact, and fail loudly whenever exactness cannot be proven. This exposes MCU execution errors (`classic move loaded behind its pending unstep`, 0.2–7 µs, `EVENT_LOG_EVENT_MOTION_STEP_LOAD_LATE`) that the 25 µs tick hides today.

## The reference problem: precise capture alone is not enough

The F401 does not execute the ideal trajectory. It executes the classic `queue_step` schedule. That schedule is compressed with `stepcompress_max_error` = 25 µs (`klippy/mcu.py:30`; `max_error_ticks` in `rust/motion-core/src/pump/stepcompress_sink.rs:150-169`). Measured against `analyze.py`'s ideal reference, a perfect capture therefore still shows up to ±25 µs error. That is design, and it is most of the 13–25 µs p99 we see today. To see MCU errors of 0.2–7 µs, the capture must be compared with the **commanded** step clocks:

- **Commanded-schedule tap (recommended).** Add an opt-in host tap to the stepcompress sink that records every `queue_step`/`set_next_step_dir`/`reset_step_clock` per oid, in send order, to `step_tap.bin` in the run dir. The analyzer replays it into exact F401-clock step times. Pairing then goes by index per motor: the i-th commanded step against the i-th captured step. Lattice keys are not needed for this comparison, and a count mismatch is a hard failure.
- **Stopgap.** Use a bench config with a tiny `stepcompress_max_error` (≥1 F401 tick). This is cheap, but it multiplies `queue_step` traffic at 113 kHz, so it measures a different operating point.

## Capture hardware

Pins are fixed. Every STEP and DIR line already sits on CH1/CH2 of one timer:

| ch | STEP | DIR | timer | bus clock | AF |
|---|---|---|---|---|---|
| a | PE9 | PE11 | TIM1 | APB2 168 MHz | 1 |
| b | PA6 | PA7 | TIM3 | APB1 84 MHz | 2 |
| z | PD12 | PD13 | TIM4 | APB1 84 MHz | 2 |
| e | PC6 | PC7 | TIM8 | APB2 168 MHz | 3 |

Changes from count mode (`capture.rs::arm_counter`):

- **Internal clock, one rate.** Counters run free on the internal clock, not in external-clock mode. Set TIM1/TIM8 to PSC=1 and TIM3/TIM4 to PSC=0, so all four count at **84 MHz** (11.9 ns per tick), ARR=0xFFFF, 16-bit. A wrap takes 65536/84 MHz = 780 µs.
- **Captures.** IC1 = TI1 (STEP, rising edge, or both edges for TMC both-edge configs). IC2 = TI2 (DIR, both edges, as today).
- **Equal filter delay.** TIM3/TIM4 use IC filter N=4 at f_CK_INT (84 MHz). TIM1/TIM8 use N=8 at f_CK_INT (168 MHz). Both are 48 ns nominal, so the filter delay matches across channels.
- **One timebase.** TIM1 is the master with MMS=Enable. TIM3, TIM4 and TIM8 use SMS=trigger with TS=ITR0, which is TIM1 for all three. CNT=0 everywhere, then set CEN on TIM1, and all four start together. TRGO here is a level (CEN), not a pulse, so crossing from APB2 to APB1 carries no pulse-width risk. The residual skew is a few timer clocks plus filter synchronization, ≤ ~40 ns, and constant per build. The analyzer applies it as a fixed per-channel offset; it is documented, not fitted.
- **No sleep.** The core never sleeps (no WFI), so DWT CYCCNT keeps running.

### STEP: DMA, one stream each (RM0090 Tables 42/43)

| ch | request | DMA | stream | channel |
|---|---|---|---|---|
| a | TIM1_CH1 | DMA2 | S1 (alt S3) | 6 |
| b | TIM3_CH1 | DMA1 | S4 | 5 (shared with TIM3_TRIG; TDE stays 0) |
| z | TIM4_CH1 | DMA1 | S0 | 2 |
| e | TIM8_CH1 | DMA2 | S2 | 7 |

Each stream runs peripheral-to-memory in circular mode. It uses **direct mode**, not FIFO: a FIFO would hold captures back until a threshold. Transfers are half-word from CCR1 into a 4096-entry ring in SRAM1, not CCM, which the DMA cannot reach. 4 × 8 KB = 32 KB total. The DMA read of CCR1 clears CC1IF, so a second capture before that read sets **CC1OF**, which is sticky and is checked every tick.

### DIR: interrupt, not DMA

TIM1_CH2's only DMA paths are DMA2 S2 ch6, which collides with TIM8_CH1 on S2, and S6 ch0. S6 ch0 is the OR of TIM1_CH1/2/3, so a STEP request would also fire S6. Mapping the inputs the other way round (CCxS=10) cannot clear the collision either. Rather than mix DMA and ISR, all four DIR captures use the CC2 interrupt at the **highest** priority. The interrupt reads CCR2, then CNT, CYCCNT and the DIR pin level, pushes `(ccr2, cnt, level)` onto a per-channel SPSC queue, and treats CC2OF as a fault. DIR edges are rare. Even ripple reversals come at most once per step, so ISR load is negligible. Nothing in the firmware may mask this interrupt (no global `interrupt::free`). Shared state uses BASEPRI or lock-free queues instead.

## Extending 16-bit captures without ambiguity

A 10 kHz tick ISR (TIM6, lower priority than DIR) takes a snapshot in this fixed order:

1. Read the DMA write position (NDTR) for each stream, and the head of each DIR queue.
2. Read CYCCNT, then TIM1 CNT. That gives `E_k`, the extended 64-bit now.

`E_k` comes from `E_{k-1}` plus `(cnt_k − cnt_{k-1}) mod 2^16`. This is valid only if fewer than 65536 ticks passed between snapshots, and the CYCCNT delta checks exactly that: anything over 600 µs (100,800 cycles) faults with **TickLate**. CYCCNT is 32-bit at 168 MHz and is itself extended every tick, so it can never be ambiguous.

**Invariant:** every 16-bit value `v` is extended against a reference `(E, e16)` read at `T_ref ≥ T_capture` with `T_ref − T_capture < 2^16` ticks. Then `ext(v) = E − ((e16 − v) mod 2^16)` is exact.

- **STEP:** a capture taken from ring positions between snapshot k−1 and k was latched after snapshot k−1's NDTR read, less the DMA service latency. It was also latched before `E_k`. The window is one tick period plus the DMA latency. Two checks guarantee the latency is small:
  - CC1OF: the next capture would overrun.
  - **DmaStall**: CC1IF still set with the same CCR1 at two consecutive snapshots.
  
  That bounds the window at 200 µs or less, well under 780 µs.
- **DIR:** the ISR's own CNT read is a time inside (snapshot k−1, snapshot k], so it is extended against `E_k`. CCR2 is then extended against the ISR's read. The ISR's latency is bounded by **DirIsrLate**: CC2IF still set at two consecutive snapshots means the interrupt was masked for over 100 µs, which is a fault.

**Ring laps.** At most 25 edges fit in one 100 µs tick at the 4 µs minimum edge spacing. That is far below the 4096-entry ring, so each snapshot's NDTR advance is exact. The consumer (main loop) tracks how far it lags the producer from those advances. Getting within 512 entries of a lap faults with **ConsumerBehind**. HT/TC interrupts are not needed.

**Consistency check.** At every snapshot, the TIM3, TIM4 and TIM8 CNT must sit at a constant offset (±2 ticks) from TIM1. Anything else faults with **TimebaseDesync**, which catches a timer that stopped or never started.

Faults are latched and sent exactly as in count mode. The new codes are TickLate, DmaStall, DirIsrLate, ConsumerBehind, TimebaseDesync and OutputOverflow; StepOvercapture and DirOvercapture already have a slot in the fault scheme. Counts are exact by construction: every edge that passes the input filter is either delivered or raises a fault.

## DIR ordering

The firmware keeps at least `step_pulse_ticks` (2 µs on the bench) between unstep and a DIR toggle, and between that toggle and the next step (`src/stepper_classic.c:149-183`). A DIR edge therefore never falls within ~2 µs of a STEP rising edge, and merging by timestamp is unambiguous by a factor of ~170 over the resolution. The host merges each channel's STEP and DIR streams by time and signs each step with the DIR level in force at that moment. It **fails** if any DIR edge lies within 100 ns of a STEP edge. It also reports the minimum DIR→STEP and STEP→DIR separations, which is a free check of the TMC DIR setup time. The idle-load path (`stepper_classic.c:187-190`) toggles DIR without that spin. The check covers it as well.

## USB bandwidth, and whether a hybrid is needed

Peak rates are a/b 113 kHz, e 57 kHz and z 10 kHz (25 mm/s at 400 steps/mm). If all four peak at once, that is 293 k events/s. A realistic worst case is XY travel at 236 k events/s.

| encoding per event | all peak | XY peak |
|---|---|---|
| u16 interval | 586 KB/s | 472 KB/s |
| varint of zigzag 2nd difference (≈1–1.5 B) | ≤440 KB/s | ≤350 KB/s |

USB FS bulk tops out at 1.216 MB/s (19 × 64 B per frame). The polled `usbd-serial` on OTG FS has no DMA, so its real throughput is unmeasured. [INFERENCE] Typically 0.6–1.0 MB/s. Count mode needs only 160 KB/s.

Plan: per-channel second-difference varints, which are lossless and cost about 20 cycles per event. Add a 64 KB output ring that rides out a 0.3 s burst even at a 200 KB/s deficit. **No intra-run hybrid.** Counts come from the same complete event stream. If the output ring overflows, the run faults with **OutputOverflow**. It does not drop timestamps behind a "counts only" marker, because that would silently create a second truth. Count mode remains the run-level fallback for anything event mode cannot carry.

**Gate 0** (half a day): a throughput probe built into the same firmware streams a counter flat out while the host measures. Go if ≥600 KB/s is sustained for 60 s.

## Wire and run-dir format

Frames reuse the `SVC1` header layout with magic `SVE1`. The `ticks` field carries the payload length, and status/detail carry faults as today. Payload is a series of 1 ms blocks:

```
block:   u32 block_index | per channel: u8 n_step | u8 n_dir
         | step deltas (first: varint Δ from that channel's previous step, rest: zigzag-varint 2nd diff)
         | dir records (varint Δ from that channel's previous DIR edge, 1 byte new level)
```

Full 64-bit extension happens on the F407, where the proof lives. The host only adds deltas, starting from 0 at timer start, and checks that `seq`, `block_index` and the per-block counts are continuous. `capture.py` writes `events.npz` (per channel: `step_t` int64 ticks, `dir_t` int64, `dir_level` int8), and `capture.json` gets `mode: "event"` and `tick_hz: 84e6`. Count-mode runs are unchanged.

## Analyzer changes

- `capture_steps` gets an event-mode loader: exact times, levels signed by the merged DIR. The epoch, alignment and matching code already works on continuous times and stays as it is. Existing checks keep judging against the ideal reference, so compression error still counts there.
- New `mcu_execution` section, present when `step_tap.bin` exists:
  - index-paired lateness, captured − commanded mapped into capture time;
  - p50/p99/max relative to the run minimum (the constant ISR entry latency is not observable, see below);
  - step-interval jitter;
  - DIR→STEP separations;
  - a cross-reference to `STEP_LOAD_LATE` events. Informational first; thresholds come after the first runs.
- Calibration: `diag_stepper_buzz` (`src/stepper.c`) produces busy-waited steps at a known `period_ticks`. That validates resolution, inter-channel skew and the ppm fit with no planner involved.

## Time alignment to the F401 clock

**Without the clock wire.** Commanded step clocks are in F401 ticks, and captures are in F407 ticks. Both come from crystals, so the map between them is affine with slow drift. It has **no** per-epoch breaks: the host's per-epoch frozen frequency (+10 ppm seen on `epochs.gcode`) is already inside the commanded clocks. Fit one robust piecewise-linear map per run, with knots about every 2 s, on all paired steps. The cost: drift slower than a knot interval cannot be told apart from slow-changing MCU lateness, and sparse stretches (long rests, z/e only) are interpolated. The constant offset is absorbed, so lateness is reported relative to its minimum.

**With F401 PA8 (MCO1 = HSE 25 MHz) → F407 PA15 (TIM2_ETR, AF1).** TIM2 is 32-bit. In external-clock mode 2 with ETPS=/2 it counts 12.5 MHz, under the f_CK_INT/4 = 21 MHz limit. Each tick snapshot reads TIM2 CNT next to `E_k`. That gives 10 kHz samples of the F401↔F407 map, each with ±80 ns quantization and ns-level precision once averaged over 100 ms. What the wire adds:

- the drift is **measured, not fitted**, leaving exactly one free constant per run, so slow lateness trends stay visible;
- the map holds through rests and sparse axes;
- an independent check of the host clock-sync estimate, which can now be judged per re-anchor against the true crystal ratio;
- the ideal reference can be mapped exactly: stream time → F401 clock through the logged `reanchor_record` (`mcu_ref`, `clock_freq`) → F407.

It does not give absolute latency, because the counter origin is still unknown. This needs a small F401 change to enable MCO1 on PA8 (config validation already reserves the pin), plus the wire, which the user must add.

## Risks and gates

1. **Gate 0: USB throughput** (above).
2. **Gate 1: on-bench capture validation.** Buzz-calibration intervals must show ≤24 ns spread. An A/B run of the same gcode in count mode and event mode must give identical per-tick counts once the event stream is binned to 25 µs.
3. RM0090 behavior to confirm on the board, not just from the manual:
   - a DMA read of CCR1 clears CC1IF;
   - the trigger-mode start skew is constant from run to run.
4. Host step tap: production code in the stepcompress sink, opt-in, writing raw frames. It must never block the pump; on backpressure it fails loudly.

## Effort

| piece | estimate |
|---|---|
| Gate 0 throughput probe | 0.5 d |
| Firmware event mode: timers, sync, DMA, DIR ISR, extension, faults, encoder, framing | 2.5 d |
| `capture.py` format + `events.npz` | 0.5 d |
| Analyzer event ingest + calibration report | 1 d |
| Host commanded-schedule tap + analyzer `mcu_execution` | 2 d |
| Bench validation (Gate 1, A/B, buzz) | 1 d |
| **Total** | **≈7.5 agent-days** |
| Clock wire: F401 MCO + TIM2 ETR + map | +1–1.5 d, after the user wires it |

## Recommendation

**GO**, staged. Do Gate 0 first: if CDC sustains <600 KB/s, stop and revisit the encoding before writing capture code. Build the commanded-schedule tap together with the firmware, not after it. Without the tap, event mode mostly re-measures the ±25 µs compression budget and cannot isolate the 0.2–7 µs MCU errors that motivate this work. The clock wire is worth adding once event mode works. It improves trend and long-rest fidelity, but it is not needed for jitter.
