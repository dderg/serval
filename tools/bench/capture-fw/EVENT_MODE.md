# Event mode: sub-µs step timestamps

Status: event mode (firmware, `capture.py --events` / `run.py --events`, SVE1 decoder `tools/bench/capture_events.py`, analyzer ingest, `buzz.py`) passed Gates 0 and 1 and is merged. The host step tap (`rust/motion-core/src/pump/step_tap.rs`, `run.py --step-tap`) and the analyzer's `mcu_execution` section (`tools/bench/step_tap.py`, `tools/bench/mcu_execution.py`) are implemented and bench-validated (results below). Count mode (`SVC1`, 40 kHz i8 deltas) is unchanged; event mode is a second mode, chosen per run with the start command (`E` + both-edges mask).

## Goal

Timestamp every STEP and DIR edge on all four channels at ≤100 ns resolution, keep the step count exact, and fail loudly whenever exactness cannot be proven. This exposes MCU execution errors (`classic move loaded behind its pending unstep`, 0.2–7 µs, `EVENT_LOG_EVENT_MOTION_STEP_LOAD_LATE`) that the 25 µs tick hides today.

## The reference problem: precise capture alone is not enough

The F401 does not execute the ideal trajectory. It executes the classic `queue_step` schedule. That schedule is compressed with `stepcompress_max_error` = 25 µs (`klippy/mcu.py:30`; `max_error_ticks` in `rust/motion-core/src/pump/stepcompress_sink.rs:150-169`). Measured against `analyze.py`'s ideal reference, a perfect capture still shows compression error. Gate 1 measured how much: with exact timestamps, error against the ideal reference is p99 2.6–6 µs and abs_max 23–29 µs. The 25 µs budget sets the tail; the 13–25 µs p99 of count mode was mostly the 25 µs tick. Compression error and the 0.2–7 µs MCU execution errors are the same order of magnitude, so separating them needs the **commanded** step clocks:

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

Each stream runs peripheral-to-memory in circular mode. It uses **direct mode**, not FIFO: a FIFO would hold captures back until a threshold. Transfers are half-word from CCR1 into a 1024-entry ring in SRAM, not CCM, which the DMA cannot reach (4 × 2 KB). The DMA read of CCR1 clears CC1IF, so a second capture before that read sets **CC1OF**, which is sticky and is checked every tick.

### DIR: interrupt, not DMA

TIM1_CH2's only DMA paths are DMA2 S2 ch6, which collides with TIM8_CH1 on S2, and S6 ch0. S6 ch0 is the OR of TIM1_CH1/2/3, so a STEP request would also fire S6. Mapping the inputs the other way round (CCxS=10) cannot clear the collision either. Rather than mix DMA and ISR, all four DIR captures use the CC2 interrupt at the **highest** priority. The interrupt reads CCR2, then CNT and the DIR pin level. It checks that the level actually toggled (**DirDesync** otherwise), pushes `(ccr2, cnt, level)` onto a per-channel SPSC queue, and treats CC2OF as a fault. DIR edges are rare. Even ripple reversals come at most once per step, so ISR load is negligible. The only interrupt mask in event mode is a few cycles around the snapshot's clock reads.

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

**Ring laps.** Snapshot processing (unwrap and encoding) runs inside the tick ISR, so the consumer is always at the last snapshot. An advance of half the ring or more faults with **DmaLap**. The stream's HT/TC flags are checked against the boundaries the advance predicts. A predicted crossing whose flag is missing, or a raised flag the advance doesn't explain (except a crossing within 4 entries of the NDTR read, which is kept for the next tick), is also DmaLap. So any lap, however many, is caught.

**Consistency check.** At every snapshot, the CYCCNT delta must equal twice the TIM1 tick delta within 128 cycles, and the TIM3, TIM4 and TIM8 CNT must sit at their start offset from TIM1 within 32 ticks. Anything else faults with **TimebaseDesync**, which catches a timer that stopped or slipped. Every anchor record carries the worst value of both since the previous anchor. On the bench they peaked at 5 cycles and 5 ticks, which is read-to-read bus jitter. A first build used a 4-tick slack and faulted spuriously.

Faults are latched and sent exactly as in count mode. Codes 5–14: StepOvercapture, TickLate, DmaStall, DirIsrLate, DmaLap, TimebaseDesync, OutputOverflow, DmaError (DMA TE/DME), DirQueueFull, NonMonotonic (an extended time not after the previous one on its channel). DirOvercapture and DirDesync reuse codes 1 and 3. Counts are exact by construction: every edge that passes the input filter is either delivered or raises a fault.

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

**Gate 0: passed.** Firmware mode `T` streams SVT1 frames (528 B, payload byte = low byte of seq) through the stock `usbd-serial` 128 B buffer; `tools/bench/cdc_throughput.py` checks every frame. Measured 830.8 KB/s sustained over 60 s (94,412 frames, every second 830.0–831.6 KB, no seq gap, no corruption), 1.4× the all-axes peak with u16 intervals and ~1.9× with second-difference varints.

## Wire and run-dir format

Frames reuse the `SVC1` header layout with magic `SVE1`. `position` is the payload's offset in one continuous byte stream, `length` its size, and status/detail carry faults as today. The stream starts with the initial DIR level mask and the both-edges mask, then one block per 100 µs snapshot: a channel mask, then for each active channel a step count, a DIR count, the steps as zigzag varints of the interval's change, and the DIR edges as a varint delta plus the new level. An anchor (u64 master time, plus the drift/skew diagnostics) is added every 1000 blocks. The exact grammar is the docstring of `tools/bench/capture_events.py`, which decodes it.

Full 64-bit extension happens on the F407, where the proof lives. The host only adds deltas starting from 0 at timer start, and checks that `seq` and `position` are continuous. The decoder checks monotonic intervals, alternating DIR levels, edges before their block's anchor, and STEP/DIR separation of at least 9 ticks. A tail cut mid-block at stop is reported as `truncated_tail_bytes`. `capture.py --events` writes the raw stream to `events.bin`, with `capture.json` `mode: "events"` and `tick_hz: 84000000`. Count-mode runs are unchanged.

## Analyzer changes

- **Done:** `load_capture` decodes `events.bin`. It builds the 25 µs binned deltas, so every count-based check works unchanged, and `capture_steps` uses the exact times and DIR-signed levels. Existing checks keep judging against the ideal reference, so compression error still counts there.
- **Next (needs the tap):** a `mcu_execution` section, present when `step_tap.bin` exists:
  - index-paired lateness, captured − commanded mapped into capture time;
  - p50/p99/max relative to the run minimum (the constant ISR entry latency is not observable, see below);
  - step-interval jitter;
  - DIR→STEP separations;
  - a cross-reference to `STEP_LOAD_LATE` events. Informational first; thresholds come after the first runs.
- **Done:** `tools/bench/buzz.py` runs `DIAG_STEPPER_BUZZ` (`src/stepper.c`; `[diag_stepper]` is now in the bench `printer.cfg`), which makes busy-waited steps at a known `period_ticks`. That checks resolution and the crystal ratio with no planner involved.

## Time alignment to the F401 clock

**Without the clock wire.** Commanded step clocks are in F401 ticks, and captures are in F407 ticks. Both come from crystals, so the map between them is affine with slow drift. It has **no** per-epoch breaks: the host's per-epoch frozen frequency (+10 ppm seen on `epochs.gcode`) is already inside the commanded clocks. Fit one robust piecewise-linear map per run, with knots about every 2 s, on all paired steps. The cost: drift slower than a knot interval cannot be told apart from slow-changing MCU lateness, and sparse stretches (long rests, z/e only) are interpolated. The constant offset is absorbed, so lateness is reported relative to its minimum.

**With F401 PA8 (MCO1 = HSE 25 MHz) → F407 PA15 (TIM2_ETR, AF1).** TIM2 is 32-bit. In external-clock mode 2 with ETPS=/2 it counts 12.5 MHz, under the f_CK_INT/4 = 21 MHz limit. Each tick snapshot reads TIM2 CNT next to `E_k`. That gives 10 kHz samples of the F401↔F407 map, each with ±80 ns quantization and ns-level precision once averaged over 100 ms. What the wire adds:

- the drift is **measured, not fitted**, leaving exactly one free constant per run, so slow lateness trends stay visible;
- the map holds through rests and sparse axes;
- an independent check of the host clock-sync estimate, which can now be judged per re-anchor against the true crystal ratio;
- the ideal reference can be mapped exactly: stream time → F401 clock through the logged `reanchor_record` (`mcu_ref`, `clock_freq`) → F407.

It does not give absolute latency, because the counter origin is still unknown. This needs a small F401 change to enable MCO1 on PA8 (config validation already reserves the pin), plus the wire, which the user must add.

## Host step tap

Gives the analyzer, for every stepcompress oid, the exact F401 clock of every commanded step and the DIR in force, in execution order.

- **Where.** Every classic step frame leaves the host through one closure per MCU (`host_io_egress`, a `FrameEgress` calling `McuHostIo::send_args_batch`). When `SERVAL_STEP_TAP_DIR` is set, `build_endpoint` opens a `StepTap` and `StepcompressEndpoint::attach_tap` wraps that closure: a batch is recorded only after the inner send returns `Ok`. Unset, nothing is built and the egress is not wrapped; the endpoint's only other touch points are three `if let Some(tap)` checks (view intake, `abort_outbound`, `reset_motor_position`).
- **Records** (`step_tap_mcu<N>.bin`): magic `SVTAP1\0\0`, then 32-byte records `kind u8 | flags u8 | u16 | oid u32 | the endpoint's projected mcu clock when recorded u64 | 16-byte payload`. Header: `mcu` (mcu id, clock freq) and one `lane` per oid (axis index, motor index within it, microstep distance, invert flag) — the mapping the Rust side holds; it carries no stepper names, so the analyzer maps axis index → capture role (a, b, z, e on the bench corexy) and the exact per-motor count match verifies it. Frames: `reset_step_clock`, `set_next_step_dir`, `queue_step`, `queue_step_hp`, `stepcompress_barrier`, `stepcompress_set_position`; `kalico_wire_probe` is skipped and any other name is fatal. `projection` records the epoch map `clock = offset + stream_t × freq` whenever an oid's views move to a new one (each reanchor), and `halt` marks `abort_outbound` / `reset_motor_position`.
- **Fails loudly, never drops.** Records go through a bounded channel (65,536) to a writer thread that flushes after 100 ms idle. A full channel, a stopped writer or an unrecognised frame latches a fault that the next egress call returns as a fatal `SendError`.
- **Replay** (`step_tap.py`): Klipper semantics from `src/stepper_classic.c` — `reset_step_clock` low 32 bits extended to the wrap nearest the record's projected clock; `queue_step` step k at `last + (k+1)·interval + add·k(k+1)/2`. High-precision frames are refused; a halt while that oid still had steps queued past the halt clock is fatal (the discarded tail is unknown).
- **Analyzer** (`mcu_execution.py`, `analysis.json` → `mcu_execution`): pairs commanded and captured steps by index per motor (count mismatch fatal; DIR must agree up to one constant inversion), fits one robust piecewise-linear F401→F407 map (2 s knots) on *isolated* steps only, and reports lateness relative to its p0.1 floor (all / isolated / coincident), interval error, DIR setup, and `motion.step_load_late` events matched to the commanded step by `min_next + behind`.

## Risks and gates

1. **Gate 0: USB throughput** (above).
2. **Gate 1: on-bench capture validation — passed.**
   - **Buzz** (`buzz.py --motor a --steps 2000 --period-us 100`, run `buzz-20261101-225557`): 2000 of 2000 steps captured. Intervals fall in two clusters, {8396, 8397} and {8401, 8402} ticks. Each cluster is two adjacent 11.9 ns ticks wide, which is the instrument's quantization. The 60 ns gap between clusters is the F401 busy-wait loop's granularity (about 5 F401 cycles). The planned "≤24 ns spread" criterion was ill-posed, because the source itself has 60 ns steps. The mean interval gives the crystal ratio: −69.7 ppm against nominal.
   - **A/B, `epochs.gcode`:** the count-mode run (`20261101-230523`) and the event-mode runs (`230256`, `230715`) match the same number of steps on every motor (a 100591, b 78249, z 1040, e 14227), with the same rest ties. All three are ok. Event mode gives timing against the ideal reference of p99 3.2–6.0 µs and abs_p50 0.9–1.9 µs. Count mode gives p99 14–18 µs and abs_p50 6.4–7.3 µs.
   - **Peak rates:** `speed_ladder` in event mode (default config `230331` and `fast.cfg` `230753`) captured 1.6 M steps each on a and b at a 114 kHz peak. The stream peaked at 212 KB/s per 100 ms (about 0.93 B per step), a quarter of the measured 830 KB/s. `extrude_ladder` (`230929`) peaked e at 57 kHz. Every run was ok, with no faults, no truncated tail, timebase drift ≤5 cycles, skew wander ≤5 ticks, and minimum DIR↔STEP separation ≥407 ticks (4.8 µs).
3. RM0090 behavior seen on the board: a DMA read of CCR1 clears CC1IF, since no StepOvercapture fired over 3.3 M steps. The start skew is stable within a run (wander ≤5 ticks, which is read jitter). Its absolute value between channels is still not measured.
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

## Tap results (`--events --step-tap`, runs `20261101-233329` epochs, `233412` speed_ladder, `233610` voron_first_layers)

- **Exact pairing.** Commanded and captured counts matched on every motor of every run (epochs a 100592, b 78250, z 1040, e 14227; speed_ladder a 1634822, b 1628256; first_layers a 888984, b 618418, z 880, e 193978), with DIR consistent throughout. All three runs also passed the ideal-reference checks (`ok`).
- **Isolated steps** (no other motor's step commanded within 3 µs): lateness p50 8–32 ns, p99 0.66–1.55 µs, p99.9 0.77–2.4 µs, max 3.5–10 µs. Interval error p50 2–6 ns.
- **Coincident steps are serialized.** When a and b steps are commanded within 1 µs the MCU fires a first and b about 1.15–1.65 µs later (speed_ladder: 716 k coincident pairs, b p50 1.65 µs, a p50 ≈ 0; epochs b p50 0.91 µs, z p50 2.8 µs). That is real step-time error the 25 µs bench could not see; pure X/Y moves on CoreXY hit it on every step. The first fit (all steps) was dragged by it (speed_ladder inliers 55%), which is why the map is now fitted on isolated steps.
- **`STEP_LOAD_LATE`.** Only speed_ladder logged them: 5 events, all matched to commanded clocks where a and b coincide (the event carries no oid, so both are candidates), behind 1.9–6.9 µs, measured lateness of the matched steps 2.9–8.4 µs — each at or above its event's `behind`, consistent with the load-late clamp plus serialization.
- **DIR setup** (DIR edge → next step): ≥1.59 ms on every run; reversals happen at rest, nowhere near the 2 µs settle.

## Recommendation

Event mode and the tap are done. Next candidates: the F401 clock wire (measured instead of fitted drift; one constant per run), and a look at the coincident-step serialization (p50 0.9–1.7 µs on the later motor), which is a firmware scheduling property worth deciding on deliberately.
