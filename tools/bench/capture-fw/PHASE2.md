# Phase 2: the F407 as printer simulator

Status: implemented and host-tested (`sim-logic`: 57 tests; `tools/bench` pytest), firmware builds; not yet run on hardware (waits for the phase-2 wiring). RAM: 113 KB static of 128 KB (bss 108 KB + data 5 KB), about 15 KB left for the stack.

Wiring: `tools/bench/WIRING_PHASE2.md`. The F407 keeps capturing STEP/DIR (count mode `SVC1` and event mode `SVE1`, both unchanged on the wire) and adds four simulated peripherals: endstops/probe, thermistors fed by a thermal model of the heaters the F401 drives, a clock reference count, and four TMC2209 drivers on a single-wire UART. Pure logic lives in a separate `no_std` crate, `tools/bench/sim-logic`, tested on the host with `cargo test`. `capture-fw` holds only the register-level glue.

## Interrupts and timers

| source | priority | work |
|---|---|---|
| TIM1/3/4/8 CC2 (DIR, event mode) | 0x00 | unchanged |
| TIM6 tick (40 kHz count / 10 kHz event) | 0x40 | capture as before, then walls, MCO sample, aux record every 1 ms |
| TIM7, 10 kHz, always running | 0x80 | sample heater/fan inputs, thermal model at 1 kHz, DAC writes, TMC reply scheduling |
| USART2 (TMC) | 0x80 | receive and parse requests, send replies |

TIM7 and USART2 share a priority, so they never preempt each other and can share the TMC state without locks. The DAC uses software writes only (`TEN=0`, no DMA, no underrun interrupt), so the shared `TIM6_DAC` vector stays a pure TIM6 tick. TIM9 counts the MCO clock. It runs from boot and has no interrupt.

## 1. Endstops and probe (PD8–PD11)

- **Boot.** The pins are push-pull outputs driven low. Low means not triggered under the convention `full.cfg` uses (`endstop_pin: PA9` with no `!`, so it triggers high). The existing `printer.cfg` uses `^PA9`, also triggered high, so its runs are unaffected.
- **Walls** come from the host config. Each wall has a pin, an active level, integer coefficients over the four channel counts, a comparison (`>=` or `<=`) and a threshold in steps. That one form covers motor space (`a ≥ N`) and CoreXY toolhead space: x = (a+b)/2 becomes `a + b ≥ 2N`, and y = (a−b)/2 becomes `a − b ≥ 2N`. Several walls may share a pin, and the pin is the OR of them. The probe is a wall on z.
- **Positions** are absolute steps: `origin + Σ signed steps since capture start`. The host sends the origin, computed from the gcode's `SET_KINEMATIC_POSITION`, with the config. In count mode the positions come from the per-tick deltas. In event mode each snapshot signs every step by the DIR level in force at its timestamp (the DIR edges are merged in time order) before summing.
- **When.** Walls are evaluated in the TIM6 tick, after the capture work. Outside a capture every pin sits at its inactive level, because positions are only known while capturing.
- **Within a tick.** A pin is asserted for a tick if its wall was crossed at any moment during that tick, not only at the tick's end. A touch-and-return inside one tick therefore still fires the pin for one tick, as a real switch would fire for that moment.
  - Event mode is exact. The snapshot keeps every step with its timestamp and sign (up to `WALL_STEPS_PER_SNAPSHOT` = 32 per channel per 100 µs; more is the fault `WALL_STEPS_OVERFLOW`). The tracker replays all channels in time order and evaluates each wall after every step of a channel it involves. CoreXY walls see the true interleaving of a and b.
  - Count mode knows each channel's turning point (the position at its single allowed DIR reversal in the tick). That is exact for single-channel walls (z, probe, motor-space walls). CoreXY walls are judged at the tick end. Between ticks this only misses an x/y excursion that both starts and ends inside one 25 µs tick, at most 3 steps (≈ 40 µm at 0.0125 mm/step and the bench's maximum step rate), which is below count mode's time resolution anyway.
- **DIR/STEP ambiguity.** In event mode a DIR edge within `MIN_DIR_STEP_TICKS` (9 ticks, 107 ns) of a step makes that step's sign ambiguous. That is the fault `DIR_AMBIGUOUS`, the same bound the event decoder enforces on the host.
- **Latency bound.** A crossing is reflected on the pin at the end of the tick that holds the step that caused it, plus the ISR's run time before the GPIO write. A step at the exact snapshot boundary belongs to the snapshot whose DMA position includes it. Count mode: ≤ 25 µs + ≤ 3 µs. Event mode: ≤ 100 µs + snapshot processing (≤ 20 µs). Each pin's driven level appears in every aux record. The analyzer recomputes every record's levels from the captured steps and reports each transition with its toolhead position.

## 2. Thermistors (DAC1/DAC2 on PA4/PA5) and heater/fan inputs

**Decision: the thermal model runs in firmware.** The two choices were a host-side model streaming DAC setpoints, or a first-order model in firmware. Firmware wins:

- the loop is closed at the source (heater pin → temperature → DAC) at 1 kHz, with no USB round trip and no host process that must stay alive;
- it is deterministic and exercised by host tests;
- the thermistor is readable from boot. Klippy reads the ADC as soon as it starts, often with no capture running, and a value out of range shuts it down at once.

A model of a few seconds' time constant gains nothing from host-side flexibility that host-set parameters don't already give.

- **Inputs.** PD14 (extruder heater), PD15 (bed heater) and PE8 (part fan) are GPIO inputs with pull-downs, sampled at 10 kHz by TIM7. Klipper's soft PWM (100 ms cycle for heaters) is resolved exactly. The cumulative high-sample counters go into the aux record, so the host can compute duty over any window.
- **Model, per heater, at 1 kHz (explicit Euler; dt is 3 orders below any time constant):** `T += dt·(P·duty − (h + h_fan·fan)·(T − T_amb)) / C`.
- **Sensor.** Klippy's own beta-thermistor model with no inline resistor: `R = R25·exp(β(1/T − 1/T25))`, ADC ratio `R/(R + pullup)`. No physical pull-up exists, so `pullup_resistor` is just the assumption shared by `full.cfg` and the firmware. DAC code = `ratio·4095`.
- **Ranges.** The output buffer is on, because the F401 ADC loads it through 1 kΩ. That limits the output to about 0.2–3.1 V, so codes are clamped to ratio 0.06–0.94 and a clamp flag is reported. The `full.cfg` sensor (`β = 3000`, `R25 = 100 kΩ`, `pullup_resistor = 10 kΩ`) stays unclamped from 20 to 280 °C (ratio 0.922 at 20 °C, 0.088 at 280 °C; 0.909 at 25 °C).
- **Defaults** at boot (config id 0) match `full.cfg`: ambient 25 °C. Extruder: P = 40 W, C = 10 J/K, h = 0.15 W/K, h_fan = 0.15 W/K, giving τ ≈ 67 s and T_max ≈ 290 °C. Bed: P = 200 W, C = 300 J/K, h = 1.5 W/K, giving τ = 200 s and T_max ≈ 158 °C. The host may override them per run; the temperature carries over.
- **Accuracy.** DAC VREF and ADC VDDA come from separate 3.3 V regulators: about 1 % ratio error, roughly ±1 °C. Treat temperatures as ±1 °C or calibrate.

## 3. Clock reference (MCO on PE5 → TIM9)

- TIM9 is in external clock mode 1 (TI1FP1, PE5 AF3, filter N=2) and counts the F401's 6.25 MHz MCO1 (HSE/4) with no prescaler. It is sampled in every TIM6 tick and extended to 32 bits: it wraps every 10.5 ms and the tick is ≤ 100 µs, so extension is unambiguous. In event mode the sample is taken inside the same short critical section as the master timer read, so each `(time, mco)` pair is exact to the 160 ns MCO period.
- Every aux record carries the extended count. A flag is set while the 1 ms delta is 6250 ± 50. With `expect_mco` in the config, a delta out of range is the fault `MCO_LOST` (15).
- **Bench tooling.** `openocd-target-live.cfg` is `openocd-target.cfg` without `connect_assert_srst` (`reset_config none`). It attaches without halting or resetting. `deploy.py mco` writes the five `mmw` from `WIRING_PHASE2.md`, reads RCC_CFGR, GPIOA_MODER and GPIOA_AFRH back, and fails if the bits differ. It runs after `flash-target` and `reset-target`, and `run.py` runs it after `prepare_printer` (every FIRMWARE_RESTART resets the F401). The poke is a read-modify-write that races Klipper's own GPIOA writes only during MCU config; running it after the printer is ready avoids that.
- **Analyzer.** With MCO samples present, `mcu_execution` maps commanded F401 clocks to capture time through them: F401 clock = 13.44 × MCO count for the bench's 84 MHz PLL from a 25 MHz HSE, plus one constant. It reports `clock_reference: "mco"` and the measured crystal ratio. The piecewise clock fit becomes a cross-check instead of the reference.

## 4. TMC2209 UART emulation (PA2, USART2 half-duplex)

- **Line.** PA2 is AF7, open-drain with the internal pull-up, `HDSEL`, 8N1 at 40 000 baud (BRR = 42 MHz / 40 000 = 1050, exact).
- **Requests**, as encoded by `klippy/extras/tmc_uart.py`. Read: `[sync, addr, reg, crc]`. Write: `[sync, addr, reg|0x80, d3..d0, crc]`. Sync is `0xF5` from Klipper; only the low nibble `0x5` is checked. The CRC is CRC-8 (polynomial 0x07) over bits LSB-first. The parser resyncs on the USART IDLE flag and on any byte that cannot start a frame.
- **Drivers.** Addresses 0–3 each have a register file:
  - writes with a good CRC store the value and increment `IFCNT` (8-bit wrap); a bad CRC is ignored and counted;
  - `GSTAT` reads `reset = 1` after F407 boot and is write-1-to-clear (what `tmc.py`'s `start_checks` clears);
  - `IOIN` reports `VERSION = 0x21`;
  - `DRV_STATUS` reports no errors;
  - write-only registers keep their last value.
- **Replies** go out after `SENDDELAY` bit times from `SLAVECONF` (8 when the field is 0 or 1, otherwise 8·(2k+1)), quantised up to TIM7's 100 µs. Klipper's MCU starts looking for the start bit 4 bit times after its own stop bit and gives up after 64, so 8–24 bit times plus up to 4 more is inside its window. The reply is `[0x05, 0xFF, reg, d3..d0, crc]`, sent back to back from the TXE interrupt with no inter-byte gap (Klipper decodes all 80 bits as one stream). The receiver is disabled while transmitting, so the half-duplex echo is never parsed as a request.
- **Step edges.** `CHOPCONF.dedge` per address is in every aux record. The host passes the capture's both-edges mask from the run config (channels whose motor has a `[tmc2209]` section). The analyzer fails if the dedge state the F401 actually wrote disagrees with that mask.

## Host protocol

- **Config command** (accepted only while stopped; sending it stops any capture): `'C' | u16 len | payload`, with payload `u16 config_id | entries`. Each entry is `u8 tag | u8 len | body`, little-endian:
  - tag 1, origin: `i32[4]` (a, b, z, e);
  - tag 2, wall: `u8 pin | u8 flags (bit0 active high, bit1 trigger at or below) | i8 coeff[4] | i32 threshold`;
  - tag 3, heater: `u8 index | f32 power_w, capacity_j_per_k, loss_w_per_k, fan_loss_w_per_k, ambient_c, r25_ohm, beta, pullup_ohm`;
  - tag 4, expect MCO: `u8`.
  
  A malformed config is never applied. The firmware keeps the previous config and its id. The host checks the id in the aux records, so a rejected config fails the run.
- **Start commands** are `S` and `E` as before, plus `s`/`e`, which also stream aux records. Plain `S`/`E` streams are byte-identical to phase 1.
- **Aux frames** (`SVA1`, sharing the stream's `seq`). `position` is the first record index, `length` the record count. The payload is 44-byte records:

```
u64 time      count mode: tick index; event mode: master 84 MHz time of the snapshot
u32 mco       extended TIM9 count at that time
u32 samples   cumulative 10 kHz input samples
u32 high[3]   cumulative high samples: extruder heater, bed heater, part fan
i16 temp[2]   centi-°C
u16 dac[2]    codes written
u16 config_id
u8  endstops  driven levels, bit n = PD(8+n)
u8  flags     bit0 MCO delta in range, bit1/2 DAC0/1 clamped
u8  dedge     bit n = TMC address n has CHOPCONF.dedge
u8  tmc_writes, tmc_reads, tmc_rejects   wrapping counters
```

- **Files.** `capture.py` writes `aux.bin` next to `capture.bin`/`events.bin`, and `capture.json` gets `"aux": true`.
- **Run config.** `tools/bench/sim_config.py` builds the config from a JSON file (`configs/full.sim.json`): walls in toolhead mm or motor steps, heaters, `expect_mco`. `run.py --sim` sends it with the origin from the gcode, and `--config configs/full.cfg` selects the matching printer config. Existing runs without these flags are unchanged.

## Fail loudly

Every check below stops the run with a named error. None of them recovers or pads.

| where | condition | result |
|---|---|---|
| firmware | MCO delta outside 6250 ± 50 per ms with `expect_mco` | fault `MCO_LOST` (15) |
| firmware | aux ring full | fault `AUX_OVERFLOW` (16) |
| firmware | DIR edge within 9 ticks of a step (event mode) | fault `DIR_AMBIGUOUS` (17) |
| firmware | more than 32 steps on one channel in one snapshot | fault `WALL_STEPS_OVERFLOW` (18) |
| firmware | unknown command byte, config longer than 512 B | panic (board needs `reset-capture`) |
| capture.py | aux record gap, aux frame without a sim config | `CaptureError` |
| capture.py | aux records carry another config id (firmware rejected the config) | `CaptureError` |
| capture.py | a sim capture ends with no aux record | `CaptureError` |
| sim_config.py | unknown key, wall with neither or both of `at_mm`/`at_steps`, toolhead wall in steps, start or wall not on a whole step, unequal a/b steps, payload > 512 B | `SimConfigError` |
| sim_report.py | config id changes, MCO lost while expected | `SimReportError` |
| sim_report.py | a record whose 10 kHz input sample count is not 10 ± 1 (missed TIM7 interrupts) | `SimReportError` |
| sim_report.py | any thermistor DAC clamp during the capture | `SimReportError` |
| sim_report.py | any TMC reject during the capture (bad CRC, stray byte, overrun, request during a pending reply) | `SimReportError` |
| sim_report.py | dedge changes, is set on an address without a `[tmc2209]` section, or disagrees with the capture's both-edges mask | `SimReportError` |
| sim_report.py | endstop levels disagree with the walls evaluated on the captured positions for ≥ 2 consecutive records | `SimReportError` |
| analyze.py | count-mode aux record beyond the captured ticks | `AnalysisError` |

Kept as counters, not faults: TMC rejects outside a capture (klippy's own CRC/IFCNT retries surface a broken bus as a klippy error), and the firmware's silent keeping of the previous config on a malformed `C` payload (the host fails on the config id instead).

## Testing without hardware

- `sim-logic` (`cargo test` in `tools/bench/sim-logic`, also `./scripts/ci.sh bench`):
  - walls: CoreXY, both comparisons, OR per pin, polarity, overflow;
  - wall timing (`tests/wall_timing.rs`):
    - a crossing step that is the last step of a snapshot fires in that snapshot;
    - one just after the boundary fires in the next;
    - a touch-and-return inside one snapshot fires for exactly that snapshot;
    - CoreXY walls follow the cross-channel time order;
    - other channels' steps are ignored;
    - count-mode excursions fire single-channel walls only when they reach the wall;
    - count and event ticks agree on monotone motion;
  - DIR merge signing, plus ambiguity at < 9 ticks on either side, at the same tick, and none at exactly 9;
  - MCO extension across wraps;
  - thermal: step response against the analytic first-order solution, ratio against the beta formula. The clamp flag turns on exactly at the ratio limits (bisected temperature ± 0.01 °C), the limits bracket 20–280 °C, and a NaN temperature is flagged;
  - TMC:
    - frames byte-identical to `tmc_uart.py`;
    - IFCNT wrap at 256;
    - GSTAT write-1-to-clear;
    - bad CRC on a write or a read (no reply, counted);
    - a false sync byte costs exactly one frame and the retry after IDLE parses;
    - back-to-back frames with no gap;
    - overrun mid-frame and between frames;
    - wrapping reject counter;
    - SENDDELAY;
    - the scheduled reply starts no earlier than SENDDELAY and, for klippy's `senddelay=2`, ends its wait inside Klipper's 64-bit window;
  - config TLV with a golden payload shared with the Python encoder; aux record layout golden-tested against the Python decoder.
- Python (`uv run python -m pytest tools/bench`):
  - TMC golden frames and replies checked against klippy's real `tmc_uart.py` encoder;
  - the thermistor ratios checked against klippy's `Thermistor` class;
  - `sim_config`: golden payload, CoreXY coefficients, dir inversion, strict keys, whole-step starts;
  - `aux.bin` decoding and MCO unwrap;
  - every `sim_report` fatal check;
  - wall-level recomputation: agreement, one-record hold tolerated, two-record disagreement fatal;
  - capture positions for count and event records;
  - MCO clock mapping under crystal wander that the step fit cannot follow.

## Hardware validation plan

Run each step in order. Stop at the first failure and fix it before going on. Every command runs under the bench lock after a sync:

```
B='uv run python tools/bench'
uv run python tools/bench/lock.py hold -- bash -c "$B/deploy.py sync && <command>"
```

`<command>` is written below with `$B` for `uv run python tools/bench`. Captures write to `/tmp/sim-probe-*` unless `--out` is given. `sim_probe.py` prints one JSON document per call.

**V0. Flash and phase-1 regression** (printer still on `printer.cfg`).
- `$B/deploy.py flash-capture`
- `$B/capture.py /tmp/p2-v0c --seconds 5` → prints `≈200000 ticks`. Pass: 200 000 ± 1 %, no fault.
- `$B/capture.py /tmp/p2-v0e --seconds 5 --events` → pass: no fault.
- `$B/run.py tools/bench/gcode/epochs.gcode --events` → pass: analysis completes; per-motor step counts equal the last phase-1 run of the same gcode; event-mode lateness within run-to-run spread of the phase-1 runs. The F407 now drives PD8–PD11 low, and `printer.cfg`'s `^PA9` endstops read not-triggered.

**V1. `full.cfg` boots: TMC init and thermistors in range.**
- `$B/deploy.py config --config tools/bench/configs/full.cfg` → pass: prints `printer ready`. Klippy refuses to become ready if any TMC2209 read fails (`Unable to read tmc uart`), if GSTAT/DRV_STATUS reports an error, or if an ADC is out of range. The F401 resets on FIRMWARE_RESTART, so then run:
- `$B/deploy.py mco` → pass: exits 0 (the five registers read back as written).

**V2. Aux stream and MCO at idle.**
- `$B/sim_probe.py capture --seconds 10 --start 100,100,10`
- Pass:
  - `simulator.mco.records_ok == simulator.mco.records` (≈ 10 000) and `|f401_vs_f407_ppm| < 100`;
  - `input_duty` all 0;
  - both heaters `start_c`/`end_c` 25.00 ± 0.05;
  - `tmc.dedge_channels_mask == 15` and `tmc.rejects == 0`;
  - `endstop_transitions == 0` and `walls.mismatched_records == 0`;
  - `walls.final_toolhead_mm == klippy_position == [100, 100, 10]`.
- Repeat with `--events`. Pass: the same, and the ppm agrees with the count-mode value within 2 ppm.

**V3. Thermistor chain at ambient.**
- `$B/sim_probe.py query extruder heater_bed` → pass: both `temperature` 25 ± 1.5 °C. The tolerance is the two-regulator VREF/VDDA error. Record the offset; V7 compares against it.

**V4. TMC register traffic.**
- `$B/sim_probe.py gcode "DUMP_TMC STEPPER=a"` → pass:
  - `IOIN` shows `version=0x21`;
  - `GSTAT` is 0 (klippy cleared the reset bit);
  - `CHOPCONF` has `dedge=1`;
  - `IFCNT` is non-zero.
- Repeat for `b`, `z` and `extruder` (addresses 1–3).
- `$B/sim_probe.py gcode "SET_TMC_CURRENT STEPPER=z CURRENT=0.6"` → pass: no error. Klippy verifies the write through IFCNT, so this exercises a write and a read-back.
- `$B/sim_probe.py query "tmc2209 a"` → pass: `run_current` 0.8 (± the TMC current-scale quantisation) and an empty `drv_status`.

**V5. Both-edge capture and the MCO clock reference.**
- `$B/run.py tools/bench/gcode/epochs.gcode --config tools/bench/configs/full.cfg --sim tools/bench/configs/full.sim.json`
- `$B/run.py tools/bench/gcode/epochs.gcode --config tools/bench/configs/full.cfg --sim tools/bench/configs/full.sim.json --events --step-tap`
- Pass:
  - both analyses complete. A both-edges mismatch shows as doubled or halved step counts, or as the dedge-vs-mask fatal error;
  - `simulator.tmc.dedge_channels_mask == 15` and `simulator.walls.mismatched_records == 0`;
  - `mcu_execution.clock_reference == "mco"`, with `lateness_ns.p50` within 50 ns of `fit_lateness_ns.p50`, and `simulator.mco.f401_vs_f407_ppm` within 2 ppm of V2.

**V6. Endstops, homing and probe.**
- Count mode: `$B/sim_probe.py capture --seconds 90 --start 50,50,10 --gcode G28 --gcode "G1 X20 Y20 Z5 F3000"`
- Event mode: the same with `--events`.
- Pass:
  - every `gcode` entry has no `!!` error (no `No trigger on x after full movement`);
  - `klippy_homed_axes == "xyz"`;
  - `klippy_position == [20, 20, 5]`;
  - `simulator.walls.final_toolhead_mm` within 0.02 mm of `[20, 20, 5]` on every axis. The homing trigger error is latency × homing speed (≤ 128 µs × 20 mm/s = 2.6 µm) plus half a step of CoreXY quantisation (6 µm);
  - `endstop_transition_examples`: each asserting edge of a homed pin is at that axis ≈ 0 (within 0.02 mm);
  - `walls.mismatched_records` ≤ the number of transitions.
- Probe: `$B/sim_probe.py capture --seconds 60 --start 50,50,10 --gcode G28 --gcode "G1 Z5" --gcode PROBE`. Pass: the PROBE response reports `z=` within 0.01 of 0.000.

**V7. Heater control.**
- `$B/sim_probe.py heat --heater extruder --target 60 --seconds 240` → pass:
  - `klippy_plateau_c` 60 ± 1;
  - `klippy_minus_model_c` equal to V3's ambient offset ± 0.5 °C;
  - `klippy_max_c` ≤ 65;
  - `simulator.input_duty.extruder_heater` > 0, and no clamp or sample error (both fatal).
- `$B/sim_probe.py heat --heater bed --target 50 --seconds 600` → the same criteria with the bed fields.
- `$B/sim_probe.py capture --seconds 16 --start 100,100,10 --gcode "M106 S128" --gcode "G4 P15000" --gcode M107` → pass: `input_duty.part_fan` 0.45–0.50 (half duty for 15 of the 16 s).

After V7: restore `printer.cfg` with `$B/deploy.py config`, then open the PR.
