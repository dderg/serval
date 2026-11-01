# Phase 2: the F407 as printer simulator

Status: implemented and host-tested (`sim-logic` 34 tests, `tools/bench` pytest), firmware builds; not yet run on hardware (waits for the phase-2 wiring). RAM: 108 KB static of 128 KB, about 17 KB left for the stack.

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
- **Latency bound.** A crossing is reflected on the pin at the first tick after the step that caused it, plus the ISR's run time before the GPIO write. Count mode: ≤ 25 µs + ≤ 3 µs. Event mode: ≤ 100 µs + snapshot processing (≤ 20 µs). Each pin's driven level appears in every aux record. The analyzer reports each transition with its capture time (1 ms aux resolution).

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

## Testing without hardware

- `sim-logic` (`cargo test` in `tools/bench/sim-logic`):
  - wall evaluation, CoreXY, both comparisons, OR per pin;
  - DIR merge signing in event mode;
  - MCO extension across wraps;
  - thermal step response against the analytic first-order solution;
  - ratio/clamp against the beta formula;
  - TMC: frames encoded exactly as `tmc_uart.py` does, golden CRCs, IFCNT, GSTAT W1C, SENDDELAY, resync, bad-CRC rejection, reply bytes that `tmc_uart.py`'s `_decode_read` accepts;
  - config TLV decoding, including a golden payload shared with the Python encoder test;
  - aux record encoding, golden-tested against the Python decoder.
- Python: `sim_config` encoding, `aux.bin` decoding, analyzer aux report and MCO mapping on synthetic data.
- Hardware validation (once wired), in order:
  1. aux stream and MCO flag at idle;
  2. temperatures at ambient in klippy with `full.cfg`;
  3. TMC init (klippy loads, `DUMP_TMC` reads registers, dedge set);
  4. both-edge capture;
  5. a homing run against walls;
  6. heater PID to 60 °C.
