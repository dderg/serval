# Bench wiring, phase 2

Adds endstops, probe, thermistors, heater/fan observation, a clock reference and a TMC UART line between the F401 (printer MCU, black-pill layout) and the F407 (DevEBox STM32F4XX_M, capture/simulator board). Phase 1 (STEP/DIR, SWD, NRST, 2×GND) stays as in `README.md`.

Pin positions: F401 = header side + position counted from the top of the pinout image (left: VB C13 C14 C15 R A0 A1 A2 A3 A4 A5 A6 A7 B0 B1 B2 B10 3V3 G 5V; right: 3V3 G 5V B9 B8 B7 B6 B5 B4 B3 A15 A12 A11 A10 A9 A8 B15 B14 B13 B12). F407 = stm32-base numbering (Header 1/2 pin #); match by the silkscreen name when in doubt.

| # | Signal | Dir | F401 pin (header, pos) | F407 pin (header, #) | 1 kΩ |
|---|---|---|---|---|---|
| 1 | X endstop | F407→F401 | PA9 (R, 15) | PD8 (H2, 7) | yes |
| 2 | Y endstop | F407→F401 | PA10 (R, 14) | PD9 (H2, 10) | yes |
| 3 | Z endstop | F407→F401 | PA15 (R, 11) | PD10 (H2, 9) | yes |
| 4 | Probe | F407→F401 | PB0 (L, 14) | PD11 (H2, 12) | yes |
| 5 | Thermistor 1 (DAC1) | F407→F401 | PA4 (L, 10) ADC1_IN4 | PA4 (H1, 30) DAC_OUT1 | yes |
| 6 | Thermistor 2 (DAC2) | F407→F401 | PA5 (L, 11) ADC1_IN5 | PA5 (H1, 29) DAC_OUT2 | yes |
| 7 | Extruder heater duty | F401→F407 | PB4 (R, 9) | PD14 (H2, 13) | no |
| 8 | Bed heater duty | F401→F407 | PB5 (R, 8) | PD15 (H2, 16) | no |
| 9 | Part fan duty | F401→F407 | PB3 (R, 10) | PE8 (H1, 21) | no |
| 10 | Clock reference | F401→F407 | PA8 (R, 16) MCO1 | PE5 (H1, 41) TIM9_CH1 (proposed; see below) | no |
| 11 | TMC UART, single wire | F401↔F407 | PB10 (L, 17) | PA2 (H1, 32) USART2_TX half-duplex | yes |
| 12 | Extra GND (analog return) | — | G (L, 19) | GND (H1, 3) | no |

## Checks against the sources

- F407 headers: PD8–PD11, PD14, PD15, PE5, PE8, PA2, PA4, PA5 carry nothing on the board (stm32-base: user LED PA1, K1 PA0, SD PC8–12/PD2, TFT PB12–15/PC5/PB1, flash PA15/PB3/PB4/PB5, USB PA11/PA12, SWD PA13/PA14). The existing capture uses PE9, PE11, PA6, PA7, PD12, PD13, PC6, PC7, TIM1/3/4/8 and TIM6 (40 kHz tick, `TIM6_DAC` vector). No new pin is on those.
- F401 pins (pinout image): PA4 ADC4, PA5 ADC5, PB0 ADC8, PA8 CK1/MCO1, PB10 SCL2/SCK2, PB3/PB4/PB5 free GPIO. None is used by the board (LED PC13, USB-C PA11/PA12, key BOOT0/NRST). PB2 (BOOT1) stays untouched.
- PA9/PA10 are OTG_FS VBUS/ID alternates, but `src/stm32/usbotg.c:497` sets `NOVBUSSENS`, so both are free as GPIO (the bench `printer.cfg` already uses PA9/PA10/PA15 as endstops).
- PA15, PB3 and PB4 are JTDI, JTDO and NJTRST at reset. Klipper's `gpio_out_setup`/`gpio_in_setup` rewrites MODER for the pin, which overrides the debug AF; the bench only needs SWD (PA13/PA14). Expect PA15 and PB4 to read high (reset pull-ups) until firmware configures them.

## Pins I would change or flag

1. **Clock: PE5 instead of PA15 (recommended).** PA15 is the DevEBox W25Q16BV flash /CS. It works only if the F407 keeps the flash idle (PB3 SCK driven low, PB5 low, PB4 input) because /CS would toggle at the MCO rate. PE5 is TIM9_CH1 (external-clock mode on a free timer, 16-bit is enough: at 40 kHz sampling, 6.25 MHz is ~156 counts per tick) and has nothing on the board. If you keep PA15, use TIM2_CH1/ETR there.
2. **MCO needs firmware, not just a wire.** Klipper has no MCO1 setup (`grep MCO src/stm32` is empty); the F401 build needs a small addition (RCC_CFGR MCO1 = HSE 25 MHz, prescaler /4 → 6.25 MHz on PA8). `pi.py` already forbids PA8 in configs.
3. **TMC UART is bit-banged.** The F401 has no USART3, so PB10 is not a hardware UART pin; Klipper's `tmc_uart` bit-bangs any GPIO at 40 kbaud (`TMC_BAUD_RATE`), so PB10 is fine with `uart_pin: PB10` and no `tx_pin`. The F407 side uses USART2 with `HDSEL` and PA2 as open-drain with the internal pull-up; the 1 kΩ keeps the two push-pull drivers from fighting.
4. **DAC/ADC reference.** F407 DAC uses VREF+ and the F401 ADC uses VDDA, both 3.3 V from separate regulators: a ~1 % rail difference is a ~1 % ratio error. Calibrate once or treat thermistor temperature as ±1 °C. Add the extra GND (row 12) next to PA4/PA5.
5. **TIM6_DAC shares a vector.** The capture tick uses `TIM6_DAC`; enable the DAC with software writes only (no DAC DMA/underrun interrupt), so the shared vector stays a pure TIM6 tick.
6. **PD14/PD15 stay GPIO.** They are TIM4_CH3/CH4 and TIM4 already counts z STEP on PD12/PD13; sample them from the 40 kHz tick as plain inputs, do not set an AF. Same for PE8 (TIM1_CH1N next to the a-step pin PE9).
7. **F407 outputs must be driven before homing.** PD8–PD11 float until the capture/sim firmware configures them; set them to the "not triggered" level at boot.

After wiring, `printer.cfg` changes: `endstop_pin` stays PA9/PA10/PA15, add the probe on PB0, heater/fan pins PB4/PB5/PB3, thermistor `sensor_pin` PA4/PA5 with a matching pullup assumption, `[tmc2209 …] uart_pin: PB10`.

## Checklist

- [ ] 12 jumpers: 11 signals + 1 extra GND
- [ ] 7 × 1 kΩ in series: rows 1–6 and 11
- [ ] 5 direct jumpers: rows 7–10 and 12
- [ ] Nothing moved on PA13/PA14/NRST, PB6–PB9, PB12–PB15
- [ ] Both boards stay at 3.3 V logic (no 5 V pin connected between them)
