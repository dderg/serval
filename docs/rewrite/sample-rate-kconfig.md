# Motion sample rate — Kconfig

`CONFIG_MOTION_SAMPLE_RATE_HZ` controls the rate at which the TIM5 ISR
evaluates motion curves and emits step pulses on the MCU. It is consumed by
the stepping-redesign engine.

## Defaults

| MCU profile      | Default | Rationale (per-MCU step-rate ceiling)                  |
|------------------|---------|--------------------------------------------------------|
| `MACH_STM32H7`   | 40 kHz  | 520 MHz core; comfortably handles 40 kHz evaluator.    |
| `MACH_STM32F4`   | 20 kHz  | 180 MHz core; 20 kHz keeps ISR budget < 50 %.          |
| other / Linux    | 10 kHz  | Conservative fallback for the host simulator.          |

Range is clamped to `1000..=100000` Hz.

## Spec-drift resolution

The implementation plan's literal stanza used `depends on RUNTIME`, but
no such symbol exists in this codebase — the runtime sizing symbol is
`RUNTIME_STORAGE_SIZE` in `src/Kconfig`, which itself
`depends on MACH_STM32H7 || MACH_STM32F4 || MACH_STM32F1 || MACH_STM32G0 ||
MACH_LINUX`. We mirror that dependency on the new option so the symbol is
only visible on the same MCU/Linux set that has runtime sizing.
