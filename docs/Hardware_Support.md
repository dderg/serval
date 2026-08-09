# Hardware support and compatibility

This page is the concise hardware gate for the Serval motion path. It summarizes [Feature status](Feature_Status.md), which remains the authoritative detailed record of verification and limitations. “Builds” does not mean “safe to print.”

## Before connecting or flashing

1. Back up a known-good configuration and firmware image.
2. Verify the microcontroller family, not just the printer model or board marketing name.
3. Build the host native modules and firmware from the **same Serval checkout** and flash every participating MCU together.
4. Configure `LimitMEMLOCK=infinity` and remove disk swap pressure as described in [Installation](Installation.md#host-memory-requirements).
5. Treat the first motion and print on a new transport, board, or drive type as bring-up: stay with the machine and use conservative limits.

## MCU targets

| Target | Motion-path status | Notes |
| --- | --- | --- |
| STM32 F4 | Print target | Supported family; select the exact supported part in `make menuconfig`. |
| STM32 G0 | Print target, constrained | Supported family. A G0B1 toolhead has been validated for one streamed axis at the default 2 kHz, but not three; do not extrapolate that result to a multi-axis controller. |
| STM32 H7 | Print target | Supported family. |
| STM32 F1 (high density) | **Not supported for printing** | It builds and boots but lacks an FPU; the streamed tick exceeds practical timing budget. Do not use it for a printer. |
| Linux-process MCU / simulator | Development/integration target | Useful for simulation and testing, not a replacement for target-board validation. |
| AVR, LPC176x, RP2040, SAMD, HC32, STM32 F0/F2/F7/L4/G4 | Unsupported on this branch | Their inherited documentation may exist, but the Serval trajectory runtime is not provided for them. |

The menu presents the actual target choices. If a board is not listed, stop rather than attempting an upstream configuration recipe. Firmware execution is part of the motion architecture, so unsupported silicon cannot be recovered by a host-only setting.

## Drives and execution modes

| Capability | Verification status | Operational note |
| --- | --- | --- |
| Step/dir, trajectory-piece execution | Solid | Normal Serval path; size mechanics and limits conservatively. |
| Phase stepping | Verified in simulator | Not recently exercised on hardware; switch-endstop homing on a phase-stepped axis is not simulator-covered. |
| EtherCAT servo | Solid on test bench | Requires its dedicated real-time endpoint and bench setup; not a generic plug-in replacement for a stepper MCU. |
| `stepping_mode: stepcompress` | Configuration-supported compatibility mode | It is not compatible with phase-stepped motors or EtherCAT endpoints. Read the current configuration reference and validate the exact MCU/drive combination. |

The motion sample rate and trajectory storage options are target build options under the extra low-level `menuconfig` options. They are derived for supported targets; changing them without measurement changes the real-time contract.

## Connections and transports

| Connection | Status | Boundary |
| --- | --- | --- |
| USB / serial | Solid | Daily path. |
| Classic CAN | Bench verified | No real print validation; perform wiring and first-print bring-up deliberately. |
| CAN-FD | Bench verified | No real print validation; the USB-to-CAN bridge is classic-only. |
| EtherCAT | Solid on test bench | Separate endpoint; requires the documented IgH/PREEMPT_RT-oriented deployment. |

For CAN, work powered off and verify a correctly terminated bus before attaching electronics: a two-terminator network should measure approximately **60 Ω** between CAN-H and CAN-L. Follow [CAN bus](CANBUS.md) and [CAN troubleshooting](CANBUS_Troubleshooting.md), but prefer this page and [Feature status](Feature_Status.md) when inherited text claims broader board compatibility.

## What to report when support is uncertain

Collect the exact Git commit, host architecture, MCU/board and `menuconfig` selection, drive type, transport, configuration sections, and logs. A simulator pass is useful evidence but does not validate physical wiring, power, thermal behavior, latency, or mechanics. Do not label a combination supported solely because it compiled.
