# Diagnostics and observability

Serval records structured host and MCU events under `~/printer_data/logs/events/` (normally `printer_data/logs/events/`). These JSONL files are the primary forensic record; `klippy.log` alone is not sufficient for motion timing or MCU runtime faults.

## Immediate incident procedure

1. Stop and make the machine safe. Do not resume based on assumed coordinates.
2. Record Git revision, board/MCU names, transport, drive mode, and relevant configuration.
3. Run `DIAG_DUMP` while the MCU is connected. It asks every capable MCU for live runtime diagnostics and reports the event-file location.
4. Preserve the relevant JSONL files and host log. For simulator failures use `tools/sim/run.sh test --keep-logs`.
5. Correct the root cause, re-establish position, home, and only then test a controlled move.

Timing errors, missing lead, clock failure, and endpoint failures are safety signals—not transient warnings to suppress. `DIAG_DUMP` may report that no MCU exposes the command; capture the host-side error and connection history in that case.

## Local and aggregated logs

Host Python, host Rust, and each MCU write separate structured streams. A deployment may ship them through Vector to VictoriaLogs; this is optional aggregation, not a replacement for the local durable files. The repository’s `.agents/skills/query-logs` and `mcu-diagnostics` guides describe the field/query and runtime-event workflows for developers. Do not expose logs publicly without reviewing printer names, paths, network data, and any G-code content.

## Simulator boundary

The simulator uses real Klippy, real MACH_LINUX firmware processes, PTYs, virtual MCU time, and emulated GPIO/SPI/PWM/ADC devices. It catches protocol and state-machine faults, but it deliberately emulates hardware and relaxes some physical real-time checks. A simulator pass is evidence, not proof of wiring, drive, thermal, power, or mechanical safety on a printer.
