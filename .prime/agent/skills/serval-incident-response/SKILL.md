---
name: serval-incident-response
description: Investigate a Serval motion, homing, transport, MCU, or EtherCAT fault using structured logs and safe recovery. Use after timing faults, crashes, endpoint loss, unexpected endstops, or position disagreement.
---
# Serval incident response

Read `docs/Diagnostics_and_Observability.md`, `docs/Homing_and_Endstops.md`, and the repository `.agents/skills/mcu-diagnostics/SKILL.md` when available.

1. Make the machine safe. Do not resume on assumed coordinates.
2. Record revision, board, transport, drive mode, and configuration.
3. Request `DIAG_DUMP` while connected; preserve `printer_data/logs/events/*.jsonl` and host logs.
4. Treat insufficient lead, start-time-in-past, clock, protocol, and endpoint faults as fail-stop. Fix the root cause, verify position, and home before controlled retest.
5. For simulator incidents, reproduce with `tools/sim/run.sh test --keep-logs`.

Never weaken watchdog, lead, or fault behavior to make an error disappear. A missing/stale live encoder position is an investigation and recovery condition, not an offset to apply blindly.
