# Homing, endstops, and probes

Homing is safety-sensitive streamed motion. Use the rail fields in [Motion configuration reference](Config_Reference_Motion.md) (`endstop_pin`, travel/endstop positions, speeds, retract, sensorless options) and keep the machine attended after any topology, firmware, or endstop change.

## Standard and sensorless homing

`G28` homes configured axes using their rail endstops; `homing_positive_dir` is inferred when unambiguous or can be declared. Virtual endstops select sensorless defaults where supported. A homing move is a guarded “drip” run with approximately 100 ms of lead, unlike normal buffered printing. Host stalls, camera load, USB/CAN stalls, or real-time endpoint failure can therefore abort it. This is intentional: a timing fault must not silently execute stale motion.

If a switch is already provably triggered as a run is armed, Serval clamps the reported position to the run start rather than accepting an impossible in-motion trip. This protects position accounting; it is not permission to ignore a stuck switch. Diagnose wiring, polarity, and mechanical preload, then home again.

## Multi-motor rails and gantry squaring

A Cartesian 1:1 rail with one switch per motor uses a keyed endstop block:

```ini
[axis x]
endstop_pin:
  x_left: ^PA1
  x_right: ^PB2
position_min: 0
position_max: 300
position_endstop: 0
```

Every motor in the lane appears exactly once; every switch belongs to the MCU driving that motor. On the first switch trip, its associated motor freezes while the remaining motor(s) continue. The final trip stops the axis, squaring the gantry. This supports cross-MCU coordination through host-mediated suppression, but the timing boundary makes correct wiring and testing especially important.

Keyed blocks are invalid for CoreXY shared lanes, virtual/sensorless switches, and non-1:1 axis-to-motor mappings. Those configurations use a single endstop strategy. `QUERY_ENDSTOPS` reports keyed switches as `axis:motor`.

## Recovery rules

After an abort, unexpected trip, MCU reconnect, or motor-disable event, do not assume the prior coordinate remains true. Remove the cause, inspect [Diagnostics and observability](Diagnostics_and_Observability.md), manually establish a safe clearance if needed, and home before printing. Probe workflows and bed transforms remain inherited subsystems; use their dedicated reference only after the Serval motion topology is valid.
