# Handoff: beacon emulator's contact model doesn't track Z

**Date:** 2026-07-06 · **Branch:** sim-improvements · **Status:** failures characterized per-scenario, fix not started

## TL;DR

The Beacon MCU emulator (`tools/sim/emulators/beacon_mcu.py`) fires its
contact trigger on a **fixed delay timer** and anchors its Z model once at
boot, instead of keying off the actual step-tracked toolhead Z. Now that the
sim firmware really emits steps (CONFIG_MOTION_MODULE_STEPPER=y), the fork
(`dderg/beacon_klipper`, branch `motion-stack-rename`) notices the
inconsistencies. Four scenarios in `tools/sim/tests/test_beacon.py` are
xfail/skip-marked with the observed failure; proximity homing, connect, and
accelerometer streaming are green.

## Per-scenario symptoms (from the marks in test_beacon.py)

| Test | Symptom |
|---|---|
| `test_proximity_probing` (xfail) | After G28 Z + `G1 Z3`, `PROBE PROBE_METHOD=proximity` → "Attempted to probe with Beacon below calibrated model range". The emulator's step-tracked Z drifted through the homing descent, so its reported frequency maps below model_range min (0.2mm). |
| `test_contact_probing` (xfail) | `PROBE PROBE_METHOD=contact` → "query host time precedes retained motion history" — the fixed-delay trigger's detect time lands outside the move's history window (overlaps with the trip-resolution handoff, but the emulator's made-up detect time is the aggravator here). |
| `test_contact_auto_calibrate` (xfail) | `BEACON_AUTO_CALIBRATE` → "beacon: descend completed with trsync reason 2, not endstop-hit" — the emulator's contact trsync completes with reason 2 (comms-timeout class) instead of the endstop-hit reason because the trigger timer didn't fire within the descend. |
| `test_poke` (skip) | `BEACON_POKE` never returns (120s+) — hang, same trigger-model gap. |
| `test_bed_mesh` (skip) | `BED_MESH_CALIBRATE` dies with an unhandled reactor exception and never responds. Reproduce first — may partly be a fork bug rather than emulator. |

## Repro

```bash
tools/sim/run.sh test -k beacon --runxfail   # skips still skip; drop the marks to run poke/mesh
```

The beacon fixture needs the fork checked out in the image
(`tools/sim/fetch_plugins.sh`, pinned to dderg/beacon_klipper
`motion-stack-rename` @ d08ea46) — the Docker build does this automatically.

## Where the emulator stands

`BeaconMcuStub` already:
- tracks toolhead Z by polling the shim's step counter over the
  `sim_control` socket (`step_sock_path`, `_step_poll_loop`, Z step-queue
  line 15, 800 steps/mm per the CoreXY config in
  `tools/sim/configs.py::beacon_homing_config`)
- models frequency as `freq = base + coeff / (z + offset)` matching the
  SAVE_CONFIG model block in the generated config
- implements the trsync + contact command surface
  (`_handle_beacon_contact_home`, `_fire_contact_trigger` — the fixed
  `_homing_trigger_delay = 0.5s` timer is the shortcut to replace)

## Suggested fix shape

1. Re-anchor `_z_anchor_mm`/`_z_anchor_steps` when a homing trigger fires
   (proximity: at threshold crossing; contact: at "bed contact"), so the
   post-home Z frame matches klippy's `position_endstop`-relative frame.
2. Contact trigger: fire when the *step-tracked* Z crosses a configurable
   bed height (e.g. 0), not after a delay; report the trigger clock at that
   moment and complete the trsync with the endstop-hit reason the fork
   expects (it currently sees reason 2).
3. Then unmark the tests one at a time; `test_bed_mesh`'s reactor exception
   needs its own look once contact works.
