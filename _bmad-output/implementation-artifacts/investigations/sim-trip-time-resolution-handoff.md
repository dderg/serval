# Handoff: G28 trip-time resolution fails under the simulator's virtual clock

**Date:** 2026-07-06 · **Branch:** sim-improvements · **Status:** deterministic repro, root cause NARROWED (clock-domain mismatch), fix not started

## TL;DR

In the unified simulator (`tools/sim/`), any homing move that resolves its final
position through the motion-history/trip pipeline fails with:

```
X trip move failed: query host time 0.692161s precedes retained motion history
for axis AxisKey { mcu_id: 0, axis: 0 } (window 23.991060..25.695626s)
```

The trigger clock maps ~23s earlier than the motion-history window recorded for
the very move that tripped. Five probe-suite tests are `xfail`-marked with this
diagnosis (`tools/sim/tests/test_probe.py`, `_TRIP_RESOLUTION_XFAIL`); remove
the mark or run pytest with `--runxfail` to work on it.

## Repro

```bash
tools/sim/run.sh test -k "probe_homing" --runxfail       # 3 variants
# or interactively:
tools/sim/run.sh shell
python3 - <<EOF
import pathlib, sys; sys.path.insert(0, "/kalico")
from tools.sim.world import SimWorld
from tools.sim import configs
w = SimWorld(pathlib.Path("/tmp/dbg")); w.dual_mcu = False
w.boot(configs.probe_config(w.h7_pty, str(w.gcode_dir), "virtual"))
print(w.gcode("G28 X", timeout=60))
EOF
```

Fails every run. Physical stepping works (the shim's auto-endstop wall on
gpio200 does trigger — steps flow, `get_steps` counts move); only the
*time-domain resolution* of the trip is wrong.

## What is known

- The failing conversion is `clock_to_host(endstop_mcu, trip_clock)` in
  `rust/motion-engine/src/homing.rs::final_cartesian_position` →
  `motion_history.rs::clock_to_host` → `PassthroughRouter::clock_to_host_secs`.
- Numbers from one instrumented run (single MCU, `probe_config("virtual")`):
  - history window: engine-relative host seconds ≈ 24.0..25.7 (consistent with
    when the homing move actually ran, ~24s after the pytest session started)
  - `clock_to_host(trip_clock)` ≈ 0.69s — i.e. `trip_clock / freq +
    clock_offset` with `clock_offset ≈ -0.36`, `freq ≈ 50e6` → the reported
    trip clock corresponds to ~1.05s of MCU uptime, though homing ran seconds
    later. Either the trsync-reported trigger clock is stale/wrong-domain, or
    the router's clocksync estimate is stale relative to the history keying.
  - `[clock-seed] set_clock_est_rebased` events show `offset_raw` in the VM
    monotonic domain and `clock_offset` rebased against `bridge_now` at rebase
    time; `last_clock` near boot. The remote-endstop variant logs the trigger
    with `clock32=0` and a synthesized `clock64` — worth auditing that
    widening path first.
- History-shadow divergence warnings (`history_shadow_divergence`, ~0.2mm)
  fire during beacon homing — same family of host-keyed vs stepper-clock-keyed
  disagreement.
- Pre-existing context: the vtime pacer commit (81ed5fa21) already noted
  "drip refill margin hovers at the engine's adoption tolerance" as a known
  issue; this was never solved, only masked because the old sim firmware had
  no stepper module compiled in at all.

## What was already fixed on this branch (don't re-diagnose)

- vtime is now a strict linear function of real time (wall-driver thread in
  `tools/sim/preload/libvtime.c`) — it used to be demand-driven and leapt,
  which broke clocksync far worse (PieceStartInPast storms). The trip
  failure survives the driver, so nonuniform vtime is NOT the whole story.
- The Linux MCU timer starts at 0 under `CONFIG_MCU_SIM` (upstream starts at
  -1s → boot-adjacent 32-bit wrap collided with the early clocksync rebase).
  The rebase-across-wrap seam itself is still unaudited (TODO in
  `src/linux/timer.c`).
- `dispatch_stepper` carries steps beyond `MAX_STEPS_PER_SAMPLE` forward under
  `mcu-sim` instead of faulting; the sensorless-homing test (pulsed TMC DIAG
  endstop, `test_phase_stepping.py`) passes — so trip resolution *can* work;
  the wall-triggered gpio-endstop path is what fails.

## Suggested attack

1. Instrument the trsync/endstop trigger clock at the source (MCU endstop.c
   report + host-rust ingestion) and log all three domains side by side:
   raw trigger clock32, widened clock64, router `clock_to_host_secs` output,
   and the history window keys for the same move.
2. Audit `clock32=0` trigger widening in the remote/trip-relay path.
3. Check whether the router's clocksync estimate and the motion-history
   recorder use the same host epoch (rebase events vs `bridge_now_instant`).

## Danger

Do NOT bump `STEP_QUEUE_DEPTH` under mcu-sim as a shortcut for anything here:
that was tried (256) and smashed the installed queue-pointer table
(`rt_storage` sizing) — axis queue_ptr came back garbage and the firmware
segfaulted in `dispatch_pulse`. Reverted; see branch history.
