# Handoff: MCU clock crawls behind clocksync under the sim's virtual clock

**Date:** 2026-07-06 · **Branch:** sim-trip-time-resolution · **Status:** diagnosed to mechanism, not fixed. Split out of `sim-trip-time-resolution-handoff.md` (that bug is resolved; this is the one remaining item).

## Symptom

`tools/sim/run.sh test -k multi_point --runxfail` — flaky (~1 in 5 runs on an
idle machine, worse under load):

```
SCREWS_TILT_CALCULATE: Z endstop did not trigger within 13.0mm of travel
```

The probe descent never physically happens (shim step counter for Z stays
flat) and klippy's real-time homing deadline (`max_travel/speed +
TRIP_DEADLINE_MARGIN`) expires. Marked `xfail(strict=False)` on
`test_probe_multi_point_tools` in `tools/sim/tests/test_probe.py`.

## Diagnostic signature

In `events/mcu.jsonl` right before the failure:

```
motion.axis_stalled        arg0=131073 (axis 2, occupancy 1) stalled_ms=2000+
motion.axis_stalled_head   start-now=-2119ms  end-now=+1197ms
```

i.e. the MCU's armed Z piece is a ~3.3s merged HOLD (the Z lane padding from
the 150mm XY travel to the probe point, coalesced by
`append_pieces_merging_holds`) whose window straddles "now" by seconds. Also
`anchor_underrun` / `history_drop_on_reanchor` storms on the host side
(re-anchors every ~1s during the probe sequence — see the 18:07 timeline in
the events of any failing run).

## Mechanism (as understood)

- The MCU tick thread is the vtime pacer and runs at `nice 19`
  (`src/linux/runtime_tick_host.c`, `host_tick_main`). Under load, virtual
  time advances slower than real time.
- klippy runs in real time and extrapolates print time from a clocksync
  estimate (~50MHz measured when the sync samples were taken).
- During a long XY travel, vtime falls behind → klippy's estimated print
  time races ahead of the MCU's actual progress → `toolhead.wait_moves()`
  returns while the MCU still owes 1–2s of queued motion (the long Z hold
  ending seconds in the future).
- The probe drip is then dispatched against a playhead the MCU hasn't
  reached; the descent pieces queue behind the still-running hold; the
  anchor sees underruns and stutter-reanchors; klippy's homing deadline is
  wall-clock and expires first.

This is the same issue the vtime pacer commit (81ed5fa21) noted as "drip
refill margin hovers at the engine's adoption tolerance" — never solved. The
original trip-time-resolution xfail text blamed this mechanism too; that
turned out to be a different bug (history eviction, now fixed), but the
mechanism itself is real and this is where it actually bites.

## Repro

```bash
tools/sim/run.sh test -k multi_point --runxfail    # ~1/5 failure rate
```

or loop the deterministic-ish sequence (fails more often with the full
preamble because PROBE_ACCURACY heats up the anchor/drip state):

```python
# in the sim container, see scratchpad repro_screws.py from the resolved
# session: boot probe_config("points"), G28, PROBE, PROBE_ACCURACY SAMPLES=3,
# SCREWS_TILT_CALCULATE
```

Concurrent load (another sim suite running on the same machine) raises the
failure rate a lot — useful for reproduction, but it fails on an idle
machine too.

## Attack ideas (untested)

1. Make klippy's clock sync see the crawl: the sim could either run
   clocksync re-sampling faster, or the vtime wall-driver could guarantee
   rate-1 (it claims "strict linear function of real time" — verify what
   VTIME_SPEED and the pacer actually negotiate when the tick thread is
   starved; the pacer may be *capping* vtime below real rate, which is the
   crawl).
2. Don't deprioritize the tick thread (`setpriority(19)`) — it was demoted so
   the main thread's ppoll advances virtual time, but that tradeoff predates
   the wall-driver. Measure whether nice 0 removes the crawl without
   deadlocking vtime.
3. Homing deadline in klippy (`TRIP_DEADLINE_MARGIN`) is wall-clock; under
   mcu-sim it could be derived from MCU-clock progress instead — masks
   rather than fixes, only do this if 1/2 are dead ends.

## Related loose ends (same doc family)

- `set_position` rebases history endpoints with a `host_now` from a
  different epoch than the piece keys (observed host=1433.8 vs keys ~1s).
  Harmless today; unify when touching this area.
- `src/linux/timer.c` rebase-across-wrap seam still unaudited (old TODO).
- Event-log pipeline drops lines under burst (failing sessions' jsonl files
  were missing whole event classes) — made this investigation much harder;
  worth a look while in here.
