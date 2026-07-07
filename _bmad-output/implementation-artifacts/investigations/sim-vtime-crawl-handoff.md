# Handoff: probe-drip flake — vtime crawl REFUTED, real cause is shaper micro-piece flood starving the pump

**Date:** 2026-07-07 (supersedes 2026-07-06 version) · **Branch:** sim-handoff-3 (sota-motion merged through PR #181) · **Status:** root-caused to the shaper's ladder fit; fix not yet written.

## TL;DR

The old "MCU clock crawls behind clocksync" mechanism is **refuted by direct
measurement**. The virtual clock tracks real time exactly and the tick thread
holds a clean 10 kHz cadence (worst gap 1.6 ms) even during failing runs.

The actual failure chain, each link verified in a failing run:

1. During homing/probe drips, the shaper's adaptive span fitter
   (`refine_shaped_span`, `rust/motion-pipeline/src/shaper.rs`) fails to
   converge on some lanes and bisects to its 50 µs floor
   (`SHAPED_FIT_MIN_SPAN_S`), emitting **tens of thousands of 69–98 µs
   micro-pieces** where ~100 × 25 ms pieces are expected. Measured via a
   temporary `drip_enqueue_lead` event: G28's moving lane = 41 942 pieces
   (min 69 µs); the probe descent after the 150 mm X travel delivers the
   *held* X lane as 16 847 micro-pieces (min 76 µs) for a 2.7 s window,
   while the moving Z lane is a clean 112 × 25 ms.
2. 16 k pieces at ~27 pieces per 1 KiB PushPieces frame ≈ 600+ synchronous
   ~2.5 ms round-trips ≈ 1.5 s of wire time. The pump saturates and stalls
   120–200 ms at a time (measured via a temporary `pump_send_projection`
   event) and releases Z descent pieces **~60–135 ms after their start
   times** — while its MCU-clock projection is accurate to ~2 ms
   (projection/clocksync exonerated; producer also exonerated — the whole
   drip reaches the pump in one enqueue ~250 ms ahead).
3. mcu-sim widens `MAX_START_IN_PAST_SECS` to 10 s, so the MCU adopts the
   late pieces and executes them as compressed catch-up bursts
   (`dispatch_pulse` carry).
4. A burst crosses the sim endstop wall's 200-step (0.25 mm) trigger window
   faster than the MCU's 1 ms endstop poll (`endstop.arm` rest_ticks) can
   sample it. Shim trace (temporary `[auto-endstop]` logging) shows the
   descent executing, gpio202/203 asserting at pos −50, the overrun slop
   releasing the latch at −250 ("MCU failed to stop"), and Z sailing 6 mm
   past the wall. Klippy sees no trip → "Z endstop did not trigger within
   13.0mm of travel" after the 7.6 s wall-clock deadline.

This is the concrete form of the old note "drip refill margin hovers at the
engine's adoption tolerance" — but the margin is eaten by wire saturation
from micro-pieces, not by clock skew.

## Evidence base

All from `tools/sim/run.sh test -k multi_point --runxfail` runs on 2026-07-06/07
with the container's TMPDIR bind-mounted so full `events/*.jsonl` survive:

```
docker run --rm -e TMPDIR=/out -v <hostdir>:/out --entrypoint python3 \
  kalico-sim-<branch> -m pytest tools/sim/tests -m needs_elf -k multi_point --runxfail -v
```

- Tick thread health: `[tick-rate] ticks=10000 vt_ms=1000 real_ms=1000
  max_gap_real_us=1600` every vtime-second, including across the failure.
  (The wall driver `vtime_driver_main` in libvtime.c raises vtime to the
  speed cap on a 1 ms cadence, deliberately ignoring pacer floors — skipped
  ticks are absorbed by dispatch_pulse carry, per commit c0bf14c55.)
- `transit_diag_alert` storm: descent pieces arriving `arrival_lead_us`
  −80 000…−112 000 in ~400 ms clumps; `axis_stalled_head` showing the
  merged 3.3 s Z hold straddling now during the 150 mm travel (benign dwell).
- `drip_enqueue_lead` (temp): piece counts/durations above; produce lead
  +232…250 ms always.
- `pump_send_projection` (temp): `release_lead_ms` swinging +89 → −135 with
  120–200 ms send gaps during descents; paired with the PushPieces response
  clock, projection error ≈ 1.7–2.8 ms total including transit.
- `-142` (`RUNTIME_ERR_STREAM_HALTED`) send transients cluster around each
  trip/rehome — pieces pushed while the stream is gated; retried, benign.

## Failure rate

~100% for `test_probe_multi_point_tools` on this machine (M-series, Docker)
on both the pre- and post-#181 trees — the planner speedup did not help
(the flood is shaper fragmentation, not planner cost). One pass was observed
in ~10 runs, so it is still nominally flaky. The safe-z variant shares the
chain (its "retract stalls, QUERY_PROBE reads TRIGGERED" shape is the same
missed/late-trip pathology).

## Next steps (in order)

1. **Root-cause the ladder-fit divergence.** Why does a *held* lane (X
   constant during a Z probe drip, right after a 150 mm X travel) fail
   `shaped_ladder` all the way to the 50 µs floor for the whole 2.7 s
   window? Suspects: shaper convolution tail of the prior travel leaking
   into the fresh drip stream; a discontinuity at the drip's `reset_to`
   seam that the quintic can never fit; NaN/derivative pathology in
   `finite_derivative` at the domain edge. Reproduce offline via
   pipeline-snapshot with a travel→drip sequence and inspect the emitted
   Bezier pieces. Note G28's own moving lane also fragments (41 942 pieces)
   — same fitter, so one fix likely covers both.
2. **Consider a fail-loud guard**: an assert/error event when a single
   segment flattens to pathologically many pieces (e.g. > 4× the
   duration/max_piece_secs bound) so this class can't regress silently.
3. After the fix: un-xfail `test_probe_multi_point_tools` and the safe-z
   variant of `test_probe_homing_and_probing`; confirm `axis_stalled`
   storms and `transit_diag_alert` negative leads are gone.
4. **Separate, newly exposed:** `test_probe_multi_point_tools`'s final
   assertion expects `Z_TILT_ADJUST` to fail with "per-motor Z adjustment
   is not yet implemented", but on the merged tree it now succeeds
   (`{'result': {}}`) — the test expectation is stale and needs updating
   once the probe flake is fixed (it's currently masked by the earlier
   SCREWS_TILT failure).

## Uncommitted instrumentation in this worktree (keep or strip when fixing)

- `src/linux/runtime_tick_host.c` — `[tick-rate]` per-vtime-second cadence
  report (CONFIG_MCU_SIM only).
- `tools/sim/preload/libsim_intercept.c` — `[auto-endstop]` trig-flip and
  800-step movement logging.
- `rust/motion-engine/src/pump.rs` — `drip_enqueue_lead` (piece count /
  duration stats / produce lead at enqueue, cohort only) and
  `pump_send_projection` (release lead + projection at send, cohort only).

The first two are cheap and arguably worth keeping permanently; the pump
events fire per PushPieces during drips (~1 kHz worst case) and should be
sampled or dropped before merging.

## Old attack ideas — status

1. "Make clocksync see the crawl / verify the pacer caps vtime" — moot;
   there is no crawl. The wall driver guarantees rate-1 vtime and the
   projection is ~2 ms accurate.
2. "Un-nice the tick thread" — unnecessary; nice 19 still holds 10 kHz here.
3. "MCU-clock-based homing deadline" — would only mask the late execution;
   the trip itself is missed at the MCU, so this fixes nothing.

## Related loose ends (unchanged from previous version)

- `set_position` rebases history endpoints with a `host_now` from a
  different epoch than the piece keys. Harmless today; unify when touching.
- `src/linux/timer.c` rebase-across-wrap seam still unaudited (old TODO).
- Event-log pipeline drops lines under burst (whole event classes missing
  from failing sessions' jsonl) — e.g. `set_clock_est` events never appear
  in `events/host-rust.jsonl` even at info level; made this investigation
  harder twice now.
