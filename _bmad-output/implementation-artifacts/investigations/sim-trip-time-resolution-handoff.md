# RESOLVED: G28 trip-time resolution fails under the simulator's virtual clock

**Date:** 2026-07-06 · **Branch:** sim-trip-time-resolution · **Status:** RESOLVED — root cause was history-ring eviction, not a clock-domain mismatch

## What it actually was

The trigger clock and the router's `clock_to_host` mapping were **correct all
along**. The failing query (`query host time 0.69s precedes retained motion
history … window 23.99..25.70s`) happened because `HistoryStore` recorded the
**entire homing move at dispatch time** — a 37.6 s homing segment flattens
into ~42 k pieces per axis, and `HISTORY_CAPACITY = 4096` popped the front of
the ring, retaining only the last ~1.8 s of the *planned future*. The endstop
trips near the *start* of the move, whose pieces had already been evicted.
The "23s earlier" in the original error was the distance between the trip
(early in the move) and the retained tail (end of the move) in stream time,
not a clock-domain offset.

Fix: motion history is now recorded in the pump at the moment each piece is
**sent to the MCU** (`HistoryRecorder` in `rust/motion-engine/src/pump.rs`),
so the store mirrors what the MCU can actually execute. An endstop can only
trip on pieces the MCU has received, so the trip query always lands inside
the retained window. This also stops history from being polluted by planned
pieces that a trip-flush later discards.

## Other bugs fixed on the way (each was masking the next)

1. **Sim step-direction sign** (`src/linux/runtime_tick_host.c`): the drain
   passed `dir ? -1 : 1` to `sim_intercept_notify_step`, but `dispatch_pulse`
   emits `dir` as **+1/−1** — every step counted as −1. Homing approaches
   looked right by coincidence (they are negative); retracts counted the
   wrong way, so the auto-endstop wall never released and re-approaches
   tripped at 0.00 mm. Now passes the signed value.
2. **Auto-endstop wall latch** (`tools/sim/preload/libsim_intercept.c`): the
   approach direction latched at the *first* wall crossing, so safe-z's
   pre-homing z-hop latched the wall upward and the real descent started
   "already tripped". The wall now unlatches when motion travels far past a
   latched wall (a real endstop can't be traveled through) and re-arms on the
   next crossing from inside.
3. **`run_probe` left the nozzle in contact** (`klippy/extras/probe.py`):
   mainline restores the toolhead after a probe session
   (`always_restore_toolhead`); the rewrite didn't, so `PROBE` →
   `PROBE_ACCURACY` hit "Probe triggered prior to movement". `run_probe` now
   retracts after the final sample.
4. **Test/config fixes**: `SCREWS_TILT_CALCULATE` (not `_ADJUST`); bed-mesh
   activation is expected to be rejected by the planner (not yet ported);
   `min_home_dist: 0` for the remote (timer-based) endstop variant; ±10 µm
   tolerance on cross-MCU trip-vs-stop reconstruction float noise.

All five probe xfails are removed; `test_contact_probing` and
`test_proximity_probing` (beacon) xfails removed too — both were downstream
of the same eviction/step-sign bugs.

## Still open

- **vtime crawl under host load** (`test_probe_multi_point_tools`, xfail
  strict=False): split into its own handoff —
  [`sim-vtime-crawl-handoff.md`](sim-vtime-crawl-handoff.md) — with the
  diagnostic signature, mechanism, repro, and attack ideas.
- `set_position` rebases history endpoints with `host_now` taken from a
  different epoch than the piece keys (observed host=1433.8 vs keys ~1s).
  Harmless today (endpoints are only consulted when rings are empty and the
  host key is not compared), but worth unifying.
- The timer.c rebase-across-wrap seam is still unaudited (pre-existing TODO).

## Tooling traps hit during this work (read before trusting sim results)

- **Shared image tag**: `run.sh` used one `kalico-sim` tag for every worktree;
  a concurrent session rebuilding it silently swaps the image under you
  mid-investigation. Fixed: the tag is now branch-partitioned
  (`kalico-sim-<branch>`), like the compile caches.
- **BuildKit + cargo staleness**: the cache-mounted cargo target dir can go
  stale (context mtime scanning on macOS misses edits; poisoned incremental
  artifacts even produced a binary with *mixed old/new code*). When sim
  behavior contradicts the source, byte-grep the built `.so` for a
  known-new string, and `docker builder prune -af` + rebuild if it's wrong.
