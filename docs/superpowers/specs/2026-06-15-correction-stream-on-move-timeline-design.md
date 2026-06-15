# Correction stream on the move timeline — design

**Goal:** Make the per-motor correction stream (motors_sync's "buzz") obey the
same scheduling timeline as regular motion, so digital events scheduled around
it — stepper enable/disable — order deterministically against it, the same way
they already do against ordinary moves.

## Problem

motors_sync energizes a stepper, buzzes it via a correction sequence, then
de-energizes — `enable → buzz → disable`, with `toolhead.dwell()` between each
step (`motors_sync.py:1235-1250`). On the bench the energize/de-energize
overlaps the buzz and clicks loudly: the enable lands *later* than the buzz,
and the skew grows through the sequence.

## Root cause

Enable and regular motion already share one timeline; the correction stream does
not.

- A regular move is anchored by the engine at `host_now + 0.25`
  (`anchor.rs:59`, `DEFAULT_LEAD_SECS = 0.25`), and `submit_move` advances the
  toolhead's `_mcu_pending_end_time` by the move's duration
  (`motion.py:339-341`).
- The enable fires at `get_last_move_time()` =
  `estimated_print_time(now) + BUFFER_TIME_START` (`motion.py:474`,
  `BUFFER_TIME_START = 0.250`) — the **same 0.25 lead**, on a timeline that
  **accumulates every move's duration**. This is why normal printing never
  shows the bug: enable and the first move land together.

The correction stream breaks that shared timeline two ways
(`bridge.rs:stream_correction_entries`):

1. It anchors at a private `host_now + CORRECTION_LEAD_SECS` (`0.15`), a
   different lead from motion's `0.25`.
2. It never advances `_mcu_pending_end_time`, so `get_last_move_time()` is blind
   to the buzz. The `dwell()`s between enable and buzz advance the enable's clock
   but not the buzz's, so the skew accumulates.

The two clock conversions are *not* the issue: the Python clocksync feeds the
Rust router (`mcu.py:1280-1295`), so `print_time_to_clock` (enable) and
`host_time_to_mcu_clock` (motion/buzz) resolve the same real instant to the same
MCU clock. Only the **target instant** differs.

## Design

Make the correction stream a first-class citizen of the move timeline. No
enable/disable changes — they are already correct relative to real moves; they
only looked broken because the buzz was off-timeline.

### 1. Anchor the buzz at the toolhead timeline, not a private lead

`submit_correction_sequence` gains an explicit `start_print_time` argument — the
toolhead's `get_last_move_time()` (a *print_time* at the end of queued work).
`Motion._stream_correction_on_timeline` passes it straight through; the bridge
converts it to a host-clock anchor **inside the router** via
`print_time_to_host_secs`, then projects with `host_time_to_mcu_clock`:

```
glmt = toolhead.get_last_move_time()          # print_time at end of queued work
# bridge, under the router lock:
start_host = router.print_time_to_host_secs(mcu, glmt)   # router Instant timebase
clock      = router.host_time_to_mcu_clock(mcu, start_host)
```

**Timebase is load-bearing.** The anchor must be a *print_time*, not a host
value built from `reactor.monotonic()`. `reactor.monotonic()` is
`CLOCK_MONOTONIC_RAW`, whereas the router's `host_time_to_mcu_clock` expects its
`Instant` timebase (`clock_offset` is rebased to `Instant` in
`set_clock_est_rebased`). Feeding a RAW-based host value would offset the buzz by
the boot-to-process epoch gap (system uptime at klippy start), scheduling it far
in the future. Routing the *print_time* through `print_time_to_host_secs`
collapses exactly — `host_time_to_mcu_clock(print_time_to_host_secs(pt)) ==
pt * freq` — so the buzz lands at precisely `print_time_to_clock(glmt)`, the same
MCU clock the enable targets, independent of the epoch. The private
`CORRECTION_LEAD_SECS` constant is removed; the lead is inherited from
`get_last_move_time()`, which already carries the shared 0.25.

`Motion._stream_correction_on_timeline` still reads `now`/`est` solely to return
the caller's wait-until-complete `(glmt - est) + duration`, not for the anchor.

**Ground before anchoring.** `_stream_correction_on_timeline` calls
`wait_moves()` first, so the queued enable + settle dwells actually *execute* (the
motor energizes in real time) and `get_last_move_time()` collapses back to
~`now + lead` before the buzz is anchored. Without this, motors_sync's
dwell-built enable→buzz gaps leave `glmt` reserved well ahead of real time; the
buzz pieces are then scheduled into the future and **never retire**, so the
correction ring fills and the MCU rejects the next push with `-309` (RING_FULL).
Grounding keeps the buzz near real-now so its pieces retire promptly and the ring
drains between buzzes. The enable still precedes the buzz (its dwell executed
during the drain), so the de-energize-mid-shake fix holds.

### 2. Reserve the buzz's time with a real dwell, not a phantom pending poke

After streaming, `submit_correction_sequence` returns the buzz duration (it
already does). `_stream_correction_on_timeline` then issues a real
`toolhead.dwell(duration)` so a subsequent `get_last_move_time()` — read by the
*disable* and any following move — falls after the buzz:

```
duration = submit_at(glmt)     # buzz anchored at glmt (print_time)
self.dwell(duration)           # proper engine dwell mirroring the buzz window
```

A real `dwell()` goes through `bridge.submit_dwell` (engine-backed, subject to
the planner's lookahead throttle) plus the toolhead bookkeeping. This is the
critical correctness point: an earlier version hand-poked
`_mcu_pending_end_time = glmt + duration` to fake the reservation. That host-only
write has **no engine backing and no backpressure**, so across motors_sync's many
enable/dwell/buzz/dwell/disable cycles it let the timeline march arbitrarily
ahead of real time — scheduling the buzz far in the future, where the
feedback-paced ring wait blocks the reactor long enough to starve MCU comms and
drop the USB transport. A real dwell is throttled and executes in real time, so
`get_last_move_time()` stays bounded (~one lead ahead) and the buzz anchors near
real-now while still ordering after the enable. The correction path does **not**
advance the planner's `last_move_time` directly — corrections ride a separate
ring; the main-ring dwell is what honestly accounts for the buzz's wall-time.

### 3. One lead constant, no "keep in sync" comment

Today the 0.25 lead exists in four places:
`anchor.rs DEFAULT_LEAD_SECS`, `planner.rs LEAD` (with a
`// Must equal anchor::DEFAULT_LEAD_SECS. Keep in sync` comment),
`motion.py BUFFER_TIME_START = 0.250`, and `bridge.rs CORRECTION_LEAD_SECS`
(a divergent `0.15`).

- Rust single source: make `anchor::DEFAULT_LEAD_SECS` `pub`; `planner.rs`
  references it and deletes both its `LEAD` copy and the keep-in-sync comment.
- Cross-language single source: expose it from the bridge
  (`PyMotionBridge.motion_lead_secs() -> f64` returning
  `anchor::DEFAULT_LEAD_SECS`). `Motion` fetches it at connect and uses it where
  `BUFFER_TIME_START` was the module constant. The literal `0.250` in
  `motion.py` is deleted.
- `CORRECTION_LEAD_SECS` is removed entirely (subsumed by the timeline anchor).

After this, there is exactly one definition of the motion lead, owned by Rust,
consumed everywhere.

## Components touched

- `rust/motion-bridge/src/anchor.rs` — `DEFAULT_LEAD_SECS` becomes `pub`.
- `rust/motion-bridge/src/planner.rs` — `LEAD` → `anchor::DEFAULT_LEAD_SECS`;
  delete the keep-in-sync comment.
- `rust/motion-bridge/src/bridge.rs` — `submit_correction_sequence` /
  `stream_correction_entries` take `start_host_secs`; remove
  `CORRECTION_LEAD_SECS`; add `motion_lead_secs()` getter.
- `klippy/motion.py` — fetch `motion_lead_secs()` at connect; replace
  `BUFFER_TIME_START` literal; `manual_move` computes `start_host_secs`, advances
  `_mcu_pending_end_time` by the buzz duration.
- `klippy/extras/motors_sync.py` — `StepperManualMove.manual_move` passes the
  computed anchor through (or calls the updated `Motion.manual_move`); no change
  to the enable/disable bracketing logic.

## Testing

- Rust unit: `host_time_to_mcu_clock` anchor for a correction matches a regular
  move scheduled at the same host instant (same MCU clock, within rounding).
- Rust unit: one lead constant — assert `planner` and `anchor` resolve to the
  same value (compile-time identity once deduped); assert
  `motion_lead_secs() == DEFAULT_LEAD_SECS`.
- Python unit: after `manual_move`, `get_last_move_time()` advanced by ≥ the
  buzz duration; a disable scheduled afterward resolves to a clock strictly
  greater than the buzz's last piece.
- Bench: `enable → buzz → disable` no longer overlaps audibly; energize precedes
  the buzz, de-energize follows it.

## Non-goals

- Routing enable/disable through a second scheduling authority (the heavier
  "Design 2"). The router is already slaved to the Python clocksync, so there is
  effectively one authority today; Design 2 pays a C/Rust boundary cost for no
  ordering benefit.
- Changing the correction ring depth, feedback pacing, or the MCU evaluator.
