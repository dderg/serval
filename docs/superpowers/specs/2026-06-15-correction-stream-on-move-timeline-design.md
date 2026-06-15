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

`submit_correction_sequence` gains an explicit `start_host_secs` argument.
`Motion.manual_move` (the per-motor correction caller in `motion.py`) computes
it from the live toolhead timeline:

```
now  = reactor.monotonic()
glmt = toolhead.get_last_move_time()          # print_time at end of queued work
est  = mcu.estimated_print_time(now)
start_host_secs = now + (glmt - est)          # same real instant as the enable
```

The bridge converts `start_host_secs` through the existing
`host_time_to_mcu_clock` — the identical conversion regular motion uses — so the
buzz lands at the same real instant a move scheduled at `glmt` would. The
private `CORRECTION_LEAD_SECS` constant is removed; the lead is inherited from
`get_last_move_time()`, which already carries the shared 0.25.

### 2. Advance the toolhead's pending-end past the buzz

After streaming, `submit_correction_sequence` returns the buzz duration (it
already does). `manual_move` advances the toolhead bookkeeping so a subsequent
`get_last_move_time()` — read by the *disable* and any following move — falls
after the buzz:

```
toolhead._advance_pending_end_to(start_host_secs_as_print_time + duration)
```

Concretely: ensure `_mcu_pending_end_time >= glmt + duration`, so
`enable@glmt → dwell → buzz@glmt → dwell → disable@(glmt+duration+…)` orders
strictly, by the same mechanism that already makes `enable → move → disable`
order strictly. The correction path does **not** advance the planner's
`last_move_time` — corrections ride a separate ring, and conflating them with the
planner's anchor/starvation logic would be wrong. Only the toolhead bookkeeping
that enable/disable depend on is advanced.

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
