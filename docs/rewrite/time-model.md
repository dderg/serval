# The time model — one authority, absolute answers

Read this before writing any code that schedules an MCU command relative to
motion, timestamps a sensor sample, or reports "when does motion end". The
rules here exist because we shipped a season of timing bugs by breaking them.

## The domains

| Domain | Unit | Defined by | Where it lives |
|---|---|---|---|
| host time | seconds, CLOCK_MONOTONIC_RAW | the host kernel | Python `reactor.monotonic()`; Rust `Instant` / `HostSecs` (anchor frame of `instant_to_f64`) |
| MCU clock | ticks, per MCU | each MCU's crystal | `u64` clocks on the wire |
| print time | seconds | primary MCU's `clock / nominal CLOCK_FREQ` | every scheduling API klippy exposes; Rust `PrintTime` |
| stream time | seconds since last stream open/reset | the motion pipeline | Rust only (`seg.t_start/t_end`, anchor `t0`). **Never crosses into Python.** |

Two frequencies exist per MCU and they are not interchangeable:

- **nominal** `CLOCK_FREQ` — a constant. `print_time ≡ clock / nominal`.
  (`clocksync.py print_time_to_clock`, router `print_time_at_host`.)
- **regression** frequency — the clocksync estimate of ticks per host second;
  drifts around nominal by ppm. It answers "what does the clock read at host
  instant t", never "what print_time is clock c". Converting print_time
  through the regression frequency accumulates ppm × uptime of error —
  seconds after hours of uptime.

## The one mapping per crossing

- host ↔ MCU clock: the clocksync regression. Python owns the estimator
  (`clocksync.py`); every update is mirrored into the Rust
  `PassthroughRouter` record (`set_clock_est_rebased`), so both sides project
  with the same numbers. The router is the Rust-side authority; do not carry
  your own `(freq, offset, last_clock)` anywhere else.
  The record belongs to one MCU boot epoch: every `_mcu_identify` calls
  `invalidate_clock_est`, and until a *converged* estimate arrives every
  projection is a loud error (`RouterError::NoClockEstimate`) and the
  stepcompress anchor refuses to run (`DispatchError::ClockRecordUnusable`).
  A record surviving a reflash would project clocks ahead of the restarted
  MCU counter by the previous boot's uptime.
  Two different quantities describe that record, and confusing them costs
  hours: `clock_offset` is the regression's decay-weighted sample **centroid**,
  which on a perfectly live record trails now by up to `1/DECAY` get_clock
  periods (~30 s), so `host_now - clock_offset` (`centroid_lag_secs`) is the
  projection's lever arm, not staleness. The record's **age** is
  `host_now - updated_at`: how long since the router last accepted an estimate.
  Both are on every `reanchor_record` event. The anchor warns past
  `DEGRADED_CLOCK_RECORD_AGE_SECS` (3 periods; measured healthy worlds gap up
  to ~10 s, so this is not a stop) and refuses past
  `MAX_CLOCK_RECORD_AGE_SECS` (the full regression window) with
  `DispatchError::ClockRecordStale`.
- print_time ↔ MCU clock: multiply/divide by **nominal** `CLOCK_FREQ`
  (secondary MCUs additionally go through `SecondarySync.clock_adj`,
  Python-side).
- stream time → anything: only through the dispatch anchor's `t0`
  (host = `t0 + t`), and only inside the bridge. If you find yourself
  wanting a stream time in Python, you want one of the authority calls
  below instead.

## The authority calls (bridge methods)

- `engine.frontier_print_time(mcu_handle)` — absolute print_time at which
  **all committed motion ends** — segments, dwells, nudges. Cheap and
  non-draining. Sits in the past while the printer is idle, deliberately:
  idle detection measures `est_now − frontier`, so flooring it to now would
  read as "busy forever". This is what status, stats, idle detection, and
  "is the printer busy" read. There is no host-side shadow of this value;
  there used to be (`Motion._mcu_pending_end_time`) and it drifted.
- `engine.fence_print_time_poll(id, mcu_handle)` — absolute print_time at
  which everything submitted **before the fence** ends. `fence_start(force)`
  arms it; `force=True` drains the pipeline (brake to rest) exactly like
  mainline's lookahead flush. Also un-floored; scheduling safety comes from
  the floor below. `toolhead.get_last_move_time()` is this plus the floor.
- `engine.print_time_now(mcu_handle)` — estimated print_time at this
  instant from the router record. `None` until clocksync establishes.

`mcu_handle` is the primary MCU's engine handle: print_time is *defined* by
the primary's clock. Passing a secondary's handle gives you that MCU's
`clock/nominal`, which is not the shared timeline.

## The rules

1. **Absolute results only.** Never return or store "seconds from now".
   A relative lead decays between the instant it is computed and the instant
   someone adds a differently-sampled "now" to it. If an API hands you a
   duration, convert it to an absolute time once, immediately, and pass that.
2. **Never compose `estimated_print_time(now) + lead`.** That sum samples
   "now" through one clock path and the lead through another. The one
   sanctioned composition is the *schedule floor* in
   `Motion.get_last_move_time` — a `max()` of absolutes over all engine
   MCUs, which is drift-safe because max cannot accumulate error.
3. **Scheduling after motion**: call `toolhead.get_last_move_time()` if you
   also want mainline's flush-to-rest semantics (most G-code handlers do),
   or `engine.frontier_print_time(...)` if committed motion is the right
   scope and you must not stall the pipe. Know which one you mean: the
   frontier excludes moves still in lookahead.
4. **Waiting for motion to finish** is `toolhead.wait_moves()` (drain) or
   `flush_step_generation()` (drain + MCU execution catch-up). Calling
   `get_last_move_time()` for its side effect and discarding the value is a
   bug pattern — say what you mean.
5. **Sample timestamping** (per-sample hot paths): stay on the cheap pure
   conversions — `mcu.clock32_to_clock64` + `mcu.clock_to_print_time` —
   and correlate positions via `motion_engine.motion_state_at`. Both ride
   the same clocksync estimates; do not mix in wall-clock or host time.
6. **The engine's timeline is complete.** Dwells and nudges advance the
   frontier like segments do (the dispatcher publishes them). If you catch
   yourself adding host-side compensation for something the engine "forgot",
   the engine is where the fix goes.

## Why this exists (the bugs the old model shipped)

- `get_last_move_time` returned `est_main(now₁) + max(lead(now₂), 0.25)` —
  two nows, two clock paths, and only the main MCU's estimate while callers
  scheduled on secondary MCUs → sporadic "scheduled with stale print_time"
  shutdowns.
- Lookahead callbacks fired with `est(now)+lead` at resolution time, which
  could be behind moves queued after registration → out-of-order pin/LED
  scheduling.
- The `_mcu_pending_end_time` shadow clock accumulated per-move lower
  bounds, was hand-bumped for dwells, grounded only on drains, and its
  drifted value fed `get_status().print_time`, `stats()`, and — through
  `mcu.check_active` — clock calibration itself.
- `print_time_to_host_secs` converted print_time through the regression
  frequency (ppm × uptime error) on the EtherCAT servo torque path.
