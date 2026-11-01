# Slewed host→MCU clock re-sync for uninterrupted motion

Status: design only. Not implemented — the change reaches the step-shim crate
and both stepcompress and sample sinks, not motion-core alone (see
[Containment](#containment)).

## Problem

Every serial MCU's span clocks are projected through a frozen host→MCU map
(`FrozenProjection { host_ref, mcu_ref, freq }` in
`rust/motion-core/src/worker/pump_sink.rs`). Spans carry that map's slope as
`clock_freq_hz`, and the shim refuses a view whose slope differs from its
lane's (`ShimError::SpanFrequencyMismatch`). The map is re-synced on the live
clock record at every rest (`resync_at_rest`: first segment, idle resume,
dwell rejoin). Between rests it stays frozen, so during uninterrupted motion
the timing error grows as

    error(t) = (freq_frozen / freq_true − 1) · (t − host_ref)

on every MCU independently; the cross-MCU error is the difference. With the
clock estimate converged (≤ 1 ppm on hardware, exact in the sim since the
ready-wait fix) that is ≤ 3.6 ms per hour per MCU; a map frozen within the
estimator's first minute carries tens of ppm (measured 7–87 ppm in the sim
before the fix).

Re-basing the map mid-motion the way a rest does — jumping to the live record
— moves the next span's start clock by the accumulated error δ. A positive δ
is a δ-long stall in the middle of a move, a negative one schedules steps
before steps already queued; either way the toolhead leaves the plan by
`v·δ` at the seam. A mid-motion re-sync therefore cannot jump; it has to slew.

## Design

At each dispatched segment of a continuing epoch (`StreamEpoch::Continuation`,
and a `Rejoin` whose rest re-sync was deferred), for each serial MCU:

1. `e = live(s) − frozen(s)` at the segment's seam `s`, in ticks.
2. If `|e| ≤ SLEW_DEADBAND` (≈ 20 µs · f), keep the map.
3. Otherwise replace it with a map that is **continuous at the seam** and
   whose slope carries the error away over a horizon `H`:

       host_ref' = s
       mcu_ref'  = frozen(s)
       freq'     = freq_live + e / H,
       |freq' / freq_live − 1| ≤ SLEW_PPM_MAX  (H grows to respect the clamp)

4. Later segments re-evaluate `e`; it decays to the deadband within `H`.

The clock never jumps; only the rate changes, by at most `SLEW_PPM_MAX`. The
toolhead's deviation from the plan stays at the error being corrected (`v·|e|`,
shrinking) plus a velocity scale error of `SLEW_PPM_MAX · v` (100 ppm at
300 mm/s is 0.03 mm/s), both inside the clock-sync contract the rest re-sync
already holds.

### What changes

- **`pump_sink.rs`**: `anchor_segment` calls a new `slew_projection(seam)` for
  continuing epochs. It reads the live record (same converged/age gates as
  `reanchor_projection`), applies steps 1–3, and logs every slew
  (`clock_slew`: mcu, error, old/new freq, horizon) the way `clock_resync` is
  logged. The epoch slope handed to the pump (`epoch_freq_of`) becomes
  "the slope this segment was cut against", not only a fresh epoch's.
- **`pump_loop.rs`**: a continuation whose first view's slope differs from the
  lane's `seam_slope_hz` gets a new `LaneCut::Reslope { at_start_clock, freq }`
  — no cut, no gap — routed to the sink.
- **`stepcompress_sink.rs` / `sample_sink.rs`**: a `PendingSeam::Reslope`
  that, when the stream reaches `at`, re-slopes the lane in place: no shim halt,
  no barrier, no MCU traffic. The step count, lattice origin and overlay
  remainder carry across.
- **step-shim** (`ring.rs`, `root_cursor.rs`, `lib.rs`): the queue must admit a
  contiguous view with a new `clock_freq_hz`, and the cursor must switch slope
  at that view's boundary (`retime` keeping count and remainder) instead of
  raising `SpanFrequencyMismatch`. Old-slope views already queued drain first.
- **`anchor.rs`**: unchanged. Slewing is a projection property; epochs keep
  their meaning.

### Invariants

1. Continuity: `frozen'(s) = frozen(s)` within one tick of rounding.
2. Rate bound: `|freq' / freq_live − 1| ≤ SLEW_PPM_MAX`; `freq' > 0`.
3. A slew only ever applies to seams not yet projected — projection happens at
   dispatch, a full lead ahead of the MCU — so no sent span changes.
4. Every slope change reaches the lane as a `Reslope` seam at exactly the view
   it starts; a lane never mixes slopes inside one view.
5. A rest re-sync replaces any slew in progress.
6. A re-slope never moves the step count or drops the sub-step remainder: the
   one-step-per-event class of bug fixed in `count_resting_at` must not come
   back through the cursor.

### Failure modes

- **Estimate noise** would make the map chase noise: the deadband and rate clamp
  bound the reaction; with sub-ppm estimates slews are rare and small.
- **Clock record outliers / reconnects**: the slew uses the same
  converged/age gates as anchoring; an unusable record leaves the map frozen
  and logs, it never guesses.
- **Halt / endstop cut mid-slew**: the cut uses the current (slewed) map; the
  next fresh epoch re-syncs at rest.
- **Nudges** (`dispatch_nudge`) project with the current map and must hand its
  slope to the pump like any segment.
- **Phase (sample) lanes** on serial MCUs need the same `Reslope` seam in the
  sample sink.
- **EtherCAT** keeps no frozen map (live projection against the endpoint's
  grid) and is unaffected.

## Containment

The rest re-sync stayed inside motion-core. The slew does not: the shim's
single-slope-per-lane invariant lives in the step-shim crate (`SpanQueue`
admissibility, cursor retime at a view boundary), and both sinks need a new
seam kind. That is the implementation boundary this note stops at.

## Test plan (for the implementation)

- Unit: `slew_projection` keeps `frozen(s)` continuous, clamps the rate, decays
  `e` below the deadband within `H`, and logs each slew.
- Unit (step-shim): a contiguous view with a new slope re-slopes the cursor at
  its boundary with step count and remainder intact.
- Sim: dual MCUs with skewed crystals (`crystal_ppm`), one long print with no
  dwell, cross-MCU step timing within `CLOCK_SYNC_STABLE_PPM × session +
  pacer slack` (the `test_clock_resync.py` bound without rests).
