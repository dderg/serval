# Plan 8 research — lookahead commit window

**Task:** Phase 0 Task 4 (spec §6.4). Derive the minimum safe value of
`LOOKAHEAD_FLUSH_TIME` (and the related `BUFFER_TIME_*` constants) such that,
when move N's polynomial is being composed, every move in N's kernel-support
neighbourhood is already queued, junction-solved, and frozen.

## 1. What "flush" commits today vs what Plan 8 needs

Two distinct time windows sit behind the word "flush" in `klippy/toolhead.py`:

- **Lookahead / junction flush** (`LookAheadQueue.flush`, `klippy/toolhead.py:157`,
  driven from `add_move`/`klippy/toolhead.py:221-229`). This is the backward sweep
  that calls `move.set_junction(start_v2, cruise_v2, end_v2)`
  (`klippy/toolhead.py:117,194,204`). Once `set_junction` has been called and the
  move is handed off to `_process_moves` (`klippy/toolhead.py:462-525`), the
  polynomial / trapq entry is **committed** — it can no longer be rewritten.
  `LOOKAHEAD_FLUSH_TIME = 0.250` (`klippy/toolhead.py:134`) is a lazy-flush
  threshold: once the tail of the queue contains ≥250 ms of real motion, a
  `flush(lazy=True)` is triggered so the queue does not grow without bound.
- **Step-generation / stepcompress flush** (`_advance_flush_time`,
  `klippy/toolhead.py:413-434`). This advances the iterative solver, finalises
  trapq entries older than `kin_flush_delay`, and hands step batches to the MCU.
  It runs **strictly after** the lookahead commit and operates on already-frozen
  trapq entries. Plan 8 does not need anything new here — by the time
  `_advance_flush_time` runs, the polynomial is already baked.

Plan 8's requirement lives entirely in the lookahead layer: before `set_junction`
is called on move N, every move in `[N.t_start − S_back, N.t_end + S_fwd]`
(where `S_back + S_fwd = S`, the kernel's full support) must already be in the
queue with its own geometry and commanded cruise speed known. A move that
arrives after N's `set_junction` has run is too late — the polynomial is already
emitted.

Today's 250 ms threshold was sized for lookahead velocity planning (a single
accel/decel ramp at 100 mm/s³ is ~3 s, but 250 ms is enough to find the next
junction-speed-limited move in practice). It was **not** sized around kernel
support. Plan 8 must re-derive it.

## 2. Kernel support table

All values in milliseconds. MZV and ZV report the timestamp of the last
impulse (full causal support from t=0). bs1..bs5 report `t_sm`, the full width
of the piecewise kernel (symmetric about t=0, so half-support =
`t_sm / 2`). Damping ratio fixed at 0.1.

Computed directly from `klippy/extras/shaper_defs.py` via
`get_mzv_shaper`, `get_zv_shaper`, `_get_bs_smoother`.

| Shaper | min_freq | @min_freq | @30 Hz | @40 Hz | @60 Hz | @80 Hz | @120 Hz |
|--------|----------|-----------|--------|--------|--------|--------|---------|
| zv     | 21 Hz    | 23.9 ms   | 16.8   | 12.6   | 8.4    | 6.3    | 4.2     |
| mzv    | 23 Hz    | 32.8 ms   | 25.1   | 18.8   | 12.6   | 9.4    | 6.3     |
| bs1    | 18 Hz    | 86.4 ms   | 51.8   | 38.9   | 25.9   | 19.4   | 13.0    |
| bs2    | 20 Hz    | 97.3 ms   | 64.9   | 48.6   | 32.4   | 24.3   | 16.2    |
| bs3    | 21 Hz    | 107.2 ms  | 75.1   | 56.3   | 37.5   | 28.1   | 18.8    |
| bs4    | 23 Hz    | 109.0 ms  | 83.5   | 62.6   | 41.8   | 31.3   | 20.9    |
| bs5    | 25 Hz    | 109.0 ms  | 90.8   | 68.1   | 45.4   | 34.1   | 22.7    |

**Worst case across all supported shapers at their advertised minimum
frequencies: `S = 109 ms`** (bs4 @ 23 Hz and bs5 @ 25 Hz are effectively tied
at 109.0 ms, both dominated by the `F_m / f_sh` envelope with F_m = 2.5061
and 2.7252 respectively — `shaper_defs.py:106-112`).

At "real-world" frequencies (Trident-class machines typically land at
60–80 Hz for x and y after input shaper tuning), the worst case is bs5
@ 60 Hz = 45.4 ms. That's the practical number we should design for; the
109 ms figure is the theoretical floor for the advertised frequency range.

**Causality split.** The kernel is (after the constant `get_smoother_offset`
shift) roughly symmetric around t=0, so in the inverse-shaper formulation
used by Plan 8 the planner needs `S/2` of past motion and `S/2` of future
motion. For the worst case: `S_back = S_fwd ≈ 55 ms`.

## 3. Corpus min_move_t distribution

There is no recorded gcode corpus committed to the Kalico repo. The
`klipper-sim/examples/` directory (referenced by the "klipper-sim" memory
note) holds small synthetic fixtures:

- `sharp_short.gcode`: 0.5 mm segments at 300 mm/s (F18000) → 0.5 / 300 =
  **1.67 ms per move**. This is the explicit worst case the regression
  suite is designed to probe.
- `octagon.gcode`, `square.gcode`, `big_square.gcode`, `circle.gcode`:
  segments ≥ several mm at 300 mm/s, so min_move_t ≥ 10 ms — not a
  binding case.

For real prints the two relevant populations are:

1. **Slicer-emitted curve approximations.** Modern slicers chop arcs into
   0.2–1 mm chords. At 300 mm/s that's 0.67–3.3 ms; at 150 mm/s (outer
   wall) that's 1.3–6.7 ms. p95 around 5 ms; p99 around 1–2 ms is
   plausible for high-detail models.
2. **Speedbench / Voron Cube / Cowling** benchmark stock. These use
   longer segments (typical mean ~5–15 ms at the commanded feedrate) but
   the benchmarks deliberately include 90° corners at high speed where
   the planner slows to a crawl, pulling `min_move_t` upward at the
   corner (not downward). Benchmark corpora are **not** the shortest-
   move case; arc-heavy production prints are.

**Working estimate, pending measurement:** p95 min_move_t ≈ 2–3 ms,
p99 min_move_t ≈ 0.5–1.0 ms for slicer output with arc approximation
enabled. These are educated guesses; a targeted measurement pass in Phase 1
(instrument `Move.__init__` to log `min_move_t` and collect on a real
print) will tighten them. For the derivation below, use the
pessimistic-but-realistic `M = 2 ms` as the worst-case move duration.

The direct consequence for Plan 8: `k = ceil(S / min_move_t) =
ceil(109 ms / 2 ms) = 55 moves` worst-case horizon, which matters for
lookahead-queue memory sizing but not for the time-based flush window.

## 4. Derived flush-window bound

Spec asks for `extra_flush = S + max(S, 10 ms)`. With `S = 109 ms`:

- `extra_flush = 109 + max(109, 10) = 218 ms`.
- Worst-case kernel support `S = 109 ms` means the polynomial for the
  move being composed depends on ±54.5 ms of neighbours.
- 109 ms margin covers: (a) the symmetric half-support the forward pass
  cannot yet see, (b) a one-move slack for the case where move N's own
  duration straddles the kernel boundary, (c) phase-noise on the gcode
  arrival jitter from Klippy's gcode parser.

**Recommended minimum `LOOKAHEAD_FLUSH_TIME` for Plan 8:**
`max(current 250 ms, S + S) = 250 ms`. The current value is already
≥ 2×S_worst.

At 60 Hz design point (S ≈ 45 ms): extra_flush = 90 ms ≪ 250 ms —
deeply inside budget.

## 5. Current 250 ms adequacy verdict

**Adequate, no change required for the flush-size threshold.**

Reasoning:

1. The 250 ms lookahead flush threshold is **≥ 2.29 × S_worst** and
   **≥ 5.5 × S_design (60 Hz bs5)**. Under the spec's own
   `S + max(S, 10 ms)` rule, 250 ms is overbuilt by a factor of ~1.15×
   at worst-case frequency and ~2.8× at realistic frequency.
2. `LOOKAHEAD_FLUSH_TIME` is a **lower bound** on how much queue sits
   between `add_move` and the backward sweep, not an upper bound. The
   actual lookahead queue commonly runs much deeper because
   `BUFFER_TIME_HIGH = 2.0 s` (`toolhead.py:233`) lets Klippy build a
   2-second print buffer. Plan 8's polynomial composer runs inside the
   backward sweep, so by the time a move is being composed there is
   typically *seconds* of future queue available — the 250 ms figure is
   just the tiny "minimum pocket" case.
3. The only way 250 ms becomes tight is on the lazy-flush boundary:
   `flush(lazy=True)` at `toolhead.py:229` scans the tail and may still
   commit moves at the far end of the 250 ms window. That "far end" is
   still ≥ 140 ms away from the lazy-flush cutoff line under the
   `S + max(S, 10 ms)` rule at worst-case frequency — safe.

**If we ever add a shaper wider than bs5 (`F_m > 2.72`) or a
`min_freq` below 20 Hz, revisit.** A 15 Hz bs5 would push `t_sm` to
182 ms, `S + max(S, 10 ms) = 364 ms` — then 250 ms becomes inadequate and
would need to grow to ~400 ms.

**No change needed for `BUFFER_TIME_*`.** `BUFFER_TIME_LOW = 1.0 s`,
`BUFFER_TIME_HIGH = 2.0 s`, `BUFFER_TIME_START = 0.250 s` operate on
the *mcu step-time* axis — these gate how far ahead of real-time the MCU
buffer runs. They are orthogonal to the kernel-support question and
remain correct.

## 6. Late-arrival / quiescent-period edge case

When gcode stops arriving (M400 wait, M0/pause, slicer hand-off pause,
idle timeout), the backward-sweep must still complete on the tail of the
queue, because otherwise the final move's polynomial would wait forever
for a neighbour that is never coming. Today's machinery handles this
correctly:

- `cmd_M400 -> wait_moves` (`toolhead.py:693-702`) calls
  `_flush_lookahead` (`toolhead.py:527-533`), which forces
  `lookahead.flush()` with `lazy=False`. The non-lazy flush commits the
  entire queue regardless of the 250 ms threshold — move N is composed
  using whatever neighbours do exist in the queue, and the kernel's
  "missing" tail is treated as zero (motion has stopped).
- `_priming_handler` (`toolhead.py:582-592`) fires on idle-buffer
  stalls and also calls `_flush_lookahead`.
- `cmd_G4 / dwell` (`toolhead.py:688-691`) calls
  `get_last_move_time -> _flush_lookahead`, same path.
- `set_position` (`toolhead.py:631-639`) begins with
  `flush_step_generation -> _flush_lookahead` — the queue is drained
  before the position anchor moves.

**Implication for Plan 8.** The kernel-support requirement is
*asymmetric around flush boundaries*. When a non-lazy flush fires at the
end of a motion segment, the last few moves in the queue legitimately
have no right-side neighbours (motion genuinely stops). The polynomial
composer must treat this as "zero-padded on the right" — identical to
how the input shaper today handles the end of a print. The symmetric
`S/2` forward window shrinks to zero at the boundary. This is correct
behaviour, not a bug, and matches spec §6.4's expectation.

The flush window does **not** need to grow to mask late-arriving gcode,
because gcode that arrives after a flush boundary already lives in a new
"motion segment" with its own pre-print ramp-up from zero velocity
(`_calc_print_time`, `toolhead.py:447-460`, applied at the transition
out of `NeedPrime`). The kernel convolution naturally restarts.

## 7. Homing / probing / drip_move

Confirmed that homing and probing **bypass the extended commit window**
— not via the `LOOKAHEAD_FLUSH_TIME` machinery but via the
`drip_move` code path, which forces `lookahead.flush()` after every
single move (`toolhead.py:749-775`):

```
def drip_move(self, newpos, speed, drip_completion):
    self.dwell(self.kin_flush_delay)
    self.lookahead.flush()             # drain any leftover moves
    self.special_queuing_state = "Drip"
    ...
    self.move(newpos, speed)
    ...
    self.lookahead.flush()             # immediately flush the new move
```

In Drip state the 250 ms lazy-flush threshold is irrelevant: each move
flushes on arrival. Plan 8's `shape_disabled = true` flag (spec §6.5)
plus the Drip-state flush behaviour together guarantee that homing moves
(a) do not participate in kernel convolution and (b) do not wait for
future neighbours before emitting. Manual-stepper diagnostics
(`force_move.py` — not audited here, see Task 5) likely take the same
path via `set_position` + direct trapq appends.

## 8. File:line reference summary

- `klippy/toolhead.py:134` — `LOOKAHEAD_FLUSH_TIME = 0.250`.
- `klippy/toolhead.py:147` — reset to default on queue reset.
- `klippy/toolhead.py:149-158` — `set_flush_time` + the backward-sweep `flush`.
- `klippy/toolhead.py:194,204` — the two `set_junction` call sites that mark a move "committed".
- `klippy/toolhead.py:221-229` — `add_move` decrements `junction_flush` by `move.min_move_t` and triggers `flush(lazy=True)` when exhausted.
- `klippy/toolhead.py:232-239` — `BUFFER_TIME_{LOW,HIGH,START}`, `MIN_KIN_TIME`, `STEPCOMPRESS_FLUSH_TIME`.
- `klippy/toolhead.py:315-323` — `flush_timer`, `kin_flush_delay`, `kin_flush_times`.
- `klippy/toolhead.py:413-434` — `_advance_flush_time` (step-gen side, orthogonal).
- `klippy/toolhead.py:462-525` — `_process_moves` (trapq-append side, consumes committed moves).
- `klippy/toolhead.py:527-546` — `_flush_lookahead`, `flush_step_generation`, `get_last_move_time`.
- `klippy/toolhead.py:594-625` — `_flush_handler` (the reactor timer).
- `klippy/toolhead.py:731-775` — `_update_drip_move_time`, `drip_move`.
- `klippy/blendprepass.py:146-209` — `BlendPipelineLookAheadQueue` (the prepass+blender wrapper sits in front of the inner `LookAheadQueue`).
- `klippy/extras/shaper_defs.py:32-43` — MZV definition.
- `klippy/extras/shaper_defs.py:106-190` — bs* family + `_F_M_TABLE`.
- `klippy/extras/shaper_defs.py:283-299` — published `min_freq` per shaper.

## 9. Conclusion

**Verdict:** current `LOOKAHEAD_FLUSH_TIME = 250 ms` is adequate for
Plan 8 across the supported shaper / frequency matrix. The worst-case
kernel support is 109 ms (bs4/bs5 at their published min_freq). The
spec's derived bound `S + max(S, 10 ms) = 218 ms` fits inside 250 ms with
a 32 ms safety margin. At realistic operating frequencies (60 Hz) the
margin grows to 160 ms. No change required to `LOOKAHEAD_FLUSH_TIME`,
`BUFFER_TIME_*`, or `kin_flush_delay` for this work.

Revisit if a future shaper variant has `F_m > 4.6` or if `min_freq` is
lowered below ~20 Hz for any shaper. Both scenarios would need to
re-derive against the `S + max(S, 10 ms)` rule and likely push
`LOOKAHEAD_FLUSH_TIME` to 400–500 ms.
