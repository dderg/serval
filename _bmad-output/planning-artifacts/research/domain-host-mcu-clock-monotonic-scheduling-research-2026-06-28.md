# Domain Research — How industry keeps a streaming motion schedule monotonic under live host→device clock sync

**Date:** 2026-06-28
**Author:** dderg (facilitated)
**Driving problem:** `HistoryStore::record` panic (`motion_history.rs:138`) — host-monotonic piece order does
not survive projection into MCU clocks because `host_time_to_mcu_clock` is a live model re-fit between
dispatches (µs-scale backward jitter → fail-loud assert → SIGABRT). See investigation case file.

---

## The question

When a host streams a motion schedule to a device whose clock is being continuously re-synchronized,
how do real systems guarantee the dispatched/recorded times stay monotonic — without freezing the clock
estimate (which would drift) and without silently shoving motion later (which hides faults)?

## Prior art surveyed

### 1. Klipper upstream — the most direct analogue (it has our exact hazard)
`clocksync.py` converts host/print time ↔ MCU clock through a **moving linear regression** updated on every
`get_clock` response. Confirmed properties:
- `print_time_to_clock(pt) = int(pt * mcu_freq)`; system-time path `get_clock(t) = clock + (t - sample_time)*freq`.
- Regression updated by exponential decay `DECAY = 1/30`; `new_freq = clock_covariance / time_variance`.
- **The mapping is NOT mathematically monotonic** — "it can step backward slightly if frequency estimates
  drop." Same hazard we hit. (WebFetch of clocksync.py.)
- Mitigations Klipper actually relies on:
  1. **Heavy smoothing / slew** — decayed regression + outlier rejection (`25 * prediction_variance`) +
     `Resetting prediction variance` guard, so `freq` moves slowly and continuously.
  2. **Forward-anchored recalibration** — `SecondarySync.calibrate_clock()` solves `adjusted_freq`/
     `adjusted_offset` to align at a sync point **~4+ seconds in the future**, so corrections take effect
     *beyond the dispatch horizon*, never retroactively near the frontier.
  3. **Single authoritative invariant, enforced at the boundary, not mid-pipeline** — Klipper does NOT put
     a process-fatal host-side per-piece monotonicity assert. "Scheduled in the past" is checked **on the
     MCU** → `Timer too close` → clean MCU **shutdown** (printer fault state), not a host `SIGABRT`.
- The proposed multi-MCU sync refinement PR #6753 was **rejected** by K. O'Connor (redundant timer update;
  the query-decrement change "would introduce a regression"). So upstream's answer remains: smooth + forward
  sync point + MCU-side boundary check — not a tighter host model.

### 2. PTP / IEEE-1588 clock discipline — the controls-theory principle
- Universal rule: **slew, don't step.** Small errors → ppb-level *frequency* correction (slewing) that keeps
  time monotonic; large errors → atomic *step* that "can cause problems for applications that require a
  smooth and monotonically increasing time base." Smoothness-critical paths avoid steps.
- Servos are **PI controllers + low-pass filters** (PTPd; fuzzy-PI variants in the literature). Corrections
  are continuous by construction.

### 3. Linux kernel monotonic clock / adjtimex
- `CLOCK_MONOTONIC`: NTP **stepping has no effect**; only **slewing** applies. "Slewing itself does not cause
  time to go backward — a vital guarantee." The OS answer to "I need a never-backward timeline off a
  disciplined clock" is to expose a view that *only ever slews*.

### 4. EtherCAT Distributed Clocks (industrial multi-axis motion)
- Different architecture: a **hardware DC in every node**, one 64-bit/1 ns **reference time** distributed and
  disciplined to <100 ns. There is **one authoritative monotonic counter**; the host does not re-project a
  per-cycle host→device map. Coordinated motion rides a single common time base.

---

## Synthesis — the three recurring patterns

Robust systems combine these; none rely on a single one:

- **P1 — Slew, never step.** Clock-model corrections are slow continuous *frequency* adjustments, never
  discontinuous offset jumps. (PTP, NTP, CLOCK_MONOTONIC.) → validates "fix the model continuity" and
  condemns abrupt per-dispatch re-fits.
- **P2 — Apply corrections beyond the commit/lookahead horizon.** Klipper's "sync ~4 s ahead": the mapping is
  only ever changed at a future point past everything already scheduled, so near-frontier ordering is
  immutable. → the lookahead/commit-horizon discipline.
- **P3 — One authority for the monotonicity invariant, checked at the boundary, failing loud-but-clean.**
  Klipper checks "in the past?" once, on the MCU (`Timer too close` → clean shutdown). EtherCAT has one DC
  reference. Nobody sprinkles *process-fatal* asserts on a *derived* structure that independently re-projects.

## Mapping to our candidate solution families (from the brainstorm)

| Family | Industry verdict |
|---|---|
| A. Single per-stream clock anchor (affine, project once) | Validated — EtherCAT single-time-base + Klipper forward sync. Monotonic by construction. |
| D. Phase-continuous / slew-limited clock model | Validated as the *cause* fix (P1) — but biggest blast radius; Klipper keeps smoothing heavy AND adds P2 rather than trusting smoothing alone. |
| B/C. Clamp forward (`max(projected, last)`) | **Anti-pattern.** Clamping forward = a silent "step"/pad — exactly what PTP avoids and CLAUDE.md forbids; masks real lateness faults. |
| E. History in host-time domain, project at query | Partially aligns with "don't duplicate the invariant on a derived structure," but leaves the wire untouched; weaker than P3. |
| F. Graceful shutdown instead of SIGABRT | **Strongly validated** — Klipper's analogous failure is a clean MCU shutdown, never a host process abort. Preserves fail-loud while keeping the process alive to report. |

## Recommendation direction (evidence-based)

The industry-standard combination, translated to our pipeline:
1. **Make the schedule monotonic by construction** — adopt Family A (single per-stream anchor) or Family D
   (slew-limited model), so `start_time` cannot invert near the frontier (P1).
2. **Don't re-project inside the committed window** — apply any clock-model correction only beyond the
   lookahead horizon, Klipper-style (P2).
3. **Enforce "monotonic / not-in-the-past" at ONE authoritative boundary that fails loud-but-clean** — keep a
   single check (pump/MCU egress), convert the history assert off `panic→SIGABRT` to a clean shutdown
   (Family F), and let the derived history *trust* the authoritative schedule (P3) rather than re-asserting it.

## Open questions for next pass
- Does our MCU step queue already reject in-the-past absolute clocks (a `Timer too close` analogue)? If yes,
  the host-side assert is a *redundant* second authority and P3 says remove/demote it.
- What is the real magnitude/frequency distribution of the backward jitter on the bench? (Quantify before
  picking any horizon/slew constant — instrument a histogram.)
- Could pieces carry *relative* clocks and let the pump assign absolute clocks at a single monotone egress
  point (a clean P3 implementation)?

## Sources
- [klipper/clocksync.py (master)](https://github.com/Klipper3d/klipper/blob/master/klippy/clocksync.py)
- [Klipper PR #6753 — Improve multi-MCU clock sync (rejected)](https://github.com/Klipper3d/klipper/pull/6753)
- [Klipper Code Overview — time](https://www.klipper3d.org/Code_Overview.html)
- [Klipper "Timer too close" knowledge base](https://klipper.discourse.group/t/timer-too-close/6634)
- [PTPd source documentation (clock servo, slew vs step)](https://ptpd.sourceforge.net/doc.html)
- [Discrete model of IEEE 1588 PTP with PI clock servo (IEEE Xplore)](https://ieeexplore.ieee.org/document/8754102/)
- [Adaptive Fuzzy-PI clock servo for IEEE 1588 (ResearchGate)](https://www.researchgate.net/publication/340214644)
- [Baeldung — Timekeeping and Clocks in Linux (slew vs step, monotonic)](https://www.baeldung.com/linux/timekeeping-clocks)
- [Clock Synchronization and Monotonic Clocks — I. Pandzic](https://inelpandzic.com/articles/clock-synchronization-and-monotonic-clocks/)
- [EtherCAT Motion Control guide — Elmo](https://www.elmomc.com/elmo_academy/ethercat-motion-control/)
- [acontis — Distributed Clocks (DC) synchronization](https://www.acontis.com/en/dcm.html)
- [Synchronous multi-axis motion via modified EtherCAT DC (IEEE Xplore)](https://ieeexplore.ieee.org/document/9327605/)
