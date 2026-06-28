# Investigation: Trident homing stale-schedule warning + z_tilt_adjust crash

## Hand-off Brief

1. **What happened.** On the Trident, `Z_TILT_ADJUST` randomly crashes with `KeyError: 'pos'`
   (`beacon.py:544`): the beacon proximity probe is a Z-only move, and any dispatch **re-anchor**
   during the slow descend runs the engine's **global `motion_history.clear()`**
   (`rust/motion-engine/src/bridge.rs:3285`), which wipes stationary X/Y endpoints the Z-only
   segment never re-records → `position_at_clock` can't return x/y → returns `None` → the sample
   gets no `pos` → beacon's unguarded `samples[0]["pos"]` crashes the host. Confirmed end-to-end.
2. **Where the case stands.** Root cause Confirmed, High confidence. The PG5 stale-print_time
   warning, `seg0_deficit` (always +250 ms healthy lead), and the once-per-session window-miss
   warning are all **noise/red herrings**, refuted as the cause. The older -308/-310 MCU faults are
   a **separate** bug (Bug B).
3. **What's needed next.** Fix the engine root: on re-anchor, **rebase per-axis to the held
   endpoint** (`motion_history.rs:161` `rebase_axis`) instead of the global `clear()`, so stationary
   axes still answer position queries; add a fail-loud guard in beacon's `_probe` for a missing
   `pos`. Then verify on the bench with repeated `Z_TILT_ADJUST`.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-28                                                                 |
| Status           | Concluded — root cause Confirmed (High); fix direction open                |
| System           | Trident bench (`dderg@trident.local`), H723 main 'main' + F446 bottom 'bottom'; branch `trident-crash` |
| Evidence sources | `klippy/mcu.py`, z_tilt_ng/probe code, VictoriaLogs (Trident), prior case files |

## Problem Statement

User report: "sometimes when I start homing I get `digital_out PG5 on mcu 'bottom' scheduled with
stale print_time: print_time=302.508044 estimated_now=302.486613 lead=21.4ms (< 50ms)` in console.
It doesn't crash; I retry homing and it works. Then during z_tilt_adjust it randomly crashes —
sometimes right away, sometimes after a few probing cycles, sometimes it finishes the cycle
correctly, but more often than not it crashes."

Premise check (pending): the two symptoms are treated as one disease (insufficient scheduling lead
after queue drain). To be confirmed/refuted by the crash fault evidence.

## Evidence Inventory

| Source   | Status    | Notes     |
| -------- | --------- | --------- |
| `klippy/mcu.py:482-509` `MCU_digital_out.set_digital` | Available | Stronghold — emits Symptom A's exact warning when `print_time < est + MIN_SCHEDULE_LEAD` |
| `MIN_SCHEDULE_LEAD = 0.050` (`klippy/mcu.py:25`) | Available | 50 ms host guard; lead observed 21.4 ms |
| Guard origin commit `0a90fb4ad` (2026-06-10, dderg) | Available | "mcu: reject stale-print_time digital_out schedules on the host" — guard is the user's own fail-loud check |
| Trident VictoriaLogs (z_tilt crash session) | **Missing** | Decisive: actual MCU fault code + event chain during a crash |
| Prior case `piece-start-in-past-clock-rebase-investigation.md` | Available | Same family (clock projection → -308) on Neptune under print load |
| Trident `printer.cfg` (what PG5 is) | Partial | Need to confirm PG5 = motor-enable / probe / output pin on 'bottom' |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Query Trident VL for the z_tilt_adjust crash fault code + chain | High | Open | Decisive; user can reproduce |
| 2 | Identify what PG5 on 'bottom' is and what schedules its digital_out at homing start | High | Open | Pins whether it's motor-enable, probe, or output_pin |
| 3 | Trace how the rewritten pipeline seeds/advances print_time after the move queue drains (homing/dwell) | High | Open | Suspected disease — analog of Klipper BUFFER_TIME_START / min_restart_time |
| 4 | Compare host `estimated_print_time` vs scheduled print_time during sparse ops | Medium | Open | Distinguishes lead-collapse from clock-projection skew |

## Confirmed Findings

### Finding 1: `seg0_deficit ≈ +250 ms` is healthy designed lead, not "in the past"

**Evidence:** `rust/motion-engine/src/anchor.rs:62` sets `t0 = host_now + lead_secs − seg_t_start`
with `DEFAULT_LEAD_SECS = 0.25` (anchor.rs:2); `seg0_deficit` (`router.rs:532`) =
`start_time − ack_now` = `host_now + 0.25 − now` ≈ +250 ms. It is logged on every `fresh` anchor
(bridge.rs:3237-3242). The log string itself reads "negative deficit_us => in past".

**Detail:** The ~250 ms `seg0_deficit` the first log pass flagged as "250 ms behind" is a POSITIVE
lead — the intended 0.25 s cushion. It is always-on noise on every re-anchor, NOT the anomaly.
(User flagged this; Confirmed.)

### Finding 2: Every re-anchor unconditionally wipes motion history

**Evidence:** `rust/motion-engine/src/bridge.rs:3285-3290` — `if fresh { motion_history…clear() }`.
`clear()` (motion_history.rs:156-159) drops all per-axis rings + endpoints.

**Detail:** `anchor_segment` (anchor.rs:36-66) returns `fresh = true` on first segment,
`timeline_reset` (`seg_t_start < last_t_end`, i.e. idle/stream restart), or `underrun`
(`t0 + seg_t_start < host_now`, the playhead overran a thinly-committed slow segment). Slow homing
and beacon-probe moves live in exactly this regime, so re-anchors — and history wipes — are frequent
there, and rare during a continuous buffered print.

### Finding 3: The crash is a silent missing-axis `None` from `position_at_clock`, not the warned window-miss

**Evidence:** Log facts (re-query): `seg0_deficit` is always **+250 ms, never negative** (7d negative
count = 0) — pure noise. The `beacon: dropping stream sample … precedes retained motion history`
warning fires **once per session in ~50 sessions, crash and healthy alike** (a startup artifact); in
the crash session it fired **18 s before** the KeyError. The crash discriminator is **`KeyError:
'pos'`** (`exception` field, `beacon.py:544 samples[0]["pos"]`), present in exactly 4 sessions, all
Z_TILT/probe. **No warning precedes the KeyError** → the `None` came from a silent path, not the
warned `BeforeRetainedWindow` (which warns on first occurrence, `beacon_motion_engine.py:227-237`).

**Detail:** The silent path is the missing-axis one: `motion_state_at_clock` (`bridge.rs:4496-4499`)
**skips `NoHistoryForAxis` axes with `continue`**, returning a dict missing those axes; the seam
`position_at_clock` (`beacon_motion_engine.py:212-219`) does `state["x"][0], state["y"][0],
state["z"][0]` and on `KeyError` returns `None` — no log. `beacon.py:952-953` only sets
`sample["pos"]` when `pos is not None`, so the sample has no `'pos'`; `_probe` (`beacon.py:544`)
reads `samples[0]["pos"]` unconditionally → `KeyError: 'pos'` → `Internal error on command
"Z_TILT_ADJUST"` → host commands shutdown (mcu/bottom reason "Command request").

### Finding 4: Re-anchor `clear()` wipes stationary X/Y, which a Z-only probe never re-records — the root

**Evidence:** `clear()` wipes **both** `rings` and `endpoints` for **all** axes
(`motion_history.rs:156-159`). `state_at_clock` returns `NoHistoryForAxis` when an axis has neither
ring nor endpoint (`motion_history.rs:198-201`). A beacon proximity descend is a **Z-only** move; the
re-anchored segment re-records only Z (`bridge.rs:3291-3301`), so X/Y stay empty after the wipe.

**Detail:** The probe descend is slow and thinly-committed → underrun-prone → `fresh` re-anchor
during the descend → global `clear()` → X/Y lose ring + endpoint → `position_at_clock` returns
`None` (Finding 3) → crash. Because a stationary axis had a valid held position that `clear()`
needlessly discarded (vs `rebase_axis`, which preserves an endpoint), the engine throws away
answerable state. This is the proximate root: the global wipe is too aggressive.

### Finding 5: -308/-310 (06-23…06-27) is a SEPARATE bug, not this crash

**Evidence:** Those sessions show MCU shutdown reason **"kalico runtime fault"** with a real fault
code and **no `KeyError`**; several began "automated MCU restart … was in shutdown state at config
time". The Z_TILT crash sessions show host reason **"Command request"**, `KeyError 'pos'`, **no fault
code**. (Log re-query.)

**Detail:** Premise correction: the user's reproducible Z_TILT crash is Bug A (host beacon
`KeyError`). The older MCU `PieceStartInPast`/`StepsPerSampleExceeded` faults are Bug B, a distinct
issue (likely during actual motion / restart recovery). They share only the background noise
(`seg0_deficit`, once-per-session beacon-drop).

### Finding 6: A continuous print avoids the bug (explains the one successful print)

**Evidence:** Deduced from Findings 2-3. A buffered print keeps the committed frontier
`buffer_time` ahead of the playhead (anchor.rs:24-26 doc), so no underrun, no `timeline_reset`, no
`fresh` → motion history accumulates uninterrupted → beacon queries always land inside the window.

## Hypothesized Paths

### Hypothesis 1: Print_time lead collapses below the MCU safety margin after the queue drains

**Status:** Refuted (as the crash cause) — Partially confirmed for Symptom A only

**Resolution:** The z_tilt crash is NOT a lead-collapse / scheduling-in-past event. `seg0_deficit`
never goes negative (+250 ms designed lead). The crash is the beacon `KeyError 'pos'` from a
re-anchor history wipe (Findings 3-4). The "lead collapse" idea survives only for **Symptom A** (the
PG5 21 ms warning), which is a genuine but separate, non-fatal host-guard event — see Side Findings.

**Theory:** During homing and between z_tilt probe points the move queue drains; the rewritten
pipeline fails to re-seed the toolhead print_time far enough ahead of `estimated_print_time`, so
the next scheduled command lands with tiny lead. The host guard catches the digital_out case
(Symptom A, non-fatal); a stepper/piece schedule with no equivalent host guard slips into the
MCU's past → hard fault → crash (Symptom B).

**Supporting indicators:** Both symptoms occur in sparse, probe-heavy, low-velocity contexts (not
under print throughput load); lead observed at 21.4 ms (positive but small); the guard is brand new
so previously these slipped through to the MCU silently.

**Would confirm:** Crash fault is a "start in past" / "timer too close" class fault on 'bottom'
during a probe move, correlated with a collapsed host lead just before it.

**Would refute:** Crash fault is unrelated (e.g. comms/USB, watchdog, an assert in z_tilt math), or
the lead is healthy at crash time.

### Hypothesis 2: Same clock-projection divergence as the prior case (two writers feed the anchor)

**Status:** Refuted

**Theory:** The host→MCU clock anchor is biased by a competing writer (per
`piece-start-in-past-clock-rebase`), making projected start_times early relative to the MCU.

**Supporting indicators:** Prior case Confirmed this mechanism on Neptune; lead=21.4 ms is within
the scale of projection excursions.

**Would confirm:** VL shows `set_clock_est_rebased` / projection-divergence events on Trident around
the crash, matching the prior signature.

**Would refute:** Crash correlates with queue-drain lead collapse, not with a clock rebase event.

**Resolution:** No `set_clock_est_rebased` / `junction_jump_anomalous` / `projection_divergence`
events occur in any of the 4 crash sessions. Refuted.

## Conclusion

**Confidence:** High — root cause Confirmed end-to-end in code and corroborated by the log crash
signature; only the exact `None` sub-path (silent missing-axis vs repeat silent window-miss) carries
minor uncertainty, and both reduce to the same root.

**Root cause (z_tilt crash, "Bug A"):** A beacon proximity probe is a Z-only move. The dispatch
anchor re-anchors (`fresh`) on the slow, thinly-committed descend (underrun) or on idle restart, and
**every re-anchor calls the global `motion_history.clear()`** (`bridge.rs:3285`), wiping the rings
**and endpoints** of *all* axes — including stationary X/Y, which the Z-only segment never
re-records. `position_at_clock` then needs x, y, z but X/Y are `NoHistoryForAxis`
(silently skipped by `motion_state_at_clock`, `bridge.rs:4498`) → the seam returns `None`
(`beacon_motion_engine.py:217-219`) → beacon never sets `sample["pos"]` → `_probe`'s unconditional
`samples[0]["pos"]` (`beacon.py:544`) raises `KeyError: 'pos'` → host shutdown. The race (a re-anchor
landing during the probe) explains the intermittency; a continuous print keeps all axes populated,
explaining the single successful print.

Two cooperating defects: the engine's **over-broad history wipe** (root) and beacon's **unguarded
`samples[0]["pos"]`** (proximate amplifier that turns a transient `None` into a fatal `KeyError`).

## Recommended Next Steps

### Fix direction

- **Engine (root, preferred): stop the global wipe from destroying answerable state for axes not in
  the re-anchored segment.** On `fresh`, instead of `motion_history.clear()` (which drops endpoints
  for every axis), **rebase each axis to its current held endpoint at the new baseline** (the
  `rebase_axis` path already exists, `motion_history.rs:161-164`) — or clear only the axes the new
  segment will re-record. A stationary axis then still answers `state_at_clock` with its hold
  position, so `position_at_clock` returns a full x/y/z and the beacon sample keeps its `pos`. This
  removes the crash *and* preserves correctness. (`rust/motion-engine/src/bridge.rs:3281-3301`,
  `rust/motion-engine/src/motion_history.rs`.)
- **Beacon (defense in depth / fail-loud):** `_probe` should not assume `samples[0]["pos"]` exists.
  Either retry/skip samples lacking `pos`, or raise a clear `command_error` ("beacon sample lacked a
  resolvable toolhead position") instead of a bare `KeyError`. This is the external beacon fork
  (`~/klipper/klippy/extras/beacon.py:544,567`); a host KeyError that aborts a probe is exactly the
  "fail loudly with a clear error" the project mandates — make the failure legible, but fix the
  engine so it does not occur.
- Do **not** widen `MIN_SCHEDULE_LEAD` or touch `seg0_deficit` — both are red herrings here.

### Diagnostic (to confirm before/after a fix)

- Add (temporarily) a one-line warn in `motion_state_at_clock` when it `continue`s on
  `NoHistoryForAxis` (currently silent) — it would have named the missing axis at crash time.
- Reproduce with `Z_TILT_ADJUST` and watch for `[anchor-decision] condition=reanchor` /
  `anchor_underrun` events interleaved with the probe; correlate a re-anchor immediately preceding a
  `position_at_clock` → `None`. LogsQL: `event:anchor_underrun OR _msg:"anchor-decision"` within the
  probe window.

## Reproduction Plan

1. Home, then run `Z_TILT_ADJUST` repeatedly on the Trident with the beacon probe.
2. Expected (pre-fix): intermittent `KeyError: 'pos'` (`beacon.py:544`) → `Internal error on command
   "Z_TILT_ADJUST"` → host shutdown, more often than not, correlated with an `anchor_underrun` /
   `reanchor` during the slow descend.
3. Post-fix (engine rebase): no `KeyError`; `position_at_clock` returns full x/y/z across re-anchors.

## Side Findings

- **Symptom A (the PG5 stale-print_time warning) is a separate, non-fatal issue.** `digital_out PG5
  on mcu 'bottom'` is scheduled at homing start with ~21 ms lead, below the 50 ms `MIN_SCHEDULE_LEAD`
  host guard (`mcu.py:25,490`), 5 occurrences in 7d, all ~21-22 ms. The guard (commit `0a90fb4ad`)
  catches it and the retry succeeds. It co-locates with the crash sessions only because both arise
  from the same sparse homing/probe regime, not because one causes the other. Worth a follow-up:
  why a homing-start digital_out lands with only ~21 ms lead (idle-restart print_time seeding).
- **`seg0_deficit` and the once-per-session beacon window-miss warning are background noise** — neither
  is diagnostic of the crash. Consider down-ranking `seg0_deficit` from `warn` to `debug` to reduce
  noise (it logs on every re-anchor with the healthy +250 ms lead).
- **Bug B (older -308/-310 MCU faults)** remains unaddressed and distinct — track separately if it
  still reproduces.

## Source Code Trace

| Element       | Detail                                      |
| ------------- | ------------------------------------------- |
| Error origin (crash) | `beacon.py:544` `samples[0]["pos"]` → `KeyError: 'pos'` |
| Proximate cause | `position_at_clock` returns `None` (`beacon_motion_engine.py:212-219`) because the engine dict lacks x/y |
| Root cause | global `motion_history.clear()` on re-anchor (`rust/motion-engine/src/bridge.rs:3285`) wipes stationary X/Y endpoints; Z-only probe never re-records them → `NoHistoryForAxis` skipped at `bridge.rs:4498` |
| Trigger       | `fresh` re-anchor (`anchor.rs:36-66`: underrun on slow Z descend, or idle restart) during a beacon probe |
| Related files | `rust/motion-engine/src/{bridge.rs,anchor.rs,motion_history.rs}`; `klippy/extras/beacon.py`, `klippy/extras/beacon_motion_engine.py` (external fork) |
| Symptom A origin (separate) | `klippy/mcu.py:490-502` `MCU_digital_out.set_digital` — non-fatal host guard, ~21 ms lead on PG5 at homing start |

## Follow-up: 2026-06-28 — fix applied

### Engine (root)
`rust/motion-engine/src/motion_history.rs`: replaced `HistoryStore::clear()` (which dropped rings
**and** endpoints for all axes) with `drop_pieces_on_reanchor()` — clears only the ring pieces and
**keeps each axis's endpoint**. The re-anchor call site (`bridge.rs:3285`) now calls it. A stationary
axis the re-anchored Z-only probe segment never re-records now answers `state_at_clock` with its held
position instead of `NoHistoryForAxis`, so `position_at_clock` returns a full x/y/z and the beacon
sample keeps its `pos`. Strictly improves `final_position` consumers (bridge.rs:3845, homing.rs:85),
which previously defaulted/errored on the wiped endpoints. New unit test
`drop_pieces_on_reanchor_keeps_unrecorded_axis_answerable`; full motion-engine + host-rt suites green
(641 passed).

### Beacon (fail-loud defense in depth)
`~/Developer/beacon_klipper/beacon.py`: added `_require_sample_pos(samples)` — returns the first
sample with a resolvable `pos`, else raises a clear `command_error` instead of a bare `KeyError:
'pos'`. `_probe` now routes both `samples[0]["pos"]` accesses (lines 544, 567) through it.

### Log clarity (prior turn)
`seg0_deficit` → `seg0_lead` (`lead_us`/`lead_ticks`, positive = healthy lead); now `debug` when
healthy and a distinct `seg0_start_in_past` **warn** only when genuinely negative.

### Status: Concluded — fixes applied, pending bench verification (rebuild host `.so` + redeploy beacon, repeat `Z_TILT_ADJUST`).

## Follow-up: 2026-06-28 #2 — "crashed just like before" — premise does not match evidence

User reports the crash recurred after deploy. Evidence on the bench contradicts a recurrence of the
original z_tilt `KeyError: 'pos'`:

### Confirmed — both fixes ARE deployed and active
- Engine: `~/klipper/klippy/_motion_engine.so` rebuilt 2026-06-28 12:28:42 (+0200), AFTER fix commit
  `0ffad2f89` (12:13). `strings` shows `seg0_lead`, `seg0_start_in_past`, and `drop_pieces_on_reanchor`
  (×2). The old `seg0_deficit` log is gone from the new binary.
- Beacon: `beacon.py` symlinks to `~/beacon_klipper` on `fix/probe-resolve-toolhead-pos` (`d5237e6`,
  has `_require_sample_pos`); `.pyc` rebuilt 12:29.
- Note: healthy `seg0_lead` is now `debug`-level → filtered from VictoriaLogs. Its absence in VL does
  NOT mean the old `.so` — corrected an earlier mis-read.

### Confirmed — the original crash has NOT recurred since deploy
- Last `KeyError: 'pos'` / `Internal error on command "Z_TILT_ADJUST"`: 09:23–09:24 UTC, sessions
  k-1782638256-4226 / k-1782638626-4437 — both BEFORE the ~10:28 UTC deploy.
- Post-deploy sessions k-1782642561-7265 (10:29) and k-1782643050-7469 (10:37, current) show only
  startup (Args, Config file); **no G28 / Z_TILT / probe / "ready" / trigger events** — the printer
  was not driven to z_tilt. The new beacon fail-loud message never fired either.

### New issue A (non-fatal): `Failed to load module 'beacon_kalico'` every boot
- `printer.py:135 import_module('klippy.extras.beacon_kalico')` → `ModuleNotFoundError`. Active config
  has `[beacon]`, NOT `[beacon_kalico]`; no active `.py`/`.cfg` references `beacon_kalico` (only `.bak`
  configs + log files). A stale **dangling symlink** `extras/beacon_kalico.py → ~/beacon_klipper/beacon_kalico.py`
  (renamed away) remains. klippy continues past it (MCUs come up), so non-fatal — but it should be
  cleaned and the source of the optional load request identified.

### New issue B (one-time): MCU-reset / USB-burst storm in session 7265
- 10:29:42 UTC: `runtime.mcu_reset` (×2), `runtime.isr_phase ring_overflow=82000`,
  `runtime.block_source usb_burst=138295 cyc`, `diag.tx_drop_kalico/klipper`, `attach_open_retry`,
  then `pump send_mcu_frames failed` (×2) at 10:36:58. Did NOT recur in session 7469. This is an
  MCU-comms/firmware-class event, orthogonal to the motion_history bug — needs the `mcu-diagnostics`
  skill if it is what the user actually hit.

### Open question (blocking): which crash did the user see, and when?
The evidence shows the original bug fixed-and-not-recurring, plus two unrelated new signals. Need the
user to (a) state the exact console text + approximate time, or (b) re-run `G28` + `Z_TILT_ADJUST`
now so a live post-deploy session can be captured.

### Status: Active — awaiting user repro/clarification; original root cause remains fixed per binary + log evidence.

## Follow-up: 2026-06-28 #3 — CONFIRMED root cause of the live crash: motion-history monotonicity panic

The user re-ran the flow ("crashed, just says moonraker can't connect to klipper"). This is a **different**
root cause than the original Python `KeyError: 'pos'` (which remains fixed) — a hard **Rust panic** that
aborts the planner thread.

### Confirmed (journald, session PID 8062, crash at 13:10:47 local)
- `systemd: klipper.service: Main process exited, code=killed, status=6/ABRT` → klippy dies → moonraker
  cannot connect. Auto-restarted 13:10:57.
- Panic: `thread 'kalico-stream-planner' panicked at motion-engine/src/motion_history.rs:138:13`
  - `out-of-order piece for AxisKey { mcu_id: 1, axis: 2 }: 447537601286 < 447537603339`
  - axis 2 = **Z**; regression = **2053 ticks** (~µs-scale, NOT a re-anchor baseline jump).
- Backtrace: `HistoryStore::record` (motion_history.rs:138) ← `init_planner::{{closure}}` (bridge.rs:3296)
  ← `dispatch_committed` (stream_planner.rs:447) ← `run_loop` (stream_planner.rs:736), spawned thread.
- Bench HEAD == worktree HEAD == `0ffad2f89` (line numbers authoritative).
- Pre-restart klippy.log ends abruptly mid-probe (Z lift to 8, then fast XY travel to 270,5) with no
  Python traceback — consistent with the native abort.

### Deduced (High confidence) — mechanism
- `HistoryStore::record` (motion_history.rs:138) asserts `piece.start_clock >= last.start_clock` per
  (mcu,axis) ring — the fail-loud monotonicity guard. It fired with a `last` present, so the crashing
  dispatch recorded against an existing baseline (non-`fresh`, or the segment right after a `fresh` one).
- `start_clock = PieceEntry.start_time` (motion_history.rs:50), set in enqueue.rs:210-211:
  `host_secs = t0 + curve_u_start + sub_offset` (monotonic in HOST time) → `start_time =
  project(mcu_id, host_secs)` where `project` = `router.host_time_to_mcu_clock` (bridge.rs:3245-3248).
- `host_time_to_mcu_clock` uses the **live clock-sync model, re-fit between dispatches**. A
  later-in-host-time piece in the next segment can project to a slightly EARLIER MCU clock than the
  prior dispatch's last piece (here 2053 ticks). Host-monotonic ordering does not survive projection
  through a mutating clock model → the assert trips → panic → SIGABRT.
- Same projection drift produces the soft homing warning the user reports
  (`stale print_time … lead=21.4ms (< 50ms)`). One root cause, two surfaces (soft at homing, hard in
  history record). Z-during-probing is most exposed: many tiny, closely-spaced moves → inter-piece
  clock gaps small enough for projection jitter to flip their order. Explains "sometimes right away,
  sometimes after a few cycles, sometimes finishes."
- record() runs BEFORE pump_tx.send (bridge.rs:3296 then 3300), so the host aborts before the
  overlapping piece reaches the MCU — hence no MCU-side complaint at the crash.

### Fix direction (needs user decision — architectural)
- **(B, recommended) Make the per-stream host→MCU projection monotonic by construction.** Anchor the
  MCU-clock origin once per stream/re-anchor and derive each piece's start_time from accumulated
  durations off that anchor (or carry the last dispatched MCU clock forward so each segment starts
  exactly where the previous ended), instead of re-projecting each dispatch through the live model.
  Removes the jitter at the source, also fixes the soft homing "stale print_time" warning, and matches
  the project's "fail loudly / monotonic-by-construction" philosophy. The wire schedule shares
  `start_time`, so this is a motion-correctness fix, not just an observability one.
- **(A, not recommended) Relax/clamp the history assert** (e.g. `start_clock = max(start_clock, last)`).
  Papers over the symptom in the derived lookup structure, diverges history from the wire, and violates
  CLAUDE.md "do not advance/pad times — raise an error."

### Status: Active — root cause CONFIRMED (High). Awaiting user decision on fix direction (B recommended).
