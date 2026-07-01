# Investigation: MCU foreground stall → host-reset mid-print

**Slug:** mcu-foreground-stall-crash
**Session:** k-1782843288-1946 (Neptune bench, `dderg@ethercatpi5.local`)
**Flashed commit:** `cf2ca03ad` (PushPieces retransmit + clock-sync revert)
**Status:** Root cause CONFIRMED (High) — the stalling task is `console_task` (MCU command processing)
**Date:** 2026-06-30

## Follow-up 2026-06-30 #2 — stalling task named via the new instrumentation

Flashed the per-task-timing build (`45ef61988`). `runtime.fg_task` fired on **three**
crashes (19:14, 19:47, 20:32 UTC), every time naming the same func:
`worst_task_func = 134286529 = 0x8010CC0` → **`console_task`** (`src/generic/serial_irq.c:79`).
Durations: clean **423 µs** (first crash); multi-second (later crashes; the `dur_cyc` field
wraps past ~51 s — cosmetic counter bug, func is solid).

`console_task` calls **`mcu_demux_pump(receive_buf, rpos)`** — it parses and dispatches the
received command stream (PushPieces et al.) in the **foreground**. So the foreground stall is
**command processing**, almost certainly the Rust-side piece application invoked synchronously
from the PushPieces handler. This ties together: the RAM-at-100% pressure (pre-existing on the
64 KB F401), the worsening-over-print pattern, and the user's "slow chip" intuition.

**Leading hypothesis (Medium):** the F401 (84 MHz, 64 KB) cannot process the piece/command
stream as fast as the host sends it → the RX backlog grows → `console_task` spends longer each
pass until it exceeds the host comms deadline → reset. I.e. a command-processing *throughput*
problem of the rewrite on this low-end MCU, not a discrete bug. Alternative: a single
PushPieces hits a pathological/long path in the Rust piece-apply.

**Next instrumentation:** record inside `mcu_demux_pump` the message-kind currently being
dispatched + the RX-buffer backlog depth, so the next crash shows whether it's PushPieces and
whether the buffer is backed up (throughput) vs one slow command.
Also: guard `worst_task_cyc` against the 32-bit wrap on multi-second stalls.

## Follow-up 2026-07-01 #3 — per-command run: contaminated data + a refuted theory

Flashed the per-command build (`1eb027456`). Two crashes (21:36, 22:04 UTC): `fg_task`
= `console_task` again (rock solid). `fg_demux` = backlog 256 / 6 msgs (small). `fg_msg`
= kind `0x275` (22:04) and `0x4200` (21:36).

**The per-command data was CONTAMINATED — my instrumentation bug:** new fields were added
to the persistent `live_snapshot` without bumping `LIVE_MAGIC`, so a reflash (RAM survives,
magic unchanged) skipped the cold-init zero pass and seeded them with stale bytes. Proof:
`0x4200` is impossible from the encoding; `0x275` decodes to Klipper msgid 117 which is in
the unused gap 96–127 (valid ids: −32..95, 128..189).

**Refuted — "malformed command infinite-loops dispatch":** `command_lookup_parser`
(`command.c:259`) does `if (!cmdid || cmdid >= command_index_size) shutdown("Invalid command")`
and `command_parsef` `shutdown("Command parser error")` — both **fail loud fast**, no hang.
And `command_find_block` CRC- and sequence-checks before dispatch. So a bad-msgid block can't
hang; the `0x275` reading was garbage, not a real malformed command.

**Still confirmed:** `console_task` (command processing) stalls ≥2 s; small backlog → not
throughput. The *which-command* answer needs clean data.

**Fix (`2d0518677`):** bumped `LIVE_MAGIC` (forces clean init on any layout change) + added
`runtime.fg_msg_head` capturing the worst/in-progress message's first 4 header bytes
(len,seq,msgid0,msgid1 for Klipper; channel,payload0..2 for kalico) so one clean crash fully
decodes the command, including 2-byte-VLQ msgids. Next run is decisive either way: `fg_msg ≈
fg_task` → a specific dispatched command/frame blocks (head names it); `fg_msg << fg_task` →
the stall is in `console_task` non-dispatch code (feed_byte / the rebase memmove / framing).

## Follow-up 2026-07-01 #4 — DIRECTION OVERTURNED: real cause is a host stream-planner abort

Clean build a3719b05d flashed (magic bump confirmed present; ELF matches flash). The
07:52 crash was traced end-to-end and the MCU-foreground-stall direction is a **red
herring** for these mid-print crashes.

**Confirmed root cause (High):** the host motion-engine stream planner aborted with
`commit: velocity plan: OverCommitted { line_no: 15842 }`
(`event=stream_planner_fatal`, `07:52:00.098Z`, session k-1782887034-7655,
print-1782891567). `fatal()` (`rust/motion-engine/src/stream_planner.rs:386`) →
`std::process::abort()` → coredump `core.kalico-stream-p.7655.1782892320`.

**Downstream (all consequences, not causes):**
- `07:52:00.282` host-ec: "endpoint exiting: bridge (klippy) disconnected — disabling
  drive (**downstream of a host-side abort**)" — the EtherCAT endpoint did NOT self-crash.
- klippy shuts down → sends `emergency_stop` → MCU logs shutdown reason **"Command
  request"** (`src/basecmd.c:383`, `command_emergency_stop`). The MCU was *told* to stop;
  it did not stall.
- `07:52:15` klippy auto-restarts (session 8587), finds MCU "in shutdown state at config
  time". Firmware never power-cycled → same prior-boot `fg_freeze`/`fg_msg`/`fg_task`
  snapshot **replayed** on every reconnect (byte-identical across 06:33/06:43/07:33/
  07:37/07:52). The msgid-117 / dur=cap snapshot is **stale**, not a live stall — it does
  not describe these crashes. (User confirmed: "it didn't firmware restart… just
  restarted the klippy process.")
- No fresh `runtime.fault_latched`/`hard_fault` at the crash — consistent with a clean
  host-requested shutdown, not a firmware fault.

**Recurring (3 aborts in ~12 h, different prints/lines):**
- `07-01 07:52` `OverCommitted { line_no: 15842 }`
- `06-30 22:24` `OverCommitted { line_no: 26933 }`
- `06-30 20:05` `RestAnchorAccel { line_no: 18499 }`

**Mechanism (Confirmed via source):** `OverCommitted` fires when the entry velocity
carried over from the *already-committed* previous look-ahead window exceeds what the
next window's first move can accept — its curvature/accel-bounded entry ceiling
(`rust/geometry/src/velocity.rs:228`) or brake-to-stop reachability (`velocity.rs:329`).
The comment at `rust/motion-engine/src/stream.rs:497-505` names the exact failure: after
a commit seam at an internal sub-piece boundary, the raw move is trimmed to the seam and
the consumed head length is fed back as a blend-budget restore so the trailing corner
re-fits to its pre-commit curvature — "else a shorter front yields a sharper apex and a
corner cap below the already-committed entry velocity — an OverCommitted abort." So the
trim-to-seam / blend-budget-restore does not perfectly reproduce the pre-commit
curvature on some geometry, dropping the recomputed entry ceiling below the committed
hand-off velocity → fail-loud abort (per the fail-loud doctrine, correct behavior; the
bug is the re-fit continuity, not the guard).

**Next:** investigate `trim_front_to_seam` / `committed_head_len` blend-budget restore in
`rust/motion-engine/src/stream.rs`; reproduce offline with klipper-sim on the offending
prints at the named lines (deterministic — same lines recur). MCU/EtherCAT instrumentation
work is not needed for this failure mode.

## Hand-off Brief (15s read)

Mid-print the STM32F401 MCU's **foreground task loop stalled ~200 ms** (other boots: 4–8 s),
starving USB servicing until the host lost communication and **klippy reset the MCU**
(`SFTRSTF`), killing the print. This is **not** the serial-retransmit path and **not**
clock-sync — the firmware logged **no** hardware fault, **no** ISR CPU hog, and a healthy
engine ISR. The one field that would name the stalling task (`last_dispatch`) is clobbered
to the fault monitor, so the trigger is unconfirmed pending better instrumentation.

## Problem Statement

Recurring mid-print crash. User: completed-then-crashed-at-end on the prior commit (irrelevant),
then **crashed mid-print on `cf2ca03ad`**. Same freeze PC signature (`134252664`) as earlier sessions.

## Timeline (Confirmed, from VL + klippy.log)

- `18:14:50` — normal `analog_in_state` activity (print running).
- `~18:15:16` (pre) — MCU becomes unresponsive; host resets it.
- `18:15:16.367` — **crash-replay** on boot 2: `runtime.mcu_reset` (cause `0x14000002`) +
  `runtime.fg_freeze pc=134252664 stall_ticks=5` + `runtime.mcu_ready`.
- `18:15:19.452` — host `bridge`: "live-position poll failed; serving stale cache".
- `18:15:19.6+` — `pump_send_blocked` storm (every ~25 ms) — pump hammering the freshly-reset MCU.

The pump stall is **downstream** of the reset (3 s later), not the cause.

## Confirmed Findings (firmware prior-boot forensics, klippy.log `#output: prior_diag_*`)

- `fg_freeze stall_ticks 5 pc 134252664 exc 0 iwdg 0 last_disp_func 134278857 last_disp_addr 536871792`
  - `pc 134252664 = 0x8008878` → `sched_check_wake` `readb(&w->wake)` (`sched.c:350`, via `run_tasks` `sched.c:381`) — **generic dispatch-loop code**, where the sample landed; not a culprit.
  - `exc 0` → no CPU exception. `iwdg 0` → no IWDG reset.
  - **`stall_ticks 5` < `FG_FREEZE_REPORT_THRESHOLD = 8`** (`fault_handler.c:92`) → firmware's own freeze-reset **did not fire**.
  - `last_disp_func 134278857 = 0x800EEC8` → **`fault_capture_and_reset`** — the fault routine itself, **not** the stalling task. `diag_note_dispatch` (`fault_handler.c:629`) was overwritten by the monitor before capture → field is self-referential / useless here.
- `prior_diag_summary_block: systick 556 stepout 0 stepout_burst 0 usb_burst 0` → **no ISR CPU hog** (the usual root cause is ruled out).
- `tim5ia min 15819 max 17200 last 16826 period 16800` + `tim5_max_cyc 809` + `isr_phase 9` (ISR_EXIT) → **engine ISR healthy** through the freeze; engine innocent.
- `prior_diag_tasks: drain_n 137649 drain_max_gap 95595 stat_n 1878 stat_max_gap 16607896`
  - `stat_max_gap 16607896 cyc / 84 MHz = ~198 ms` foreground stall this boot. **Other boots in rotated logs: `stat_max_gap` 395 M / 345 M / 659 M cyc = 4.7 / 4.1 / 7.8 s.**
- Reset cause `0x14000002` = `SFTRSTF | PINRSTF` (software reset). With no firmware fault, this is the **host (klippy) connect-reset** after losing the MCU (documented masking gotcha).

## Deduced Conclusion (High)

The MCU **foreground task loop stalled** (~200 ms this boot, multi-second on bad boots) with
**no hardware fault, no ISR hog, and a healthy RT engine ISR**. The stall starved USB
servicing past klippy's comms deadline; klippy declared the MCU lost and **software-reset it
mid-print**. That mid-print reset (not the foreground stall itself) is the proximate print-killer.

This is a **distinct, third failure mode**, separate from:
- the serial PushPieces response-loss (fixed by `c3596161f` — still valid, addresses a real different case), and
- clock-sync drift (reverted `cf2ca03ad` — unrelated).

## Hypothesized (needs instrumentation to confirm/refute)

- **H1 — a foreground task blocks/loops ~200 ms.** Heaviest mid-print foreground work is host-command
  processing (PushPieces piece-application via the Rust runtime). A pathological batch/loop would stall
  the foreground without an ISR burst. *Confirm:* per-task timing naming the offending task func.
- **H2 — the stall is in our (rewrite) code, not stock Klipper scheduler.** Engine ISR healthy but
  foreground stalls → a foreground task added by the rewrite (command/bridge path) is the suspect.

## Missing Evidence (itself a finding) → instrumentation backlog

1. **`last_dispatch` names the monitor, not the culprit.** Capture the dispatched task func/addr
   so it survives into the freeze snapshot *without* being overwritten by `fault_capture_and_reset`
   (save-before-run, or exclude the monitor from `diag_note_dispatch`).
2. **No per-task timing.** Add a budget timer around each task in `ctr_run_taskfuncs` (or in
   `diag_note_dispatch`) recording the worst task func + its duration → next freeze names the task.
3. **Freeze forensics are klippy.log-only.** `runtime.fg_freeze` carries only `pc`+`stall_ticks`
   (`log_codes.rs:127`); `last_disp`, `block`, `tim5ia`, `prior_diag_*` reach only the legacy
   `output()` text path. Promote them to the structured store so freezes are queryable in VL.

## Reproduction

Recurring across sessions (same `pc=134252664`). Mid-print on `cf2ca03ad`. No deterministic trigger yet.
