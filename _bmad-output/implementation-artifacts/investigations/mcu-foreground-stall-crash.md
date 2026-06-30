# Investigation: MCU foreground stall → host-reset mid-print

**Slug:** mcu-foreground-stall-crash
**Session:** k-1782843288-1946 (Neptune bench, `dderg@ethercatpi5.local`)
**Flashed commit:** `cf2ca03ad` (PushPieces retransmit + clock-sync revert)
**Status:** Active — root mechanism High; stalling task NOT yet pinned (instrumentation gap)
**Date:** 2026-06-30

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
