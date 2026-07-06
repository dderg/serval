# Investigation: M106 does not activate the part fan (partfan branch)

## Hand-off Brief

1. **What happened.** On branch `partfan`, `M106` no longer turns the fan on; the fan extras code is byte-for-byte unchanged vs `main`, so the regression lives below the fan layer in the rewritten host→MCU command transport. **(Confirmed)**
2. **Where the case stands.** The whole command transport was rerouted from the binary serialqueue path to a text-based `engine_send` path through the Rust motion engine; the "unknown-command silent drop" theory is **Refuted** (the full MCU dict is handed to the engine and all fan commands are still declared in unchanged firmware). Two hypotheses remain Open: (H2) the new `MIN_SCHEDULE_LEAD` stale-`print_time` guard on `MCU_digital_out.set_digital`, and (H3) the command is dispatched but mis-scheduled/not-transmitted inside `dispatch_fire_and_forget`.
3. **What's needed next.** Query the structured logs (query-logs skill) for the instrumentation already added on this branch — `fire_and_forget_encode_error`, `[py-trace] _engine_send … cmd=queue_pwm_out/queue_digital_out`, `[config-send] … config_pwm_out` — which deterministically tells us whether the fan command (a) reached the engine, (b) encoded, (c) was transmitted. This is an offline log read, not a live-printer test.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-23                                                                 |
| Status           | Active                                                                     |
| System           | kalico fork, branch `partfan`, motion-engine rewrite; merge-base with main `b3061d21b6` |
| Evidence sources | git diff vs `main`, source trace (klippy + rust/host-rt + src/), Explore subagent JSON |

## Problem Statement

User report: "M106 gcode is not activating the fan. Did we touch any code related to partfan or fan code, compared to main? The diff to main is huge, so first find the fan-related code and diff individually." Registered as Hypothesis #1: the `partfan` branch changed fan/partfan code in a way that breaks M106.

## Evidence Inventory

| Source                          | Status    | Notes                                                                                  |
| ------------------------------- | --------- | -------------------------------------------------------------------------------------- |
| git diff `main` (fan extras)    | Available | `fan.py`, `heater_fan.py`, `heated_fan.py`, `temperature_fan.py`, `fan_generic.py`, `controller_fan.py` — **none changed** |
| git diff `main` (klippy/mcu.py) | Available | 283 insertions / 232 deletions — transport rewrite + new digital_out guard             |
| git diff `main` (serialhdl.py)  | Available | `send`/`send_with_response` rerouted through motion engine                              |
| Rust host-rt passthrough/reactor| Available | Explore subagent traced engine_send → fire_and_forget → parser.encode → dispatch        |
| MCU firmware (pwmcmds/gpiocmds) | Available | All fan commands declared; both files **unchanged** vs main                            |
| Structured runtime logs         | Missing   | Not yet read — the decisive evidence; see Missing Evidence + Backlog                    |

## Investigation Backlog

| # | Path to Explore                                                                 | Priority | Status | Notes                                                                 |
| - | ------------------------------------------------------------------------------- | -------- | ------ | --------------------------------------------------------------------- |
| 1 | query-logs: `fire_and_forget_encode_error` event                                | High     | Open   | Confirms/refutes encode-time silent drop for the fan command          |
| 2 | query-logs: `[py-trace] _engine_send` with `cmd=queue_pwm_out`/`queue_digital_out` | High     | Open   | Confirms the fan command reached the engine at all                    |
| 3 | query-logs: `[config-send]` `config_pwm_out`/`config_digital_out`               | Medium   | Open   | Confirms config phase set up the fan oid on the MCU                    |
| 4 | rust `dispatch_fire_and_forget` (reactor.rs:1070) — does it gate on clock/ready? | High     | Open   | H3: command encoded but never transmitted / mis-scheduled             |
| 5 | Does user's `[fan]` config carry an `enable_pin`? (→ MCU_digital_out path)      | Medium   | Open   | Determines whether the new MIN_SCHEDULE_LEAD guard (H2) is even reachable |
| 6 | MCU-side structured log: did it receive/schedule queue_pwm_out?                 | Medium   | Open   | Distinguishes host-drop vs MCU-reject                                 |

## Confirmed Findings

### Finding 1: Fan extras code is unchanged vs main

**Evidence:** `git diff --stat main -- klippy/extras/fan.py klippy/extras/heater_fan.py …` returns empty. `klippy/extras/fan.py` `_apply_speed`/`set_pwm` path (fan.py:96-129) is untouched.

**Detail:** The break is not in the fan layer. Answers the user's literal question: we did **not** touch fan/partfan extras; the regression is downstream.

### Finding 2: All host→MCU commands are rerouted through the Rust motion engine as text

**Evidence:** `klippy/mcu.py:790` sets `self._motion_engine = printer.lookup_object("motion_engine", None)` on every MCU. `CommandWrapper.send` (mcu.py ~196-218) and `CommandQueryWrapper.send` (mcu.py ~125-176) branch to `self._serial.send(_format_engine_msg(...))` / `_engine_send(...)` whenever `_motion_engine` is present, instead of binary `raw_send`/`_do_send`. `serialhdl.py:592-614` `SerialReader.send` routes to `engine.engine_send(handle, msg)`.

**Detail:** The legacy `steppersync`/`_stepqueues` machinery was removed (mcu.py diff: `self._steppersync = None`, stepqueues deleted). `reqclock` and `minclock` are dropped on the engine path — but the scheduled execution clock is preserved inside the message payload (e.g. `queue_pwm_out oid=5 clock=… value=…`), so the schedule itself is not lost at this layer.

### Finding 3: Encode-time silent-drop path exists but does not apply to fan commands

**Evidence:** Explore trace — `bridge.rs:2348-2365` `engine_send` → `io.send_fire_and_forget(msg)` → `reactor.rs:1050-1092` `ReactorCommand::FireAndForget` → `parser.rs:634-641` `encode()` does `by_command_name.get(name).ok_or(ParseError::UnknownCommand)`. On `UnknownCommand` the command is **silently dropped** with only `event="fire_and_forget_encode_error"` logged (reactor.rs:1084-1091) — no error returns to Python.

**Detail:** This is a real silent-drop mechanism, but it fires only for command names absent from the parser dictionary. See Refuted Hypothesis #3 — the fan commands are present.

### Finding 4: Fan MCU commands are declared in unchanged firmware AND the full dict is given to the engine

**Evidence:** `src/pwmcmds.c:78,105,125` (`config_pwm_out`, `queue_pwm_out`, `set_pwm_out`) and `src/gpiocmds.c:127,141,174,195` (`config_digital_out`, `set_digital_out_pwm_cycle`, `queue_digital_out`, `update_digital_out`) all declared; `git diff --stat main -- src/pwmcmds.c src/gpiocmds.c` is empty (unchanged). `klippy/mcu.py:1257-1259` passes `msgparser.get_raw_data_dictionary()` (the FULL MCU dict) to `self._motion_engine.set_msgproto_dict(raw_dict)`.

**Detail:** Therefore `queue_pwm_out`/`queue_digital_out`/`config_*` are present in the Rust parser's `by_command_name`; `encode()` succeeds for them; they are not dropped as UnknownCommand in the normal case.

### Finding 5: A new fail-loud stale-print_time guard was added to MCU_digital_out.set_digital

**Evidence:** `klippy/mcu.py:482-501` — `set_digital` now raises `command_error("digital_out … scheduled with stale print_time …")` when `print_time < est + MIN_SCHEDULE_LEAD` (`MIN_SCHEDULE_LEAD = 0.050`, mcu.py:25). `MCU_pwm.set_pwm` (mcu.py:617-626) has **no** such guard.

**Detail:** The standard `[fan]` part fan uses `setup_pin("pwm", …)` → `MCU_pwm` (fan.py:61), so the guard is reachable only via a configured `enable_pin` (fan.py:76,109-111) or a digital_out-based fan. If reachable, it would surface as a loud `command_error`, not a silent no-op.

## Deduced Conclusions

### Deduction 1: The fan command is constructed and (at minimum) handed to the engine

**Based on:** Findings 1, 2, 4.

**Reasoning:** Fan layer untouched → `set_pwm` is called with a valid `print_time`/`value`; transport routes it to `engine_send`; the command name is in the parser dict so `encode()` does not reject it.

**Conclusion:** The failure is either (a) inside `dispatch_fire_and_forget` after a successful encode (not transmitted, or scheduled at a bad clock), or (b) the loud-guard path (H2) if the user's fan is digital/`enable_pin`-based. A plain silent "unknown command" drop is unlikely for the default part fan.

## Hypothesized Paths

### Hypothesis 1: partfan branch changed fan code (user's premise)

**Status:** Refuted

**Resolution:** All fan extras and the fan MCU firmware commands are unchanged vs `main` (Findings 1, 4). The regression is in the rewritten transport, not the fan code. The user's instinct that "we touched something related" is correct in spirit — the broken layer is the engine command transport, not the fan modules.

### Hypothesis 2: New MIN_SCHEDULE_LEAD guard rejects the fan's set_digital (stale print_time)

**Status:** Open

**Theory:** If the fan path reaches `MCU_digital_out.set_digital` (enable_pin or digital fan), and the rewritten engine produces an `estimated_print_time` ahead of the fan's scheduled `print_time` by < 50 ms, the new guard raises `command_error` and the pin is never set.

**Supporting indicators:** Guard is new on this branch (mcu.py:482-501); the fan from console schedules at `estimated_print_time(now)+lead`, a small lead that can fall under MIN_SCHEDULE_LEAD if the engine's clock estimate runs ahead.

**Would confirm:** A `command_error` containing "digital_out … scheduled with stale print_time" in the logs at the time of M106; user's `[fan]` config has `enable_pin`.

**Would refute:** User's fan has no enable_pin and uses MCU_pwm only (guard unreachable); no such error in logs.

### Hypothesis 3: Command encodes but is not transmitted / mis-scheduled in dispatch_fire_and_forget

**Status:** Open

**Theory:** After a successful `encode()`, `dispatch_fire_and_forget(payload, false)` (reactor.rs:1070) may gate transmission on MCU readiness / clock sync, or schedule at a clock the MCU treats as past/far-future, so the pin change never effectively occurs.

**Supporting indicators:** Transport was fully rewritten (steppersync removed); `reqclock`/`minclock` dropped on the engine path; heavy diagnostic instrumentation was added around exactly this path (see Side Findings), suggesting an in-progress hunt for this class of bug.

**Would confirm:** `[py-trace] _engine_send` shows the fan command sent but no corresponding MCU schedule/ack; or MCU-side log shows reject/"in the past".

**Would refute:** Logs show the fan command transmitted and the MCU scheduling the pin at the expected clock (→ look further toward pin/PWM cycle setup).

## Missing Evidence

| Gap                                          | Impact                                                              | How to Obtain                                                        |
| -------------------------------------------- | ------------------------------------------------------------------- | -------------------------------------------------------------------- |
| Structured logs around an M106 invocation    | Decides between H2 (loud guard), H3 (transmit/schedule), encode-drop | query-logs skill: filter `fire_and_forget_encode_error`, `_engine_send`, `config-send` |
| User's `[fan]` config (enable_pin? hw pwm?)  | Determines whether H2 guard path is even reachable                  | Read printer.cfg `[fan]` section                                     |
| `dispatch_fire_and_forget` body behavior     | Confirms whether encoded commands are unconditionally transmitted   | Read rust/host-rt reactor.rs around 1070 + dispatch_fire_and_forget  |

## Source Code Trace

| Element       | Detail                                                                                              |
| ------------- | --------------------------------------------------------------------------------------------------- |
| Error origin  | Not a single line yet; failure is in the engine transport below `klippy/extras/fan.py` (untouched)  |
| Trigger       | `M106` → `PrinterFan.set_speed_from_command` (fan.py:134,192) → gcrq → `_apply_speed` → `MCU_pwm.set_pwm` (fan.py:96-129) |
| Condition     | `_motion_engine` present → command routed as text via `serialhdl.send` → `engine_send` → `fire_and_forget` → `parser.encode` → `dispatch_fire_and_forget` |
| Related files | klippy/mcu.py (25,196-218,482-501,617-626,1257-1259), klippy/serialhdl.py (592-630), klippy/extras/fan.py, klippy/extras/output_pin.py (GCodeRequestQueue), rust/motion-engine/src/bridge.rs:2348, rust/host-rt/src/host_io/reactor.rs:1050-1092, rust/host-rt/src/host_io/parser.rs:634-641, src/pwmcmds.c, src/gpiocmds.c |

## Conclusion

**Confidence:** Medium

The user's premise — that fan/partfan code was changed — is **Refuted** in the literal sense: every fan extra and every fan MCU-firmware command is unchanged vs `main`. What changed is the entire host→MCU command transport, now routed through the Rust motion engine as text (`engine_send` → fire-and-forget → `parser.encode` → `dispatch_fire_and_forget`). The "unknown-command silent drop" theory is refuted for the default part fan because the full MCU dictionary is handed to the engine and the fan commands remain declared. The surviving root-cause candidates are the new `MIN_SCHEDULE_LEAD` stale-`print_time` guard (H2, only if the fan uses a digital/enable pin) and a transmit/scheduling failure inside `dispatch_fire_and_forget` (H3). The structured logs — for which this branch already carries targeted instrumentation — will discriminate between them in a single offline read.

## Recommended Next Steps

### Diagnostic

1. **query-logs** (offline) around an M106 event:
   - `event="fire_and_forget_encode_error"` → if present with a fan command, encode-drop after all (would re-open Hyp #3-style drop).
   - `[py-trace] _engine_send … cmd=queue_pwm_out` / `cmd=queue_digital_out` → confirms the command reached the engine.
   - `command_error … "digital_out … scheduled with stale print_time"` → confirms H2.
   - `[config-send] … config_pwm_out`/`config_digital_out` → confirms fan oid setup.
2. Read the user's `[fan]` config to settle whether H2's `MCU_digital_out` path is reachable.
3. Read `dispatch_fire_and_forget` (reactor.rs:~1070) to confirm whether a successful encode is unconditionally transmitted.

### Fix direction

Deferred until the logs discriminate H2 vs H3 — investigation stops at diagnosis.

## Side Findings

- This branch carries heavy diagnostic instrumentation already wired into exactly this path: `[py-trace] _engine_send enter/exit` (mcu.py CommandQueryWrapper._engine_send), `[config-send]`/`[config-send-restart]` (mcu.py ~1001-1010), `[trsync-diag]` (mcu.py MCU_trsync._handle_trsync_state), and `fire_and_forget_encode_error` (reactor.rs:1085). This strongly suggests the engine command-transport path was already under active debugging — likely for this or a sibling symptom. **(Confirmed)**
- `reqclock`/`minclock` are silently dropped on the engine send path (`engine.engine_send(handle, msg)` takes only the text). Harmless for commands that carry their execution clock in-payload (fan, stepper), but a latent footgun for any command that relied on transport-level `reqclock` ordering. **(Deduced)**
- `_is_engine_transport_drop` (serialhdl.py:529-533) silently swallows sends only when the error string contains "transport closed" (connection-down), and latches `_engine_detached`. Narrow, so not the generic fan-failure mechanism — but it does mean any send after a transport drop is silently no-op'd. **(Confirmed)**

## Follow-up: 2026-06-23 #2

**Trigger:** User merged latest `sota-motion` after the initial pass; the original investigation ran on pre-merge code. Re-anchored every stronghold against current HEAD `93110834a`.

### Re-verification of prior findings (all survive the merge)

| Prior finding | Current state | Verdict |
| ------------- | ------------- | ------- |
| F1: fan extras unchanged vs `main` | `git diff --stat main -- klippy/extras/{fan,heater_fan,heated_fan,temperature_fan,fan_generic,controller_fan}.py` empty | **Holds** |
| F4: fan MCU firmware unchanged | `git diff --stat main -- src/pwmcmds.c src/gpiocmds.c` empty | **Holds** |
| F5: stale-`print_time` guard | `mcu.py:25` `MIN_SCHEDULE_LEAD=0.050`; guard at `mcu.py:482-501` (`set_digital`, check at 490). `MCU_pwm.set_pwm` `mcu.py:617-626` still guard-free | **Holds** (line shift only) |
| F2/F3: engine-transport routing + encode UnknownCommand drop | `_engine_send` `mcu.py:128`; `serialhdl.py:599` `engine.engine_send`; encode-drop `reactor.rs:1083-1092` (`event="fire_and_forget_encode_error"`) unchanged behavior | **Holds** |

merge-base with `main` still `b3061d21b6` — fan-relevant comparison baseline did not move.

### New evidence: dispatch_fire_and_forget was reworked by the merge

**Confirmed.** The merge changed exactly the H3 code path. Most recent transport commit: `d437ce8a0 fix(motion): pace backpressure on the dispatched frontier, not the submitted one`; new test file `rust/host-rt/src/host_io/reactor/a8_fire_and_forget_backpressure.rs`.

`dispatch_fire_and_forget` (`reactor.rs:325-374`) now behaves:
1. `unacked_window` **not full** → build frame, `write_frame`, push to unacked window. Command IS transmitted. (the normal idle-printer case)
2. window **full**, pending `< PENDING_FIRE_AND_FORGET_CEILING` (256, `reactor.rs:238`) → **queued** to `pending_fire_and_forget`, redispatched later by `drain_pending_submissions` (`reactor.rs:416-446`). Not dropped.
3. window **full** AND pending `>= 256` → **refused** with `TransportError::Backpressure`, logged `event="fire_and_forget_ceiling"`. Only here is the command lost.

### Impact on hypotheses

- **H3 (encoded-but-not-transmitted) — weakened.** The only loss path is case 3, a deep-saturation condition (window full + 256 queued fire-and-forgets). A single manual `M106` on an otherwise idle printer does not plausibly reach it. If H3 is the cause, the merge's reworked path makes it *less* likely, and it would now surface as a loud `fire_and_forget_ceiling` error rather than a silent drop. New refute criterion: absence of `fire_and_forget_ceiling` / `fire_and_forget_send_error` at M106 time refutes H3-via-dispatch.
- **H2 (stale-`print_time` guard) — unchanged.** Still reachable only via a digital/`enable_pin` fan. Default soft-PWM part fan routes through `MCU_pwm.set_pwm` (no guard).
- **Decisive gap unchanged but must use POST-MERGE logs.** Any pre-merge log evidence is now stale. The query-logs read (`fire_and_forget_encode_error`, `[py-trace] _engine_send cmd=queue_pwm_out/queue_digital_out`, `fire_and_forget_ceiling`, `command_error … stale print_time`, `[config-send] config_pwm_out/config_digital_out`) must be run against a session created on current HEAD.

### Open question gating all of this

**Unconfirmed:** whether `M106` still fails to activate the fan on the merged code. The merge touched the prime H3 suspect — it may have changed or fixed the symptom. This single fact (does it still reproduce post-merge?) is the cheapest discriminator and must be re-established before further log analysis is worth running.

## Follow-up: 2026-06-23 #3 — ROOT CAUSE CONFIRMED

**Trigger:** Bench experiment on `/neptune-bench`. User swapped the part-fan and hotend-fan pins. Result: the heater-driven fan now spins on the swapped port when the heater is on; `M106` still does nothing on any port.

### Decisive experiment (Confirmed)

The pin swap exonerates hardware, wiring, the physical pin, and the MCU PWM output: a fan on the part-fan port spins when the *heater_fan* path commands it. The fault is exclusively in the command path **unique to `M106`/part fan**.

### The fork between working and broken paths (Confirmed)

Both `heater_fan` and the `[fan]` part fan end at the same `Fan._apply_speed` → `MCU_pwm.set_pwm`. They diverge in how the request is scheduled, both inside `klippy/extras/output_pin.py` `GCodeRequestQueue` (file unchanged vs `main`):

- **heater_fan (WORKS):** `heater_fan.py:44` `fan.set_speed(speed)` → `fan.py:131` → `GCodeRequestQueue.send_async_request` (`output_pin.py:69`). When `print_time is None` it computes `print_time` from `mcu.estimated_print_time` and **invokes the callback inline** (`output_pin.py:77` `self.callback(next_time, value)`). Never depends on flush callbacks or the toolhead.
- **M106 (BROKEN):** `set_speed_from_command` (`fan.py:134`) → `GCodeRequestQueue.queue_gcode_request` (`output_pin.py:64`) → `register_lookahead_callback` fires `_queue_request` (`output_pin.py:60`), which **appends to `rqueue`** and calls `note_mcu_movequeue_activity`. The request is applied only later by `_flush_notification` (`output_pin.py:33`), which is registered via `mcu.register_flush_callback` (`output_pin.py:27`).

### Root cause (Confirmed, High)

**`MCU.flush_moves` in the rewrite is a no-op stub that never invokes `_flush_callbacks`.**

- `klippy/mcu.py:1535-1536` — `def flush_moves(self, print_time, clear_history_time): return`
- `_flush_callbacks` is appended to at `klippy/mcu.py:1532` but **never iterated anywhere in `klippy/`** (grep: only the init site `mcu.py:779` and append site `mcu.py:1532` exist).
- Contrast mainline `main:klippy/mcu.py:1464-1470`, whose `flush_moves` runs `for cb in self._flush_callbacks: cb(print_time, clock)`.

Therefore `GCodeRequestQueue._flush_notification` is dead on this branch. M106 appends its request to `rqueue` and nothing ever drains it → `_apply_speed`/`set_pwm` are never called → the fan never turns on. `heater_fan` is unaffected because `send_async_request` applies inline and never touches the flush-callback machinery.

**Contributing secondary stub:** `ToolheadShim.note_mcu_movequeue_activity` (`klippy/motion.py:1434-1435`) is `pass`. Mainline (`main:klippy/toolhead.py:776`) advances `need_flush_time` and kicks the flush timer. Even if `flush_moves` fired periodically, nothing advances the flush horizon to cover an idle, gcode-queued request. Both stubs must be addressed for the queued path to work.

### Hypothesis resolutions

- **H1 (fan code changed):** Refuted (unchanged) — but the user's instinct was directionally right: the rewrite broke a host-side seam the fan path depends on.
- **H2 (MIN_SCHEDULE_LEAD guard):** **Refuted.** Default part fan uses `MCU_pwm` soft-PWM (no guard), and the failure is upstream — `set_pwm` is never reached because the request is never drained.
- **H3 (dispatch_fire_and_forget transmit/schedule):** **Refuted.** `heater_fan` working proves the engine transport delivers fan commands. M106 never generates an MCU command at all.

### Blast radius (Deduced)

Every consumer of `register_flush_callback` whose work is purely callback-driven is broken on this branch: `GCodeRequestQueue.queue_gcode_request` — i.e. `M106`/part-fan, `[output_pin]` `SET_PIN` async requests, and any other gcode-queued pin/PWM/LED request routed this way. Inline `send_async_request` consumers (heater_fan, temperature_fan, controller_fan) are unaffected.

### Fix direction (investigation stops at diagnosis)

The rewrite must drive `_flush_callbacks` from wherever the motion engine advances the host→MCU flush timeline — invoking `cb(print_time, clock)` for each registered callback, equivalent to mainline `flush_moves` — and `ToolheadShim.note_mcu_movequeue_activity` must advance the engine's flush horizon and kick a flush so idle gcode-queued requests (M106 with no following motion) are drained. This is an architectural seam in the rewrite, not a one-liner.

### Conclusion (supersedes prior): Confidence High

Deterministic, code-traced root cause with a matching bench experiment. `M106` does not activate the fan because `MCU.flush_moves` (`klippy/mcu.py:1535-1536`) was stubbed to a no-op and never invokes the `_flush_callbacks` that drain `GCodeRequestQueue`, leaving the queued part-fan request stranded in `rqueue`. The heater-fan path survives only because it bypasses that machinery via the inline `send_async_request`.
