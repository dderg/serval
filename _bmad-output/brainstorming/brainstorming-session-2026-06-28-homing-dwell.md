---
stepsCompleted: [1, 2]
inputDocuments: ['_bmad-output/implementation-artifacts/investigations/homing-current-not-applied-investigation.md']
session_topic: 'Redesign planner.dwell() so a dwell is a durable, real pause for ALL consumers (G4, stall/sensor settle, homing current)'
session_goals: 'Diverge on mechanisms for a robust general-purpose dwell primitive that survives drain / idle-re-anchor / home-re-anchor; converge on an approach worth specifying'
selected_approach: 'ai-recommended'
techniques_used: ['First Principles Thinking', 'Morphological Analysis', 'Solution Matrix']
ideas_generated: []
context_file: ''
---

# Brainstorming Session Results

**Facilitator:** dderg
**Date:** 2026-06-28

## Session Overview

**Topic:** Redesign `planner.dwell()` so a dwell is a durable, real pause for ALL consumers — not just the homing-current case where it was noticed.

**Goals:** Diverge on mechanisms for a robust general-purpose dwell primitive that survives drain / idle-re-anchor / home-re-anchor, then converge on an approach worth turning into a spec.

### Problem Context (confirmed)

`planner.dwell()` expresses a pause as `state.advance_time(duration_s)` (stream.rs:235) — a soft bump of the planner's committed-time clock. That bump is durable only while the stream keeps flowing; it is silently erased whenever the timeline is grounded or re-anchored:
- `wait_moves()` → `_ground_pending_end_time_after_engine_drain` (motion.py:636-642) lowers `_mcu_pending_end_time`.
- idle re-anchor: `reanchor = state.is_empty() && esc > t_committed` (stream_planner.rs:575).
- `HomeDrip`: `state.reset(.., 0.0)` + `sync_instant = None/Instant::now()` (stream_planner.rs:638-640).

So dwell fails precisely when it is the **last action before the machine goes idle** — which is most consumers (settle-then-measure, settle-then-home, enable-then-move).

**Consumers (blast radius of any change):** G4 (`cmd_G4`), stepper_enable `DISABLE_STALL_TIME` (×6), sensor settles (`ldc1612`, `angle`, `resonance_tester`, eddy probe), `gcode.py:471`, homing current change.

### Session Setup

_(facilitator approach + technique selection pending)_

## Technique Selection

**Approach:** AI-Recommended Techniques

**Sequence:**
- **Phase 1 — First Principles Thinking:** redefine what a "dwell" is on this machine, below the `advance_time()` assumption.
- **Phase 2 — Morphological Analysis:** enumerate design axes (where the pause lives / what it waits on / what survives re-anchor / who triggers) and combine into candidate mechanisms.
- **Phase 3 — Solution Matrix:** score finalists against durability, MCU-clock correctness, throughput, blast radius, risk.

## Idea Generation

### Phase 1 — First Principles Thinking
