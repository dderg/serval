---
stepsCompleted: [1, 2]
inputDocuments: ['_bmad-output/brainstorming/brainstorming-session-2026-06-28-homing-dwell.md', '_bmad-output/implementation-artifacts/spec-durable-dwell-idle-segment.md']
session_topic: 'Make the durable dwell coherent with the homing path (HomeDrip timeline reset + endstop abort) so the current-settle works without crashing'
session_goals: 'Diverge on mechanisms to keep the dwell timeline-coherent across HomeDrip reset and endstop abort; converge on a safe approach to re-spec'
selected_approach: 'ai-recommended'
techniques_used: ['First Principles Thinking', 'Constraint Mapping', 'Solution Matrix']
ideas_generated: []
context_file: ''
---

# Brainstorming Session Results

**Facilitator:** dderg
**Date:** 2026-06-29

## Session Overview

**Topic:** Make the durable dwell (committed idle segment) coherent with the homing path so the current-change settle works without crashing the MCU.

**Goals:** Diverge on mechanisms, converge on a safe approach worth re-speccing.

### What the bench proved (confirmed, Trident, 2026-06-29 10:20:34)

The durable-dwell branch crashes homing. Sequence:
1. Endstop trips, homing move done (`traveled=90.7`).
2. Post-trip `_set_homing_current(pre_homing=False)` restores run current on all 4 CoreXY motors → each calls `toolhead.dwell()`.
3. Host grounding guard **false-raises**: "grounding would drop committed motion coverage: engine frontier 30.836 past target 29.892 (durable dwell)".
4. MCU fault **65226 = `piece_start_in_past`** (`motion_core.rs:127`, ~95 ms deficit) on BOTH mcu+bottom → dual shutdown.

Earlier in the same home: **rattle** — one belt-pair motor held while the other moved (steppers split across both MCUs), then stabilized. = pre-homing settle dwell's held-position pieces racing the homing drip pieces per-stepper.

### The two confirmed defects

- **D1 (crash):** after the endstop *aborts* the move, planner `t_committed` and the MCU clock diverge (move cut short). The post-trip dwell emits a real held-position piece anchored at the stale `t_committed` → MCU sees start-in-past → PieceStartInPast. The old `advance_time` dwell dispatched no piece, so never tripped this.
- **D2 (guard false-raise):** `_ground_pending_end_time_after_engine_drain` compares `queued_motion_secs()` vs `motion_lead` — fires on legitimate committed dwell coverage during homing finalize, not a real swallow. (Blind Hunter flagged this; mis-triaged as a non-issue.)

### The design assumption that broke

The 2026-06-28 design assumed homing composes as `write@T + idle[T,T+N] + home@T+N` on ONE monotonic timeline. Reality: homing uses **HomeDrip (resets the planner timeline to 0)** and the endstop **aborts mid-move** — so the idle and the home never share a timeline, and the post-abort anchor is stale.

### Session Setup

**Approach:** AI-Recommended. Sequence: Phase 1 First Principles (re-derive each consumer's need; split homing-settle from print-dwell) → Phase 2 Constraint Mapping (HomeDrip reset / abort / MCU-clock / per-MCU rings) → Phase 3 Solution Matrix (converge).

## Idea Generation

### Phase 1 — First Principles Thinking
