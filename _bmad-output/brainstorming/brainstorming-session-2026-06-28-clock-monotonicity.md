---
stepsCompleted: [1]
inputDocuments: ['_bmad-output/implementation-artifacts/investigations/trident-homing-ztilt-crash-investigation.md']
session_topic: 'Fixing the motion-history monotonicity panic (host→MCU clock projection jitter) without losing trajectory optimality'
session_goals: 'Enumerate candidate solutions AND the problems each one might cause; converge on the architecturally-correct direction'
selected_approach: 'Progressive flow: diverge on solution candidates, then adversarial pre-mortem on each'
techniques_used: []
ideas_generated: []
context_file: ''
---

# Brainstorming Session Results

**Facilitator:** dderg
**Date:** 2026-06-28

## Session Overview

**Topic:** Fixing the motion-history monotonicity panic (`HistoryStore::record` assert at
`motion_history.rs:138`) — caused by re-projecting host time through a live, re-fit clock-sync model
between dispatches, so host-monotonic piece order does not survive into MCU clocks.

**Goals:** Generate the full space of candidate solutions, and for each, surface the problems /
failure modes it might introduce. Converge on a direction that holds the non-negotiables: print
throughput / trajectory optimality, fail-loud philosophy, and monotonic-by-construction scheduling.

### Confirmed root-cause facts (from investigation case file)
- Panic: `out-of-order piece for AxisKey { mcu_id: 1, axis: 2 }: 447537601286 < 447537603339` (Z, 2053-tick regression).
- `start_clock = PieceEntry.start_time = project(mcu_id, host_secs)`, `host_secs = t0 + curve_u_start + sub_offset` (monotonic in host time).
- `project` = `router.host_time_to_mcu_clock` — live clock model re-fit between dispatches → backward µs-scale jitter.
- Same drift surfaces as the soft homing warning `stale print_time … lead=21.4ms`.
- `record()` runs before `pump_tx.send()`, so the host aborts before the overlapping piece hits the MCU.

### Session Setup
Progressive flow — first diverge widely on solution candidates (no filtering), then run an adversarial
pre-mortem per candidate. Facilitated collaboratively; ideas count when developed in dialogue.

