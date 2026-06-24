# Investigation: Junction position discontinuity (pump.rs fail-loud panic)

## Hand-off Brief

1. **What happened.** The streaming planner emits two consecutive Bézier pieces on the **serial Y axis (mcu0 axis1)** that meet at the same host time but with a **position gap** (0.14–0.41 mm); `check_junction_position_continuity` (`rust/motion-engine/src/pump.rs:352`) fail-loud `panic!`s, aborting klippy (SIGABRT, core-dump). (Confirmed)
2. **Where the case stands.** Active — root cause localized to the **core streaming commit/seam path** (this branch's own work). The check is sound (coeffs are Bézier control points; comparison is valid), the gap is a *real* trajectory discontinuity, and the merged shaper/PA post-processor is **ruled out** (input shaper disabled; PA is extruder-only). Not yet localized to the specific seam mechanism.
3. **What's needed next.** Deterministic offline repro from the captured cube gcode through `motion-engine` (example/sim) with the panic active, then bisect ours' seam logic (`trim_front_to_seam` / `brake_to_rest_setback` / `commit_stall_brake` / `head_trim_feasible`) to the producing boundary.

## Case Info

| Field            | Value |
| ---------------- | ----- |
| Ticket           | N/A (branch `ethercat-ipc-hardening` ← `curvature-profile-1`, merge `a3a446bfd`) |
| Date opened      | 2026-06-23 |
| Status           | Active |
| System           | Neptune 3 Pro bench (`ethercatpi5.local`); Y/Z/E steppers on F401 (mcu0), X = A6-EC EtherCAT servo; host klippy + Rust motion-engine |
| Evidence sources | Pi journald (panic + SIGABRT), VictoriaLogs (`KALICO_VL=http://ethercatpi5.local:9428`), `ec-rt-capture.log`, source (`pump.rs`, `enqueue.rs`), printer.cfg, cube gcode |

## Problem Statement

"Investigate the discontinuity." Bench prints of the merged branch crash; the prior investigation established the EtherCAT drive-fault / `ECONNRESET` / broken-pipe chain was **downstream noise** — the true trigger is a host-side `panic!` in the pump's junction position-continuity check. This case investigates **why the planner produces the discontinuity**. Fixture: `COLD_Voron_Design_Cube_v7` (and `short_` / `cold_run` variants).

## Evidence Inventory

| Source | Status | Notes |
| ------ | ------ | ----- |
| Panic + core-dump | Available | journald `17:42:12 panicked at pump.rs:352`; `klipper.service ... status=6/ABRT ... core-dump` |
| Panic message (exact values) | Available | `mcu0 axis1: prev ends 114.25038 (t=1450.043609), next starts 114.663 (t=1450.043609), |Δ|=0.41262054mm` |
| Panic history (3×, today) | Available | `16:03:20` line **388** (Δ0.1455), `17:23:50` line 352 (Δ0.1425), `17:42:12` line 352 (Δ0.4126) — all mcu0 axis1, all same-host-time |
| Cube gcode | Available | pulled `short_COLD_Voron_Design_Cube` (393 lines) → scratchpad `short_cold_cube.gcode`; full 57m43s + `cold_run.gcode` on Pi |
| printer.cfg | Available | input_shaper **disabled** (`:230`), PA `linear_pressure_advance` extruder-only, `max_v=100 accel=1000 scv=5` |
| Piece coeffs semantics | Available | `enqueue.rs:163-178` Bernstein control points; `[0]`=P0=start, `[3]`=P3=end |
| Offline deterministic repro | **Missing** | have the gcode; need it driven through motion-engine with the panic live (see Backlog #1) |
| Pre-merge reproduction | **Missing** | would settle ours-seam vs pre-existing; obtain by bisect/toggle (Backlog #2) |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Offline repro: drive `short_cold_cube.gcode` through `motion-engine` (dump_stream_trajectory example or klipper-sim) with the continuity panic active + `RUST_BACKTRACE=1` | High | Open | Makes it deterministic; yields the exact seam/segment context |
| 2 | Bisect ours' seam logic: toggle `brake_to_rest_setback` / `trim_front_to_seam` / `commit_stall_brake` and re-run repro | High | Open | Discriminates H-A vs H-B |
| 3 | Map the Y≈114.25→114.66 move pair in the gcode to a feature/layer transition | Medium | Open | Characterizes the trigger geometry (travel? infill seam?) |
| 4 | Inspect `extract_bezier_pieces` / reparam boundary continuity across commits | Medium | Open | H-B path |

## Timeline of Events

| Time (2026-06-23) | Event | Source | Confidence |
| ----------------- | ----- | ------ | ---------- |
| 16:03:20 | panic pump.rs:**388** mcu0 axis1 Δ0.1455 | journald | Confirmed |
| 17:23:50 | panic pump.rs:352 mcu0 axis1 Δ0.1425 | journald | Confirmed |
| 17:42:10.857 | host still emitting normal pipeline events | VL | Confirmed |
| 17:42:12 | panic pump.rs:352 Δ0.4126 → SIGABRT core-dump → socket RST → ec-rt ECONNRESET | journald + ec-rt-capture | Confirmed |

## Confirmed Findings

### Finding 1: The check is sound — coeffs are Bézier control points

**Evidence:** `enqueue.rs:108-112` `is_constant_piece` tests all four coeffs equal (a power-basis constant would be `[p,0,0,0]`); `enqueue.rs:163,173-178` builds `coeffs` from `to_bernstein()` control points. So `coeffs[0]=P0=start`, `coeffs[3]=P3=end`.

**Detail:** The check compares `prev last_entry.coeffs[3]` (P3, end) vs `next first_entry.coeffs[0]` (P0, start) — `pump.rs:475-481`. Apples-to-apples; both render as ~114 mm absolute positions. **The 0.41 mm gap is a real trajectory discontinuity, not a check artifact.**

### Finding 2: The check guards absolute-coordinate pieces at a streaming batch boundary

**Evidence:** `pump.rs:468` gates on `first_entry.motor_mask == 0`; `enqueue.rs:216-221` makes coeffs relative (subtract P0) only when `motor_mask != 0`. `junction_ends` (`pump.rs:420,523-530`) stores the previous Enqueue batch's last piece; the check fires when the next batch's first piece does not continue it. `fresh_stream` clears it (`pump.rs:462-463`).

**Detail:** Discontinuity is between two **consecutive Enqueue batches** — i.e., at a streaming commit/seam boundary, not within a single lowered move.

### Finding 3: Merged shaper/PA post-processor is ruled out for Y

**Evidence:** `printer.cfg:230` "[input_shaper] disabled for MVP — no shaper = passthrough"; `shaper_type_y`/`_x` commented (`:232-235`). PA is `linear_pressure_advance` on the extruder only (`:91`), commented coefficient (`:173`).

**Detail:** Theirs' `apply_axis_chains` post-pass is passthrough on Y; PA never touches axis 1. The discontinuity cannot originate in the merged post-processor → it is in the **core streaming/lowering** path.

### Finding 4: Symptom is on the serial Y axis, not EtherCAT

**Evidence:** panic `mcu0 axis1`. mcu0 = F401 serial; axis1 = Y. The X servo (EtherCAT) is uninvolved; the prior case's drive-fault/`ECONNRESET`/broken-pipe chain was all downstream of the SIGABRT.

## Deduced Conclusions

### Deduction 1: The producing condition is in this branch's commit/seam pipeline

**Based on:** Findings 1–4.

**Reasoning:** The gap is real (F1), occurs at a batch/commit boundary (F2), is not from the merged post-processor (F3), and is on a plain stepper axis (F4). The boundary between consecutive committed batches is exactly what this branch's streaming/backpressure work owns (`commit_stall_brake`, `head_trim_feasible`, `trim_front_to_seam`, `brake_to_rest_setback`). The same fail-loud also fired on an earlier build (line 388 @ 16:03), so the discontinuity is not unique to the post-merge binary.

**Conclusion:** Prime suspect is ours' seam/setback re-anchoring the new batch's first piece at a position offset from the committed batch's last-piece endpoint. A pre-existing reparam/lowering seam (H-B) is the alternative.

## Hypothesized Paths

### Hypothesis A: Seam/setback re-anchors the new batch off the committed endpoint

**Status:** Open. **Theory:** `brake_to_rest_setback` / `trim_front_to_seam` chooses a seam position for the re-planned head that does not equal the last committed piece's P3, so the next batch's first P0 jumps. **Supporting:** discontinuity is batch-boundary-only (F2); same-host-time means timing is continuous but position is not (a re-anchor, not a re-time). **Would confirm:** offline repro shows the gap appears exactly at a `commit`/seam emission and disappears when setback/trim is bypassed. **Would refute:** gap reproduces with seam logic disabled.

### Hypothesis B: Pre-existing reparam/lowering seam discontinuity

**Status:** Open. **Theory:** `extract_bezier_pieces` / reparam produces non-C0 boundaries independent of the streaming seam. **Would confirm:** repro on a pre-streaming-refactor baseline. **Would refute:** gap only appears at streaming commit boundaries.

### Hypothesis C: Check compares wrong coefficients

**Status:** **Refuted** (Finding 1 — coeffs are Bézier control points; `[3]`=end, `[0]`=start).

### Hypothesis D: Merged input-shaper / pressure-advance post-processor

**Status:** **Refuted** (Finding 3 — shaper disabled, PA extruder-only).

## Source Code Trace

- **Error origin:** `rust/motion-engine/src/pump.rs:352` (`panic!` in `check_junction_position_continuity`).
- **Trigger:** next Enqueue batch's first piece `coeffs[0]` (P0) differs from the stored `JunctionEnd.end_pos` (prev batch's last piece `coeffs[3]`, P3) by ≥ `JUNCTION_POSITION_FATAL_MM = 0.1` (`pump.rs:341`).
- **Condition:** `first_entry.motor_mask == 0` and a `JunctionEnd` exists for the key (not `fresh_stream`).
- **Related:** `enqueue.rs:147-252` (Bézier piece flattening), `stream.rs` seam/commit + lowering, `trajectory/src/reparam.rs` / `post_processor.rs`.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Deterministic offline repro | Turns an intermittent bench crash into a debuggable unit | Drive `short_cold_cube.gcode` through motion-engine (Backlog #1) |
| Pre-merge / seam-disabled reproduction | Settles H-A vs H-B | Toggle seam logic + re-run (Backlog #2) |
| Trigger geometry (which move pair) | Characterizes the offending boundary | Map Y≈114.25→114.66 to the gcode (Backlog #3) |

## Final Conclusion

**Confidence: Medium-High** (root cause area Confirmed; exact seam mechanism Open). A real C0 position discontinuity (0.14–0.41 mm) on the serial Y axis at a streaming **batch/commit boundary** trips the fail-loud `check_junction_position_continuity`, aborting klippy; the check is correct and the merged shaper/PA post-processor is ruled out, localizing the cause to this branch's commit/seam pipeline. Next: deterministic offline repro from the cube gcode, then bisect the seam logic.

**Status:** Superseded by the 2026-06-23 follow-up — root cause is host→MCU **clock-projection**, not planner geometry.

## Follow-up: 2026-06-23 — root cause found (clock-projection, not geometry)

### Hand-off Brief (revised)

1. **What happened.** Consecutive trajectory pieces are each independently projected from host-time to MCU-ticks against the **live, continuously-rebased clock estimate**. When the estimate is revised between enqueuing two consecutive segments, the later segment's tick anchor shifts by the revision delta, opening a **±tens-to-hundreds-of-µs gap/overlap in MCU-tick space** at the segment boundary — while the trajectory is perfectly continuous in host-time. (Confirmed)
2. **Where it stands.** Root cause Confirmed by code + bench instrumentation. The planner geometry is exonerated (offline harness: every ShapedSegment continuous at every commit cadence; bench: `host_jump ≈ 0`). The tick gap/overlap stutters the steppers and faults the EtherCAT servo.
3. **Next.** Decide between the two fix directions below (smooth-rebasing vs tick-chaining); chaining is the principled fix. Needs bench validation.

### How it was found

Instrumented `pump.rs` to name both colliding gcode lines and suppress the panic (commit `0393aaf04` on `ethercat-ipc-hardening`), deployed to the bench, ran the cube. Bench logs (18:09 UTC):

| event | tick_jump | host_jump | lines | axes |
| ----- | --------- | --------- | ----- | ---- |
| overlap_risk | −93 µs | ≈0 | 7928→7928 | Y,Z,E |
| overlap_risk | −85 µs | ≈0 | 8001→8001 | Y,Z,E |
| projection_divergence | +218 µs | ≈0 | 7828→7828 | Y,Z,E |
| projection_divergence | +170 µs | ≈0 | 7710→7711 | Y,Z,E |

`host_jump ≈ 0` everywhere; `prev_line == next_line` on most (the gap is *within one move's own sub-pieces*); all three mcu0 axes shift identically (a whole-segment time shift). That run crashed on an EtherCAT drive fault (`code=0xfecc`), not the position panic — consistent with timing stutter → servo following-error.

### Confirmed root cause (Finding 5)

**Evidence chain (code):**
- `PieceEntry::end_time(freq)` = `start_time + (duration*freq)` is the **MCU ISR's own piece-advance test** (`runtime/src/motion_core.rs:130` `if now < entry.end_time(cycles_per_second)`) — not diagnostic-only.
- Each piece's `start_time` is set by `project()` = `host_time_to_mcu_clock` (`enqueue.rs:211`), the **live** estimate `set_clock_est_rebased` keeps revising.
- `mcu_clock_of`/`freq_of` return `router.ack_clock_and_freq` → `rec.clock_freq`, the **live** freq (`host-rt/src/passthrough_queue/router.rs:427`).
- The dispatch `Anchor` (`anchor.rs:30-66`) holds `t0` **stable** across a healthy stream → host-time continuity (matches `host_jump≈0`).

**Mechanism:** with a *stable* estimate, `tick_jump = project(host_{N+1}) − [project(host_N) + dur·freq] = 0`. The observed ±100 µs is exactly the **estimate-revision delta between the two enqueue times** `T_N` and `T_{N+1}`. Because the MCU plays each piece at its frozen `start_time` and advances at `start_time + dur·cycles_per_second`, that delta is a **real** gap/overlap on the MCU, not just a diagnostic artifact.

### Hypotheses updated

- **H-A (seam/setback re-anchor)** — **Refuted.** Offline harness: ShapedSegments continuous at every cap (worst=0.0).
- **H-C (check compares wrong coeffs)** — Refuted (Bézier control points).
- **H-D (shaper/PA post-processor)** — Refuted (passthrough on Y).
- **H-E (stall-brake re-dispatch)** — Refuted (no `force=true` before the crash).
- **H-F (clock-projection: per-segment re-projection against a revising estimate)** — **Confirmed** (Finding 5).

### Fix directions

- **(B) Tick-chaining (principled).** Within a continuous stream, derive each segment's first tick from the *previous* segment's last tick (one shared delta across all axes of the segment, preserving multi-axis sync), instead of re-projecting `t0` per segment. Only the stream's first piece / a `fresh_stream` re-anchor projects from host-time; the `Anchor`'s underrun path already handles gross drift. Makes MCU timing immune to clock-sync jitter by construction (this is how absolute step-time generation normally works).
- **(A) Smooth-rebasing.** If the ±100 µs is a *jump* introduced by `set_clock_est_rebased` rather than legitimate sync correction, fix the rebasing to keep `project()` continuous across revisions. Open question — `set_clock_est` is sampled 1/100 so the raw revision magnitude wasn't directly observed.

**Recommendation:** verify whether the per-revision projection shift is a rebasing jump or legitimate correction (one targeted log of `project(fixed_host)` across a rebase), then implement **(B)** — it's correct regardless of (A). Both need bench validation.

### Open / missing evidence

| Gap | Impact | How to obtain |
| --- | ------ | ------------- |
| Raw clock-estimate revision magnitude across a rebase | Distinguishes fix (A) vs (B) necessity | Un-sample `set_clock_est`, or log `project(fixed_host)` each rebase |
| Whether the `0xfecc` drive fault is downstream of the timing stutter | Confirms one root vs two | Correlate servo following-error with tick-overlap events |

**Status:** Root cause Confirmed (clock-projection). Fix direction (B) recommended, pending bench validation. Instrumentation lives on `ethercat-ipc-hardening` (`0393aaf04`).

## Follow-up: 2026-06-24 — the POSITION panic is a separate bug; clock-projection was the TICK manifestation

### Thread

After the backpressure gate fix (`ff4affea4`) the cube still crashes; restoring the junction panic (`d8258310c`) confirmed the crash *is* the position panic — it was only ever masked by the `0393aaf04` skip, not fixed. The 2026-06-23 "clock-projection" follow-up explained the **tick** anomalies (`tick_jump` µs, `host_jump≈0`) and TickChain fixed those; but `check_junction_position_continuity` compares **positions in mm** (`coeffs[3]` vs `coeffs[0]`), a quantity TickChain cannot touch. The two were conflated. This follow-up isolates the position bug.

### New Evidence — deterministic offline repro (overturns "H-A refuted")

`examples/repro_junction` on `crash_short_cube.gcode` (456 moves), sweeping the commit cap:

| cap | FATAL(≥0.1mm) | worst |
| --- | ------------- | ----- |
| 1, 2, 8, 16, 24 | **1** | **0.15450 mm** |
| 32, 48, 64, 256 | 0 | 0.0 |

The discontinuity is **commit-cadence-dependent** and reproduces at the **ShapedSegment** level (not just the lowered pieces): Y axis, `end=125.30250 → start=125.14800`, **same time** `t=5.208962`, `|Δ|=0.1545mm`. Small/frequent commits open the seam gap; one large batch (cap≥32) fits the whole feature continuously. The 2026-06-23 offline harness "refuted" H-A only because it tested large caps (64/167/340/512); at the real streaming cadence (small incremental commits) it reproduces.

### Confirmed Findings

- **Finding 6: the position discontinuity is the cadence-dependent seam re-fit (H-A confirmed).** Committing a prefix ending at the corner fixes its endpoint at one blended position; re-fitting the continuation (with the committed moves trimmed off the front) re-solves the biclothoid blend and places the leading position elsewhere → C0 break at the seam. Deterministic at cap≤24 (above).
- **Finding 7: the commit seam pins velocity and curvature but NOT position.** `StreamState` (`stream.rs:129-148`) carries `entry_v` (seam velocity) and `committed_head_len` (take-3: re-fits the leading corner to its pre-trim *curvature*, `windowed-fit-ceiling-jitter.md`). Neither pins the seam **position**: the continuation's start is re-derived from the trimmed move list, so a blend whose apex moved on re-fit yields a position jump. This is the position-completeness gap of the take-3 fix.

### Updated Hypotheses

- **H-A (seam/setback re-anchors the continuation off the committed endpoint): Confirmed** — deterministic repro, cadence-dependent, ShapedSegment-level.
- **H-F (clock-projection): re-scoped** — explains the TICK anomalies (fixed by TickChain); does NOT explain the mm position panic. The "Superseded" note above applied to the tick symptom only.

### Fix Direction

Pin **C0 position** at the committed seam, the way `entry_v`/`committed_head_len` pin velocity/curvature: the re-fit of the continuation must START at the exact committed endpoint position, not re-derive it from the trimmed move list. Mechanism lives in `fit_chain_with_head_restore` / the `committed_head_len` feedback (`stream.rs` commit path, `geometry` head-restore). Verify against the offline repro (must reach `worst=0.0` at **all** caps 1..256, not just ≥32).

### Reproduction (deterministic, ~20 s, no bench)

`cargo run --release -p motion-engine --example repro_junction -- crash_short_cube.gcode --cap 8` → `FATAL 1=Y |Δ|=0.15450mm`. Fixed when FATAL=0 at every cap.

**Status:** Position root cause Confirmed (cadence-dependent seam re-fit; seam position not pinned). Fix not yet implemented. Deterministic offline repro in hand.
