# Investigation: klippy aborts on every print — fatal junction-position discontinuity on the EtherCAT servo X axis

## Hand-off Brief

1. **What happened.** *(Confirmed)* A Rust panic in the motion engine's step-pump thread (`pump.rs:472`, the junction-continuity fail-loud guard) aborts the whole klippy process — because the planner emits a **0.1337 mm backward position jump** between two consecutive pump-message pieces on `{mcu_id:1, axis:0}` (the EtherCAT servo X axis); `panic = "abort"` turns that thread panic into a full-process abort, so moonraker loses klippy with no Python traceback.
2. **Where the case stands.** Proximate cause and mechanism are Confirmed and the symptom is fully explained; the discontinuity is bit-for-bit reproducible and is **X-servo-specific** (no other axis logs even a sub-fatal jump), which refutes both the `align_travels` and "generic arc-fit seam" theories and localizes the producing bug to the servo / multi-drive piece-streaming path (#138). The exact producing function is still Hypothesized.
3. **What's needed next.** Trace the EtherCAT-servo X piece-generation / pump-message split path for a position re-anchor or counts↔mm conversion that differs across batch boundaries, and reproduce it offline in `seam_test_harness.rs` with a servo-keyed axis — that pins the producer without bench time.

## Case Info

| Field            | Value                                                                                  |
| ---------------- | -------------------------------------------------------------------------------------- |
| Ticket           | N/A (branch `neptune-crash`)                                                            |
| Date opened      | 2026-06-28                                                                              |
| Status           | Active — proximate cause Confirmed, producer Hypothesized                               |
| System           | Neptune 3 Pro EtherCAT bench (`ethercatpi5.local`); X = A6-EC servo on `[ethercat_node node]`, Y/Z/E steppers on `[mcu]`; deployed commit `016fbea3d` (== local HEAD) |
| Evidence sources | host-process coredumps, gdb backtrace, VictoriaLogs structured events, printer config, source (`rust/motion-engine`, `rust/geometry`) |

## Problem Statement

User report: "When I try to print `crash_short_COLD_Voron_Design_Cube_short.gcode` it crashes either right away, or a little bit into the print. Same for any print really. It doesn't show any error, just moonraker can't connect to klipper."

Verified independently — the "no error" is because the failure is a **native abort below Python**, not a Python exception.

## Evidence Inventory

| Source                          | Status    | Notes                                                                                                  |
| ------------------------------- | --------- | ------------------------------------------------------------------------------------------------------ |
| Host-process coredumps          | Available | `~/printer_data/logs/coredumps/core.{push-pieces-pum,python}.*` — 4 cores 22:34–22:55, all the klippy process; `core_pattern` writes them here (not systemd) |
| gdb backtrace (latest core)     | Available | Rust panic chain → `pump.rs:472` `check_junction_position_continuity`, thread "push-pieces-pump", spawned by `bridge.rs:2935` `init_planner` |
| VictoriaLogs (`event=junction_position_discontinuity`) | Available | 2 fatal events, identical payload; VL `/health` = OK (results trustworthy) |
| Printer config                  | Available | `[mcu]`, `[ethercat_node node]`, `[arc_fit]`, `square_corner_velocity: 20`, `[include servos/servo_x.cfg]` |
| Triggering G-code               | Available | `~/printer_data/gcodes/crash_short_COLD_Voron_Design_Cube_short.gcode` lines ~95–115 — contiguous extruding perimeter near X≈106 |
| Source (motion-engine, geometry)| Available | `pump.rs`, `bridge.rs`, `geometry/src/fitter/causal.rs`, `seam_test_harness.rs` |
| Rust panic message text         | Missing   | Not in journalctl/VL — klippy stderr not captured to the structured pipeline (see Missing Evidence) |
| Servo-X piece-generation path   | Partial   | Located `ethercat_mcu_ids` handling in `bridge.rs` (2777, 2829–3032, 3741–3749); not yet traced end-to-end for the position seam |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Trace servo-X piece position across a pump-message boundary | High | Open | Find where `coeffs[3]` of batch N and `coeffs[0]` of batch N+1 are produced for the servo axis; look for a counts↔mm round-trip (131072 counts/rev) or odometer re-seed |
| 2 | Reproduce in `seam_test_harness.rs` with a servo-keyed axis | High | Open | Offline, no bench; confirms producer and becomes the regression test |
| 3 | Inspect `dispatch.rs` / `build_serial_seed_sends` for `ethercat_mcu_ids` | Medium | Open | Does multi-drive (#138) split/seed the servo stream differently than steppers? |
| 4 | Capture klippy stderr to disk/structured pipeline | Medium | Open | The panic message names the exact Δ and lines; currently lost on abort |
| 5 | Decode whether 0.1337127685546875 mm maps to an integer servo-count quantum | Low | Open | Exact-repeatable value smells like a fixed conversion offset, not float noise |

## Timeline of Events

| Time (local 2026-06-28) | Event | Source | Confidence |
| ----------------------- | ----- | ------ | ---------- |
| 22:34:00.469 | Fatal junction discontinuity logged, `{mcu1,axis0}`, jump 0.1337 mm, line 106; klippy aborts (`core.push-pieces-pum.13635`) | VL + coredump | Confirmed |
| 22:39 | A `core.python.*` core (restart-path / second abort of same process class) | coredump listing | Confirmed |
| 22:53:40.330 | Fatal junction discontinuity again, **identical** payload, new session; abort (`core.push-pieces-pum.14395`) | VL + coredump | Confirmed |
| 22:55 | Further abort core (`core.push-pieces-pum.14597`) | coredump listing | Confirmed |

## Confirmed Findings

### Finding 1: klippy dies by Rust panic → process abort, not a Python error

**Evidence:** gdb backtrace of `core.push-pieces-pum.14597.1782680118` (binary `/home/dderg/klippy-env/bin/python … klippy.py`):
frames `panic_abort::__rust_start_panic` → `core::panicking::panic_fmt` → `_motion_engine::pump::check_junction_position_continuity (motion-engine/src/pump.rs:472)` → `run_pump` (`pump.rs:648`) → `bridge::…::init_planner::{closure#23}` (`bridge.rs:2935`) → `thread_start`.

**Detail:** The motion engine is a native library loaded into klippy's Python process. The panic occurs on the spawned "push-pieces-pump" thread. With `panic = "abort"`, a thread panic aborts the entire process — so moonraker's RPC connection drops and there is no Python traceback. This exactly matches "no error, moonraker can't connect."

### Finding 2: The panic is the fail-loud junction-continuity guard, firing on a real 0.1337 mm jump

**Evidence:** `rust/motion-engine/src/pump.rs:471-472` panics when `jump >= JUNCTION_POSITION_FATAL_MM` (`= 0.1`, `pump.rs:363`). VL `event=junction_position_discontinuity` (`pump.rs:455-468`, fires at `>= JUNCTION_POSITION_LOG_MM = 0.0125`, `pump.rs:362`):
`fatal=true, jump_mm=0.1337127685546875, prev_end=106.6069793701172 (line 105), next_start=106.4732666015625 (line 106), key=AxisKey{mcu_id:1, axis:0}`, prev/next host times equal to ~1.8e-7 s.

**Detail:** `jump = |next_start_pos − prev_end_pos|` where `prev_end_pos = prev piece coeffs[3]` (Bézier end control point) and `next_start_pos = next batch's first piece coeffs[0]` (start control point) — `pump.rs:380-381, 413-414`. The two pump-message batches disagree about the X position at their shared seam by 0.1337 mm **backward**. f32 epsilon at ~106 mm is ~1e-5 mm, so this is a genuine macroscopic discontinuity, not rounding. The guard exists precisely to stop a one-sample step burst (`fault -300/-310`) reaching the MCU — it is behaving correctly.

### Finding 3: The discontinuity is exclusive to the EtherCAT servo X axis

**Evidence:** VL `event=junction_position_discontinuity _time:6h | stats by (key, fatal)` returns a **single** key, `{mcu_id:1, axis:0}` (2 events). No other axis appears even at the 0.0125 mm log threshold. Config: `printer.cfg` has `[mcu]` (main, mcu_id 0) and `servos/ethercat.cfg` `[ethercat_node node]` (mcu_id 1); `[include servos/servo_x.cfg]`. So `{mcu1, axis0}` = the X servo.

**Detail:** All axes share the same `[arc_fit]` and `square_corner_velocity: 20`. If the seam were a generic fitter/arc-fit artifact, Y (a stepper executing the same curved corner, X≈106/Y≈114) would log sub-fatal jumps. It logs none. The defect is specific to the servo / multi-drive code path.

### Finding 4: Reproducible and deterministic

**Evidence:** Two independent sessions (`k-1782678772-13635`, `k-1782679244-14395`) produced **bit-identical** `jump_mm=0.1337127685546875` at the same `next_source_line=106`. Deployed commit `016fbea3d` equals local HEAD.

**Detail:** Identical to the last bit ⇒ a deterministic computation in the planner output, not a timing/concurrency race. The same slicer geometry will fail every run, consistent with "same for any print" (any file with a comparable servo-axis perimeter seam).

## Deduced Conclusions

### Deduction 1: The producing bug is in the servo / multi-drive piece path, not the generic fitter

**Based on:** Findings 2, 3.

**Reasoning:** The discontinuity is in fitter/planner *output* (mm-domain piece coeffs), yet appears on **only** the servo axis while the steppers — same fitter, same config, same corner — stay below even the 0.0125 mm log threshold. An axis-agnostic stage (arc-fit, `align_travels`) cannot produce an X-only seam.

**Conclusion:** Localize to where the X-servo trajectory is generated, keyed, split into pump messages, or position-anchored differently from steppers — i.e. the EtherCAT/multi-drive path merged in #138.

### Deduction 2: The existing ingress guard cannot catch this class

**Based on:** commit `66f3da172` ("reject discontinuous moves at stream ingress").

**Reasoning:** That guard validates contiguity at `StreamState::push` — *input G-code-move* endpoints vs the previous buffered move / odometer. The G-code here is contiguous (lines 95–115 each start where the prior ended). The 0.1337 mm gap appears *downstream* in the fitter's *piece output* at a pump-message seam, which `StreamState::push` never inspects.

**Conclusion:** The pump-side `check_junction_position_continuity` is the only guard positioned to catch it — and it does, by aborting. Fixing the producer is required; the guard is not the bug.

## Hypothesized Paths

### Hypothesis 1: `align_travels` snapping is the direct cause — REFUTED

**Status:** Refuted.

**Theory:** (From the input narrative.) `align_travels` (causal-fitter #126) snaps move endpoints and introduced the seam.

**Resolution:** `rust/geometry/src/fitter/causal.rs:157-159` — `align_travels` skips any move where `!is_travel(...)`. G-code lines 105–106 are **extruding** moves (`E.04053`, `E.02798`), which it never rewrites. It cannot be the direct rewriter of these pieces. (It may still perturb a *neighbouring* travel move; not in evidence here and not axis-selective, so not the primary cause.)

### Hypothesis 2: Generic arc-fit seam deviation — REFUTED as primary

**Status:** Refuted (as primary).

**Theory:** `[arc_fit]` replaces a run of short perimeter segments with an arc deviating up to tolerance; a pump-message boundary mid-arc leaves the two batches disagreeing by ~0.1 mm.

**Resolution:** Plausible in magnitude, but arc-fit is axis-agnostic. Finding 3 shows the seam is X-servo-only with Y completely clean — refutes arc-fit as the producer. (Arc-fit may still be the *trigger geometry* that the servo-specific bug then mishandles.)

### Hypothesis 3: Servo-axis position re-anchor / counts↔mm round-trip at pump-message boundaries

**Status:** Open — leading candidate.

**Theory:** The X-servo piece path converts position through servo counts (131072 counts/rev) or re-seeds an odometer/anchor at each pump-message batch. Batch N's last `coeffs[3]` and batch N+1's first `coeffs[0]` are computed via slightly different anchors, leaving a fixed ~0.1337 mm offset unique to the servo axis.

**Supporting indicators:** X-only (Finding 3); exact-repeatable value (Finding 4) smells like a deterministic conversion/quantization, not float noise; the path is new (#138).

**Would confirm:** A point in the servo piece-generation/streaming code where the seam position is derived from a count-rounded or re-anchored value that differs across batch boundaries; offline repro in `seam_test_harness.rs` with a servo-keyed axis reproducing 0.1337 mm.

**Would refute:** The servo path computes seam positions identically across batches (then look at dispatch/splitting — Hypothesis 4).

### Hypothesis 4: Multi-drive dispatch (#138) mis-splits/seeds the servo stream

**Status:** Open.

**Theory:** `build_serial_seed_sends` / dispatch handles `ethercat_mcu_ids` such that the servo axis's stream is split or seeded so two batches gap/overlap by 0.1337 mm.

**Supporting indicators:** `bridge.rs:2829-3032, 3741-3749` special-case `ethercat_mcu_ids`; the regression coincides with the #138 multi-drive merge.

**Would confirm:** A split/seed boundary in the dispatch path that drops or double-counts a sub-segment of the servo trajectory.

**Would refute:** Dispatch streams the servo axis byte-identically to steppers.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Rust panic message text (it embeds the exact Δ, ticks, host times, lines) | Redundant with VL here, but would corroborate independently | Capture klippy stderr to `~/printer_data/logs/` (Backlog #4); or `gdb … -ex 'frame 11' -ex 'info args'` (seam is `<optimized out>` in release — may not recover) |
| Servo-X piece values straddling the seam (the actual two pieces) | Pins which side is wrong (does N overshoot, or N+1 under-start?) | Add a debug dump of the two boundary pieces, or reproduce in `seam_test_harness.rs` |
| Whether 0.1337127685546875 mm is an integer servo-count multiple | Distinguishes Hypothesis 3 (quantization) from a logic gap | Divide by mm-per-count for the X servo (lead/pitch × 1/131072) |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Error origin | `rust/motion-engine/src/pump.rs:471-472` — `check_junction_position_continuity` `panic!` when `jump >= JUNCTION_POSITION_FATAL_MM` |
| Trigger | `rust/motion-engine/src/pump.rs:648` — `run_pump` calls the guard after `JunctionTracker::observe` (`pump.rs:400-436`) returns a seam for an enqueued batch |
| Condition | Two consecutive pump-message batches for `{mcu1, axis0}` whose boundary positions (`prev coeffs[3]` vs `next coeffs[0]`) differ by 0.1337 mm |
| Producing area (to trace) | EtherCAT-servo X piece generation / pump-message split / anchoring — `bridge.rs` `init_planner` (around `2935`) and the `ethercat_mcu_ids` path (`bridge.rs:2777, 2829-3032, 3741-3749`); `dispatch.rs` |
| Refuted areas | `geometry/src/fitter/causal.rs:154` `align_travels` (travel-only); generic `[arc_fit]` (axis-agnostic) |
| Guard rationale | Prevents a one-sample step burst → MCU `fault -300/-310`; correct behaviour, not the defect |

## Conclusion

**Confidence:** High on the proximate cause and mechanism; Medium on localization of the producer; the exact producing function is Hypothesized (Low) pending one trace.

**Confirmed:** Every print aborts because the motion-engine step-pump thread panics in its junction-continuity guard (`pump.rs:472`) on a deterministic **0.1337 mm backward position discontinuity** at a pump-message seam, exclusively on the **EtherCAT servo X axis** (`{mcu_id:1, axis:0}`). `panic = "abort"` propagates the thread panic to a full klippy-process abort, which is why moonraker drops the connection with no Python error. The guard is functioning as designed (fail-loud, blocking an MCU step burst); the true defect is that the **servo-X trajectory is genuinely discontinuous across the seam**.

**Refuted from the initial narrative:** `align_travels` is not the direct rewriter (it skips the extruding moves involved), and a generic arc-fit seam is ruled out by the X-only signature. The producer lives in the servo / multi-drive (#138) piece path — most likely a position re-anchor or counts↔mm round-trip that differs across pump-message batches (Hypothesis 3), or a dispatch split/seed (Hypothesis 4).

## Recommended Next Steps

### Fix direction

Two layers, distinct mechanisms:
- **Producer (the actual bug):** make the servo-X piece path position-continuous across pump-message boundaries — the end of batch N's last piece must equal the start of batch N+1's first piece to within the log threshold. Pin it via Hypothesis 3 (anchor/quantization) or 4 (dispatch split) before changing code.
- **Guard (already correct):** keep the fail-loud abort. Optionally, the only safe *softening* is diagnostic richness (dump both boundary pieces on fire), never silent snapping — snapping the seam would just relocate the step burst onto the MCU.

### Diagnostic

1. Reproduce offline in `rust/motion-engine/src/seam_test_harness.rs` with a servo-keyed axis and this perimeter geometry — fastest disambiguation, no bench.
2. Add a one-shot debug dump of the two boundary pieces (coeffs, start_time/ticks, source_line) at the guard site to see which side is displaced.
3. Check whether 0.1337127685546875 mm is an integer multiple of the X servo mm-per-count (Backlog #5) — confirms/denies a quantization root.

### Reproduction Plan

- **Setup:** Neptune EtherCAT bench at `016fbea3d`; `crash_short_COLD_Voron_Design_Cube_short.gcode` present.
- **Trigger:** Start the print. (No motion-command permission needed to *predict* — it aborts deterministically at the servo-X seam near G-code line 106.)
- **Expected:** klippy process aborts (new `core.push-pieces-pum.*`); VL logs `event=junction_position_discontinuity fatal=true jump_mm=0.1337… key={mcu1,axis0}`; moonraker loses klippy.
- **Offline:** drive the same move chain through the planner in `seam_test_harness` and assert the servo-axis seam < `JUNCTION_POSITION_LOG_MM`.

## Side Findings

- *(Confirmed)* Host-process coredumps accumulate **unbounded** in `~/printer_data/logs/coredumps/` (~90 MB each; ~350 MB from 4 crashes here). A crash storm will fill the SD card and shows up only as a gap in the structured stream — worth a rotation/cap. (`core_pattern = …/coredumps/core.%e.%p.%t`.)
- *(Confirmed)* The high-volume `analog_in_state` `warn` lines in VL during the print are unrelated verbose ADC logging — noise for this case, but they dominate `level:warn` queries.
- *(Observation)* `core.push-pieces-pum` is the truncated thread name "push-pieces-pump" (kernel caps `%e` at 15 chars); all such cores are the klippy process, not a separate binary.

## Follow-up: 2026-06-28 — sim reproduction + root-cause bisection

### New Evidence (mcu-sim, full-firmware Docker sim, branch == bench commit)

The EtherCAT/servo path cannot be emulated, so it was replaced with plain
steppers and the topology varied to isolate the cause. G-code = the bench file
with E stripped (X/Y trajectory unchanged) and a `SET_KINEMATIC_POSITION`
prepended (the sim's G28 homing instrumentation is broken on this branch —
`HomingMove` was refactored into `load_cell_probe.py`; unrelated to the bug).

| # | Config | Result |
| - | ------ | ------ |
| A | X+Y steppers on F4 (`mcu1`), Z on H7 (`mcu0`), `[arc_fit]` ON | **CRASH** — `thread 'push-pieces-pump' panicked at pump.rs:472: junction position discontinuity on mcu1 axis0: prev 106.60698 → next 106.47327, |Δ|=0.13371277mm` |
| B | All steppers on one MCU (`mcu0`), `[arc_fit]` ON | **CRASH** — identical, `mcu0 axis0`, same `106.60698 → 106.47327`, `|Δ|=0.13371277mm` |
| C | All steppers on one MCU (`mcu0`), `[arc_fit]` OFF | **PASS** — all 309 moves execute, no panic |

The panic message, axis index, both positions, and the jump are **bit-for-bit
identical to the bench** (only the G-code line number shifts, 96/97 vs 105/106,
because E-only lines were stripped). Plain steppers, no servo, no EtherCAT, no
131072-count encoder, single MCU — and it still reproduces exactly.

### Updated Hypotheses

- **Hypothesis 3 (servo counts↔mm quantization): REFUTED.** Run A/B use plain
  steppers (`rotation_distance: 40`, `microsteps: 16`) and reproduce the exact
  0.1337 mm jump. The servo and its encoder are irrelevant.
- **Hypothesis 4 (multi-drive / second-MCU dispatch): REFUTED.** Run B puts every
  axis on a single MCU and still crashes on `mcu0 axis0`. The multi-MCU topology
  (#138) is irrelevant.
- **Hypothesis 2 (arc-fit seam): CONFIRMED — this is the root cause.** Toggling
  `[arc_fit]` is the single variable that flips crash (B) ↔ clean (C). My earlier
  refutation of H2 ("axis-agnostic ⇒ would hit Y too") was wrong: the seam is
  **data-dependent**, not axis-symmetric. `[arc_fit]` replaces a run of short
  perimeter segments with an arc whose piece endpoint, at a pump-message boundary,
  deviates from the next piece's start. Only an axis whose arc-fitted trajectory
  crosses a boundary with >0.1 mm deviation trips the fatal guard — here that is X
  near 106.5 mm. Y stayed under the 0.0125 mm log threshold on the bench, so it
  was silent; it was never immune.

### Updated Conclusion

**Confidence: High (deterministic, isolated by single-variable bisection).**

Root cause = **`[arc_fit]`** (the causal-fitter arc replacement, #126). When arc-fit
collapses a run of perimeter line segments into an arc, the resulting motion pieces
are not position-continuous across a pump-message (stream-batch) boundary; on the X
axis at X≈106.5 mm the seam jumps 0.1337 mm backward, and the pump's fail-loud
junction-continuity guard (`pump.rs:472`) panics → `panic=abort` → klippy process
aborts → moonraker drops. The EtherCAT servo and the multi-drive merge were both
red herrings — the crash reproduces with a single MCU and plain steppers, and
vanishes the instant `[arc_fit]` is removed.

**Immediate user workaround (not a fix):** removing `[arc_fit]` from `printer.cfg`
makes prints complete. The proper fix is to make arc-fit's piece output
position-continuous across stream-batch boundaries (or to snap the next batch's
start to the previous batch's committed end within the fitter, never at the pump).

### Next trace target (fix investigation)

`rust/geometry/src/fitter/causal.rs` — `detect_runs` / `chain_runs` (arc fitting)
and where fitted-arc pieces are emitted and split into pump messages. Confirm
whether the deviation is (a) the arc chord vs. the original vertices, or (b) a
re-fit at the batch boundary producing a different arc start. The offline
`rust/motion-engine/src/seam_test_harness.rs` can now reproduce this with this
move chain and no Docker.

## Follow-up: 2026-06-28 #2 — root cause localized to fitter (clothoid run-boundary C0 gap)

### Decisive offline reproduction (seam_test_harness)

`run_schedule(crash_gcode, bench_limits + arc_fit(min_run=3), cadence)` — no Docker,
no MCU. Bench limits = `VelocityLimits::try_new(500, 8000, 20)`; `tol = scv²(√2−1)/accel
= 0.0207 mm`.

**Cadence sweep** (commit cap = moves buffered before a commit):

| cap | commits | boundaries | fatal | worst |
| --- | ------- | ---------- | ----- | ----- |
| 1–16 | 85→46 | 2 | 0 | 0.0865 |
| 64 | 6 | 4 | **2** | **0.1778** |
| ∞ (single commit) | 1 | 4 | **2** | **0.1778** |

Single-commit (cap=∞) still produces the fatal seam ⇒ **the discontinuity is intrinsic
to the fitter output, not a streaming/batching artifact**. Magnitude scales with how long
an arc run the fitter chains (short runs 0.0865 mm → whole-perimeter run 0.178 mm); the
bench's runtime cadence landed at 0.1337 mm in between.

**Exact bench seam reproduced bit-for-bit** (single commit, all boundaries):

```
axis0 delta=0.133713 FATAL prev=106.60698(move 97) next=106.47327(move 98)   <-- == bench/sim
axis1 delta=0.177773 FATAL prev=108.28202(move 97) next=108.45979(move 98)
axis0 delta=0.086510 log   prev=125.28500(move 305) next=125.37151(move 306)
axis1 delta=0.026154 log   prev=102.70900(move 305) next=102.68285(move 306)
```

The X figures (`0.133713`, `106.60698 → 106.47327`) match the bench coredump and the sim
panic exactly. The junction is a single move boundary (97↔98) where the trajectory breaks
on **both** axes (~0.22 mm 2D jump); the pump aborts on whichever axis it checks first
(axis 0 = X on the bench).

### Fitter-level confirmation (instrumented `causal::fit()`, reverted after)

A temporary `FIT_C0_DEBUG` probe over `fit()`'s emitted segment list:

```
[FIT_C0] gap=0.222446mm between Clothoid(line 97) end=[106.60698,108.28202]
                            and Clothoid(line 98) start=[106.47326,108.45979]
[FIT_C0] gap=0.090382mm between Clothoid(line 305) end=[125.28500,102.70900]
                            and Clothoid(line 306) start=[125.37151,102.68284]
```

The gap is **between two clothoid segments** — the *down*-easing clothoid of one arc-fit
run and the *up*-easing clothoid of the next adjacent run. Their endpoints do not coincide.

### Confirmed root cause

**The arc-fit "causal" reconstruction emits eased runs (arc + entry/exit clothoids) whose
clothoid endpoints are not made continuous at a run-to-run boundary.** Where two arc-fit
reconstructions abut (here moves 97↔98 and 305↔306), the first run's exit clothoid ends at
one point and the next run's entry clothoid starts at a different point, leaving a C0
position gap (0.22 mm at 97/98). The pump's per-axis fail-loud guard (`pump.rs:472`) sees
X exceed `JUNCTION_POSITION_FATAL_MM` (0.1) and aborts → klippy dies.

### Why the prior fix did not catch it (answers the user's recollection)

Commit `cc705381b` ("Anchor reconstructed arcs to boundary vertices") fixed a ~10 µm C0 gap
for the **bare-arc → line/travel** case by pinning the *arc* endpoints through the boundary
vertices (with an in-band guard, else keep the LS centre), and regenerated the snapshot
baselines to "C0 to 0.0 mm." That fix addresses *bare arc endpoints*; it does **not** make
the **clothoid easing endpoints of two adjacent eased reconstructions** meet. The snapshot
cases (`neptune_cube layer_5/6`, fillet, circle, straight_to_arc) either don't exercise two
abutting eased runs at this scale or weren't run at the bench's tolerance regime (scv 20 /
accel 8000 → tol 0.0207). So the suite stayed green while this geometry breaks — exactly the
"fixed in snapshots, maybe not properly" intuition. The fix was incomplete, not wrong.

### Updated conclusion

**Confidence: High.** Root cause = **missing C0 continuity between adjacent arc-fit eased
reconstructions** (clothoid exit/entry endpoints not reconciled) in
`rust/geometry/src/fitter/causal.rs`. Reproduced bit-for-bit offline; isolated to `fit()`
output by direct instrumentation; orthogonal to MCU count, motor type, and the servo.

### Fix direction

In `causal.rs`, at a run-to-run boundary the exit clothoid of run *k* and the entry clothoid
of run *k+1* must share an endpoint. Options, in order of preference:
1. Reconcile the two clothoid endpoints to the shared boundary vertex (extend
   `cc705381b`'s vertex-anchoring from the bare arc to the eased clothoid ends), with the
   same in-band guard — and when the guard fails, **refuse the reconstruction** rather than
   emit a discontinuous one (fall back to the raw line legs, which print fine).
2. `resolve_run_boundaries` / `overlap` should blend abutting eased runs into a single C0
   path instead of leaving two free clothoid ends.
The guard at `pump.rs:472` is correct and must stay; never snap the seam at the pump (that
relocates the step burst onto the MCU).

### Regression test (recommended, not yet added)

`seam_test_harness::run_schedule` with this perimeter + bench limits + `arc_fit(3)` asserting
`SeamReport::fatal() == 0` across a cadence sweep (cap ∈ {1,4,16,64,∞}) — it reproduces in
~0.3 s with no Docker and would have caught this. The crash G-code is preserved at
`scratchpad/crash.gcode` (and on the bench at `~/printer_data/gcodes/`).
