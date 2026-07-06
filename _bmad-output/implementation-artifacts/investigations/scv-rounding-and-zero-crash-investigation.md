# Investigation: SCV has no visible effect on corner rounding; SCV=0 crashes

## Hand-off Brief

1. **What happened.** On the Neptune bench, varying `square_corner_velocity` (SCV) produced *identical* corner rounding on a square of G1 moves, and setting SCV=0 "crashed."
2. **Where the case stands.** Root cause of the "no change" symptom is **Confirmed (High)** and is NOT the junction-deviation/biclothoid path I first traced: the **chain arc-fitter** collapses the square into its r=25 mm inscribed circle (each corner cut ~10.4 mm) because all four sides are tangent to one circle within `cocircular_tol`=5µm. That radius is pure geometry — SCV doesn't enter it; SCV only nudges the clothoid easing length, a sub-0.3 mm second-order effect. SCV=0 does **not** crash the core fit+velocity pipeline (verified locally) — it just disables the arc fit and leaves a sharp square. The bench "crash" is not in retained logs; the bench was mid-[limit]/[printer] cutover and the running klippy *rejected* `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY` as unsupported.
3. **What's needed next.** Decide whether the aggressive square→inscribed-circle arc fit is intended; if not, this is the real bug to fix (chain-fitter acceptance criteria), independent of SCV.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-19                                                                 |
| Status           | Active                                                                     |
| System           | curvature-profile branch; Neptune 3 Pro bench; Rust motion engine          |
| Evidence sources | Source code (rust/geometry, rust/motion-engine); user observation          |

## Problem Statement

User-reported: "I tried different SCV settings on neptune and all of them worked exactly the same for a square of g1 commands, rounding was exactly the same, and only when I set it to 0 it just crashed."

Two distinct claims: (1) SCV has no effect on observed corner rounding for a square; (2) SCV=0 crashes.

## Evidence Inventory

| Source                          | Status    | Notes                                                                      |
| ------------------------------- | --------- | -------------------------------------------------------------------------- |
| SCV → fitter code path          | Available | `junction_deviation` and biclothoid solver fully traced                    |
| SCV plumbing (runtime + config) | Available | `submit_move` reads runtime SCV per-move; viz passes SCV through           |
| Velocity planner sharp-corner   | Available | Unblended junction → forced stop; warm-start can raise OverCommitted       |
| Bench crash log for SCV=0       | Missing   | Exact error text / fault not captured — needed to confirm crash mechanism  |
| Bench test method + accel       | Missing   | accel value, SCV-change method, rounding-observation method all unknown    |

## Confirmed Findings

### Finding 0 (ROOT CAUSE): the chain arc-fitter collapses the square into its inscribed circle; rounding radius is SCV-independent

**Evidence:** Local run of `_motion_engine.pipeline_snapshot` on a closed 50 mm square (`klippy/_motion_engine.so`, accel=3000):

| SCV | result | blended | unblended | chain_fits | traversal | closest approach to vertex (50,0) |
|-----|--------|---------|-----------|------------|-----------|-----------------------------------|
| 0   | OK     | 0       | 3         | 0          | 2.504 s   | 0 µm (sharp)                      |
| 5   | OK     | 0       | 0         | 1          | 1.804 s   | 10 360 µm                         |
| 40  | OK     | 0       | 0         | 1          | 1.794 s   | 10 580 µm                         |

The fitted path rides at radius ≈25 mm about (25,25) — the **inscribed circle** of the square (passing 73 µm from the side-midpoints). 35.36−25 = 10.36 mm = the measured corner cut. `chain.rs:130-138`: `incircle` fits one tangent circle to the corner facets; for a square the residual is ~0 < `cocircular_tol` = 5e-3 mm (`fitter.rs:31`), so the arc fit is accepted and the square's corners are replaced by the inscribed-circle arc.

**Detail:** `blended=0` for all SCV>0 — the per-junction biclothoid path (Finding 1) never runs for this geometry; the chain fitter consumes the corners first. The arc radius `rho` comes from `incircle`, pure geometry. SCV affects only `l_t = sqrt(24·rho·delta)…` (`chain.rs:155`), the clothoid easing length (≈1.7 mm at SCV=5 vs ≈13.7 mm at SCV=40) — which moves the closest approach by 0.22 mm (10.36→10.58), i.e. *visually identical*. **This is why every SCV produced the same rounding.**

### Finding 0b: SCV=0 disables the arc fit and does NOT crash the core pipeline

**Evidence:** SCV=0 row above (OK, chain_fits=0, unblended=3). `chain.rs:144-148`: `delta = min junction_deviation`; SCV=0 → delta=0 → `delta>0` guard returns `None`, so no arc fit; corners fall through to the per-junction classifier, which marks them `Unblended(ZeroDeviation)` (`fitter.rs:302`) → forced stops (`velocity.rs:193`). No panic in fit or from-rest velocity planning.

### Finding 1: SCV drives a junction-*deviation* fillet, not a velocity-only corner like classic Klipper

**Evidence:** `rust/geometry/src/fitter.rs:399-402`

```
fn junction_deviation(limits) -> f64 {
    let scv = limits.square_corner_velocity_mm_s;
    scv * scv * (SQRT_2 - 1.0) / limits.accel_mm_s2
}
```

**Detail:** `delta = min(jd_in, jd_out)` (`fitter.rs:301`) is fed as the deviation budget into `biclothoid::solve`. This is the classic Klipper junction-deviation formula `scv²(√2−1)/accel`, but here it is repurposed to size an actual geometric biclothoid blend, not just a cornering speed.

### Finding 2: blend size scales linearly with `delta` (∝ SCV²) and is far below `budget` for a normal square

**Evidence:** `rust/geometry/src/fitter/biclothoid.rs:32-33`

```
let trim_at_delta = trim_ref * delta / deviation_ref;
let trim = trim_at_delta.min(budget);   // budget = 0.5*min(side lengths)
```

**Detail:** For a square with multi-cm sides, `budget` is centimetres while `trim_at_delta` is governed by `delta`. So `trim` is delta-limited, i.e. it *does* respond to SCV — but in absolute terms it is tiny (see Deduction 1).

### Finding 3: SCV=0 is accepted by every guard and routes corners to a hard stop, not a panic

**Evidence:** `frontend.rs:38-42`, `config.rs:450`, `bridge.rs:3727-3738` all gate `scv >= 0.0` (0 allowed). `fitter.rs:302-303` returns `Unblended(ZeroDeviation)` when `delta<=0`. `velocity.rs:193-194` turns an unblended junction into `report.stops += 1` with junction velocity pinned to 0. Test `zero_square_corner_velocity_is_left_sharp` (`fitter/tests.rs:195`) confirms SCV=0 → sharp corner.

**Detail:** Nothing in the fitter or the from-rest velocity planner panics on SCV=0. The crash therefore originates downstream, most likely in the streaming warm-start path.

## Deduced Conclusions

### Deduction 1: ~~sub-50µm fillet explains "same rounding"~~ — REFUTED by Finding 0

**Status:** Refuted. I originally reasoned that SCV feeds the biclothoid `junction_deviation` fillet (`fitter.rs:399`), which is micron-scale at accel=3000, so corners look identically sharp. The local run (Finding 0) refuted this: `blended=0` — the biclothoid path never runs for the square, and the actual rounding is ~10 mm, not microns. The biclothoid formula is real code but it is not the path that governs this geometry. The chain arc-fitter (Finding 0) is. Lesson recorded per "hypotheses are never deleted."

## Hypothesized Paths

### Hypothesis 1: SCV=0 crash is a loud streaming over-commit, not a numeric fault

**Status:** Open — narrowed. Finding 0b refutes any crash in the *core* fit + from-rest velocity pipeline (SCV=0 runs clean). Retained `klippy.log` / VL contain **no** planner panic, `Diverged`, or `OverCommitted` — the only retained crashes are config-cutover errors (ethercat_node, `[printer] kinematics is not supported`, `axis 0: no limit set declares a finite max_velocity`). So the SCV=0 "crash" is either (a) the streaming warm-start over-commit (untested — viz uses the from-rest planner, not `plan_velocity_warm_start`), or (b) a config/restart failure during the limits cutover, or (c) the user's perception of the `SET_VELOCITY_LIMIT … is not supported` rejection (Hypothesis 2). Not separable from retained evidence; the crash event has rolled out of the logs.

**Theory:** With SCV=0 every corner is a forced stop (Finding 3). In the streaming planner, `commit()` calls `plan_velocity_warm_start(..., self.entry_v)` (`stream.rs:169`), where `entry_v` is the velocity already dispatched to the MCU. If SCV is dropped to 0 mid-stream (or corners are short), the committed entry velocity may be unable to brake to the now-mandatory stop within the look-ahead window → `VelocityError::OverCommitted` → `StreamError::Velocity` propagates to the host as a raised error ("crash"). Per project policy this is fail-loud-by-design, not a bug — but the *trigger* (SCV=0 making every corner an un-absorbable stop) may be the real complaint.

**Supporting indicators:** `stream.rs:31,52` wrap `VelocityError`; warm-start doc comment (`velocity.rs:90-97`) explicitly raises `OverCommitted` rather than clamping.

**Would confirm:** Bench log showing `OverCommitted` / `Diverged` / `StreamError::Velocity` at the moment SCV was set to 0.

**Would refute:** A different error (e.g. a Python-side exception, a panic/abort with a backtrace, an MCU fault) — which would point at a different mechanism.

### Hypothesis 2: the bench's `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY` was *rejected*, so SCV never changed at all

**Status:** Confirmed (as a contributing factor).

**Evidence:** VL + `klippy.log` at 2026-06-19T13:46:49/51 show `SQUARE_CORNER_VELOCITY is not supported: declare limits in [limit] config sections`. `git show 04c823c5c~1:klippy/motion.py` confirms `SQUARE_CORNER_VELOCITY` was in the *unsupported* list of `cmd_SET_VELOCITY_LIMIT` until commit `04c823c5c`. The running klippy at test time predated that commit (the Pi's git HEAD has since advanced to `c3310b160`, which supports it, but the process under test did not). So if the user changed SCV via `SET_VELOCITY_LIMIT`, every value was rejected and SCV stayed at the `[printer]` config value (`= 5`, per all config dumps) — a second, independent reason rounding never changed, on top of Finding 0.

**Note:** Finding 0 means that even with SCV *correctly* applied (current code), rounding of a cocircular square still barely moves — so fixing the runtime plumbing alone would not have satisfied the user's expectation.

## Missing Evidence

| Gap                                    | Impact                                              | How to Obtain                                              |
| -------------------------------------- | --------------------------------------------------- | --------------------------------------------------------- |
| Exact SCV=0 crash message / log        | Confirms or refutes Hypothesis 1 mechanism          | query-logs / mcu-diagnostics on the bench session         |
| accel in use during the test           | Determines whether Deduction 1 fully explains "same" | Bench config / `SET_VELOCITY_LIMIT` echo                  |
| How SCV was changed + how observed     | Separates Deduction 1 from Hypothesis 2             | User recollection of the test procedure                   |

## Follow-up: 2026-06-19 — where the square→circle decision lives, and intent

### Finding 2: the decision is in the chain-fitter `grow_run`; its turn-angle gate has no upper bound

**Evidence:**
- `rust/geometry/src/fitter/chain.rs:31` `detect_runs` → `grow_run` (`:60`) → `reconstruct` (`:108`). `grow_run` extends a run while `theta_min_rad < theta < theta_max_rad` and plane/turn-sign are consistent.
- `fitter.rs:22-27`: `CornerFitConfig::default` sets `theta_max_rad = PI − COLLINEAR_EPS_RAD` (≈180°) and `theta_min_rad ≈ 0`. So **any** turn from ~0° to near-reversal is eligible — a 90° square corner is included.
- Gates that *do* exist: `min_run_junctions = 2` (`fitter.rs:32`) and `cocircular_tol = 5e-3` mm (`fitter.rs:31`). A square's 3 equal same-direction corners and its exact incircle satisfy both.

**Detail:** The fitter cannot distinguish a *faceted arc* (many small per-corner turns from a slicer-polylined curve) from an *intentional polygon* (few large per-corner turns), because the only turn-angle bound is the near-reversal limit. This is the root mechanism behind Finding 0.

### Finding 3: arc-recovery-from-G1 is not in any spec; it was an ad-hoc throughput feature

**Evidence:** Introduced in commit `dd0361478` ("global continuous-κ chain fit — reconstruct faceted arcs…"), authored 2026-06-18, with no design doc. Stated intent: cruise faceted runs at √(a·R) instead of the per-corner sawtooth √(a/κ_peak). No spec under `docs/superpowers/specs/` or `docs/rewrite/` references it.

### Finding 4: native arcs come only from G2/G3; line↔arc clothoid blending is unimplemented

**Evidence:** `frontend.rs:137` `arc_move` builds `Segment::Arc` (the G2/G3 path). `classify_junction` (`fitter.rs:279-286`) blends only when both incident moves are lines; otherwise it returns `UnblendReason::ArcIncident` (left unblended/stop). This matches the user's intent ("the only circle is a G2/G3") and confirms the desired line↔arc clothoid-half easement does not exist yet.

### Design direction (per user, 2026-06-19)
Arcs should originate only from G2/G3. The synthetic faceted-arc recovery (`detect_runs`) either should be removed, or its `grow_run` gate tightened with a real per-facet turn-angle cap so it can never absorb polygon corners. Separately, the intended line↔arc joint should be eased with a clothoid half rather than tagged `ArcIncident`.

## Source Code Trace

| Element       | Detail                                                                                  |
| ------------- | --------------------------------------------------------------------------------------- |
| SCV → fillet  | `fitter.rs:399` `junction_deviation` → `fitter.rs:301` `delta` → `biclothoid.rs:32-33`  |
| SCV plumbing  | `bridge.rs:3369` (`submit_move`), `config.rs:521-523` (runtime override), `viz.rs:21`   |
| SCV=0 path    | `fitter.rs:302` ZeroDeviation → `velocity.rs:193` forced stop → `stream.rs:169` warm-start |
| Crash surface | `stream.rs:31,52` `StreamError::Velocity(VelocityError)`                                 |

## Conclusion

**Confidence:** High on "no rounding change"; Low/unresolved on the exact SCV=0 crash (event rolled out of logs).

"All SCV settings, same rounding" has a confirmed root cause that is **not** SCV-related: the chain arc-fitter (`fitter/chain.rs`) recognizes the square's four cocircular sides and replaces them with the r=25 mm inscribed-circle arc, cutting each corner ~10.4 mm. The radius is geometry-only; SCV merely tunes the clothoid easing length (sub-0.3 mm effect). A second, independent contributor: the klippy under test rejected `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY` outright (pre-`04c823c5c` code), so SCV never changed regardless.

SCV=0 does **not** crash the core fit/velocity pipeline (verified). The bench "crash" is unconfirmed — no planner panic survives in the logs; the retained failures are all [limit]/[printer] cutover config errors. It is most plausibly a streaming-path over-commit (untested) or a cutover config/restart failure.

## Recommended Next Steps

### Fix direction (the real lead)

The headline issue is the **arc-fitter collapsing a non-arc square into its inscribed circle**. Decide whether that is intended:
- If a 50 mm square should print square, the chain-fit acceptance criteria need tightening — `incircle` residual < 5µm is satisfied by *any* regular polygon, so the fitter currently treats squares/rectangles/hexagons as coarse arc approximations. Candidate guards: cap `theta_run` per chain, require a minimum facet count or maximum per-facet turn before arc-fitting, or bound the chord error against the original polyline (not just cocircularity).
- This is `bmad-quick-dev` / `bmad-create-story` territory once the desired behavior is decided.

### Diagnostic (for the SCV=0 crash, if it recurs)

1. Reproduce the **streaming** path (not viz's from-rest planner) at SCV=0 — a `plan_velocity_warm_start` unit test on a short square with `entry_v>0`, watching for `OverCommitted`/`Diverged`.
2. If it only happens on the bench, capture the next occurrence before log rotation: it would be a `host-rust motion` error or a `StreamError`, not a `Config error`.

## Side Findings

- The bench's git HEAD (`c3310b160`) was ahead of the klippy process actually running at test time (pre-`04c823c5c`) — i.e. pulled but not restarted. Worth confirming the bench is restarted after pulls before drawing behavioral conclusions.
- `[junction] anomalous jump` and `[seg0-deficit] (negative deficit_us => in past)` warnings recur in `host-rust motion` (13:18–14:23) — unrelated to SCV but possibly worth a separate look.
- The fork repurposes the classic-Klipper junction-deviation formula as a *geometric* fillet (`fitter.rs:399`), a real departure from classic Klipper (where SCV sets only cornering speed, never path shape).
