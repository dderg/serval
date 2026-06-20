---
title: 'Opt-in [arc_fit] config section: gate faceted-arc recovery behind two knobs, off by default'
type: 'feature'
created: '2026-06-19'
status: 'done'
baseline_commit: '07deb7d9bc9fd83f5d2ac91a6153796dc7f08881'
context:
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-8-chain-fit.md'
  - '{project-root}/CLAUDE.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The step-8 chain fitter (`fit_chain`/`detect_runs`) recovers an arc from any run of co-circular, same-turn `Line` facets. Its only turn gate is the near-reversal cap (`θ_max ≈ 180°`) and it has no facet-length gate, so a real square — whose four sides are trivially tangent to one inscribed circle — gets collapsed into that circle (~10 mm corner cut), independent of any setting. Confirmed empirically on a 50 mm square. spec-motion-8 pre-flagged "moving chain thresholds onto a user knob" as an Ask-First item; this resolves it.

**Approach:** Make faceted-arc recovery **opt-in** via a new `[arc_fit]` config section with two knobs — `facet_length_mm` and `max_angle_deg`. When the section is absent, `detect_runs` does nothing and every corner takes the unchanged per-corner biclothoid path (a square stays a square; arcs come only from G2/G3). When present, a run grows only across junctions that both turn ≤ `max_angle_deg` and join facets ≤ `facet_length_mm`, so genuine slicer faceting is recovered while intentional polygons never are.

## Boundaries & Constraints

**Always:**
- Off by default: no `[arc_fit]` ⇒ `arc_fit = None` ⇒ `detect_runs` empty ⇒ per-corner biclothoid handles every junction, byte-identical to today's isolated-corner path.
- The two knobs are the only user params; `cocircular_tol`/`min_run_junctions` stay internal. `max_angle_deg` = per-junction turn (deviation from collinear); `facet_length_mm` = max arclength of each run `Line`.
- A run falls through to per-corner if any junction turn > `max_angle_deg` OR either incident facet > `facet_length_mm`. Existing near-reversal `θ_max` break retained.
- Plumb Python→FFI→planner like `[printer]`/`[limit]`: motion.py reads + passes through `init_planner`; bridge builds + stores `ChainFitConfig` so live `StreamConfig.chain` uses it, not `::default()`.
- Fail loud (CLAUDE.md): a present `[arc_fit]` with non-positive knob errors at config time. No silent clamping.

**Ask First:**
- Implementing the line↔arc clothoid-half blend (still `ArcIncident`/unblended) — out of scope here, a separate follow-up.
- Exposing `cocircular_tol`/`min_run_junctions` as knobs, or changing default knob values away from the proposed `facet_length_mm=1.0`, `max_angle_deg=12.0`.

**Never:**
- No change to `biclothoid`/`fit_corners` per-corner behavior, to the `Line/Arc/Clothoid` alphabet, or to the `plan_velocity` read-contract.
- No velocity/timing/shaper logic — geometry only.
- Do not keep arc recovery on by default to preserve faceted-arc throughput; the user owns this tradeoff (slicers should emit G2/G3 or opt in).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| No section | `[arc_fit]` absent; any geometry | `arc_fit=None`; `chains==0`; square stays sharp; isolated corners blend as before | N/A |
| Section, square | `[arc_fit]` present; 50 mm square (90° turns) | run never grows (90° > `max_angle_deg`); `chains==0`; corners per-corner | N/A |
| Section, faceted arc | present; short co-circular facets within both knobs | reconstructed `Clothoid·Arc·Clothoid`; `chains==1` | N/A |
| Section, long facets | present; co-circular but facet len > `facet_length_mm` | run breaks on length; per-corner | N/A |
| Section, shallow but long polygon | present; 10° turns, 17 mm sides | run breaks on length; stays polygon | N/A |
| Bad param | `[arc_fit]` with `facet_length_mm<=0` or `max_angle_deg<=0` | config error at startup | config.error |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/fitter.rs` — `ArcFitConfig`, `ChainFitConfig.arc_fit: Option`, `with_arc_fit(..)` (drafted in working tree).
- `rust/geometry/src/fitter/chain.rs` — the gate logic: `detect_runs` early-return + `grow_run` length/turn breaks.
- `rust/motion-engine/src/{config.rs,bridge.rs}` — `PlannerConfig.chain` field; `init_planner` arg; build/store/use at the live `StreamConfig` site.
- `rust/motion-engine/src/viz.rs` — keeps `::default()` (now disabled); square renders sharp.
- `klippy/motion.py` — reads/validates `[arc_fit]`, passes `None` when absent.
- test sites (`fitter/chain/tests.rs`, `fitter/tests.rs`, motion-engine `{viz,stream,bridge,stream_planner,lowering}/tests.rs`, `geometry/tests/integration_pipeline.rs`) — opt reconstruction asserts into `with_arc_fit`; add new gate tests.
- `docs/Config_Reference.md` — `[arc_fit]` section.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/fitter.rs` — `ArcFitConfig`/`arc_fit: Option`/`with_arc_fit` (default `None`); re-exported via `lib.rs`.
- [x] `rust/geometry/src/fitter/chain.rs` — `detect_runs` early-return when disabled; `grow_run` length + turn gates.
- [x] `rust/motion-engine/src/config.rs` + `bridge.rs` — `PlannerConfig.chain`, `init_planner` `arc_fit` arg, build + store + use at the live `StreamConfig` site; `config/tests.rs` literal updated.
- [x] `klippy/motion.py` + `motion_engine.py` — parse `[arc_fit]`, validate (`above=0.0`), plumb through `init_planner` (None when absent).
- [x] tests — chain reconstruction sites routed through an ungated `cfg()` helper; added `no_arc_fit_config_never_chains`, `sharp_corners_rejected_by_angle_gate`, `faceted_arc_within_default_gates_reconstructs`, `long_facets_rejected_by_length_gate`; integration test repurposed to `square_stays_sharp_without_arc_fit`; topology test asserts `arc_fit` plumbed. Bad-param enforced by klippy `getfloat(above=0.0)` + bridge `PyValueError` backstop.
- [x] `docs/Config_Reference.md` — `[arc_fit]` section.

**Acceptance Criteria:**
- Given no `[arc_fit]` section, when a square of G1 moves is planned, then `report.chains == 0` and the fitted path keeps sharp corners.
- Given `[arc_fit]` with default knobs, when the same square is planned, then it still keeps sharp corners; when a sub-`facet_length_mm`, sub-`max_angle_deg` co-circular facet run is planned, then it reconstructs (`chains == 1`).
- Given `[arc_fit]` with a non-positive knob, when klippy loads, then it raises a config error.
- `./scripts/ci.sh quick` green; `./scripts/ci.sh py` green (motion.py touched).

## Design Notes

Default knobs `facet_length_mm = 1.0`, `max_angle_deg = 12.0`: slicer facets typically turn 3–16° over sub-mm–~2 mm chords; polygons start at 45° (octagon)/30° (12-gon)/15° (24-gon). The angle gate is the primary discriminator (kills the square at 90°); the length gate is the backstop for shallow-but-coarse polygons. Caveat to record in code/docs: large-radius finely-faceted arcs have long-but-shallow facets, so a tight `facet_length_mm` will skip them (revert to per-corner) — acceptable since recovery is now opt-in; a deviation-based gate is the future refinement.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p geometry` -- expected: green, incl. new square/gate tests
- `cd rust && cargo nextest run -p motion-engine` -- expected: green (bridge/viz/stream config plumbing)
- `./scripts/ci.sh quick` -- expected: green (ruff, rust tests, clippy -D warnings, fmt, watchdog)
- `./scripts/ci.sh py` -- expected: green (motion.py config parsing)

**Manual checks:**
- `pipeline_snapshot` on a 50 mm square with default `ChainFitConfig` reports `chains==0` and min-distance-to-corner ≈ 0 (sharp), reproducing the fix for the investigated bug.

## Suggested Review Order

**The gate (design core)**

- The two-knob break — a run stops growing on a too-long facet or a too-sharp turn.
  [`chain.rs:80`](../../rust/geometry/src/fitter/chain.rs#L80)
- Off-switch: with no `[arc_fit]`, detection never runs (square stays a square).
  [`chain.rs:36`](../../rust/geometry/src/fitter/chain.rs#L36)
- The opt-in config type and its `None`-by-default.
  [`fitter.rs:35`](../../rust/geometry/src/fitter.rs#L35)

**Config plumbing (Python → FFI → planner)**

- Reads `[arc_fit]`; `None` when absent; bounds `max_angle_deg` to (0, 180).
  [`motion.py:632`](../../klippy/motion.py#L632)
- FFI: validates knobs (fail-loud), converts deg→rad once, builds the config.
  [`bridge.rs:2627`](../../rust/motion-engine/src/bridge.rs#L2627)
- The load-bearing wire: live stream planner uses the configured chain, not the default.
  [`bridge.rs:3318`](../../rust/motion-engine/src/bridge.rs#L3318)
- New `PlannerConfig` field carrying the chain config.
  [`config.rs:389`](../../rust/motion-engine/src/config.rs#L389)

**Tests & docs (supporting)**

- Square stays sharp end-to-end with arc fitting off (the fixed bug).
  [`integration_pipeline.rs:203`](../../rust/geometry/tests/integration_pipeline.rs#L203)
- Angle gate isolates the square; length gate isolates long facets.
  [`chain/tests.rs:143`](../../rust/geometry/src/fitter/chain/tests.rs#L143)
- User-facing `[arc_fit]` section reference.
  [`Config_Reference.md:248`](../../docs/Config_Reference.md#L248)
