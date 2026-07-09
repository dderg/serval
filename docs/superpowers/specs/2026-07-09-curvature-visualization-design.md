# Curvature-based visualization for the snapshot viewer/playground

## Motivation

The web viewer and playground (`snapshots/web/static/`, sharing `trajectory-view.js`)
currently color path segments by the fitter's self-reported type: line, arc, or
clothoid (`SegmentType` in `rust/snapshot-viewer/src/lib.rs`, driven by
`FittedSegment` data in the snapshot). That label describes the *pre-shaper*
geometry the fitter chose — not what the firmware actually executes. The
executed trajectory (`traj_x_pieces`/`traj_y_pieces` etc., the post-fit,
post-plan, post-shape polynomial pieces) can differ from the fitter's intent,
e.g. because the input shaper's convolution distorts the path. Coloring by the
fitter's label can't show that.

The goal of this change: measure curvature directly from the executed
trajectory and color by what it actually does — zero, constant non-zero,
linearly changing (clothoid-like), or "other" (a catch-all for behavior that
doesn't fit those patterns, e.g. a shaper-distorted stretch). This makes the
viewer a tool for finding pipeline issues, not just illustrating fitter intent.
As a consequence, the fitter-type data (`FittedSegment`) becomes unnecessary
for visualization and can be dropped from the snapshot schema.

A guiding principle that shaped several decisions below: **the visualizer
should assume nothing about how the trajectory was constructed internally**
(piece boundaries, fitter segment identity) and should only reason about the
literal output — because a pipeline bug could just as easily corrupt that
internal structure as the trajectory itself, and trusting it would hide
exactly the class of bug this tool exists to catch.

## Goals

- Add a curvature-vs-time graph panel, alongside the existing
  velocity/acceleration/jerk panels.
- Replace segment-type path coloring with curvature-behavior classification:
  Zero, Constant, Linear, Other, plus two distinct point-anomaly flags (Gap,
  Cusp — see below).
- Remove `FittedSegment`/fitted-segment-type data from the snapshot schema.
- Retire the Python matplotlib visualizer (`scripts/viz_pipeline.py`) — fully
  superseded by the WASM viewer/playground — while preserving the G-code
  parsing and config-reading logic that `snapshots/harness.py` still needs to
  run cases.

## Non-goals

- No change to the automated pass/fail snapshot comparison. Curvature
  classification is a rendering-only feature; the existing generic
  float-tolerant dict diff in `harness.py`'s `compare()` already catches any
  real change to `traj_x_pieces`/`traj_y_pieces`, which is what curvature is
  derived from.
- No change to the fitter, planner, or shaper themselves.
- No new snapshot schema field for curvature — it's computed at render time
  from data already stored.

## Scope

**Touched:**
- `rust/snapshot-viewer/src/lib.rs` — curvature/classification computation,
  new WASM-bound getters, removal of `SegmentType`/`segment_type`/`segment_data`.
- `rust/pipeline-snapshot/src/lib.rs` — remove `FittedSegment`, `sample_segment`,
  the `fitted_segments` field and its population in `pipeline_snapshot`.
- `rust/motion-engine/src/viz.rs` — remove the `FittedSegment` PyO3 bridge.
- `snapshots/web/static/trajectory-view.js` — new curvature graph panel;
  path coloring driven by the new classification getter instead of
  `segment_type`/`COLORS`-by-type.
- `snapshots/web/static/app.js` — remove the `fitted_segments` loop.
- `snapshots/web/static/viewer.html` — remove the dead PNG-overlay
  markup/CSS (confirmed unreferenced by any current frontend code).
- `snapshots/web/server.py` — remove `_render_png`, the `.png` route, and the
  `viz_pipeline.render` call.
- `snapshots/harness.py` — absorb `read_printer_config`, `parse_gcode`,
  `PrinterConfigData` from `viz_pipeline.py`.
- `scripts/viz_pipeline.py` — deleted.
- `scripts/test_viz_pipeline.py` — deleted (or folded into harness's own
  tests if any parsing-specific assertions are worth keeping).
- `snapshots/README.md` — drop the "VISUALIZE tool" framing, describe the
  new curvature view.
- `rust/pipeline-snapshot/src/tests.rs`, `snapshots/test_harness.py` — update
  fixtures/assertions referencing `fitted_segments`.

**Playground** (`playground.html`/`playground.js`) needs no separate work — it
shares `trajectory-view.js` and its pipeline (`rust/motion-playground`, which
shares `rust/pipeline-snapshot`) produces the same piece format, so it
inherits both the graph and the new coloring automatically.

## Architecture

The snapshot schema shrinks to: `raw_x`, `raw_y`, `traj_x_pieces`,
`traj_y_pieces`, `traj_z_pieces`, `traj_e_pieces`, `traj_t_end`,
`traversal_time_s`, `seam_max_dp/dv/da`, `worst_seams`. Curvature is not
stored — it's derived every time from the pieces already present.

Computation lives in `rust/snapshot-viewer`, next to the existing analytic
derivative evaluation that already drives the velocity/acceleration/jerk
panels and `frenet_components`:

- Reuse the existing dense time-sampling grid built for those panels — no new
  sampling infrastructure.
- At each sample, evaluate `x′,y′,x″,y″` (already available) plus `x‴,y‴`
  (new — needed for dκ/dt) from whichever piece's coefficients cover that `t`.
- Compute κ(t) and dκ/dt(t) in closed form; `speed(t) = hypot(x′,y′)`
  (already computed elsewhere) gives `dκ/ds = (dκ/dt) / speed(t)` pointwise.
- While walking the grid, check consecutive pieces are contiguous
  (`piece[i].u_end == piece[i+1].u_start` within tolerance); a gap or overlap
  is flagged, not bridged.
- A sample where `speed(t)` is below a small floor (reuse the existing
  `FRENET_SPEED_FLOOR` constant) is flagged as a cusp, not divided into.

New WASM-bound getters expose: κ(t) per sample (for the graph) and a
per-sample class/flag tag (for path coloring), replacing `segment_type`/
`segment_data`.

## Why time-domain, not arc-length

κ itself is parameterization-invariant — the same value whether computed via
t-derivatives or s-derivatives. Plotting and evaluating in the time domain
(rather than reparameterizing to arc length) avoids introducing a numerically
integrated axis (`sqrt` of a polynomial has no closed-form antiderivative in
general) purely to re-present something that doesn't need it, and it keeps
the new panel's x-axis aligned with velocity/acceleration/jerk for visual
correlation. Only the *rate of change along the path* (dκ/ds) needs the
speed normalization, and that's a pointwise algebraic division, not a
cumulative integral — no arc-length axis required anywhere.

## Classification algorithm

**Per-sample (pointwise, exact):**
- κ(t) — the curvature graph value.
- speed(t) — below floor → tag **Cusp**.
- dκ/ds(t) = (dκ/dt)/speed(t) — skipped where Cusp.
- Sample in a detected piece gap/overlap → tag **Gap**.

**Per fixed-size window** (a fixed count of consecutive samples off the
existing dense grid, not tied to piece boundaries):
- All |κ| in the window below a small threshold → **Zero**.
- Otherwise all |dκ/ds| near zero → **Constant**.
- Otherwise dκ/ds roughly steady (small spread, either sign) across the
  window → **Linear**.
- Otherwise → **Other**.

A window's Zero/Constant/Linear/Other statistics are computed only from its
non-Cusp, non-Gap samples; a Cusp or Gap sample itself always displays as its
own flag, regardless of how the rest of the window classifies.

Exact epsilon thresholds are tuning parameters, picked and adjusted against
real snapshot cases during implementation rather than fixed here.

### Noise mitigation

Splines only guarantee continuity up to some order below their degree, so
dκ/ds can legitimately jump slightly at *every* knot even in a perfectly
healthy trajectory — and shaper convolution multiplies knot density
substantially. A naive windowed classifier would speckle "Other" at every
seam regardless of whether anything is actually wrong, burying the real
signal. Three mitigations, used together:

1. **Window spans multiple pieces.** Size the window (in sample count) large
   enough that a typical window covers several knots, so one seam's
   discontinuity is a small perturbation on the window's overall statistics,
   not the whole story.
2. **Robust spread statistic**, not raw min/max (e.g. a percentile-based
   spread) — a handful of seam-adjacent samples with a genuine-but-tiny
   higher-derivative jump can't single-handedly flip a window's class.
3. **Hysteresis between adjacent windows** — a new class must persist across
   a couple of consecutive windows (or: overlapping windows, majority vote)
   before the displayed color changes, turning flicker into stable,
   contiguous colored stretches.

The property that makes this tractable: a genuine pipeline anomaly persists
over many consecutive samples, while a seam artifact is confined to a
handful of samples at one knot. This needs a visual sanity check once built —
run against a real case with heavy shaper convolution and confirm the
coloring reads as clean contiguous regions, not speckled.

## Visual design

Colors reuse existing hues where the meaning lines up, for continuity with
the tool as it exists today: Zero ≈ old line color, Constant ≈ old arc color,
Linear ≈ old clothoid color. **Other** gets a new, distinctly alarming color
since it's the new diagnostic signal. **Gap** and **Cusp** are point
anomalies, not stretches of behavior — they get a distinct marker (not a
fill color) at the exact sample, similar to how `worst_seams` are already
flagged.

## Python visualizer removal

`scripts/viz_pipeline.py` is fully superseded by the WASM viewer/playground.
Its matplotlib rendering (`render`, `main`, the plotting/Frenet/gradient
helpers it duplicates from what `snapshot-viewer` already does in Rust, and
the `_reexec_in_printer_env` klippy-env dance that exists solely to reach
matplotlib) is deleted outright. The only load-bearing pieces —
`read_printer_config`, `parse_gcode`, `PrinterConfigData`, used by
`harness.py` to actually run cases from `.gcode`/`.cfg` files — move directly
into `harness.py`. `snapshots/web/server.py`'s PNG rendering path
(`_render_png`, the `.png` route) is dead code (confirmed unreferenced by the
current frontend) and is removed along with the unused `png-overlay`
markup/CSS in `viewer.html`.

## Snapshot schema change and baseline impact

Removing `fitted_segments` changes the snapshot schema, so every existing
`.baseline.json.gz` will differ in shape from a freshly generated snapshot
the moment this lands. The generic dict diff in `harness.py`'s `compare()`
will report every case as `CHANGED` on the first run after this change,
purely from the schema diff, not from any real trajectory change. Baseline
regeneration is done explicitly by the user, not automatically — landing
this requires running the snapshot suite once and accepting all cases, and
that first pass showing universal changes is expected and harmless.

## Testing

Per project convention, unit tests live in a separate file from the tested
code (`rust/snapshot-viewer` already follows this with its `#[cfg(test)] mod
tests`):

- κ(t)/dκ/dt formulas against synthetic cases with known closed-form
  answers: a straight line (κ≡0 everywhere), a circular arc parameterized
  with a **non-constant** speed profile (κ must stay constant regardless —
  this is the exact check that would catch a t/s-domain mixup), a
  synthetic clothoid-like curve with a known σ (dκ/ds constant).
- The classification logic tested directly against synthetic (κ, dκ/ds)
  sequences: clean examples of each of the four classes; a sequence with a
  single-sample seam-like jump inserted (must **not** flip the window's
  class, proving the robust-stat + hysteresis mitigation works); a sequence
  with a sustained shift inserted (must still get flagged, proving real
  anomalies aren't suppressed along with the seam noise).
- Gap/overlap detection against a deliberately malformed piece list; cusp
  detection against a synthetic reversal.
- Existing `FittedSegment`-related tests in `rust/pipeline-snapshot/src/tests.rs`
  and any in `snapshot-viewer` are removed/updated for the schema change.

**Manual verification:** run the snapshot suite locally, review a few real
cases in the browser — especially ones with heavy shaper convolution — to
confirm the coloring reads as clean, stable, contiguous regions rather than
speckled, and confirm the playground renders the same way since it shares
the code path.
