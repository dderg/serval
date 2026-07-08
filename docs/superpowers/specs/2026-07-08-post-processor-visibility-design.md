# Post-processor visibility in the playground and snapshots

Date: 2026-07-08

## Motivation

Post-processors (`[post_processor]` sections — input shaping and pressure
advance, chained per axis) are a first-class Kalico concept, but today they're
invisible to both verification tools:

- The interactive playground (`rust/motion-playground` +
  `snapshots/web/static/playground.*`) has 3 dead config fields
  (`pressure_advance`, `smooth_zv_hz`, `e_smooth_zv_hz`) wired into its WASM
  binding, but the HTML/JS never exposes them as inputs — there's no way to
  turn shaping or PA on from the playground UI at all.
- Every committed snapshot baseline was generated with post-processing
  disabled: the Python harness (`scripts/viz_pipeline.py::read_printer_config`)
  never reads `[post_processor]`/`[axis]` sections from case `.cfg` files, so
  no baseline in the repo reflects post-processor behavior.
- Even where post-processor params reach `pipeline_snapshot()`, the shared
  `build_axis_chains()` only recognizes 2 of the 4 registered post-processor
  types (`smooth_zv`, `linear_pressure_advance`) by hardcoded name — `smooth_mzv`
  and `smooth_triangle` aren't reachable at all.
- The playground's Path panel plots position purely from the fitter stage's
  `fitted_segments`, never the actual post-shaper trajectory — so even once
  post-processors are wired up, the one panel most likely to show their effect
  wouldn't reflect it.

Goal: build durable infrastructure to (a) visually investigate post-processor
behavior in the playground, and (b) capture interesting configurations as
snapshot baselines so their behavior can be tracked for drift over time as the
planner changes. This is verification/observability tooling — it does not
change the motion engine's runtime behavior.

## Architecture: one shared compile path

The core fix is replacing `pipeline-snapshot`'s own hand-rolled
`build_axis_chains()` with direct reuse of `motion-core`'s real config-compile
path: `AxisRegistry` + `PostProcessorDecl` + `PostProcessorSet::try_new(...).compile(...)`
— the exact same code the live engine uses to turn declared config into an
`AxisChainSet`.

`SnapshotParams` drops its 3 dead flat fields in favor of:

```rust
pub struct SnapshotParams {
    // ...existing fitter/planner params unchanged...
    pub axis_decls: Vec<AxisDecl>,
    pub post_processor_decls: Vec<PostProcessorDecl>,
}
```

Both consumers (the Python-facing PyO3 binding and the WASM playground) only
need to *produce* those decl lists — from real config parsing on the Python
side, from a small new text parser on the WASM side — and hand them to the
same shared compile call. Validation (unknown post-processor type,
composition-slot conflicts, undeclared axis references) happens once, in one
place, and behaves identically to what a live printer would do with the same
config. This is deliberately the more invasive option over extending
`build_axis_chains()`'s hardcoded match arms in place — a parallel, hand-rolled
validation path risks drifting from production behavior over time, which
directly undermines the "verify it's actually working correctly" goal.

**Future-proofing note:** the engine's current composition model
(`rust/trajectory/src/chain.rs`) caps each axis chain at 2 slots — one shaping
kernel (`smooth_zv`/`smooth_mzv`/`smooth_triangle`, mutually exclusive) and one
derivative-gain (`linear_pressure_advance`) — and errors
(`UnsupportedComposition`) if a chain would need more. Extending this to
arbitrary N-deep chains is a separate, future core-engine project, out of
scope here. Because the playground's config surface is free-form text (not
fixed dropdowns — see below), nothing in this design imposes an additional
cap on top of the engine's: you can already reference as many post-processors
per axis as you like, and the *only* place the 2-slot limit is enforced is
`CompiledChain::compile()`. When that limit is lifted, the playground needs no
changes to take advantage of it.

## Components

### 1. `rust/pipeline-snapshot` (shared core)

- `SnapshotParams` takes `axis_decls`/`post_processor_decls` as described
  above.
- `pipeline_snapshot()` builds an `AxisRegistry` from `axis_decls` and calls
  `PostProcessorSet::try_new(&registry, post_processor_decls)?.compile(&registry)`
  to get the `AxisChainSet`, replacing the old `build_axis_chains()` entirely.
- All 4 registered post-processor types become reachable from both
  playground and snapshots, with no special-casing per type.

### 2. PyO3 binding (`rust/motion-engine/src/viz.rs`)

- The `#[pyfunction] pipeline_snapshot` signature changes from the 3 flat
  `Option<f64>` kwargs to accepting axis and post-processor declarations,
  reusing the same `FromPyObject` extraction shapes `planner_api.rs` already
  defines for `init_planner` (e.g. the existing `PostProcessor` struct) rather
  than inventing a parallel shape.

### 3. Python harness (`scripts/viz_pipeline.py`, `snapshots/harness.py`)

- `read_printer_config()` is extended to also run klippy's own
  `motion_setup.py::read_axes()` / `read_post_processors()` against the loaded
  config object — the same parsing and validation a live printer goes
  through, not a reimplementation.
- Because this pushes the function's return value past the current fixed
  9-tuple, it's restructured into a small dataclass instead of growing the
  tuple further (in-scope cleanup, not a detour — the tuple was already
  awkward).
- `harness.run_case()` passes the parsed axis/post-processor decls through to
  `engine.pipeline_snapshot(...)` instead of omitting them.
- A case `.cfg` with a `[post_processor]` section now behaves identically in
  the harness as it would on a real printer, including surfacing the same
  config errors.

### 4. WASM playground config parser (new)

- A new, small Rust parser, living in `rust/motion-playground` (it's only
  needed by the WASM binding — the Python side already has a real config
  loader), understands `[axis <name>]` / `[post_processor <name>]` section
  syntax —
  mirroring the grammar `klippy/motion_setup.py` uses (section header,
  `key: value` lines, `post_processors: a, b` comma lists) — and produces the
  same `AxisDecl`/`PostProcessorDecl` shapes `motion-core` expects.
- Since the playground pipeline has a fixed axis topology (X, Y, Z, E — no
  configurable kinematics), the parser only needs `post_processors:` per
  axis; `follows:`/`motors:` fields, if pasted in from a real `printer.cfg`,
  are accepted and ignored rather than rejected.
- Parse/validation errors flow through the existing `{seq, error}`
  worker-message channel already used for G-code errors — no new error UI.

### 5. Playground front end (`playground.html`/`playground.js`)

- New textarea for pasting `[axis]`/`[post_processor]` sections, alongside
  the existing `[printer]`/`[extruder]`/`[arc_fit]` number-field panel.
- Free-form text, not fixed per-type fields — this is what keeps the number
  of post-processors and per-axis assignments unbounded at the UI layer (see
  Architecture, future-proofing note above).
- Debounced re-plan on edit (same 250ms debounce already used for
  gcode/config), persisted to `localStorage` alongside existing state.

### 6. Path panel fitted/shaped toggle (`rust/snapshot-viewer`, `trajectory-view.js`)

- Add position-sampling accessors to `TrajectoryData` that sample X/Y
  directly from `traj_x/y_pieces` (the final, post-shaper trajectory) —
  alongside the derivative accessors that already sample from the same
  pieces.
- Add a toolbar toggle (next to Pin baseline / reset zoom / toggle peaks) to
  switch the Path panel between "fitted" (current behavior — segment-colored
  line/arc/clothoid from `fitted_segments`) and "shaped" (new — from
  `traj_x/y_pieces`) views.
- Not overlaid: fitted and shaped routinely differ even with no
  post-processors configured, because the lowering stage sits between them —
  overlaying by default would be visual noise rather than signal. A toggle
  lets you deliberately inspect either the fitter's output or the pipeline's
  actual output.
- Default view: "fitted", matching current behavior — least disruptive for
  existing playground/snapshot-review usage.
- `trajectory-view.js` is shared unmodified between the playground and the
  snapshot-review viewer, so this toggle becomes available in both for free.

### 7. Starter snapshot cases

- New `snapshots/cases/post_processor/` group.
- One new G-code fixture with a 90° turn (junction/cornering behavior is
  where shaping and PA effects are actually visible — a straight line mostly
  isn't).
- 5 new `.cfg` files reusing that fixture: `smooth_zv.cfg`, `smooth_mzv.cfg`,
  `smooth_triangle.cfg`, `linear_pressure_advance.cfg`, and
  `chained_shaper_pa.cfg` (one shaper + PA together, exercising the 2-slot
  composition path).
- These case files are added as part of this work. The actual baseline
  `.json.gz` files are **not** — per repo convention, baselines are always
  generated by the user running the snapshot tool, not by an agent.

## Testing

- New Rust unit tests (separate test files, per repo convention) for:
  - The new raw-config-text parser (valid sections, malformed sections,
    unknown post-processor type, undeclared axis reference).
  - The generalized `pipeline-snapshot` chain-building: all 4 post-processor
    types reachable, composition-conflict errors surface, undeclared-axis
    errors surface.
- New Python tests for the extended `read_printer_config()` — since it calls
  `motion_setup.py` directly rather than reimplementing parsing, these mainly
  confirm plumbing (decls reach `pipeline_snapshot` correctly), not parsing
  correctness (already covered by `motion_setup.py`'s own tests).
- A unit test for the new shaped-position sampling accessors against known
  trajectory pieces.

## Error handling

Consistent with the repo's fail-loudly principle: unknown post-processor
type, undeclared axis references, and composition-slot conflicts all raise/
error rather than silently defaulting or dropping — in the WASM error
channel, in the Python harness (a bad case `.cfg` fails that harness run
rather than being silently skipped), and in the underlying compile path
itself (unchanged from production behavior, since it's the same code).

## Non-goals

- Extending the engine's composition model beyond 2 slots per axis — future
  work, tracked as a separate core-engine project.
- Capturing a new "pre-shaper" stage in the `Snapshot` schema. The existing
  fitted + shaped capture is enough for drift tracking over time; comparing a
  post-processor's raw effect is done via the playground's existing
  before/after "Pin baseline" A/B flow (same gcode/config, toggle the
  post-processor chain, pin + flip).
