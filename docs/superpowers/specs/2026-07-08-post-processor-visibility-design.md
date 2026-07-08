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
- The playground's Path panel already plots the post-shaper trajectory
  (`kin_x`/`kin_y` sample `traj_x/y_pieces`, the final lowered-and-shaped
  output) — `fitted_segments` is used only to color-classify each sample as
  line/arc/clothoid. So once post-processors are wired up, this panel will
  show their effect automatically. What's missing is the *other* view: seeing
  what the fitter alone produced, before lowering and shaping touch it —
  there's currently no way to sample position from `fitted_segments` directly.

Goal: build durable infrastructure to (a) visually investigate post-processor
behavior in the playground, and (b) capture interesting configurations as
snapshot baselines so their behavior can be tracked for drift over time as the
planner changes. This is verification/observability tooling — it does not
change the motion engine's runtime behavior.

## Architecture: one shared compile path

The core fix is replacing `pipeline-snapshot`'s own hand-rolled
`build_axis_chains()` with direct reuse of the real config-compile path:
`AxisRegistry` + `PostProcessorDecl` + `PostProcessorSet::try_new(...).compile(...)`
— the exact same code the live engine uses to turn declared config into an
`AxisChainSet`. Today that path lives in `motion-core::config`.

**Crate extraction required.** `motion-core` unconditionally depends on
`host-rt`, `mcu-protocol`, `runtime`, and `libc` — needed by its other 14
modules (pump, worker, enqueue, homing, etc.), none of which compile to
`wasm32-unknown-unknown`. Cargo dependencies are per-crate, not per-module, so
`pipeline-snapshot` depending on `motion-core` at all would drag those
hardware-facing crates into the `motion-playground` WASM build and break it —
defeating the entire point of sharing one compile path with the WASM
playground. `config.rs`'s own code only touches `trajectory`, `geometry`, and
`thiserror` (confirmed by reading the file in full), so it's extracted into a
new, small crate — `rust/planner-config` — with just those three
dependencies. `motion-core` re-exports it (`pub use planner_config as
config;` in its `lib.rs`), so every existing reference to `config::AxisDecl`,
`config::PostProcessorSet`, etc. across `motion-core` and
`motion-engine`'s PyO3 bridge keeps compiling unchanged. `pipeline-snapshot`
depends on `planner-config` directly and stays wasm-compatible.

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

### 0. `rust/planner-config` (new crate, extracted from `motion-core::config`)

- New workspace member holding exactly what's in today's
  `motion-core/src/config.rs`: `AxisDecl`, `AxisRegistry`, `PostProcessorDecl`,
  `PostProcessorSet`, `PlannerConfig`, `LimitSection`, `CartesianLimits`,
  `RuntimeCaps`, and their error enums — unchanged code, moved wholesale.
  Dependencies: `trajectory`, `geometry`, `thiserror` only.
- `motion-core` drops its own `config` module and instead depends on
  `planner-config`, re-exporting it as `pub use planner_config as config;` —
  zero changes required anywhere else in `motion-core` or in
  `motion-engine/src/bridge/planner_api.rs`, which references `config::*`
  throughout.

### 1. `rust/pipeline-snapshot` (shared core)

- Takes a new dependency on `rust/planner-config`.
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

- Today's Path panel already shows the shaped (post-lowering, post-shaper)
  trajectory: `kin_x`/`kin_y` on `TrajectoryData` sample `traj_x/y_pieces` via
  `build_time_series` → `eval_lane` → `eval_piece`. `fitted_segments` is used
  only by `parse_segments()` to color-classify each sample as line/arc/
  clothoid — it is not an alternate position source today. No change needed
  to make the Path panel reflect post-processors; it already will, the
  moment they're wired up.
- What's added: a "fitted" path drawn directly from each segment's own
  recorded geometry (line endpoints; arc/clothoid pre-sampled point arrays)
  — independent of `traj_x/y_pieces` entirely, i.e. before lowering or
  shaping touch it. This reuses the `segment_count`/`segment_type`/
  `segment_data` accessors `TrajectoryData` already exposes (today used only
  to color-classify the shaped path); no new Rust/WASM surface is needed —
  the new drawing logic lives entirely in `trajectory-view.js`.
- Add a toolbar toggle (next to Pin baseline / reset zoom / toggle peaks) to
  switch the Path panel between "shaped" (current/default behavior, from
  `kin_x`/`kin_y`) and "fitted" (new, from each segment's own points).
- Not overlaid: fitted and shaped routinely differ even with no
  post-processors configured, because the lowering stage sits between them —
  overlaying by default would be visual noise rather than signal. A toggle
  lets you deliberately inspect either the fitter's output or the pipeline's
  actual output.
- Default view: "shaped", matching current behavior — least disruptive for
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
- The fitted/shaped Path panel toggle is implemented entirely in JS, reusing
  the already-tested `segment_count`/`segment_type`/`segment_data` WASM
  accessors rather than adding new ones — no new Rust unit test needed for
  it. It's verified manually in a browser instead, per this repo's usual
  practice for `trajectory-view.js` changes (there's no JS test framework
  for this file).

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
