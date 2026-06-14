# Live G5 (cubic Bézier) support — design

Date: 2026-06-14
Status: approved for planning

## Goal

Make a `G5` command typed at the console (or emitted from a macro) actually
move the toolhead along the cubic Bézier it describes. Today `G5` hits the
default handler and is rejected as `Unknown command`.

This is a **plumbing** task, not a motion-math task. The planner is already
Bézier-native and already runs the full time-optimal optimizer on every live
move; G5 only needs the control points to survive the trip from the g-code
line into the planner's segment.

## Scope

In scope:

- A live `cmd_G5` handler accepting Marlin G5 semantics: endpoint `X Y`,
  first control-point offset `I J`, second control-point offset `P Q`,
  optional `Z E F`.
- "Smooth" G5 chaining: a follow-on `G5` may omit `I J`, reflecting the
  previous G5's exit control point for C1 continuity.
- A new geometric bridge entry point that carries control points.
- Extraction of the one geometry primitive the live engine borrows from
  `compat` into `geometry`, severing the planner's dependency on `compat`.
- A fail-loud gate rejecting **activation** of a bed mesh (the move-transform
  layer is not curve-correct yet — see "Bed mesh" below).

Explicitly out of scope (deferred, not prerequisites):

- Live `G2` / `G3` / `G5.1`. Arc-to-Bézier and degree elevation are the
  slicer's job; their math stays in `compat`, which is moving to the slicer.
- Any bed-mesh / skew / bed-tilt interaction with curves. The move-transform
  layer needs rework for G1 too; that is a separate project. Until then bed
  mesh activation is gated off.
- A dedicated "motion-gcode" crate. The new math is ~30 lines of arithmetic
  and belongs in `geometry`.

## Background: how a live move works today

`G1 X100 F3000` flows:

```
gcode_move.cmd_G1            # owns g-code coordinate semantics
  -> Motion.move(newpos,spd) # validates (kin/extruder check_move), computes deltas
  -> bridge.submit_move(dx,dy,dz,de,F)        # pyo3 seam; geometry only
  -> classify_and_build(start,dx,dy,dz,...)   # bridge knows `start`
  -> to_collinear_bezier(start,end)           # a degenerate (straight) cubic
  -> append_and_replan -> plan_velocity       # full SOCP + SLP optimizer,
       -> temporal::multi::plan_batch              with multi-move lookahead
  -> emit_shaped -> Bernstein pieces -> MCU ring
  -> ISR ~40 kHz: eval_horner -> step times
```

There is no trapezoidal profiler. The planner's native primitive is a cubic
Bézier; a G1 is just the special case where the four control points are
collinear, fed to the same optimizer. **G5 reaches the identical optimizer —
the only difference is that its four control points are not collinear.**

## Architecture: where each responsibility lives

The pyo3 seam already separates the two jobs G5 needs, and the split does
not move:

- **Python (`gcode_move.py`, `motion.py`) owns g-code semantics and
  coordinate state** — absolute/relative (`G90/G91`), `base_position`
  (`G92`), gcode offset, extrude factor (`M221`) and absolute extrude
  (`M82/M83`), speed factor (`M220`/`F`), `SAVE`/`RESTORE_GCODE_STATE`, the
  move-transform chain, and the per-move validation (`kin.check_move`,
  `extruder.check_move`). All of this is shared across every motion command
  and is reused by G5 unchanged.
- **Rust (`motion-bridge`) owns curve geometry** — turning control points
  into a `CubicSegment`, holding the chaining state, and feeding the
  optimizer. The bridge already tracks the running `start` position.

Rejected alternative: moving motion-gcode parsing into Rust. The coordinate
state and transforms are Python objects shared by many commands; relocating
them across the FFI boundary is large, invasive, and buys nothing for G5.

### Plain-English summary

Python stays the translator and bookkeeper: it knows where the nozzle is,
whether numbers are absolute or relative, and what the speed override is. It
already does this for a straight line's endpoint; for a curve it does the
exact same thing for the endpoint, reads four extra "steering" numbers off
the line, and hands nine plain numbers to Rust. Python never learns what a
Bézier is; Rust never learns g-code state. The fence stays where it is.

## Compat extraction

`compat` is a g-code -> g-code preprocessor (the offline `kalico-compat`
binary, bound for the slicer). The planner and g-code parser must not depend
on it.

Workspace audit: the live engine's *entire* compat footprint is one function,
`to_collinear_bezier` (`compat/src/collinear.rs:20`), called only from
`motion-bridge/src/classify.rs`. Compat's own preprocessor logic never calls
it (it uses `to_collinear_g5`, the text emitter). The control-point primitive
is simply mis-filed in a g-code-text crate.

Changes:

1. Move `to_collinear_bezier` (and its two unit tests) from `compat` into
   `geometry`. Update `classify.rs` to import from `geometry`.
2. Remove `compat` from `motion-bridge/Cargo.toml`. After this the planner
   has zero compat edges.
3. Add the G5 control-point builder alongside it in `geometry` (see below).
4. Everything else in compat (`converter`, `fitter`, `arc`, `hausdorff`,
   `degree_elev`, `g5_canon`, `to_collinear_g5`, `modal`, `corner`, `run`,
   `emit`) stays put.

Dependency direction stays one-way: a preprocessor may depend on shared
primitives in `geometry`; `geometry` never depends on `compat`.

## Components and data flow

```
cmd_G5 (gcode_move.py)
  parse X Y Z E F I J P Q
  resolve ENDPOINT (X Y Z E) through existing coordinate state, like cmd_G1
  forward I J P Q as raw offsets (no base_position; they are deltas)
  -> Motion.move_curve(newpos, i, j, p, q, speed)   # sibling of Motion.move
       validate endpoint via kin/extruder check_move
       compute dx,dy,dz,de
  -> bridge.submit_bezier(i, j, p, q, dx, dy, dz, de, feedrate)  # motion_bridge.py passthrough
  -> Rust pyo3 submit_bezier
  -> classify_bezier(start, i, j, p, q, dx, dy, dz, followers, feedrate)
       assemble control points -> CubicSegment
  -> (unchanged) append_and_replan -> optimizer -> MCU
```

### The handoff: nine scalars

`submit_bezier(i, j, p, q, dx, dy, dz, de, feedrate)` where `i, j` are
`Option<f64>` (`None` => chained, reflect previous exit control point). Python
resolves everything into machine-frame numbers; it never constructs a control
point as geometry.

### Control-point assembly (Rust, in `geometry` + `classify`)

`G5` is an XY-plane curve with Z interpolated linearly along the parameter.
With `start = P0` (known to the bridge) and `end = start + (dx, dy, dz)`:

- `P0 = start`
- `P1 = (start.x + I, start.y + J, start.z + dz/3)`
- `P2 = (end.x + P,   end.y + Q,   start.z + 2*dz/3)`
- `P3 = end`

The Z thirds keep Z linear across the segment, matching the offline
converter's behavior. `I J P Q` are XY-only offsets.

`distance_mm` for the segment must be the curve's **arc length** (not the
chord) so duration bookkeeping in `Motion` stays honest; the optimizer itself
always works from the true curve. Use the arc-length facility in the
nurbs/geometry layer.

### Chaining (smooth G5)

Marlin's headline G5 feature: omitting `I J` on a follow-on `G5` makes the
first control point the reflection of the previous G5's second control point
across the start point, giving C1 continuity.

State lives in the bridge, where the running position already lives:

- After building any G5 segment, store its absolute `P2`.
- On `submit_bezier` with `i,j = None`: `P1 = 2*P0 - prev_P2`.
- Any non-G5 move (`submit_move`, dwell, etc.) **clears** the stored `P2`;
  chaining only bridges consecutive G5s.

## Error handling (fail loudly)

- `P` or `Q` missing -> error (`G5 requires P and Q`). Validated in Python
  (cheap, has `gcmd.error`).
- `I` present without `J` or vice-versa -> error (`G5 I and J must both be
  present or both omitted`). Validated in Python.
- `X` or `Y` missing -> error (`G5 requires an X Y endpoint`). Validated in
  Python. (Strict per Marlin; no implicit "keep current axis".)
- `I J` omitted with no previous G5 in the chain -> error
  (`G5 without I J must follow another G5`). Validated in the **bridge**
  (only it holds the chain state); surfaced as a pyo3 error -> `gcmd.error`.
- Zero-displacement / zero-length curve -> reuse the existing
  `ClassifyError::ZeroDisplacement` path.

No silent recovery anywhere; every rejected G5 raises with a clear message.

## Bed mesh activation gate

The move-transform layer (`bed_mesh`, `skew_correction`, `bed_tilt`) warps
moves *above* the bridge and is not curve-correct — and bed mesh in
particular follows the surface by **splitting** a move into many short pieces
(`MoveSplitter`, `split_delta_z = 0.025`), which has not been extended to
Béziers. Rather than silently print a mesh-ignoring curve (or line), we fail
loud at activation.

Gate point: `bed_mesh.py` `set_mesh(self, mesh)` (the single chokepoint;
both profile `LOAD` and calibration's `probe_finalize` funnel through it).
When `mesh is not None`, raise a clear `gcode.error` *before* `self.z_mesh`
is assigned:

```
bed_mesh: activating a mesh is not supported yet — the move-transform layer
does not follow the surface under the new motion planner. Calibration and
profile saving still work; clearing a mesh (BED_MESH_CLEAR) is allowed.
```

Consequences, by design:

- `BED_MESH_PROFILE LOAD=<name>` -> rejected (activation).
- `BED_MESH_CALIBRATE` -> probes and computes the mesh, then surfaces the
  gate when it auto-applies. The measured profile can still be saved
  (`BED_MESH_PROFILE SAVE=<name>` / `SAVE_CONFIG`) because persistence does
  not route through `set_mesh`.
- `set_mesh(None)` (`BED_MESH_CLEAR`) -> allowed (deactivation).

This gate is removed when the move-transform layer is reworked for curves
(separate project).

## Testing

Rust (`cargo nextest run`):

- `geometry`: control-point assembly for explicit `I J P Q`; Z-thirds give
  linear Z; arc-length vs chord on a known curve; relocated
  `to_collinear_bezier` keeps its existing tests green.
- `motion-bridge` `classify`: builds a non-degenerate `CubicSegment` from
  control points; chaining reflection (`P1 = 2*P0 - prev_P2`); chain cleared
  by an intervening `submit_move`; chain-without-prior-G5 returns an error.

Python (`./scripts/ci.sh py`):

- `cmd_G5` parameter parsing and endpoint resolution under absolute/relative,
  `base_position`, extrude factor; `I J P Q` forwarded as raw offsets.
- Each error case raises (`gcmd.error`) with the specified message.
- Bed mesh gate: activating a mesh raises; `BED_MESH_CLEAR` and profile save
  do not.

End-to-end: a `G5` through the sim reaches the planner as a curved segment
(distinct from the collinear G1 path) and produces the expected step stream.

## Future work (not prerequisites)

- Live `G2` / `G3` / `G5.1` if ever wanted in-planner (extract `arc` /
  `degree_elev` from compat at that point).
- Mesh-aware curves: teach `MoveSplitter` to subdivide a Bézier (de Casteljau
  is exact in XY) with per-piece Z; extend the transform interface to carry a
  curve; remove the activation gate.
- Affine transforms (skew, bed_tilt) applied exactly to control points
  without splitting.
