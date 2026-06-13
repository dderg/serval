# Motion channels / multi-kinematics — PARKED (do not delete)

> **STATUS: DEFERRED — not an active spec, not for implementation.**
> Parked design note. **Do not delete during doc cleanup.** Resume the
> brainstorm only when a real machine needs it (a second independent gantry,
> IDEX, a toolchanger, or rotary A/B/C axes) — **deferred until Fable is back
> online.** This is the captured intent so the idea survives; the design is not
> finished.

## Where this builds from

- Foundational model: `docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md`.
- Shipped schema it extends (branch `e-follows-xy`): `[motor <name>]` (drive
  mandatory, short-name identity), the single `[kinematics]` section with
  `type:` + role→axis + role→motor bindings, `[axis <name>]` (range/homing,
  `motors:`, `follows:`), `[extruder] axis:`.
- **Part 1 (the config/vocabulary layer) is being done now** as its own plan:
  named `[kinematics <name>]`, `follows: <group-name>`, etc. This note is
  **Part 2** — the actual multi-planner engine, deferred.

## The idea (user's thoughts, captured verbatim-in-intent)

A printer is not one toolhead — it is a set of **planning groups ("motion
channels")**. Each `[kinematics <name>]` block is a group: its motors, its
axes, its transform, its own limits, **its own planner**. The toolhead is just
the most common group.

- **`type: follower` becomes its own kinematics file.** In Part 1 a follower is
  an `[axis e] follows: <group>` member of the followed group's single solve. In
  the Part 2 vision a follower is itself a `[kinematics <name>] type: follower`
  — a first-class kinematics, not an axis attribute. *(This supersedes Part 1's
  follower representation; reconcile when building Part 2.)*
- **When a move affects multiple kinematics together, all their limits are
  respected.** A move touching more than one group is constrained by every
  involved group's limit rows.
- **Coupled kinematics: one affects the other.** Where two kinematics are
  coupled (e.g. a follower coupled to a gantry, or two gantries sharing a
  constraint), the motion of one enters the other's solve.

## The model we brainstormed (for resuming)

- **Coordination = group membership.** Axes in the same group are jointly,
  time-optimally planned (shared jerk budget, one solve). Different groups are
  independent planners.
- **`G1` fans out across groups** it addresses. Default semantic: **start
  together, independent arrival** (each group plans its own slice under its own
  limits). Want two axes coordinated → put them in the same group. `G1 A10 F50`
  routes to whichever group owns axis `A`.
- **One pot per group.** The foundational spec's "one pot / one shared clock per
  move" becomes per-group: a `[limit]` may only name axes within one group;
  mandatory-coverage is per-group.
- **§4 post-processor folding is unchanged inside a group.** corexy stays
  corexy; the follower folds its post-chain (shaper window / PA derivative-gain)
  into *its group's* rows exactly as today. The grouping wraps the existing
  math; it does not redefine it.
- **No cross-group dependency cycles are possible** when `follows:` names one
  group (the follower is a member of it) — the dependency graph is trivially
  acyclic. The Part 2 "follower as its own kinematics that couples to a gantry"
  reintroduces a cross-group producer→consumer edge; that is the part that needs
  the most careful design.

## Open questions / weaknesses to resolve before building

1. **Primary group designation.** ~108 `lookup_object("toolhead")` consumers
   plus probe / bed_mesh / gcode_move / homing / GET_POSITION / skew / display
   assume one XYZ context. Need an explicit `primary:` marker (name stays
   arbitrary — do **not** let a group *named* "toolhead" be magic; that is the
   role-encoding-name lie we already killed with `[motor x]`). The
   `ToolheadShim` points at the designated primary.
2. **Single G-code stream ⇒ lockstep, not concurrency.** With one command
   stream, "start-together per G1" makes groups lockstep at every G1 boundary
   (the slower group gates the next line). That is fine for *one active gantry at
   a time* (toolchanger / IDEX handoff). **Two gantries doing different work
   simultaneously needs multiple command streams** — a much larger concept.
   Decide which is actually wanted.
3. **Cross-group timing.** N planners share one MCU step clock. Needs a shared
   time origin for the start barrier and coordinated lookahead/flush so queues
   don't starve. The MCU side is unaffected (per-stepper tapes already).
4. **Cross-group / coupled limits.** "A move affecting multiple kinematics
   respects all their limits" + "coupled kinematics affect each other" is the
   genuinely new solver work — it pushes past one-pot-per-group toward shared
   constraints across solves. This is the hard core of Part 2.
5. **G-code letter space.** Axis letters are globally unique across groups
   (X/Y/Z in one, A/B/C in another); the letter space is small (26 minus
   structural G5 words I/J/P/Q/F).

## Reuse / cost note

This extends; it does **not** rewrite. The `[motor]`/`[axis]`/`[kinematics]`
schema, the kinematics modules, the follower folding, and the post-processor
abstraction are all reused. The expensive, isolated part is the planner
orchestration (N planners on one clock, letter→group routing, cross-group
barriers, primary-group plumbing) — buildable later, on top of Part 1, without
un-building anything.
