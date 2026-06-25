---
id: SPEC-motion-viz-validation
companions:
  - snapshot-contents.md
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only.

# Motion-planner snapshot validation harness

## Why

A pain to solve, ahead of a mandate. We validate motion-planner and arc-fit changes by hand-rendering the `viz_pipeline.py` panels (path, velocity, acceleration, jerk) and eyeballing them. That manual loop has been genuinely productive — it surfaced real defects (acceleration overshooting `a_max` when the jerk limit binds on a curved segment; well-formed tangent fillets left un-eased with a raw curvature step) that the Rust unit suite did not catch. But eyeballing does not scale, leaves no regression guard, and depends on a human remembering to look. We are about to make more fitter/planner changes (the tangent-fillet refit, the jerk-on-the-disk-rail coupling), and we want a net under them first. The model is **snapshot testing, like UI snapshot libraries**: record a case's full trajectory output as a checked-in baseline, fail when a later run deviates, show the developer before/after for every changed snapshot, and only re-baseline on explicit approval.

## Capabilities

- id: CAP-1
  intent: A developer adds a case as a G-code snippet plus printer config (max_velocity / max_accel / square_corner_velocity / max_jerk and arc_fit knobs) and runs it through the real planner + fitter to a deterministic trajectory output.
  success: A case is as cheap to add as dropping a G-code file (plus config) into a folder; running it reads the planner snapshot (`kin_*`, `fitted_segments`, `traversal_time_s`) through the existing `_motion_engine` path — not a reimplemented model — and emits one canonical, deterministic raw result.

- id: CAP-2
  intent: A case's full raw trajectory is recorded as a checked-in baseline, and a run fails when the new result deviates from that baseline — including a newly-added case, which has no baseline yet and is pending until approved.
  success: On an unchanged planner every case re-runs green; any change to a case's trajectory turns that case red; a brand-new case is red/pending until its first baseline is approved — there are no hand-authored numeric thresholds and no auto-blessed baselines, new or changed.

- id: CAP-3
  intent: A review interface presents before/after for every changed or pending snapshot and lets the developer accept them — individually or all at once — with acceptance writing the new raw snapshots to disk to be committed.
  success: A run with deviations opens a review showing each before/after; an accept-all action writes the new baselines to disk (staged for commit) and the cases go green; nothing is re-baselined without that explicit accept.

- id: CAP-4
  intent: The review starts from the rendered panel PNGs, but because the baseline is the full raw trajectory, richer comparison views can be added later without re-recording any baseline.
  success: The first cut renders before/after as the existing `viz_pipeline` panels; the same stored raw trajectories support a later UI that highlights the changed region of the path, zooms, and diffs profiles — and PNGs are not required for the baseline to exist.

- id: CAP-5
  intent: A case fully specifies the planner configuration it runs under — velocity/accel/scv/jerk limits and arc-fit enable + knobs — including limit changes that take effect partway through the case.
  success: A case can disable arc-fit, or lower the velocity limit before a chosen move, and the recorded trajectory reflects exactly that; two cases with identical G-code but different config (e.g. arc-fit on vs off) produce different baselines.

## Constraints

- Store the **full raw trajectory** as the baseline (the sampled result the panels render from), not a digest — so any change is caught and richer diff views can be built on the same artifact later.
- Build on existing tooling: `scripts/viz_pipeline.py`, the `_motion_engine` snapshot fields, and the `seam_harness` / `fit_chain` / `plan_velocity` APIs. Do not reimplement the planner or re-derive the trajectory from an independent kinematic model.
- The recorded acceleration and jerk are the exact quantities the viz panels display (the user trusts those numbers and found the overshoot with them): acceleration is the disk magnitude `√(a_t² + a_n²)` from the same samples, not a per-axis or tangential-only value. See `snapshot-contents.md`.
- Deterministic, reproducible baseline: a re-run of an unchanged planner must reproduce the raw result, so every red is a real output change. Because the baseline is raw (not tolerance-softened), this requires a reproducible generate/compare environment — see open question.
- Human-in-the-loop re-baseline: a baseline — new or changed — is written only on an explicit accept; a deviation never auto-blesses a baseline.
- Lands before the next fitter/planner change — it is the regression net for that work, not a follow-up to it.

## Non-goals

- No hand-authored pass/fail thresholds. The raw baseline characterizes current behavior; a deviation is flagged for review and the human judges good-vs-bad at accept time — this harness is a change detector, not a correctness oracle.
- The PNG (and any later review UI) is a **view** rendered from the raw trajectory, not the baseline itself; pixel-diffing the PNG is not the gate — comparison is on the recorded raw trajectory.
- Not a replacement for the Rust unit suite (`cargo nextest`) or the `seam-continuity-test-platform` harness (narrow C0/C1 commit-seam checks); this sits at the whole-trajectory layer, complementary to both.
- Not bench/hardware validation — offline planner output only.

## Success signal

A motion change that alters any case's trajectory — reintroduces the on-curve acceleration overshoot, stops easing a tangent fillet, shatters an arc into a clothoid forest, inflates traversal time, or fixes any of these — turns that case red and surfaces a before/after view, so the change is reviewed and explicitly accepted into a new committed baseline instead of slipping through unseen.

## Assumptions

- The planner is deterministic, so a recorded raw baseline is stable run-to-run; existing determinism tests support this. Finite-difference jerk spikes at clothoid↔arc seams are part of the recorded result and stable — they do not cause false failures, and a change to them is itself a reviewable signal.
- The planner snapshot already exposes everything the raw result needs (confirmed in `viz_pipeline.py`: `kin_s`, `kin_v`, `kin_kappa`, `kin_heading_{x,y}`, `fitted_segments`, `blended_corners`, `chain_fits`, `traversal_time_s`).
- "Pieces" means the fitted geometry segments (Line / Arc / Clothoid) the fitter emits, counted as in the `analyze_arc_fit` census / `fitted_segments`.

## Open Questions

- Per-case config syntax (CAP-5): the knobs that don't live in G-code — velocity/accel/scv/jerk limits, arc-fit enable + knobs, and any mid-case limit change — need a home. Options: a per-folder/per-case `printer.cfg` (matches the existing viz `-c` flag and Klipper's config model), in-G-code commands (`M204` / `SET_VELOCITY_LIMIT` for limits plus a custom directive to toggle arc-fit), or a sidecar case file. Mid-case changes lean toward G-code commands; static config leans toward `printer.cfg`; the two can combine (a base `printer.cfg` per folder + in-line G-code overrides).
- Reproducibility of the raw baseline across environments: is the planner bit-identical between the generation host and wherever the suite runs (e.g. a dev Mac vs the Pi vs CI)? If not bit-reproducible, raw comparison will false-fail on sub-ulp drift, so the harness must pin a canonical environment to generate and compare baselines (or define the one tolerance that still counts as "no change"). Defer until we get to it.
