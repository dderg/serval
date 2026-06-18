# Deferred Work

Findings surfaced during review that are real but out of the originating story's scope.

## From spec-motion-1-typed-segment-ir (review 2026-06-18)

- **Anchor-data finiteness validation (step-2 lowering).** `Arc::origin` / `Arc::start_angle` and `Clothoid::start_pose` are accepted without `is_finite` checks, and `Line::try_new` rejects only `len == 0.0` (a NaN coordinate yields `len = NaN`, which is neither `== 0.0` nor `> 0.0`, so it slips through). This does not affect the step-1 κ-space contract — none of those fields feed any `CurvatureProfile` method, and the spec explicitly stores anchors as inert data evaluated only at lowering. Validate anchor finiteness where position is first evaluated (step 2), or fold it into the lowering constructor. **(RESOLVED in step-2 lowering: `lower_constant_speed` rejects non-finite Line start/end, Arc origin+start_angle, and Clothoid start_pose with `InvalidLowering`; paired test `non_finite_anchor_rejected`.)**

## From spec-motion-2-execution-lowering (review 2026-06-18)

- **Small-σ Fresnel precision cliff.** `clothoid_offset`'s σ=0 closed form fires only at exactly `sigma == 0.0`; for tiny nonzero σ the completion-of-square produces large Fresnel arguments whose differenced ~0.5-magnitude values cancel (error grows to ~3e-9 at σ=1e-9). Sub-physical (picometer-scale) and degenerate-arc-like, so not patched now. Add a tolerance-band fallback to the arc/line limit if the fitter ever emits near-arc clothoids.
- **`LoweredSample.followers` drops `axis_index`.** Follower samples are a positional `Vec<f64>` of `ratio·s` in `seg.followers` order; `FollowerDemand.axis_index` is not carried. Fine for observability (zip with `seg.followers`), but revisit the struct shape when a real follower consumer (front-end, step 3+) needs the axis mapping.

- **Signed-curvature convention (step-2 heading).** `Arc::kappa` returns `+1/radius` regardless of sweep sign, while `Clothoid::kappa` is signed (`κ₀ + σs`). The velocity cap consumes `|κ|` via `kappa_peak`, so step 1 is correct. If a downstream consumer needs signed curvature to distinguish turn direction (heading integration), settle the sign convention when that consumer lands.
