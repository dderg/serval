# Deferred Work

Findings surfaced during review that are real but out of the originating story's scope.

## From spec-motion-1-typed-segment-ir (review 2026-06-18)

- **Anchor-data finiteness validation (step-2 lowering).** `Arc::origin` / `Arc::start_angle` and `Clothoid::start_pose` are accepted without `is_finite` checks, and `Line::try_new` rejects only `len == 0.0` (a NaN coordinate yields `len = NaN`, which is neither `== 0.0` nor `> 0.0`, so it slips through). This does not affect the step-1 κ-space contract — none of those fields feed any `CurvatureProfile` method, and the spec explicitly stores anchors as inert data evaluated only at lowering. Validate anchor finiteness where position is first evaluated (step 2), or fold it into the lowering constructor.

- **Signed-curvature convention (step-2 heading).** `Arc::kappa` returns `+1/radius` regardless of sweep sign, while `Clothoid::kappa` is signed (`κ₀ + σs`). The velocity cap consumes `|κ|` via `kappa_peak`, so step 1 is correct. If a downstream consumer needs signed curvature to distinguish turn direction (heading integration), settle the sign convention when that consumer lands.
