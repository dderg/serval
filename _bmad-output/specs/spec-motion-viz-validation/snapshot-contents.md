# Snapshot contents

What a case's recorded baseline captures, and the exact derivation of each value from the planner snapshot. Everything comes from the same fields `viz_pipeline.py` reads, after its `np.diff(s) > 1e-9` dedup mask — so the recorded result and the rendered panels agree by construction. These are **recorded**, not asserted against hand-set bounds: the test compares a new result to this baseline and flags any deviation (CAP-2).

Snapshot fields read: `kin_s`, `kin_v`, `kin_heading_x`, `kin_heading_y`, `kin_kappa`, `fitted_segments`, `traversal_time_s`.

| Captured | Derivation | Notes |
|---|---|---|
| velocity profile `v(s)` | `kin_v` over `kin_s` | The velocity panel's series; envelope (min/max) falls out of it. |
| acceleration `a(s)` | `a_scalar = √(a_t² + a_n²)`, `a_t = v·(dv/ds)`, `a_n = v²·κ` | **The load-bearing one.** Disk magnitude as the viz plots it, not per-axis or tangential-only — this is the quantity that exposed the on-curve overshoot, so it must be captured identically (`a_t` from `np.gradient(v, s)`, `a_n` from `v²·κ`, magnitude in quadrature). |
| jerk `j(s)` | `j_scalar` from the panel | Carries finite-difference spikes at clothoid↔arc seams (unsigned `Arc::kappa` vs signed `Clothoid::kappa`). Recorded as-is; it is deterministic, so it never false-fails, and a change to it is a legitimate reviewable signal. |
| piece census | count `fitted_segments` by type → `{line, arc, clothoid}` | Behavioral fingerprint: a fillet that stops being eased drops Clothoid count; a shattered arc spikes Clothoid and zeroes Arc. |
| `total_time_s` | `traversal_time_s` | Throughput tripwire — an inflation is a slower trajectory and must be reviewed. |
| path `(x, y)` | `kin_v`·heading integrated, or the fitted-segment polyline | The path panel's series; lets the before/after PNG show geometry change, not just kinematics. |

The recorded result is the **full raw trajectory** — every sample of the above, canonically serialized, not a digest — so any change is caught and a later UI can highlight/zoom the changed region from the same artifact. The before/after view (CAP-3) is rendered from two such serializations — the stored baseline and the new result — starting with the `viz_pipeline` panels the developer already reads.
