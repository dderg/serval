use nurbs::bezier::BezierPiece;
use nurbs::VectorNurbs;

/// Velocity threshold below which a grid interval is treated as near-zero
/// (constant-position). Both endpoints must be below this threshold.
const NEAR_ZERO_V: f64 = 0.01;

/// Arc-length table accuracy for the s→u inversion (built once per segment).
pub(crate) const ARC_TABLE_TOL: f64 = 1e-9;
pub(crate) const ARC_TABLE_SAMPLES: usize = 16384;
/// |x'(u)| (mm per unit u) below this is a cusp — not a smooth segment.
const TANGENT_SPEED_FLOOR: f64 = 1e-6;
/// Per-piece time-domain position fit: degree and accuracy gate (geometry budget,
/// far below the 5 µm user fit_tolerance_mm).
const POS_FIT_DEGREE: usize = 9;
const POS_FIT_TOL_MM: f64 = 1e-6;
const POS_FIT_MAX_SUBDIV: usize = 8;

/// 5-point Gauss-Legendre nodes on [-1, 1].
const GL5_NODES: [f64; 5] = [
    -0.906_179_845_938_664,
    -0.538_469_310_105_683_1,
    0.0,
    0.538_469_310_105_683_1,
    0.906_179_845_938_664,
];
/// 5-point Gauss-Legendre weights.
const GL5_WEIGHTS: [f64; 5] = [
    0.236_926_885_056_189_1,
    0.478_628_670_499_366_5,
    0.568_888_888_888_888_9,
    0.478_628_670_499_366_5,
    0.236_926_885_056_189_1,
];

/// Arc length from 0 to `u`: the bracketing table node's stored arc length
/// (accurate to the table build tolerance) plus one 5-point GL integration
/// over the short residual interval `[u_node, u]`. O(5) curve evals.
fn arc_length_to_u(
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    deriv: &VectorNurbs<f64, 3>,
    u: f64,
) -> f64 {
    let u_arr = table.u();
    let s_arr = table.s();
    let u_c = u.clamp(u_arr[0], u_arr[u_arr.len() - 1]);
    let idx = match u_arr.binary_search_by(|x| x.partial_cmp(&u_c).unwrap()) {
        Ok(i) => i,
        Err(i) => i.saturating_sub(1),
    };
    let u_node = u_arr[idx];
    let s_node = s_arr[idx];
    let half = 0.5 * (u_c - u_node);
    if half <= 0.0 {
        return s_node;
    }
    let mid = 0.5 * (u_node + u_c);
    let mut seg = 0.0;
    for i in 0..GL5_NODES.len() {
        let t = mid + half * GL5_NODES[i];
        let d = nurbs::eval::vector_eval(deriv, t);
        seg += GL5_WEIGHTS[i] * (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    }
    s_node + seg * half
}

/// Invert arc length `s` to curve parameter `u`: seed from the table, then two
/// Newton steps using O(1) arc-length via table node + local GL residual.
/// Returns `ZeroTangent` at a cusp.
fn invert_s_to_u(
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    deriv: &VectorNurbs<f64, 3>,
    s: f64,
    index: usize,
) -> Result<f64, crate::ShapeError> {
    let s_clamped = s.clamp(0.0, table.s_max());
    let u_max = table.u_max();

    let mut u = nurbs::arc_length::param_from_arc_length(table, s_clamped);

    for _ in 0..2 {
        let d = nurbs::eval::vector_eval(deriv, u);
        let speed = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if speed < TANGENT_SPEED_FLOOR {
            return Err(crate::ShapeError::ZeroTangent { index, u });
        }
        let s_u = arc_length_to_u(table, deriv, u);
        u = (u - (s_u - s_clamped) / speed).clamp(0.0, u_max);
    }

    Ok(u)
}

/// Solve for power-basis coefficients c[k] of p(x) = Σ c[k] (x - origin)^k that
/// interpolate (nodes[i], vals[i]). Square system (len == degree+1), Gaussian
/// elimination with partial pivoting. Nodes must be distinct.
fn solve_power_basis(nodes: &[f64], vals: &[f64], origin: f64) -> Vec<f64> {
    let n = nodes.len();
    let mut a = vec![vec![0.0_f64; n + 1]; n];
    for i in 0..n {
        let dx = nodes[i] - origin;
        let mut p = 1.0;
        for k in 0..n {
            a[i][k] = p;
            p *= dx;
        }
        a[i][n] = vals[i];
    }
    for col in 0..n {
        let mut piv = col;
        for r in (col + 1)..n {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        a.swap(col, piv);
        let d = a[col][col];
        for r in 0..n {
            if r == col {
                continue;
            }
            let f = a[r][col] / d;
            for c in col..=n {
                a[r][c] -= f * a[col][c];
            }
        }
    }
    (0..n).map(|k| a[k][n] / a[k][k]).collect()
}

/// Fit position-over-time x(t) by sampling the EXACT curve at the inverted
/// parameter. Bisects the time interval on residual miss. Returns one or more
/// contiguous [x,y,z] power-basis pieces over s_of_t's time domain.
fn fit_position_of_t(
    curve: &VectorNurbs<f64, 3>,
    deriv: &VectorNurbs<f64, 3>,
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    s_of_t: &nurbs::bezier::BezierPiece<f64>,
    index: usize,
) -> Result<Vec<[nurbs::bezier::BezierPiece<f64>; 3]>, crate::ShapeError> {
    fit_position_of_t_rec(curve, deriv, table, s_of_t, index, 0)
}

fn fit_position_of_t_rec(
    curve: &VectorNurbs<f64, 3>,
    deriv: &VectorNurbs<f64, 3>,
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    s_of_t: &nurbs::bezier::BezierPiece<f64>,
    index: usize,
    depth: usize,
) -> Result<Vec<[nurbs::bezier::BezierPiece<f64>; 3]>, crate::ShapeError> {
    let t_lo = s_of_t.u_start;
    let t_hi = s_of_t.u_end;
    let n = POS_FIT_DEGREE + 1;

    let mut nodes_t = Vec::with_capacity(n);
    let mut vals: [Vec<f64>; 3] = [Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n)];
    let mid = 0.5 * (t_lo + t_hi);
    let half = 0.5 * (t_hi - t_lo);
    for i in 0..n {
        let theta = (i as f64) * std::f64::consts::PI / ((n - 1) as f64);
        let t = (mid + half * theta.cos()).clamp(t_lo, t_hi);
        let s = s_of_t.evaluate(t);
        let u = invert_s_to_u(table, deriv, s, index)?;
        let p = nurbs::eval::vector_eval(curve, u);
        nodes_t.push(t);
        for axis in 0..3 {
            vals[axis].push(p[axis]);
        }
    }

    let axes: [nurbs::bezier::BezierPiece<f64>; 3] = std::array::from_fn(|axis| nurbs::bezier::BezierPiece {
        u_start: t_lo,
        u_end: t_hi,
        coeffs: solve_power_basis(&nodes_t, &vals[axis], t_lo),
    });

    let mut max_err = 0.0_f64;
    let checks = 4 * n;
    for i in 0..=checks {
        let t = t_lo + (t_hi - t_lo) * (i as f64 / checks as f64);
        let s = s_of_t.evaluate(t);
        let u = invert_s_to_u(table, deriv, s, index)?;
        let truth = nurbs::eval::vector_eval(curve, u);
        for axis in 0..3 {
            max_err = max_err.max((axes[axis].evaluate(t) - truth[axis]).abs());
        }
    }

    if max_err <= POS_FIT_TOL_MM {
        return Ok(vec![axes]);
    }
    if depth >= POS_FIT_MAX_SUBDIV {
        return Err(crate::ShapeError::FitFailure {
            index,
            detail: nurbs::algebra::FitError::ToleranceNotReached {
                achieved_mm: max_err,
                at_degree: POS_FIT_DEGREE as u8,
            },
        });
    }
    let (left, right) = nurbs::bezier::split_piece_at(s_of_t, mid);
    let mut out = fit_position_of_t_rec(curve, deriv, table, &left, index, depth + 1)?;
    out.extend(fit_position_of_t_rec(curve, deriv, table, &right, index, depth + 1)?);
    Ok(out)
}

#[derive(Debug, Clone)]
pub struct SOfTPieces {
    pub pieces: Vec<BezierPiece<f64>>,
    pub near_zero: Vec<bool>,
    pub t_start: f64,
    pub t_end: f64,
    #[allow(dead_code)]
    pub total_duration: f64,
}

pub fn build_s_of_t_pieces(profile: &temporal::TopProfile, t_global_offset: f64) -> SOfTPieces {
    let n = profile.samples.len();
    assert!(n >= 2, "TopProfile must have at least 2 samples");

    let mut pieces = Vec::with_capacity(n - 1);
    let mut near_zero = Vec::with_capacity(n - 1);
    let mut t_cursor = t_global_offset;

    for k in 0..n - 1 {
        let s_k = profile.samples[k].s;
        let s_k1 = profile.samples[k + 1].s;
        let v_k = profile.samples[k].v;
        let v_k1 = profile.samples[k + 1].v;
        let b_k = profile.samples[k].b;
        let b_k1 = profile.samples[k + 1].b;

        let ds = s_k1 - s_k;

        let is_near_zero = v_k < NEAR_ZERO_V && v_k1 < NEAR_ZERO_V;

        if is_near_zero {
            let dt = ds / NEAR_ZERO_V;
            let t_start = t_cursor;
            let t_end = t_cursor + dt;

            pieces.push(BezierPiece {
                u_start: t_start,
                u_end: t_end,
                coeffs: vec![s_k, 0.0, 0.0],
            });
            near_zero.push(true);
            t_cursor = t_end;
        } else {
            let v_sum = v_k + v_k1;
            let dt = if v_sum > 1e-12 {
                2.0 * ds / v_sum
            } else {
                ds / NEAR_ZERO_V
            };

            let a_k = if ds.abs() > 1e-15 {
                (b_k1 - b_k) / (2.0 * ds)
            } else {
                0.0
            };

            let t_start = t_cursor;
            let t_end = t_cursor + dt;

            pieces.push(BezierPiece {
                u_start: t_start,
                u_end: t_end,
                coeffs: vec![s_k, v_k, a_k / 2.0],
            });
            near_zero.push(false);
            t_cursor = t_end;
        }
    }

    let t_start = t_global_offset;
    let t_end = t_cursor;
    SOfTPieces {
        pieces,
        near_zero,
        t_start,
        t_end,
        total_duration: t_end - t_start,
    }
}

pub fn compose_segment(
    curve: &nurbs::VectorNurbs<f64, 3>,
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    s_pieces: &SOfTPieces,
    fit_tolerance: f64,
) -> Result<Vec<[BezierPiece<f64>; 3]>, crate::ShapeError> {
    let mut result = Vec::with_capacity(s_pieces.pieces.len());

    for (k, s_piece) in s_pieces.pieces.iter().enumerate() {
        if s_pieces.near_zero[k] {
            let s_k = s_piece.coeffs[0];
            let u_k = nurbs::arc_length::param_from_arc_length(table, s_k);
            let pos = nurbs::eval::vector_eval(curve, u_k);

            let axes: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| BezierPiece {
                u_start: s_piece.u_start,
                u_end: s_piece.u_end,
                coeffs: vec![pos[axis]],
            });
            result.push(axes);
        } else {
            let s_lo = s_piece.evaluate(s_piece.u_start);
            let s_hi = s_piece.evaluate(s_piece.u_end);
            let s_hi_clamped = s_hi.min(table.s_max());
            let s_lo_safe = s_lo.max(0.0);

            if s_hi_clamped - s_lo_safe < 1e-15 {
                let u_k = nurbs::arc_length::param_from_arc_length(table, s_lo_safe);
                let pos = nurbs::eval::vector_eval(curve, u_k);
                let axes: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| BezierPiece {
                    u_start: s_piece.u_start,
                    u_end: s_piece.u_end,
                    coeffs: vec![pos[axis]],
                });
                result.push(axes);
                continue;
            }

            let x_of_s: [BezierPiece<f64>; 3] = nurbs::algebra::fit_x_to_arc_length_piece::<3>(
                curve,
                table,
                s_lo_safe,
                s_hi_clamped,
                3,
                5,
                fit_tolerance,
            )
            .map_err(|detail| crate::ShapeError::FitFailure { index: k, detail })?;

            let s_piece_adjusted =
                if (s_lo_safe - s_lo).abs() > 1e-15 || (s_hi_clamped - s_hi).abs() > 1e-15 {
                    let mut adj = s_piece.clone();
                    adj.coeffs[0] = s_lo_safe;
                    let dt = adj.u_end - adj.u_start;
                    if dt > 1e-15 && adj.coeffs.len() >= 3 {
                        let v_k = adj.coeffs[1];
                        adj.coeffs[2] = (s_hi_clamped - s_lo_safe - v_k * dt) / (dt * dt);
                    }
                    adj
                } else {
                    s_piece.clone()
                };

            let outer_refs: [&BezierPiece<f64>; 3] = [&x_of_s[0], &x_of_s[1], &x_of_s[2]];

            let composed =
                nurbs::algebra::compose_vector_piece::<3>(&outer_refs, &s_piece_adjusted)
                    .map_err(|detail| crate::ShapeError::Algebra { index: k, detail })?;

            result.push(composed);
        }
    }

    Ok(result)
}

#[cfg(test)]
mod tests;
