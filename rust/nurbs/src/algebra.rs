use crate::AlgebraError;
use crate::bezier::binomial;

pub fn scalar_multiply(curve: &crate::ScalarNurbs, scalar: f64) -> crate::ScalarNurbs {
    let new_cps: Vec<f64> = curve.control_points().iter().map(|c| *c * scalar).collect();
    crate::ScalarNurbs::try_new(curve.degree(), curve.knots().to_vec(), new_cps)
        .expect("scalar_multiply preserves invariants")
}

pub fn add(
    a: &crate::ScalarNurbs,
    b: &crate::ScalarNurbs,
) -> Result<crate::ScalarNurbs, AlgebraError> {
    if a.degree() != b.degree() {
        return Err(AlgebraError::KnotMismatch);
    }
    if a.knots() != b.knots() {
        return Err(AlgebraError::KnotMismatch);
    }
    let new_cps: Vec<f64> = a
        .control_points()
        .iter()
        .zip(b.control_points().iter())
        .map(|(x, y)| *x + *y)
        .collect();
    crate::ScalarNurbs::try_new(a.degree(), a.knots().to_vec(), new_cps)
        .map_err(|_| AlgebraError::KnotMismatch)
}

pub fn add_with_knot_union(
    a: &crate::ScalarNurbs,
    b: &crate::ScalarNurbs,
) -> Result<crate::ScalarNurbs, AlgebraError> {
    if a.degree() != b.degree() {
        return Err(AlgebraError::KnotMismatch);
    }

    if a.knots() == b.knots() {
        return add(a, b);
    }

    let a_pieces = crate::bezier::extract_bezier_pieces(a);
    let b_pieces = crate::bezier::extract_bezier_pieces(b);

    if a_pieces.is_empty() || b_pieces.is_empty() {
        return Err(AlgebraError::KnotMismatch);
    }

    let a_start = a_pieces[0].u_start;
    let a_end = a_pieces[a_pieces.len() - 1].u_end;
    let b_start = b_pieces[0].u_start;
    let b_end = b_pieces[b_pieces.len() - 1].u_end;
    let domain_tol = 1e-12;
    if (a_start - b_start).abs() > domain_tol || (a_end - b_end).abs() > domain_tol {
        return Err(AlgebraError::SupportMismatch);
    }

    let breakpoints = union_breakpoints(&a_pieces, &b_pieces);
    let a_refined = refine_pieces_to_breakpoints(&a_pieces, &breakpoints);
    let b_refined = refine_pieces_to_breakpoints(&b_pieces, &breakpoints);

    assert_eq!(
        a_refined.len(),
        b_refined.len(),
        "add_with_knot_union: refine produced mismatched piece counts \
         (a_refined={}, b_refined={}); this is an internal invariant violation",
        a_refined.len(),
        b_refined.len(),
    );

    let sum_pieces: Vec<crate::bezier::BezierPiece> = a_refined
        .iter()
        .zip(b_refined.iter())
        .map(|(ap, bp)| {
            assert_eq!(
                ap.coeffs.len(),
                bp.coeffs.len(),
                "add_with_knot_union: CP count mismatch after union refine \
                 (ap.coeffs={}, bp.coeffs={})",
                ap.coeffs.len(),
                bp.coeffs.len(),
            );
            let coeffs: Vec<f64> = ap
                .coeffs
                .iter()
                .zip(bp.coeffs.iter())
                .map(|(ac, bc)| *ac + *bc)
                .collect();
            crate::bezier::BezierPiece {
                u_start: ap.u_start,
                u_end: ap.u_end,
                coeffs,
            }
        })
        .collect();

    Ok(crate::bezier::bezier_pieces_to_nurbs(&sum_pieces))
}

#[derive(Debug, Clone)]
pub struct PiecewisePolynomialKernel {
    pub pieces: Vec<crate::bezier::BezierPiece>,
}

impl PiecewisePolynomialKernel {
    pub fn single_poly(coeffs: Vec<f64>, support: (f64, f64)) -> Self {
        let piece = crate::bezier::BezierPiece {
            u_start: support.0,
            u_end: support.1,
            coeffs,
        };
        Self {
            pieces: vec![piece],
        }
    }

    #[allow(clippy::needless_pass_by_value)]
    pub fn single_poly_from_absolute(coeffs: Vec<f64>, support: (f64, f64)) -> Self {
        let shifted = absolute_to_pascal_shift(&coeffs, support.0);
        Self::single_poly(shifted, support)
    }

    pub fn support(&self) -> (f64, f64) {
        (
            self.pieces.first().unwrap().u_start,
            self.pieces.last().unwrap().u_end,
        )
    }

    pub fn from_pieces(pieces: Vec<crate::bezier::BezierPiece>) -> Result<Self, AlgebraError> {
        if pieces.is_empty() {
            return Err(AlgebraError::SupportMismatch);
        }
        for w in pieces.windows(2) {
            if w[0].u_end != w[1].u_start {
                return Err(AlgebraError::SupportMismatch);
            }
        }
        Ok(Self { pieces })
    }
}

fn union_breakpoints(
    a: &[crate::bezier::BezierPiece],
    b: &[crate::bezier::BezierPiece],
) -> Vec<f64> {
    let mut breaks: Vec<f64> = Vec::new();
    let push_unique = |u: f64, breaks: &mut Vec<f64>| {
        if !breaks.contains(&u) {
            breaks.push(u);
        }
    };
    for piece in a {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    for piece in b {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    breaks.sort_by(|x, y| x.total_cmp(y));
    breaks
}

fn refine_pieces_to_breakpoints(
    pieces: &[crate::bezier::BezierPiece],
    breakpoints: &[f64],
) -> Vec<crate::bezier::BezierPiece> {
    let mut result: Vec<crate::bezier::BezierPiece> = Vec::new();
    for piece in pieces {
        let mut current = piece.clone();
        let mut interior: Vec<f64> = breakpoints
            .iter()
            .filter(|&&b| b > current.u_start && b < current.u_end)
            .copied()
            .collect();
        interior.sort_by(|x, y| x.total_cmp(y));
        for u in interior {
            let (left, right) = crate::bezier::split_piece_at(&current, u);
            result.push(left);
            current = right;
        }
        result.push(current);
    }
    result
}

fn absolute_to_pascal_shift(absolute: &[f64], shift: f64) -> Vec<f64> {
    let d = absolute.len() - 1;
    let mut out = vec![0.0; d + 1];
    let mut shift_pow = vec![1.0; d + 1];
    for k in 1..=d {
        shift_pow[k] = shift_pow[k - 1] * shift;
    }
    for n in 0..=d {
        for k in 0..=n {
            let bin = binomial(n, k) as f64;
            out[k] += absolute[n] * bin * shift_pow[n - k];
        }
    }
    out
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests;
