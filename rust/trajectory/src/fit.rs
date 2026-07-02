use nurbs::bezier::BezierPiece;
use nurbs::ScalarNurbs;

const HERMITE_REFIT_MAX_SUBDIVISIONS: usize = 8;
const MIN_HERMITE_PIECE_DURATION: f64 = 1e-12;
const PHASE1_HERMITE_DEGREE: u8 = 4;
const PHASE2_BOTH_ENDS_PINNED_HERMITE_DEGREE: u8 = 5;
const LAST_PIECE_TIME_MATCH_EPSILON: f64 = 1e-12;

#[derive(Debug, Clone)]
pub struct FittedSegment {
    pub axes: [ScalarNurbs<f64>; 3],
    pub t_start: f64,
    pub t_end: f64,
}

#[derive(Debug, thiserror::Error)]
pub enum FitError {
    #[error("fit failure: {0:?}")]
    Nurbs(nurbs::algebra::FitError),
    #[error("empty segment buffer")]
    EmptySegments,
}

impl From<nurbs::algebra::FitError> for FitError {
    fn from(value: nurbs::algebra::FitError) -> Self {
        Self::Nurbs(value)
    }
}

pub fn fit_and_split(
    composed: &[[BezierPiece<f64>; 3]],
    tolerance: f64,
    start_d2_override: Option<[f64; 3]>,
) -> Result<FittedSegment, FitError> {
    use nurbs::bezier::bezier_pieces_to_nurbs;

    if composed.is_empty() {
        return Err(FitError::EmptySegments);
    }

    let t_start = composed[0][0].u_start;
    let t_end = composed.last().unwrap()[0].u_end;

    let fit_input = nondegenerate_composed_pieces(composed)?;

    let d2_start =
        start_d2_override.unwrap_or_else(|| boundary_second_derivative_start(&fit_input));
    let d2_end = boundary_second_derivative_end(&fit_input);

    let mut fitted = fit_hermite_c2_adaptive(&fit_input, tolerance, d2_start, d2_end)?;

    let uniform_degree_required_by_bezier_pieces_to_nurbs = fitted
        .iter()
        .flat_map(|axis_pieces| axis_pieces.iter().map(|p| p.coeffs.len().saturating_sub(1)))
        .max()
        .unwrap_or(usize::from(PHASE1_HERMITE_DEGREE));
    for axis_pieces in fitted.iter_mut() {
        for piece in axis_pieces.iter_mut() {
            while piece.coeffs.len() <= uniform_degree_required_by_bezier_pieces_to_nurbs {
                piece.coeffs.push(0.0);
            }
        }
    }

    let axes = [
        bezier_pieces_to_nurbs(&fitted[0]),
        bezier_pieces_to_nurbs(&fitted[1]),
        bezier_pieces_to_nurbs(&fitted[2]),
    ];

    Ok(FittedSegment {
        axes,
        t_start,
        t_end,
    })
}

fn boundary_second_derivative_start(composed: &[[BezierPiece<f64>; 3]]) -> [f64; 3] {
    std::array::from_fn(|axis| {
        let piece = &composed[0][axis];
        piece
            .differentiate()
            .differentiate()
            .evaluate(piece.u_start)
    })
}

fn boundary_second_derivative_end(composed: &[[BezierPiece<f64>; 3]]) -> [f64; 3] {
    std::array::from_fn(|axis| {
        let piece = composed.last().unwrap()[axis].clone();
        piece.differentiate().differentiate().evaluate(piece.u_end)
    })
}

fn nondegenerate_composed_pieces(
    composed: &[[BezierPiece<f64>; 3]],
) -> Result<Vec<[BezierPiece<f64>; 3]>, FitError> {
    let filtered: Vec<[BezierPiece<f64>; 3]> = composed
        .iter()
        .filter(|piece_set| {
            let duration = piece_set[0].u_end - piece_set[0].u_start;
            duration.is_finite() && duration > MIN_HERMITE_PIECE_DURATION
        })
        .cloned()
        .collect();

    if filtered.is_empty() {
        return Err(FitError::Nurbs(nurbs::algebra::FitError::DegenerateInput {
            reason: "fit_and_split: no non-degenerate Hermite input pieces",
        }));
    }

    Ok(filtered)
}

fn fit_hermite_c2_adaptive(
    composed: &[[BezierPiece<f64>; 3]],
    tolerance: f64,
    d2_start: [f64; 3],
    d2_end: [f64; 3],
) -> Result<[Vec<BezierPiece<f64>>; 3], nurbs::algebra::FitError> {
    use nurbs::algebra::{fit_hermite_c1_clamped, FitError};

    let mut refined = composed.to_vec();
    let mut fitted: [Vec<BezierPiece<f64>>; 3] = std::array::from_fn(|_| Vec::new());

    for depth in 0..=HERMITE_REFIT_MAX_SUBDIVISIONS {
        let phase1_start_pin_only = None;
        match fit_hermite_c1_clamped::<3>(
            &refined,
            tolerance,
            PHASE1_HERMITE_DEGREE,
            d2_start,
            phase1_start_pin_only,
        ) {
            Ok(f) => {
                fitted = f;
                break;
            }
            Err(err @ FitError::ToleranceNotReached { .. }) => {
                if depth == HERMITE_REFIT_MAX_SUBDIVISIONS {
                    return Err(err);
                }
                refined = split_composed_midpoints(&refined)?;
            }
            Err(err) => return Err(err),
        }
    }

    refit_last_piece_with_end_pin(&mut fitted, &refined, tolerance, d2_end)?;

    Ok(fitted)
}

fn refit_last_piece_with_end_pin(
    fitted: &mut [Vec<BezierPiece<f64>>; 3],
    refined: &[[BezierPiece<f64>; 3]],
    tolerance: f64,
    d2_end: [f64; 3],
) -> Result<(), nurbs::algebra::FitError> {
    use nurbs::algebra::{fit_hermite_c1_clamped, FitError};

    let representative_axis = 0;
    let last_out_start = match fitted[representative_axis].last() {
        Some(p) => p.u_start,
        None => return Ok(()),
    };

    let last_refined: Vec<[BezierPiece<f64>; 3]> = refined
        .iter()
        .filter(|ps| ps[0].u_start >= last_out_start - LAST_PIECE_TIME_MATCH_EPSILON)
        .cloned()
        .collect();

    if last_refined.is_empty() {
        return Ok(());
    }

    let last_d2_start: [f64; 3] = std::array::from_fn(|axis| {
        let piece = &last_refined[0][axis];
        piece
            .differentiate()
            .differentiate()
            .evaluate(piece.u_start)
    });

    let mut refined_last = last_refined;
    let mut last_fitted: Option<[Vec<BezierPiece<f64>>; 3]> = None;

    for depth in 0..=HERMITE_REFIT_MAX_SUBDIVISIONS {
        match fit_hermite_c1_clamped::<3>(
            &refined_last,
            tolerance,
            PHASE2_BOTH_ENDS_PINNED_HERMITE_DEGREE,
            last_d2_start,
            Some(d2_end),
        ) {
            Ok(f) => {
                last_fitted = Some(f);
                break;
            }
            Err(err @ FitError::ToleranceNotReached { .. }) => {
                if depth == HERMITE_REFIT_MAX_SUBDIVISIONS {
                    return Err(err);
                }
                refined_last = split_composed_midpoints(&refined_last)?;
            }
            Err(err) => return Err(err),
        }
    }

    if let Some(new_last) = last_fitted {
        for axis in 0..3 {
            let phase1_last_piece_end_free = fitted[axis].pop();
            debug_assert!(phase1_last_piece_end_free.is_some());
            fitted[axis].extend(new_last[axis].iter().cloned());
        }
    }

    Ok(())
}

fn split_composed_midpoints(
    composed: &[[BezierPiece<f64>; 3]],
) -> Result<Vec<[BezierPiece<f64>; 3]>, nurbs::algebra::FitError> {
    use nurbs::algebra::FitError;
    use nurbs::bezier::split_piece_at;

    let mut refined = Vec::with_capacity(composed.len() * 2);

    for piece_set in composed {
        let u_start = piece_set[0].u_start;
        let u_end = piece_set[0].u_end;
        let duration = u_end - u_start;

        if !duration.is_finite() || duration <= MIN_HERMITE_PIECE_DURATION {
            refined.push(piece_set.clone());
            continue;
        }

        let u_mid = 0.5 * (u_start + u_end);

        if !u_mid.is_finite() || u_mid <= u_start || u_mid >= u_end {
            return Err(FitError::DegenerateInput {
                reason: "fit_and_split: cannot split degenerate Hermite input piece",
            });
        }

        let left: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| {
            let (left, _) = split_piece_at(&piece_set[axis], u_mid);
            left
        });
        let right: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| {
            let (_, right) = split_piece_at(&piece_set[axis], u_mid);
            right
        });

        refined.push(left);
        refined.push(right);
    }

    Ok(refined)
}

#[cfg(test)]
mod tests;
