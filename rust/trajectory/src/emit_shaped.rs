use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};
use nurbs::eval::eval as nurbs_eval;
use nurbs::ScalarNurbs;

use crate::beta::kernel_half_support;
use crate::fit::FittedSegment;
use crate::odometer::Odometer;
use crate::pad::pad_segment_axis_with_history;
use crate::post_processor::{apply_derivative_gain, AxisChainSet, CompiledChain};
use crate::smooth_fit::fit_c2_cubic_with_bc;
use crate::{ShapeError, ShapedSegment};

const SMOOTH_FIT_TOLERANCE_MM: f64 = 5.0e-3;
const ODOMETER_MIN_INTERVALS: usize = 16;
const CONSTANT_AXIS_EPS: f64 = 1e-12;

#[derive(Debug, Clone, Default)]
pub struct PerAxisHistory<'a> {
    pub axes: Vec<&'a [BezierPiece<f64>]>,
}

impl PerAxisHistory<'_> {
    #[must_use]
    pub const fn empty() -> Self {
        Self { axes: Vec::new() }
    }

    fn axis(&self, axis: usize) -> &[BezierPiece<f64>] {
        self.axes.get(axis).copied().unwrap_or(&[])
    }
}

#[derive(Debug)]
pub struct ShapeEmission {
    pub segments: Vec<ShapedSegment>,
    pub follower_inputs: Vec<Vec<BezierPiece<f64>>>,
}

#[derive(Debug, Clone)]
pub struct EmitSegmentMeta {
    pub followers: Vec<geometry::segment::FollowerDemand>,
}

pub fn emit_shaped(
    planned: &[FittedSegment],
    meta: &[EmitSegmentMeta],
    chains: &AxisChainSet,
    history: &PerAxisHistory<'_>,
    follower_anchor: &FollowerAnchor<'_>,
    batch_t_start: f64,
    batch_t_end: f64,
) -> Result<ShapeEmission, ShapeError> {
    emit_shaped_with_left_bc(
        planned,
        meta,
        chains,
        history,
        follower_anchor,
        batch_t_start,
        batch_t_end,
        &[],
    )
}

#[derive(Debug, Clone, Copy)]
pub struct FollowerAnchor<'a> {
    pub t: f64,
    pub values: &'a [f64],
}

impl FollowerAnchor<'_> {
    #[must_use]
    pub const fn none() -> Self {
        FollowerAnchor {
            t: f64::NEG_INFINITY,
            values: &[],
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn emit_shaped_with_left_bc(
    planned: &[FittedSegment],
    meta: &[EmitSegmentMeta],
    chains: &AxisChainSet,
    history: &PerAxisHistory<'_>,
    follower_anchor: &FollowerAnchor<'_>,
    batch_t_start: f64,
    batch_t_end: f64,
    first_seg_left_bc: &[Option<f64>],
) -> Result<ShapeEmission, ShapeError> {
    debug_assert_eq!(
        planned.len(),
        meta.len(),
        "emit_shaped: planned and meta lengths must match",
    );
    let n_axes = chains.n_axes();
    assert!(
        n_axes >= 3,
        "emit_shaped: chain set must cover at least the 3 spatial axes"
    );
    assert_eq!(
        follower_anchor.values.len(),
        chains.followers.len(),
        "emit_shaped: follower anchor values must be parallel to chains.followers"
    );
    for axis in 3..n_axes {
        assert!(
            chains.is_follower_axis(axis),
            "emit_shaped: axis {axis} has a chain but no follower declaration — \
             non-spatial axes must follow other axes"
        );
    }

    if planned.is_empty() {
        return Ok(ShapeEmission {
            segments: Vec::new(),
            follower_inputs: vec![Vec::new(); chains.followers.len()],
        });
    }

    let mut shaped: Vec<Vec<Option<ScalarNurbs<f64>>>> = planned
        .iter()
        .map(|_| (0..n_axes).map(|_| None).collect())
        .collect();

    let left_bc_for = |axis: usize| first_seg_left_bc.get(axis).copied().flatten();

    for axis in 0..3 {
        if chains.is_follower_axis(axis) {
            continue;
        }
        emit_spatial_axis(
            planned,
            axis,
            &chains.chains[axis],
            history,
            batch_t_start,
            batch_t_end,
            left_bc_for(axis),
            &mut shaped,
        )?;
    }

    let mut follower_inputs: Vec<Vec<BezierPiece<f64>>> =
        Vec::with_capacity(chains.followers.len());
    for (follower_idx, (f_axis, followed)) in chains.followers.iter().enumerate() {
        let ledger = emit_follower_axis(
            planned,
            meta,
            *f_axis,
            followed,
            &chains.chains[*f_axis],
            history,
            FollowerAnchor {
                t: follower_anchor.t,
                values: &follower_anchor.values[follower_idx..=follower_idx],
            },
            batch_t_start,
            batch_t_end,
            left_bc_for(*f_axis),
            &mut shaped,
        )?;
        follower_inputs.push(ledger);
    }

    let segments = planned
        .iter()
        .zip(shaped)
        .zip(meta)
        .map(|((fitted, axes), m)| ShapedSegment {
            axes: axes
                .into_iter()
                .map(|a| a.expect("every axis emitted by one of the two passes"))
                .collect(),
            followers: m.followers.clone(),
            t_start: fitted.t_start,
            t_end: fitted.t_end,
            motor_mask: 0,
        })
        .collect();

    Ok(ShapeEmission {
        segments,
        follower_inputs,
    })
}

#[allow(clippy::too_many_arguments)]
fn emit_spatial_axis(
    planned: &[FittedSegment],
    axis: usize,
    chain: &CompiledChain,
    history: &PerAxisHistory<'_>,
    batch_t_start: f64,
    batch_t_end: f64,
    first_left_bc: Option<f64>,
    shaped: &mut [Vec<Option<ScalarNurbs<f64>>>],
) -> Result<(), ShapeError> {
    let gain_applied_storage: Option<Vec<FittedSegment>> = (chain.gain != 0.0).then(|| {
        planned
            .iter()
            .map(|seg| {
                let mut transformed = seg.clone();
                transformed.axes[axis] = apply_derivative_gain(&seg.axes[axis], chain.gain);
                transformed
            })
            .collect()
    });
    let input: &[FittedSegment] = gain_applied_storage.as_deref().unwrap_or(planned);

    let half_support = chain.kernel.as_ref().map_or(0.0, kernel_half_support);
    let mut prev_right_bc = first_left_bc;

    for (seg_idx, fitted) in input.iter().enumerate() {
        let t_start = fitted.t_start;
        let t_end = fitted.t_end;

        let cps = fitted.axes[axis].control_points();
        let &first = cps.first().unwrap_or_else(|| {
            panic!("emit_shaped: seg {seg_idx} axis {axis} has empty control points — fitter produced a degenerate FittedSegment")
        });
        let axis_is_constant = cps.iter().all(|c| (*c - first).abs() < CONSTANT_AXIS_EPS);

        let axis_shaped = if axis_is_constant {
            crate::beta::constant_cubic_nurbs(first, t_start, t_end)
        } else if let Some(kernel) = chain.kernel.as_ref() {
            let padded = pad_segment_axis_with_history(
                seg_idx,
                axis,
                input,
                history.axis(axis),
                half_support,
                batch_t_start,
                batch_t_end,
            );
            let sig = crate::shaper::ShapedSignal::new(&padded, kernel, t_start, t_end);
            fit_track(&|t| sig.eval(t), t_start, t_end, prev_right_bc, seg_idx)?
        } else {
            let passthrough = fitted.axes[axis].clone();
            fit_track(
                &|t| nurbs_eval(&passthrough, t),
                t_start,
                t_end,
                prev_right_bc,
                seg_idx,
            )?
        };

        prev_right_bc = Some(right_boundary_velocity(&axis_shaped, t_end));
        shaped[seg_idx][axis] = Some(axis_shaped);
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn emit_follower_axis(
    planned: &[FittedSegment],
    meta: &[EmitSegmentMeta],
    f_axis: usize,
    followed: &[usize],
    chain: &CompiledChain,
    history: &PerAxisHistory<'_>,
    anchor: FollowerAnchor<'_>,
    batch_t_start: f64,
    batch_t_end: f64,
    first_left_bc: Option<f64>,
    shaped: &mut [Vec<Option<ScalarNurbs<f64>>>],
) -> Result<Vec<BezierPiece<f64>>, ShapeError> {
    let odo = build_batch_odometer(planned, followed, shaped)?;
    let start = 0.0;

    let ratios: Vec<f64> = meta
        .iter()
        .map(|m| {
            m.followers
                .iter()
                .find(|d| d.axis_index == f_axis)
                .map_or(0.0, |d| d.ratio)
        })
        .collect();

    let mut input_tracks: Vec<ScalarNurbs<f64>> = Vec::with_capacity(planned.len());
    let mut ledger_pieces: Vec<BezierPiece<f64>> = Vec::new();
    let mut cursor = start;
    let _ = start;

    for (seg_idx, fitted) in planned.iter().enumerate() {
        let t_start = fitted.t_start;
        let t_end = fitted.t_end;
        let ratio = ratios[seg_idx];

        let track = if ratio == 0.0 {
            let constant = crate::beta::constant_cubic_nurbs(cursor, t_start, t_end);
            ledger_pieces.extend(extract_bezier_pieces(&constant));
            constant
        } else if let Some(s_curve) = fitted.virtual_s_of_t.as_ref() {
            let s_deriv = nurbs::eval::derivative(s_curve);
            let s0 = nurbs_eval(s_curve, t_start);
            let value = |t: f64| cursor + ratio * (nurbs_eval(s_curve, t) - s0);
            let deriv = |t: f64| ratio * nurbs_eval(&s_deriv, t);
            let raw = fit_with_gain(&value, &deriv, chain.gain, t_start, t_end, None, seg_idx)?;
            push_ledger(
                &mut ledger_pieces,
                &raw,
                &value,
                chain.gain,
                t_start,
                t_end,
                seg_idx,
            )?;
            cursor = value(t_end);
            raw
        } else {
            let d0 = odo.distance_at(t_start);
            let value = |t: f64| cursor + ratio * (odo.distance_at(t) - d0);
            let deriv = |t: f64| ratio * odo.speed_at(t);
            let raw = fit_with_gain(&value, &deriv, chain.gain, t_start, t_end, None, seg_idx)?;
            push_ledger(
                &mut ledger_pieces,
                &raw,
                &value,
                chain.gain,
                t_start,
                t_end,
                seg_idx,
            )?;
            cursor = value(t_end);
            raw
        };
        input_tracks.push(track);
    }

    let mut prev_right_bc = first_left_bc;
    if let Some(kernel) = chain.kernel.as_ref() {
        let half_support = kernel_half_support(kernel);
        let track_segments: Vec<FittedSegment> = planned
            .iter()
            .zip(&input_tracks)
            .map(|(fitted, track)| FittedSegment {
                axes: [
                    track.clone(),
                    crate::beta::constant_cubic_nurbs(0.0, fitted.t_start, fitted.t_end),
                    crate::beta::constant_cubic_nurbs(0.0, fitted.t_start, fitted.t_end),
                ],
                t_start: fitted.t_start,
                t_end: fitted.t_end,
                virtual_s_of_t: None,
            })
            .collect();

        for (seg_idx, fitted) in planned.iter().enumerate() {
            let t_start = fitted.t_start;
            let t_end = fitted.t_end;
            let padded = pad_segment_axis_with_history(
                seg_idx,
                0,
                &track_segments,
                history.axis(f_axis),
                half_support,
                batch_t_start,
                batch_t_end,
            );
            let sig = crate::shaper::ShapedSignal::new(&padded, kernel, t_start, t_end);
            let axis_shaped = fit_track(&|t| sig.eval(t), t_start, t_end, prev_right_bc, seg_idx)?;
            prev_right_bc = Some(right_boundary_velocity(&axis_shaped, t_end));
            shaped[seg_idx][f_axis] = Some(axis_shaped);
        }
    } else {
        for (seg_idx, track) in input_tracks.into_iter().enumerate() {
            let t_end = planned[seg_idx].t_end;
            let refitted = if prev_right_bc.is_some() && !nurbs_is_constant(&track) {
                fit_track(
                    &|t| nurbs_eval(&track, t),
                    planned[seg_idx].t_start,
                    t_end,
                    prev_right_bc,
                    seg_idx,
                )?
            } else {
                track
            };
            prev_right_bc = Some(right_boundary_velocity(&refitted, t_end));
            shaped[seg_idx][f_axis] = Some(refitted);
        }
    }

    let raw_at_anchor = eval_ledger_pieces(&ledger_pieces, anchor.t).unwrap_or_else(|| {
        panic!(
            "emit_shaped: follower anchor t={} outside the batch ledger domain [{}, {}]",
            anchor.t,
            planned[0].t_start,
            planned[planned.len() - 1].t_end,
        )
    });
    let shift = anchor.values[0] - raw_at_anchor;
    for seg_axes in shaped.iter_mut() {
        let track = seg_axes[f_axis]
            .take()
            .expect("follower track emitted above");
        seg_axes[f_axis] = Some(shift_nurbs_value(&track, shift));
    }
    for piece in &mut ledger_pieces {
        piece.coeffs[0] += shift;
    }

    Ok(ledger_pieces)
}

fn eval_ledger_pieces(pieces: &[BezierPiece<f64>], t: f64) -> Option<f64> {
    const EDGE_TOL: f64 = 1e-9;
    if pieces.is_empty() {
        return None;
    }
    let last = pieces.last().unwrap();
    if t >= last.u_end && t <= last.u_end + EDGE_TOL {
        return Some(last.evaluate(last.u_end));
    }
    pieces
        .iter()
        .find(|p| p.u_start - EDGE_TOL <= t && t < p.u_end)
        .map(|p| p.evaluate(t))
}

fn shift_nurbs_value(curve: &ScalarNurbs<f64>, shift: f64) -> ScalarNurbs<f64> {
    if shift == 0.0 {
        return curve.clone();
    }
    let cps: Vec<f64> = curve.control_points().iter().map(|c| c + shift).collect();
    ScalarNurbs::try_new(curve.degree(), curve.knots().to_vec(), cps)
        .expect("constant shift preserves NURBS invariants")
}

fn push_ledger(
    ledger_pieces: &mut Vec<BezierPiece<f64>>,
    post_gain_fit: &ScalarNurbs<f64>,
    value: &impl Fn(f64) -> f64,
    gain: f64,
    t_start: f64,
    t_end: f64,
    seg_idx: usize,
) -> Result<(), ShapeError> {
    if gain == 0.0 {
        ledger_pieces.extend(extract_bezier_pieces(post_gain_fit));
    } else {
        let pre_gain = fit_track(value, t_start, t_end, None, seg_idx)?;
        ledger_pieces.extend(extract_bezier_pieces(&pre_gain));
    }
    Ok(())
}

fn build_batch_odometer(
    planned: &[FittedSegment],
    followed: &[usize],
    shaped: &[Vec<Option<ScalarNurbs<f64>>>],
) -> Result<Odometer, ShapeError> {
    let t_start = planned.first().expect("planned non-empty").t_start;
    let t_end = planned.last().expect("planned non-empty").t_end;

    let axes: Vec<ScalarNurbs<f64>> = followed
        .iter()
        .map(|&fa| {
            let pieces: Vec<BezierPiece<f64>> = shaped
                .iter()
                .enumerate()
                .flat_map(|(seg_idx, axes)| {
                    let curve = axes[fa].as_ref().unwrap_or_else(|| {
                        panic!(
                            "emit_shaped: follower references axis {fa} which was \
                             not emitted in pass one (segment {seg_idx})"
                        )
                    });
                    extract_bezier_pieces(curve)
                })
                .collect();
            bezier_pieces_to_nurbs(&pieces)
        })
        .collect();

    Odometer::build(&axes, t_start, t_end, ODOMETER_MIN_INTERVALS).map_err(|e| {
        ShapeError::ArcLength {
            index: 0,
            detail: e.to_string(),
        }
    })
}

fn fit_with_gain(
    value: &impl Fn(f64) -> f64,
    deriv: &impl Fn(f64) -> f64,
    gain: f64,
    t_start: f64,
    t_end: f64,
    left_bc: Option<f64>,
    seg_idx: usize,
) -> Result<ScalarNurbs<f64>, ShapeError> {
    if gain == 0.0 {
        fit_track(value, t_start, t_end, left_bc, seg_idx)
    } else {
        fit_track(
            &|t| value(t) + gain * deriv(t),
            t_start,
            t_end,
            left_bc,
            seg_idx,
        )
    }
}

fn fit_track(
    f: &impl Fn(f64) -> f64,
    t_start: f64,
    t_end: f64,
    left_bc: Option<f64>,
    seg_idx: usize,
) -> Result<ScalarNurbs<f64>, ShapeError> {
    fit_c2_cubic_with_bc(f, t_start, t_end, SMOOTH_FIT_TOLERANCE_MM, left_bc, None).map_err(|e| {
        ShapeError::FitFailure {
            index: seg_idx,
            detail: nurbs::algebra::FitError::ToleranceNotReached {
                achieved_mm: e.achieved_mm,
                at_degree: 3,
            },
        }
    })
}

fn right_boundary_velocity(curve: &ScalarNurbs<f64>, t_end: f64) -> f64 {
    let d1 = nurbs::eval::derivative(curve);
    nurbs::eval::eval(&d1, t_end)
}

fn nurbs_is_constant(curve: &ScalarNurbs<f64>) -> bool {
    let cps = curve.control_points();
    let Some(&first) = cps.first() else {
        return true;
    };
    cps.iter().all(|c| (*c - first).abs() < CONSTANT_AXIS_EPS)
}

#[cfg(test)]
mod tests;
