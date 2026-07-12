use crate::kinematics::{KinematicsModule, SPATIAL_AXES};
use crate::mcu_config::McuAxisConfig;
use crate::pump::EnqueueMsg;
use crate::types::AxisKey;
use nurbs::ScalarNurbs;
use nurbs::bezier::BezierPiece;
use runtime::piece_ring::{MAX_PIECE_COEFFS, PieceEntry};
use trajectory::ShapedSegment;

fn scale_curve_exact(curve: &ScalarNurbs, weight: f64) -> ScalarNurbs {
    if weight == 1.0 {
        curve.clone()
    } else {
        nurbs::algebra::scalar_multiply(curve, weight)
    }
}

pub(crate) fn lane_curve(
    module: &KinematicsModule,
    seg_axes: &[ScalarNurbs],
    lane: usize,
) -> ScalarNurbs {
    if lane >= SPATIAL_AXES || module.lane_is_identity(lane) {
        return seg_axes[lane].clone();
    }
    let w = module.lane_weights(lane);
    let mut acc: Option<ScalarNurbs> = None;
    for (axis, &weight) in w.iter().enumerate() {
        if weight == 0.0 {
            continue;
        }
        let term = scale_curve_exact(&seg_axes[axis], weight);
        acc = Some(match acc {
            None => term,
            Some(prev) => nurbs::algebra::add_with_knot_union(&prev, &term).unwrap_or_else(|e| {
                panic!("lane combine knot-union failed (invariant violation — all ShapedSegment axes share one time domain): {e:?}")
            }),
        });
    }
    acc.expect("kinematics lane with all-zero weights is a module construction bug")
}

pub struct EnqueueCtx<P> {
    pub t0: f64,
    pub epoch: crate::anchor::StreamEpoch,
    pub host_now: f64,
    pub lead_secs: f64,
    pub project: P,
    pub max_piece_secs: Option<f64>,
}

pub fn enqueue_segment<P>(
    seg: &ShapedSegment,
    mcu_configs: &[McuAxisConfig],
    ctx: &EnqueueCtx<P>,
) -> Vec<EnqueueMsg>
where
    P: Fn(u32, f64) -> u64,
{
    let mut out = Vec::new();

    for cfg in mcu_configs {
        let module = KinematicsModule::from_tag(cfg.kinematics)
            .expect("build_mcu_configs validated the kinematics tag");

        for &axis_idx in &cfg.axes {
            if axis_idx >= seg.axes.len() {
                continue;
            }

            let curve = lane_curve(&module, &seg.axes, axis_idx);

            tracing::trace!(
                subsystem = "motion",
                event = "pipe_axis",
                line = seg.source_line,
                mcu = cfg.mcu_id,
                axis = axis_idx,
                pos_start = nurbs::eval::eval(&curve, seg.t_start),
                pos_end = nurbs::eval::eval(&curve, seg.t_end),
                "[pipe] axis lane (post-kinematics motor command)"
            );

            let key = AxisKey {
                mcu_id: cfg.mcu_id,
                axis: axis_idx as u8,
            };

            let pieces = flatten_axis(
                &curve,
                &FlattenCtx {
                    t0: ctx.t0,
                    mcu_id: cfg.mcu_id,
                    axis_idx,
                    host_now: ctx.host_now,
                    project: &ctx.project,
                    max_piece_secs: ctx.max_piece_secs,
                    motor_mask: seg.motor_mask,
                },
            );
            check_step_rate_ceiling(cfg, axis_idx, &pieces, seg.source_line);
            if !pieces.is_empty() {
                out.push(EnqueueMsg {
                    key,
                    pieces,
                    epoch: ctx.epoch,
                    lead_secs: ctx.lead_secs,
                    source_line: seg.source_line,
                });
            }
        }
    }

    out
}

/// Host-side mirror of the MCU's per-sample step budget (-310) and the
/// runtime's -307 StepRateExceedsMcuCeiling: a track demanding more motor
/// velocity than the MCU can physically step fails loud here, with the axis,
/// the demand and the gcode line, instead of latching a bare fault mid-print.
fn check_step_rate_ceiling(
    cfg: &McuAxisConfig,
    axis_idx: usize,
    pieces: &[(runtime::piece_ring::PieceEntry, f64)],
    source_line: u32,
) {
    let ceiling = cfg.motor_velocity_ceiling(axis_idx);
    if ceiling == f64::INFINITY {
        return;
    }
    for (piece, _) in pieces {
        let demand = f64::from(piece.vel_start().abs().max(piece.vel_end().abs()));
        assert!(
            demand <= ceiling,
            "step rate exceeds MCU ceiling (-307) on mcu{} axis{axis_idx}: the shaped \
             track demands {demand:.1} mm/s but the motor can only be stepped at \
             {ceiling:.1} mm/s (gcode line {source_line}) — lower the velocity/accel \
             demand on this axis (pressure advance and smoothing add to it) or raise \
             the motor's step-rate ceiling (dedge, shorter step pulse, coarser \
             microstepping)",
            cfg.mcu_id,
        );
    }
}

/// The NURBS carrier's Bernstein round trip turns a lower-degree piece's zero
/// tail into ~1e-16-of-scale monomial noise, so the wire `coeff_count` is
/// recovered by Chebyshev truncation instead — budgeted far below f32 wire
/// resolution in position AND in seam velocity/acceleration, so dropping the
/// tail can never step the servo feedforward.
const WIRE_TRUNC_POS_MM: f64 = 1e-6;
const WIRE_TRUNC_VEL_MM_S: f64 = 1e-3;
const WIRE_TRUNC_ACC_MM_S2: f64 = 0.1;

/// Monomial-in-τ coefficients with the exactly-zero tail trimmed (exact zeros
/// survive constant pieces and pre-carrier callers).
fn trimmed_monomial(bp: &BezierPiece) -> Vec<f64> {
    let mut coeffs = bp.coeffs.clone();
    while coeffs.len() > 1 && coeffs.last().is_some_and(|c| *c == 0.0) {
        coeffs.pop();
    }
    assert!(
        !coeffs.is_empty() && coeffs.len() <= MAX_PIECE_COEFFS,
        "{} monomial coeffs — the wire format carries at most {MAX_PIECE_COEFFS} \
         Chebyshev coefficients per piece; truncating silently corrupts the \
         dispatched polynomial",
        coeffs.len()
    );
    coeffs
}

struct MergedPiece {
    coeffs: Vec<f64>,
    duration: f64,
    curve_u_start: f64,
}

/// Pieces shorter than this are knot dust from curve arithmetic (observed at
/// 2-4 ULP of the timeline, ~4e-16 s): far below the nanosecond start_time
/// quantization and the MCU sample period, they cannot be executed, and
/// their epsilon duration turns f32 coefficient noise into six-figure
/// endpoint velocities. They are dropped exactly — provided they carry no
/// real motion, which is asserted.
const PIECE_DUST_FLOOR_SECS: f64 = 1e-9;

/// Upper bound on how far a monomial-in-τ piece can move over its duration.
fn motion_span_bound(coeffs: &[f64], duration: f64) -> f64 {
    let mut pow = 1.0;
    coeffs
        .iter()
        .skip(1)
        .map(|c| {
            pow *= duration;
            c.abs() * pow
        })
        .sum()
}

pub(crate) struct FlattenCtx<'a, P> {
    pub t0: f64,
    pub mcu_id: u32,
    pub axis_idx: usize,
    pub host_now: f64,
    pub project: &'a P,
    pub max_piece_secs: Option<f64>,
    pub motor_mask: u8,
}

fn flatten_axis<P>(curve: &ScalarNurbs, ctx: &FlattenCtx<'_, P>) -> Vec<(PieceEntry, f64)>
where
    P: Fn(u32, f64) -> u64,
{
    let bps = nurbs::bezier::extract_bezier_pieces(curve);
    flatten_bezier_pieces(&bps, ctx)
}

pub(crate) fn flatten_bezier_pieces<P>(
    bps: &[BezierPiece],
    ctx: &FlattenCtx<'_, P>,
) -> Vec<(PieceEntry, f64)>
where
    P: Fn(u32, f64) -> u64,
{
    let mut merged: Vec<MergedPiece> = Vec::with_capacity(bps.len());

    for bp in bps.iter() {
        let coeffs = trimmed_monomial(bp);
        let duration = bp.u_end - bp.u_start;

        if duration < PIECE_DUST_FLOOR_SECS {
            let span = motion_span_bound(&coeffs, duration);
            assert!(
                span <= WIRE_TRUNC_POS_MM,
                "sub-nanosecond piece at u={} moves {span} mm in {duration} s — \
                 a real discontinuity crammed into zero time, not droppable knot dust",
                bp.u_start
            );
            continue;
        }

        if coeffs.len() == 1 {
            if let Some(last) = merged.last_mut() {
                if last.coeffs.len() == 1 && last.coeffs[0].to_bits() == coeffs[0].to_bits() {
                    last.duration += duration;
                    continue;
                }
            }
        }
        merged.push(MergedPiece {
            coeffs,
            duration,
            curve_u_start: bp.u_start,
        });
    }

    let mut out = Vec::with_capacity(merged.len() * 8);

    for (piece_idx, mp) in merged.iter().enumerate() {
        let subs: Vec<(Vec<f64>, f64)> = match ctx.max_piece_secs {
            Some(m) => subdivide_monomial(&mp.coeffs, mp.duration, m),
            None => vec![(mp.coeffs.clone(), mp.duration)],
        };

        let mut sub_offset = 0.0_f64;
        for (sub_idx, (sub_coeffs, sub_dur)) in subs.iter().enumerate() {
            let host_secs = ctx.t0 + mp.curve_u_start + sub_offset;
            let start_time = (ctx.project)(ctx.mcu_id, host_secs);

            let cheb = nurbs::chebyshev::monomial_tau_to_chebyshev(sub_coeffs, *sub_dur);
            let cheb = nurbs::chebyshev::truncate_chebyshev_c2(
                &cheb,
                *sub_dur,
                WIRE_TRUNC_POS_MM,
                WIRE_TRUNC_VEL_MM_S,
                WIRE_TRUNC_ACC_MM_S2,
            );
            let mut entry = PieceEntry::zeroed();
            entry.start_time = start_time;
            entry.duration = *sub_dur as f32;
            entry.motor_mask = ctx.motor_mask;
            entry.coeff_count = cheb.len() as u8;
            for (dst, &c) in entry.coeffs.iter_mut().zip(&cheb) {
                *dst = c as f32;
            }
            if ctx.motor_mask != 0 {
                entry.coeffs[0] -= entry.pos_start();
            }

            let margin_us = (host_secs - ctx.host_now) * 1e6;
            tracing::trace!(
                mcu_id = ctx.mcu_id,
                axis = ctx.axis_idx,
                piece_idx,
                sub_idx,
                u_start = host_secs - ctx.t0,
                margin_us,
                start_ns = start_time,
                "[dispatch-margin]"
            );

            out.push((entry, host_secs));

            sub_offset += sub_dur;
        }
    }

    out
}

/// Split a monomial-in-τ piece into equal-duration chunks via Taylor shift —
/// each chunk's polynomial is the original re-based at its own start.
pub fn subdivide_monomial(
    coeffs: &[f64],
    duration: f64,
    max_piece_secs: f64,
) -> Vec<(Vec<f64>, f64)> {
    if duration <= max_piece_secs {
        return vec![(coeffs.to_vec(), duration)];
    }
    let n = (duration / max_piece_secs).ceil() as usize;
    let sub = duration / n as f64;
    let mut out = Vec::with_capacity(n);
    let mut cur = coeffs.to_vec();
    for _ in 0..n - 1 {
        out.push((cur.clone(), sub));
        cur = nurbs::chebyshev::taylor_shift(&cur, sub);
    }
    out.push((cur, sub));
    out
}

#[cfg(test)]
mod tests;
