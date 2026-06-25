use crate::dispatch::McuAxisConfig;
use crate::kinematics::{KinematicsModule, SPATIAL_AXES};
use crate::pump::{AxisKey, EnqueueMsg};
use nurbs::ScalarNurbs;
use nurbs::bezier::BezierPiece;
use runtime::piece_ring::PieceEntry;
use trajectory::ShapedSegment;

fn scale_curve_exact(curve: &ScalarNurbs<f64>, weight: f64) -> ScalarNurbs<f64> {
    if weight == 1.0 {
        curve.clone()
    } else {
        nurbs::algebra::scalar_multiply(curve, weight)
    }
}

pub(crate) fn lane_curve(
    module: &KinematicsModule,
    seg_axes: &[ScalarNurbs<f64>],
    lane: usize,
) -> ScalarNurbs<f64> {
    if lane >= SPATIAL_AXES || module.lane_is_identity(lane) {
        return seg_axes[lane].clone();
    }
    let w = module.lane_weights(lane);
    let mut acc: Option<ScalarNurbs<f64>> = None;
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

pub fn enqueue_segment<P>(
    seg: &ShapedSegment,
    mcu_configs: &[McuAxisConfig],
    t0: f64,
    fresh_stream: bool,
    host_now: f64,
    lead_secs: f64,
    project: P,
    max_piece_secs: Option<f64>,
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

            tracing::info!(
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
                t0,
                cfg.mcu_id,
                axis_idx,
                host_now,
                &project,
                max_piece_secs,
                seg.motor_mask,
            );
            if !pieces.is_empty() {
                out.push(EnqueueMsg {
                    key,
                    pieces,
                    fresh_stream,
                    lead_secs,
                    source_line: seg.source_line,
                    // stage-3b dispatch wiring threads the planner's brake
                    // generation and provisional tail through here.
                    generation: 0,
                    brake_tail: Vec::new(),
                });
            }
        }
    }

    out
}

fn is_constant_piece(coeffs: &[f64; 4]) -> bool {
    coeffs[0].to_bits() == coeffs[1].to_bits()
        && coeffs[1].to_bits() == coeffs[2].to_bits()
        && coeffs[2].to_bits() == coeffs[3].to_bits()
}

struct MergedPiece {
    coeffs: [f64; 4],
    duration: f64,
    curve_u_start: f64,
}

fn flatten_axis<P>(
    curve: &ScalarNurbs<f64>,
    t0: f64,
    mcu_id: u32,
    axis_idx: usize,
    host_now: f64,
    project: &P,
    max_piece_secs: Option<f64>,
    motor_mask: u8,
) -> Vec<(PieceEntry, f64)>
where
    P: Fn(u32, f64) -> u64,
{
    let bps = nurbs::bezier::extract_bezier_pieces(curve);
    flatten_bezier_pieces(
        &bps,
        t0,
        mcu_id,
        axis_idx,
        host_now,
        project,
        max_piece_secs,
        motor_mask,
    )
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn flatten_bezier_pieces<P>(
    bps: &[BezierPiece<f64>],
    t0: f64,
    mcu_id: u32,
    axis_idx: usize,
    host_now: f64,
    project: &P,
    max_piece_secs: Option<f64>,
    motor_mask: u8,
) -> Vec<(PieceEntry, f64)>
where
    P: Fn(u32, f64) -> u64,
{
    let mut merged: Vec<MergedPiece> = Vec::with_capacity(bps.len());

    for bp in bps.iter() {
        let bern = bp.to_bernstein();

        assert!(
            !bern.is_empty() && bern.len() <= 4,
            "{} Bernstein coeffs — truncating above-cubic pieces silently \
             corrupts the dispatched polynomial (Neptune fault -310, 299 steps/sample); \
             the pipeline is uniform-cubic per CLAUDE.md",
            bern.len()
        );

        let n = bern.len();
        let last_f64 = bern[n - 1];
        let mut coeffs_f64 = [last_f64; 4];
        for k in 0..n {
            coeffs_f64[k] = bern[k];
        }

        let duration = bp.u_end - bp.u_start;

        if is_constant_piece(&coeffs_f64) {
            if let Some(last) = merged.last_mut() {
                if is_constant_piece(&last.coeffs)
                    && last.coeffs[0].to_bits() == coeffs_f64[0].to_bits()
                {
                    last.duration += duration;
                    continue;
                }
            }
        }
        merged.push(MergedPiece {
            coeffs: coeffs_f64,
            duration,
            curve_u_start: bp.u_start,
        });
    }

    let mut out = Vec::with_capacity(merged.len() * 8);

    for (piece_idx, mp) in merged.iter().enumerate() {
        let subs: Vec<([f64; 4], f64)> = match max_piece_secs {
            Some(m) => subdivide_bernstein(mp.coeffs, mp.duration, m),
            None => vec![(mp.coeffs, mp.duration)],
        };

        let mut sub_offset = 0.0_f64;
        for (sub_idx, (sub_coeffs, sub_dur)) in subs.iter().enumerate() {
            let host_secs = t0 + mp.curve_u_start + sub_offset;
            let start_time = project(mcu_id, host_secs);

            let mut coeffs = [0.0_f32; 4];
            for k in 0..4 {
                coeffs[k] = sub_coeffs[k] as f32;
            }
            if motor_mask != 0 {
                let b0 = coeffs[0];
                for c in &mut coeffs {
                    *c -= b0;
                }
            }
            let duration_f32 = *sub_dur as f32;

            let margin_us = (host_secs - host_now) * 1e6;
            tracing::trace!(
                mcu_id,
                axis = axis_idx,
                piece_idx,
                sub_idx,
                u_start = host_secs - t0,
                margin_us,
                start_ns = start_time,
                "[dispatch-margin]"
            );

            out.push((
                PieceEntry {
                    start_time,
                    coeffs,
                    duration: duration_f32,
                    motor_mask,
                    _reserved: [0; 3],
                },
                host_secs,
            ));

            sub_offset += sub_dur;
        }
    }

    out
}

pub fn subdivide_bernstein(
    coeffs: [f64; 4],
    duration: f64,
    max_piece_secs: f64,
) -> Vec<([f64; 4], f64)> {
    if duration <= max_piece_secs {
        return vec![(coeffs, duration)];
    }
    let n = (duration / max_piece_secs).ceil() as usize;
    let sub = duration / n as f64;
    let mut out = Vec::with_capacity(n);
    let mut rest = coeffs;
    for i in 0..n - 1 {
        let u = sub / (duration - i as f64 * sub);
        let (left, right) = de_casteljau_split(rest, u);
        out.push((left, sub));
        rest = right;
    }
    out.push((rest, sub));
    out
}

fn de_casteljau_split(c: [f64; 4], u: f64) -> ([f64; 4], [f64; 4]) {
    let b01 = lerp(c[0], c[1], u);
    let b12 = lerp(c[1], c[2], u);
    let b23 = lerp(c[2], c[3], u);
    let b012 = lerp(b01, b12, u);
    let b123 = lerp(b12, b23, u);
    let b = lerp(b012, b123, u);
    ([c[0], b01, b012, b], [b, b123, b23, c[3]])
}

fn lerp(a: f64, b: f64, u: f64) -> f64 {
    a + (b - a) * u
}

#[cfg(test)]
mod tests;
