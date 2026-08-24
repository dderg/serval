use std::sync::Arc;

use crate::kinematics::{KinematicsModule, SPATIAL_AXES};
use crate::mcu_config::McuAxisConfig;
use crate::pump::EnqueueMsg;
use crate::types::AxisKey;
use nurbs::ScalarNurbs;
use trajectory::{
    AnalyticMoveSpan, ClockedMotorSpan, ContinuousAxis, ContinuousError, ContinuousSegment,
    MotorGroup, MotorSpan, MotorTerm, RelativeSplinePiece,
};

fn lane_terms(
    module: &KinematicsModule,
    axes: &[ContinuousAxis],
    lane: usize,
) -> Result<Vec<MotorTerm>, ContinuousError> {
    let term_of = |source_axis: usize, scale: f64| -> Result<MotorTerm, ContinuousError> {
        let axis = axes
            .get(source_axis)
            .ok_or(ContinuousError::AxisOutsideMove { axis: source_axis })?
            .clone();
        Ok(MotorTerm {
            source_axis,
            axis,
            scale,
        })
    };

    if lane >= SPATIAL_AXES || module.lane_is_identity(lane) {
        return Ok(vec![term_of(lane, 1.0)?]);
    }
    module
        .lane_weights(lane)
        .into_iter()
        .enumerate()
        .filter(|(_, weight)| *weight != 0.0)
        .map(|(source_axis, weight)| term_of(source_axis, weight))
        .collect()
}

/// Correlated terms must reach the evaluator as one group: a CoreXY lane whose
/// X and Y terms come from the same [`AnalyticMoveSpan`] cancels exactly, and
/// splitting it into independent boxes would bound a stationary motor by the
/// sum of two large spatial projections.
fn group_terms(terms: Vec<MotorTerm>) -> Arc<[MotorGroup]> {
    let mut analytic: Vec<(Arc<AnalyticMoveSpan>, Vec<MotorTerm>)> = Vec::new();
    let mut spline: Vec<(Arc<ScalarNurbs>, f64)> = Vec::new();
    let mut relative: Vec<(Arc<ScalarNurbs>, f64, f64)> = Vec::new();
    let mut independent: Vec<MotorTerm> = Vec::new();

    for term in terms {
        match term.axis.clone() {
            ContinuousAxis::Analytic { span, axis } if axis == term.source_axis => {
                match analytic
                    .iter_mut()
                    .find(|(shared, _)| Arc::ptr_eq(shared, &span))
                {
                    Some((_, bucket)) => bucket.push(term),
                    None => analytic.push((span, vec![term])),
                }
            }
            ContinuousAxis::Spline(curve) => {
                match spline
                    .iter_mut()
                    .find(|(shared, _)| Arc::ptr_eq(shared, &curve))
                {
                    Some((_, summed_scale)) => *summed_scale += term.scale,
                    None => spline.push((curve, term.scale)),
                }
            }
            ContinuousAxis::RelativeSpline {
                base_position,
                curve,
            } => {
                match relative.iter_mut().find(|(shared, base, _)| {
                    Arc::ptr_eq(shared, &curve) && base.to_bits() == base_position.to_bits()
                }) {
                    Some((_, _, summed_scale)) => *summed_scale += term.scale,
                    None => relative.push((curve, base_position, term.scale)),
                }
            }
            ContinuousAxis::PiecewiseRelativeSpline(_)
            | ContinuousAxis::Analytic { .. }
            | ContinuousAxis::Hold { .. }
            | ContinuousAxis::Nudge(_)
            | ContinuousAxis::Buzz { .. } => independent.push(term),
        }
    }

    analytic
        .into_iter()
        .map(|(span, terms)| MotorGroup::Analytic {
            span,
            terms: terms.into(),
        })
        .chain(
            spline
                .into_iter()
                .map(|(curve, summed_scale)| MotorGroup::Spline {
                    curve,
                    summed_scale,
                }),
        )
        .chain(
            relative
                .into_iter()
                .map(
                    |(curve, base_position, summed_scale)| MotorGroup::RelativeSpline {
                        curve,
                        base_position,
                        summed_scale,
                    },
                ),
        )
        .chain(independent.into_iter().map(MotorGroup::Independent))
        .collect()
}

/// Hold merging rewrites a span's time domain, so it may only claim a lane the
/// host declared stationary: a time-domain [`ContinuousAxis::Hold`] or a spline
/// whose absolute control positions are bit-identical — for a base-relative
/// curve that is `base_position` plus each control point, the absolute value
/// the endpoint reconstructs. A piecewise curve carries one base per piece, so
/// every piece's controls must reconstruct that same absolute anchor. A
/// numerically small analytic move and an exactly cancelling kinematic
/// combination both still carry a trajectory the endpoint must evaluate on its
/// own clock.
fn terms_are_explicit_hold(terms: &[MotorTerm]) -> bool {
    let mut moving_terms = 0usize;
    for term in terms {
        if term.scale == 0.0 {
            continue;
        }
        moving_terms += 1;
        let held = match &term.axis {
            ContinuousAxis::Hold { .. } => true,
            ContinuousAxis::Spline(curve) => controls_are_constant(curve, 0.0),
            ContinuousAxis::RelativeSpline {
                base_position,
                curve,
            } => controls_are_constant(curve, *base_position),
            ContinuousAxis::PiecewiseRelativeSpline(pieces) => piece_controls_are_constant(pieces),
            ContinuousAxis::Analytic { .. }
            | ContinuousAxis::Nudge(_)
            | ContinuousAxis::Buzz { .. } => false,
        };
        if !held {
            return false;
        }
    }
    moving_terms > 0
}

fn controls_are_constant(curve: &ScalarNurbs, base_position: f64) -> bool {
    match curve.control_points().split_first() {
        Some((first, rest)) => {
            let anchor = (base_position + first).to_bits();
            rest.iter()
                .all(|cp| (base_position + cp).to_bits() == anchor)
        }
        None => false,
    }
}

fn piece_controls_are_constant(pieces: &[RelativeSplinePiece]) -> bool {
    let mut anchor: Option<u64> = None;
    for piece in pieces {
        let controls = piece.curve.control_points();
        if controls.is_empty() {
            return false;
        }
        for cp in controls {
            let bits = (piece.base_position + cp).to_bits();
            match anchor {
                Some(expected) if bits != expected => return false,
                Some(_) => {}
                None => anchor = Some(bits),
            }
        }
    }
    anchor.is_some()
}

pub(crate) fn lane_span(
    module: &KinematicsModule,
    seg: &ContinuousSegment,
    lane: usize,
) -> Result<MotorSpan, ContinuousError> {
    let terms = lane_terms(module, &seg.axes, lane)?;
    let is_explicit_hold = terms_are_explicit_hold(&terms);
    MotorSpan::try_new(
        group_terms(terms),
        seg.t_start,
        seg.t_end,
        seg.motor_mask,
        seg.source_line,
        is_explicit_hold,
    )
}

pub struct EnqueueCtx<'a, P> {
    pub t0: f64,
    pub epoch: crate::anchor::StreamEpoch,
    pub host_now: f64,
    pub lead_secs: f64,
    /// Unrounded host-seconds to MCU-cycles map for this dispatch. Every
    /// rounded view endpoint is derived from this exact anchor, so a chain of
    /// abutting views cannot accumulate the slope error that recovering the
    /// rate from two rounded clocks would introduce.
    pub project_exact: P,
    pub clock_freq_hz: &'a dyn Fn(u32) -> f64,
    pub epoch_freq: &'a dyn Fn(u32) -> Option<f64>,
    /// Whether this lane currently executes on the phase (coil-write)
    /// transport: phase samples carry no step pulses, so the pulse-path
    /// step-rate ceiling does not bound them.
    pub lane_is_phase: &'a dyn Fn(AxisKey) -> bool,
}

pub fn enqueue_segment<P>(
    seg: &ContinuousSegment,
    mcu_configs: &[McuAxisConfig],
    ctx: &EnqueueCtx<'_, P>,
) -> Result<Vec<EnqueueMsg>, ContinuousError>
where
    P: Fn(u32, f64) -> f64,
{
    let mut out = Vec::new();

    for cfg in mcu_configs {
        let module = KinematicsModule::from_tag(cfg.kinematics)
            .expect("build_mcu_configs validated the kinematics tag");

        for &axis_idx in &cfg.axes {
            if axis_idx >= seg.axes.len() {
                continue;
            }

            let span = lane_span(&module, seg, axis_idx)?;
            let pos_start = span.position(span.t_start)?;
            let pos_end = span.position(span.t_end)?;

            tracing::trace!(
                subsystem = "motion",
                event = "pipe_axis",
                line = seg.source_line,
                mcu = cfg.mcu_id,
                axis = axis_idx,
                pos_start,
                pos_end,
                "[pipe] axis lane (post-kinematics motor command)"
            );

            let key = AxisKey {
                mcu_id: cfg.mcu_id,
                axis: axis_idx as u8,
            };

            if !(ctx.lane_is_phase)(key) {
                check_step_rate_ceiling(cfg, axis_idx, &span, seg.source_line)?;
            }
            if cfg.ethercat && span.is_explicit_hold {
                if ctx.epoch.position_redefined() {
                    out.push(EnqueueMsg {
                        epoch_freq: (ctx.epoch_freq)(cfg.mcu_id),
                        key,
                        spans: Vec::new(),
                        epoch: ctx.epoch,
                        lead_secs: ctx.lead_secs,
                        source_line: seg.source_line,
                        batch_end: false,
                    });
                }
                continue;
            }

            let spans = clock_span(Arc::new(span), cfg.mcu_id, axis_idx, ctx)?;
            if !spans.is_empty() {
                out.push(EnqueueMsg {
                    epoch_freq: (ctx.epoch_freq)(cfg.mcu_id),
                    key,
                    spans,
                    epoch: ctx.epoch,
                    lead_secs: ctx.lead_secs,
                    source_line: seg.source_line,
                    batch_end: false,
                });
            }
        }
    }
    if let Some(last) = out.last_mut() {
        last.batch_end = true;
    }

    Ok(out)
}

/// Anchor one motor signal on `mcu_id`'s clock and cut it into the bounded
/// zero-copy views the transports admit. The signal itself is never refit,
/// resampled, or copied: every view shares its `Arc`.
pub(crate) fn clock_span<P>(
    signal: Arc<MotorSpan>,
    mcu_id: u32,
    axis_idx: usize,
    ctx: &EnqueueCtx<'_, P>,
) -> Result<Vec<ClockedMotorSpan>, ContinuousError>
where
    P: Fn(u32, f64) -> f64,
{
    let start_host = ctx.t0 + signal.t_start;
    let end_host = ctx.t0 + signal.t_end;
    let start_clock_exact = (ctx.project_exact)(mcu_id, start_host);
    if !(start_clock_exact > 0.0) {
        return Err(ContinuousError::InvalidSpan {
            reason: "projected start clock must be positive",
        });
    }
    let stream_t_start = signal.t_start;
    let stream_t_end = signal.t_end;
    let view = match ClockedMotorSpan::try_new(
        signal,
        stream_t_start,
        stream_t_end,
        start_host,
        end_host,
        start_clock_exact,
        (ctx.clock_freq_hz)(mcu_id),
    ) {
        Ok(view) => view,
        Err(ContinuousError::InvalidSpan {
            reason: "positive-duration clocked view must span at least one clock",
        }) => return Ok(Vec::new()),
        Err(error) => return Err(error),
    };
    let views = view.split_max_duration()?;

    for (view_idx, view) in views.iter().enumerate() {
        tracing::trace!(
            mcu_id,
            axis = axis_idx,
            view_idx,
            u_start = view.stream_t_start,
            margin_us = (view.start_host - ctx.host_now) * 1e6,
            start_clock = view.start_clock,
            end_clock = view.end_clock,
            "[dispatch-margin]"
        );
    }

    Ok(views)
}

/// Host-side mirror of the MCU's per-sample step budget (-310) and the
/// runtime's -307 StepRateExceedsMcuCeiling: a track demanding more motor
/// velocity than the MCU can physically step fails loud here, with the axis,
/// the demand and the gcode line, instead of latching a bare fault mid-print.
fn check_step_rate_ceiling(
    cfg: &McuAxisConfig,
    axis_idx: usize,
    span: &MotorSpan,
    source_line: u32,
) -> Result<(), ContinuousError> {
    let ceiling = cfg.motor_velocity_ceiling(axis_idx);
    if ceiling == f64::INFINITY {
        return Ok(());
    }
    for window in span.breakpoints.windows(2) {
        let bounds = span.pva_bounds(window[0], window[1])?;
        let demand = bounds.velocity_min.abs().max(bounds.velocity_max.abs());
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
    Ok(())
}

#[cfg(test)]
mod tests;
