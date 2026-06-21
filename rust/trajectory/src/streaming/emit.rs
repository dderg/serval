use nurbs::bezier::BezierPiece;

use super::{EmitContext, ShaperState};
use crate::emit_shaped::{emit_shaped, emit_shaped_with_left_bc, FollowerAnchor, PerAxisHistory};
use crate::ShapeError;
use crate::ShapedSegment;

const T_EPSILON: f64 = 1e-12;

impl ShaperState {
    pub fn emit_committed(
        &mut self,
        ctx: &EmitContext<'_>,
    ) -> Result<Vec<ShapedSegment>, ShapeError> {
        let max_h = self.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);

        let target = self.t_decel_start - max_h;
        if target <= self.t_dispatched + T_EPSILON {
            return Ok(Vec::new());
        }

        if self.planned_fitted.is_empty() && self.pending_freeze.is_empty() {
            return Ok(Vec::new());
        }

        let t_freeze = self.t_dispatched + max_h;

        let mut dispatched: Vec<ShapedSegment> = Vec::new();

        let mut freeze_end_velocities: Vec<Option<f64>> = vec![None; self.axes.len()];

        if !self.pending_freeze.is_empty() {
            let pending = std::mem::take(&mut self.pending_freeze);

            let last_pending = pending.last();
            if let Some(last_seg) = last_pending {
                for (axis, slot) in freeze_end_velocities
                    .iter_mut()
                    .enumerate()
                    .take(last_seg.axes.len())
                {
                    *slot = Some(shaped_axis_velocity_at(&last_seg.axes[axis], t_freeze));
                }
            }

            for seg in pending {
                if seg.t_end <= self.t_dispatched + T_EPSILON {
                    continue;
                }
                if seg.t_start >= target - T_EPSILON {
                    break;
                }
                let lo = self.t_dispatched.max(seg.t_start);
                let hi = target.min(seg.t_end);
                if hi <= lo + T_EPSILON {
                    continue;
                }
                if (lo - seg.t_start).abs() < T_EPSILON && (hi - seg.t_end).abs() < T_EPSILON {
                    dispatched.push(seg);
                } else {
                    let restricted = restrict_segment_lo_hi(&seg, lo, hi).map_err(|detail| {
                        ShapeError::Algebra {
                            index: dispatched.len(),
                            detail,
                        }
                    })?;
                    dispatched.push(restricted);
                }
            }

            if target <= t_freeze + T_EPSILON {
                self.t_shaped = target;
                self.t_dispatched = target;
                trim_per_axis_history(&mut self.axes, self.t_dispatched, max_h);
                return Ok(dispatched);
            }
        }

        let emit_start = if !dispatched.is_empty() {
            t_freeze
        } else {
            self.t_dispatched
        };

        if self.planned_fitted.is_empty() {
            self.t_shaped = target;
            self.t_dispatched = target;
            trim_per_axis_history(&mut self.axes, self.t_dispatched, max_h);
            return Ok(dispatched);
        }

        let window_start = self
            .planned_fitted
            .first()
            .map_or(emit_start, |f| f.t_start.max(emit_start));
        let history_storage = build_history_storage(&self.axes, window_start);
        let history = PerAxisHistory {
            axes: history_storage.iter().map(Vec::as_slice).collect(),
        };

        let batch_t_end = self.t_appended;

        let left_bc: Vec<Option<f64>> = if dispatched.is_empty() {
            vec![None; self.axes.len()]
        } else {
            freeze_end_velocities
        };

        let anchor_values = self.follower_anchor_values(ctx, emit_start);
        let emission = emit_shaped_with_left_bc(
            &self.planned_fitted,
            &self.planned_meta,
            ctx.chains,
            &history,
            &FollowerAnchor {
                t: emit_start.max(self.planned_fitted[0].t_start),
                values: &anchor_values,
            },
            emit_start,
            batch_t_end,
            &left_bc,
        )?;
        let shaped = emission.segments;
        self.store_follower_ledgers(ctx, emission.follower_inputs);

        let new_pending_start = target;
        let new_pending_end = (target + max_h).min(self.t_appended);

        let mut new_pending: Vec<ShapedSegment> = Vec::new();

        for seg in shaped {
            if seg.t_end <= emit_start + T_EPSILON {
                continue;
            }
            if seg.t_start >= new_pending_end - T_EPSILON {
                break;
            }

            let lo = emit_start.max(seg.t_start);
            let hi = new_pending_end.min(seg.t_end);
            if hi <= lo + T_EPSILON {
                continue;
            }

            let seg_to_store =
                if (lo - seg.t_start).abs() < T_EPSILON && (hi - seg.t_end).abs() < T_EPSILON {
                    seg.clone()
                } else {
                    restrict_segment_lo_hi(&seg, lo, hi).map_err(|detail| ShapeError::Algebra {
                        index: dispatched.len() + new_pending.len(),
                        detail,
                    })?
                };

            if lo < new_pending_start - T_EPSILON {
                let dispatch_hi = new_pending_start.min(hi);
                if dispatch_hi > lo + T_EPSILON {
                    let dispatch_seg = if (lo - seg.t_start).abs() < T_EPSILON
                        && (dispatch_hi - seg.t_end).abs() < T_EPSILON
                    {
                        seg.clone()
                    } else {
                        restrict_segment_lo_hi(&seg, lo, dispatch_hi).map_err(|detail| {
                            ShapeError::Algebra {
                                index: dispatched.len(),
                                detail,
                            }
                        })?
                    };
                    dispatched.push(dispatch_seg);
                }

                if hi > new_pending_start + T_EPSILON {
                    let pending_lo = new_pending_start;
                    let pending_hi = hi;
                    let pending_seg = restrict_segment_lo_hi(&seg, pending_lo, pending_hi)
                        .map_err(|detail| ShapeError::Algebra {
                            index: dispatched.len() + new_pending.len(),
                            detail,
                        })?;
                    new_pending.push(pending_seg);
                }
            } else {
                new_pending.push(seg_to_store);
            }
        }

        self.pending_freeze = new_pending;

        self.t_shaped = target;
        self.t_dispatched = target;

        let delta_safety = max_h;
        let trim_cutoff = self.t_dispatched - max_h - delta_safety;
        for axis in &mut self.axes {
            while let Some(front) = axis.pieces.front() {
                if front.u_end < trim_cutoff - T_EPSILON {
                    axis.pieces.pop_front();
                } else {
                    break;
                }
            }
        }

        Ok(dispatched)
    }

    pub fn commit_decel_to_zero(
        &mut self,
        ctx: &EmitContext<'_>,
    ) -> Result<Vec<ShapedSegment>, ShapeError> {
        if self.t_dispatched >= self.t_appended - T_EPSILON {
            return Ok(Vec::new());
        }

        if self.planned_fitted.is_empty() && self.pending_freeze.is_empty() {
            return Ok(Vec::new());
        }

        let max_h = self.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);
        let target = self.t_appended;

        let mut dispatched: Vec<ShapedSegment> = Vec::new();

        if !self.pending_freeze.is_empty() {
            let pending = std::mem::take(&mut self.pending_freeze);
            for seg in pending {
                if seg.t_end <= self.t_dispatched + T_EPSILON {
                    continue;
                }
                let lo = self.t_dispatched.max(seg.t_start);
                let hi = target.min(seg.t_end);
                if hi <= lo + T_EPSILON {
                    continue;
                }
                if (lo - seg.t_start).abs() < T_EPSILON && (hi - seg.t_end).abs() < T_EPSILON {
                    dispatched.push(seg);
                } else {
                    let restricted = restrict_segment_lo_hi(&seg, lo, hi).map_err(|detail| {
                        ShapeError::Algebra {
                            index: dispatched.len(),
                            detail,
                        }
                    })?;
                    dispatched.push(restricted);
                }
            }
        }

        if !self.planned_fitted.is_empty() {
            let t_freeze = self.t_dispatched + max_h;
            let emit_start = if !dispatched.is_empty() {
                t_freeze
            } else {
                self.t_dispatched
            };

            let history_storage = build_history_storage(&self.axes, emit_start);
            let history = PerAxisHistory {
                axes: history_storage.iter().map(Vec::as_slice).collect(),
            };

            let anchor_values = self.follower_anchor_values(ctx, emit_start);
            let emission = emit_shaped(
                &self.planned_fitted,
                &self.planned_meta,
                ctx.chains,
                &history,
                &FollowerAnchor {
                    t: emit_start.max(self.planned_fitted[0].t_start),
                    values: &anchor_values,
                },
                emit_start,
                self.t_appended,
            )?;
            let shaped = emission.segments;
            self.store_follower_ledgers(ctx, emission.follower_inputs);

            for seg in shaped {
                if seg.t_end <= emit_start + T_EPSILON {
                    continue;
                }
                if seg.t_start >= target - T_EPSILON {
                    break;
                }
                let lo = emit_start.max(seg.t_start);
                let hi = target.min(seg.t_end);
                if hi <= lo + T_EPSILON {
                    continue;
                }
                if (lo - seg.t_start).abs() < T_EPSILON && (hi - seg.t_end).abs() < T_EPSILON {
                    dispatched.push(seg);
                } else {
                    let restricted = restrict_segment_lo_hi(&seg, lo, hi).map_err(|detail| {
                        ShapeError::Algebra {
                            index: dispatched.len(),
                            detail,
                        }
                    })?;
                    dispatched.push(restricted);
                }
            }
        }

        self.t_shaped = target;
        self.t_dispatched = target;

        trim_per_axis_history(&mut self.axes, self.t_dispatched, max_h);

        Ok(dispatched)
    }

    fn follower_anchor_values(&self, ctx: &EmitContext<'_>, emit_start: f64) -> Vec<f64> {
        let anchor_t = emit_start.max(
            self.planned_fitted
                .first()
                .map_or(emit_start, |f| f.t_start),
        );
        ctx.chains
            .followers
            .iter()
            .zip(&self.follower_emit_start)
            .map(|((f_axis, _), &fallback)| {
                self.axis_position_at(*f_axis, anchor_t)
                    .or_else(|| {
                        self.axes[*f_axis]
                            .pieces
                            .back()
                            .map(|p| p.evaluate(p.u_end))
                    })
                    .unwrap_or(fallback)
            })
            .collect()
    }

    fn store_follower_ledgers(
        &mut self,
        ctx: &EmitContext<'_>,
        follower_inputs: Vec<Vec<BezierPiece<f64>>>,
    ) {
        for ((f_axis, _), new_pieces) in ctx.chains.followers.iter().zip(follower_inputs) {
            let Some(first_new) = new_pieces.first() else {
                continue;
            };
            let boundary = first_new.u_start;
            let lane = &mut self.axes[*f_axis].pieces;
            while let Some(back) = lane.back() {
                if back.u_start >= boundary - T_EPSILON {
                    lane.pop_back();
                } else if back.u_end > boundary + T_EPSILON {
                    let (left, _) = nurbs::bezier::split_piece_at(back, boundary);
                    *lane.back_mut().unwrap() = left;
                    break;
                } else {
                    break;
                }
            }
            lane.extend(new_pieces);
        }
    }
}

fn build_history_storage(axes: &[super::AxisLane], t_start: f64) -> Vec<Vec<BezierPiece<f64>>> {
    axes.iter()
        .map(|axis| {
            axis.pieces
                .iter()
                .filter(|p| p.u_start < t_start + T_EPSILON)
                .cloned()
                .collect()
        })
        .collect()
}

fn trim_per_axis_history(axes: &mut [super::AxisLane], t_dispatched: f64, max_h: f64) {
    let delta_safety = max_h;
    let trim_cutoff = t_dispatched - max_h - delta_safety;
    for axis in axes {
        while let Some(front) = axis.pieces.front() {
            if front.u_end < trim_cutoff - T_EPSILON {
                axis.pieces.pop_front();
            } else {
                break;
            }
        }
    }
}

fn shaped_axis_velocity_at(axis: &nurbs::ScalarNurbs<f64>, t: f64) -> f64 {
    let d1 = nurbs::eval::derivative(axis);
    nurbs::eval::eval(&d1, t)
}

fn restrict_segment_lo_hi(
    seg: &ShapedSegment,
    t_lo: f64,
    t_hi: f64,
) -> Result<ShapedSegment, nurbs::AlgebraError> {
    use nurbs::algebra::restrict_to_domain;

    let restricted_axes: Vec<nurbs::ScalarNurbs<f64>> = seg
        .axes
        .iter()
        .map(|axis| restrict_to_domain(axis, t_lo, t_hi))
        .collect::<Result<_, _>>()?;
    Ok(ShapedSegment {
        axes: restricted_axes,
        followers: seg.followers.clone(),
        t_start: t_lo,
        t_end: t_hi,
        motor_mask: seg.motor_mask,
    })
}
