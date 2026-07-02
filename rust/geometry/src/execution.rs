use crate::path::Segment;
use crate::path::lowering::{LoweredSample, PositionProfile};
use crate::path::{Arc, Clothoid, Line, PathSegment};
use crate::{FitOutcome, GeometryError, LENGTH_EPS_MM, VelocityProfile};

struct Phase<'a> {
    seg: &'a PathSegment,
    t_start: f64,
    s_start: f64,
    v_start: f64,
    accel: f64,
    jerk: f64,
    s_end: f64,
}

pub fn lower_profile(
    geometry: &FitOutcome,
    profile: &VelocityProfile,
    rate_hz: f64,
) -> Result<Vec<LoweredSample>, GeometryError> {
    if !(rate_hz.is_finite() && rate_hz > 0.0) {
        return Err(GeometryError::InvalidLowering {
            reason: "rate must be finite and positive",
        });
    }
    if geometry.moves.len() != profile.moves.len() {
        return Err(GeometryError::InvalidLowering {
            reason: "geometry and profile move counts differ",
        });
    }
    if geometry.moves.is_empty() {
        return Ok(Vec::new());
    }

    let mut phases = Vec::new();
    let mut t_acc = 0.0;
    for (gm, pm) in geometry.moves.iter().zip(&profile.moves) {
        if gm.source != pm.source {
            return Err(GeometryError::InvalidLowering {
                reason: "geometry and profile move sources differ",
            });
        }
        let seg = &gm.segment;
        if let Some(spatial) = &seg.spatial {
            if !spatial_anchors_finite(spatial) {
                return Err(GeometryError::InvalidLowering {
                    reason: "spatial anchor is not finite",
                });
            }
        }
        let s_len = seg.s_len();
        if !(s_len.is_finite() && s_len > 0.0) {
            return Err(GeometryError::InvalidLowering {
                reason: "segment length is not finite and positive",
            });
        }
        // A straight move carries its exact closed-form jerk phases: lower one
        // per phase so the timing matches the planner and the v = 0 ends are not
        // mistimed by the sampled `2ds/(v0+v1)` estimate.
        if !pm.phases.is_empty() {
            for (i, ph) in pm.phases.iter().enumerate() {
                let s_end = pm.phases.get(i + 1).map_or(s_len, |next| next.s0);
                phases.push(Phase {
                    seg,
                    t_start: t_acc + ph.t0,
                    s_start: ph.s0,
                    v_start: ph.v0,
                    accel: ph.a0,
                    jerk: ph.j,
                    s_end,
                });
            }
            t_acc += pm.phases.iter().map(|p| p.dt).sum::<f64>();
            continue;
        }

        let samples = &pm.samples;
        if samples.len() < 2
            || samples[0].s.abs() > LENGTH_EPS_MM
            || (samples[samples.len() - 1].s - s_len).abs() > LENGTH_EPS_MM * s_len.max(1.0)
        {
            return Err(GeometryError::InvalidLowering {
                reason: "profile samples must span both segment endpoints",
            });
        }
        for w in samples.windows(2) {
            let (s0, v0) = (w[0].s, w[0].v);
            let (s1, v1) = (w[1].s, w[1].v);
            if !(s0.is_finite() && s1.is_finite() && v0.is_finite() && v1.is_finite()) {
                return Err(GeometryError::InvalidLowering {
                    reason: "profile sample is not finite",
                });
            }
            if v0 < 0.0 || v1 < 0.0 {
                return Err(GeometryError::InvalidLowering {
                    reason: "profile sample velocity is negative",
                });
            }
            let ds = s1 - s0;
            if ds <= 0.0 {
                return Err(GeometryError::InvalidLowering {
                    reason: "profile samples must strictly increase in arc length",
                });
            }
            let v_sum = v0 + v1;
            if v_sum <= 0.0 {
                return Err(GeometryError::InvalidLowering {
                    reason: "profile is stalled at zero velocity over a positive interval",
                });
            }
            phases.push(Phase {
                seg,
                t_start: t_acc,
                s_start: s0,
                v_start: v0,
                accel: (v1 * v1 - v0 * v0) / (2.0 * ds),
                jerk: 0.0,
                s_end: s1,
            });
            t_acc += 2.0 * ds / v_sum;
        }
    }

    let total_t = t_acc;
    let dt = 1.0 / rate_hz;
    let count = total_t / dt;
    if !count.is_finite() || count + 1.0 >= usize::MAX as f64 {
        return Err(GeometryError::InvalidLowering {
            reason: "lowered sample count exceeds addressable range",
        });
    }
    let n = count.ceil() as usize;

    let mut out = Vec::with_capacity(n + 1);
    let mut phase = 0usize;
    for k in 0..=n {
        let t = (k as f64 * dt).min(total_t);
        while phase + 1 < phases.len() && t >= phases[phase + 1].t_start {
            phase += 1;
        }
        let p = &phases[phase];
        let dt_local = t - p.t_start;
        let s = (p.s_start
            + p.v_start * dt_local
            + 0.5 * p.accel * dt_local * dt_local
            + p.jerk * dt_local * dt_local * dt_local / 6.0)
            .clamp(p.s_start, p.s_end);
        let position = p.seg.spatial.as_ref().map(|spatial| spatial.point_at(s));
        let followers = p.seg.followers.iter().map(|f| f.ratio * s).collect();
        out.push(LoweredSample {
            t_s: t,
            position,
            followers,
        });
    }
    Ok(out)
}

fn spatial_anchors_finite(seg: &Segment) -> bool {
    match seg {
        Segment::Line(Line { start, end }) => start.iter().chain(end.iter()).all(|c| c.is_finite()),
        Segment::Arc(Arc {
            origin,
            start_angle,
            ..
        }) => start_angle.is_finite() && origin.iter().all(|c| c.is_finite()),
        Segment::Clothoid(Clothoid { start_pose, .. }) => start_pose.iter().all(|p| p.is_finite()),
    }
}

#[cfg(test)]
mod tests;
