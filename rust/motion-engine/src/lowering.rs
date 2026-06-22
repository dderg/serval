use geometry::path::lowering::PositionProfile;
use geometry::{Move, MoveVelocity, VelSample};
use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::{CompiledChain, ShapedSegment};

const MIN_PIECE_DURATION_S: f64 = 1e-9;
const MAX_SUBDIVISION_DEPTH: u32 = 22;

const MIN_FIT_PIECE_S: f64 = 5e-4;

const RESIDUAL_PROBES: [f64; 3] = [0.25, 0.5, 0.75];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoweringError {
    SourceMismatch,
    EmptyProfile,
    DegeneratePhase,
    FollowerAxisOutOfRange { axis_index: usize },
}

impl std::fmt::Display for LoweringError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SourceMismatch => write!(f, "geometry and velocity move sources differ"),
            Self::EmptyProfile => write!(f, "velocity move has fewer than two samples"),
            Self::DegeneratePhase => write!(f, "velocity profile phase is degenerate"),
            Self::FollowerAxisOutOfRange { axis_index } => {
                write!(
                    f,
                    "follower axis {axis_index} exceeds the registry start vector"
                )
            }
        }
    }
}

impl std::error::Error for LoweringError {}

#[derive(Clone)]
struct Phase {
    t0: f64,
    dt: f64,
    s0: f64,
    v0: f64,
    accel: f64,
    s_end: f64,
}

impl Phase {
    fn s_v_at(&self, t_local: f64) -> (f64, f64) {
        let tau = (t_local - self.t0).clamp(0.0, self.dt);
        let s = self
            .v0
            .mul_add(tau, 0.5 * self.accel * tau * tau + self.s0)
            .clamp(self.s0, self.s_end);
        let v = self.accel.mul_add(tau, self.v0).max(0.0);
        (s, v)
    }
}

fn build_phases(samples: &[VelSample]) -> Result<(Vec<Phase>, f64), LoweringError> {
    if samples.len() < 2 {
        return Err(LoweringError::EmptyProfile);
    }
    let mut phases = Vec::with_capacity(samples.len() - 1);
    let mut t_acc = 0.0;
    for w in samples.windows(2) {
        let (s0, v0) = (w[0].s, w[0].v);
        let (s1, v1) = (w[1].s, w[1].v);
        let ds = s1 - s0;
        let v_sum = v0 + v1;
        if !(ds.is_finite() && ds > 0.0 && v_sum > 0.0) {
            return Err(LoweringError::DegeneratePhase);
        }
        let accel = (v1 * v1 - v0 * v0) / (2.0 * ds);
        let dt = 2.0 * ds / v_sum;
        if !(dt.is_finite() && dt > 0.0) {
            return Err(LoweringError::DegeneratePhase);
        }
        phases.push(Phase {
            t0: t_acc,
            dt,
            s0,
            v0,
            accel,
            s_end: s1,
        });
        t_acc += dt;
    }
    Ok((phases, t_acc))
}

#[derive(Clone)]
struct MoveBaseCtx {
    t_start: f64,
    total_t: f64,
    phases: Vec<Phase>,
    followers: Vec<(usize, f64)>,
    start_pos: Vec<f64>,
}

impl MoveBaseCtx {
    fn follower_base(&self, axis: usize, t_local: f64) -> (f64, f64) {
        let (s, v) = locate(&self.phases, t_local).s_v_at(t_local);
        match self.followers.iter().find(|(a, _)| *a == axis) {
            Some(&(_, ratio)) => (ratio.mul_add(s, self.start_pos[axis]), ratio * v),
            None => (self.start_pos.get(axis).copied().unwrap_or(0.0), 0.0),
        }
    }
}

fn integ_pos(b: f64, s: f64, h: f64, a: f64, c: f64) -> f64 {
    let f = |x: f64| h.mul_add(x * x * x / 3.0, s.mul_add(x * x / 2.0, b * x));
    f(c) - f(a)
}

fn integ_time_pos(b: f64, s: f64, h: f64, a: f64, c: f64) -> f64 {
    let f = |x: f64| {
        h.mul_add(
            x * x * x * x / 4.0,
            s.mul_add(x * x * x / 3.0, b * x * x / 2.0),
        )
    };
    f(c) - f(a)
}

fn quad_contrib(
    quad: (f64, f64, f64, f64),
    span: (f64, f64),
    t: f64,
    lo: f64,
    hi: f64,
) -> (f64, f64, f64) {
    let (b, s, h, tp0) = quad;
    let (a, c) = span;
    let mut weighted = 0.0;
    let mut bwd = 0.0;
    let mut fwd = 0.0;
    let back_a = a.max(lo);
    let back_c = c.min(t);
    if back_c > back_a {
        let i0 = integ_pos(b, s, h, back_a - tp0, back_c - tp0);
        let ix = tp0.mul_add(i0, integ_time_pos(b, s, h, back_a - tp0, back_c - tp0));
        weighted += lo.mul_add(-i0, ix);
        bwd += i0;
    }
    let fwd_a = a.max(t);
    let fwd_c = c.min(hi);
    if fwd_c > fwd_a {
        let i0 = integ_pos(b, s, h, fwd_a - tp0, fwd_c - tp0);
        let ix = tp0.mul_add(i0, integ_time_pos(b, s, h, fwd_a - tp0, fwd_c - tp0));
        weighted += hi.mul_add(i0, -ix);
        fwd += i0;
    }
    (weighted, bwd, fwd)
}

struct BatchExtruder {
    moves: Vec<MoveBaseCtx>,
}

impl BatchExtruder {
    fn smoothed(&self, axis: usize, pa: f64, hst: f64, t: f64) -> (f64, f64) {
        let (lo, hi) = (t - hst, t + hst);
        let mut weighted = 0.0;
        let mut bwd = 0.0;
        let mut fwd = 0.0;
        let mut add = |quad, span| {
            let (w, b, f) = quad_contrib(quad, span, t, lo, hi);
            weighted += w;
            bwd += b;
            fwd += f;
        };
        for m in &self.moves {
            match m.followers.iter().find(|(a, _)| *a == axis) {
                Some(&(_, ratio)) => {
                    for ph in &m.phases {
                        let tp0 = m.t_start + ph.t0;
                        let b = pa.mul_add(ratio * ph.v0, ratio.mul_add(ph.s0, m.start_pos[axis]));
                        let s = pa.mul_add(ratio * ph.accel, ratio * ph.v0);
                        let h = 0.5 * ratio * ph.accel;
                        add((b, s, h, tp0), (tp0, tp0 + ph.dt));
                    }
                }
                None => {
                    let b = m.start_pos.get(axis).copied().unwrap_or(0.0);
                    add((b, 0.0, 0.0, m.t_start), (m.t_start, m.t_start + m.total_t));
                }
            }
        }
        let first = &self.moves[0];
        let batch_t0 = first.t_start;
        if lo < batch_t0 {
            let (p, v) = first.follower_base(axis, 0.0);
            add((pa.mul_add(v, p), v, 0.0, batch_t0), (lo, batch_t0));
        }
        let last = &self.moves[self.moves.len() - 1];
        let batch_t1 = last.t_start + last.total_t;
        if hi > batch_t1 {
            let (p, v) = last.follower_base(axis, last.total_t);
            add((pa.mul_add(v, p), v, 0.0, batch_t1), (batch_t1, hi));
        }
        let inv = 1.0 / (hst * hst);
        (weighted * inv, (fwd - bwd) * inv)
    }
}

fn hermite_cubic(u_start: f64, u_end: f64, p0: f64, v0: f64, p1: f64, v1: f64) -> BezierPiece<f64> {
    let h = u_end - u_start;
    if h <= 0.0 {
        return BezierPiece {
            u_start,
            u_end,
            coeffs: vec![p0, 0.0, 0.0, 0.0],
        };
    }
    let a = p1 - p0 - v0 * h;
    let b = v1 - v0;
    let a2 = (3.0 * a - b * h) / (h * h);
    let a3 = (b * h - 2.0 * a) / (h * h * h);
    BezierPiece {
        u_start,
        u_end,
        coeffs: vec![p0, v0, a2, a3],
    }
}

struct Sampler<'a> {
    phases: &'a [Phase],
    spatial: Option<&'a geometry::path::Segment>,
    start_pos: &'a [f64],
    followers: &'a [geometry::FollowerDemand],
    gains: &'a [f64],
    smooth_times: &'a [f64],
    t_start: f64,
    batch: &'a BatchExtruder,
}

impl Sampler<'_> {
    fn axis_state(&self, axis: usize, t: f64, with_pa: bool) -> (f64, f64) {
        let phase = locate(self.phases, t);
        let (s, v) = phase.s_v_at(t);
        if axis < 3 {
            match self.spatial {
                Some(seg) => (seg.point_at(s)[axis], seg.heading_at(s)[axis] * v),
                None => (self.start_pos.get(axis).copied().unwrap_or(0.0), 0.0),
            }
        } else if let Some(f) = self.followers.iter().find(|f| f.axis_index == axis) {
            let pos = f.ratio.mul_add(s, self.start_pos[axis]);
            let e_dot = f.ratio * v;
            let k = if with_pa {
                self.gains.get(axis).copied().unwrap_or(0.0)
            } else {
                0.0
            };
            if k == 0.0 {
                return (pos, e_dot);
            }
            let smooth_time = self.smooth_times.get(axis).copied().unwrap_or(0.0);
            if smooth_time > 0.0 {
                self.batch
                    .smoothed(axis, k, 0.5 * smooth_time, self.t_start + t)
            } else {
                let e_ddot = f.ratio * phase.accel;
                (k.mul_add(e_dot, pos), k.mul_add(e_ddot, e_dot))
            }
        } else {
            (self.start_pos.get(axis).copied().unwrap_or(0.0), 0.0)
        }
    }

    fn span_residual(&self, driven: &[usize], ta: f64, tb: f64) -> f64 {
        let mut worst = 0.0_f64;
        for &axis in driven {
            let (pa, va) = self.axis_state(axis, ta, false);
            let (pb, vb) = self.axis_state(axis, tb, false);
            let piece = hermite_cubic(ta, tb, pa, va, pb, vb);
            for &frac in &RESIDUAL_PROBES {
                let tm = frac.mul_add(tb - ta, ta);
                let (truth, _) = self.axis_state(axis, tm, false);
                worst = worst.max((piece.evaluate(tm) - truth).abs());
            }
        }
        worst
    }
}

fn locate(phases: &[Phase], t: f64) -> &Phase {
    phases
        .iter()
        .find(|p| t < p.t0 + p.dt)
        .unwrap_or(&phases[phases.len() - 1])
}

fn refine_span(
    sampler: &Sampler<'_>,
    driven: &[usize],
    tol_mm: f64,
    ta: f64,
    tb: f64,
    depth: u32,
    out: &mut Vec<f64>,
) {
    let h = tb - ta;
    let accept = depth >= MAX_SUBDIVISION_DEPTH
        || h <= 2.0 * MIN_FIT_PIECE_S
        || sampler.span_residual(driven, ta, tb) <= tol_mm;
    if accept {
        out.push(tb);
    } else {
        let tm = 0.5 * (ta + tb);
        refine_span(sampler, driven, tol_mm, ta, tm, depth + 1, out);
        refine_span(sampler, driven, tol_mm, tm, tb, depth + 1, out);
    }
}

fn lower_one(
    gm: &Move,
    ctx: &MoveBaseCtx,
    batch: &BatchExtruder,
    fit_tol_mm: f64,
    axis_chains: &[CompiledChain],
) -> Result<ShapedSegment, LoweringError> {
    let spatial = gm.segment.spatial.as_ref();
    let n_axes = ctx.start_pos.len().max(3);
    for f in &gm.segment.followers {
        if f.axis_index >= n_axes {
            return Err(LoweringError::FollowerAxisOutOfRange {
                axis_index: f.axis_index,
            });
        }
    }

    let gains: Vec<f64> = (0..n_axes)
        .map(|axis| axis_chains.get(axis).map_or(0.0, |c| c.gain))
        .collect();
    let smooth_times: Vec<f64> = (0..n_axes)
        .map(|axis| axis_chains.get(axis).map_or(0.0, |c| c.smooth_time))
        .collect();

    let sampler = Sampler {
        phases: &ctx.phases,
        spatial,
        start_pos: &ctx.start_pos,
        followers: &gm.segment.followers,
        gains: &gains,
        smooth_times: &smooth_times,
        t_start: ctx.t_start,
        batch,
    };
    let mut driven: Vec<usize> = (0..3).collect();
    driven.extend(gm.segment.followers.iter().map(|f| f.axis_index));

    let mut bounds = vec![0.0];
    refine_span(
        &sampler,
        &driven,
        fit_tol_mm,
        0.0,
        ctx.total_t,
        0,
        &mut bounds,
    );

    let mut axes_pieces: Vec<Vec<BezierPiece<f64>>> = vec![Vec::new(); n_axes];
    for w in bounds.windows(2) {
        let (ta, tb) = (w[0], w[1]);
        if tb - ta <= MIN_PIECE_DURATION_S {
            continue;
        }
        let (ua, ub) = (ctx.t_start + ta, ctx.t_start + tb);
        for (axis, pieces) in axes_pieces.iter_mut().enumerate() {
            let (pa, va) = sampler.axis_state(axis, ta, true);
            let (pb, vb) = sampler.axis_state(axis, tb, true);
            pieces.push(hermite_cubic(ua, ub, pa, va, pb, vb));
        }
    }

    let axes: Vec<ScalarNurbs<f64>> = axes_pieces
        .iter()
        .map(|p| bezier_pieces_to_nurbs(p))
        .collect();

    Ok(ShapedSegment {
        axes,
        followers: gm.segment.followers.clone(),
        t_start: ctx.t_start,
        t_end: ctx.t_start + ctx.total_t,
        motor_mask: 0,
    })
}

fn advance_odometer(pos: &mut [f64], gm: &Move) {
    let s_len = gm.segment.s_len();
    if let Some(seg) = &gm.segment.spatial {
        let end = seg.point_at(s_len);
        for (axis, slot) in pos.iter_mut().enumerate().take(3) {
            *slot = end[axis];
        }
    }
    for f in &gm.segment.followers {
        if let Some(slot) = pos.get_mut(f.axis_index) {
            *slot += f.ratio * s_len;
        }
    }
}

pub fn lower_batch(
    moves: &[Move],
    vels: &[MoveVelocity],
    t_start: f64,
    start_pos: &[f64],
    fit_tol_mm: f64,
    axis_chains: &[CompiledChain],
) -> Result<Vec<ShapedSegment>, LoweringError> {
    if moves.len() != vels.len() {
        return Err(LoweringError::SourceMismatch);
    }
    let mut ctxs: Vec<MoveBaseCtx> = Vec::with_capacity(moves.len());
    let mut t = t_start;
    let mut pos = start_pos.to_vec();
    for (gm, vm) in moves.iter().zip(vels) {
        if gm.source != vm.source {
            return Err(LoweringError::SourceMismatch);
        }
        let (phases, total_t) = build_phases(&vm.samples)?;
        let followers = gm
            .segment
            .followers
            .iter()
            .map(|f| (f.axis_index, f.ratio))
            .collect();
        ctxs.push(MoveBaseCtx {
            t_start: t,
            total_t,
            phases,
            followers,
            start_pos: pos.clone(),
        });
        t += total_t;
        advance_odometer(&mut pos, gm);
    }

    let batch = BatchExtruder { moves: ctxs };
    moves
        .iter()
        .enumerate()
        .map(|(i, gm)| lower_one(gm, &batch.moves[i], &batch, fit_tol_mm, axis_chains))
        .collect()
}

pub fn lower_move(
    gm: &Move,
    vm: &MoveVelocity,
    t_start: f64,
    start_pos: &[f64],
    fit_tol_mm: f64,
    axis_chains: &[CompiledChain],
) -> Result<ShapedSegment, LoweringError> {
    let mut segs = lower_batch(
        std::slice::from_ref(gm),
        std::slice::from_ref(vm),
        t_start,
        start_pos,
        fit_tol_mm,
        axis_chains,
    )?;
    Ok(segs.remove(0))
}

#[cfg(test)]
mod tests;
