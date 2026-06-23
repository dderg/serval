use std::f64::consts::FRAC_PI_2;

use super::scurve;

const RK_MIN_STEP_FRAC: f64 = 1e-6;
const RK_MAX_STEPS: u32 = 100_000;
const SAMPLE_MAX_DEPTH: u32 = 24;
const SAMPLE_MAX_POINTS: usize = 16_384;

pub(super) struct Kinematics {
    pub length: f64,
    pub accel: f64,
    pub jerk: f64,
    pub kappa0: f64,
    pub sigma: f64,
    pub flat_ceiling: f64,
}

impl Kinematics {
    fn reversed(&self) -> Kinematics {
        Kinematics {
            length: self.length,
            accel: self.accel,
            jerk: self.jerk,
            kappa0: self.kappa0 + self.sigma * self.length,
            sigma: -self.sigma,
            flat_ceiling: self.flat_ceiling,
        }
    }

    fn kappa_abs(&self, s: f64) -> f64 {
        (self.kappa0 + self.sigma * s).abs()
    }

    fn ceiling_w(&self, s: f64) -> f64 {
        let flat_w = self.flat_ceiling * self.flat_ceiling;
        let k = self.kappa_abs(s);
        if k > 0.0 {
            flat_w.min(self.accel / k)
        } else {
            flat_w
        }
    }
}

pub(super) fn limit_speed(kappa_abs: f64, accel: f64) -> f64 {
    if kappa_abs > 0.0 {
        (accel / kappa_abs).sqrt()
    } else {
        f64::INFINITY
    }
}

pub(super) fn const_kappa_reach_w(w_in: f64, length: f64, accel: f64, kappa_abs: f64) -> f64 {
    if kappa_abs == 0.0 {
        return w_in + 2.0 * accel * length;
    }
    let w_lim = accel / kappa_abs;
    let x0 = (w_in / w_lim).clamp(0.0, 1.0);
    let arg = 2.0 * kappa_abs * length + x0.asin();
    if arg >= FRAC_PI_2 {
        w_lim
    } else {
        w_lim * arg.sin()
    }
}

fn rk4_step<F: Fn(f64, f64) -> f64>(f: &F, s: f64, w: f64, h: f64) -> f64 {
    let k1 = f(s, w);
    let k2 = f(s + 0.5 * h, w + 0.5 * h * k1);
    let k3 = f(s + 0.5 * h, w + 0.5 * h * k2);
    let k4 = f(s + h, w + h * k3);
    w + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
}

fn integrate_ode(kin: &Kinematics, w_in: f64, target: f64, tol: f64) -> Option<f64> {
    let f = |s: f64, w: f64| {
        let k = kin.kappa0 + kin.sigma * s;
        2.0 * (kin.accel * kin.accel - (k * k) * (w * w)).max(0.0).sqrt()
    };
    let min_h = (kin.length * RK_MIN_STEP_FRAC).max(f64::MIN_POSITIVE);
    let mut s = 0.0;
    let mut w = w_in.min(kin.ceiling_w(0.0)).max(0.0);
    let mut next_h = target;
    let mut steps = 0u32;
    loop {
        if s >= target {
            return Some(w);
        }
        if steps >= RK_MAX_STEPS {
            return None;
        }
        let mut h = next_h.min(target - s);
        loop {
            let big = rk4_step(&f, s, w, h);
            let half = rk4_step(&f, s, w, 0.5 * h);
            let small = rk4_step(&f, s + 0.5 * h, half, 0.5 * h);
            let err = (big - small).abs();
            let bound = tol * (1.0 + small.abs());
            if err <= bound || h <= min_h {
                s += h;
                w = small.min(kin.ceiling_w(s)).max(0.0);
                next_h = if err < 0.1 * bound { 2.0 * h } else { h };
                break;
            }
            h *= 0.5;
        }
        steps += 1;
    }
}

fn disk_reach_w(kin: &Kinematics, w_in: f64, s: f64, tol: f64) -> Option<f64> {
    if kin.sigma == 0.0 {
        let flat_w = kin.flat_ceiling * kin.flat_ceiling;
        Some(const_kappa_reach_w(w_in, s, kin.accel, kin.kappa0.abs()).min(flat_w))
    } else {
        integrate_ode(kin, w_in, s, tol)
    }
}

pub(super) fn disk_reach_v(kin: &Kinematics, v_in: f64, s: f64, tol: f64) -> Option<f64> {
    Some(disk_reach_w(kin, v_in * v_in, s, tol)?.max(0.0).sqrt())
}

pub(super) fn disk_reach_v_rev(kin: &Kinematics, v_in: f64, s: f64, tol: f64) -> Option<f64> {
    disk_reach_v(&kin.reversed(), v_in, s, tol)
}

pub(super) struct JerkAnchors {
    pub fwd_v: f64,
    pub fwd_s: f64,
    pub bwd_v: f64,
    pub bwd_s: f64,
}

const BRIDGE_EPS_A: f64 = 1e-6;
const BRIDGE_MIN_ARC_FRAC: f64 = 1e-4;
const ROOT_ITERS: u32 = 80;
const CEILING_V_EPS: f64 = 1e-6;

#[derive(Clone, Copy, PartialEq, Eq)]
enum BridgeKind {
    SignFlip,
    CeilingEntry,
    CeilingExit,
}

impl BridgeKind {
    fn is_ceiling(self) -> bool {
        matches!(self, BridgeKind::CeilingEntry | BridgeKind::CeilingExit)
    }

    // The entry roll-off departs the forward branch below the cruise rail (the
    // rising accel ramp); the exit roll-off departs the rail itself.
    fn departs_below_ceiling(self) -> bool {
        matches!(self, BridgeKind::CeilingEntry)
    }
}

#[derive(Clone, Copy)]
pub(super) struct ProfilePoint {
    pub v: f64,
    pub a: f64,
}

fn disk_rail_accel(accel: f64, kappa_abs: f64, v: f64) -> f64 {
    let a_n = kappa_abs * v * v;
    (accel * accel - a_n * a_n).max(0.0).sqrt()
}

fn clamp_to_disk(a_t: f64, accel: f64, kappa_abs: f64, v: f64) -> f64 {
    let budget = disk_rail_accel(accel, kappa_abs, v);
    a_t.clamp(-budget, budget)
}

fn forward_seg(kin: &Kinematics, jerk: &JerkAnchors) -> Option<scurve::SevenSeg> {
    scurve::breakpoints(
        jerk.fwd_v,
        0.0,
        jerk.fwd_s + kin.length,
        kin.accel,
        kin.jerk,
    )
    .ok()
}

fn backward_seg(kin: &Kinematics, jerk: &JerkAnchors) -> Option<scurve::SevenSeg> {
    scurve::breakpoints(
        jerk.bwd_v,
        0.0,
        jerk.bwd_s + kin.length,
        kin.accel,
        kin.jerk,
    )
    .ok()
}

const KAPPA_EPS: f64 = 1e-9;

fn curvature_ceiling_accel(kin: &Kinematics, s: f64) -> f64 {
    let kappa = kin.kappa0 + kin.sigma * s;
    let kappa_abs = kappa.abs();
    if kappa_abs <= KAPPA_EPS {
        return 0.0;
    }
    let a = -0.5 * kin.accel * kin.sigma * kappa.signum() / (kappa_abs * kappa_abs);
    a.clamp(-kin.accel, kin.accel)
}

fn ceiling_accel(kin: &Kinematics, s: f64) -> f64 {
    let v_curv = limit_speed(kin.kappa_abs(s), kin.accel);
    if v_curv <= kin.flat_ceiling {
        curvature_ceiling_accel(kin, s)
    } else {
        0.0
    }
}

fn forward_branch(
    kin: &Kinematics,
    entry: f64,
    seg: &scurve::SevenSeg,
    jerk: &JerkAnchors,
    s: f64,
    tol: f64,
) -> Option<ProfilePoint> {
    let v_disk = disk_reach_v(kin, entry, s, tol)?;
    let v_jerk = scurve::velocity_at(seg, jerk.fwd_s + s);
    let ceil = kin
        .flat_ceiling
        .min(limit_speed(kin.kappa_abs(s), kin.accel));
    let v = v_disk.min(v_jerk).min(ceil);
    let a = if v_jerk <= v_disk && v_jerk <= ceil {
        scurve::accel_at(seg, jerk.fwd_s + s)
    } else if v >= ceil - tol * (1.0 + ceil) {
        ceiling_accel(kin, s)
    } else {
        disk_rail_accel(kin.accel, kin.kappa_abs(s), v_disk)
    };
    Some(ProfilePoint { v, a })
}

fn backward_branch(
    kin: &Kinematics,
    exit: f64,
    seg: &scurve::SevenSeg,
    jerk: &JerkAnchors,
    s: f64,
    tol: f64,
) -> Option<ProfilePoint> {
    let rest = kin.length - s;
    let v_disk = disk_reach_v(&kin.reversed(), exit, rest, tol)?;
    let v_jerk = scurve::velocity_at(seg, jerk.bwd_s + rest);
    let ceil = kin
        .flat_ceiling
        .min(limit_speed(kin.kappa_abs(s), kin.accel));
    let v = v_disk.min(v_jerk).min(ceil);
    let a = if v_jerk <= v_disk && v_jerk <= ceil {
        -scurve::accel_at(seg, jerk.bwd_s + rest)
    } else if v >= ceil - tol * (1.0 + ceil) {
        ceiling_accel(kin, s)
    } else {
        -disk_rail_accel(kin.accel, kin.kappa_abs(s), v_disk)
    };
    Some(ProfilePoint { v, a })
}

fn eval_profile(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    fwd: &scurve::SevenSeg,
    bwd: &scurve::SevenSeg,
    jerk: &JerkAnchors,
    s: f64,
    tol: f64,
) -> Option<ProfilePoint> {
    let f = forward_branch(kin, entry, fwd, jerk, s, tol)?;
    let b = backward_branch(kin, exit, bwd, jerk, s, tol)?;
    Some(if f.v <= b.v { f } else { b })
}

fn profile_speed(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    jerk: &JerkAnchors,
    s: f64,
    tol: f64,
) -> Option<f64> {
    let fwd = forward_seg(kin, jerk)?;
    let bwd = backward_seg(kin, jerk)?;
    Some(eval_profile(kin, entry, exit, &fwd, &bwd, jerk, s, tol)?.v)
}

#[allow(clippy::too_many_arguments)]
fn refine(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    jerk: &JerkAnchors,
    tol: f64,
    s0: f64,
    v0: f64,
    s1: f64,
    v1: f64,
    depth: u32,
    out: &mut Vec<(f64, f64)>,
) -> Option<()> {
    let mid = 0.5 * (s0 + s1);
    let actual = profile_speed(kin, entry, exit, jerk, mid, tol)?;
    let interp = 0.5 * (v0 + v1);
    let needs_refine = (actual - interp).abs() > tol * (1.0 + actual.abs());
    if needs_refine && depth < SAMPLE_MAX_DEPTH && out.len() < SAMPLE_MAX_POINTS {
        refine(
            kin,
            entry,
            exit,
            jerk,
            tol,
            s0,
            v0,
            mid,
            actual,
            depth + 1,
            out,
        )?;
        out.push((mid, actual));
        refine(
            kin,
            entry,
            exit,
            jerk,
            tol,
            mid,
            actual,
            s1,
            v1,
            depth + 1,
            out,
        )?;
    } else if needs_refine {
        out.push((mid, actual));
    }
    Some(())
}

pub(super) struct RunMember<'a> {
    pub kin: &'a Kinematics,
    pub entry_v: f64,
    pub exit_v: f64,
    pub fwd_s: f64,
    pub bwd_s: f64,
}

struct MemberCtx<'a> {
    m: &'a RunMember<'a>,
    fwd_seg: scurve::SevenSeg,
    bwd_seg: scurve::SevenSeg,
    jerk: JerkAnchors,
}

fn build_ctxs<'a>(members: &'a [RunMember<'a>], run_start_v: f64) -> Option<Vec<MemberCtx<'a>>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        let jerk = JerkAnchors {
            fwd_v: run_start_v,
            fwd_s: m.fwd_s,
            bwd_v: 0.0,
            bwd_s: m.bwd_s,
        };
        let fwd_seg = forward_seg(m.kin, &jerk)?;
        let bwd_seg = backward_seg(m.kin, &jerk)?;
        out.push(MemberCtx {
            m,
            fwd_seg,
            bwd_seg,
            jerk,
        });
    }
    Some(out)
}

fn locate(ctxs: &[MemberCtx], s_run: f64) -> (usize, f64) {
    let mut i = 0;
    for (k, c) in ctxs.iter().enumerate() {
        if s_run + 1e-12 >= c.m.fwd_s {
            i = k;
        } else {
            break;
        }
    }
    let local = (s_run - ctxs[i].m.fwd_s).clamp(0.0, ctxs[i].m.kin.length);
    (i, local)
}

fn run_forward(ctxs: &[MemberCtx], s_run: f64, tol: f64) -> Option<ProfilePoint> {
    let (i, local) = locate(ctxs, s_run);
    let c = &ctxs[i];
    forward_branch(c.m.kin, c.m.entry_v, &c.fwd_seg, &c.jerk, local, tol)
}

fn run_backward(ctxs: &[MemberCtx], s_run: f64, tol: f64) -> Option<ProfilePoint> {
    let (i, local) = locate(ctxs, s_run);
    let c = &ctxs[i];
    backward_branch(c.m.kin, c.m.exit_v, &c.bwd_seg, &c.jerk, local, tol)
}

fn run_eval(ctxs: &[MemberCtx], s_run: f64, tol: f64) -> Option<ProfilePoint> {
    let (i, local) = locate(ctxs, s_run);
    let c = &ctxs[i];
    eval_profile(
        c.m.kin,
        c.m.entry_v,
        c.m.exit_v,
        &c.fwd_seg,
        &c.bwd_seg,
        &c.jerk,
        local,
        tol,
    )
}

fn base_samples(ctxs: &[MemberCtx], tol: f64) -> Option<Vec<(f64, f64, f64)>> {
    let mut out: Vec<(f64, f64, f64)> = Vec::new();
    for c in ctxs {
        let mut sv = vec![(0.0, c.m.entry_v)];
        refine(
            c.m.kin,
            c.m.entry_v,
            c.m.exit_v,
            &c.jerk,
            tol,
            0.0,
            c.m.entry_v,
            c.m.kin.length,
            c.m.exit_v,
            0,
            &mut sv,
        )?;
        sv.push((c.m.kin.length, c.m.exit_v));
        for (sl, v) in sv {
            let s_run = c.m.fwd_s + sl;
            if out.last().is_some_and(|p| (p.0 - s_run).abs() < 1e-12) {
                continue;
            }
            let a = eval_profile(
                c.m.kin,
                c.m.entry_v,
                c.m.exit_v,
                &c.fwd_seg,
                &c.bwd_seg,
                &c.jerk,
                sl,
                tol,
            )?
            .a;
            out.push((s_run, v, a));
        }
    }
    Some(out)
}

struct Shot {
    s_end: f64,
    v_end: f64,
    a_end: f64,
}

fn shoot(
    ctxs: &[MemberCtx],
    tol: f64,
    s_left: f64,
    v0: f64,
    a0: f64,
    j: f64,
    accel_bound: f64,
    right_forward: bool,
) -> Option<Shot> {
    let arc_at = |tau: f64| -> (f64, f64, f64) {
        let a = a0 + j * tau;
        let v = v0 + a0 * tau + 0.5 * j * tau * tau;
        let s = s_left + v0 * tau + 0.5 * a0 * tau * tau + (1.0 / 6.0) * j * tau * tau * tau;
        (s, v, a)
    };
    let right_a = |s: f64| -> Option<f64> {
        if right_forward {
            run_forward(ctxs, s, tol)
        } else {
            run_backward(ctxs, s, tol)
        }
        .map(|p| p.a)
    };
    let gap = |tau: f64| -> Option<f64> {
        let (s, _, a) = arc_at(tau);
        Some(a - right_a(s)?)
    };
    let tau_max = 1.05 * (a0.abs() + accel_bound) / j.abs();
    let tau = scan_cross(&gap, 0.0, tau_max, 32, j < 0.0)?;
    let (s_end, v_end, a_end) = arc_at(tau);
    Some(Shot {
        s_end,
        v_end,
        a_end,
    })
}

/// Bracket the first zero of `f` crossed in a chosen direction: `descending`
/// keeps the `+ -> -` crossing, otherwise the `- -> +` one. The `shoot` arc's
/// accel moves monotonically with `sign(j)`, so its genuine landing on the
/// target branch crosses in that direction; a discontinuity in the target
/// (e.g. the backward branch's own cruise-exit step) throws an opposite-sign
/// crossing that this rejects.
fn scan_cross<F: Fn(f64) -> Option<f64>>(
    f: &F,
    x0: f64,
    x1: f64,
    k: usize,
    descending: bool,
) -> Option<f64> {
    let mut prev_x = x0;
    let mut prev = f(x0);
    for i in 1..=k {
        let x = x0 + (x1 - x0) * (i as f64) / (k as f64);
        let cur = f(x);
        if let (Some(p), Some(c)) = (prev, cur) {
            let crosses = if descending {
                p > 0.0 && c < 0.0
            } else {
                p < 0.0 && c > 0.0
            };
            if crosses {
                let (mut lo, mut hi) = (prev_x, x);
                for _ in 0..ROOT_ITERS {
                    let mid = 0.5 * (lo + hi);
                    match f(mid) {
                        Some(m) if (m > 0.0) == descending => lo = mid,
                        Some(_) => hi = mid,
                        None => return None,
                    }
                }
                return Some(0.5 * (lo + hi));
            }
        }
        prev_x = x;
        prev = cur;
    }
    None
}

fn scan_root<F: Fn(f64) -> Option<f64>>(f: &F, x0: f64, x1: f64, k: usize) -> Option<f64> {
    let mut prev_x = x0;
    let mut prev = f(x0);
    if matches!(prev, Some(v) if v == 0.0) {
        return Some(x0);
    }
    for i in 1..=k {
        let x = x0 + (x1 - x0) * (i as f64) / (k as f64);
        let cur = f(x);
        if let (Some(p), Some(c)) = (prev, cur) {
            if c == 0.0 {
                return Some(x);
            }
            if p * c < 0.0 {
                let (mut lo, mut hi) = (prev_x, x);
                let s_lo = p.signum();
                for _ in 0..ROOT_ITERS {
                    let mid = 0.5 * (lo + hi);
                    match f(mid) {
                        Some(m) if m.signum() == s_lo => lo = mid,
                        Some(_) => hi = mid,
                        None => return None,
                    }
                }
                return Some(0.5 * (lo + hi));
            }
        }
        prev_x = x;
        prev = cur;
    }
    None
}

fn build_run_bridge(
    ctxs: &[MemberCtx],
    tol: f64,
    sa: f64,
    sb: f64,
    apex: f64,
    kind: BridgeKind,
) -> Option<(f64, f64, Vec<(f64, f64, f64)>)> {
    let (mid_i, _) = locate(ctxs, 0.5 * (sa + sb));
    let jerk_mag = ctxs[mid_i].m.kin.jerk;
    let accel = ctxs[mid_i].m.kin.accel;
    if !jerk_mag.is_finite() {
        return None;
    }
    let j = if apex > 0.0 { -jerk_mag } else { jerk_mag };

    let left = |s: f64| -> Option<ProfilePoint> {
        if apex > 0.0 {
            run_forward(ctxs, s, tol)
        } else {
            run_backward(ctxs, s, tol)
        }
    };
    let right_v = |s: f64| -> Option<f64> {
        if apex > 0.0 {
            run_backward(ctxs, s, tol)
        } else {
            run_forward(ctxs, s, tol)
        }
        .map(|p| p.v)
    };

    let gap = |s: f64| -> Option<f64> { Some(left(s)?.v - right_v(s)?) };
    let half_max = {
        let l = left(0.5 * (sa + sb)).map_or(1.0, |p| p.v);
        l.max(1.0) * (2.0 * accel / jerk_mag)
    };
    let fc = ctxs[mid_i].m.kin.flat_ceiling;
    let s_star = match kind {
        BridgeKind::SignFlip => scan_root(&gap, sa - half_max, sb + half_max, 16)?,
        _ => 0.5 * (sa + sb),
    };
    let window = (3.0 * half_max).max(2.0 * (sb - sa));

    // A ceiling roll-off departs the forward branch on a fixed side of the
    // cruise rail: the entry leaves the rising accel ramp (`v < flat_ceiling`),
    // the exit leaves the rail itself (`v >= flat_ceiling`). Pinning `s_left` to
    // that side selects the single physical root and lands the arc past the
    // transition `s_star`, never on the opposite edge of the plateau.
    let on_departure_side = |v: f64| -> bool {
        if !kind.is_ceiling() {
            return true;
        }
        let below = v < fc - CEILING_V_EPS * (1.0 + fc);
        below == kind.departs_below_ceiling()
    };
    let residual = |s_left: f64| -> Option<f64> {
        let l = left(s_left)?;
        if !on_departure_side(l.v) {
            return None;
        }
        let shot = shoot(ctxs, tol, s_left, l.v, l.a, j, accel, apex <= 0.0)?;
        if kind.is_ceiling() && shot.s_end < s_star {
            return None;
        }
        Some(shot.v_end - right_v(shot.s_end)?)
    };
    let s_left = match kind {
        BridgeKind::SignFlip => scan_root(&residual, (s_star - window).max(0.0), s_star, 48)?,
        _ => scan_root(&residual, s_star, (s_star - window).max(0.0), 48)?,
    };
    let l = left(s_left)?;
    let shot = shoot(ctxs, tol, s_left, l.v, l.a, j, accel, apex <= 0.0)?;
    let s_right = shot.s_end;
    if s_right <= s_left || s_right - s_left < BRIDGE_MIN_ARC_FRAC * (sb - sa).max(1e-6) {
        return None;
    }

    let a0 = l.a;
    let v0 = l.v;
    let total_tau = (shot.a_end - a0) / j;
    let n = 48usize;
    let mut pts = Vec::with_capacity(n + 1);
    for k in 0..=n {
        let tau = total_tau * (k as f64) / (n as f64);
        let a = a0 + j * tau;
        let v = v0 + a0 * tau + 0.5 * j * tau * tau;
        let s = s_left + v0 * tau + 0.5 * a0 * tau * tau + (1.0 / 6.0) * j * tau * tau * tau;
        let env = {
            let fv = run_forward(ctxs, s, tol)?.v;
            let bv = run_backward(ctxs, s, tol)?.v;
            fv.min(bv)
        };
        if v > env + 1e-6 * (1.0 + env) {
            return None;
        }
        pts.push((s, v, a));
    }
    Some((s_left, s_right, pts))
}

fn at_flat_ceiling(ctxs: &[MemberCtx], s: f64, v: f64) -> bool {
    let (i, _) = locate(ctxs, s);
    let fc = ctxs[i].m.kin.flat_ceiling;
    v >= fc - CEILING_V_EPS * (1.0 + fc)
}

struct Transition {
    sa: f64,
    sb: f64,
    apex: f64,
    kind: BridgeKind,
}

fn reconstruct_flat(ctxs: &[MemberCtx], tol: f64) -> Option<Vec<(f64, f64, f64)>> {
    let base = base_samples(ctxs, tol)?;
    let mut trans: Vec<Transition> = Vec::new();
    for w in base.windows(2) {
        let aa = w[0].2;
        let ab = w[1].2;
        if (aa - ab).abs() <= BRIDGE_EPS_A {
            continue;
        }
        let (apex, kind) = if aa > 0.0 && ab < 0.0 {
            (1.0, BridgeKind::SignFlip)
        } else if aa < 0.0 && ab > 0.0 {
            (-1.0, BridgeKind::SignFlip)
        } else if aa > BRIDGE_EPS_A
            && ab.abs() <= BRIDGE_EPS_A
            && at_flat_ceiling(ctxs, w[1].0, w[1].1)
        {
            (1.0, BridgeKind::CeilingEntry)
        } else if aa.abs() <= BRIDGE_EPS_A
            && ab < -BRIDGE_EPS_A
            && at_flat_ceiling(ctxs, w[0].0, w[0].1)
        {
            (1.0, BridgeKind::CeilingExit)
        } else {
            continue;
        };
        trans.push(Transition {
            sa: w[0].0,
            sb: w[1].0,
            apex,
            kind,
        });
    }

    let mut bridges: Vec<(f64, f64, Vec<(f64, f64, f64)>)> = Vec::new();
    let mut t = 0;
    while t < trans.len() {
        let cur = &trans[t];
        // A cruise entry immediately followed by its exit on a plateau too short
        // to hold both roll-offs is a move the base sweep reports as reaching
        // `flat_ceiling` but which is not jerk-feasible there (it needs >2x the
        // jerk-limited accel distance). No valid arc lands on that over-reported
        // ceiling, so the roll-offs overlap; leaving both out keeps the base
        // profile rather than emitting interleaved garbage. Realistic high-jerk
        // configs never hit this (roll-offs are sub-millimetre); the real fix is
        // jerk-aware peak estimation in the velocity sweep — tracked separately.
        if cur.kind == BridgeKind::CeilingEntry
            && trans
                .get(t + 1)
                .is_some_and(|n| n.kind == BridgeKind::CeilingExit)
        {
            let exit = &trans[t + 1];
            let entry_arc =
                build_run_bridge(ctxs, tol, cur.sa, cur.sb, 1.0, BridgeKind::CeilingEntry);
            let exit_arc =
                build_run_bridge(ctxs, tol, exit.sa, exit.sb, 1.0, BridgeKind::CeilingExit);
            let overlaps = matches!((&entry_arc, &exit_arc), (Some(e), Some(x)) if e.1 > x.0);
            if !overlaps {
                bridges.extend(entry_arc);
                bridges.extend(exit_arc);
            }
            t += 2;
            continue;
        }
        if let Some(bridge) = build_run_bridge(ctxs, tol, cur.sa, cur.sb, cur.apex, cur.kind) {
            bridges.push(bridge);
        }
        t += 1;
    }

    let mut out: Vec<(f64, f64, f64)> = base
        .into_iter()
        .filter(|p| !bridges.iter().any(|(lo, hi, _)| p.0 > *lo && p.0 < *hi))
        .collect();
    for (_, _, pts) in bridges {
        out.extend(pts);
    }
    out.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
    Some(out)
}

/// Linearly interpolate the reconstructed (bridged) flat profile at run-arc `s`.
/// Member boundaries must read this, not the base `run_eval`: when a ceiling
/// roll-off arc straddles a collinear junction, `run_eval` would return the
/// un-bridged cruise rail (`a=0`) and splice that step back into the arc.
fn interp_flat(flat: &[(f64, f64, f64)], s: f64) -> Option<(f64, f64)> {
    let first = flat.first()?;
    let last = flat.last()?;
    if s <= first.0 {
        return Some((first.1, first.2));
    }
    if s >= last.0 {
        return Some((last.1, last.2));
    }
    let i = flat.partition_point(|p| p.0 < s);
    let (lo, hi) = (flat[i - 1], flat[i]);
    let span = hi.0 - lo.0;
    let t = if span > 1e-12 { (s - lo.0) / span } else { 0.0 };
    Some((lo.1 + t * (hi.1 - lo.1), lo.2 + t * (hi.2 - lo.2)))
}

pub(super) fn reconstruct_run(
    members: &[RunMember],
    run_start_v: f64,
    tol: f64,
) -> Option<Vec<Vec<(f64, f64, f64)>>> {
    let ctxs = build_ctxs(members, run_start_v)?;
    let flat = reconstruct_flat(&ctxs, tol)?;

    let mut per_member: Vec<Vec<(f64, f64, f64)>> = Vec::with_capacity(ctxs.len());
    for c in &ctxs {
        let s0 = c.m.fwd_s;
        let s1 = c.m.fwd_s + c.m.kin.length;
        // Boundary samples read the reconstructed (bridged) profile, not the
        // nominal junction velocity `entry_v`/`exit_v`: a roll-off straddling a
        // collinear junction dips below the cruise speed, so pinning the nominal
        // there would spike `v` back to the ceiling. Off a bridge these agree.
        let (v0, _) = interp_flat(&flat, s0)?;
        let (v1, _) = interp_flat(&flat, s1)?;
        let mut local: Vec<(f64, f64, f64)> = flat
            .iter()
            .filter(|p| p.0 >= s0 - 1e-9 && p.0 <= s1 + 1e-9)
            .map(|p| ((p.0 - s0).clamp(0.0, c.m.kin.length), p.1, p.2))
            .collect();
        local.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12);
        if local.first().is_none_or(|p| p.0 > 1e-9) {
            let (v, a) = interp_flat(&flat, s0)?;
            local.insert(0, (0.0, v, a));
        }
        if local
            .last()
            .is_none_or(|p| (p.0 - c.m.kin.length).abs() > 1e-9)
        {
            let (v, a) = interp_flat(&flat, s1)?;
            local.push((c.m.kin.length, v, a));
        }
        if let Some(first) = local.first_mut() {
            first.0 = 0.0;
            first.1 = v0;
        }
        if let Some(last) = local.last_mut() {
            last.0 = c.m.kin.length;
            last.1 = v1;
        }
        for p in &mut local {
            p.2 = clamp_to_disk(p.2, c.m.kin.accel, c.m.kin.kappa_abs(p.0), p.1);
        }
        per_member.push(local);
    }
    Some(per_member)
}

#[cfg(test)]
pub(super) fn sample_profile(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    anchors: &JerkAnchors,
    tol: f64,
) -> Option<Vec<(f64, f64, f64)>> {
    let member = RunMember {
        kin,
        entry_v: entry,
        exit_v: exit,
        fwd_s: anchors.fwd_s,
        bwd_s: anchors.bwd_s,
    };
    reconstruct_run(&[member], anchors.fwd_v, tol)?
        .into_iter()
        .next()
}

#[cfg(test)]
mod tests;
