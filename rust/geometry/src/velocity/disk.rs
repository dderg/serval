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

/// Run-anchored tangential-jerk context for one move's profile reconstruction.
/// The forward jerk ramp is measured from the run's start anchor velocity
/// `fwd_v` over cumulative arc `fwd_s + s`; the backward ramp from the run's
/// end anchor velocity `bwd_v` over `bwd_s + (length - s)`. The jerk magnitude
/// therefore only relaxes to zero at the run's rest anchors (stops / chain
/// ends), not at internal clothoid seams.
pub(super) struct JerkAnchors {
    pub fwd_v: f64,
    pub fwd_s: f64,
    pub bwd_v: f64,
    pub bwd_s: f64,
}

const BRIDGE_EPS_A: f64 = 1e-6;
const BRIDGE_MIN_ARC_FRAC: f64 = 1e-4;
const ROOT_ITERS: u32 = 80;

#[derive(Clone, Copy)]
pub(super) struct ProfilePoint {
    pub v: f64,
    pub a: f64,
}

fn disk_rail_accel(accel: f64, kappa_abs: f64, v: f64) -> f64 {
    let a_n = kappa_abs * v * v;
    (accel * accel - a_n * a_n).max(0.0).sqrt()
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
    let tau = scan_root(&gap, 0.0, tau_max, 32)?;
    let (s_end, v_end, a_end) = arc_at(tau);
    Some(Shot {
        s_end,
        v_end,
        a_end,
    })
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
    let s_star = scan_root(&gap, sa - half_max, sb + half_max, 16)?;
    let window = (3.0 * half_max).max(2.0 * (sb - sa));

    let residual = |s_left: f64| -> Option<f64> {
        let l = left(s_left)?;
        let shot = shoot(ctxs, tol, s_left, l.v, l.a, j, accel, apex <= 0.0)?;
        Some(shot.v_end - right_v(shot.s_end)?)
    };
    let s_left = scan_root(&residual, (s_star - window).max(0.0), s_star, 48)?;
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

fn reconstruct_flat(ctxs: &[MemberCtx], tol: f64) -> Option<Vec<(f64, f64, f64)>> {
    let base = base_samples(ctxs, tol)?;
    let mut bridges: Vec<(f64, f64, Vec<(f64, f64, f64)>)> = Vec::new();
    for w in base.windows(2) {
        let aa = w[0].2;
        let ab = w[1].2;
        if (aa - ab).abs() <= BRIDGE_EPS_A {
            continue;
        }
        let apex = if aa > 0.0 && ab < 0.0 {
            1.0
        } else if aa < 0.0 && ab > 0.0 {
            -1.0
        } else {
            continue;
        };
        if let Some(bridge) = build_run_bridge(ctxs, tol, w[0].0, w[1].0, apex) {
            bridges.push(bridge);
        }
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
        let mut local: Vec<(f64, f64, f64)> = flat
            .iter()
            .filter(|p| p.0 >= s0 - 1e-9 && p.0 <= s1 + 1e-9)
            .map(|p| ((p.0 - s0).clamp(0.0, c.m.kin.length), p.1, p.2))
            .collect();
        local.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12);
        if local.first().is_none_or(|p| p.0 > 1e-9) {
            let pt = run_eval(&ctxs, s0, tol)?;
            local.insert(0, (0.0, pt.v, pt.a));
        }
        if local
            .last()
            .is_none_or(|p| (p.0 - c.m.kin.length).abs() > 1e-9)
        {
            let pt = run_eval(&ctxs, s1, tol)?;
            local.push((c.m.kin.length, pt.v, pt.a));
        }
        if let Some(first) = local.first_mut() {
            first.0 = 0.0;
            first.1 = c.m.entry_v;
        }
        if let Some(last) = local.last_mut() {
            last.0 = c.m.kin.length;
            last.1 = c.m.exit_v;
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
