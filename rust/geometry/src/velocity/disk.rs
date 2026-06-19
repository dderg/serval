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

fn profile_speed(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    jerk: &JerkAnchors,
    s: f64,
    tol: f64,
) -> Option<f64> {
    let rest = kin.length - s;
    let forward = disk_reach_v(kin, entry, s, tol)?;
    let backward = disk_reach_v(&kin.reversed(), exit, rest, tol)?;
    let jerk_forward =
        scurve::max_reachable_velocity(jerk.fwd_v, jerk.fwd_s + s, kin.accel, kin.jerk);
    let jerk_backward =
        scurve::max_reachable_velocity(jerk.bwd_v, jerk.bwd_s + rest, kin.accel, kin.jerk);
    let ceiling = kin
        .flat_ceiling
        .min(limit_speed(kin.kappa_abs(s), kin.accel));
    Some(
        forward
            .min(backward)
            .min(jerk_forward)
            .min(jerk_backward)
            .min(ceiling),
    )
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

pub(super) fn sample_profile(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    jerk: &JerkAnchors,
    tol: f64,
) -> Option<Vec<(f64, f64)>> {
    let mut out = vec![(0.0, entry)];
    refine(
        kin, entry, exit, jerk, tol, 0.0, entry, kin.length, exit, 0, &mut out,
    )?;
    out.push((kin.length, exit));
    Some(out)
}

#[cfg(test)]
mod tests;
