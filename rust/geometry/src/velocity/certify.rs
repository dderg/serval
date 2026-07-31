//! Bernstein nonnegativity certificate for one constant-jerk phase.
//!
//! On local time `tau` in `[0, dt]` the phase state is exact polynomial:
//! `a = a0 + j*tau`, `v = v0 + a0*tau + j*tau^2/2`,
//! `s = s0 + v0*tau + a0*tau^2/2 + j*tau^3/6`, `kappa = kappa0 + sigma*s`.
//! The three feasibility residuals are therefore polynomials too,
//! `R_d = A^2 - a^2 - kappa^2 v^4` (degree 14),
//! `R_b = J^2 - (j - kappa^2 v^3)^2 - (sigma v^3 + 3 kappa v a)^2` (degree 24)
//! and `R_v = v` (degree 2), which forbids motion reversal.
//! A polynomial whose Bernstein coefficients on an interval are all nonnegative
//! is nonnegative on the whole interval — the certificate is that test, tightened
//! by de Casteljau subdivision. It is one-sided by construction: a certified
//! dwell can be shorter than the true feasible span, never longer.

use super::disk::Kinematics;

const MAX_DEGREE: usize = 24;
const COEFFS: usize = MAX_DEGREE + 1;
const SUBDIVISION_DEPTH: u32 = 5;
const DWELL_BISECT_ITERS: u32 = 40;

/// Rounding slack, in ulps of the summed magnitude, charged to each Bernstein
/// coefficient of a degree-`deg` polynomial. Forming one coefficient commits up
/// to `deg` roundings accumulating `span^k`, `deg` more in the weight
/// recurrence, and one each in the product and the sum.
fn conversion_ulps(deg: usize) -> f64 {
    2.0 * (deg + 2) as f64
}

/// Slack, relative to the natural scale of each residual (`A^2` for the disk,
/// `J^2` for the ball, the flat ceiling for the speed), inside which a residual
/// counts as nonnegative. Without it an exactly-on-the-rail phase (`a == A`,
/// `j == J`) would fail on its own rounding. It also absorbs the rounding the
/// residual polynomials pick up while `residuals` composes them.
pub(super) const CERTIFICATE_REL_TOL: f64 = 1e-11;

/// Shortfall of the dwell bisection against `dt` that [`certified_span`]
/// forgives, relative to `dt`: the bisection's own resolution and nothing more.
pub(super) const DWELL_REL_TOL: f64 = 1.0 / ((1u64 << (DWELL_BISECT_ITERS - 2)) as f64);

#[derive(Clone, Copy)]
struct Poly {
    c: [f64; COEFFS],
    deg: usize,
}

impl Poly {
    fn zero(deg: usize) -> Self {
        Poly {
            c: [0.0; COEFFS],
            deg,
        }
    }

    fn from_coeffs(src: &[f64]) -> Self {
        let mut p = Poly::zero(src.len() - 1);
        p.c[..src.len()].copy_from_slice(src);
        p
    }

    fn constant(x: f64) -> Self {
        Poly::from_coeffs(&[x])
    }

    fn mul(&self, other: &Poly) -> Self {
        let deg = self.deg + other.deg;
        assert!(
            deg <= MAX_DEGREE,
            "certify: product degree {deg} exceeds the {MAX_DEGREE} the residuals need"
        );
        let mut out = Poly::zero(deg);
        for i in 0..=self.deg {
            let ci = self.c[i];
            if ci == 0.0 {
                continue;
            }
            for k in 0..=other.deg {
                out.c[i + k] += ci * other.c[k];
            }
        }
        out
    }

    fn scaled(&self, k: f64) -> Self {
        let mut out = *self;
        for i in 0..=self.deg {
            out.c[i] *= k;
        }
        out
    }

    fn add(&self, other: &Poly) -> Self {
        let mut out = Poly::zero(self.deg.max(other.deg));
        for i in 0..=self.deg {
            out.c[i] += self.c[i];
        }
        for i in 0..=other.deg {
            out.c[i] += other.c[i];
        }
        out
    }

    fn sub(&self, other: &Poly) -> Self {
        self.add(&other.scaled(-1.0))
    }

    fn plus_constant(&self, x: f64) -> Self {
        let mut out = *self;
        out.c[0] += x;
        out
    }

    fn eval(&self, x: f64) -> f64 {
        let mut acc = 0.0;
        for i in (0..=self.deg).rev() {
            acc = acc * x + self.c[i];
        }
        acc
    }
}

#[derive(Clone, Copy)]
struct Bernstein {
    b: [f64; COEFFS],
    slack: [f64; COEFFS],
    deg: usize,
}

impl Bernstein {
    fn zero(deg: usize) -> Self {
        Bernstein {
            b: [0.0; COEFFS],
            slack: [0.0; COEFFS],
            deg,
        }
    }

    /// Coefficients of `p` in the Bernstein basis of `[0, span]`, each paired with
    /// a bound on the rounding committed while forming it.
    fn of(p: &Poly, span: f64) -> Self {
        let n = p.deg;
        let mut scaled = [0.0; COEFFS];
        let mut power = 1.0;
        for k in 0..=n {
            scaled[k] = p.c[k] * power;
            power *= span;
        }
        let mut out = Bernstein::zero(n);
        for i in 0..=n {
            let mut weight = 1.0;
            let mut sum = 0.0;
            let mut magnitude = 0.0;
            for k in 0..=i {
                let term = scaled[k] * weight;
                sum += term;
                magnitude += term.abs();
                if k < n {
                    weight *= (i - k) as f64 / (n - k) as f64;
                }
            }
            out.b[i] = sum;
            out.slack[i] = magnitude * conversion_ulps(n) * f64::EPSILON;
        }
        out
    }

    fn hull_is_nonneg(&self, tol: f64) -> bool {
        (0..=self.deg).all(|i| self.b[i] - self.slack[i] >= -tol)
    }

    fn an_endpoint_violates(&self, tol: f64) -> bool {
        self.b[0] + self.slack[0] < -tol || self.b[self.deg] + self.slack[self.deg] < -tol
    }

    fn halves(&self) -> (Bernstein, Bernstein) {
        let n = self.deg;
        let mut work = self.b;
        let mut work_slack = self.slack;
        let mut left = Bernstein::zero(n);
        let mut right = Bernstein::zero(n);
        left.b[0] = work[0];
        left.slack[0] = work_slack[0];
        right.b[n] = work[n];
        right.slack[n] = work_slack[n];
        for level in 1..=n {
            for i in 0..=(n - level) {
                work[i] = 0.5 * (work[i] + work[i + 1]);
                work_slack[i] =
                    0.5 * (work_slack[i] + work_slack[i + 1]) + work[i].abs() * f64::EPSILON;
            }
            left.b[level] = work[0];
            left.slack[level] = work_slack[0];
            right.b[n - level] = work[n - level];
            right.slack[n - level] = work_slack[n - level];
        }
        (left, right)
    }
}

fn hull_proves_nonneg(hull: &Bernstein, tol: f64, depth: u32) -> bool {
    if hull.hull_is_nonneg(tol) {
        return true;
    }
    if depth == 0 || hull.an_endpoint_violates(tol) {
        return false;
    }
    let (left, right) = hull.halves();
    hull_proves_nonneg(&left, tol, depth - 1) && hull_proves_nonneg(&right, tol, depth - 1)
}

fn is_nonneg_on(p: &Poly, span: f64, tol: f64) -> bool {
    if span <= 0.0 {
        return p.c[0] >= -tol;
    }
    hull_proves_nonneg(&Bernstein::of(p, span), tol, SUBDIVISION_DEPTH)
}

struct Residuals {
    disk: Poly,
    ball: Poly,
    speed: Poly,
    disk_tol: f64,
    ball_tol: f64,
    speed_tol: f64,
}

impl Residuals {
    fn certified_on(&self, span: f64) -> bool {
        is_nonneg_on(&self.disk, span, self.disk_tol)
            && is_nonneg_on(&self.ball, span, self.ball_tol)
            && is_nonneg_on(&self.speed, span, self.speed_tol)
    }

    fn feasible_at(&self, tau: f64) -> bool {
        self.disk.eval(tau) >= -self.disk_tol
            && self.ball.eval(tau) >= -self.ball_tol
            && self.speed.eval(tau) >= -self.speed_tol
    }
}

fn residuals(kin: &Kinematics, s0: f64, v0: f64, a0: f64, j: f64) -> Residuals {
    let accel = Poly::from_coeffs(&[a0, j]);
    let speed = Poly::from_coeffs(&[v0, a0, 0.5 * j]);
    let arc = Poly::from_coeffs(&[s0, v0, 0.5 * a0, j / 6.0]);
    let kappa = arc.scaled(kin.sigma).plus_constant(kin.kappa0);

    let kappa2 = kappa.mul(&kappa);
    let v2 = speed.mul(&speed);
    let v3 = v2.mul(&speed);
    let v4 = v2.mul(&v2);

    let disk = Poly::constant(kin.accel * kin.accel)
        .sub(&accel.mul(&accel))
        .sub(&kappa2.mul(&v4));

    let tangential = Poly::constant(j).sub(&kappa2.mul(&v3));
    let normal = v3
        .scaled(kin.sigma)
        .add(&kappa.mul(&speed).mul(&accel).scaled(3.0));
    let ball = Poly::constant(kin.jerk * kin.jerk)
        .sub(&tangential.mul(&tangential))
        .sub(&normal.mul(&normal));

    Residuals {
        disk_tol: CERTIFICATE_REL_TOL * kin.accel * kin.accel,
        ball_tol: CERTIFICATE_REL_TOL * kin.jerk * kin.jerk,
        speed_tol: CERTIFICATE_REL_TOL * kin.flat_ceiling,
        disk,
        ball,
        speed,
    }
}

fn require_finite(name: &str, x: f64) {
    assert!(
        x.is_finite(),
        "certify: {name} must be finite, got {x} — the caller handed the certificate a broken state"
    );
}

fn require_positive(name: &str, x: f64) {
    require_finite(name, x);
    assert!(
        x > 0.0,
        "certify: degenerate kinematics — {name} must be strictly positive, got {x}"
    );
}

fn validate(kin: &Kinematics, s0: f64, v0: f64, a0: f64, j: f64, dt: f64) {
    require_positive("kinematics.accel", kin.accel);
    require_positive("kinematics.jerk", kin.jerk);
    require_positive("kinematics.flat_ceiling", kin.flat_ceiling);
    require_finite("kinematics.kappa0", kin.kappa0);
    require_finite("kinematics.sigma", kin.sigma);
    require_finite("kinematics.length", kin.length);
    assert!(
        kin.length >= 0.0,
        "certify: degenerate kinematics — length must be nonnegative, got {}",
        kin.length
    );
    require_finite("s0", s0);
    require_finite("v0", v0);
    require_finite("a0", a0);
    require_finite("j", j);
    require_finite("dt", dt);
    assert!(dt >= 0.0, "certify: dt must be nonnegative, got {dt}");
}

/// Largest `tau <= dt` for which all three residuals are *proved* nonnegative on
/// the whole of `[0, tau]`. Never exceeds the true first-violation time.
pub(super) fn certified_dwell(kin: &Kinematics, s0: f64, v0: f64, a0: f64, j: f64, dt: f64) -> f64 {
    validate(kin, s0, v0, a0, j, dt);
    if dt == 0.0 {
        return 0.0;
    }
    let residuals = residuals(kin, s0, v0, a0, j);
    if residuals.certified_on(dt) {
        return dt;
    }
    if !residuals.certified_on(0.0) {
        return 0.0;
    }
    let mut lo = 0.0;
    let mut hi = dt;
    for _ in 0..DWELL_BISECT_ITERS {
        let mid = 0.5 * (lo + hi);
        if residuals.certified_on(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// Span a caller may emit as one phase: `dt` when the whole of it is proved, and
/// otherwise the certified dwell. A shortfall inside the bisection's own
/// resolution is rounded up to `dt` only when the state at `dt` is itself
/// feasible, so a violation living in that terminal sliver is never absorbed.
pub(super) fn certified_span(kin: &Kinematics, s0: f64, v0: f64, a0: f64, j: f64, dt: f64) -> f64 {
    let dwell = certified_dwell(kin, s0, v0, a0, j, dt);
    if dwell >= dt {
        return dwell;
    }
    if dwell >= dt * (1.0 - DWELL_REL_TOL) && residuals(kin, s0, v0, a0, j).feasible_at(dt) {
        dt
    } else {
        dwell
    }
}

pub(super) fn is_certified(kin: &Kinematics, s0: f64, v0: f64, a0: f64, j: f64, dt: f64) -> bool {
    certified_span(kin, s0, v0, a0, j, dt) >= dt
}
