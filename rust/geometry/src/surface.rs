//! Bed surface transform: a bicubic cardinal-spline mesh over probed points,
//! combined with a Z-height fade. Evaluated directly at runtime (no dense
//! pre-interpolation): the spline is C¹ with analytic gradients and second
//! derivatives, which the lowerer needs for chain-rule Z velocity/accel
//! feedforward. See docs/rewrite/toolpath-surface-transforms.md.

/// Sampled cells are probed on an interior grid this fine; dense enough that
/// inter-sample variation of a cubic patch is negligible. Gross-error-gate
/// accuracy, not interval arithmetic — no padding is applied, so the bounds
/// can undershoot the true extrema by a sliver.
const BOUND_SAMPLES_PER_CELL: usize = 16;

use crate::path::Segment;
use crate::path::lowering::PositionProfile;
use crate::path::profile::CurvatureProfile;

const TRANSITION_SOLVE_ITERATIONS: usize = 80;
const TRANSITION_S_TOLERANCE: f64 = 1e-11;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceContinuity {
    C1,
    C0,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceTransition {
    pub s: f64,
    pub continuity: SurfaceContinuity,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceTransitionError {
    NonFiniteSegment,
    UnresolvedCrossing { axis: usize, boundary: f64 },
}

impl std::fmt::Display for SurfaceTransitionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFiniteSegment => write!(f, "surface transition path is not finite"),
            Self::UnresolvedCrossing { axis, boundary } => {
                write!(
                    f,
                    "surface transition crossing unresolved on axis {axis} at {boundary}"
                )
            }
        }
    }
}

impl std::error::Error for SurfaceTransitionError {}

#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceError {
    GridTooSmall { nx: usize, ny: usize },
    PointCountMismatch { expected: usize, got: usize },
    NonPositiveSpacing { dx: f64, dy: f64 },
    NonFinitePoint { index: usize },
    FadeBandInverted { start: f64, end: f64 },
}

impl std::fmt::Display for SurfaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::GridTooSmall { nx, ny } => {
                write!(f, "mesh grid must be at least 2x2, got {nx}x{ny}")
            }
            Self::PointCountMismatch { expected, got } => {
                write!(f, "mesh grid expects {expected} points, got {got}")
            }
            Self::NonPositiveSpacing { dx, dy } => {
                write!(f, "mesh grid spacing must be positive, got dx={dx} dy={dy}")
            }
            Self::NonFinitePoint { index } => write!(f, "mesh point {index} is not finite"),
            Self::FadeBandInverted { start, end } => {
                write!(f, "fade band must have end > start, got [{start}, {end}]")
            }
        }
    }
}

impl std::error::Error for SurfaceError {}

/// Surface height and its derivatives at one (x, y).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceSample {
    pub z: f64,
    pub zx: f64,
    pub zy: f64,
    pub zxx: f64,
    pub zxy: f64,
    pub zyy: f64,
}

/// Worst-case magnitudes over the whole mesh (densely sampled, unpadded),
/// for the activation-time gross-error gate and sampled range estimates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceBounds {
    /// max ‖∇z‖ (dimensionless slope).
    pub max_gradient: f64,
    /// max Gershgorin bound on the Hessian's spectral radius (1/mm).
    pub max_curvature: f64,
    pub z_min: f64,
    pub z_max: f64,
}

/// Uniform grid of probed heights, interpolated by a tensor-product cardinal
/// spline (the same interpolant mainline bed_mesh uses to build its dense
/// grid, evaluated directly). Points are row-major: `z[j * nx + i]` is the
/// height at `(x_min + i·dx, y_min + j·dy)`. Queries outside the grid clamp
/// to the boundary value with zero gradient, matching mainline's clamped
/// index lookup.
#[derive(Debug, Clone, PartialEq)]
pub struct MeshGrid {
    x_min: f64,
    y_min: f64,
    dx: f64,
    dy: f64,
    nx: usize,
    ny: usize,
    z: Vec<f64>,
    tension: f64,
}

fn hermite(p: [f64; 4], t: f64, tension: f64) -> (f64, f64, f64) {
    let m1 = tension * (p[2] - p[0]);
    let m2 = tension * (p[3] - p[1]);
    let t2 = t * t;
    let t3 = t2 * t;
    let v = p[1] * (2.0 * t3 - 3.0 * t2 + 1.0)
        + m1 * (t3 - 2.0 * t2 + t)
        + p[2] * (-2.0 * t3 + 3.0 * t2)
        + m2 * (t3 - t2);
    let d = p[1] * (6.0 * t2 - 6.0 * t)
        + m1 * (3.0 * t2 - 4.0 * t + 1.0)
        + p[2] * (6.0 * t - 6.0 * t2)
        + m2 * (3.0 * t2 - 2.0 * t);
    let dd = p[1] * (12.0 * t - 6.0)
        + m1 * (6.0 * t - 4.0)
        + p[2] * (6.0 - 12.0 * t)
        + m2 * (6.0 * t - 2.0);
    (v, d, dd)
}

/// Cell index and local parameter for a clamped query along one axis.
/// `inside` is false when the raw coordinate fell outside the grid.
fn locate(coord: f64, min: f64, spacing: f64, count: usize) -> (usize, f64, bool) {
    let raw = (coord - min) / spacing;
    let max_t = (count - 1) as f64;
    let inside = (0.0..=max_t).contains(&raw);
    let clamped = raw.clamp(0.0, max_t);
    let cell = (clamped.floor() as usize).min(count - 2);
    (cell, clamped - cell as f64, inside)
}

impl MeshGrid {
    pub fn new(
        x_min: f64,
        y_min: f64,
        dx: f64,
        dy: f64,
        nx: usize,
        ny: usize,
        z: Vec<f64>,
        tension: f64,
    ) -> Result<Self, SurfaceError> {
        if nx < 2 || ny < 2 {
            return Err(SurfaceError::GridTooSmall { nx, ny });
        }
        if z.len() != nx * ny {
            return Err(SurfaceError::PointCountMismatch {
                expected: nx * ny,
                got: z.len(),
            });
        }
        if !(dx > 0.0 && dy > 0.0) {
            return Err(SurfaceError::NonPositiveSpacing { dx, dy });
        }
        if let Some(index) = z.iter().position(|v| !v.is_finite()) {
            return Err(SurfaceError::NonFinitePoint { index });
        }
        Ok(Self {
            x_min,
            y_min,
            dx,
            dy,
            nx,
            ny,
            z,
            tension,
        })
    }

    pub fn x_range(&self) -> (f64, f64) {
        (self.x_min, (self.nx - 1) as f64 * self.dx + self.x_min)
    }

    pub fn y_range(&self) -> (f64, f64) {
        (self.y_min, (self.ny - 1) as f64 * self.dy + self.y_min)
    }

    pub fn contains(&self, x: f64, y: f64) -> bool {
        let (x0, x1) = self.x_range();
        let (y0, y1) = self.y_range();
        (x0..=x1).contains(&x) && (y0..=y1).contains(&y)
    }

    fn point(&self, i: isize, j: isize) -> f64 {
        let i = i.clamp(0, self.nx as isize - 1) as usize;
        let j = j.clamp(0, self.ny as isize - 1) as usize;
        self.z[j * self.nx + i]
    }

    pub fn sample(&self, x: f64, y: f64) -> SurfaceSample {
        let (ci, u, x_inside) = locate(x, self.x_min, self.dx, self.nx);
        let (cj, v, y_inside) = locate(y, self.y_min, self.dy, self.ny);

        let mut rows = [(0.0, 0.0, 0.0); 4];
        for (k, row) in rows.iter_mut().enumerate() {
            let j = cj as isize + k as isize - 1;
            let p = [
                self.point(ci as isize - 1, j),
                self.point(ci as isize, j),
                self.point(ci as isize + 1, j),
                self.point(ci as isize + 2, j),
            ];
            *row = hermite(p, u, self.tension);
        }
        let col = |pick: fn(&(f64, f64, f64)) -> f64| {
            [
                pick(&rows[0]),
                pick(&rows[1]),
                pick(&rows[2]),
                pick(&rows[3]),
            ]
        };
        let (z, zy_v, zyy_v) = hermite(col(|r| r.0), v, self.tension);
        let (zx_u, zxy_uv, _) = hermite(col(|r| r.1), v, self.tension);
        let (zxx_u, _, _) = hermite(col(|r| r.2), v, self.tension);

        let sx = if x_inside { 1.0 / self.dx } else { 0.0 };
        let sy = if y_inside { 1.0 / self.dy } else { 0.0 };
        SurfaceSample {
            z,
            zx: zx_u * sx,
            zy: zy_v * sy,
            zxx: zxx_u * sx * sx,
            zxy: zxy_uv * sx * sy,
            zyy: zyy_v * sy * sy,
        }
    }

    /// Shift the whole grid so the surface evaluates to exactly 0 at the
    /// reference point. The mesh then expresses only relative bed deviation
    /// and cannot move the global Z datum.
    pub fn zero_at(&mut self, x: f64, y: f64) {
        let offset = self.sample(x, y).z;
        for p in &mut self.z {
            *p -= offset;
        }
    }

    /// Sampled worst-case magnitudes at [`BOUND_SAMPLES_PER_CELL`] density.
    pub fn bounds(&self) -> SurfaceBounds {
        let mut max_gradient: f64 = 0.0;
        let mut max_curvature: f64 = 0.0;
        let mut z_min = f64::INFINITY;
        let mut z_max = f64::NEG_INFINITY;
        let steps = BOUND_SAMPLES_PER_CELL;
        for j in 0..(self.ny - 1) * steps + 1 {
            for i in 0..(self.nx - 1) * steps + 1 {
                let x = self.x_min + self.dx * i as f64 / steps as f64;
                let y = self.y_min + self.dy * j as f64 / steps as f64;
                let s = self.sample(x, y);
                max_gradient = max_gradient.max(libm::hypot(s.zx, s.zy));
                max_curvature = max_curvature
                    .max(s.zxx.abs() + s.zxy.abs())
                    .max(s.zyy.abs() + s.zxy.abs());
                z_min = z_min.min(s.z);
                z_max = z_max.max(s.z);
            }
        }
        SurfaceBounds {
            max_gradient,
            max_curvature,
            z_min,
            z_max,
        }
    }

    /// Sound bound on the surface's z spread over an XY box: sampled range
    /// padded by the gradient bound times the sample spacing. Used to decide
    /// whether a move can treat its correction as constant.
    pub fn z_spread_over(&self, x0: f64, x1: f64, y0: f64, y1: f64, max_gradient: f64) -> f64 {
        let (xa, xb) = (x0.min(x1), x0.max(x1));
        let (ya, yb) = (y0.min(y1), y0.max(y1));
        let steps_x = ((xb - xa) / self.dx).ceil() as usize * 2 + 1;
        let steps_y = ((yb - ya) / self.dy).ceil() as usize * 2 + 1;
        let mut z_min = f64::INFINITY;
        let mut z_max = f64::NEG_INFINITY;
        for j in 0..=steps_y {
            for i in 0..=steps_x {
                let x = xa + (xb - xa) * i as f64 / steps_x as f64;
                let y = ya + (yb - ya) * j as f64 / steps_y as f64;
                let z = self.sample(x, y).z;
                z_min = z_min.min(z);
                z_max = z_max.max(z);
            }
        }
        let spacing = libm::hypot((xb - xa) / steps_x as f64, (yb - ya) / steps_y as f64);
        (z_max - z_min) + max_gradient * spacing
    }
}

/// Mainline fade semantics: full correction below `start`, linear ramp to
/// zero at `end`, correction fading toward `target` rather than toward 0,
/// constant `target` above the band. A disabled fade keeps factor 1 at every
/// height (`start = end = +∞`), which also makes `target` irrelevant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fade {
    start: f64,
    end: f64,
    pub target: f64,
}

impl Fade {
    pub fn new(start: f64, end: f64, target: f64) -> Result<Self, SurfaceError> {
        if !(end > start) {
            return Err(SurfaceError::FadeBandInverted { start, end });
        }
        Ok(Self { start, end, target })
    }

    pub fn disabled() -> Self {
        Self {
            start: f64::INFINITY,
            end: f64::INFINITY,
            target: 0.0,
        }
    }

    pub fn is_disabled(&self) -> bool {
        self.start.is_infinite()
    }

    pub fn band(&self) -> (f64, f64) {
        (self.start, self.end)
    }

    pub fn factor(&self, z: f64) -> f64 {
        if z >= self.end {
            0.0
        } else if z >= self.start {
            (self.end - z) / (self.end - self.start)
        } else {
            1.0
        }
    }

    pub fn dfactor(&self, z: f64) -> f64 {
        if (self.start..self.end).contains(&z) {
            -1.0 / (self.end - self.start)
        } else {
            0.0
        }
    }
}

/// The warp value `w = fade(z)·(mesh(x,y) − target) + target` and its partial
/// derivatives; `z_machine = z_gcode + w`. `wzz` is identically zero (the
/// fade is piecewise linear in z), so it is not carried.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WarpSample {
    pub w: f64,
    pub wx: f64,
    pub wy: f64,
    pub wz: f64,
    pub wxx: f64,
    pub wxy: f64,
    pub wyy: f64,
    pub wxz: f64,
    pub wyz: f64,
}

/// The complete surface-following transform the lowerer applies: mesh × fade,
/// with the mesh's worst-case bounds precomputed once at construction.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceTransform {
    mesh: MeshGrid,
    fade: Fade,
    bounds: SurfaceBounds,
}

fn push_partition(partitions: &mut Vec<f64>, s: f64, length: f64) {
    if s.is_finite() && s > 0.0 && s < length {
        partitions.push(s);
    }
}

fn periodic_values_between(alpha: f64, low: f64, high: f64) -> Vec<f64> {
    let pi = std::f64::consts::PI;
    let first = libm::ceil((low - alpha) / pi);
    let last = libm::floor((high - alpha) / pi);
    let mut values = Vec::new();
    let mut k = first;
    while k <= last {
        values.push(alpha + k * pi);
        k += 1.0;
    }
    values
}

fn heading_partitions(segment: &Segment, axis: usize, length: f64) -> Vec<f64> {
    let mut partitions = vec![0.0, length];
    match segment {
        Segment::Line(_) => {}
        Segment::Arc(arc) => {
            let u = arc.u[axis];
            let v = arc.v[axis];
            if u != 0.0 || v != 0.0 {
                let theta_end = arc.start_angle + arc.sweep;
                let alpha = libm::atan2(v, u);
                for theta in periodic_values_between(
                    alpha,
                    arc.start_angle.min(theta_end),
                    arc.start_angle.max(theta_end),
                ) {
                    push_partition(
                        &mut partitions,
                        (theta - arc.start_angle) * arc.radius / arc.sweep.signum(),
                        length,
                    );
                }
            }
        }
        Segment::Clothoid(clothoid) => {
            let u = clothoid.u[axis];
            let v = clothoid.v[axis];
            if u != 0.0 || v != 0.0 {
                let phi = |s: f64| clothoid.kappa_0 * s + 0.5 * clothoid.sigma * s * s;
                let mut low = phi(0.0).min(phi(length));
                let mut high = phi(0.0).max(phi(length));
                if clothoid.sigma != 0.0 {
                    let vertex = -clothoid.kappa_0 / clothoid.sigma;
                    if vertex > 0.0 && vertex < length {
                        low = low.min(phi(vertex));
                        high = high.max(phi(vertex));
                    }
                }
                let alpha = libm::atan2(-u, v);
                for target in periodic_values_between(alpha, low, high) {
                    if clothoid.sigma == 0.0 {
                        if clothoid.kappa_0 != 0.0 {
                            push_partition(&mut partitions, target / clothoid.kappa_0, length);
                        }
                    } else {
                        let discriminant =
                            clothoid.kappa_0 * clothoid.kappa_0 + 2.0 * clothoid.sigma * target;
                        if discriminant >= 0.0 {
                            let root = libm::sqrt(discriminant);
                            push_partition(
                                &mut partitions,
                                (-clothoid.kappa_0 - root) / clothoid.sigma,
                                length,
                            );
                            push_partition(
                                &mut partitions,
                                (-clothoid.kappa_0 + root) / clothoid.sigma,
                                length,
                            );
                        }
                    }
                }
            }
        }
    }
    partitions.sort_by(f64::total_cmp);
    partitions.dedup_by(|a, b| (*a - *b).abs() <= TRANSITION_S_TOLERANCE * (1.0 + length));
    partitions
}

fn solve_crossing(
    segment: &Segment,
    axis: usize,
    boundary: f64,
    mut lo: f64,
    mut hi: f64,
    length: f64,
) -> Result<f64, SurfaceTransitionError> {
    let value = |s: f64| segment.point_at(s)[axis] - boundary;
    let mut flo = value(lo);
    let fhi = value(hi);
    if !(flo.is_finite() && fhi.is_finite() && flo * fhi < 0.0) {
        return Err(SurfaceTransitionError::UnresolvedCrossing { axis, boundary });
    }
    let tolerance = TRANSITION_S_TOLERANCE * (1.0 + length);
    let mut s = 0.5 * (lo + hi);
    for _ in 0..TRANSITION_SOLVE_ITERATIONS {
        let f = value(s);
        let derivative = segment.heading_at(s)[axis];
        if !(f.is_finite() && derivative.is_finite()) {
            return Err(SurfaceTransitionError::NonFiniteSegment);
        }
        if f == 0.0 || hi - lo <= tolerance {
            return Ok(s);
        }
        if flo.signum() == f.signum() {
            lo = s;
            flo = f;
        } else {
            hi = s;
        }
        let newton = s - f / derivative;
        s = if derivative != 0.0 && newton.is_finite() && newton > lo && newton < hi {
            newton
        } else {
            0.5 * (lo + hi)
        };
    }
    Err(SurfaceTransitionError::UnresolvedCrossing { axis, boundary })
}

fn transition_boundaries(
    min: f64,
    spacing: f64,
    count: usize,
) -> impl Iterator<Item = (f64, SurfaceContinuity)> {
    (0..count).map(move |index| {
        let continuity = if index == 0 || index + 1 == count {
            SurfaceContinuity::C0
        } else {
            SurfaceContinuity::C1
        };
        (min + spacing * index as f64, continuity)
    })
}

impl SurfaceTransform {
    pub fn new(mesh: MeshGrid, fade: Fade) -> Self {
        let bounds = mesh.bounds();
        Self { mesh, fade, bounds }
    }

    pub fn mesh(&self) -> &MeshGrid {
        &self.mesh
    }

    pub fn fade(&self) -> &Fade {
        &self.fade
    }

    pub fn bounds(&self) -> SurfaceBounds {
        self.bounds
    }

    pub fn path_transition_distances(
        &self,
        segment: &Segment,
    ) -> Result<Vec<SurfaceTransition>, SurfaceTransitionError> {
        let length = segment.s_len();
        if !(length.is_finite() && length > 0.0) {
            return Err(SurfaceTransitionError::NonFiniteSegment);
        }
        for s in [0.0, length] {
            if segment
                .point_at(s)
                .into_iter()
                .chain(segment.heading_at(s))
                .any(|value| !value.is_finite())
            {
                return Err(SurfaceTransitionError::NonFiniteSegment);
            }
        }

        let mut transitions = Vec::new();
        for (axis, boundaries) in [
            transition_boundaries(self.mesh.x_min, self.mesh.dx, self.mesh.nx).collect::<Vec<_>>(),
            transition_boundaries(self.mesh.y_min, self.mesh.dy, self.mesh.ny).collect::<Vec<_>>(),
            if self.fade.is_disabled() {
                Vec::new()
            } else {
                vec![
                    (self.fade.start, SurfaceContinuity::C0),
                    (self.fade.end, SurfaceContinuity::C0),
                ]
            },
        ]
        .into_iter()
        .enumerate()
        {
            let partitions = heading_partitions(segment, axis, length);
            if partitions.iter().any(|s| {
                !s.is_finite()
                    || !segment.point_at(*s)[axis].is_finite()
                    || !segment.heading_at(*s)[axis].is_finite()
            }) {
                return Err(SurfaceTransitionError::NonFiniteSegment);
            }
            for (boundary, continuity) in boundaries {
                for interval in partitions.windows(2) {
                    let lo = interval[0];
                    let hi = interval[1];
                    let flo = segment.point_at(lo)[axis] - boundary;
                    let fhi = segment.point_at(hi)[axis] - boundary;
                    if !(flo.is_finite() && fhi.is_finite()) {
                        return Err(SurfaceTransitionError::NonFiniteSegment);
                    }
                    if flo * fhi < 0.0 {
                        transitions.push(SurfaceTransition {
                            s: solve_crossing(segment, axis, boundary, lo, hi, length)?,
                            continuity,
                        });
                    }
                }
                for index in 1..partitions.len() - 1 {
                    let s = partitions[index];
                    let at = segment.point_at(s)[axis] - boundary;
                    let before =
                        segment.point_at(0.5 * (partitions[index - 1] + s))[axis] - boundary;
                    let after =
                        segment.point_at(0.5 * (s + partitions[index + 1]))[axis] - boundary;
                    if !(at.is_finite() && before.is_finite() && after.is_finite()) {
                        return Err(SurfaceTransitionError::NonFiniteSegment);
                    }
                    let coordinate_tolerance = TRANSITION_S_TOLERANCE * (1.0 + boundary.abs());
                    if at.abs() <= coordinate_tolerance && before * after < 0.0 {
                        transitions.push(SurfaceTransition { s, continuity });
                    }
                }
            }
        }

        transitions.sort_by(|a, b| a.s.total_cmp(&b.s));
        let tolerance = TRANSITION_S_TOLERANCE * (1.0 + length);
        let mut deduplicated: Vec<SurfaceTransition> = Vec::with_capacity(transitions.len());
        for transition in transitions {
            match deduplicated.last_mut() {
                Some(previous) if (previous.s - transition.s).abs() <= tolerance => {
                    if transition.continuity == SurfaceContinuity::C0 {
                        previous.continuity = SurfaceContinuity::C0;
                    }
                }
                _ => deduplicated.push(transition),
            }
        }
        Ok(deduplicated)
    }

    /// Sound bound on how much the correction can vary over a gcode-space
    /// box: the mesh spread scaled by the largest fade factor in the z range,
    /// plus the fade factor's own variation times the largest deviation from
    /// the fade target anywhere on the mesh.
    pub fn correction_spread_over(
        &self,
        x0: f64,
        x1: f64,
        y0: f64,
        y1: f64,
        z0: f64,
        z1: f64,
    ) -> f64 {
        let f_hi = self.fade.factor(z0.min(z1));
        if f_hi == 0.0 {
            return 0.0;
        }
        let f_lo = self.fade.factor(z0.max(z1));
        let spread = self
            .mesh
            .z_spread_over(x0, x1, y0, y1, self.bounds.max_gradient);
        let dev_max = (self.bounds.z_min - self.fade.target)
            .abs()
            .max((self.bounds.z_max - self.fade.target).abs());
        f_hi * spread + (f_hi - f_lo) * dev_max
    }

    /// Warp and partials at one gcode-space point.
    pub fn warp(&self, x: f64, y: f64, z: f64) -> WarpSample {
        let s = self.mesh.sample(x, y);
        let f = self.fade.factor(z);
        let df = self.fade.dfactor(z);
        let dev = s.z - self.fade.target;
        WarpSample {
            w: f * dev + self.fade.target,
            wx: f * s.zx,
            wy: f * s.zy,
            wz: df * dev,
            wxx: f * s.zxx,
            wxy: f * s.zxy,
            wyy: f * s.zyy,
            wxz: df * s.zx,
            wyz: df * s.zy,
        }
    }

    pub fn correction_at(&self, x: f64, y: f64, z: f64) -> f64 {
        self.warp(x, y, z).w
    }

    /// Recover the gcode Z from a measured machine Z at (x, y): the inverse
    /// of `z_machine = z_g + fade(z_g)·(mesh − target) + target`. The forward
    /// map is strictly increasing in z_g whenever the fade band is wider than
    /// the mesh deviation (validated at activation), so exactly one branch —
    /// below, inside, or above the band — is consistent.
    pub fn gcode_z(&self, x: f64, y: f64, z_machine: f64) -> f64 {
        let mesh_z = self.mesh.sample(x, y).z;
        let target = self.fade.target;
        let (start, end) = self.fade.band();

        let full = z_machine - mesh_z;
        if full <= start {
            return full;
        }
        let faded = z_machine - target;
        if faded >= end {
            return faded;
        }
        let dist = end - start;
        let dev = mesh_z - target;
        let denom = 1.0 - dev / dist;
        assert!(
            denom > 0.0,
            "surface inverse is not monotonic: fade band {dist}mm narrower than mesh \
             deviation {dev}mm — activation validation must reject this"
        );
        let mid = (z_machine - target - end * dev / dist) / denom;
        assert!(
            (start - 1e-9..=end + 1e-9).contains(&mid),
            "no fade branch consistent inverting machine z {z_machine} at ({x}, {y})"
        );
        mid.clamp(start, end)
    }

    /// Exact inverse of the lowerer's `warped_z` chain rule for one instant:
    /// recover the gcode-space `(z, ż, z̈)` from the machine-space Z state
    /// recorded in motion history, given the (warp-invariant) XY state at the
    /// same instant. Machine forward is
    /// `z_m = z_g + w(x, y, z_g)`,
    /// `ż_m = ż_g·(1 + w_z) + w_x·ẋ + w_y·ẏ`,
    /// `z̈_m = z̈_g·(1 + w_z) + 2·(w_xz·ẋ + w_yz·ẏ)·ż_g
    ///        + w_xx·ẋ² + 2·w_xy·ẋẏ + w_yy·ẏ² + w_x·ẍ + w_y·ÿ`
    /// (`w_zz ≡ 0`: the fade is piecewise linear). `1 + w_z > 0` is the
    /// monotonicity the activation gate validates, so the division is safe;
    /// assert it anyway rather than return a wrong state.
    pub fn unwarp_z_state(
        &self,
        xy_pos: [f64; 2],
        xy_vel: [f64; 2],
        xy_accel: [f64; 2],
        z_machine: (f64, f64, f64),
    ) -> (f64, f64, f64) {
        let (x, y) = (xy_pos[0], xy_pos[1]);
        let z_g = self.gcode_z(x, y, z_machine.0);
        let w = self.warp(x, y, z_g);
        let denom = 1.0 + w.wz;
        assert!(
            denom > 0.0,
            "surface unwarp is not monotonic at ({x}, {y}, {z_g}): 1 + wz = {denom} — \
             activation validation must reject this"
        );
        let (vx, vy) = (xy_vel[0], xy_vel[1]);
        let zg_vel = (z_machine.1 - w.wx * vx - w.wy * vy) / denom;
        let zg_accel = (z_machine.2
            - 2.0 * (w.wxz * vx + w.wyz * vy) * zg_vel
            - (w.wxx * vx * vx + 2.0 * w.wxy * vx * vy + w.wyy * vy * vy)
            - w.wx * xy_accel[0]
            - w.wy * xy_accel[1])
            / denom;
        (z_g, zg_vel, zg_accel)
    }
}

#[cfg(test)]
mod tests;
