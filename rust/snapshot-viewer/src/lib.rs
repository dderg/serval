use js_sys::Float64Array;
use pipeline_snapshot::{ExactTrajectory, Pvaj, SampleSide, Snapshot};
use wasm_bindgen::prelude::*;

#[cfg(test)]
mod tests;

const AXIS_X: usize = 0;
const AXIS_Y: usize = 1;
const AXIS_Z: usize = 2;
const AXIS_E: usize = 3;

fn scalar_derivative(comp_x: &[f64], comp_y: &[f64]) -> Vec<f64> {
    comp_x
        .iter()
        .zip(comp_y)
        .map(|(ax, ay)| libm::hypot(*ax, *ay))
        .collect()
}

// Project a per-sample XY vector series onto the velocity's tangent/normal
// frame: tangential = (v·f)/|v| (signed — negative while braking), normal =
// |v×f|/|v|. For acceleration this splits speed change from direction change
// (centripetal); the same projection of jerk shows which of the two is
// changing. Where the toolhead is stopped the frame is undefined, so both
// components read zero.
const FRENET_SPEED_FLOOR: f64 = 1e-9;

// FRENET_SPEED_FLOOR (above) is tuned for Frenet-projection's 1/speed
// sensitivity; kappa's 1/speed^3 sensitivity blows up far sooner as speed
// shrinks -- a speed that's a perfectly fine floor for tangential/normal
// projection still produces an astronomically large, physically
// meaningless kappa near a genuine full stop.
const CURVATURE_CUSP_SPEED_FLOOR: f64 = 1e-3;

fn frenet_components(vx: &[f64], vy: &[f64], fx: &[f64], fy: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = vx.len();
    let mut tang = Vec::with_capacity(n);
    let mut norm = Vec::with_capacity(n);
    for i in 0..n {
        let speed = libm::hypot(vx[i], vy[i]);
        if speed <= FRENET_SPEED_FLOOR {
            tang.push(0.0);
            norm.push(0.0);
        } else {
            tang.push((vx[i] * fx[i] + vy[i] * fy[i]) / speed);
            norm.push((vx[i] * fy[i] - vy[i] * fx[i]).abs() / speed);
        }
    }
    (tang, norm)
}

// -- Curvature -----------------------------------------------------------

// Signed planar curvature kappa = (vx*ay - vy*ax) / speed^3 and its time
// derivative, from one instant's velocity/accel/jerk. kappa is
// parameterization-invariant -- its *value* at a point along the path
// doesn't depend on how fast that point was reached -- so this is exactly
// as valid whether vx/vy/ax/ay/jx/jy came from a fast or slow traversal of
// the same geometric path. Precondition: speed > 0 (a zero-speed sample is
// a cusp, handled by the caller before this is ever invoked).
fn kappa_and_dkappa_dt(vx: f64, vy: f64, ax: f64, ay: f64, jx: f64, jy: f64) -> (f64, f64) {
    let speed2 = vx * vx + vy * vy;
    let speed = speed2.sqrt();
    let n = vx * ay - vy * ax;
    let kappa = n / (speed2 * speed);
    let n_dot = vx * jy - vy * jx;
    let dkappa_dt =
        n_dot / (speed2 * speed) - 3.0 * n * (vx * ax + vy * ay) / (speed2 * speed2 * speed);
    (kappa, dkappa_dt)
}

// -- Curvature classification -----------------------------------------------

const KAPPA_ZERO_EPS: f64 = 1e-4; // 1/mm -- radius > 10 m reads as straight
const DKAPPA_DS_ZERO_EPS: f64 = 1e-3; // 1/mm^2 -- first-pass, tune against real cases
const DKAPPA_DS_SPREAD_EPS: f64 = 1e-3; // 1/mm^2 -- first-pass, tune against real cases

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CurvatureClass {
    Zero,
    Constant,
    Linear,
    Other,
    Cusp,
    Gap,
}

impl CurvatureClass {
    fn code(self) -> f64 {
        match self {
            CurvatureClass::Zero => 0.0,
            CurvatureClass::Constant => 1.0,
            CurvatureClass::Linear => 2.0,
            CurvatureClass::Other => 3.0,
            CurvatureClass::Cusp => 4.0,
            CurvatureClass::Gap => 5.0,
        }
    }
}

// ~10th-to-90th-percentile spread of a sorted slice: robust to a handful of
// outliers (e.g. the one or two samples nearest a carrier seam, where
// dkappa/ds can legitimately jump even in a perfectly healthy trajectory) in a
// way a raw max-min is not. Trims at least 1 sample off each end whenever there
// are at least 3 -- plain `n / 10` truncates to 0 (i.e. no trim at all,
// degrading to raw min-max) for any n under 10, which is exactly the
// small-window case (a trailing partial window, or one shrunk by excluding
// Cusp/Gap samples) this robustness exists to cover. At n=3 this trims to a
// single middle element, so spread always reads as 0 regardless of the two
// outer samples — an accepted tradeoff: a 3-sample window erring toward "not
// enough data to call it anomalous" is safer than the alternative of no outlier
// protection at all.
fn percentile_spread(sorted: &[f64]) -> f64 {
    let n = sorted.len();
    if n < 3 {
        return sorted.last().copied().unwrap_or(0.0) - sorted.first().copied().unwrap_or(0.0);
    }
    let trim = (n / 10).max(1).min((n - 1) / 2);
    let lo = sorted[trim];
    let hi = sorted[n - 1 - trim];
    hi - lo
}

// Classifies one window's worth of (kappa, dkappa/ds) samples -- both
// already restricted by the caller to non-Cusp, non-Gap samples. Zero is
// checked against kappa itself (not its rate) so a dead-straight stretch
// reads as Zero rather than Constant; Constant vs. Linear both hinge on
// whether dkappa/ds is steady across the window (a percentile spread, not
// raw min-max), splitting on whether that steady rate is itself ~zero.
fn classify_window(kappa: &[f64], dkappa_ds: &[f64]) -> CurvatureClass {
    let max_abs_kappa = kappa.iter().fold(0.0_f64, |m, k| m.max(k.abs()));
    if max_abs_kappa < KAPPA_ZERO_EPS {
        return CurvatureClass::Zero;
    }
    let mut sorted: Vec<f64> = dkappa_ds.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let spread = percentile_spread(&sorted);
    let median = sorted[sorted.len() / 2];
    if spread >= DKAPPA_DS_SPREAD_EPS {
        return CurvatureClass::Other;
    }
    if median.abs() < DKAPPA_DS_ZERO_EPS {
        CurvatureClass::Constant
    } else {
        CurvatureClass::Linear
    }
}

// Despikes a per-window class sequence: a window whose class differs from
// BOTH neighbors, when those neighbors agree with each other, is overwritten
// to match them. This is what turns "one seam artifact flips one window"
// into a stable, contiguous stretch, while a class change that actually
// persists across several windows -- a real pipeline anomaly -- survives
// untouched.
fn smooth_classes(raw: &[CurvatureClass]) -> Vec<CurvatureClass> {
    if raw.len() < 3 {
        return raw.to_vec();
    }
    let mut out = raw.to_vec();
    for i in 1..raw.len() - 1 {
        if raw[i] != raw[i - 1] && raw[i] != raw[i + 1] && raw[i - 1] == raw[i + 1] {
            out[i] = raw[i - 1];
        }
    }
    out
}

const CLASSIFY_WINDOW_SAMPLES: usize = 24; // first-pass; tune against real cases

// One kappa value and one CurvatureClass per sample of an already-evaluated
// XY velocity/acceleration/jerk series -- the exact derivatives read straight
// off the trajectory's carriers, so kappa and dkappa/ds are exact too. A
// near-zero-speed sample is flagged Cusp; everything else feeds a fixed-size,
// carrier-agnostic sliding window that gets classified as a unit and then
// despiked against its neighbors.
fn curvature_series(
    vx: &[f64],
    vy: &[f64],
    ax: &[f64],
    ay: &[f64],
    jx: &[f64],
    jy: &[f64],
) -> (Vec<f64>, Vec<CurvatureClass>) {
    let n = vx.len();
    let mut kappa = vec![0.0; n];
    let mut dkappa_ds = vec![0.0; n];
    let mut flag: Vec<Option<CurvatureClass>> = vec![None; n];

    for i in 0..n {
        let speed = libm::hypot(vx[i], vy[i]);
        if speed < CURVATURE_CUSP_SPEED_FLOOR {
            flag[i] = Some(CurvatureClass::Cusp);
            continue;
        }
        let (k, dk_dt) = kappa_and_dkappa_dt(vx[i], vy[i], ax[i], ay[i], jx[i], jy[i]);
        kappa[i] = k;
        dkappa_ds[i] = dk_dt / speed;
    }

    // Classify per window first (one entry per window, NOT expanded to
    // per-sample) so smooth_classes's neighbor comparison actually compares
    // adjacent WINDOWS. Expanding to per-sample before smoothing would make
    // every non-boundary sample its own "neighbor" by construction (they're
    // all the same repeated value within a window), so the despike pass
    // would never find anything to despike.
    let mut window_starts: Vec<usize> = Vec::new();
    let mut window_classes: Vec<CurvatureClass> = Vec::new();
    let mut i = 0;
    while i < n {
        let end = (i + CLASSIFY_WINDOW_SAMPLES).min(n);
        let idxs: Vec<usize> = (i..end).filter(|&j| flag[j].is_none()).collect();
        let cls = if idxs.is_empty() {
            CurvatureClass::Other
        } else {
            let k: Vec<f64> = idxs.iter().map(|&j| kappa[j]).collect();
            let dk: Vec<f64> = idxs.iter().map(|&j| dkappa_ds[j]).collect();
            classify_window(&k, &dk)
        };
        window_starts.push(i);
        window_classes.push(cls);
        i = end;
    }
    let smoothed_windows = smooth_classes(&window_classes);

    let mut classes = vec![CurvatureClass::Zero; n];
    for (w, &start) in window_starts.iter().enumerate() {
        let end = window_starts.get(w + 1).copied().unwrap_or(n);
        for j in start..end {
            classes[j] = flag[j].unwrap_or(smoothed_windows[w]);
        }
    }
    (kappa, classes)
}

// -- Display grid ------------------------------------------------------------

// A grid instant plus which carrier owns it. Every carrier boundary appears
// twice -- once as the closing sample of the interval that ends there
// (`Left`), once as the opening sample of the next one (`Right`) -- so a step
// in any derivative is rendered as a step instead of being averaged away by
// whichever side an arbitrary tie-break picked.
struct GridSample {
    t: f64,
    left: bool,
}

impl GridSample {
    fn side(&self) -> SampleSide {
        if self.left {
            SampleSide::Left
        } else {
            SampleSide::Right
        }
    }
}

// Sample density must follow time, not carrier count: a single analytic span
// or spline can cover tens of milliseconds of perfectly smooth motion, so a
// fixed per-interval count would leave millimeter-scale chords that render a
// genuinely smooth trajectory as a polyline.
const TARGET_SAMPLE_DT_S: f64 = 2.5e-4;

fn samples_in(a: f64, b: f64) -> usize {
    (((b - a) / TARGET_SAMPLE_DT_S).ceil() as usize).clamp(4, 512)
}

fn display_grid(traj: &ExactTrajectory) -> Vec<GridSample> {
    let bounds = traj.breakpoints();
    let cap: usize = bounds.windows(2).map(|w| samples_in(w[0], w[1])).sum();
    let mut grid = Vec::with_capacity(cap);
    for w in bounds.windows(2) {
        let (a, b) = (w[0], w[1]);
        let n = samples_in(a, b);
        for k in 0..n {
            grid.push(GridSample {
                t: a + (b - a) * (k as f64) / ((n - 1) as f64),
                left: k == n - 1,
            });
        }
    }
    grid
}

// An axis with no carriers plots as a flat zero track -- a snapshot case that
// never touches Z or E has nothing to render there. Every other evaluation
// failure is a pipeline or schema defect and surfaces as a load error.
fn eval(traj: &ExactTrajectory, axis: usize, sample: &GridSample) -> Result<Pvaj, JsValue> {
    if traj.rows(axis).is_empty() {
        return Ok(Pvaj {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
            jerk: 0.0,
        });
    }
    traj.eval_axis(axis, sample.t, sample.side())
        .map_err(|e| JsValue::from_str(&format!("exact evaluation failed: {e}")))
}

fn eval_side(
    traj: &ExactTrajectory,
    axis: usize,
    t: f64,
    side: SampleSide,
) -> Result<Pvaj, JsValue> {
    let left = matches!(side, SampleSide::Left);
    eval(traj, axis, &GridSample { t, left })
}

// -- Time series -------------------------------------------------------------

struct TimeSeries {
    t: Vec<f64>,
    // Toolhead position sampled on the same time grid as the derivatives, so the
    // path marker and the graph cursor index the same way.
    kin_x: Vec<f64>,
    kin_y: Vec<f64>,
    vx: Vec<f64>,
    vy: Vec<f64>,
    vz: Vec<f64>,
    ve: Vec<f64>,
    v_scalar: Vec<f64>,
    ax: Vec<f64>,
    ay: Vec<f64>,
    az: Vec<f64>,
    ae: Vec<f64>,
    a_scalar: Vec<f64>,
    jx: Vec<f64>,
    jy: Vec<f64>,
    jz: Vec<f64>,
    je: Vec<f64>,
    j_scalar: Vec<f64>,
}

impl TimeSeries {
    fn zeroed(n: usize) -> Self {
        let z = || vec![0.0; n];
        Self {
            t: z(),
            kin_x: z(),
            kin_y: z(),
            vx: z(),
            vy: z(),
            vz: z(),
            ve: z(),
            v_scalar: z(),
            ax: z(),
            ay: z(),
            az: z(),
            ae: z(),
            a_scalar: z(),
            jx: z(),
            jy: z(),
            jz: z(),
            je: z(),
            j_scalar: z(),
        }
    }
}

// The motor command on the display grid: position, velocity, acceleration and
// jerk all read straight off the exact carriers, so the jerk lane is the
// trajectory's own third derivative rather than a difference of samples.
fn time_series(traj: &ExactTrajectory, grid: &[GridSample]) -> Result<TimeSeries, JsValue> {
    if grid.is_empty() {
        return Ok(TimeSeries::zeroed(1));
    }
    let n = grid.len();
    let new = || Vec::with_capacity(n);
    let mut t = new();
    let (mut kin_x, mut vx, mut ax, mut jx) = (new(), new(), new(), new());
    let (mut kin_y, mut vy, mut ay, mut jy) = (new(), new(), new(), new());
    let (mut vz, mut az, mut jz) = (new(), new(), new());
    let (mut ve, mut ae, mut je) = (new(), new(), new());

    for sample in grid {
        let x = eval(traj, AXIS_X, sample)?;
        let y = eval(traj, AXIS_Y, sample)?;
        let z = eval(traj, AXIS_Z, sample)?;
        let e = eval(traj, AXIS_E, sample)?;
        t.push(sample.t);
        kin_x.push(x.position);
        vx.push(x.velocity);
        ax.push(x.acceleration);
        jx.push(x.jerk);
        kin_y.push(y.position);
        vy.push(y.velocity);
        ay.push(y.acceleration);
        jy.push(y.jerk);
        vz.push(z.velocity);
        az.push(z.acceleration);
        jz.push(z.jerk);
        ve.push(e.velocity);
        ae.push(e.acceleration);
        je.push(e.jerk);
    }

    let v_scalar = scalar_derivative(&vx, &vy);
    let a_scalar = scalar_derivative(&ax, &ay);
    let j_scalar = scalar_derivative(&jx, &jy);

    Ok(TimeSeries {
        t,
        kin_x,
        kin_y,
        vx,
        vy,
        vz,
        ve,
        v_scalar,
        ax,
        ay,
        az,
        ae,
        a_scalar,
        jx,
        jy,
        jz,
        je,
        j_scalar,
    })
}

#[derive(Default)]
struct ToolheadSeries {
    x: Vec<f64>,
    y: Vec<f64>,
    vx: Vec<f64>,
    vy: Vec<f64>,
    ax: Vec<f64>,
    ay: Vec<f64>,
    jx: Vec<f64>,
    jy: Vec<f64>,
    v_scalar: Vec<f64>,
    a_scalar: Vec<f64>,
    j_scalar: Vec<f64>,
    a_tang: Vec<f64>,
    a_cent: Vec<f64>,
    j_tang: Vec<f64>,
    j_cent: Vec<f64>,
    kappa: Vec<f64>,
}

// The toolhead signal sampled on the exact grid the motor-command series use,
// so the panels can overlay both without any resampling. Every derived motor
// series (|XY| scalars, Frenet ∥/⊥ projections, kappa) is mirrored with the
// identical formulas so the two signal families are directly comparable.
fn toolhead_series(traj: &ExactTrajectory, grid: &[GridSample]) -> Result<ToolheadSeries, JsValue> {
    let mut s = ToolheadSeries::default();
    for sample in grid {
        let x = eval(traj, AXIS_X, sample)?;
        let y = eval(traj, AXIS_Y, sample)?;
        s.x.push(x.position);
        s.y.push(y.position);
        s.vx.push(x.velocity);
        s.vy.push(y.velocity);
        s.ax.push(x.acceleration);
        s.ay.push(y.acceleration);
        s.jx.push(x.jerk);
        s.jy.push(y.jerk);
    }
    s.v_scalar = scalar_derivative(&s.vx, &s.vy);
    s.a_scalar = scalar_derivative(&s.ax, &s.ay);
    s.j_scalar = scalar_derivative(&s.jx, &s.jy);
    (s.a_tang, s.a_cent) = frenet_components(&s.vx, &s.vy, &s.ax, &s.ay);
    (s.j_tang, s.j_cent) = frenet_components(&s.vx, &s.vy, &s.jx, &s.jy);
    if has_xy(traj) {
        (s.kappa, _) = curvature_series(&s.vx, &s.vy, &s.ax, &s.ay, &s.jx, &s.jy);
    }
    Ok(s)
}

fn has_xy(traj: &ExactTrajectory) -> bool {
    !traj.rows(AXIS_X).is_empty() && !traj.rows(AXIS_Y).is_empty()
}

// -- Derivative impulses -----------------------------------------------------

// An acceleration step at a carrier boundary is a jerk impulse: a true Dirac
// -- infinite height, zero width -- so no exact jerk value can ever plot it
// and the jerk panel looks deceptively smooth across the step. Surface it
// honestly. The finite, physical strength of each impulse is the acceleration
// jump |Δa| across the boundary (∫ jerk dt over the impulse), read exactly as
// the difference between the two carriers' own values at that instant. A
// relative floor keeps float noise and trivial joints off the panel.
fn jerk_impulses(
    traj: &ExactTrajectory,
    a_peak: f64,
) -> Result<(Vec<f64>, Vec<f64>), JsValue> {
    discontinuities(traj, a_peak, |p| p.acceleration)
}

// A velocity step at a carrier boundary is an acceleration impulse. Surface it
// separately because finite graph samples cannot represent its infinite height.
fn accel_impulses(
    traj: &ExactTrajectory,
    v_peak: f64,
) -> Result<(Vec<f64>, Vec<f64>), JsValue> {
    discontinuities(traj, v_peak, |p| p.velocity)
}

// Shared boundary-step detector: evaluates `derivative_of` infinitesimally
// inside the carrier on each side of every interior XY boundary and reports
// the ones whose jump clears a relative floor of `scale_peak` (the peak value
// of the derivative one order up, e.g. the acceleration peak when comparing
// jerk steps).
fn discontinuities(
    traj: &ExactTrajectory,
    scale_peak: f64,
    derivative_of: impl Fn(&Pvaj) -> f64,
) -> Result<(Vec<f64>, Vec<f64>), JsValue> {
    if !has_xy(traj) {
        return Ok((Vec::new(), Vec::new()));
    }
    let mut bounds = traj.axis_breakpoints(AXIS_X);
    bounds.extend(traj.axis_breakpoints(AXIS_Y));
    bounds.sort_by(|a, b| a.partial_cmp(b).unwrap());
    bounds.dedup();

    let floor = (scale_peak * 1e-3).max(1e-9);
    let (mut times, mut mags) = (Vec::new(), Vec::new());
    for &b in bounds
        .iter()
        .take(bounds.len().saturating_sub(1))
        .skip(1)
    {
        let dxl = derivative_of(&eval_side(traj, AXIS_X, b, SampleSide::Left)?);
        let dxr = derivative_of(&eval_side(traj, AXIS_X, b, SampleSide::Right)?);
        let dyl = derivative_of(&eval_side(traj, AXIS_Y, b, SampleSide::Left)?);
        let dyr = derivative_of(&eval_side(traj, AXIS_Y, b, SampleSide::Right)?);
        let d = libm::hypot(dxr - dxl, dyr - dyl);
        if d > floor {
            times.push(b);
            mags.push(d);
        }
    }
    Ok((times, mags))
}

// -- WASM export -------------------------------------------------------------

#[wasm_bindgen]
pub struct TrajectoryData {
    raw_x: Vec<f64>,
    raw_y: Vec<f64>,
    kin_x: Vec<f64>,
    kin_y: Vec<f64>,
    kappa: Vec<f64>,
    curvature_class: Vec<f64>,
    t: Vec<f64>,
    vx: Vec<f64>,
    vy: Vec<f64>,
    vz: Vec<f64>,
    ve: Vec<f64>,
    v_scalar: Vec<f64>,
    ax: Vec<f64>,
    ay: Vec<f64>,
    az: Vec<f64>,
    ae: Vec<f64>,
    a_scalar: Vec<f64>,
    jx: Vec<f64>,
    jy: Vec<f64>,
    jz: Vec<f64>,
    je: Vec<f64>,
    j_scalar: Vec<f64>,
    a_tang: Vec<f64>,
    a_cent: Vec<f64>,
    j_tang: Vec<f64>,
    j_cent: Vec<f64>,
    toolhead: ToolheadSeries,
    jerk_impulse_t: Vec<f64>,
    jerk_impulse_mag: Vec<f64>,
    accel_impulse_t: Vec<f64>,
    accel_impulse_mag: Vec<f64>,
    seam_max_dp: Vec<f64>,
    seam_max_dv: Vec<f64>,
    seam_max_da: Vec<f64>,
    worst_seams_json: String,
    traversal_time_s: f64,
}

#[wasm_bindgen]
impl TrajectoryData {
    #[wasm_bindgen(constructor)]
    pub fn from_json(json: &str) -> Result<TrajectoryData, JsValue> {
        let snap: Snapshot =
            serde_json::from_str(json).map_err(|e| JsValue::from_str(&e.to_string()))?;

        let traj = &snap.trajectory;
        let grid = display_grid(traj);
        let ts = time_series(traj, &grid)?;

        let (kappa, classes) = if has_xy(traj) {
            curvature_series(&ts.vx, &ts.vy, &ts.ax, &ts.ay, &ts.jx, &ts.jy)
        } else {
            (
                vec![0.0; ts.t.len()],
                vec![CurvatureClass::Gap; ts.t.len()],
            )
        };
        let curvature_class: Vec<f64> = classes.iter().map(|c| c.code()).collect();

        let a_peak = ts.a_scalar.iter().copied().fold(0.0_f64, f64::max);
        let v_peak = ts.v_scalar.iter().copied().fold(0.0_f64, f64::max);
        let (jerk_impulse_t, jerk_impulse_mag) = jerk_impulses(traj, a_peak)?;
        let (accel_impulse_t, accel_impulse_mag) = accel_impulses(traj, v_peak)?;

        let toolhead = match &snap.toolhead {
            Some(th) => toolhead_series(th, &grid)?,
            None => ToolheadSeries::default(),
        };

        let (a_tang, a_cent) = frenet_components(&ts.vx, &ts.vy, &ts.ax, &ts.ay);
        let (j_tang, j_cent) = frenet_components(&ts.vx, &ts.vy, &ts.jx, &ts.jy);

        let worst_seams_json = serde_json::to_string(&snap.worst_seams)
            .map_err(|e| JsValue::from_str(&e.to_string()))?;

        Ok(TrajectoryData {
            raw_x: snap.raw_x,
            raw_y: snap.raw_y,
            kin_x: ts.kin_x,
            kin_y: ts.kin_y,
            kappa,
            curvature_class,
            t: ts.t,
            vx: ts.vx,
            vy: ts.vy,
            vz: ts.vz,
            ve: ts.ve,
            v_scalar: ts.v_scalar,
            ax: ts.ax,
            ay: ts.ay,
            az: ts.az,
            ae: ts.ae,
            a_scalar: ts.a_scalar,
            jx: ts.jx,
            jy: ts.jy,
            jz: ts.jz,
            je: ts.je,
            j_scalar: ts.j_scalar,
            a_tang,
            a_cent,
            j_tang,
            j_cent,
            toolhead,
            jerk_impulse_t,
            jerk_impulse_mag,
            accel_impulse_t,
            accel_impulse_mag,
            seam_max_dp: snap.seam_max_dp.to_vec(),
            seam_max_dv: snap.seam_max_dv.to_vec(),
            seam_max_da: snap.seam_max_da.to_vec(),
            worst_seams_json,
            traversal_time_s: snap.traversal_time_s,
        })
    }

    // Path getters
    pub fn raw_x(&self) -> Float64Array {
        Float64Array::from(&self.raw_x[..])
    }
    pub fn raw_y(&self) -> Float64Array {
        Float64Array::from(&self.raw_y[..])
    }
    pub fn kin_x(&self) -> Float64Array {
        Float64Array::from(&self.kin_x[..])
    }
    pub fn kin_y(&self) -> Float64Array {
        Float64Array::from(&self.kin_y[..])
    }

    // Time-series getters
    pub fn t(&self) -> Float64Array {
        Float64Array::from(&self.t[..])
    }
    pub fn vx(&self) -> Float64Array {
        Float64Array::from(&self.vx[..])
    }
    pub fn vy(&self) -> Float64Array {
        Float64Array::from(&self.vy[..])
    }
    pub fn v_scalar(&self) -> Float64Array {
        Float64Array::from(&self.v_scalar[..])
    }
    pub fn ax(&self) -> Float64Array {
        Float64Array::from(&self.ax[..])
    }
    pub fn ay(&self) -> Float64Array {
        Float64Array::from(&self.ay[..])
    }
    pub fn a_scalar(&self) -> Float64Array {
        Float64Array::from(&self.a_scalar[..])
    }
    pub fn jx(&self) -> Float64Array {
        Float64Array::from(&self.jx[..])
    }
    pub fn jy(&self) -> Float64Array {
        Float64Array::from(&self.jy[..])
    }
    pub fn j_scalar(&self) -> Float64Array {
        Float64Array::from(&self.j_scalar[..])
    }

    // Times and |Δaccel| of the jerk impulses at acceleration discontinuities.
    pub fn jerk_impulse_t(&self) -> Float64Array {
        Float64Array::from(&self.jerk_impulse_t[..])
    }
    pub fn jerk_impulse_mag(&self) -> Float64Array {
        Float64Array::from(&self.jerk_impulse_mag[..])
    }

    // Times and |Δvel| of the accel impulses at velocity discontinuities.
    pub fn accel_impulse_t(&self) -> Float64Array {
        Float64Array::from(&self.accel_impulse_t[..])
    }
    pub fn accel_impulse_mag(&self) -> Float64Array {
        Float64Array::from(&self.accel_impulse_mag[..])
    }

    // Z and E lane derivatives (axes 2 and 3). Flat zero when the case never
    // moves that axis.
    pub fn vz(&self) -> Float64Array {
        Float64Array::from(&self.vz[..])
    }
    pub fn ve(&self) -> Float64Array {
        Float64Array::from(&self.ve[..])
    }
    pub fn az(&self) -> Float64Array {
        Float64Array::from(&self.az[..])
    }
    pub fn ae(&self) -> Float64Array {
        Float64Array::from(&self.ae[..])
    }
    pub fn jz(&self) -> Float64Array {
        Float64Array::from(&self.jz[..])
    }
    pub fn je(&self) -> Float64Array {
        Float64Array::from(&self.je[..])
    }

    // XY acceleration and jerk projected onto the velocity tangent/normal
    // frame: tangential (signed) and centripetal/normal (magnitude).
    pub fn a_tang(&self) -> Float64Array {
        Float64Array::from(&self.a_tang[..])
    }
    pub fn a_cent(&self) -> Float64Array {
        Float64Array::from(&self.a_cent[..])
    }
    pub fn j_tang(&self) -> Float64Array {
        Float64Array::from(&self.j_tang[..])
    }
    pub fn j_cent(&self) -> Float64Array {
        Float64Array::from(&self.j_cent[..])
    }

    // The toolhead signal on the same grid as t(). All empty — and
    // has_toolhead() false — when the snapshot's motor command IS the
    // toolhead signal (no motor-side derivative-gain stage).
    pub fn has_toolhead(&self) -> bool {
        !self.toolhead.x.is_empty()
    }
    pub fn th_x(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.x[..])
    }
    pub fn th_y(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.y[..])
    }
    pub fn th_vx(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.vx[..])
    }
    pub fn th_vy(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.vy[..])
    }
    pub fn th_ax(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.ax[..])
    }
    pub fn th_ay(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.ay[..])
    }
    pub fn th_jx(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.jx[..])
    }
    pub fn th_jy(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.jy[..])
    }
    pub fn th_v_scalar(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.v_scalar[..])
    }
    pub fn th_a_scalar(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.a_scalar[..])
    }
    pub fn th_j_scalar(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.j_scalar[..])
    }
    pub fn th_a_tang(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.a_tang[..])
    }
    pub fn th_a_cent(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.a_cent[..])
    }
    pub fn th_j_tang(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.j_tang[..])
    }
    pub fn th_j_cent(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.j_cent[..])
    }
    pub fn th_kappa(&self) -> Float64Array {
        Float64Array::from(&self.toolhead.kappa[..])
    }

    // Per-axis (x, y, z, e) worst seam continuity jumps.
    pub fn seam_max_dp(&self) -> Float64Array {
        Float64Array::from(&self.seam_max_dp[..])
    }
    pub fn seam_max_dv(&self) -> Float64Array {
        Float64Array::from(&self.seam_max_dv[..])
    }
    pub fn seam_max_da(&self) -> Float64Array {
        Float64Array::from(&self.seam_max_da[..])
    }

    // The top offending seams as a JSON array of {t, axis, dp, dv, da}.
    pub fn worst_seams_json(&self) -> String {
        self.worst_seams_json.clone()
    }

    // Metadata
    pub fn traversal_time(&self) -> f64 {
        self.traversal_time_s
    }
    pub fn point_count(&self) -> usize {
        self.t.len()
    }

    // Signed curvature per sample, same grid as t()/vx()/etc. -- the
    // curvature-vs-time graph.
    pub fn kappa(&self) -> Float64Array {
        Float64Array::from(&self.kappa[..])
    }

    // Curvature-behavior class per sample as an integer code: 0=Zero,
    // 1=Constant, 2=Linear, 3=Other, 4=Cusp, 5=Gap. Same grid as kappa().
    pub fn curvature_class(&self) -> Float64Array {
        Float64Array::from(&self.curvature_class[..])
    }
}
