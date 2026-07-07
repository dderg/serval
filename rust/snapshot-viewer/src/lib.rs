use js_sys::Float64Array;
use serde::Deserialize;
use wasm_bindgen::prelude::*;

#[cfg(test)]
mod tests;

// -- Snapshot input types (matching Python snapshot dict) ---------------------

#[derive(Deserialize)]
struct Snapshot {
    raw_x: Vec<f64>,
    raw_y: Vec<f64>,
    fitted_segments: Vec<serde_json::Value>,
    traversal_time_s: f64,
    // The lowered trajectory the firmware executes: per-axis pieces
    // [t0, t1, c0, c1, …] of position vs time (cubic = 6 floats). Variable-length
    // so any baseline degree loads — missing high coefficients read as zero — and
    // the executed (post-lowering) path/derivatives never drop to the sampled
    // planner fallback on a format mismatch. Differentiated analytically.
    traj_x_pieces: Option<Vec<Vec<f64>>>,
    traj_y_pieces: Option<Vec<Vec<f64>>>,
    // Z (axis 2) and E (axis 3) lanes. Optional so legacy baselines that only
    // stored X/Y still deserialize; a missing lane plots as a flat zero track.
    #[serde(default)]
    traj_z_pieces: Option<Vec<Vec<f64>>>,
    #[serde(default)]
    traj_e_pieces: Option<Vec<Vec<f64>>>,
    traj_t_end: Option<f64>,
    // Per-axis (x, y, z, e) worst continuity jumps across piece seams, plus the
    // top offending seams. Optional so pre-seam-metric baselines still load.
    #[serde(default)]
    seam_max_dp: Option<Vec<f64>>,
    #[serde(default)]
    seam_max_dv: Option<Vec<f64>>,
    #[serde(default)]
    seam_max_da: Option<Vec<f64>>,
    #[serde(default)]
    worst_seams: Option<Vec<serde_json::Value>>,
    // Legacy baselines stored sampled position + speed instead of the cubics.
    kin_x: Option<Vec<f64>>,
    kin_y: Option<Vec<f64>>,
    kin_v: Option<Vec<f64>>,
    kin_s: Option<Vec<f64>>,
    kin_heading_x: Option<Vec<f64>>,
    kin_heading_y: Option<Vec<f64>>,
}

#[derive(Clone, Debug)]
enum SegmentType {
    Line { x0: f64, y0: f64, x1: f64, y1: f64 },
    Arc { points: Vec<[f64; 2]> },
    Clothoid { points: Vec<[f64; 2]> },
}

// -- Numerical gradient (matches numpy.gradient) ----------------------------

fn gradient(values: &[f64], times: &[f64]) -> Vec<f64> {
    let n = values.len();
    if n == 0 {
        return vec![];
    }
    if n == 1 {
        return vec![0.0];
    }

    let mut result = vec![0.0; n];

    // Forward difference for first point
    let dt0 = times[1] - times[0];
    result[0] = if dt0.abs() > 1e-30 {
        (values[1] - values[0]) / dt0
    } else {
        0.0
    };

    // Central differences for interior
    for i in 1..n - 1 {
        let dt = times[i + 1] - times[i - 1];
        result[i] = if dt.abs() > 1e-30 {
            (values[i + 1] - values[i - 1]) / dt
        } else {
            0.0
        };
    }

    // Backward difference for last point
    let dt_last = times[n - 1] - times[n - 2];
    result[n - 1] = if dt_last.abs() > 1e-30 {
        (values[n - 1] - values[n - 2]) / dt_last
    } else {
        0.0
    };

    result
}

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

// -- Toolhead position (handles legacy format) ------------------------------

fn toolhead_position(snap: &Snapshot) -> (Vec<f64>, Vec<f64>) {
    if let (Some(kx), Some(ky)) = (&snap.kin_x, &snap.kin_y) {
        return (kx.clone(), ky.clone());
    }
    // Legacy: integrate heading along arc length
    let s: Vec<f64> = snap.kin_s.as_ref().cloned().unwrap_or_default();
    let hx: Vec<f64> = snap.kin_heading_x.as_ref().cloned().unwrap_or_default();
    let hy: Vec<f64> = snap.kin_heading_y.as_ref().cloned().unwrap_or_default();
    let n = s.len();
    let mut x = vec![0.0; n];
    let mut y = vec![0.0; n];
    if n == 0 {
        return (x, y);
    }
    x[0] = snap.raw_x.first().copied().unwrap_or(0.0);
    y[0] = snap.raw_y.first().copied().unwrap_or(0.0);
    let mut cx = x[0];
    let mut cy = y[0];
    for i in 1..n {
        let ds = s[i] - s[i - 1];
        cx += hx[i] * ds;
        cy += hy[i] * ds;
        x[i] = cx;
        y[i] = cy;
    }
    (x, y)
}

// -- Time series computation ------------------------------------------------

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

// Evaluate one per-axis monomial piece [t0, t1, c0, c1, …, cn] -- the lowered
// trajectory the firmware runs -- and its analytic derivatives at time `ti`.
// Within a piece pos = sum(ck * tau^k) (tau = ti - t0), so velocity,
// acceleration, and jerk are exact polynomial derivatives, no differencing.
// Degree-generic: any coefficient count from the row (c0..cn, n = len - 3) is
// summed via Horner; a piece with fewer than 6 floats (e.g. a linear or
// quadratic row) simply has no higher terms to contribute.
fn eval_piece(p: &[f64], ti: f64) -> (f64, f64, f64, f64) {
    let tau = ti - p[0];
    let coeffs = &p[2..];
    let (mut pos, mut vel, mut acc, mut jerk) = (0.0, 0.0, 0.0, 0.0);
    for (k, &ck) in coeffs.iter().enumerate().rev() {
        pos = pos * tau + ck;
        if k >= 1 {
            vel = vel * tau + ck * (k as f64);
        }
        if k >= 2 {
            acc = acc * tau + ck * (k as f64) * ((k - 1) as f64);
        }
        if k >= 3 {
            jerk = jerk * tau + ck * (k as f64) * ((k - 1) as f64) * ((k - 2) as f64);
        }
    }
    (pos, vel, acc, jerk)
}

// Last piece whose start is <= ti, clamped into range (searchsorted-right - 1).
fn piece_at(pieces: &[Vec<f64>], ti: f64) -> usize {
    pieces
        .partition_point(|p| p[0] <= ti)
        .saturating_sub(1)
        .min(pieces.len() - 1)
}

// Evaluate a lane on the shared time grid; an absent lane (empty) reads zero so
// legacy X/Y-only baselines still produce full-length Z/E tracks.
fn eval_lane(pieces: &[Vec<f64>], ti: f64) -> (f64, f64, f64, f64) {
    if pieces.is_empty() {
        return (0.0, 0.0, 0.0, 0.0);
    }
    eval_piece(&pieces[piece_at(pieces, ti)], ti)
}

fn time_series_from_pieces(
    xp: &[Vec<f64>],
    yp: &[Vec<f64>],
    zp: &[Vec<f64>],
    ep: &[Vec<f64>],
    t_end: f64,
) -> TimeSeries {
    if xp.is_empty() || yp.is_empty() || t_end <= 0.0 {
        return TimeSeries::zeroed(1);
    }

    // Acceleration is linear within a cubic piece and steps at a piece boundary,
    // so every accel peak sits exactly on a boundary -- a uniform grid lands a
    // hair off it and the peak wobbles as t_end shifts between before/after.
    // Sample each interval between consecutive boundaries (of ALL lanes) on its
    // own piece with both endpoints included: every step gets a sample on each
    // side, so the plotted accel/jerk are exact and stable.
    let mut bounds: Vec<f64> = vec![0.0, t_end];
    for p in xp.iter().chain(yp).chain(zp).chain(ep) {
        for edge in [p[0], p[1]] {
            if edge > 0.0 && edge < t_end {
                bounds.push(edge);
            }
        }
    }
    bounds.sort_by(|a, b| a.partial_cmp(b).unwrap());
    bounds.dedup();

    // Sample density must follow time, not piece count: since the lowering
    // fits one long piece where the signal is smooth (an arc can span tens of
    // milliseconds), a fixed per-interval count leaves millimeter-scale chords
    // that render a genuinely smooth trajectory as a polyline.
    const TARGET_SAMPLE_DT_S: f64 = 2.5e-4;
    let per_interval_of =
        |a: f64, b: f64| -> usize { (((b - a) / TARGET_SAMPLE_DT_S).ceil() as usize).clamp(4, 512) };
    let cap: usize = bounds.windows(2).map(|w| per_interval_of(w[0], w[1])).sum();

    let new = || Vec::with_capacity(cap);
    let mut t = new();
    let (mut kin_x, mut vx, mut ax, mut jx) = (new(), new(), new(), new());
    let (mut kin_y, mut vy, mut ay, mut jy) = (new(), new(), new(), new());
    let (mut vz, mut az, mut jz) = (new(), new(), new());
    let (mut ve, mut ae, mut je) = (new(), new(), new());

    for w in bounds.windows(2) {
        let (a, b) = (w[0], w[1]);
        let per_interval = per_interval_of(a, b);
        for k in 0..per_interval {
            let ti = a + (b - a) * (k as f64) / ((per_interval - 1) as f64);
            let (x, vxk, axk, jxk) = eval_lane(xp, ti);
            let (y, vyk, ayk, jyk) = eval_lane(yp, ti);
            let (_, vzk, azk, jzk) = eval_lane(zp, ti);
            let (_, vek, aek, jek) = eval_lane(ep, ti);
            t.push(ti);
            kin_x.push(x);
            vx.push(vxk);
            ax.push(axk);
            jx.push(jxk);
            kin_y.push(y);
            vy.push(vyk);
            ay.push(ayk);
            jy.push(jyk);
            vz.push(vzk);
            az.push(azk);
            jz.push(jzk);
            ve.push(vek);
            ae.push(aek);
            je.push(jek);
        }
    }

    let v_scalar = scalar_derivative(&vx, &vy);
    let a_scalar = scalar_derivative(&ax, &ay);
    let j_scalar = scalar_derivative(&jx, &jy);

    TimeSeries {
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
    }
}

// Acceleration steps at a piece boundary are jerk impulses: a true Dirac --
// infinite height, zero width -- so the per-piece analytic jerk can never plot
// them and the jerk panel looks deceptively smooth across the step. Surface
// them honestly. The finite, physical strength of each impulse is the
// acceleration jump |Δa| across the boundary (∫ jerk dt over the impulse). The
// C1 Hermite lowering matches position and velocity at joints but not
// acceleration, so most joints step a little; a relative floor keeps float
// noise and trivial joints off the panel.
fn jerk_impulses(
    xp: &[Vec<f64>],
    yp: &[Vec<f64>],
    t_end: f64,
    a_peak: f64,
) -> (Vec<f64>, Vec<f64>) {
    discontinuities(xp, yp, t_end, a_peak, |p, t| eval_piece(p, t).2)
}

// A step in velocity at a piece boundary is an acceleration impulse: the same
// Dirac-delta reasoning as `jerk_impulses`, one derivative order down. The C1
// Hermite lowering is supposed to match velocity at every joint, so a nonzero
// one here is a real discontinuity the position/velocity graphs render as a
// sharp corner rather than a smooth curve -- surface it the same way.
fn accel_impulses(
    xp: &[Vec<f64>],
    yp: &[Vec<f64>],
    t_end: f64,
    v_peak: f64,
) -> (Vec<f64>, Vec<f64>) {
    discontinuities(xp, yp, t_end, v_peak, |p, t| eval_piece(p, t).1)
}

// Shared boundary-step detector: samples `derivative_at` on both sides of
// every piece boundary and reports the ones whose jump clears a relative
// floor of `scale_peak` (the peak value of the derivative one order up, e.g.
// the acceleration peak when comparing jerk steps).
fn discontinuities(
    xp: &[Vec<f64>],
    yp: &[Vec<f64>],
    t_end: f64,
    scale_peak: f64,
    derivative_at: impl Fn(&[f64], f64) -> f64,
) -> (Vec<f64>, Vec<f64>) {
    if xp.is_empty() || yp.is_empty() || t_end <= 0.0 {
        return (Vec::new(), Vec::new());
    }
    let mut bounds: Vec<f64> = vec![0.0, t_end];
    for p in xp.iter().chain(yp.iter()) {
        for edge in [p[0], p[1]] {
            if edge > 0.0 && edge < t_end {
                bounds.push(edge);
            }
        }
    }
    bounds.sort_by(|a, b| a.partial_cmp(b).unwrap());
    bounds.dedup();

    let floor = (scale_peak * 1e-3).max(1e-9);
    let (mut times, mut mags) = (Vec::new(), Vec::new());
    for i in 1..bounds.len().saturating_sub(1) {
        let b = bounds[i];
        let lo = 0.5 * (bounds[i - 1] + b);
        let hi = 0.5 * (b + bounds[i + 1]);
        let dxl = derivative_at(&xp[piece_at(xp, lo)], b);
        let dxr = derivative_at(&xp[piece_at(xp, hi)], b);
        let dyl = derivative_at(&yp[piece_at(yp, lo)], b);
        let dyr = derivative_at(&yp[piece_at(yp, hi)], b);
        let d = libm::hypot(dxr - dxl, dyr - dyl);
        if d > floor {
            times.push(b);
            mags.push(d);
        }
    }
    (times, mags)
}

fn build_time_series(snap: &Snapshot) -> TimeSeries {
    if let (Some(xp), Some(yp)) = (&snap.traj_x_pieces, &snap.traj_y_pieces) {
        let t_end = snap.traj_t_end.unwrap_or(0.0);
        let empty: Vec<Vec<f64>> = Vec::new();
        let zp = snap.traj_z_pieces.as_ref().unwrap_or(&empty);
        let ep = snap.traj_e_pieces.as_ref().unwrap_or(&empty);
        return time_series_from_pieces(xp, yp, zp, ep, t_end);
    }
    time_series_from_position(snap)
}

fn time_series_from_position(snap: &Snapshot) -> TimeSeries {
    let (x_raw, y_raw) = toolhead_position(snap);
    let v_raw: Vec<f64> = snap.kin_v.clone().unwrap_or_default();

    if x_raw.is_empty() || v_raw.is_empty() {
        return TimeSeries::zeroed(1);
    }

    // Filter distinct points
    let mut x = Vec::with_capacity(x_raw.len());
    let mut y = Vec::with_capacity(y_raw.len());
    let mut v = Vec::with_capacity(v_raw.len());
    x.push(x_raw[0]);
    y.push(y_raw[0]);
    v.push(v_raw[0]);
    for i in 1..x_raw.len() {
        let dx = x_raw[i] - x_raw[i - 1];
        let dy = y_raw[i] - y_raw[i - 1];
        if libm::hypot(dx, dy) > 1e-9 {
            x.push(x_raw[i]);
            y.push(y_raw[i]);
            v.push(v_raw[i]);
        }
    }

    let n = x.len();
    if n < 2 {
        let mut ts = TimeSeries::zeroed(n);
        ts.kin_x = x;
        ts.kin_y = y;
        return ts;
    }

    // Build time axis: dt = ds / v_avg
    let v_safe: Vec<f64> = v.iter().map(|vi| vi.max(1e-6)).collect();
    let mut ds = Vec::with_capacity(n - 1);
    for i in 0..n - 1 {
        let dx = x[i + 1] - x[i];
        let dy = y[i + 1] - y[i];
        ds.push(libm::hypot(dx, dy));
    }

    let mut v_avg = Vec::with_capacity(n - 1);
    for i in 0..n - 1 {
        v_avg.push(0.5 * (v_safe[i] + v_safe[i + 1]));
    }

    let mut t = vec![0.0; n];
    let mut cumsum = 0.0;
    for i in 0..n - 1 {
        let dt = if v_avg[i] > 1e-30 {
            ds[i] / v_avg[i]
        } else {
            0.0
        };
        cumsum += dt;
        t[i + 1] = cumsum;
    }

    // Derivatives
    let vx = gradient(&x, &t);
    let vy = gradient(&y, &t);
    let v_scalar: Vec<f64> = vx
        .iter()
        .zip(&vy)
        .map(|(vx, vy)| libm::hypot(*vx, *vy))
        .collect();

    let ax = gradient(&vx, &t);
    let ay = gradient(&vy, &t);
    let a_scalar = scalar_derivative(&ax, &ay);

    let jx = gradient(&ax, &t);
    let jy = gradient(&ay, &t);
    let j_scalar = scalar_derivative(&jx, &jy);

    let zeros = || vec![0.0; n];
    TimeSeries {
        t,
        kin_x: x,
        kin_y: y,
        vx,
        vy,
        vz: zeros(),
        ve: zeros(),
        v_scalar,
        ax,
        ay,
        az: zeros(),
        ae: zeros(),
        a_scalar,
        jx,
        jy,
        jz: zeros(),
        je: zeros(),
        j_scalar,
    }
}

// -- Fitted segment parsing -------------------------------------------------

fn parse_segments(raw: &[serde_json::Value]) -> Vec<SegmentType> {
    let mut segments = Vec::new();
    for val in raw {
        let Some(typ) = val.get("type").and_then(serde_json::Value::as_str) else {
            continue;
        };
        match typ {
            "line" => {
                let x0 = val
                    .get("x0")
                    .and_then(serde_json::Value::as_f64)
                    .unwrap_or(0.0);
                let y0 = val
                    .get("y0")
                    .and_then(serde_json::Value::as_f64)
                    .unwrap_or(0.0);
                let x1 = val
                    .get("x1")
                    .and_then(serde_json::Value::as_f64)
                    .unwrap_or(0.0);
                let y1 = val
                    .get("y1")
                    .and_then(serde_json::Value::as_f64)
                    .unwrap_or(0.0);
                segments.push(SegmentType::Line { x0, y0, x1, y1 });
            }
            "arc" | "clothoid" => {
                let xs = val.get("x").and_then(serde_json::Value::as_array);
                let ys = val.get("y").and_then(serde_json::Value::as_array);
                let points = match (xs, ys) {
                    (Some(xs), Some(ys)) => xs
                        .iter()
                        .zip(ys.iter())
                        .map(|(x, y)| [x.as_f64().unwrap_or(0.0), y.as_f64().unwrap_or(0.0)])
                        .collect(),
                    _ => vec![],
                };
                if typ == "arc" {
                    segments.push(SegmentType::Arc { points });
                } else {
                    segments.push(SegmentType::Clothoid { points });
                }
            }
            _ => {}
        }
    }
    segments
}

// -- WASM export -------------------------------------------------------------

#[wasm_bindgen]
pub struct TrajectoryData {
    raw_x: Vec<f64>,
    raw_y: Vec<f64>,
    kin_x: Vec<f64>,
    kin_y: Vec<f64>,
    segments: Vec<SegmentType>,
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

        let ts = build_time_series(&snap);
        let segments = parse_segments(&snap.fitted_segments);

        let a_peak = ts.a_scalar.iter().copied().fold(0.0_f64, f64::max);
        let v_peak = ts.v_scalar.iter().copied().fold(0.0_f64, f64::max);
        let (jerk_impulse_t, jerk_impulse_mag, accel_impulse_t, accel_impulse_mag) =
            match (&snap.traj_x_pieces, &snap.traj_y_pieces) {
                (Some(xp), Some(yp)) => {
                    let t_end = snap.traj_t_end.unwrap_or(0.0);
                    let (jt, jm) = jerk_impulses(xp, yp, t_end, a_peak);
                    let (at, am) = accel_impulses(xp, yp, t_end, v_peak);
                    (jt, jm, at, am)
                }
                _ => (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
            };

        let seam_axis = |v: &Option<Vec<f64>>| v.clone().unwrap_or_default();

        let (a_tang, a_cent) = frenet_components(&ts.vx, &ts.vy, &ts.ax, &ts.ay);
        let (j_tang, j_cent) = frenet_components(&ts.vx, &ts.vy, &ts.jx, &ts.jy);

        Ok(TrajectoryData {
            raw_x: snap.raw_x,
            raw_y: snap.raw_y,
            kin_x: ts.kin_x,
            kin_y: ts.kin_y,
            segments,
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
            jerk_impulse_t,
            jerk_impulse_mag,
            accel_impulse_t,
            accel_impulse_mag,
            seam_max_dp: seam_axis(&snap.seam_max_dp),
            seam_max_dv: seam_axis(&snap.seam_max_dv),
            seam_max_da: seam_axis(&snap.seam_max_da),
            worst_seams_json: snap
                .worst_seams
                .as_ref()
                .and_then(|w| serde_json::to_string(w).ok())
                .unwrap_or_else(|| "[]".to_string()),
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

    // Z and E lane derivatives (axes 2 and 3). Empty on legacy X/Y baselines.
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

    // Per-axis (x, y, z, e) worst seam continuity jumps. Empty on baselines
    // recorded before seam metrics existed.
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

    // Segment access
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    pub fn segment_type(&self, i: usize) -> String {
        match self.segments.get(i) {
            Some(SegmentType::Line { .. }) => "line".to_string(),
            Some(SegmentType::Arc { .. }) => "arc".to_string(),
            Some(SegmentType::Clothoid { .. }) => "clothoid".to_string(),
            None => "unknown".to_string(),
        }
    }

    /// Returns flattened segment data: [x0,y0,x1,y1] for lines, [x0,y0,...,xN,yN] for arcs/clothoids
    pub fn segment_data(&self, i: usize) -> Float64Array {
        let flat: Vec<f64> = match self.segments.get(i) {
            Some(SegmentType::Line { x0, y0, x1, y1 }) => vec![*x0, *y0, *x1, *y1],
            Some(SegmentType::Arc { points }) | Some(SegmentType::Clothoid { points }) => {
                points.iter().flat_map(|p| [p[0], p[1]]).collect()
            }
            None => vec![],
        };
        Float64Array::from(&flat[..])
    }
}
