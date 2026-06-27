use js_sys::Float64Array;
use serde::Deserialize;
use wasm_bindgen::prelude::*;

// -- Snapshot input types (matching Python snapshot dict) ---------------------

#[derive(Deserialize)]
struct Snapshot {
    raw_x: Vec<f64>,
    raw_y: Vec<f64>,
    fitted_segments: Vec<serde_json::Value>,
    traversal_time_s: f64,
    blended_corners: usize,
    chain_fits: usize,
    unblended_corners: usize,
    // The lowered trajectory the firmware executes: per-axis pieces
    // [t0, t1, c0, c1, …] of position vs time (cubic = 6 floats). Variable-length
    // so any baseline degree loads — missing high coefficients read as zero — and
    // the executed (post-lowering) path/derivatives never drop to the sampled
    // planner fallback on a format mismatch. Differentiated analytically.
    traj_x_pieces: Option<Vec<Vec<f64>>>,
    traj_y_pieces: Option<Vec<Vec<f64>>>,
    traj_t_end: Option<f64>,
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
        .map(|(ax, ay)| ax.hypot(*ay))
        .collect()
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
    v_scalar: Vec<f64>,
    ax: Vec<f64>,
    ay: Vec<f64>,
    a_scalar: Vec<f64>,
    jx: Vec<f64>,
    jy: Vec<f64>,
    j_scalar: Vec<f64>,
}

// Evaluate per-axis monomial pieces [t0, t1, c0, c1, …] -- the lowered trajectory
// the firmware runs -- and their analytic derivatives at times `t`. Within a piece,
// pos = c0 + c1*tau + … (tau = t - t0), so velocity, acceleration, and jerk are
// exact polynomial derivatives, no differencing. Length-tolerant: a 6-float cubic
// reads c4 = c5 = 0, so it evaluates as the cubic it is.
fn eval_pieces(pieces: &[Vec<f64>], t: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let n = pieces.len();
    let mut pos = Vec::with_capacity(t.len());
    let mut vel = Vec::with_capacity(t.len());
    let mut acc = Vec::with_capacity(t.len());
    let mut jerk = Vec::with_capacity(t.len());
    for &ti in t {
        // Last piece whose start is <= ti, clamped into range (searchsorted-right - 1).
        let upper = pieces.partition_point(|p| p[0] <= ti);
        let idx = upper.saturating_sub(1).min(n - 1);
        let p = &pieces[idx];
        let g = |i: usize| p.get(i).copied().unwrap_or(0.0);
        let tau = ti - p[0];
        let (c0, c1, c2, c3, c4, c5) = (g(2), g(3), g(4), g(5), g(6), g(7));
        let (t2, t3, t4, t5) = (tau * tau, tau.powi(3), tau.powi(4), tau.powi(5));
        pos.push(c0 + c1 * tau + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5);
        vel.push(c1 + 2.0 * c2 * tau + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4);
        acc.push(2.0 * c2 + 6.0 * c3 * tau + 12.0 * c4 * t2 + 20.0 * c5 * t3);
        jerk.push(6.0 * c3 + 24.0 * c4 * tau + 60.0 * c5 * t2);
    }
    (pos, vel, acc, jerk)
}

fn time_series_from_pieces(xp: &[Vec<f64>], yp: &[Vec<f64>], t_end: f64) -> TimeSeries {
    if xp.is_empty() || t_end <= 0.0 {
        let z = vec![0.0];
        return TimeSeries {
            t: z.clone(),
            kin_x: z.clone(),
            kin_y: z.clone(),
            vx: z.clone(),
            vy: z.clone(),
            v_scalar: z.clone(),
            ax: z.clone(),
            ay: z.clone(),
            a_scalar: z.clone(),
            jx: z.clone(),
            jy: z.clone(),
            j_scalar: z,
        };
    }

    // Dense grid plus the exact piece boundaries, where the C1 Hermite lowering
    // lets acceleration step -- so the piecewise structure draws faithfully.
    let n_grid = (8 * xp.len()).max(2000);
    let mut t: Vec<f64> = (0..n_grid)
        .map(|i| t_end * (i as f64) / ((n_grid - 1) as f64))
        .collect();
    t.extend(xp.iter().map(|p| p[1]));
    t.sort_by(|a, b| a.partial_cmp(b).unwrap());
    t.dedup();

    let (kin_x, vx, ax, jx) = eval_pieces(xp, &t);
    let (kin_y, vy, ay, jy) = eval_pieces(yp, &t);
    let v_scalar = scalar_derivative(&vx, &vy);
    let a_scalar = scalar_derivative(&ax, &ay);
    let j_scalar = scalar_derivative(&jx, &jy);

    TimeSeries {
        t,
        kin_x,
        kin_y,
        vx,
        vy,
        v_scalar,
        ax,
        ay,
        a_scalar,
        jx,
        jy,
        j_scalar,
    }
}

fn build_time_series(snap: &Snapshot) -> TimeSeries {
    if let (Some(xp), Some(yp)) = (&snap.traj_x_pieces, &snap.traj_y_pieces) {
        let t_end = snap.traj_t_end.unwrap_or(0.0);
        return time_series_from_pieces(xp, yp, t_end);
    }
    time_series_from_position(snap)
}

fn time_series_from_position(snap: &Snapshot) -> TimeSeries {
    let (x_raw, y_raw) = toolhead_position(snap);
    let v_raw: Vec<f64> = snap.kin_v.clone().unwrap_or_default();

    if x_raw.is_empty() || v_raw.is_empty() {
        let z = vec![0.0];
        return TimeSeries {
            t: z.clone(),
            kin_x: z.clone(),
            kin_y: z.clone(),
            vx: z.clone(),
            vy: z.clone(),
            v_scalar: z.clone(),
            ax: z.clone(),
            ay: z.clone(),
            a_scalar: z.clone(),
            jx: z.clone(),
            jy: z.clone(),
            j_scalar: z,
        };
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
        if dx.hypot(dy) > 1e-9 {
            x.push(x_raw[i]);
            y.push(y_raw[i]);
            v.push(v_raw[i]);
        }
    }

    let n = x.len();
    if n < 2 {
        return TimeSeries {
            t: vec![0.0; n],
            kin_x: x,
            kin_y: y,
            vx: vec![0.0; n],
            vy: vec![0.0; n],
            v_scalar: vec![0.0; n],
            ax: vec![0.0; n],
            ay: vec![0.0; n],
            a_scalar: vec![0.0; n],
            jx: vec![0.0; n],
            jy: vec![0.0; n],
            j_scalar: vec![0.0; n],
        };
    }

    // Build time axis: dt = ds / v_avg
    let v_safe: Vec<f64> = v.iter().map(|vi| vi.max(1e-6)).collect();
    let mut ds = Vec::with_capacity(n - 1);
    for i in 0..n - 1 {
        let dx = x[i + 1] - x[i];
        let dy = y[i + 1] - y[i];
        ds.push(dx.hypot(dy));
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
    let v_scalar: Vec<f64> = vx.iter().zip(&vy).map(|(vx, vy)| vx.hypot(*vy)).collect();

    let ax = gradient(&vx, &t);
    let ay = gradient(&vy, &t);
    let a_scalar = scalar_derivative(&ax, &ay);

    let jx = gradient(&ax, &t);
    let jy = gradient(&ay, &t);
    let j_scalar = scalar_derivative(&jx, &jy);

    TimeSeries {
        t,
        kin_x: x,
        kin_y: y,
        vx,
        vy,
        v_scalar,
        ax,
        ay,
        a_scalar,
        jx,
        jy,
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
    v_scalar: Vec<f64>,
    ax: Vec<f64>,
    ay: Vec<f64>,
    a_scalar: Vec<f64>,
    jx: Vec<f64>,
    jy: Vec<f64>,
    j_scalar: Vec<f64>,
    traversal_time_s: f64,
    blended_corners: usize,
    chain_fits: usize,
    unblended_corners: usize,
}

#[wasm_bindgen]
impl TrajectoryData {
    #[wasm_bindgen(constructor)]
    pub fn from_json(json: &str) -> Result<TrajectoryData, JsValue> {
        let snap: Snapshot =
            serde_json::from_str(json).map_err(|e| JsValue::from_str(&e.to_string()))?;

        let ts = build_time_series(&snap);
        let segments = parse_segments(&snap.fitted_segments);

        Ok(TrajectoryData {
            raw_x: snap.raw_x,
            raw_y: snap.raw_y,
            kin_x: ts.kin_x,
            kin_y: ts.kin_y,
            segments,
            t: ts.t,
            vx: ts.vx,
            vy: ts.vy,
            v_scalar: ts.v_scalar,
            ax: ts.ax,
            ay: ts.ay,
            a_scalar: ts.a_scalar,
            jx: ts.jx,
            jy: ts.jy,
            j_scalar: ts.j_scalar,
            traversal_time_s: snap.traversal_time_s,
            blended_corners: snap.blended_corners,
            chain_fits: snap.chain_fits,
            unblended_corners: snap.unblended_corners,
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

    // Metadata
    pub fn traversal_time(&self) -> f64 {
        self.traversal_time_s
    }
    pub fn blended_corners(&self) -> usize {
        self.blended_corners
    }
    pub fn chain_fits(&self) -> usize {
        self.chain_fits
    }
    pub fn unblended_corners(&self) -> usize {
        self.unblended_corners
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
