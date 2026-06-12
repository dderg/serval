use nurbs::ScalarNurbs;

const GL8_NODES: [f64; 8] = [
    -0.960_289_856_497_536_3,
    -0.796_666_477_413_626_7,
    -0.525_532_409_916_329,
    -0.183_434_642_495_649_8,
    0.183_434_642_495_649_8,
    0.525_532_409_916_329,
    0.796_666_477_413_626_7,
    0.960_289_856_497_536_3,
];
const GL8_WEIGHTS: [f64; 8] = [
    0.101_228_536_290_376_26,
    0.222_381_034_453_374_47,
    0.313_706_645_877_887_3,
    0.362_683_783_378_362,
    0.362_683_783_378_362,
    0.313_706_645_877_887_3,
    0.222_381_034_453_374_47,
    0.101_228_536_290_376_26,
];

const BREAKPOINT_MERGE_TOL: f64 = 1e-12;
const SPAN_COVERAGE_TOL: f64 = 1e-9;

#[derive(Debug, thiserror::Error)]
pub enum OdometerError {
    #[error("odometer: no axes given")]
    EmptyAxes,
    #[error("odometer: invalid time domain [{t_start}, {t_end}]")]
    InvalidDomain { t_start: f64, t_end: f64 },
    #[error(
        "odometer: ratio spans must cover [{t_start}, {t_end}] exactly; \
         got span ends {span_ends:?}"
    )]
    SpanCoverage {
        t_start: f64,
        t_end: f64,
        span_ends: Vec<f64>,
    },
}

#[derive(Debug)]
pub struct Odometer {
    breakpoints: Vec<f64>,
    cumulative: Vec<f64>,
    derivatives: Vec<ScalarNurbs<f64>>,
}

impl Odometer {
    pub fn build(
        axes: &[ScalarNurbs<f64>],
        t_start: f64,
        t_end: f64,
        min_intervals: usize,
    ) -> Result<Self, OdometerError> {
        if axes.is_empty() {
            return Err(OdometerError::EmptyAxes);
        }
        if !(t_start.is_finite() && t_end.is_finite() && t_end > t_start) {
            return Err(OdometerError::InvalidDomain { t_start, t_end });
        }

        let breakpoints = refined_breakpoints(axes, t_start, t_end, min_intervals);
        let derivatives: Vec<ScalarNurbs<f64>> = axes.iter().map(nurbs::eval::derivative).collect();

        let mut cumulative = Vec::with_capacity(breakpoints.len());
        cumulative.push(0.0);
        for window in breakpoints.windows(2) {
            let segment_length = gl8_speed_integral(&derivatives, window[0], window[1]);
            cumulative.push(cumulative.last().unwrap() + segment_length);
        }

        Ok(Self {
            breakpoints,
            cumulative,
            derivatives,
        })
    }

    #[must_use]
    pub fn t_start(&self) -> f64 {
        self.breakpoints[0]
    }

    #[must_use]
    pub fn t_end(&self) -> f64 {
        *self.breakpoints.last().unwrap()
    }

    #[must_use]
    pub fn distance_at(&self, t: f64) -> f64 {
        assert!(
            t >= self.t_start() - BREAKPOINT_MERGE_TOL && t <= self.t_end() + BREAKPOINT_MERGE_TOL,
            "odometer: t={t} outside domain [{}, {}]",
            self.t_start(),
            self.t_end(),
        );
        let t = t.clamp(self.t_start(), self.t_end());
        let interval = match self
            .breakpoints
            .binary_search_by(|bp| bp.partial_cmp(&t).unwrap())
        {
            Ok(exact) => return self.cumulative[exact],
            Err(insertion) => insertion - 1,
        };
        self.cumulative[interval]
            + gl8_speed_integral(&self.derivatives, self.breakpoints[interval], t)
    }

    #[must_use]
    pub fn speed_at(&self, t: f64) -> f64 {
        speed(&self.derivatives, t)
    }
}

pub fn follower_track<'a>(
    odo: &'a Odometer,
    start: f64,
    ratio_spans: &[(f64, f64)],
    t_start: f64,
    t_end: f64,
) -> Result<impl Fn(f64) -> f64 + 'a, OdometerError> {
    validate_span_coverage(ratio_spans, t_start, t_end)?;
    let spans = ratio_spans.to_vec();
    Ok(move |t: f64| {
        let mut position = start;
        let mut span_start = t_start;
        for &(span_end, ratio) in &spans {
            if t <= span_start {
                break;
            }
            let upto = t.min(span_end);
            position += ratio * (odo.distance_at(upto) - odo.distance_at(span_start));
            span_start = span_end;
        }
        position
    })
}

fn validate_span_coverage(
    ratio_spans: &[(f64, f64)],
    t_start: f64,
    t_end: f64,
) -> Result<(), OdometerError> {
    let coverage_error = || OdometerError::SpanCoverage {
        t_start,
        t_end,
        span_ends: ratio_spans.iter().map(|s| s.0).collect(),
    };
    let mut cursor = t_start;
    for &(span_end, _) in ratio_spans {
        if span_end <= cursor {
            return Err(coverage_error());
        }
        cursor = span_end;
    }
    if (cursor - t_end).abs() > SPAN_COVERAGE_TOL {
        return Err(coverage_error());
    }
    Ok(())
}

fn refined_breakpoints(
    axes: &[ScalarNurbs<f64>],
    t_start: f64,
    t_end: f64,
    min_intervals: usize,
) -> Vec<f64> {
    let mut points = vec![t_start, t_end];
    for axis in axes {
        for piece in nurbs::bezier::extract_bezier_pieces(axis) {
            for boundary in [piece.u_start, piece.u_end] {
                if boundary > t_start + BREAKPOINT_MERGE_TOL
                    && boundary < t_end - BREAKPOINT_MERGE_TOL
                {
                    points.push(boundary);
                }
            }
        }
    }
    points.sort_by(|a, b| a.partial_cmp(b).unwrap());
    points.dedup_by(|a, b| (*a - *b).abs() <= BREAKPOINT_MERGE_TOL);

    while points.len() - 1 < min_intervals.max(1) {
        let (widest, _) = points
            .windows(2)
            .enumerate()
            .map(|(i, w)| (i, w[1] - w[0]))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        points.insert(widest + 1, 0.5 * (points[widest] + points[widest + 1]));
    }
    points
}

fn speed(derivatives: &[ScalarNurbs<f64>], t: f64) -> f64 {
    derivatives
        .iter()
        .map(|d| {
            let v = nurbs::eval::eval(d, t);
            v * v
        })
        .sum::<f64>()
        .sqrt()
}

fn gl8_speed_integral(derivatives: &[ScalarNurbs<f64>], a: f64, b: f64) -> f64 {
    let half = 0.5 * (b - a);
    let mid = 0.5 * (a + b);
    GL8_NODES
        .iter()
        .zip(GL8_WEIGHTS.iter())
        .map(|(node, weight)| weight * speed(derivatives, mid + half * node))
        .sum::<f64>()
        * half
}

#[cfg(test)]
mod tests;
