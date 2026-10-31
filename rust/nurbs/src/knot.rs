use crate::{ConstructError, ScalarNurbs};

#[derive(Debug, Clone, PartialEq)]
pub struct KnotVector {
    knots: Vec<f64>,
}

impl KnotVector {
    pub fn try_new(knots: Vec<f64>) -> Result<Self, ConstructError> {
        if knots.len() < 2 {
            return Err(ConstructError::KnotCountMismatch {
                expected: 2,
                got: knots.len(),
            });
        }
        for window in knots.windows(2) {
            if window[1] < window[0] {
                return Err(ConstructError::KnotsNotMonotone);
            }
        }
        Ok(Self { knots })
    }

    pub fn as_slice(&self) -> &[f64] {
        &self.knots
    }
}

pub fn find_knot_span(knots: &[f64], p: usize, n: usize, u: f64) -> usize {
    debug_assert!(knots.len() == n + p + 1);
    if u >= knots[n] {
        return n - 1;
    }
    if u <= knots[p] {
        return p;
    }
    let mut low = p;
    let mut high = n;
    let mut mid = usize::midpoint(low, high);
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = usize::midpoint(low, high);
    }
    mid
}

pub fn refined_to_full_multiplicity(curve: &ScalarNurbs) -> ScalarNurbs {
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();

    let refinement = build_refinement_vector(knots, p);
    if refinement.is_empty() {
        return curve.clone();
    }

    let (new_knots, new_cps) = refine_knot_vect_curve(knots, cps, p, &refinement);

    ScalarNurbs::try_new(curve.degree(), new_knots, new_cps)
        .expect("refined_to_full_multiplicity: result invariants should hold")
}

fn build_refinement_vector(knots: &[f64], p: usize) -> Vec<f64> {
    let interior_start = p + 1;
    let interior_end = knots.len() - p - 1;
    if interior_end <= interior_start {
        return Vec::new();
    }

    let mut x: Vec<f64> = Vec::new();
    let mut i = interior_start;
    while i < interior_end {
        let u = knots[i];
        let mut s = 0usize;
        while i + s < interior_end && knots[i + s] == u {
            s += 1;
        }
        assert!(
            s <= p + 1,
            "interior knot {u} has multiplicity {s} above degree + 1"
        );
        let deficit = p.saturating_sub(s);
        for _ in 0..deficit {
            x.push(u);
        }
        i += s;
    }
    x
}

fn refine_knot_vect_curve(knots: &[f64], cps: &[f64], p: usize, x: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n_pt = cps.len() - 1;
    let n_span = cps.len();
    let m = knots.len() - 1;
    let r = x.len() - 1;

    let a = find_knot_span(knots, p, n_span, x[0]);
    let b = find_knot_span(knots, p, n_span, x[r]) + 1;

    let new_cp_count = cps.len() + x.len();
    let new_knot_count = knots.len() + x.len();

    let mut new_cps = vec![0.0; new_cp_count];
    let mut new_knots = vec![0.0; new_knot_count];

    for j in 0..=(a - p) {
        new_cps[j] = cps[j];
    }
    for j in (b - 1)..=n_pt {
        new_cps[j + r + 1] = cps[j];
    }
    for j in 0..=a {
        new_knots[j] = knots[j];
    }
    for j in (b + p)..=m {
        new_knots[j + r + 1] = knots[j];
    }

    let mut i = b + p - 1;
    let mut k = b + p + r;

    for xi in (0..=r).rev() {
        while x[xi] <= knots[i] && i > a {
            new_cps[k - p - 1] = cps[i - p - 1];
            new_knots[k] = knots[i];
            k -= 1;
            i -= 1;
        }

        new_cps[k - p - 1] = new_cps[k - p];

        for l in 1..=p {
            let ind = k - p + l;
            let alpha_num = new_knots[k + l] - x[xi];
            if alpha_num == 0.0 {
                new_cps[ind - 1] = new_cps[ind];
            } else {
                let denom = new_knots[k + l] - knots[i - p + l];
                let alpha = alpha_num / denom;
                new_cps[ind - 1] = alpha * new_cps[ind - 1] + (1.0 - alpha) * new_cps[ind];
            }
        }

        new_knots[k] = x[xi];
        k -= 1;
    }

    (new_knots, new_cps)
}

#[cfg(test)]
mod tests;
