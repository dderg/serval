use crate::{AlgebraError, ScalarNurbs};

#[derive(Debug, Clone, PartialEq)]
pub struct BezierPiece {
    pub u_start: f64,
    pub u_end: f64,
    pub coeffs: Vec<f64>,
}

impl BezierPiece {
    pub fn degree(&self) -> usize {
        self.coeffs.len().saturating_sub(1)
    }

    pub fn evaluate(&self, u: f64) -> f64 {
        let dx = u - self.u_start;
        let mut acc = 0.0;
        for c in self.coeffs.iter().rev() {
            acc = acc * dx + *c;
        }
        acc
    }

    pub fn differentiate(&self) -> Self {
        if self.coeffs.len() <= 1 {
            return Self {
                u_start: self.u_start,
                u_end: self.u_end,
                coeffs: vec![0.0],
            };
        }
        let coeffs = (1..self.coeffs.len())
            .map(|k| self.coeffs[k] * (k as f64))
            .collect();
        Self {
            u_start: self.u_start,
            u_end: self.u_end,
            coeffs,
        }
    }

    pub fn to_bernstein(&self) -> Vec<f64> {
        let d = self.degree();
        let h = self.u_end - self.u_start;
        let mut h_pow = 1.0;
        let normalized: Vec<f64> = self
            .coeffs
            .iter()
            .map(|c| {
                let v = *c * h_pow;
                h_pow *= h;
                v
            })
            .collect();

        let mut bernstein = vec![0.0; d + 1];
        for k in 0..=d {
            let mut acc = 0.0;
            for i in 0..=k {
                let num = binomial(k, i) as f64;
                let den = binomial(d, i) as f64;
                acc += (num / den) * normalized[i];
            }
            bernstein[k] = acc;
        }
        bernstein
    }

    pub fn from_bernstein(bernstein: &[f64], u_start: f64, u_end: f64) -> Self {
        let d = bernstein.len() - 1;
        let h = u_end - u_start;

        let mut h_pow = 1.0;
        let mut coeffs = vec![0.0; d + 1];
        for k in 0..=d {
            let mut acc = 0.0;
            for i in 0..=k {
                let sign = if (k - i) % 2 == 0 { 1.0 } else { -1.0 };
                let c_d_k = binomial(d, k) as f64;
                let c_k_i = binomial(k, i) as f64;
                acc += sign * c_d_k * c_k_i * bernstein[i];
            }
            coeffs[k] = acc / h_pow;
            h_pow *= h;
        }
        Self {
            u_start,
            u_end,
            coeffs,
        }
    }
}

impl std::ops::Add<&BezierPiece> for &BezierPiece {
    type Output = Result<BezierPiece, AlgebraError>;
    fn add(self, rhs: &BezierPiece) -> Self::Output {
        if self.u_start != rhs.u_start || self.u_end != rhs.u_end {
            return Err(AlgebraError::SupportMismatch);
        }
        let max_len = self.coeffs.len().max(rhs.coeffs.len());
        let mut coeffs = vec![0.0; max_len];
        for (i, c) in self.coeffs.iter().enumerate() {
            coeffs[i] += *c;
        }
        for (i, c) in rhs.coeffs.iter().enumerate() {
            coeffs[i] += *c;
        }
        Ok(BezierPiece {
            u_start: self.u_start,
            u_end: self.u_end,
            coeffs,
        })
    }
}

pub fn extract_bezier_pieces(curve: &ScalarNurbs) -> Vec<BezierPiece> {
    let refined = crate::knot::refined_to_full_multiplicity(curve);
    let p = refined.degree() as usize;
    let knots = refined.knots();
    let cps = refined.control_points();

    let mut breakpoints: Vec<f64> = Vec::new();
    let mut last: Option<f64> = None;
    for k in knots {
        if last.is_none_or(|l| *k != l) {
            breakpoints.push(*k);
            last = Some(*k);
        }
    }

    let mut pieces = Vec::with_capacity(breakpoints.len() - 1);
    let mut cp_idx = 0;
    for window in breakpoints.windows(2) {
        let u_start = window[0];
        let u_end = window[1];
        let bernstein: Vec<f64> = cps[cp_idx..=(cp_idx + p)].to_vec();
        pieces.push(BezierPiece::from_bernstein(&bernstein, u_start, u_end));
        cp_idx += p;
    }

    pieces
}

pub fn bezier_pieces_to_nurbs(pieces: &[BezierPiece]) -> ScalarNurbs {
    assert!(!pieces.is_empty(), "bezier_pieces_to_nurbs: empty input");
    let p = pieces[0].degree();
    for w in pieces.windows(2) {
        assert!(w[0].u_end == w[1].u_start, "non-contiguous Bezier pieces");
        assert!(w[1].degree() == p, "inconsistent degrees");
    }

    let mut knots = Vec::with_capacity((pieces.len() + 1) * p + 2);
    for _ in 0..=p {
        knots.push(pieces[0].u_start);
    }
    for piece in &pieces[..pieces.len() - 1] {
        for _ in 0..p {
            knots.push(piece.u_end);
        }
    }
    for _ in 0..=p {
        knots.push(pieces[pieces.len() - 1].u_end);
    }

    let mut cps: Vec<f64> = Vec::with_capacity(pieces.len() * p + 1);
    for (i, piece) in pieces.iter().enumerate() {
        let bernstein = piece.to_bernstein();
        if i == 0 {
            cps.extend_from_slice(&bernstein);
        } else {
            cps.extend_from_slice(&bernstein[1..]);
        }
    }

    ScalarNurbs::try_new(p as u8, knots, cps)
        .expect("bezier_pieces_to_nurbs: invariants should hold")
}

pub fn split_piece_at(piece: &BezierPiece, u_split: f64) -> (BezierPiece, BezierPiece) {
    assert!(
        u_split > piece.u_start && u_split < piece.u_end,
        "u_split must be strictly interior"
    );
    let d = piece.degree();

    let left = BezierPiece {
        u_start: piece.u_start,
        u_end: u_split,
        coeffs: piece.coeffs.clone(),
    };

    let delta = u_split - piece.u_start;
    let mut right_coeffs = vec![0.0; d + 1];
    let mut delta_pow = vec![1.0; d + 1];
    for k in 1..=d {
        delta_pow[k] = delta_pow[k - 1] * delta;
    }

    for i in 0..=d {
        let mut acc = 0.0;
        for k in i..=d {
            let c_k_i = binomial(k, i) as f64;
            acc += piece.coeffs[k] * c_k_i * delta_pow[k - i];
        }
        right_coeffs[i] = acc;
    }

    let right = BezierPiece {
        u_start: u_split,
        u_end: piece.u_end,
        coeffs: right_coeffs,
    };

    (left, right)
}

pub(crate) fn binomial(n: usize, k: usize) -> u64 {
    debug_assert!(
        n <= 50,
        "binomial intermediate products overflow u64 above n = 50"
    );
    if k > n {
        return 0;
    }
    let k = k.min(n - k);
    let mut result: u64 = 1;
    for i in 0..k {
        result = result * (n - i) as u64 / (i + 1) as u64;
    }
    result
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests;
