use super::{Ball, TaylorModel, Window, derivative_of};

const SAMPLES: u32 = 400;

fn eval(model: &TaylorModel, x: f64) -> f64 {
    model
        .coefficients()
        .iter()
        .rev()
        .fold(0.0, |acc, &c| acc.mul_add(x, c))
}

/// Asserts `truth` lies within the model's bound of its polynomial at
/// dense points, up to the polynomial's own float evaluation.
fn assert_encloses(model: &TaylorModel, truth: impl Fn(f64) -> f64, what: &str) {
    let magnitude: f64 = model.coefficients().iter().map(|c| c.abs()).sum();
    for k in 0..=SAMPLES {
        let x = -1.0 + 2.0 * f64::from(k) / f64::from(SAMPLES);
        let miss = (truth(x) - eval(model, x)).abs();
        let evaluation = 64.0 * f64::EPSILON * (magnitude + truth(x).abs());
        assert!(
            miss <= model.rad() + evaluation,
            "{what}: at x={x} the model misses by {miss:e}, past its bound {:e}",
            model.rad()
        );
    }
}

fn argument(centre: f64, half_width: f64) -> TaylorModel {
    TaylorModel::affine(Ball::exact(centre), half_width)
}

#[test]
fn a_tanh_model_encloses_tanh() {
    for (centre, half_width) in [(0.0, 0.4), (1.3, 0.3), (-2.5, 0.7), (8.0, 0.1), (0.6, 0.02)] {
        let model = argument(centre, half_width).tanh();
        assert_encloses(
            &model,
            |x| libm::tanh(x.mul_add(half_width, centre)),
            "tanh",
        );
        if half_width <= 0.05 {
            assert!(model.rad() < 1e-8, "tanh bound {:e}", model.rad());
        }
    }
}

#[test]
fn a_reciprocal_model_encloses_the_reciprocal_off_zero() {
    for (centre, half_width) in [(1.0, 0.5), (-3.0, 2.0), (1e-3, 2e-4)] {
        let model = argument(centre, half_width)
            .reciprocal()
            .expect("the range keeps off zero");
        assert_encloses(&model, |x| 1.0 / x.mul_add(half_width, centre), "1/x");
    }
    assert!(argument(1.0, 1.0).reciprocal().is_none());
}

#[test]
fn a_product_past_the_kept_degree_bounds_what_it_drops() {
    let unit = argument(0.0, 1.0);
    let falling_coefficients = [1.0, -0.5, 0.25, -0.125, 0.0625, 0.5, 0.75];
    let rising = derivative_of(&[1.0; 12], 0, &unit);
    let falling = derivative_of(&falling_coefficients, 0, &unit);
    let product = rising.mul(&falling);
    let value = |coefficients: &[f64], x: f64| {
        coefficients
            .iter()
            .rev()
            .fold(0.0_f64, |acc, &c| acc.mul_add(x, c))
    };
    assert_encloses(
        &product,
        |x| value(&[1.0; 12], x) * value(&falling_coefficients, x),
        "product",
    );
    assert!(product.rad() > 0.0, "the dropped terms join the bound");
}

#[test]
fn a_polynomial_in_shifted_time_and_its_derivative_are_enclosed() {
    let coefficients = [0.3, -2.0, 40.0, -900.0, 1.5e4];
    let window = Window::new(10.002, 10.006);
    let origin = 10.0;
    let tau = window.time_since(origin);
    let at = |x: f64| x.mul_add(window.radius, window.center - origin);
    let value = |tau: f64| coefficients.iter().rev().fold(0.0, |acc, &c| acc * tau + c);
    let slope = |tau: f64| {
        coefficients
            .iter()
            .enumerate()
            .skip(1)
            .rev()
            .fold(0.0, |acc, (power, &c)| acc * tau + power as f64 * c)
    };
    assert_encloses(
        &derivative_of(&coefficients, 0, &tau),
        |x| value(at(x)),
        "value",
    );
    assert_encloses(
        &derivative_of(&coefficients, 1, &tau),
        |x| slope(at(x)),
        "slope",
    );
}

/// `TaylorModel::mul` before its product was unrolled over fixed bounds,
/// kept as the bit-for-bit reference the unrolled product must match.
fn reference_mul(a: &TaylorModel, b: &TaylorModel) -> TaylorModel {
    if a.len == 1 || b.len == 1 {
        return a.mul(b);
    }
    let mut coefficients = [0.0; super::TAYLOR_LEN];
    let mut truncated = 0.0;
    let (left, right) = (a.coefficients(), b.coefficients());
    for (i, &x) in left.iter().enumerate() {
        let kept = right.len().min(super::TAYLOR_LEN - i);
        for (slot, &y) in coefficients[i..i + kept].iter_mut().zip(&right[..kept]) {
            *slot += x * y;
        }
        for &y in &right[kept..] {
            truncated += (x * y).abs();
        }
    }
    let len = (a.len + b.len).saturating_sub(1).min(super::TAYLOR_LEN);
    let (a_magnitude, b_magnitude) = (a.magnitude(), b.magnitude());
    let mut out = TaylorModel {
        coefficients,
        len,
        rad: super::upper(
            a_magnitude * b.rad
                + a.rad * b_magnitude
                + a.rad * b.rad
                + truncated
                + super::PRODUCT_ROUNDING * a_magnitude * b_magnitude,
        ),
        working_share: a.working_share.max(b.working_share),
    };
    if out.len > 1 {
        let deviation: f64 = out.coefficients[1..out.len].iter().map(|c| c.abs()).sum();
        let negligible = super::TRIM_SHARE * (out.coefficients[0].abs() + deviation);
        let working = out.working_share * deviation;
        let mut dropped = 0.0;
        while out.len > 1 {
            let last = out.coefficients[out.len - 1].abs();
            if last > negligible && dropped + last > working {
                break;
            }
            out.len -= 1;
            dropped += last;
            out.coefficients[out.len] = 0.0;
        }
        out.rad = super::upper(out.rad + dropped);
    }
    out
}

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0 >> 11
    }

    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }

    fn coefficient(&mut self) -> f64 {
        match self.below(12) {
            0 => 0.0,
            1 => -0.0,
            2 => f64::INFINITY,
            3 => f64::NAN,
            4 => 1e-300 * (self.below(1000) as f64 - 500.0),
            _ => {
                let mantissa = self.below(1 << 30) as f64 / (1u64 << 29) as f64 - 1.0;
                mantissa * 10f64.powi(self.below(24) as i32 - 12)
            }
        }
    }

    fn model(&mut self) -> TaylorModel {
        let len = 1 + self.below(super::TAYLOR_LEN as u64) as usize;
        let coefficients: Vec<f64> = (0..len)
            .map(|_| {
                let c = self.coefficient();
                if self.below(4) == 0 && c.is_finite() {
                    c
                } else {
                    c.clamp(-1e6, 1e6)
                }
            })
            .collect();
        let rad = match self.below(4) {
            0 => 0.0,
            1 => f64::INFINITY,
            _ => self.below(1 << 20) as f64 * 1e-12,
        };
        let model = TaylorModel::from_polynomial(&coefficients, rad);
        match self.below(3) {
            0 => model,
            1 => model.working_to(1e-9),
            _ => model.working_to(1e-3),
        }
    }
}

fn bits(model: &TaylorModel) -> ([u64; super::TAYLOR_LEN], usize, u64, u64) {
    (
        model.coefficients.map(f64::to_bits),
        model.len,
        model.rad.to_bits(),
        model.working_share.to_bits(),
    )
}

#[test]
fn a_product_is_bit_for_bit_the_reference_product() {
    let mut rng = Lcg(0x5eed_1234_abcd_0001);
    for _ in 0..200_000 {
        let (a, b) = (rng.model(), rng.model());
        assert_eq!(
            bits(&a.mul(&b)),
            bits(&reference_mul(&a, &b)),
            "{a:?} x {b:?}"
        );
    }
}
