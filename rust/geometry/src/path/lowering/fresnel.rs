#![allow(clippy::excessive_precision)] // verbatim Cephes coefficients

use core::f64::consts::{FRAC_PI_2, PI};

const SN: [f64; 6] = [
    -2.99181919401019853726e3,
    7.08840045257738576863e5,
    -6.29741486205862506537e7,
    2.54890880573376359104e9,
    -4.42979518059697779103e10,
    3.18016297876567817986e11,
];
const SD: [f64; 6] = [
    2.81376268889994315696e2,
    4.55847810806532581675e4,
    5.17343888770096400730e6,
    4.19320245898111231129e8,
    2.24411795645340920940e10,
    6.07366389490084639049e11,
];
const CN: [f64; 6] = [
    -4.98843114573573548651e-8,
    9.50428062829859605134e-6,
    -6.45191435683965050962e-4,
    1.88843319396703850064e-2,
    -2.05525900955013891793e-1,
    9.99999999999999998822e-1,
];
const CD: [f64; 7] = [
    3.99982968972495980367e-12,
    9.15439215774657478799e-10,
    1.25001862479598821474e-7,
    1.22262789024179030997e-5,
    8.68029542941784300606e-4,
    4.12142090722199792936e-2,
    1.00000000000000000118e0,
];
const FN: [f64; 10] = [
    4.21543555043677546506e-1,
    1.43407919780758885261e-1,
    1.15220955073585758835e-2,
    3.45017939782574027900e-4,
    4.63613749287867322088e-6,
    3.05568983790257605827e-8,
    1.02304514164907233465e-10,
    1.72010743268161828879e-13,
    1.34283276233062758925e-16,
    3.76329711269987889006e-20,
];
const FD: [f64; 10] = [
    7.51586398353378947175e-1,
    1.16888925859191382142e-1,
    6.44051526508858611005e-3,
    1.55934409164153020873e-4,
    1.84627567348930545870e-6,
    1.12699224763999035261e-8,
    3.60140029589371370404e-11,
    5.88754533621578410010e-14,
    4.52001434074129701496e-17,
    1.25443237090011264384e-20,
];
const GN: [f64; 11] = [
    5.04442073643383265887e-1,
    1.97102833525523411709e-1,
    1.87648584092575249293e-2,
    6.84079380915393090172e-4,
    1.15138826111884280931e-5,
    9.82852443688422223854e-8,
    4.45344415861750144738e-10,
    1.08268041139020870318e-12,
    1.37555460633261799868e-15,
    8.36354435630677421531e-19,
    1.86958710162783235106e-22,
];
const GD: [f64; 11] = [
    1.47495759925128324529e0,
    3.37748989120019970451e-1,
    2.53603741420338795122e-2,
    8.14679107184306179049e-4,
    1.27545075667729118702e-5,
    1.04314589657571990585e-7,
    4.60680728146520428211e-10,
    1.10273215066240270757e-12,
    1.38796531259578871258e-15,
    8.39158816283118707363e-19,
    1.86958710162783236342e-22,
];

/// Cephes switches `C`/`S` from the origin polynomials to the auxiliary pair
/// at this squared argument.
const AUXILIARY_X2: f64 = 2.5625;

fn polevl(x: f64, coef: &[f64]) -> f64 {
    coef.iter().fold(0.0, |acc, &c| acc * x + c)
}

fn p1evl(x: f64, coef: &[f64]) -> f64 {
    coef.iter().fold(1.0, |acc, &c| acc * x + c)
}

/// The Cephes auxiliary pair: for `x² >= AUXILIARY_X2`,
/// `C(x) = ½ + (f·sin a − g·cos a)/(πx)` and
/// `S(x) = ½ − (f·cos a + g·sin a)/(πx)`, with `a = πx²/2`.
fn auxiliary(x2: f64) -> (f64, f64) {
    let pix2 = PI * x2;
    let u = 1.0 / (pix2 * pix2);
    let inv = 1.0 / pix2;
    let f = 1.0 - u * polevl(u, &FN) / p1evl(u, &FD);
    let g = inv * polevl(u, &GN) / p1evl(u, &GD);
    (f, g)
}

fn fresnel_cs(x: f64) -> (f64, f64) {
    let ax = x.abs();
    let x2 = ax * ax;

    let (c, s) = if x2 < AUXILIARY_X2 {
        let t = x2 * x2;
        let s = ax * x2 * polevl(t, &SN) / p1evl(t, &SD);
        let c = ax * polevl(t, &CN) / polevl(t, &CD);
        (c, s)
    } else if ax > 36974.0 {
        (0.5, 0.5)
    } else {
        let (f, g) = auxiliary(x2);
        let arg = FRAC_PI_2 * x2;
        let (sin_a, cos_a) = libm::sincos(arg);
        let pix = PI * ax;
        let c = 0.5 + (f * sin_a - g * cos_a) / pix;
        let s = 0.5 - (f * cos_a + g * sin_a) / pix;
        (c, s)
    };

    if x < 0.0 { (-c, -s) } else { (c, s) }
}

pub(super) fn clothoid_offset(kappa_0: f64, sigma: f64, s: f64) -> (f64, f64) {
    if sigma == 0.0 {
        return constant_curvature_offset(kappa_0, s);
    }
    if sigma < 0.0 {
        let (cx, cy) = rising_curvature_offset(-kappa_0, -sigma, s);
        return (cx, -cy);
    }
    rising_curvature_offset(kappa_0, sigma, s)
}

/// `∫₀ˢ (cos, sin)(κ₀·t) dt`. The sagitta is taken through the half-angle
/// sine, which keeps its relative accuracy as the turn `κ₀·s` vanishes —
/// `1 − cos` does not.
fn constant_curvature_offset(kappa_0: f64, s: f64) -> (f64, f64) {
    if kappa_0 == 0.0 {
        return (s, 0.0);
    }
    let half_turn_sin = libm::sin(0.5 * kappa_0 * s);
    (
        libm::sin(kappa_0 * s) / kappa_0,
        2.0 * half_turn_sin * half_turn_sin / kappa_0,
    )
}

/// `∫₀ˢ (cos, sin)(κ₀·t + σ·t²/2) dt` for `σ > 0`.
///
/// Completing the square moves the integral onto the Cornu spiral centred at
/// arc `−κ₀/σ`, where it is a difference of Fresnel values. That difference
/// is only well conditioned while the segment spans the spiral centre or
/// stays near it: further out, `s + κ₀/σ` loses `s` to the offset and each
/// Fresnel value is a `½` pedestal plus a vanishing tail. The endpoint
/// curvatures carry the same geometry with neither cancellation, so the
/// segment is evaluated from its two tails there.
fn rising_curvature_offset(kappa_0: f64, sigma: f64, s: f64) -> (f64, f64) {
    let kappa_1 = kappa_0 + sigma * s;
    let inv_spiral_scale = 1.0 / (PI * sigma).sqrt();
    let x0 = kappa_0 * inv_spiral_scale;
    let x1 = kappa_1 * inv_spiral_scale;
    if x0 * x1 > 0.0 && x0 * x0 >= AUXILIARY_X2 && x1 * x1 >= AUXILIARY_X2 {
        let turn = kappa_0 * s + 0.5 * sigma * s * s;
        return tail_offset(kappa_0, kappa_1, x0 * x0, x1 * x1, turn);
    }

    let spiral_scale = (sigma / PI).sqrt();
    let centre_arc = kappa_0 / sigma;
    let (c0, s0) = fresnel_cs(centre_arc * spiral_scale);
    let (c1, s1) = fresnel_cs((s + centre_arc) * spiral_scale);
    let d_c = c1 - c0;
    let d_s = s1 - s0;

    let k = (PI / sigma).sqrt();
    let (sin_a, cos_a) = libm::sincos(kappa_0 * kappa_0 / (2.0 * sigma));
    (
        k * (cos_a * d_c + sin_a * d_s),
        k * (cos_a * d_s - sin_a * d_c),
    )
}

/// `∫₀ˢ e^{i(κ₀t + σt²/2)} dt = (g₀ + i·f₀)/κ₀ − (g₁ + i·f₁)·e^{i·turn}/κ₁`
/// for a segment that stays on one arm of the spiral: the spiral-centre phase
/// `κ₀²/(2σ)` — unbounded as `σ → 0`, and meaningless modulo 2π once it is —
/// cancels analytically between the two endpoint tails, leaving only the
/// segment's own turn.
fn tail_offset(kappa_0: f64, kappa_1: f64, x0_sq: f64, x1_sq: f64, turn: f64) -> (f64, f64) {
    let (f0, g0) = auxiliary(x0_sq);
    let (f1, g1) = auxiliary(x1_sq);
    let (sin_turn, cos_turn) = libm::sincos(turn);
    (
        g0 / kappa_0 - (g1 * cos_turn - f1 * sin_turn) / kappa_1,
        f0 / kappa_0 - (f1 * cos_turn + g1 * sin_turn) / kappa_1,
    )
}
