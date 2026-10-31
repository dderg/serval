//! Property campaign over the arc-length path alphabet: line, arc, clothoid.
//!
//! Every property compares the crate against an oracle built here from the
//! segment's *declared* invariants — the curvature law `κ(s)`, the tangent
//! `∫κ`, and the position `∫(cos, sin)(∫κ)` — never against another crate
//! routine on the same code path.

use geometry::path::lowering::PositionProfile;
use geometry::path::{Arc, Clothoid, CurvatureProfile, Line, Segment};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// Anchor coordinates stay inside a print volume; a segment far outside one is
/// not geometry this pipeline ever sees, and its sample rounding would swamp
/// every finite-difference budget below.
const ANCHOR_SPAN_MM: f64 = 200.0;
/// Turn budget over one segment. The alphabet's members are blends and G2/G3
/// arcs, not spirals; bounding the total turn also bounds the oracle's panel
/// count.
const MAX_TURN_RAD: f64 = 20.0;
const MIN_LENGTH_MM: f64 = 1e-3;
const MAX_LENGTH_MM: f64 = 100.0;
/// Smallest turn treated as curved. Below it the segment is a straight line
/// to float precision, and `κ₀` is no longer resolvable against `σ·s`.
const MIN_TURN_RAD: f64 = 1e-4;
/// The chord sum's roundings do not cancel — each sample coordinate carries
/// several ulps of its own magnitude and Richardson re-weights the two sums
/// by `(4 + 1)/3`. Measured headroom over the modelled budget at 8k cases: 5x.
const POLYGON_BUDGET_SLACK: f64 = 32.0;

const GAUSS_NODES: [(f64, f64); 8] = [
    (-0.9602898564975362, 0.10122853629037652),
    (-0.7966664774136267, 0.22238103445337445),
    (-0.525532409916329, 0.31370664587788716),
    (-0.18343464249564984, 0.3626837833783618),
    (0.18343464249564984, 0.3626837833783618),
    (0.525532409916329, 0.31370664587788716),
    (0.7966664774136267, 0.22238103445337445),
    (0.9602898564975362, 0.10122853629037652),
];

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(k: f64, a: [f64; 3]) -> [f64; 3] {
    [k * a[0], k * a[1], k * a[2]]
}

fn unit(a: [f64; 3]) -> [f64; 3] {
    scale(1.0 / norm(a), a)
}

fn combine(a: f64, u: [f64; 3], b: f64, v: [f64; 3]) -> [f64; 3] {
    [
        a * u[0] + b * v[0],
        a * u[1] + b * v[1],
        a * u[2] + b * v[2],
    ]
}

/// An orthonormal in-plane pair, exact to a rounding of the normalisation.
#[derive(Debug, Clone, Copy)]
struct Frame {
    u: [f64; 3],
    v: [f64; 3],
}

fn arb_frame() -> impl Strategy<Value = Frame> {
    (
        -1.0f64..1.0,
        0.0f64..std::f64::consts::TAU,
        0.0f64..std::f64::consts::TAU,
    )
        .prop_map(|(z, azimuth, roll)| {
            let radial = (1.0 - z * z).max(0.0).sqrt();
            let u = unit([radial * libm::cos(azimuth), radial * libm::sin(azimuth), z]);
            let seed = if u[0].abs() < 0.8 {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 1.0, 0.0]
            };
            let e0 = unit(cross(u, seed));
            let e1 = cross(u, e0);
            let v = unit(combine(libm::cos(roll), e0, libm::sin(roll), e1));
            Frame { u, v }
        })
}

fn arb_anchor() -> impl Strategy<Value = [f64; 3]> {
    prop::array::uniform3(-ANCHOR_SPAN_MM..ANCHOR_SPAN_MM)
}

fn log_uniform(min: f64, max: f64) -> impl Strategy<Value = f64> {
    (libm::log(min)..libm::log(max)).prop_map(libm::exp)
}

fn arb_length() -> impl Strategy<Value = f64> {
    log_uniform(MIN_LENGTH_MM, MAX_LENGTH_MM)
}

/// A turn contribution, signed and log-spread, with an exact-zero arm so the
/// degenerate limits (straight line, constant curvature) are sampled densely.
fn arb_turn(min: f64) -> impl Strategy<Value = f64> {
    prop_oneof![
        1 => Just(0.0),
        9 => (log_uniform(min, MAX_TURN_RAD), prop::bool::ANY)
            .prop_map(|(turn, negative)| if negative { -turn } else { turn }),
    ]
}

fn arb_line() -> impl Strategy<Value = Line> {
    (arb_anchor(), arb_frame(), arb_length()).prop_map(|(start, frame, length)| {
        Line::try_new(start, combine(1.0, start, length, frame.u)).expect("a non-degenerate line")
    })
}

fn arb_arc() -> impl Strategy<Value = Arc> {
    (
        arb_anchor(),
        arb_frame(),
        arb_length(),
        arb_turn(MIN_TURN_RAD),
        0.0f64..std::f64::consts::TAU,
    )
        .prop_filter_map(
            "an arc needs a nonzero sweep",
            |(origin, frame, length, turn, start_angle)| {
                let sweep = if turn == 0.0 { MIN_TURN_RAD } else { turn };
                let radius = length / sweep.abs();
                Arc::try_new(origin, frame.u, frame.v, radius, start_angle, sweep).ok()
            },
        )
}

fn arb_clothoid() -> impl Strategy<Value = Clothoid> {
    (
        arb_anchor(),
        arb_frame(),
        arb_length(),
        arb_turn(MIN_TURN_RAD),
        // The `σ` turn reaches far below the `κ₀` floor: that is the near-arc
        // corner where completing the square loses the segment inside the
        // spiral offset.
        arb_turn(1e-13),
    )
        .prop_filter_map(
            "the total turn stays inside the budget",
            |(start_pose, frame, length, base_turn, rate_turn)| {
                if base_turn.abs() + rate_turn.abs() > MAX_TURN_RAD {
                    return None;
                }
                let kappa_0 = base_turn / length;
                let sigma = 2.0 * rate_turn / (length * length);
                Clothoid::try_new(start_pose, frame.u, frame.v, kappa_0, sigma, length).ok()
            },
        )
}

fn arb_segment() -> impl Strategy<Value = Segment> {
    prop_oneof![
        1 => arb_line().prop_map(Segment::Line),
        2 => arb_arc().prop_map(Segment::Arc),
        3 => arb_clothoid().prop_map(Segment::Clothoid),
    ]
}

/// The in-plane frame each kind rotates its heading in, and the heading angle
/// the segment starts at inside it.
fn heading_frame(segment: &Segment) -> (Frame, f64) {
    match segment {
        Segment::Line(line) => {
            let u = unit(sub(line.end, line.start));
            let seed = if u[0].abs() < 0.8 {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 1.0, 0.0]
            };
            let v = unit(cross(u, seed));
            (Frame { u, v }, 0.0)
        }
        Segment::Arc(arc) => (
            Frame { u: arc.u, v: arc.v },
            arc.start_angle + arc.sweep.signum() * std::f64::consts::FRAC_PI_2,
        ),
        Segment::Clothoid(clothoid) => (
            Frame {
                u: clothoid.u,
                v: clothoid.v,
            },
            0.0,
        ),
    }
}

/// `∫₀ˢ κ(t) dt` by composite Simpson. `κ` is affine on every alphabet member,
/// so Simpson is exact and the only error is the summation's own rounding.
fn integrated_curvature(segment: &Segment, s: f64) -> f64 {
    const PANELS: usize = 256;
    let h = s / PANELS as f64;
    let mut total = 0.0;
    for panel in 0..PANELS {
        let a = h * panel as f64;
        let b = a + h;
        total +=
            (h / 6.0) * (segment.kappa(a) + 4.0 * segment.kappa(0.5 * (a + b)) + segment.kappa(b));
    }
    total
}

fn quadrature_panels(total_turn: f64) -> usize {
    32 + (4.0 * total_turn.abs()) as usize
}

/// `∫₀ˢ f(t) dt` by composite 8-point Gauss–Legendre.
fn gauss_integral<F: Fn(f64) -> [f64; 3]>(s: f64, panels: usize, f: F) -> [f64; 3] {
    let h = s / panels as f64;
    let mut total = [0.0; 3];
    for panel in 0..panels {
        let mid = h * (panel as f64 + 0.5);
        for (node, weight) in GAUSS_NODES {
            let t = mid + 0.5 * h * node;
            let w = 0.5 * h * weight;
            let value = f(t);
            total[0] += w * value[0];
            total[1] += w * value[1];
            total[2] += w * value[2];
        }
    }
    total
}

/// The clothoid's own displacement law, straight off `(κ₀, σ)`:
/// `(∫cos φ, ∫sin φ)` with `φ(t) = κ₀·t + σ·t²/2`. No Fresnel identity, no
/// completed square — just the integral the segment is defined to be.
fn clothoid_offset_quadrature(clothoid: &Clothoid, s: f64) -> (f64, f64) {
    let panels = quadrature_panels(clothoid.kappa_0 * s + 0.5 * clothoid.sigma * s * s);
    let offset = gauss_integral(s, panels, |t| {
        let phi = clothoid.kappa_0 * t + 0.5 * clothoid.sigma * t * t;
        let (sin_phi, cos_phi) = libm::sincos(phi);
        [cos_phi, sin_phi, 0.0]
    });
    (offset[0], offset[1])
}

fn sample_arcs(length: f64, fractions: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0, length];
    out.extend(fractions.iter().map(|f| f * length));
    out
}

/// Differencing absolute coordinates cannot resolve finer than a rounding of
/// the larger magnitude, so every position-space tolerance carries that floor.
fn coordinate_noise(scale_mm: f64) -> f64 {
    8.0 * f64::EPSILON * scale_mm
}

fn max_coordinate(points: &[[f64; 3]]) -> f64 {
    points.iter().fold(0.0_f64, |acc, p| {
        acc.max(p[0].abs()).max(p[1].abs()).max(p[2].abs())
    })
}

/// Polygonal length on `2·n` chords with one Richardson step against `n`: the
/// `(κ·h)²/24` chord deficit cancels, leaving `(κ·h)⁴/1920` truncation.
fn richardson_polygon_length(segment: &Segment, length: f64, n: usize) -> (f64, f64) {
    let fine: Vec<[f64; 3]> = (0..=2 * n)
        .map(|i| segment.point_at(length * i as f64 / (2 * n) as f64))
        .collect();
    let anchor_scale = max_coordinate(&fine);
    let fine_length: f64 = fine.windows(2).map(|w| norm(sub(w[1], w[0]))).sum();
    let coarse_length: f64 = fine
        .chunks_exact(2)
        .zip(fine.iter().skip(2).step_by(2))
        .map(|(lo, hi)| norm(sub(*hi, lo[0])))
        .sum();
    ((4.0 * fine_length - coarse_length) / 3.0, anchor_scale)
}

fn relative_failure(what: &str, got: f64, want: f64, tol: f64) -> Result<(), TestCaseError> {
    let error = (got - want).abs();
    if error <= tol {
        return Ok(());
    }
    Err(TestCaseError::fail(format!(
        "{what}: got {got:e}, want {want:e}, error {error:e} > tol {tol:e}"
    )))
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 384,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/path_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// The tangent rotates in the segment plane at exactly the declared
    /// curvature: `ĥ(s)` is the start heading turned by `∫₀ˢ κ`.
    #[test]
    fn heading_turns_at_the_integrated_curvature(
        segment in arb_segment(),
        fractions in prop::collection::vec(0.0f64..=1.0, 6),
    ) {
        let length = segment.s_len();
        let (frame, heading_0) = heading_frame(&segment);
        for s in sample_arcs(length, &fractions) {
            let turn = integrated_curvature(&segment, s);
            let (sin_psi, cos_psi) = libm::sincos(heading_0 + turn);
            let want = combine(cos_psi, frame.u, sin_psi, frame.v);
            let got = segment.heading_at(s);
            let tol = 1e-12 * (1.0 + turn.abs());
            relative_failure("heading x", got[0], want[0], tol)?;
            relative_failure("heading y", got[1], want[1], tol)?;
            relative_failure("heading z", got[2], want[2], tol)?;
            relative_failure("heading is a unit vector", norm(got), 1.0, 1e-14)?;
        }
    }

    /// The position parametrisation is the running integral of the heading —
    /// which is what makes `s` an arc length at all.
    #[test]
    fn position_is_the_running_integral_of_the_heading(
        segment in arb_segment(),
        fractions in prop::collection::vec(0.0f64..=1.0, 4),
    ) {
        let length = segment.s_len();
        let (_, kappa_peak) = segment.kappa_peak();
        let panels = quadrature_panels(kappa_peak * length);
        let start = segment.point_at(0.0);
        for s in sample_arcs(length, &fractions) {
            let end = segment.point_at(s);
            let want = gauss_integral(s, panels, |t| segment.heading_at(t));
            let got = sub(end, start);
            let tol = 1e-9 * length + coordinate_noise(max_coordinate(&[start, end]));
            relative_failure("displacement x", got[0], want[0], tol)?;
            relative_failure("displacement y", got[1], want[1], tol)?;
            relative_failure("displacement z", got[2], want[2], tol)?;
        }
    }

    /// The clothoid's Fresnel evaluation reproduces `(∫cos φ, ∫sin φ)` over
    /// the whole `σ` range the type accepts — including the near-arc corner
    /// where the completed square puts the segment `κ₀/σ` away from the
    /// spiral centre.
    #[test]
    fn clothoid_position_matches_fresnel_quadrature(
        clothoid in arb_clothoid(),
        fractions in prop::collection::vec(0.0f64..=1.0, 4),
    ) {
        let length = clothoid.length;
        let segment = Segment::Clothoid(clothoid.clone());
        let noise = coordinate_noise(max_coordinate(&[clothoid.start_pose]) + length);
        for s in sample_arcs(length, &fractions) {
            let (want_u, want_v) = clothoid_offset_quadrature(&clothoid, s);
            let offset = sub(segment.point_at(s), clothoid.start_pose);
            let tol = 1e-9 * length + noise;
            relative_failure("clothoid u offset", dot(offset, clothoid.u), want_u, tol)?;
            relative_failure("clothoid v offset", dot(offset, clothoid.v), want_v, tol)?;
            relative_failure(
                "clothoid stays in its plane",
                dot(offset, cross(clothoid.u, clothoid.v)),
                0.0,
                1e-12 * length + noise,
            )?;
        }
    }

    /// Every point of an arc is `radius` from the origin, and the arc covers
    /// exactly `sweep` over `radius·|sweep|` of arc length.
    #[test]
    fn arc_stays_on_its_circle_and_sweeps_its_angle(
        arc in arb_arc(),
        fractions in prop::collection::vec(0.0f64..=1.0, 6),
    ) {
        let segment = Segment::Arc(arc.clone());
        let length = segment.s_len();
        let noise = coordinate_noise(max_coordinate(&[arc.origin]) + arc.radius);
        for s in sample_arcs(length, &fractions) {
            let radial = sub(segment.point_at(s), arc.origin);
            relative_failure(
                "point lies on the circle",
                norm(radial),
                arc.radius,
                1e-13 * arc.radius + noise,
            )?;
        }
        let (sin_end, cos_end) = libm::sincos(arc.start_angle + arc.sweep);
        let want_end = combine(1.0, arc.origin, arc.radius, combine(cos_end, arc.u, sin_end, arc.v));
        let got_end = segment.point_at(length);
        let tol = 1e-13 * arc.radius * (1.0 + arc.sweep.abs()) + noise;
        relative_failure("arc end x", got_end[0], want_end[0], tol)?;
        relative_failure("arc end y", got_end[1], want_end[1], tol)?;
        relative_failure("arc end z", got_end[2], want_end[2], tol)?;
        relative_failure(
            "length is radius times sweep",
            length * segment.kappa(0.0),
            arc.sweep,
            1e-14 * arc.sweep.abs(),
        )?;
    }

    /// A line's point is affine in the arc length and lands on its endpoint.
    #[test]
    fn line_is_affine_in_arc_length(
        line in arb_line(),
        fractions in prop::collection::vec(0.0f64..=1.0, 6),
    ) {
        let segment = Segment::Line(line.clone());
        let length = segment.s_len();
        let direction = unit(sub(line.end, line.start));
        let scale_mm = length + norm(line.start);
        for s in sample_arcs(length, &fractions) {
            let want = combine(1.0, line.start, s, direction);
            let got = segment.point_at(s);
            let tol = 1e-14 * scale_mm;
            relative_failure("line x", got[0], want[0], tol)?;
            relative_failure("line y", got[1], want[1], tol)?;
            relative_failure("line z", got[2], want[2], tol)?;
        }
        relative_failure("line ends at its end", norm(sub(segment.point_at(length), line.end)), 0.0, 1e-14 * scale_mm)?;
    }

    /// `s_len()` is the length of the curve `point_at` traces: the Richardson
    /// polygon of the position parametrisation converges to it inside the
    /// chord rule's own truncation-plus-rounding budget.
    #[test]
    fn length_equals_the_integrated_speed(segment in arb_segment()) {
        const CHORDS: usize = 1024;
        let length = segment.s_len();
        let (_, kappa_peak) = segment.kappa_peak();
        let (measured, anchor_scale) = richardson_polygon_length(&segment, length, CHORDS);
        let h = length / (2 * CHORDS) as f64;
        let kappa_h = kappa_peak * h;
        let truncation = length * kappa_h * kappa_h * kappa_h * kappa_h / 1920.0;
        let rounding = (2 * CHORDS) as f64 * f64::EPSILON * anchor_scale;
        relative_failure(
            "polygon length",
            measured,
            length,
            POLYGON_BUDGET_SLACK * (truncation + rounding),
        )?;
    }
}
