use super::*;
use crate::path::CurvatureProfile;
use crate::path::lowering::PositionProfile;

const Z: [f64; 3] = [0.0, 0.0, 1.0];
const X: [f64; 3] = [1.0, 0.0, 0.0];

fn end_pose(c: &Clothoid) -> ([f64; 3], [f64; 3]) {
    let l = c.s_len();
    (c.point_at(l), c.heading_at(l))
}

fn assert_connects(
    got: &ClothoidPair,
    p_a: [f64; 3],
    k_a: f64,
    p_b: [f64; 3],
    t_b: [f64; 3],
    k_b: f64,
) {
    assert!(
        dist(got.half1.start_pose, p_a) < 1e-9,
        "half1 must start at p_a"
    );
    let (ks, _) = got.half1.kappa_endpoints();
    assert!((ks - k_a).abs() < 1e-12, "entry kappa {ks} != {k_a}");

    let (pos, heading) = end_pose(&got.half2);
    assert!(dist(pos, p_b) < 1e-7, "exit pos {pos:?} != {p_b:?}");
    assert!(
        signed_angle(heading, t_b, Z).abs() < 1e-7,
        "exit tangent mismatch"
    );
    let (_, ke) = got.half2.kappa_endpoints();
    assert!((ke - k_b).abs() < 1e-9, "exit kappa {ke} != {k_b}");

    let (k1_end_s, k1_end) = got.half1.kappa_endpoints();
    let (k2_start, _) = got.half2.kappa_endpoints();
    let _ = k1_end_s;
    assert!(
        (k1_end - k2_start).abs() < 1e-9,
        "internal kappa step {k1_end} -> {k2_start}"
    );
    let (m1, h1) = end_pose(&got.half1);
    assert!(dist(m1, got.half2.start_pose) < 1e-9, "internal G0 gap");
    assert!(
        signed_angle(h1, got.half2.heading_at(0.0), Z).abs() < 1e-9,
        "internal G1 kink"
    );
}

#[test]
fn hermite_recovers_general_g2_endpoints() {
    let start = Endpoint {
        pose: [0.0; 3],
        tangent: X,
        kappa: 0.2,
    };
    let truth = build_pair(&start, -0.3, Z, 0.8, 2.0, 3.0).unwrap();
    let (p_b, t_b) = end_pose(&truth.half2);

    let got = hermite_g2([0.0; 3], X, 0.2, p_b, t_b, -0.3, Z).expect("hermite must converge");
    assert_connects(&got, [0.0; 3], 0.2, p_b, t_b, -0.3);
}

#[test]
fn hermite_zero_boundary_curvature_corner() {
    let start = Endpoint {
        pose: [0.0; 3],
        tangent: X,
        kappa: 0.0,
    };
    let truth = build_pair(&start, 0.0, Z, 1.1, 1.5, 1.5).unwrap();
    let (p_b, t_b) = end_pose(&truth.half2);

    let got = hermite_g2([0.0; 3], X, 0.0, p_b, t_b, 0.0, Z).expect("hermite must converge");
    assert_connects(&got, [0.0; 3], 0.0, p_b, t_b, 0.0);
}

#[test]
fn hermite_opposite_sign_sharp_apex() {
    let start = Endpoint {
        pose: [0.0; 3],
        tangent: X,
        kappa: 0.46,
    };
    let truth = build_pair(&start, -0.42, Z, 2.4, 0.6, 0.7).unwrap();
    let (p_b, t_b) = end_pose(&truth.half2);

    let got = hermite_g2([0.0; 3], X, 0.46, p_b, t_b, -0.42, Z).expect("hermite must converge");
    assert_connects(&got, [0.0; 3], 0.46, p_b, t_b, -0.42);
}

fn rot_z(v: [f64; 3], ang: f64) -> [f64; 3] {
    [
        v[0] * libm::cos(ang) - v[1] * libm::sin(ang),
        v[0] * libm::sin(ang) + v[1] * libm::cos(ang),
        0.0,
    ]
}

fn assert_blend_g2(b: &GeneralBlend, kappa_in: f64, kappa_out: f64) {
    let (ks, _) = b.half1.kappa_endpoints();
    assert!(
        (ks - kappa_in).abs() < 1e-9,
        "entry kappa {ks} != {kappa_in}"
    );
    let (_, ke) = b.half2.kappa_endpoints();
    assert!(
        (ke - kappa_out).abs() < 1e-9,
        "exit kappa {ke} != {kappa_out}"
    );
    let (_, k1) = b.half1.kappa_endpoints();
    let (k2, _) = b.half2.kappa_endpoints();
    assert!((k1 - k2).abs() < 1e-9, "internal kappa step {k1} -> {k2}");
    let m1 = b.half1.point_at(b.half1.s_len());
    assert!(dist(m1, b.half2.start_pose) < 1e-9, "internal G0 gap");
    let h1 = b.half1.heading_at(b.half1.s_len());
    assert!(
        signed_angle(h1, b.half2.heading_at(0.0), Z).abs() < 1e-9,
        "internal G1 kink"
    );
}

fn anchor(pose: [f64; 3], tangent: [f64; 3], kappa: f64) -> Anchor {
    Anchor {
        pose,
        tangent,
        kappa,
    }
}

#[test]
fn general_blend_rounds_opposite_sign_apex() {
    let vertex = [10.0, 5.0, 0.0];
    let t_in = rot_z(X, 0.9);
    let t_out = rot_z(X, 0.9 - 2.1);
    let delta = 0.08;
    let blend = solve_general(
        anchor(vertex, t_in, 0.46),
        anchor(vertex, t_out, -0.42),
        vertex,
        Z,
        delta,
        5.0,
        5.0,
    )
    .expect("apex must blend");
    assert_blend_g2(&blend, 0.46, -0.42);

    let join = blend.half2.start_pose;
    assert!(
        dist(vertex, join) <= delta + 1e-6,
        "deviation {} exceeds delta {delta}",
        dist(vertex, join)
    );
    assert!(blend.trim_in > 0.0 && blend.trim_out > 0.0);
}

#[test]
fn general_blend_bridges_gapped_arc_endpoints() {
    let apex = [0.0, 0.0, 0.0];
    let p_in = [-0.003, 0.002, 0.0];
    let p_out = [0.003, -0.002, 0.0];
    let t_in = rot_z(X, 0.4);
    let t_out = rot_z(X, 0.4 - 1.6);
    let delta = 0.06;
    let blend = solve_general(
        anchor(p_in, t_in, 0.5),
        anchor(p_out, t_out, -0.45),
        apex,
        Z,
        delta,
        4.0,
        4.0,
    )
    .expect("gapped apex must blend");
    assert_blend_g2(&blend, 0.5, -0.45);
}

#[test]
fn general_blend_handles_line_to_arc_corner() {
    let vertex = [0.0, 0.0, 0.0];
    let t_in = X;
    let t_out = rot_z(X, 1.2);
    let delta = 0.05;
    let blend = solve_general(
        anchor(vertex, t_in, 0.0),
        anchor(vertex, t_out, 0.6),
        vertex,
        Z,
        delta,
        4.0,
        4.0,
    )
    .expect("must blend");
    assert_blend_g2(&blend, 0.0, 0.6);
    assert!(dist(vertex, blend.half2.start_pose) <= delta + 1e-6);
}

fn corner_setup(theta: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let t_in = X;
    let t_out = rot_z(X, theta);
    let v = crate::vec3::turn_normal(t_in, t_out).expect("corner turns");
    (t_in, t_out, v)
}

fn assert_line_line_contract(
    blend: &GeneralBlend,
    vertex: [f64; 3],
    t_in: [f64; 3],
    t_out: [f64; 3],
    delta: f64,
) {
    assert_blend_g2(blend, 0.0, 0.0);
    let a = madd(vertex, -blend.trim_in, t_in);
    assert!(
        dist(blend.half1.start_pose, a) < 1e-9,
        "blend must start on the inbound line at trim_in"
    );
    let b = madd(vertex, blend.trim_out, t_out);
    let end = blend.half2.point_at(blend.half2.s_len());
    assert!(
        dist(end, b) < 1e-7,
        "blend must end on the outbound line at trim_out: {end:?} vs {b:?}"
    );
    let dev = max_dev_from_corner(
        &ClothoidPair {
            half1: blend.half1.clone(),
            half2: blend.half2.clone(),
        },
        a,
        vertex,
        b,
    );
    assert!(
        dev <= delta + 1e-9,
        "corner deviation {dev} exceeds delta {delta}"
    );
}

#[test]
fn line_line_blend_stays_symmetric_when_unclamped() {
    let vertex = [3.0, -2.0, 0.0];
    let (t_in, t_out, v) = corner_setup(1.0);
    let delta = 0.02;
    let blend = solve_line_line(vertex, t_in, t_out, v, 1.0, delta, 20.0, 20.0)
        .expect("solver ok")
        .expect("must blend");
    assert!(
        (blend.trim_in - blend.trim_out).abs() < 1e-12,
        "roomy corner must keep the symmetric analytic blend"
    );
    assert_line_line_contract(&blend, vertex, t_in, t_out, delta);
}

#[test]
fn clamped_corner_extends_into_the_longer_side() {
    let vertex = [3.0, -2.0, 0.0];
    let (t_in, t_out, v) = corner_setup(1.0);
    let delta = 0.05;
    let (budget_in, budget_out) = (0.25, 10.0);
    let symmetric = solve_line_line(vertex, t_in, t_out, v, 1.0, delta, budget_in, budget_in)
        .expect("solver ok")
        .expect("must blend");
    assert!(
        symmetric.trim_in >= budget_in - 1e-9,
        "setup must be budget-clamped, got trim {}",
        symmetric.trim_in
    );

    let blend = solve_line_line(vertex, t_in, t_out, v, 1.0, delta, budget_in, budget_out)
        .expect("solver ok")
        .expect("must blend");
    assert!(
        blend.trim_in <= budget_in + 1e-9,
        "short side must stay within its budget"
    );
    assert!(
        blend.trim_out > 1.5 * budget_in,
        "long side must extend past the symmetric trim, got {}",
        blend.trim_out
    );
    assert!(blend.trim_out <= budget_out + 1e-9);
    assert_line_line_contract(&blend, vertex, t_in, t_out, delta);

    let sym_len = symmetric.half1.s_len() + symmetric.half2.s_len();
    let asym_len = blend.half1.s_len() + blend.half2.s_len();
    assert!(
        asym_len > sym_len,
        "extension must buy a longer blend: {asym_len} vs {sym_len}"
    );
    let (_, sym_peak) = symmetric.half1.kappa_peak();
    let peak = blend.half1.kappa_peak().1.max(blend.half2.kappa_peak().1);
    assert!(
        peak < sym_peak,
        "extension must lower peak curvature: {peak} vs {sym_peak}"
    );
}

#[test]
fn clamped_corner_with_even_budgets_keeps_symmetric_blend() {
    let vertex = [0.0, 0.0, 0.0];
    let (t_in, t_out, v) = corner_setup(0.8);
    let delta = 0.05;
    let blend = solve_line_line(vertex, t_in, t_out, v, 0.8, delta, 0.25, 0.3)
        .expect("solver ok")
        .expect("must blend");
    assert!(
        (blend.trim_in - blend.trim_out).abs() < 1e-12,
        "near-even budgets must not trigger the asymmetric path"
    );
    assert_line_line_contract(&blend, vertex, t_in, t_out, delta);
}

#[test]
fn general_blend_clamps_to_runway_budget() {
    let vertex = [0.0, 0.0, 0.0];
    let t_in = X;
    let t_out = rot_z(X, 1.0);
    let delta = 100.0;
    let budget = 0.4;
    let blend = solve_general(
        anchor(vertex, t_in, 0.3),
        anchor(vertex, t_out, -0.3),
        vertex,
        Z,
        delta,
        budget,
        budget,
    )
    .expect("must blend");
    assert_blend_g2(&blend, 0.3, -0.3);
    assert!(
        blend.trim_in <= budget + 1e-9 && blend.trim_out <= budget + 1e-9,
        "trims must respect runway budget"
    );
}
