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
        v[0] * ang.cos() - v[1] * ang.sin(),
        v[0] * ang.sin() + v[1] * ang.cos(),
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
