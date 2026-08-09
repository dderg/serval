use super::chain::phase_end_s;
use super::state::time_to_cross;
use super::*;

#[test]
fn time_to_cross_from_rest_is_the_jerk_cubic() {
    let st = State {
        t: 0.0,
        s: 0.0,
        v: 0.0,
        a: 0.0,
    };
    let (j, ds) = (1e5, 0.01);
    let t = time_to_cross(st, j, ds).unwrap();
    let expected = libm::cbrt(6.0 * ds / j);
    assert!((t - expected).abs() < 1e-12 * expected, "t={t}");
}

#[test]
fn time_to_cross_reports_a_stall() {
    // Braking to rest before covering the span.
    let st = State {
        t: 0.0,
        s: 0.0,
        v: 1.0,
        a: -100.0,
    };
    assert!(time_to_cross(st, 0.0, 1.0).is_none());
}

#[test]
fn advance_is_the_constant_jerk_cubic() {
    let st = State {
        t: 1.0,
        s: 2.0,
        v: 10.0,
        a: 100.0,
    };
    let (j, dt) = (1e4, 0.02);
    let n = advance(st, j, dt);
    assert!((n.s - (2.0 + 10.0 * dt + 50.0 * dt * dt + j * dt * dt * dt / 6.0)).abs() < 1e-12);
    assert!((n.v - (10.0 + 100.0 * dt + 0.5 * j * dt * dt)).abs() < 1e-12);
    assert!((n.a - (100.0 + j * dt)).abs() < 1e-12);
    assert!((n.t - 1.02).abs() < 1e-15);
}

fn eval_chain(chain: &[StraightPhase], t: f64) -> (f64, f64, f64) {
    for p in chain {
        if t <= p.t0 + p.dt + 1e-15 {
            let tau = (t - p.t0).max(0.0);
            return (
                p.s0 + p.v0 * tau + 0.5 * p.a0 * tau * tau + p.j * tau * tau * tau / 6.0,
                p.v0 + p.a0 * tau + 0.5 * p.j * tau * tau,
                p.a0 + p.j * tau,
            );
        }
    }
    let p = chain.last().unwrap();
    eval_chain(std::slice::from_ref(p), p.t0 + p.dt)
}

fn two_phase_chain() -> Vec<StraightPhase> {
    // Jerk up from rest, then hold the accel.
    let p0 = StraightPhase {
        t0: 0.0,
        dt: 0.01,
        s0: 0.0,
        v0: 0.0,
        a0: 0.0,
        j: 1e5,
    };
    let st = advance(
        State {
            t: 0.0,
            s: 0.0,
            v: 0.0,
            a: 0.0,
        },
        p0.j,
        p0.dt,
    );
    let p1 = StraightPhase {
        t0: p0.dt,
        dt: 0.05,
        s0: st.s,
        v0: st.v,
        a0: st.a,
        j: 0.0,
    };
    vec![p0, p1]
}

#[test]
fn clip_phases_tiles_the_chain() {
    let chain = two_phase_chain();
    let end = phase_end_s(&chain[1]);
    let cut = 0.6 * end;
    let head = clip_phases(&chain, 0.0, cut);
    let tail = clip_phases(&chain, cut, end);
    let total: f64 = chain.iter().map(|p| p.dt).sum();
    let split: f64 = head.iter().chain(&tail).map(|p| p.dt).sum();
    assert!((total - split).abs() < 1e-12, "durations must tile");
    let head_len = phase_end_s(head.last().unwrap());
    assert!((head_len - cut).abs() < 1e-9, "head spans to the cut");
    let (s0, v0, a0) = eval_chain(&tail, 0.0);
    assert!(s0.abs() < 1e-12, "tail rebased to s=0");
    let t_cut: f64 = head.iter().map(|p| p.dt).sum();
    let (_, vh, ah) = eval_chain(&chain, t_cut);
    assert!(
        (v0 - vh).abs() < 1e-9 && (a0 - ah).abs() < 1e-6,
        "C1 at cut"
    );
}

#[test]
fn reverse_chain_maps_states_into_the_forward_frame() {
    let chain = two_phase_chain();
    let total_len = phase_end_s(&chain[1]);
    let fwd = reverse_chain(&chain, total_len);
    assert_eq!(fwd.len(), 2);
    // Backward start (rest at the exit) becomes the forward end.
    let last = &fwd[1];
    let end = advance(
        State {
            t: 0.0,
            s: last.s0,
            v: last.v0,
            a: last.a0,
        },
        last.j,
        last.dt,
    );
    assert!((end.s - total_len).abs() < 1e-9);
    assert!(end.v.abs() < 1e-12, "forward profile ends at rest");
    assert!(end.a.abs() < 1e-9, "forward profile ends at zero accel");
    // Chain is time-contiguous from zero.
    assert!(fwd[0].t0.abs() < 1e-15);
    assert!((fwd[1].t0 - fwd[0].dt).abs() < 1e-12);
}

/// A biclothoid corner dip under an effectively-unlimited jerk: the flight
/// out of the dip must return to full acceleration. Regression for the
/// zero-progress limit cycle where a jerk swing narrower than one
/// floor-duration step ping-ponged `a` across the rail until the cell guard
/// stalled the pass, pinning the whole remaining line to the corner speed.
#[test]
fn flight_reaccelerates_after_dip_with_huge_jerk() {
    let (l1, blen, sigma) = (9.9026_f64, 0.0819_f64, 234.47_f64);
    let total = 2.0 * l1 + 2.0 * blen;
    let kappa_of = |x: f64| -> f64 {
        if x < l1 || x > l1 + 2.0 * blen {
            0.0
        } else if x < l1 + blen {
            sigma * (x - l1)
        } else {
            sigma * (l1 + 2.0 * blen - x)
        }
    };
    let mut s: Vec<f64> = Vec::new();
    let mut x = 0.0;
    while x < total {
        s.push(x);
        x += 0.01;
    }
    s.push(total);
    let n = s.len();
    let kappa: Vec<f64> = s.iter().map(|&x| kappa_of(x)).collect();
    let cap_v: Vec<f64> = kappa
        .iter()
        .map(|&k| {
            if k > 1e-12 {
                (70000.0_f64 / k).sqrt().min(1000.0)
            } else {
                1000.0
            }
        })
        .collect();
    let cap_a: Vec<f64> = s
        .windows(2)
        .zip(cap_v.windows(2))
        .map(|(sw, vw)| (vw[1] * vw[1] - vw[0] * vw[0]) / (2.0 * (sw[1] - sw[0])))
        .collect();
    let accel = vec![70000.0_f64; n];
    let track = Track {
        s: &s,
        cap_v: &cap_v,
        cap_a: &cap_a,
        accel: &accel,
        kappa: &kappa,
        j_max: 1e11,
    };
    let pass = reach_pass(&track, 0.0, 0.0, None);
    let at = |x: f64| pass.v[s.iter().position(|&q| q >= x).unwrap()];
    let apex = at(l1 + blen);
    assert!(
        (55.0..70.0).contains(&apex),
        "corner speed should sit at the curvature limit, got {apex}"
    );
    assert!(
        at(15.0) > 800.0,
        "flight must re-accelerate after the dip, got {} at s=15",
        at(15.0)
    );
    assert!(
        pass.v[n - 1] > 990.0,
        "flight must regain the flat ceiling, got {}",
        pass.v[n - 1]
    );
}

/// A straight track over `cap_v` on a 0.01 mm grid; the pass starts on the
/// cap at the first node.
fn cap_pass(cap_v: Vec<f64>, accel_v: f64, j_max: f64) -> Pass {
    let n = cap_v.len();
    let s: Vec<f64> = (0..n).map(|i| i as f64 * 0.01).collect();
    let mut cap_a: Vec<f64> = s
        .windows(2)
        .zip(cap_v.windows(2))
        .map(|(sw, vw)| (vw[1] * vw[1] - vw[0] * vw[0]) / (2.0 * (sw[1] - sw[0])))
        .collect();
    cap_a.push(*cap_a.last().unwrap());
    let accel = vec![accel_v; n];
    let kappa = vec![0.0; n];
    let track = Track {
        s: &s,
        cap_v: &cap_v,
        cap_a: &cap_a,
        accel: &accel,
        kappa: &kappa,
        j_max,
    };
    reach_pass(&track, cap_v[0], 0.0, None)
}

/// A straight track whose cap descends through `wall_caps` between flat
/// stretches, with grid step 0.01 mm; the pass starts on the cap.
fn wall_pass(wall_caps: &[f64], accel_v: f64, j_max: f64) -> Pass {
    let mut cap_v: Vec<f64> = vec![100.0; 6];
    cap_v.extend_from_slice(wall_caps);
    cap_v.extend(std::iter::repeat(*wall_caps.last().unwrap()).take(6));
    let n = cap_v.len();
    let s: Vec<f64> = (0..n).map(|i| i as f64 * 0.01).collect();
    let mut cap_a: Vec<f64> = s
        .windows(2)
        .zip(cap_v.windows(2))
        .map(|(sw, vw)| (vw[1] * vw[1] - vw[0] * vw[0]) / (2.0 * (sw[1] - sw[0])))
        .collect();
    cap_a.push(*cap_a.last().unwrap());
    let accel = vec![accel_v; n];
    let kappa = vec![0.0; n];
    let track = Track {
        s: &s,
        cap_v: &cap_v,
        cap_a: &cap_a,
        accel: &accel,
        kappa: &kappa,
        j_max,
    };
    reach_pass(&track, cap_v[0], 0.0, None)
}

/// A raw-vlc wall dropping faster than the accel rail is infeasible from
/// everywhere within its cells: the pass must cross it as cap chords, marked
/// infeasible, with a complete phase chain — not peel against it (the peel
/// storm of ride-pass defect 2: the departure bisect degenerates onto the
/// current position and the contact bisection converges onto touch = 0).
#[test]
fn super_rail_wall_is_crossed_as_infeasible_chords() {
    let pass = wall_pass(&[30.0, 1.0], 3000.0, 1e6);
    assert!(pass.complete);
    assert_eq!(pass.v[5..9], [100.0, 30.0, 1.0, 1.0]);
    assert_eq!(pass.feasible[5..9], [true, false, false, true]);
}

/// Same treatment cell by cell down a multi-cell super-rail descent.
#[test]
fn multi_cell_super_rail_wall_is_crossed_as_infeasible_chords() {
    let pass = wall_pass(&[80.0, 60.0, 40.0, 20.0, 1.0], 3000.0, 1e6);
    assert!(pass.complete);
    assert_eq!(pass.v[6..11], [80.0, 60.0, 40.0, 20.0, 1.0]);
    assert!(pass.feasible[6..11].iter().all(|f| !f));
    assert!(pass.feasible[11]);
}

/// A feed-step wall within the accel rail but beyond what jerk can land on
/// tangentially (its chord demands a brake slope that would shed more speed
/// than the cap holds). The pass must brake early and anchor on the wall's
/// end node — arriving at exactly the bottom value with a jerk-continuous
/// chain — instead of overbraking toward rest against the chord slope.
#[test]
fn jerk_wall_is_taken_by_an_anchored_brake() {
    let mut cap_v = vec![20.0_f64; 80];
    cap_v.extend(std::iter::repeat(5.0).take(20));
    let n = cap_v.len();
    let pass = cap_pass(cap_v, 33777.0, 1e5);
    assert!(pass.complete);
    assert!(chain_is_continuous(&pass.phases, true));
    assert!(pass.feasible.iter().all(|&f| f));
    let min_v = pass.v.iter().copied().fold(f64::INFINITY, f64::min);
    assert!(
        min_v >= 5.0 - 1e-6,
        "anchored brake must not dip below the wall bottom, got {min_v}"
    );
    assert!((pass.v[80] - 5.0).abs() < 1e-6, "wall node lands its value");
    assert!((pass.v[n - 1] - 5.0).abs() < 1e-6);
}

/// The naive jerk-reachability detach lost chain completeness on a step-up
/// wall followed by a wall drop; the wall-aware pass must cross the notch's
/// inverse (a mesa) with a complete chain and no collapse toward rest.
#[test]
fn mesa_step_up_then_drop_keeps_the_chain_complete() {
    let mut cap_v = vec![5.0_f64; 40];
    cap_v.extend(std::iter::repeat(30.0).take(3));
    cap_v.extend(std::iter::repeat(5.0).take(40));
    let pass = cap_pass(cap_v, 33777.0, 1e5);
    assert!(pass.complete);
    let min_v = pass.v.iter().copied().fold(f64::INFINITY, f64::min);
    assert!(
        min_v >= 5.0 - 1e-6,
        "profile must cruise through the mesa, got {min_v}"
    );
}

/// An ascending wall (feed step up) detaches to flight instead of riding the
/// chord as an instantaneous acceleration staircase: the emitted chain stays
/// state-continuous through the wall foot.
#[test]
fn ascending_wall_detaches_to_flight() {
    let mut cap_v = vec![1.0_f64; 40];
    cap_v.extend(std::iter::repeat(20.0).take(60));
    let n = cap_v.len();
    let pass = cap_pass(cap_v, 33777.0, 1e5);
    assert!(pass.complete);
    assert!(chain_is_continuous(&pass.phases, true));
    assert!(
        pass.v[n - 1] > 10.0,
        "flight must climb after the wall, got {}",
        pass.v[n - 1]
    );
}
