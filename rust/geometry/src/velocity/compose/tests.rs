use super::*;

fn straight(length: f64) -> Kinematics {
    Kinematics {
        length,
        accel: 1000.0,
        jerk: 1.0e5,
        kappa0: 0.0,
        sigma: 0.0,
        flat_ceiling: 300.0,
    }
}

fn arc(length: f64, kappa0: f64) -> Kinematics {
    Kinematics {
        kappa0,
        ..straight(length)
    }
}

fn clothoid(length: f64, kappa0: f64, sigma: f64) -> Kinematics {
    Kinematics {
        kappa0,
        sigma,
        ..straight(length)
    }
}

fn run_members<'a>(kins: &'a [Kinematics], boundary: &[(f64, f64)]) -> Vec<RunMember<'a>> {
    kins.iter()
        .zip(&boundary[1..])
        .map(|(kin, &(exit_v, exit_a))| RunMember {
            kin,
            exit_v,
            exit_a,
            exit_ceiling: kin.flat_ceiling,
        })
        .collect()
}

/// The envelope a seam-by-seam settlement leaves: every seam carries the fastest
/// state its predecessor can hand it, which is exactly what composition has to
/// beat.
fn greedy_boundary(kins: &[Kinematics], entry: (f64, f64)) -> Vec<(f64, f64)> {
    let mut boundary = vec![entry];
    for kin in kins {
        let carried = boundary[boundary.len() - 1];
        boundary.push(curved::reachable_exit(kin, carried).expect("the member reaches an exit"));
    }
    boundary
}

fn settle(members: &[RunMember], boundary: &[(f64, f64)]) -> Vec<MemberPlan> {
    members
        .iter()
        .enumerate()
        .map(|(i, m)| MemberPlan {
            entry: boundary[i],
            exit: boundary[i + 1],
            chain: member_chain(m.kin, boundary[i], boundary[i + 1]),
        })
        .collect()
}

#[test]
fn a_sliced_straight_composes_back_into_the_whole_ramp() {
    let halves = [straight(5.0), straight(5.0)];
    let mut boundary = greedy_boundary(&halves, (0.0, 0.0));
    boundary[2] = (0.0, 0.0);
    let members = run_members(&halves, &boundary);
    let mut plans = settle(&members, &boundary);
    let settled = settled_time(&plans);

    absorb_seams(&members, &boundary, &mut plans);
    let composed = settled_time(&plans);

    let whole = straight(10.0);
    let one_ramp = chain_time(
        &member_chain(&whole, boundary[0], boundary[2]).expect("the whole ramp is plannable"),
    );
    assert!(
        (composed - one_ramp).abs() <= 1e-12 * one_ramp,
        "composed {composed}, one ramp {one_ramp}"
    );
    assert!(
        composed < settled,
        "composition must beat the settled seam: {composed} vs {settled}"
    );
}

#[test]
fn composition_never_lengthens_a_run() {
    let runs = [
        vec![straight(5.0), straight(3.0), straight(7.0)],
        vec![straight(10.0), arc(2.0, 0.01), straight(10.0)],
        vec![
            straight(10.0),
            clothoid(0.5, 0.0, 0.004),
            clothoid(0.5, 0.002, -0.004),
            straight(10.0),
        ],
        vec![arc(1.0, 0.05), arc(1.0, 0.05), arc(1.0, 0.05)],
    ];
    for kins in runs {
        let boundary = greedy_boundary(&kins, (0.0, 0.0));
        let members = run_members(&kins, &boundary);
        let mut plans = settle(&members, &boundary);
        let settled = settled_time(&plans);
        absorb_seams(&members, &boundary, &mut plans);
        let composed = settled_time(&plans);
        assert!(
            composed <= settled,
            "composition lengthened a run of {} members: {composed} vs {settled}",
            kins.len()
        );
    }
}

#[test]
fn a_composed_run_hands_one_continuous_state_across_every_seam() {
    let kins = [straight(10.0), arc(2.0, 0.01), straight(10.0)];
    let boundary = greedy_boundary(&kins, (0.0, 0.0));
    let members = run_members(&kins, &boundary);
    let mut plans = settle(&members, &boundary);
    absorb_seams(&members, &boundary, &mut plans);
    for w in plans.windows(2) {
        let leaves = end_state(w[0].chain.as_ref().expect("a planned member"), w[0].entry);
        let resumes = w[1].chain.as_ref().expect("a planned member")[0];
        assert!(
            (leaves.0 - resumes.v0).abs() < 1e-9,
            "speed steps at a seam"
        );
        assert!(
            (leaves.1 - resumes.a0).abs() < 1e-6,
            "acceleration steps at a seam"
        );
    }
}

#[test]
fn a_seam_the_geometry_pins_below_both_members_is_not_absorbed() {
    let kins = [straight(5.0), straight(5.0)];
    let boundary = greedy_boundary(&kins, (0.0, 0.0));
    let mut members = run_members(&kins, &boundary);
    assert_eq!(stretches(&members), vec![0..2]);

    members[0].exit_ceiling = 0.5 * kins[0].flat_ceiling;
    assert!(
        stretches(&members).is_empty(),
        "a binding seam ceiling is a bottleneck the settlement owns"
    );
}

#[test]
fn only_a_continued_clothoid_fuses_into_one_band() {
    let same_arc = [arc(1.0, 0.05), arc(2.0, 0.05)];
    let boundary = greedy_boundary(&same_arc, (0.0, 0.0));
    let bands = bands_of(&run_members(&same_arc, &boundary));
    assert_eq!(bands.len(), 1);
    assert_eq!(bands[0].kin.length, 3.0);
    assert_eq!(bands[0].cuts, vec![0.0, 1.0, 3.0]);

    let turning = [clothoid(1.0, 0.0, 0.1), clothoid(1.0, 0.1, 0.1)];
    let boundary = greedy_boundary(&turning, (0.0, 0.0));
    assert_eq!(bands_of(&run_members(&turning, &boundary)).len(), 1);

    let kinked = [arc(1.0, 0.05), arc(1.0, -0.05)];
    let boundary = greedy_boundary(&kinked, (0.0, 0.0));
    assert_eq!(bands_of(&run_members(&kinked, &boundary)).len(), 2);

    let mixed = [straight(1.0), arc(1.0, 0.05)];
    let boundary = greedy_boundary(&mixed, (0.0, 0.0));
    assert_eq!(bands_of(&run_members(&mixed, &boundary)).len(), 2);
}
