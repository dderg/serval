use super::certify;
use super::curved::{
    certified_chain, curved_chain, curved_reach, entry_requirement, top_speed_ceiling,
};
use super::disk::{Kinematics, const_kappa_reach_w};
use super::profile::{self, StraightPhase};
use super::{BoundaryInfeasibility, VelocityError};

use crate::fitter::{CornerFitConfig, fit_corners};
use crate::frontend::{MoveContext, VelocityLimits, line_move};
use crate::path::{CurvatureProfile, Segment};
use crate::segment::SourceRange;

const REFERENCE_FEED: f64 = 300.0;
const REFERENCE_ACCEL: f64 = 60_000.0;
const REFERENCE_JERK: f64 = 1.5e8;
const REFERENCE_DEVIATION: f64 = 0.05;

fn kin(kappa0: f64, sigma: f64, length: f64, accel: f64, jerk: f64, flat: f64) -> Kinematics {
    Kinematics {
        length,
        accel,
        jerk,
        kappa0,
        sigma,
        flat_ceiling: flat,
    }
}

struct Lcg(u64);

impl Lcg {
    fn next_unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }

    fn in_range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_unit()
    }

    fn signed(&mut self, magnitude: f64) -> f64 {
        self.in_range(-magnitude, magnitude)
    }
}

fn assert_certified(kin: &Kinematics, chain: &[StraightPhase], what: &str) {
    for (i, p) in chain.iter().enumerate() {
        assert!(
            certify::is_certified(kin, p.s0, p.v0, p.a0, p.j, p.dt),
            "{what}: phase {i} of {} is not certified: s0={} v0={} a0={} j={} dt={} \
             (length={} accel={} jerk={} kappa0={} sigma={} flat={})",
            chain.len(),
            p.s0,
            p.v0,
            p.a0,
            p.j,
            p.dt,
            kin.length,
            kin.accel,
            kin.jerk,
            kin.kappa0,
            kin.sigma,
            kin.flat_ceiling
        );
    }
}

fn assert_continuous(chain: &[StraightPhase], what: &str) {
    for (i, pair) in chain.windows(2).enumerate() {
        let (s, v, a) = pair[0].end_state();
        let next = pair[1];
        let close = |lhs: f64, rhs: f64, name: &str| {
            let tol = 1.0e-9 * (1.0 + lhs.abs().max(rhs.abs()));
            assert!(
                (lhs - rhs).abs() <= tol,
                "{what}: {name} jumps at joint {i}: {lhs} -> {rhs}"
            );
        };
        close(s, next.s0, "arc length");
        close(v, next.v0, "speed");
        close(a, next.a0, "acceleration");
        close(pair[0].t0 + pair[0].dt, next.t0, "time");
    }
}

/// A member whose curvature and jerk rail leave the flat ceiling in charge, so
/// the disk ride and the constant-jerk chain agree on where the reach lands.
#[test]
fn arc_reduces_to_the_closed_form_reach() {
    let kappa = 0.02;
    let accel = 60_000.0;
    let flat = 300.0;
    let k = kin(kappa, 0.0, 20.0, accel, 1.0e9, flat);
    assert!(
        top_speed_ceiling(&k) >= flat,
        "the arc must be feedrate bound for the closed form to saturate"
    );

    let (v, a) = curved_reach(&k, (50.0, 0.0));
    let closed_form = const_kappa_reach_w(50.0 * 50.0, k.length, accel, kappa).min(flat * flat);
    assert!(
        (v * v - closed_form).abs() <= 1.0e-9 * closed_form,
        "reach w={} disagrees with the closed form w={closed_form}",
        v * v
    );
    assert!(a.abs() <= 1.0e-6 * accel, "a saturated reach must be flat");

    let mut rng = Lcg(0x5EED_C0FFEE);
    for _ in 0..400 {
        let kappa = rng.in_range(1.0e-3, 1.0);
        let accel = rng.in_range(1.0e3, 1.0e5);
        let flat = rng.in_range(20.0, 400.0);
        let length = rng.in_range(0.05, 20.0);
        let k = kin(kappa, 0.0, length, accel, rng.in_range(1.0e6, 2.0e8), flat);
        let v_in = rng.in_range(0.0, top_speed_ceiling(&k));
        let (v, _) = curved_reach(&k, (v_in, 0.0));
        let bound = const_kappa_reach_w(v_in * v_in, length, accel, kappa).min(flat * flat);
        assert!(
            v * v <= bound * (1.0 + 1.0e-9),
            "reach w={} exceeded the infinite-jerk disk ride w={bound}",
            v * v
        );
    }
}

#[test]
fn line_reduces_to_the_straight_chain() {
    let (v0, v1, length, flat, accel, jerk) = (50.0, 200.0, 40.0, 300.0, 3000.0, 1.0e5);
    let k = kin(0.0, 0.0, length, accel, jerk, flat);
    let mine = curved_chain(&k, (v0, 0.0), (v1, 0.0)).unwrap();
    let theirs = profile::straight_chain(v0, v1, length, flat, accel, jerk);

    assert_eq!(mine.len(), theirs.len(), "phase count must match exactly");
    for (i, (a, b)) in mine.iter().zip(&theirs).enumerate() {
        for (name, lhs, rhs) in [
            ("t0", a.t0, b.t0),
            ("dt", a.dt, b.dt),
            ("s0", a.s0, b.s0),
            ("v0", a.v0, b.v0),
            ("a0", a.a0, b.a0),
            ("j", a.j, b.j),
        ] {
            assert!(
                (lhs - rhs).abs() <= 1.0e-12 * (1.0 + rhs.abs()),
                "phase {i} field {name}: {lhs} != {rhs}"
            );
        }
    }
}

struct Sweep {
    chains: usize,
    phases: usize,
    worst_phases: usize,
}

/// Randomised clothoid members, each planned between the boundary states its own
/// backward solve declares, plus the plain forward reach. Every phase of every
/// chain is put to the caller's check.
fn sweep(check: &mut dyn FnMut(&Kinematics, &[StraightPhase], &str)) -> Sweep {
    let mut rng = Lcg(0xC10_7401D);
    let mut out = Sweep {
        chains: 0,
        phases: 0,
        worst_phases: 0,
    };
    for _ in 0..600 {
        let length = rng.in_range(0.05, 8.0);
        let sigma = rng.signed(6.0);
        let kappa0 = rng.signed(0.8);
        let accel = rng.in_range(1.0e3, 1.0e5);
        let jerk = rng.in_range(1.0e5, 2.0e8);
        let flat = rng.in_range(20.0, 400.0);
        let k = kin(kappa0, sigma, length, accel, jerk, flat);
        let ceiling = top_speed_ceiling(&k);

        let entry = (rng.in_range(0.0, ceiling), 0.0);
        let reach = curved_reach(&k, entry);
        assert!(
            reach.0.is_finite() && reach.0 >= 0.0,
            "reach speed {} is not a speed",
            reach.0
        );

        let exit = (rng.in_range(0.0, ceiling), 0.0);
        if let Ok(required) = entry_requirement(&k, exit) {
            if let Ok(chain) = curved_chain(&k, required, exit) {
                check(&k, &chain, "backward-required chain");
                out.chains += 1;
                out.phases += chain.len();
                out.worst_phases = out.worst_phases.max(chain.len());
            }
        }
        if let Ok(chain) = curved_chain(&k, entry, exit) {
            check(&k, &chain, "forward chain");
            out.chains += 1;
            out.phases += chain.len();
            out.worst_phases = out.worst_phases.max(chain.len());
        }
    }
    out
}

#[test]
fn every_emitted_phase_is_certified() {
    let summary = sweep(&mut |k, chain, what| {
        assert!(!chain.is_empty(), "{what}: empty chain");
        assert_certified(k, chain, what);
    });
    assert!(
        summary.chains >= 400,
        "the sweep produced only {} chains — it is not exercising the solver",
        summary.chains
    );
    assert!(
        summary.worst_phases <= 12,
        "worst chain took {} phases over {} chains ({} total)",
        summary.worst_phases,
        summary.chains,
        summary.phases
    );
}

#[test]
fn chain_is_state_continuous() {
    let summary = sweep(&mut |_, chain, what| assert_continuous(chain, what));
    assert!(summary.chains >= 400, "sweep too thin");
}

fn reference_blend_halves() -> Vec<Kinematics> {
    let ctx = |line_no: u32| MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: REFERENCE_FEED,
        limits: VelocityLimits::try_new(
            REFERENCE_FEED,
            REFERENCE_ACCEL,
            REFERENCE_DEVIATION,
            REFERENCE_JERK,
        )
        .unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    };
    let moves = vec![
        line_move([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0, ctx(1)).unwrap(),
        line_move([10.0, 0.0, 0.0], [10.0, 10.0, 0.0], 0.0, ctx(2)).unwrap(),
    ];
    let fitted = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    let halves: Vec<Kinematics> = fitted
        .moves
        .iter()
        .filter_map(|m| match &m.segment.spatial {
            Some(Segment::Clothoid(c)) => Some(kin(
                c.kappa_endpoints().0,
                c.dkappa_ds(0.0),
                c.s_len(),
                REFERENCE_ACCEL,
                REFERENCE_JERK,
                REFERENCE_FEED,
            )),
            _ => None,
        })
        .collect();
    assert_eq!(
        halves.len(),
        2,
        "the reference corner must fit to a biclothoid"
    );
    halves
}

#[test]
fn biclothoid_apex_respects_the_two_sided_bound() {
    let halves = reference_blend_halves();
    let entry_half = &halves[0];
    let exit_half = &halves[1];
    let kappa_apex = entry_half.kappa0 + entry_half.sigma * entry_half.length;
    assert!(
        (kappa_apex - exit_half.kappa0).abs() <= 1.0e-9 * kappa_apex.abs(),
        "the halves must meet at the curvature peak: {kappa_apex} vs {}",
        exit_half.kappa0
    );

    let (v, a) = curved_reach(entry_half, (0.0, 0.0));
    let credit = entry_half.jerk - entry_half.sigma.abs() * v * v * v;
    let demand = 3.0 * kappa_apex.abs() * v * a.abs();
    assert!(
        demand <= credit,
        "apex demand 3|k|v|a| = {demand} exceeds the two-sided credit j - |sigma|v^3 = {credit} \
         (v={v} a={a} kappa={kappa_apex} sigma={})",
        entry_half.sigma
    );

    let required = entry_requirement(exit_half, (v, 0.0)).unwrap();
    let credit = exit_half.jerk - exit_half.sigma.abs() * required.0 * required.0 * required.0;
    let demand = 3.0 * kappa_apex.abs() * required.0 * required.1.abs();
    assert!(
        demand <= credit,
        "the exit half demands {demand} at the apex against a credit of {credit}"
    );
}

#[test]
fn entry_requirement_round_trips() {
    let cases = [
        kin(0.0, 5.0967, 0.1308, 60_000.0, 1.5e8, 300.0),
        kin(0.6667, -5.0967, 0.1308, 60_000.0, 1.5e8, 300.0),
        kin(0.05, 0.0, 4.0, 30_000.0, 1.0e7, 250.0),
        kin(0.0, 0.0, 4.0, 30_000.0, 1.0e7, 250.0),
        kin(-0.4, 1.5, 1.0, 10_000.0, 5.0e6, 180.0),
    ];
    for k in &cases {
        for exit_v in [0.0, 20.0, 0.5 * top_speed_ceiling(k)] {
            let exit = (exit_v, 0.0);
            let required = entry_requirement(k, exit).unwrap();
            let chain = curved_chain(k, required, exit).unwrap_or_else(|e| {
                panic!("round trip failed for exit {exit:?} required {required:?}: {e:?}")
            });
            assert_certified(k, &chain, "round trip");
            assert_continuous(&chain, "round trip");
            let last = chain.last().expect("round trip produced no phases");
            let (s, v, a) = last.end_state();
            assert!(
                (v - exit.0).abs() <= 1.0e-9 * (1.0 + exit.0.abs()),
                "round trip landed at v={v}, wanted {}",
                exit.0
            );
            assert!(
                (a - exit.1).abs() <= 1.0e-9 * (1.0 + k.accel),
                "round trip landed at a={a}, wanted {}",
                exit.1
            );
            assert!(
                (s - k.length).abs() <= 1.0e-9 * k.length,
                "round trip closed {s} of {} mm",
                k.length
            );
            assert!(
                (chain[0].v0 - required.0).abs() <= 1.0e-9 * (1.0 + required.0),
                "round trip started at v={}, required {}",
                chain[0].v0,
                required.0
            );
        }
    }
}

#[test]
fn infeasible_boundaries_fail_loudly() {
    let k = kin(0.5, 0.0, 1.0, 30_000.0, 1.0e7, 300.0);
    let ceiling = top_speed_ceiling(&k);

    match curved_chain(&k, (ceiling * 4.0, 0.0), (10.0, 0.0)) {
        Err(VelocityError::InfeasibleBoundary(BoundaryInfeasibility::UnwindOverCeiling {
            v,
            v_max,
        })) => {
            assert!(v > v_max, "reported {v} against ceiling {v_max}");
        }
        other => panic!("expected UnwindOverCeiling, got {other:?}"),
    }

    match curved_chain(&k, (0.0, 0.0), (-1.0, 0.0)) {
        Err(VelocityError::InfeasibleBoundary(BoundaryInfeasibility::UnwindBelowRest { v })) => {
            assert!(v < 0.0, "reported unwind speed {v} must be negative")
        }
        other => panic!("expected UnwindBelowRest, got {other:?}"),
    }

    match curved_chain(&k, (f64::NAN, 0.0), (10.0, 0.0)) {
        Err(VelocityError::InfeasibleBoundary(BoundaryInfeasibility::NonFinite)) => {}
        other => panic!("expected NonFinite, got {other:?}"),
    }

    let short = kin(0.5, 0.0, 1.0e-4, 30_000.0, 1.0e7, 300.0);
    match curved_chain(&short, (0.0, 0.0), (top_speed_ceiling(&short), 0.0)) {
        Err(VelocityError::InfeasibleBoundary(BoundaryInfeasibility::LengthTooShort {
            length,
            minimum,
        })) => {
            assert!(length < minimum, "reported {length} against {minimum}");
        }
        other => panic!("expected LengthTooShort, got {other:?}"),
    }
}

#[test]
fn a_reference_blend_costs_a_handful_of_phases() {
    let halves = reference_blend_halves();
    let mut total = 0usize;
    let mut worst = 0usize;
    for half in &halves {
        let ceiling = top_speed_ceiling(half);
        let exit = curved_reach(half, (0.0, 0.0));
        let required = entry_requirement(half, exit).unwrap();
        for chain in [
            curved_chain(half, required, exit).unwrap(),
            curved_chain(half, (ceiling, 0.0), (ceiling, 0.0)).unwrap(),
        ] {
            assert_certified(half, &chain, "reference blend");
            assert_continuous(&chain, "reference blend");
            total += chain.len();
            worst = worst.max(chain.len());
        }
    }
    assert!(
        total <= 12,
        "the reference biclothoid took {total} phases ({worst} worst chain); today's marcher \
         emits ~598 for one blend and this solver must stay in the tens"
    );
}

/// The certificate is one-sided, so a phase it refuses is split at its dwell and
/// retried from the certified end state. A phase that leaves the disk part way
/// through can make no further progress once the march reaches the violation, and
/// that must surface as an error rather than an unbounded stream of ever shorter
/// phases — an earlier planner allocated 32 GB doing exactly that.
#[test]
fn an_uncertifiable_phase_fails_loudly_instead_of_spinning() {
    let k = kin(0.5, 0.0, 10.0, 30_000.0, 1.0e7, 300.0);
    let leaves_the_disk = StraightPhase {
        t0: 0.0,
        dt: 0.05,
        s0: 0.0,
        v0: 10.0,
        a0: 0.0,
        j: 4.0e7,
    };
    match certified_chain(&k, &[leaves_the_disk]) {
        Err(VelocityError::UncertifiedPhase { j, dt, .. }) => {
            assert_eq!(j, leaves_the_disk.j);
            assert!(
                dt > 0.0 && dt <= leaves_the_disk.dt,
                "the reported remainder {dt} must be a live piece of the phase"
            );
        }
        other => panic!("expected UncertifiedPhase, got {other:?}"),
    }

    let feasible = StraightPhase {
        t0: 0.0,
        dt: 1.0e-3,
        s0: 0.0,
        v0: 10.0,
        a0: 0.0,
        j: 1.0e6,
    };
    let kept = certified_chain(&k, &[feasible]).unwrap();
    assert_eq!(kept.len(), 1, "a certified phase must pass through whole");
    assert_eq!(kept[0], feasible);
}
