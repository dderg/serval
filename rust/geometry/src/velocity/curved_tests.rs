use super::certify;
use super::curved::{
    LADDER_RUNGS, MAX_BANDS, caps_at, certified_chain, curved_chain, curved_reach,
    entry_requirement, top_speed_ceiling,
};
use super::disk::{Kinematics, const_kappa_reach_w};
use super::profile::{self, StraightPhase};
use super::{BoundaryInfeasibility, VelocityError};

use crate::fitter::{CornerFitConfig, fit_corners};
use crate::frontend::{MoveContext, VelocityLimits, line_move};
use crate::path::{CurvatureProfile, Segment};
use crate::segment::SourceRange;

/// One cap set plans at most the triple-limited alphabet: swing to the cap,
/// hold, swing back, cruise, and the flanks a nonzero boundary acceleration adds
/// at each end.
const PHASES_PER_CAP_SET: usize = 8;

/// The ladder spends one cap set per rung instead of one per member: a swing
/// up, a hold and a swing down for each rung at each end of the band, plus the
/// cruise the two climbs meet at.
const PHASES_PER_LADDER: usize = 6 * LADDER_RUNGS + 1;

/// Banded emission is the default, so a member's budget is the wider of the two
/// alphabets once per band.
const WORST_PHASES: usize = MAX_BANDS
    * if PHASES_PER_LADDER > PHASES_PER_CAP_SET {
        PHASES_PER_LADDER
    } else {
        PHASES_PER_CAP_SET
    };

/// Phases the committed marcher emits for one reference blend, measured.
const MARCHER_PHASES_PER_BLEND: usize = 598;

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

/// Feasibility judged in this file alone — the state, the curvature and both
/// residuals recomputed from the phase's own coefficients. Gutting
/// `certify.rs` must not be able to keep an emitted chain green, so nothing
/// here may consult it.
const ORACLE_REL_TOL: f64 = 1.0e-9;
const ORACLE_SAMPLES: usize = 64;

fn oracle_state_at(p: &StraightPhase, tau: f64) -> (f64, f64, f64) {
    (
        p.s0 + p.v0 * tau + 0.5 * p.a0 * tau * tau + p.j * tau * tau * tau / 6.0,
        p.v0 + p.a0 * tau + 0.5 * p.j * tau * tau,
        p.a0 + p.j * tau,
    )
}

fn oracle_residuals(kin: &Kinematics, p: &StraightPhase, tau: f64) -> (f64, f64, f64) {
    let (s, v, a) = oracle_state_at(p, tau);
    let k = kin.kappa0 + kin.sigma * s;
    let v2 = v * v;
    let v3 = v2 * v;
    let tangential = p.j - k * k * v3;
    let normal = kin.sigma * v3 + 3.0 * k * v * a;
    (
        kin.accel * kin.accel - a * a - k * k * v2 * v2,
        kin.jerk * kin.jerk - tangential * tangential - normal * normal,
        v,
    )
}

fn assert_feasible_by_oracle(kin: &Kinematics, chain: &[StraightPhase], what: &str) {
    let disk_tol = ORACLE_REL_TOL * kin.accel * kin.accel;
    let ball_tol = ORACLE_REL_TOL * kin.jerk * kin.jerk;
    let speed_tol = ORACLE_REL_TOL * kin.flat_ceiling;
    for (i, p) in chain.iter().enumerate() {
        for n in 0..=ORACLE_SAMPLES {
            let tau = p.dt * (n as f64) / (ORACLE_SAMPLES as f64);
            let (disk, ball, speed) = oracle_residuals(kin, p, tau);
            let breach = |name: &str, residual: f64, tol: f64| {
                assert!(
                    residual >= -tol,
                    "{what}: phase {i} of {} breaches {name} by {residual} at tau={tau} of {} \
                     (s0={} v0={} a0={} j={}; length={} accel={} jerk={} kappa0={} sigma={} \
                     flat={})",
                    chain.len(),
                    p.dt,
                    p.s0,
                    p.v0,
                    p.a0,
                    p.j,
                    kin.length,
                    kin.accel,
                    kin.jerk,
                    kin.kappa0,
                    kin.sigma,
                    kin.flat_ceiling
                );
            };
            breach("the acceleration disk", disk, disk_tol);
            breach("the jerk ball", ball, ball_tol);
            breach("non-reversal", speed, speed_tol);
        }
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
    accelerating_boundaries: usize,
    round_trip_refusals: usize,
    free_refusals: usize,
}

impl Sweep {
    fn record(&mut self, chain: &[StraightPhase]) {
        self.chains += 1;
        self.phases += chain.len();
        self.worst_phases = self.worst_phases.max(chain.len());
    }
}

/// Randomised clothoid members, each planned between the boundary states its own
/// backward solve declares, plus a free forward pass. Both ends carry
/// acceleration: a seam that hands acceleration on is the scenario this solver
/// exists for, and a sweep of `(v, 0)` boundaries never reaches it.
fn sweep(check: &mut dyn FnMut(&Kinematics, &[StraightPhase], &str)) -> Sweep {
    let mut rng = Lcg(0xC10_7401D);
    let mut out = Sweep {
        chains: 0,
        phases: 0,
        worst_phases: 0,
        accelerating_boundaries: 0,
        round_trip_refusals: 0,
        free_refusals: 0,
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

        let entry = (rng.in_range(0.0, ceiling), rng.signed(0.3 * accel));
        let exit = (rng.in_range(0.0, ceiling), rng.signed(0.3 * accel));
        if exit.1 != 0.0 {
            out.accelerating_boundaries += 1;
        }

        let reach = curved_reach(&k, (entry.0, 0.0));
        assert!(
            reach.0.is_finite() && reach.0 >= 0.0,
            "reach speed {} is not a speed",
            reach.0
        );

        match entry_requirement(&k, exit).and_then(|required| curved_chain(&k, required, exit)) {
            Ok(chain) => {
                check(&k, &chain, "backward-required chain");
                out.record(&chain);
            }
            Err(_) => out.round_trip_refusals += 1,
        }
        match curved_chain(&k, entry, exit) {
            Ok(chain) => {
                check(&k, &chain, "forward chain");
                out.record(&chain);
            }
            Err(_) => out.free_refusals += 1,
        }
    }
    out
}

/// The refusal rate is part of the contract: a sweep that silently skips what it
/// cannot plan would pass just as happily with a solver that refuses everything.
fn assert_sweep_is_substantial(s: &Sweep) {
    assert!(
        s.chains >= 650,
        "the sweep produced only {} chains — it is not exercising the solver",
        s.chains
    );
    assert!(
        s.accelerating_boundaries >= 500,
        "only {} of 600 members were given a nonzero exit acceleration",
        s.accelerating_boundaries
    );
    assert!(
        s.round_trip_refusals <= 110,
        "the solver refused {} of 600 of its own backward-required boundary pairs",
        s.round_trip_refusals
    );
    assert!(
        s.free_refusals <= 460,
        "the solver refused {} of 600 free boundary pairs",
        s.free_refusals
    );
}

#[test]
fn every_emitted_phase_is_certified() {
    let summary = sweep(&mut |k, chain, what| {
        assert!(!chain.is_empty(), "{what}: empty chain");
        assert_certified(k, chain, what);
    });
    assert_sweep_is_substantial(&summary);
    assert!(
        summary.worst_phases <= WORST_PHASES,
        "worst chain took {} phases over {} chains ({} total)",
        summary.worst_phases,
        summary.chains,
        summary.phases
    );
}

/// The check the certificate cannot mark its own homework on: every emitted
/// chain is re-judged against residuals this file computes for itself.
#[test]
fn every_emitted_phase_is_feasible_by_an_independent_oracle() {
    let summary = sweep(&mut assert_feasible_by_oracle);
    assert_sweep_is_substantial(&summary);
}

#[test]
fn chain_is_state_continuous() {
    let summary = sweep(&mut |_, chain, what| assert_continuous(chain, what));
    assert_sweep_is_substantial(&summary);
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
            assert_feasible_by_oracle(k, &chain, "round trip");
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
        Err(VelocityError::InfeasibleBoundary(
            BoundaryInfeasibility::SpeedChangeWithoutAuthority { from, to },
        )) => {
            assert_eq!(from, 0.0);
            assert_eq!(to, top_speed_ceiling(&short));
        }
        other => panic!("expected SpeedChangeWithoutAuthority, got {other:?}"),
    }

    let too_short_to_ramp = kin(0.0, 0.0, 1.0e-4, 30_000.0, 1.0e7, 300.0);
    match curved_chain(&too_short_to_ramp, (0.0, 0.0), (200.0, 0.0)) {
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
            assert_feasible_by_oracle(half, &chain, "reference blend");
            assert_continuous(&chain, "reference blend");
            total += chain.len();
            worst = worst.max(chain.len());
        }
    }
    assert!(
        total <= MARCHER_PHASES_PER_BLEND / 10,
        "the reference biclothoid took {total} phases ({worst} worst chain); today's marcher \
         emits ~{MARCHER_PHASES_PER_BLEND} for one blend and this solver must stay an order of \
         magnitude under it"
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

/// `certified_chain` is the module's emission gate: nothing reaches a caller
/// without passing it. Feed it raw phases the solver's own cap model would never
/// produce — full-authority accelerations and jerks held long enough to leave
/// the disk, the ball, or reverse the motion — and judge every phase it lets
/// through with this file's own residuals. A weakened certificate shows up here
/// as an emitted phase the oracle refuses.
#[test]
fn the_emission_gate_never_passes_an_infeasible_phase() {
    let mut rng = Lcg(0x9A7E_0FFE_1234);
    let mut emitted = 0usize;
    let mut refused = 0usize;
    for _ in 0..3000 {
        let accel = rng.in_range(1.0e3, 1.0e5);
        let jerk = rng.in_range(1.0e5, 2.0e8);
        let flat = rng.in_range(20.0, 400.0);
        let length = rng.in_range(0.05, 8.0);
        let k = kin(rng.signed(0.8), rng.signed(6.0), length, accel, jerk, flat);
        let aggression = rng.in_range(0.02, 1.0);
        let raw = StraightPhase {
            t0: 0.0,
            dt: aggression * rng.in_range(1.0e-4, 2.0e-2),
            s0: rng.in_range(0.0, length),
            v0: rng.in_range(0.0, flat),
            a0: rng.signed(aggression * accel),
            j: rng.signed(aggression * jerk),
        };
        match certified_chain(&k, &[raw]) {
            Ok(chain) => {
                assert_feasible_by_oracle(&k, &chain, "emission gate");
                assert_continuous(&chain, "emission gate");
                let span: f64 = chain.iter().map(|p| p.dt).sum();
                assert!(
                    (span - raw.dt).abs() <= 1.0e-12 * raw.dt,
                    "the gate must re-emit the whole phase or refuse it: {span} against {}",
                    raw.dt
                );
                emitted += 1;
            }
            Err(VelocityError::UncertifiedPhase { .. }) => refused += 1,
            other => panic!("the gate must refuse loudly, got {other:?}"),
        }
    }
    assert!(
        refused >= 300,
        "only {refused} of 3000 adversarial phases were refused — the fixture is not reaching \
         the infeasible side and cannot judge the certificate"
    );
    assert!(
        emitted >= 300,
        "only {emitted} of 3000 adversarial phases were emitted — the fixture is not reaching \
         the feasible side"
    );
}

/// The jerk budget is spent where it is owed, not reserved by a flat share: at a
/// member's ceiling a steady pass owes the whole of it, so no authority is left
/// to swing the acceleration with, and below the ceiling the leftover is what
/// the swings may spend.
#[test]
fn the_ceiling_spends_the_whole_jerk_budget_on_a_steady_pass() {
    let mut rng = Lcg(0x10F5_1234_9ABC);
    let mut jerk_bound = 0usize;
    for _ in 0..20_000 {
        let k = kin(
            rng.signed(0.8),
            rng.signed(6.0),
            rng.in_range(0.05, 8.0),
            rng.in_range(1.0e3, 1.0e5),
            rng.in_range(1.0e5, 2.0e8),
            rng.in_range(20.0, 400.0),
        );
        let ceiling = top_speed_ceiling(&k);
        let kappa_peak = k.kappa0.abs().max((k.kappa0 + k.sigma * k.length).abs());
        let gain = libm::hypot(kappa_peak * kappa_peak, k.sigma.abs());
        let steady = ceiling * ceiling * ceiling * gain;
        assert!(
            steady <= k.jerk * (1.0 + 1.0e-9),
            "a steady pass at the ceiling {ceiling} demands {steady} of a {} budget",
            k.jerk
        );
        let caps = caps_at(&k, ceiling);
        if steady >= k.jerk * (1.0 - 1.0e-9) && kappa_peak > 0.0 {
            jerk_bound += 1;
            assert!(
                caps.a <= 1.0e-9 * k.accel && caps.j <= 1.0e-9 * k.jerk,
                "the jerk rail left authority a={} j={} behind (kappa0={} sigma={})",
                caps.a,
                caps.j,
                k.kappa0,
                k.sigma
            );
        }
        assert!(
            caps_at(&k, 0.9 * ceiling).j > 0.0,
            "no jerk authority at nine tenths of the ceiling {ceiling}"
        );
    }
    assert!(
        jerk_bound > 1_000,
        "only {jerk_bound} members were jerk-rail bound — the sweep no longer reaches the regime \
         the full-budget ceiling exists for"
    );
}

#[test]
fn a_nonzero_exit_acceleration_never_reverses_the_motion() {
    let mut rng = Lcg(0xBAD5_EED0);
    let mut chains = 0usize;
    let mut worst = f64::INFINITY;
    for _ in 0..4000 {
        let length = rng.in_range(0.05, 8.0);
        let sigma = rng.signed(6.0);
        let kappa0 = rng.signed(0.8);
        let accel = rng.in_range(1.0e3, 1.0e5);
        let jerk = rng.in_range(1.0e5, 2.0e8);
        let flat = rng.in_range(20.0, 400.0);
        let k = kin(kappa0, sigma, length, accel, jerk, flat);
        let ceiling = top_speed_ceiling(&k);
        let exit = (rng.in_range(0.0, ceiling), rng.signed(0.3 * accel));
        let Ok(required) = entry_requirement(&k, exit) else {
            continue;
        };
        let Ok(chain) = curved_chain(&k, required, exit) else {
            continue;
        };
        chains += 1;
        for p in &chain {
            for i in 0..=64 {
                let tau = p.dt * (i as f64) / 64.0;
                let (_, v, _) = oracle_state_at(p, tau);
                worst = worst.min(v);
            }
        }
    }
    assert!(chains >= 1000, "probe built only {chains} chains");
    assert!(
        worst >= -1.0e-9,
        "an emitted chain reversed: interior speed reached {worst} mm/s over {chains} chains"
    );
}

/// Highest speed the chain reaches. A phase's speed is quadratic in local time,
/// so it peaks at an end unless the jerk is winding a drive down, where the
/// interior stationary point is the peak.
fn chain_peak_speed(chain: &[StraightPhase]) -> f64 {
    chain.iter().fold(0.0_f64, |peak, p| {
        let (_, v_end, _) = p.end_state();
        let winds_down = p.j < 0.0 && p.a0 > 0.0;
        let crest = if winds_down {
            let tau = (-p.a0 / p.j).min(p.dt);
            p.v0 + p.a0 * tau + 0.5 * p.j * tau * tau
        } else {
            0.0
        };
        peak.max(p.v0).max(v_end).max(crest)
    })
}

const RIM_ARC_RADIUS: f64 = 20.0;
const RIM_ARC_ACCEL: f64 = 2000.0;
const RIM_ARC_JERK: f64 = 1.0e5;

/// Peak the member's single cap set reaches on the reference arc, measured. The
/// whole ramp is priced at the authority left at the top speed, so the search
/// settles for a top speed the disk would let it beat by 28 mm/s.
const SINGLE_CAP_SET_ARC_PEAK: f64 = 171.785;

/// Share of the disk rim the ladder rides on the reference arc, measured at
/// 198.253 of 200 mm/s.
const LADDER_ARC_RIM_SHARE: f64 = 0.99;

/// A half-circle long enough that nothing but the acceleration disk can bound
/// it: the rim `sqrt(accel * radius)` is the speed a steady pass spends the
/// whole disk on, and a rest-to-rest pass has to buy every mm/s of it with the
/// authority the disk has left at the speed it is doing at the time.
#[test]
fn the_ladder_rides_the_disk_rim_one_cap_set_prices_out_of_reach() {
    let rim = (RIM_ARC_ACCEL * RIM_ARC_RADIUS).sqrt();
    let k = kin(
        1.0 / RIM_ARC_RADIUS,
        0.0,
        std::f64::consts::PI * RIM_ARC_RADIUS,
        RIM_ARC_ACCEL,
        RIM_ARC_JERK,
        500.0,
    );
    assert_eq!(
        top_speed_ceiling(&k),
        rim,
        "the arc must be disk bound for the rim to be the thing being ridden"
    );

    let chain = curved_chain(&k, (0.0, 0.0), (0.0, 0.0)).unwrap();
    assert_certified(&k, &chain, "reference arc");
    assert_feasible_by_oracle(&k, &chain, "reference arc");
    assert_continuous(&chain, "reference arc");

    let peak = chain_peak_speed(&chain);
    assert!(
        peak > SINGLE_CAP_SET_ARC_PEAK,
        "peak {peak} did not beat the single cap set's measured {SINGLE_CAP_SET_ARC_PEAK}"
    );
    assert!(
        peak >= rim * LADDER_ARC_RIM_SHARE,
        "peak {peak} fell off the disk rim {rim}"
    );
    assert!(peak <= rim, "peak {peak} rode over the disk rim {rim}");
}
