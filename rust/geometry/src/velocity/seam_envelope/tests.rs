use super::*;

fn straight(ceiling: f64, run_mm: f64) -> SeamTerm {
    SeamTerm {
        limit: SeamLimit::ceiling(ceiling),
        run_mm,
    }
}

fn curved(ceiling: f64, kappa: f64, run_mm: f64) -> SeamTerm {
    SeamTerm {
        limit: SeamLimit {
            ceiling_mm_s: ceiling,
            kappa,
        },
        run_mm,
    }
}

fn envelope(terms: &[SeamTerm]) -> SeamEnvelope {
    terms[1..]
        .iter()
        .fold(SeamEnvelope::of(terms[0].limit).across(terms[0].run_mm), |envelope, term| {
            envelope.with(*term)
        })
}

#[test]
fn a_rest_drops_every_seam_beyond_it() {
    let kept = envelope(&[straight(80.0, 30.0), curved(200.0, 2.0, 12.0), straight(0.0, 5.0)]);
    assert_eq!(kept.terms(), &[straight(0.0, 5.0)]);
}

#[test]
fn a_far_tight_curve_and_a_near_ceiling_both_bind() {
    let near = straight(100.0, 0.2);
    let far = curved(300.0, 50.0, 4.0);
    let kept = envelope(&[far, near]);
    assert_eq!(kept.terms(), &[far, near]);
    let low_budget = 200.0;
    let high_budget = 20000.0;
    assert!(far.speed_sq(low_budget) < near.speed_sq(low_budget));
    assert!(near.speed_sq(high_budget) < far.speed_sq(high_budget));
    for accel in [low_budget, 2000.0, high_budget] {
        assert_eq!(
            kept.speed_sq(accel),
            far.speed_sq(accel).min(near.speed_sq(accel))
        );
    }
}

#[test]
fn a_seam_slower_and_nearer_drops_one_beyond_it() {
    let kept = envelope(&[curved(150.0, 0.5, 9.0), curved(120.0, 0.5, 1.0)]);
    assert_eq!(kept.terms(), &[curved(120.0, 0.5, 1.0)]);
}

#[test]
fn the_envelope_holds_the_least_of_its_seams_at_every_budget() {
    let seams = [
        straight(250.0, 40.0),
        curved(180.0, 3.0, 22.0),
        straight(90.0, 15.0),
        curved(300.0, 20.0, 6.0),
        straight(140.0, 0.5),
    ];
    let kept = envelope(&seams);
    for step in 1..200 {
        let accel = 150.0 * f64::from(step);
        let least = seams
            .iter()
            .map(|seam| seam.speed_sq(accel))
            .fold(f64::INFINITY, f64::min);
        assert_eq!(kept.speed_sq(accel), least, "at {accel} mm/s²");
    }
}

#[test]
fn a_move_between_rests_takes_its_trapezoid() {
    let length = 10.0;
    let reach = ReachableSpeed {
        peak_mm_s: 100.0,
        behind: SeamEnvelope::of(SeamLimit::ceiling(0.0)),
        ahead: SeamEnvelope::of(SeamLimit::ceiling(0.0)),
        length_mm: length,
    };
    let (ceiling, accel) = (50.0, 1000.0);
    let ramp_mm = ceiling * ceiling / (2.0 * accel);
    let expected = 2.0 * ceiling / accel + (length - 2.0 * ramp_mm) / ceiling;
    assert!((reach.run_time(ceiling, accel) - expected).abs() <= 1e-12 * expected);
    let triangle = 2.0 * (length / accel).sqrt();
    assert!((reach.run_time(1e6, accel) - triangle).abs() <= 1e-12 * triangle);
}

#[test]
fn the_run_reads_the_seams_beyond_the_move() {
    let length = 4.0;
    let accel = 2000.0;
    let reach = ReachableSpeed {
        peak_mm_s: 100.0,
        behind: SeamEnvelope::of(SeamLimit::ceiling(0.0)).across(6.0),
        ahead: SeamEnvelope::of(SeamLimit::ceiling(0.0)),
        length_mm: length,
    };
    let triangle = 2.0 * ((6.0 + length) / accel).sqrt();
    assert!((reach.run_time(1e6, accel) - triangle).abs() <= 1e-12 * triangle);
    let peak_sq = 2.0 * accel * 6.0 * 0.5 + accel * length;
    assert!((reach.run_peak(accel) - peak_sq.min(2.0 * accel * length).sqrt()).abs() <= 1e-9);
}
