//! A ratio-ramped advance builds its lead as the leaders leave a rest and
//! releases it as they come to one no faster than the advance itself would:
//! the extruder demand at a start or a stop stays within the advance's
//! natural rate, `k·r·A` in velocity, and like the corner's within the
//! extruder's limits the planner holds it to.

use motion_core::seam_test_harness::LiveReplay;
use motion_pipeline::TrajectoryItem;
use trajectory::ContinuousSegment;

const EXTRUDER_AXIS: usize = 3;
const SAMPLE_DT_S: f64 = 1e-5;
const MOVING_MM_S: f64 = 5.0;
const CORNER_WINDOW_MM: f64 = 15.0;
const RATIO: f64 = 0.4 * 0.2 / (std::f64::consts::PI * 0.875 * 0.875);
const P1_ADVANCE_K: f64 = 0.5;
const P1_MAX_ACCEL: f64 = 3000.0;
const P1_EXTRUDER_MAX_VELOCITY: f64 = 45.0;
const P1_EXTRUDER_MAX_ACCEL: f64 = 2000.0;
/// `max (d/du)(u·ψ(u²))` for the rise `ψ(x) = 1 − (1 − x)³` a valley side
/// takes from a rest: how far the lead's rate overshoots `k·r·A` while it
/// catches up with the advance under a constant acceleration.
const RISE_RATE_OVERSHOOT: f64 = 1.653;

fn planner_config(name: &str) -> planner_config::PlannerConfig {
    let path = source_tree::manifest_dir().join("tests/configs").join(name);
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let doc =
        config_doc::Document::parse(&text, &path.display().to_string()).expect("config parses");
    let (settings, _) =
        planner_config::from_doc::read_motion_settings(&doc).expect("motion settings");
    planner_config::from_doc::planner_config_from_settings(&settings).expect("planner config")
}

fn replay(config: &str, gcode: &str) -> Vec<ContinuousSegment> {
    let replay = LiveReplay::new(gcode, &planner_config(config)).expect("the replay builds");
    let (emitted_tx, emitted) = crossbeam_channel::unbounded();
    let mut pipeline = replay.pipeline().into_staged(move |item| {
        if let TrajectoryItem::Seg(segment) = item {
            emitted_tx
                .send(segment)
                .expect("the test collects every segment");
        }
        true
    });
    for m in replay.moves {
        assert!(pipeline.feed(m.into()), "the pipeline refused a move");
    }
    assert!(pipeline.finish(), "the pipeline failed to drain");
    emitted.try_iter().collect()
}

struct Sample {
    path: f64,
    x: f64,
    y: f64,
    speed: f64,
    ve: f64,
    ae: f64,
}

fn sample(segments: &[ContinuousSegment]) -> Vec<Sample> {
    let mut samples = Vec::new();
    let mut path = 0.0;
    for segment in segments {
        let mut t = segment.t_start;
        while t < segment.t_end {
            let x = segment.eval_axis(0, t).expect("x evaluates");
            let y = segment.eval_axis(1, t).expect("y evaluates");
            let e = segment.eval_axis(EXTRUDER_AXIS, t).expect("e evaluates");
            let speed = libm::hypot(x.velocity, y.velocity);
            path += speed * SAMPLE_DT_S;
            samples.push(Sample {
                path,
                x: x.position,
                y: y.position,
                speed,
                ve: e.velocity,
                ae: e.acceleration,
            });
            t += SAMPLE_DT_S;
        }
    }
    samples
}

/// The peak `|ė|` and `|ë|` while the nozzle moves, inside the corner
/// window around `apex` and outside it: the starts and stops.
fn peaks(samples: &[Sample], apex: (f64, f64)) -> ((f64, f64), (f64, f64)) {
    let apex_path = samples
        .iter()
        .min_by(|a, b| {
            let distance = |s: &Sample| libm::hypot(s.x - apex.0, s.y - apex.1);
            distance(a).total_cmp(&distance(b))
        })
        .expect("the run samples")
        .path;
    let fold = |near: bool| {
        samples
            .iter()
            .filter(|s| s.speed > MOVING_MM_S)
            .filter(|s| ((s.path - apex_path).abs() <= CORNER_WINDOW_MM) == near)
            .fold((0.0_f64, 0.0_f64), |(ve, ae), s| {
                (ve.max(s.ve.abs()), ae.max(s.ae.abs()))
            })
    };
    (fold(true), fold(false))
}

fn corner_from_rest_to_rest(speed: f64) -> String {
    format!(
        "G90\nM83\nG1 X0 Y0 F{:.0}\nG1 X60 Y0 E{e:.6}\nG1 X60 Y60 E{e:.6}\n",
        speed * 60.0,
        e = RATIO * 60.0
    )
}

#[test]
fn a_ratio_ramp_builds_and_releases_the_lead_at_rest_within_the_advance_rate() {
    for speed in [50.0, 100.0, 150.0] {
        let samples = sample(&replay(
            "p1_bowden_ramp.cfg",
            &corner_from_rest_to_rest(speed),
        ));
        let ((corner_ve, corner_ae), (rest_ve, rest_ae)) = peaks(&samples, (60.0, 0.0));
        let natural_ve = RATIO * speed + RISE_RATE_OVERSHOOT * P1_ADVANCE_K * RATIO * P1_MAX_ACCEL;
        assert!(
            rest_ve <= natural_ve,
            "at {speed} mm/s the lead at rest moves the extruder at {rest_ve:.1} mm/s, past \
             the advance's rate {natural_ve:.1}"
        );
        for (place, ve, ae) in [
            ("at rest", rest_ve, rest_ae),
            ("at the corner", corner_ve, corner_ae),
        ] {
            assert!(
                ve <= P1_EXTRUDER_MAX_VELOCITY && ae <= P1_EXTRUDER_MAX_ACCEL,
                "at {speed} mm/s the lead {place} drives the extruder at {ve:.1} mm/s and \
                 {ae:.0} mm/s^2, past its {P1_EXTRUDER_MAX_VELOCITY} mm/s and \
                 {P1_EXTRUDER_MAX_ACCEL} mm/s^2"
            );
        }
    }
}
