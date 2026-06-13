use std::sync::{Arc, Mutex};

use motion_bridge_native::classify::classify_and_build;
use motion_bridge_native::config::{
    AxisDecl, AxisRegistry, LimitSection, PlannerConfig, PostProcessorDecl, PostProcessorSet,
};
use motion_bridge_native::planner::{DispatchError, PlannerHandle};
use nurbs::bezier::extract_bezier_pieces;
use trajectory::ShapedSegment;

type Recorded = Arc<Mutex<Vec<ShapedSegment>>>;

fn smooth_zv_post_processors(freq_x: f64, freq_y: f64) -> (AxisRegistry, PostProcessorSet) {
    let decls: Vec<AxisDecl> = [("x", "is_x"), ("y", "is_y"), ("z", "")]
        .iter()
        .map(|(name, pp)| AxisDecl {
            name: (*name).to_string(),
            follows: vec![],
            motors: vec![],
            post_processors: if pp.is_empty() {
                vec![]
            } else {
                vec![(*pp).to_string()]
            },
        })
        .collect();
    let registry = AxisRegistry::try_new(decls).unwrap();
    let pp = |name: &str, freq: f64| PostProcessorDecl {
        name: name.to_string(),
        ty: "smooth_zv".to_string(),
        params: vec![("frequency_hz".to_string(), freq)],
    };
    let set =
        PostProcessorSet::try_new(&registry, &[pp("is_x", freq_x), pp("is_y", freq_y)]).unwrap();
    (registry, set)
}

fn recording_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Recorded,
) {
    let recorded: Recorded = Arc::new(Mutex::new(Vec::new()));
    let rec_for_closure = Arc::clone(&recorded);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            rec_for_closure.lock().unwrap().push(seg.clone());
            Ok(())
        });
    (cb, recorded)
}

fn neptune_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    let (registry, post_processors) = smooth_zv_post_processors(55.4, 39.2);
    c.axis_registry = registry;
    c.post_processors = post_processors;
    c.fit_tolerance_mm = 0.05;
    c
}

fn neptune_limit_sections() -> Vec<LimitSection> {
    vec![
        LimitSection {
            name: "gantry".into(),
            axes: vec![0, 1],
            max_velocity: Some(200.0),
            max_accel: Some(2000.0),
            max_jerk: None,
        },
        LimitSection {
            name: "z".into(),
            axes: vec![2],
            max_velocity: Some(20.0),
            max_accel: Some(150.0),
            max_jerk: None,
        },
    ]
}

const Z_HOP_MM: f64 = 10.0;
const Z_HOP_FEEDRATE: f64 = 15.0;

const STEPS_PER_MM: f64 = 400.0;
const MICROSTEP_MM: f64 = 1.0 / STEPS_PER_MM;
const SAMPLE_PERIOD_S: f64 = 50e-6;
const MAX_STEPS_PER_SAMPLE: u32 = 256;

fn z_bernstein_coeffs_of_first_piece(seg: &ShapedSegment) -> Vec<f64> {
    let pieces = extract_bezier_pieces(&seg.axes[2]);
    let first = pieces
        .first()
        .unwrap_or_else(|| panic!("Z axis has no Bezier pieces in segment"));
    first.to_bernstein()
}

fn z_start_of_segment(seg: &ShapedSegment) -> f64 {
    let pieces = extract_bezier_pieces(&seg.axes[2]);
    let first = pieces
        .first()
        .unwrap_or_else(|| panic!("Z axis has no Bezier pieces in segment"));
    first.evaluate(first.u_start)
}

fn z_end_of_segment(seg: &ShapedSegment) -> f64 {
    let pieces = extract_bezier_pieces(&seg.axes[2]);
    let last = pieces
        .last()
        .unwrap_or_else(|| panic!("Z axis has no Bezier pieces in segment"));
    last.evaluate(last.u_end)
}

fn max_z_step_delta_per_sample(seg: &ShapedSegment) -> u32 {
    use nurbs::eval::eval;

    let z_curve = &seg.axes[2];
    let dt = SAMPLE_PERIOD_S;
    let t_start = seg.t_start;
    let t_end = seg.t_end;

    let mut last_step_count: i32 = (z_start_of_segment(seg) / MICROSTEP_MM).round() as i32;
    let mut max_abs_steps: u32 = 0;

    let n_samples = ((t_end - t_start) / dt).ceil() as usize + 1;
    for i in 1..=n_samples {
        let t = (t_start + (i as f64) * dt).min(t_end);
        let p_end = eval(z_curve, t);
        let target: i32 = (p_end / MICROSTEP_MM).round() as i32;
        let abs_steps = target.wrapping_sub(last_step_count).unsigned_abs();
        if abs_steps > max_abs_steps {
            max_abs_steps = abs_steps;
        }
        last_step_count = target;
    }
    max_abs_steps
}

#[test]
fn cold_boot_z_hop_first_piece_starts_at_seed_position() {
    let (dispatch, recorded) = recording_dispatch();
    let mut cfg = neptune_config();
    cfg.limit_sections = neptune_limit_sections();
    let mut h = PlannerHandle::spawn(cfg, dispatch);

    h.kalico_stream_open([0.0, 0.0, 0.0, 0.0])
        .expect("kalico_stream_open (cold-boot Z=0)");

    h.submit_move(
        classify_and_build([0.0, 0.0, 0.0], 0.0, 0.0, Z_HOP_MM, &[], Z_HOP_FEEDRATE)
            .expect("classify Z hop"),
    )
    .expect("submit Z hop");

    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "cold-boot Z hop produced zero dispatched segments — \
         the planner dropped the move entirely",
    );

    let first_seg = &segs[0];
    let z_start = z_start_of_segment(first_seg);

    assert!(
        z_start.abs() < 1e-4,
        "first dispatched Z piece starts at {z_start:.6} mm but seed was 0.0 mm — \
         position discontinuity of {:.4} mm ({:.0} steps at {STEPS_PER_MM} st/mm). \
         This is the cold-boot STEPS_PER_SAMPLE_EXCEEDED bug: the MCU sees \
         abs_steps={:.0} on segment 0 because the first piece does not start \
         at the seeded position.",
        z_start.abs(),
        (z_start / MICROSTEP_MM).abs(),
        (z_start / MICROSTEP_MM).abs(),
    );

    h.shutdown();
}

#[test]
fn cold_boot_z_hop_first_piece_is_cubic() {
    let (dispatch, recorded) = recording_dispatch();
    let mut cfg = neptune_config();
    cfg.limit_sections = neptune_limit_sections();
    let mut h = PlannerHandle::spawn(cfg, dispatch);

    h.kalico_stream_open([0.0, 0.0, 0.0, 0.0])
        .expect("kalico_stream_open (cold-boot Z=0)");

    h.submit_move(
        classify_and_build([0.0, 0.0, 0.0], 0.0, 0.0, Z_HOP_MM, &[], Z_HOP_FEEDRATE)
            .expect("classify Z hop"),
    )
    .expect("submit Z hop");

    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    assert!(!segs.is_empty(), "no segments dispatched");

    let first_seg = &segs[0];
    let bern = z_bernstein_coeffs_of_first_piece(first_seg);

    let pieces = extract_bezier_pieces(&first_seg.axes[2]);
    let first_piece = &pieces[0];
    let duration = first_piece.u_end - first_piece.u_start;

    assert_eq!(
        bern.len(),
        4,
        "first Z piece has {} Bernstein coefficients (degree {}), expected 4 (degree 3). \
         CLAUDE.md mandates uniform cubic. \
         piece domain=[{:.6}, {:.6}] duration={:.6}s. \
         bern={bern:?}",
        bern.len(),
        bern.len().saturating_sub(1),
        first_piece.u_start,
        first_piece.u_end,
        duration,
    );

    h.shutdown();
}

#[test]
fn cold_boot_z_hop_steps_per_sample_within_mcu_limit() {
    let (dispatch, recorded) = recording_dispatch();
    let mut cfg = neptune_config();
    cfg.limit_sections = neptune_limit_sections();
    let mut h = PlannerHandle::spawn(cfg, dispatch);

    h.kalico_stream_open([0.0, 0.0, 0.0, 0.0])
        .expect("kalico_stream_open (cold-boot Z=0)");

    h.submit_move(
        classify_and_build([0.0, 0.0, 0.0], 0.0, 0.0, Z_HOP_MM, &[], Z_HOP_FEEDRATE)
            .expect("classify Z hop"),
    )
    .expect("submit Z hop");

    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "cold-boot Z hop produced zero dispatched segments",
    );

    for (i, seg) in segs.iter().enumerate() {
        let max_delta = max_z_step_delta_per_sample(seg);
        assert!(
            max_delta <= MAX_STEPS_PER_SAMPLE,
            "segment {i}: max Z step-delta per {SAMPLE_PERIOD_S}s sample = {max_delta} \
             exceeds MCU limit {MAX_STEPS_PER_SAMPLE} at {STEPS_PER_MM} st/mm — \
             this reproduces STEPS_PER_SAMPLE_EXCEEDED, axis_idx=2, \
             which faults the MCU on the very first Z motion after boot",
        );
    }

    h.shutdown();
}

#[test]
fn z_hop_after_stream_open_with_nonzero_seed_starts_at_seed() {
    let z_seed = 5.0_f64;

    let (dispatch, recorded) = recording_dispatch();
    let mut cfg = neptune_config();
    cfg.limit_sections = neptune_limit_sections();
    let mut h = PlannerHandle::spawn(cfg, dispatch);

    h.kalico_stream_open([100.0, 200.0, z_seed, 0.0])
        .expect("kalico_stream_open");

    h.submit_move(
        classify_and_build(
            [100.0, 200.0, z_seed],
            0.0,
            0.0,
            Z_HOP_MM,
            &[],
            Z_HOP_FEEDRATE,
        )
        .expect("classify Z hop"),
    )
    .expect("submit Z hop");

    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "Z hop from non-zero seed produced zero dispatched segments",
    );

    let first_seg = &segs[0];
    let z_start = z_start_of_segment(first_seg);
    assert!(
        (z_start - z_seed).abs() < 1e-4,
        "first dispatched Z piece starts at {z_start:.6} mm but seed was {z_seed} mm — \
         position discontinuity of {:.4} mm ({:.0} steps at {STEPS_PER_MM} st/mm)",
        (z_start - z_seed).abs(),
        ((z_start - z_seed) / MICROSTEP_MM).abs(),
    );

    let last_seg = segs.last().unwrap();
    let z_end = z_end_of_segment(last_seg);
    assert!(
        (z_end - (z_seed + Z_HOP_MM)).abs() < 1e-2,
        "terminal Z position {z_end:.6} mm should be {:.4} mm (seed + hop) within 0.01 mm",
        z_seed + Z_HOP_MM,
    );

    h.shutdown();
}

#[test]
fn z_hop_inter_piece_continuity() {
    let (dispatch, recorded) = recording_dispatch();
    let mut cfg = neptune_config();
    cfg.limit_sections = neptune_limit_sections();
    let mut h = PlannerHandle::spawn(cfg, dispatch);

    h.kalico_stream_open([0.0, 0.0, 0.0, 0.0])
        .expect("kalico_stream_open");

    h.submit_move(
        classify_and_build([0.0, 0.0, 0.0], 0.0, 0.0, Z_HOP_MM, &[], Z_HOP_FEEDRATE)
            .expect("classify Z hop"),
    )
    .expect("submit Z hop");

    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    assert!(!segs.is_empty(), "no segments dispatched");

    for (i, seg) in segs.iter().enumerate() {
        let pieces = extract_bezier_pieces(&seg.axes[2]);
        for j in 0..pieces.len().saturating_sub(1) {
            let a = &pieces[j];
            let b = &pieces[j + 1];
            let end_val = a.evaluate(a.u_end);
            let start_val = b.evaluate(b.u_start);
            assert!(
                (end_val - start_val).abs() < 1e-6,
                "segment {i} Z: piece {j} ends at {end_val:.9} mm but piece {} starts at \
                 {start_val:.9} mm — intra-segment Z discontinuity of {:.9} mm",
                j + 1,
                (end_val - start_val).abs(),
            );
        }
    }

    for i in 0..segs.len().saturating_sub(1) {
        let a = &segs[i];
        let b = &segs[i + 1];
        assert!(
            (a.t_end - b.t_start).abs() < 1e-9,
            "seam {i}: t_end {:.12} != next t_start {:.12}",
            a.t_end,
            b.t_start,
        );
        let z_left = z_end_of_segment(a);
        let z_right = z_start_of_segment(b);
        assert!(
            (z_left - z_right).abs() < 1e-4,
            "seam {i}: Z discontinuity of {:.6} mm between segment {i} end ({z_left:.6}) \
             and segment {} start ({z_right:.6})",
            (z_left - z_right).abs(),
            i + 1,
        );
    }

    h.shutdown();
}
