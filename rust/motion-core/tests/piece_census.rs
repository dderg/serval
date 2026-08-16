// Offline piece-duration census over a full print replay with the Trident
// chain set (X/Y smooth_mzv, E linear PA + smooth_triangle). Quantifies how
// many wire pieces each axis mints per second and where sub-200 us pieces
// cluster, attributing the dense streams that saturate the pump (#405/#408).
// Gcode path comes from KALICO_REPRO_GCODE; skipped when unset.

use geometry::path::lowering::PositionProfile;
use motion_core::seam_test_harness::{default_stream_config, parse_gcode_to_moves};
use motion_pipeline::{ShapedItem, StreamInput, setup_stages};
use trajectory::ShapedSegment;

const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];
const BUCKETS_US: [f64; 6] = [150.0, 300.0, 600.0, 1200.0, 5000.0, f64::INFINITY];

fn trident_chain_set() -> trajectory::AxisChainSet {
    let sx = trajectory::PostProcessorInstance::new(
        "shaper_x",
        &trajectory::algos::SmoothMzv,
        vec![191.0],
    );
    let sy = trajectory::PostProcessorInstance::new(
        "shaper_y",
        &trajectory::algos::SmoothMzv,
        vec![129.4],
    );
    let pa = trajectory::PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![0.02],
    );
    let st = trajectory::PostProcessorInstance::new(
        "st",
        &trajectory::algos::SmoothTriangle,
        vec![0.013],
    );
    trajectory::AxisChainSet {
        chains: vec![
            trajectory::CompiledChain::compile(&[sx]).expect("x chain composes"),
            trajectory::CompiledChain::compile(&[sy]).expect("y chain composes"),
            trajectory::CompiledChain::default(),
            trajectory::CompiledChain::compile(&[pa, st]).expect("e chain composes"),
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

#[test]
fn piece_duration_census() {
    let Ok(path) = std::env::var("KALICO_REPRO_GCODE") else {
        eprintln!("KALICO_REPRO_GCODE unset — skipping census");
        return;
    };
    let source = std::fs::read_to_string(&path).expect("gcode file readable");

    let mut cfg = default_stream_config();
    let corner_dev = geometry::corner_deviation_from_scv(5.0, 50000.0);
    cfg.limits = geometry::VelocityLimits::try_new(1000.0, 50000.0, corner_dev, f64::INFINITY)
        .expect("trident limits are valid");
    let moves = parse_gcode_to_moves(&source, cfg.limits);
    eprintln!("parsed {} moves from {path}", moves.len());
    assert!(!moves.is_empty());

    let script: Vec<StreamInput> = moves.iter().cloned().map(Into::into).collect();
    let spatial_home = script
        .iter()
        .find_map(|item| match item {
            StreamInput::Move(m) => m.segment.spatial.as_ref(),
            _ => None,
        })
        .map_or([0.0, 0.0, 0.0], |seg| seg.point_at(0.0));
    let mut home = spatial_home.to_vec();
    home.push(0.0);
    let handle = setup_stages(cfg, trident_chain_set(), home, 0.0);
    let output = handle.output;
    let collector = std::thread::spawn(move || {
        let mut segs: Vec<ShapedSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let ShapedItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
    });
    let mut fed = 0usize;
    for item in script {
        if handle.input.send(item).is_err() {
            eprintln!("pipeline stage died after {fed} inputs — censusing what was emitted");
            break;
        }
        fed += 1;
    }
    drop(handle.input);
    let segs = collector.join().expect("collector panicked");
    eprintln!(
        "pipeline emitted {} segments (fed {fed} inputs)",
        segs.len()
    );

    let mut hist = [[0u64; BUCKETS_US.len()]; 4];
    let mut counts = [0u64; 4];
    let mut time_total = [0f64; 4];
    let mut per_window: std::collections::BTreeMap<(u64, usize), (u64, u32)> =
        std::collections::BTreeMap::new();

    for seg in &segs {
        for (axis, curve) in seg.axes.iter().enumerate().take(4) {
            let pieces = nurbs::bezier::extract_bezier_pieces(curve);
            for bp in &pieces {
                let d = bp.u_end - bp.u_start;
                if d <= 0.0 {
                    continue;
                }
                counts[axis] += 1;
                time_total[axis] += d;
                let dus = d * 1e6;
                let b = BUCKETS_US.iter().position(|&hi| dus <= hi).unwrap();
                hist[axis][b] += 1;
                let window = (seg.t_start + bp.u_start).max(0.0) as u64;
                let e = per_window.entry((window, axis)).or_insert((0, 0));
                e.0 += 1;
                e.1 = seg.source_line;
            }
        }
    }

    let stream_end = segs.last().map_or(0.0, |s| s.t_end);
    eprintln!("stream spans {stream_end:.1}s");
    for axis in 0..4 {
        if counts[axis] == 0 {
            continue;
        }
        let avg_us = time_total[axis] / counts[axis] as f64 * 1e6;
        eprint!(
            "axis {} ({}): {} pieces, avg piece {:.0}us, mean rate {:.0}/s | buckets ",
            axis,
            AXIS_NAMES[axis],
            counts[axis],
            avg_us,
            counts[axis] as f64 / stream_end.max(1e-9),
        );
        for (i, &hi) in BUCKETS_US.iter().enumerate() {
            let label = if hi.is_finite() {
                format!("<={:.0}us", hi)
            } else {
                ">5ms".to_string()
            };
            eprint!("{label}:{} ", hist[axis][i]);
        }
        eprintln!();
    }

    let mut worst: Vec<((u64, usize), (u64, u32))> = per_window.into_iter().collect();
    worst.sort_by(|a, b| b.1.0.cmp(&a.1.0));
    eprintln!("worst 1s windows (pieces/s per axis):");
    for ((t, axis), (n, line)) in worst.iter().take(16) {
        eprintln!(
            "  t={t}s axis={} pieces={n} (last gcode line {line})",
            AXIS_NAMES[*axis]
        );
    }
    let total: u64 = counts.iter().sum();
    eprintln!(
        "TOTAL pieces {total} over {stream_end:.1}s = {:.0} pieces/s mean",
        total as f64 / stream_end.max(1e-9)
    );
}
