// Repro for the post-shaper resolution loss on the PA path: the snapshot
// playground's smooth_pressure_advance/extruding_corner scenario, run with
// KALICO_FIT_DEBUG=1 so every fit_axis_from_signal call reports its dense
// deviation from the truth signal it was fitting.
use motion_core::seam_test_harness::{
    collect_shaped_segments_scripted, default_stream_config, parse_gcode_to_moves,
};

fn snapshot_chain_set() -> trajectory::AxisChainSet {
    let xy_shaper = || {
        trajectory::PostProcessorInstance::new(
            "shaper",
            &trajectory::algos::SmoothBell,
            vec![0.02390625],
        )
    };
    let pa = trajectory::PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![0.04],
    );
    let st_e = trajectory::PostProcessorInstance::new(
        "st_e",
        &trajectory::algos::SmoothBell,
        vec![0.02675],
    );
    let xy = || trajectory::CompiledChain::compile(&[xy_shaper()]).expect("shaper compiles");
    let e_chain =
        trajectory::CompiledChain::compile(&[pa, st_e]).expect("pa + smooth_bell composes");
    trajectory::AxisChainSet {
        chains: vec![xy(), xy(), trajectory::CompiledChain::default(), e_chain],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

#[test]
fn pa_extruding_corner_fit_fidelity() {
    let gcode = "G90\nM83\nG1 X0 Y0 F9000\nG1 X40 Y0 E2\nG1 X40 Y40 E2\n";
    let mut cfg = default_stream_config();
    cfg.limits = geometry::VelocityLimits::try_new(300.0, 3000.0, 5.0, f64::INFINITY)
        .expect("printer limits are valid");
    let moves = parse_gcode_to_moves(gcode, cfg.limits);
    assert!(!moves.is_empty());
    let segs = collect_shaped_segments_scripted(&moves, cfg, snapshot_chain_set(), None);
    eprintln!("pipeline emitted {} segments", segs.len());
    assert!(!segs.is_empty());
    let (mut v_max, mut v_min) = (f64::NEG_INFINITY, f64::INFINITY);
    for seg in &segs {
        let e = &seg.axes[3];
        let (lo, hi) = (seg.t_start, seg.t_end);
        for k in 0..=400 {
            let t = lo + (hi - lo) * (k as f64) / 400.0;
            let v = nurbs::eval::eval_derivative(e.control_points(), e.knots(), e.degree(), t);
            v_max = v_max.max(v);
            v_min = v_min.min(v);
        }
    }
    eprintln!("shaped E velocity: max={v_max:.3} min={v_min:.3} (feed 7.5, k*a = 6.0)");
    if let Ok(path) = std::env::var("KALICO_FIT_CSV") {
        let mut csv = String::new();
        for seg in &segs {
            let e = &seg.axes[3];
            let n = 2000;
            for k in 0..n {
                let t = seg.t_start + (seg.t_end - seg.t_start) * (k as f64) / (n as f64);
                let p = nurbs::eval::eval_polynomial(e.control_points(), e.knots(), e.degree(), t);
                let v = nurbs::eval::eval_derivative(e.control_points(), e.knots(), e.degree(), t);
                csv.push_str(&format!("{t:.7},{p:.9},{v:.6}\n"));
            }
        }
        std::fs::write(path, csv).expect("csv written");
    }
    assert!(
        v_max > 10.0,
        "advance term missing: peak shaped E velocity {v_max:.3} ~ bare feed"
    );
}
