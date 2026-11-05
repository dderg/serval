use std::env;
use std::fs;

use motion_core::seam_test_harness::LiveReplay;
use motion_pipeline::TrajectoryItem;
use trajectory::ContinuousSegment;

fn main() {
    let args: Vec<String> = env::args().collect();
    let source = fs::read_to_string(&args[1]).unwrap();
    let text = fs::read_to_string(&args[2]).unwrap();
    let v_ref: f64 = args.get(3).map_or(50.0, |v| v.parse().unwrap());
    let planner = planner_config::from_doc::planner_config_from_text(&text).unwrap();
    let replay = LiveReplay::new(&source, &planner, None).unwrap();
    let mut segs: Vec<ContinuousSegment> = Vec::new();
    let mut pipeline = replay.pipeline();
    let mut sink = |item| {
        if let TrajectoryItem::Seg(seg) = item {
            segs.push(seg);
        }
        true
    };
    for m in replay.moves {
        assert!(pipeline.feed(m.into(), &mut sink));
    }
    assert!(pipeline.finish(&mut sink));
    let mut samples: Vec<(f64, f64, f64, f64)> = Vec::new();
    for seg in &segs {
        let n = 64;
        for k in 0..n {
            let t = seg.t_start + (seg.t_end - seg.t_start) * k as f64 / n as f64;
            let p = |a: usize| seg.axes[a].eval_pva(t).unwrap();
            let (x, y, z) = (p(0), p(1), p(2));
            samples.push((t, x.velocity.hypot(y.velocity), z.velocity, z.position));
        }
    }
    let mut i = 0;
    let (mut total_z, mut total_w, mut count) = (0.0, 0.0, 0);
    while i < samples.len() {
        if samples[i].2.abs() < 1e-6 {
            i += 1;
            continue;
        }
        let a = i;
        while i < samples.len() && samples[i].2.abs() >= 1e-6 {
            i += 1;
        }
        let b = i.min(samples.len() - 1);
        let mut lo = a;
        while lo > 0 && samples[lo].1 < v_ref {
            lo -= 1;
        }
        let mut hi = b;
        while hi + 1 < samples.len() && samples[hi].1 < v_ref {
            hi += 1;
        }
        let dz = samples[b].3 - samples[a].3;
        let (zd, w) = (samples[b].0 - samples[a].0, samples[hi].0 - samples[lo].0);
        total_z += zd;
        total_w += w;
        count += 1;
        println!(
            "z t={:.4} dz={dz:.3} z_dur={:.1}ms window(xy<{v_ref})={:.1}ms xy_at_z_start={:.1} xy_at_z_end={:.1}",
            samples[a].0,
            zd * 1e3,
            w * 1e3,
            samples[a].1,
            samples[b].1
        );
    }
    println!(
        "total z moves={count} z_dur={total_z:.3}s window={total_w:.3}s stream={:.3}s",
        segs.last().unwrap().t_end
    );
}
