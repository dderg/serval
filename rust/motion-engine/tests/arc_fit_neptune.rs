use _motion_engine::seam_harness::{default_stream_config, parse_gcode_to_moves};
use geometry::path::CurvatureProfile;
use geometry::path::Segment;
use geometry::{ChainFitConfig, fit_chain};

const NEPTUNE: &str = include_str!("gcode/neptune_crash_short.gcode");

struct ArcCensus {
    total_arc_mm: f64,
    max_arc_mm: f64,
    arcs: u32,
    blended: u32,
}

fn census_with_arc_fit() -> ArcCensus {
    let limits = default_stream_config().limits;
    let moves = parse_gcode_to_moves(NEPTUNE, limits);
    let outcome = fit_chain(&moves, ChainFitConfig::with_arc_fit(0.05, 3))
        .expect("neptune fixture fits without a geometry error");

    let mut total_arc_mm = 0.0;
    let mut max_arc_mm = 0.0;
    let mut arcs = 0;
    for m in &outcome.moves {
        if let Some(Segment::Arc(a)) = &m.segment.spatial {
            let s = a.s_len();
            total_arc_mm += s;
            max_arc_mm = f64::max(max_arc_mm, s);
            arcs += 1;
        }
    }
    ArcCensus {
        total_arc_mm,
        max_arc_mm,
        arcs,
        blended: outcome.report.blended,
    }
}

#[test]
fn arc_fit_recognizes_the_neptune_circle() {
    let c = census_with_arc_fit();
    assert!(
        c.max_arc_mm >= 8.0,
        "the ~13mm center circle must be fit as a single arc, not shattered: \
         largest arc was only {:.2}mm across {} arcs (total arc {:.2}mm), \
         with {} junctions biclothoid-blended into a forest instead",
        c.max_arc_mm,
        c.arcs,
        c.total_arc_mm,
        c.blended,
    );
}

#[test]
fn arc_fit_does_not_shatter_curves_into_a_blend_forest() {
    let c = census_with_arc_fit();
    assert!(
        c.blended <= 40,
        "curved features must be consumed into arc runs, not blended per-junction: \
         {} junctions were biclothoid-blended (arcs found: {}, total arc {:.2}mm)",
        c.blended,
        c.arcs,
        c.total_arc_mm,
    );
}
