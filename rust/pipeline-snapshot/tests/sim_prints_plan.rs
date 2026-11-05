//! The sim prints plan through the snapshot path exactly as the live path
//! accepts them: a repeated `G1 Z.2` is no move, not a zero-length one.

use std::path::Path;

mod corpus;

#[test]
fn trident_mesh_clock_regression_plans_on_both_configs() {
    let print = corpus::sim_cases(None)
        .into_iter()
        .filter(|(name, _, _)| name.starts_with("trident_mesh_clock_regression/"))
        .collect::<Vec<_>>();
    assert_eq!(print.len(), 2);
    for (name, gcode, config) in print {
        let snapshot = corpus::plan(&name, &gcode, Path::new(&config))
            .unwrap_or_else(|| panic!("{name}: fewer than two waypoints"));
        assert!(snapshot.traversal_time_s > 0.0, "{name}");
    }
}
