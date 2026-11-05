//! `tools/corner_flow.py`'s 45° corner at 150 mm/s on `trident_kernel.cfg`,
//! planned without the extruder's pressure advance as `corner_flow` plans
//! its reference run. The leaders leave rest under the bell kernel, so for
//! the first milliseconds the path creeps a few attometres while the ratio
//! ramp stacks its rest's width nodes where the width leaves zero. No seam's
//! window reaches that stretch, so the laid correction is zero on both sides
//! of the stacked nodes; the follower's enclosure read a reach across them
//! as unprovable instead, and the shaper refused the stream's first piece.

use std::path::PathBuf;

mod corpus;

const CORNER_45_AT_150_MM_S: &str = "G21
G90
M83
G1 X0 Y0 F9000
G1 X60.00000 Y0.00000 E1.995608
G1 X17.57359 Y42.42641 E1.995608
";

fn trident_kernel_without_advance() -> PathBuf {
    let config =
        std::fs::read_to_string(corpus::repo().join("tools/bench/configs/trident_kernel.cfg"))
            .expect("read trident_kernel.cfg");
    let with_advance = "post_processors = pa, st";
    assert_eq!(
        config.matches(with_advance).count(),
        1,
        "trident_kernel.cfg no longer chains its extruder as `{with_advance}`"
    );
    let path = std::env::temp_dir().join(format!(
        "trident_kernel_without_advance_{}.cfg",
        std::process::id()
    ));
    std::fs::write(&path, config.replace(with_advance, "post_processors = st"))
        .expect("write the config without the advance");
    path
}

#[test]
fn a_corner_leaving_rest_under_the_kernel_plans_without_its_advance() {
    let config = trident_kernel_without_advance();
    corpus::plan("kernel_start_from_rest", CORNER_45_AT_150_MM_S, &config)
        .expect("the corner plans");
}
