//! Where the shaped axes curve into a stop unlike each other, the leaders
//! cover a few micrometres more path than the tail's straight distance from
//! the rest point. Laid at the next move's ratio, that idle arc stepped the
//! ratio while they still moved at several mm/s, and the rest valley opened
//! there: the kernel-free advance stepped the extruder's velocity by up to
//! 0.47 mm/s at a fit join with no unshaped leader near, and the ratio step
//! itself asked for over 100000 mm/s² of a 20000 mm/s² extruder.

mod corpus;

use std::path::PathBuf;

const EXTRUDER_MAX_ACCEL_MM_S2: f64 = 20000.0;

/// The opening lines of each sim print, through its first stops after an
/// extrusion on the kernel-free fast bench config.
const EXCERPTS: [(&str, usize); 2] = [("voron_first_layers", 100), ("voron_dense_top_layers", 200)];

fn kernel_free_fast() -> PathBuf {
    let path = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("rest_spill_steps_kf_fast.cfg");
    std::fs::write(
        &path,
        corpus::kernel_free("tools/bench/configs/fast.cfg", EXTRUDER_MAX_ACCEL_MM_S2),
    )
    .expect("write the kernel-free config");
    path
}

#[test]
fn a_kernel_free_advance_holds_the_extruder_to_its_limits_into_a_curved_stop() {
    let config = kernel_free_fast();
    for (print, lines) in EXCERPTS {
        let gcode: String =
            std::fs::read_to_string(corpus::repo().join(format!("tools/sim/gcode/{print}.gcode")))
                .expect("read the sim print")
                .lines()
                .take(lines)
                .flat_map(|line| [line, "\n"])
                .collect();
        let snapshot = corpus::plan(print, &gcode, &config).expect("the excerpt plans");
        let audit = corpus::extruder::extruder_audit(&snapshot, &gcode, &config)
            .expect("the config limits the extruder");
        let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
        assert!(
            excesses.is_empty(),
            "{print}, first {lines} lines: {excesses:#?}"
        );
    }
}
