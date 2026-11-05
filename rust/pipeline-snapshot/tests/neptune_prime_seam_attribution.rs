//! The print-start prime of the dense neptune print brakes its toolhead
//! from a triangle velocity peak, and the kernel-free advance steps the
//! extruder velocity by k·r·Δa there. The audit attributes that step to the
//! leader's acceleration step only when the leader's breakpoint sits where
//! its evaluator switches phase.

mod corpus;

const DENSE_PRINT: &str = "tools/sim/gcode/voron_dense_top_layers.gcode";
const NEPTUNE: &str = "rust/motion-core/tests/configs/neptune_sim_print.cfg";
const PRINT_START_LINES: usize = 60;

#[test]
fn the_print_start_prime_velocity_step_is_attributed_to_the_leader_seam() {
    let print = std::fs::read_to_string(corpus::repo().join(DENSE_PRINT))
        .expect("the dense print is in the repo");
    let gcode: String = print
        .lines()
        .take(PRINT_START_LINES)
        .flat_map(|line| [line, "\n"])
        .collect();
    let config = corpus::repo().join(NEPTUNE);
    let snapshot =
        corpus::plan("neptune print start", &gcode, &config).expect("the print start plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, &gcode, &config)
        .expect("neptune limits its extruder");
    assert!(
        !audit.exempt_steps.is_empty(),
        "the prime must step e velocity within the advance allowance"
    );
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(
        excesses.is_empty(),
        "{} extruder excesses: {}",
        excesses.len(),
        excesses.join("; ")
    );
}
