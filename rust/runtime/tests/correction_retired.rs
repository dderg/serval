use runtime::engine::Engine;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

#[test]
fn correction_retired_counts_reads_correction_ring() {
    let mut e = Engine::new(520_000_000, 40_000);
    let b = pulse_binding();
    assert_eq!(
        e.configure_axis(0, StepMode::Pulse, 0.0125, 64, &[b], 512),
        0
    );
    let counts = e.correction_retired_counts();
    assert_eq!(counts[0], 0);
}

#[test]
fn correction_retired_counts_all_zero_when_unconfigured() {
    let e = Engine::new(520_000_000, 40_000);
    assert_eq!(e.correction_retired_counts(), [0u32; MAX_AXES]);
}
