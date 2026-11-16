use super::*;
use crate::host_io::runtime_events::FaultEvent as RuntimeFaultEvent;

fn make_event(fault_code: u16, synthesized: bool) -> RuntimeFaultEvent {
    RuntimeFaultEvent {
        fault_code,
        fault_detail: 0,
        segment_id: 0,
        synthesized,
    }
}

#[test]
fn dispatch_latches_first_event() {
    let mut latch = FaultLatch::default();
    latch.dispatch(make_event(17, false));
    assert_eq!(latch.cell.as_ref().unwrap().fault_code, 17);
}

#[test]
fn dispatch_does_not_overwrite_real_with_real() {
    let mut latch = FaultLatch::default();
    latch.dispatch(make_event(1, false));
    latch.dispatch(make_event(2, false));
    assert_eq!(latch.cell.as_ref().unwrap().fault_code, 1);
}

#[test]
fn dispatch_upgrades_synthesized_with_edge() {
    let mut latch = FaultLatch::default();
    latch.dispatch(make_event(99, true)); // synthesized
    latch.dispatch(make_event(42, false)); // real MCU edge
    assert_eq!(latch.cell.as_ref().unwrap().fault_code, 42);
    assert!(!latch.cell.as_ref().unwrap().synthesized);
}
