use super::*;

#[test]
fn step_entry_carries_stepper_sel() {
    let entry = StepEntry::pulse(100, 1, 3);
    assert_eq!(core::mem::size_of::<StepEntry>(), 8);
    assert_eq!(entry.cycle_abs, 100);
    assert_eq!(entry.dir(), 1);
    assert_eq!(entry.stepper_sel(), 3);
}

#[test]
fn step_entry_pulse_roundtrips_negative_dir() {
    let entry = StepEntry::pulse(7, -1, 2);
    assert_eq!(entry.dir(), -1);
    assert_eq!(entry.stepper_sel(), 2);
}

#[test]
fn step_entry_xdirect_carries_signed_offset() {
    let entry = StepEntry::xdirect(42, -12345);
    assert_eq!(entry.cycle_abs, 42);
    assert_eq!(entry.offset_steps(), -12345);
}

#[test]
fn clear_empties_queue() {
    let mut q = StepQueue::new();
    q.tail = 5;
    q.head = 2;
    assert_ne!(q.tail, q.head);
    q.clear();
    assert_eq!(q.tail, 0);
    assert_eq!(q.head, 0);
}
