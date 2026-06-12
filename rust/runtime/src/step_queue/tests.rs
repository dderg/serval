use super::*;

#[test]
fn step_entry_carries_stepper_sel() {
    let entry = StepEntry {
        cycle_abs: 100,
        dir: 1,
        stepper_sel: 3,
        _pad: [0; 2],
    };
    assert_eq!(core::mem::size_of::<StepEntry>(), 8);
    assert_eq!(entry.stepper_sel, 3);
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
