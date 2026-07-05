use super::*;

#[test]
fn open_by_default() {
    let halt = StreamHalt::default();
    assert!(!halt.is_halted());
    assert_eq!(halt.check_push_allowed(), Ok(()));
}

#[test]
fn halt_rejects_pushes_until_resume() {
    let mut halt = StreamHalt::default();
    halt.halt();
    assert!(halt.is_halted());
    assert_eq!(halt.check_push_allowed(), Err(ERR_PIECES_WHILE_HALTED));
    assert_eq!(halt.resume(), Ok(()));
    assert_eq!(halt.check_push_allowed(), Ok(()));
}

#[test]
fn halt_is_idempotent() {
    let mut halt = StreamHalt::default();
    halt.halt();
    halt.halt();
    assert_eq!(halt.check_push_allowed(), Err(ERR_PIECES_WHILE_HALTED));
    assert_eq!(halt.resume(), Ok(()));
}

#[test]
fn resume_without_halt_is_a_state_violation() {
    let mut halt = StreamHalt::default();
    assert_eq!(halt.resume(), Err(ERR_RESUME_STREAM_NOT_HALTED));
    halt.halt();
    assert_eq!(halt.resume(), Ok(()));
    assert_eq!(halt.resume(), Err(ERR_RESUME_STREAM_NOT_HALTED));
}
