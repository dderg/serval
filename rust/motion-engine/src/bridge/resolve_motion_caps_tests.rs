use super::resolve_motion_caps;
use crate::dispatch::McuCaps;
use mcu_protocol::messages::RuntimeCapsResponse;

#[test]
fn some_caps_returns_ok_with_correct_value() {
    let caps = Some(RuntimeCapsResponse {
        total_piece_memory: 62 * 1024,
    });
    let result = resolve_motion_caps(caps, "octopus", 1);
    assert_eq!(
        result,
        Ok(McuCaps {
            total_piece_memory: 62 * 1024
        })
    );
}

#[test]
fn none_caps_returns_err_containing_label_and_handle() {
    let result = resolve_motion_caps(None, "f446", 7);
    assert!(result.is_err(), "expected Err for None caps");
    let msg = result.unwrap_err();
    assert!(
        msg.contains("f446"),
        "error message should contain the MCU label; got: {msg}"
    );
    assert!(
        msg.contains('7'),
        "error message should contain the handle; got: {msg}"
    );
}
