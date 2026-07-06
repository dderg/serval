use super::require_events_dir_for_mcu_transport;
use std::path::Path;

#[test]
fn non_native_no_events_dir_is_ok() {
    assert!(
        require_events_dir_for_mcu_transport(false, None, "mcu-stock").is_ok(),
        "non-native MCU must not require events_dir"
    );
}

#[test]
fn non_native_with_events_dir_is_ok() {
    assert!(
        require_events_dir_for_mcu_transport(
            false,
            Some(Path::new("/tmp/kalico-events")),
            "mcu-stock",
        )
        .is_ok(),
        "non-native MCU must be Ok regardless of events_dir"
    );
}

#[test]
fn native_with_events_dir_is_ok() {
    assert!(
        require_events_dir_for_mcu_transport(
            true,
            Some(Path::new("/tmp/kalico-events")),
            "mcu-h7",
        )
        .is_ok(),
        "native MCU must be Ok when events_dir is set"
    );
}

#[test]
fn native_no_events_dir_is_err_containing_label() {
    let result = require_events_dir_for_mcu_transport(true, None, "mcu-h7");
    assert!(
        result.is_err(),
        "native MCU without events_dir must return Err"
    );
    let msg = result.unwrap_err();
    assert!(
        msg.contains("mcu-h7"),
        "error message must contain the MCU label; got: {msg}"
    );
    assert!(
        msg.contains("init_logging"),
        "error message must mention init_logging; got: {msg}"
    );
}

#[test]
fn native_no_events_dir_err_mentions_mculog_discard() {
    let result = require_events_dir_for_mcu_transport(true, None, "mcu-f4");
    let msg = result.unwrap_err();
    assert!(
        msg.contains("McuLog") || msg.contains("discarded"),
        "error message must explain McuLog discard; got: {msg}"
    );
}
