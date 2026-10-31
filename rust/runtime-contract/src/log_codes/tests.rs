use super::*;

#[test]
fn event_info_unknown_pair() {
    let (name, tmpl) = event_info(0xFF, 0x7FFF);
    assert_eq!(name, "unknown");
    assert_eq!(tmpl, "");
}

#[test]
fn event_info_wrong_subsystem_returns_unknown() {
    let (name, _) = event_info(SUBSYSTEM_ENDSTOP, 99);
    assert_eq!(name, "unknown");
}
