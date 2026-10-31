use super::compose_msg;

#[test]
fn compose_msg_substitutes_both_args() {
    let msg = compose_msg("got={arg0} limit={arg1}", 5, 10);
    assert_eq!(msg, "got=5 limit=10");
}

#[test]
fn compose_msg_substitutes_arg0_only() {
    let msg = compose_msg("engine reset epoch={arg0}", 7, 0);
    assert_eq!(msg, "engine reset epoch=7");
}

#[test]
fn compose_msg_no_placeholders() {
    let msg = compose_msg("engine reset", 0, 0);
    assert_eq!(msg, "engine reset");
}

#[test]
fn compose_msg_i32_renders_negative() {
    let msg = compose_msg(
        "start-now={arg0:i32}ms end-now={arg1:i32}ms",
        (-2000i32) as u32,
        (-1i32) as u32,
    );
    assert_eq!(msg, "start-now=-2000ms end-now=-1ms");
}

#[test]
fn compose_msg_hex_renders_with_prefix() {
    let msg = compose_msg("pc={arg0:hex} lr={arg1:hex}", 0x0800_1234, 0xDEAD_BEEF);
    assert_eq!(msg, "pc=0x8001234 lr=0xdeadbeef");
}

#[test]
fn compose_msg_hi16_lo16_split_packed_field() {
    let packed = (7u32 << 16) | 42u32;
    let msg = compose_msg("axis={arg0:hi16} occupancy={arg0:lo16}", packed, 0);
    assert_eq!(msg, "axis=7 occupancy=42");
}

#[test]
fn compose_msg_mixes_typed_and_plain_placeholders() {
    let msg = compose_msg(
        "func={arg0:hex} dur_cyc={arg1} plain={arg0}",
        0x2000_0100,
        99,
    );
    assert_eq!(msg, "func=0x20000100 dur_cyc=99 plain=536871168");
}
