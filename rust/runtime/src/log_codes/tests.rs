use super::*;

#[test]
fn subsystem_name_known() {
    assert_eq!(subsystem_name(SUBSYSTEM_RUNTIME), "runtime");
    assert_eq!(subsystem_name(SUBSYSTEM_MOTION), "motion");
    assert_eq!(subsystem_name(SUBSYSTEM_TICK), "tick");
    assert_eq!(subsystem_name(SUBSYSTEM_ENDSTOP), "endstop");
}

#[test]
fn subsystem_name_unknown_returns_unknown() {
    assert_eq!(subsystem_name(0xFF), "unknown");
    assert_eq!(subsystem_name(5), "unknown");
    assert_eq!(subsystem_name(100), "unknown");
}

#[test]
fn mcu_ready_resolves() {
    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_MCU_READY);
    assert_eq!(name, "runtime.mcu_ready");
    assert_eq!(tmpl, "mcu firmware ready, log drain online");
}

#[test]
fn event_info_all_runtime_events() {
    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FAULT_LATCHED);
    assert_eq!(name, "runtime.fault_latched");
    assert!(tmpl.contains("{arg0}"), "template must reference {{arg0}}");
    assert!(
        !tmpl.contains("{arg1}"),
        "template must NOT reference {{arg1}} — fault identity moved to code field"
    );

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_ENGINE_RESET);
    assert_eq!(name, "runtime.engine_reset");
    assert!(tmpl.contains("{arg0}"));

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_LOG_DROPS);
    assert_eq!(name, "runtime.log_drops");
    assert!(
        tmpl.contains("{arg0}"),
        "log_drops template must reference {{arg0}}"
    );

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_MCU_RESET);
    assert_eq!(name, "runtime.mcu_reset");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_HARD_FAULT);
    assert_eq!(name, "runtime.hard_fault");
    assert!(tmpl.contains("{arg0:hex}") && tmpl.contains("{arg1:hex}"));

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FAULT_STATUS);
    assert_eq!(name, "runtime.fault_status");
    assert!(tmpl.contains("{arg0:hex}") && tmpl.contains("{arg1:hex}"));

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FG_FREEZE);
    assert_eq!(name, "runtime.fg_freeze");
    assert!(tmpl.contains("{arg0:hex}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_RT_PROGRESS);
    assert_eq!(name, "runtime.rt_progress");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));
}

#[test]
fn event_info_all_motion_events() {
    let (name, tmpl) = event_info(SUBSYSTEM_MOTION, EVENT_MOTION_PIECE_START_PAST);
    assert_eq!(name, "motion.piece_start_past");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_MOTION, EVENT_MOTION_RING_FULL);
    assert_eq!(name, "motion.ring_full");
    assert!(tmpl.contains("{arg0}"));
}

#[test]
fn event_info_tick_interval_exceeded() {
    let (name, tmpl) = event_info(SUBSYSTEM_TICK, EVENT_TICK_INTERVAL_EXCEEDED);
    assert_eq!(name, "tick.interval_exceeded");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));
}

#[test]
fn event_info_all_tick_events() {
    let (name, tmpl) = event_info(SUBSYSTEM_TICK, EVENT_TICK_UNDERRUN);
    assert_eq!(name, "tick.underrun");
    assert!(tmpl.contains("{arg0}"));
}

#[test]
fn event_info_all_endstop_events() {
    let (name, tmpl) = event_info(SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRIP);
    assert_eq!(name, "endstop.trip");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_ARM_TIMEOUT);
    assert_eq!(name, "endstop.arm_timeout");
    assert!(tmpl.contains("{arg0}"));

    let (name, tmpl) = event_info(SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRSYNC_TRIGGER_CMD);
    assert_eq!(name, "endstop.trsync_trigger_cmd");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRSYNC_DO_TRIGGER);
    assert_eq!(name, "endstop.trsync_do_trigger");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_STOP_CB_ENTER);
    assert_eq!(name, "endstop.stop_cb_enter");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_SOFTWARE_TRIP);
    assert_eq!(name, "endstop.software_trip");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));
}

#[test]
fn event_info_unknown_pair() {
    let (name, tmpl) = event_info(0xFF, 0x7FFF);
    assert_eq!(name, "unknown");
    assert_eq!(tmpl, "");
}

#[test]
fn event_info_zero_event_is_unknown() {
    let (name, _tmpl) = event_info(SUBSYSTEM_TICK, 0);
    assert_eq!(name, "unknown");
}

#[test]
fn event_info_wrong_subsystem_returns_unknown() {
    let (name, _) = event_info(SUBSYSTEM_TICK, 99);
    assert_eq!(name, "unknown");
}

#[test]
fn subsystem_name_diag() {
    assert_eq!(subsystem_name(SUBSYSTEM_DIAG), "diag");
    let diag = subsystem_name(SUBSYSTEM_DIAG);
    assert_ne!(diag, subsystem_name(SUBSYSTEM_RUNTIME));
    assert_ne!(diag, subsystem_name(SUBSYSTEM_MOTION));
    assert_ne!(diag, subsystem_name(SUBSYSTEM_TICK));
    assert_ne!(diag, subsystem_name(SUBSYSTEM_ENDSTOP));
}

#[test]
fn event_info_all_diag_events() {
    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, EVENT_DIAG_TIM5_LONG);
    assert_eq!(name, "diag.tim5_long");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, EVENT_DIAG_OTG_LONG);
    assert_eq!(name, "diag.otg_long");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, EVENT_DIAG_USB_OUT_GAP);
    assert_eq!(name, "diag.usb_out_gap");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, EVENT_DIAG_USB_IN_GAP);
    assert_eq!(name, "diag.usb_in_gap");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, EVENT_DIAG_TX_DROP_KAL);
    assert_eq!(name, "diag.tx_drop_kalico");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, EVENT_DIAG_TX_DROP_KLP);
    assert_eq!(name, "diag.tx_drop_klipper");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, EVENT_DIAG_ENGINE_XITION);
    assert_eq!(name, "diag.engine_xition");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, EVENT_DIAG_RUST_FAULT);
    assert_eq!(name, "diag.rust_fault");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));
}

#[test]
fn event_info_names_the_idle_stepper_rearm_events() {
    let (name, tmpl) = event_info(SUBSYSTEM_MOTION, EVENT_MOTION_STEP_REARM);
    assert_eq!(name, "motion.step_rearm");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1:i32}"));

    let (name, tmpl) = event_info(SUBSYSTEM_MOTION, EVENT_MOTION_STEP_REARM_TIGHT);
    assert_eq!(name, "motion.step_rearm_tight");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_MOTION, EVENT_MOTION_STEP_REARM_LATE);
    assert_eq!(name, "motion.step_rearm_late");
    assert!(tmpl.contains("{arg0:i32}") && tmpl.contains("{arg1}"));
}

#[test]
fn event_info_diag_unknown_boundaries() {
    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, 0);
    assert_eq!(name, "unknown");
    assert_eq!(tmpl, "");

    let (name, tmpl) = event_info(SUBSYSTEM_DIAG, 99);
    assert_eq!(name, "unknown");
    assert_eq!(tmpl, "");
}

#[test]
fn event_info_new_runtime_crash_discriminators() {
    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_LAST_DISPATCH);
    assert_eq!(name, "runtime.last_dispatch");
    assert!(tmpl.contains("{arg0:hex}") && tmpl.contains("{arg1:hex}"));

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_ISR_PHASE);
    assert_eq!(name, "runtime.isr_phase");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_BLOCK_SOURCE);
    assert_eq!(name, "runtime.block_source");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_TIM5_IA);
    assert_eq!(name, "runtime.tim5_ia");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));

    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_DIAG_DUMP);
    assert_eq!(name, "runtime.diag_dump");
    assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_substitutes_both_args() {
    let msg = compose_msg("got={arg0} limit={arg1}", 5, 10);
    assert_eq!(msg, "got=5 limit=10");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_substitutes_arg0_only() {
    let msg = compose_msg("engine reset epoch={arg0}", 7, 0);
    assert_eq!(msg, "engine reset epoch=7");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_no_placeholders() {
    let msg = compose_msg("engine reset", 0, 0);
    assert_eq!(msg, "engine reset");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_real_tick_template() {
    let (_name, tmpl) = event_info(SUBSYSTEM_TICK, EVENT_TICK_INTERVAL_EXCEEDED);
    let msg = compose_msg(tmpl, 120, 100);
    assert_eq!(msg, "TIM5 inter-arrival exceeded: got=120 limit=100");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_zero_args() {
    let msg = compose_msg("code={arg0} detail={arg1}", 0, 0);
    assert_eq!(msg, "code=0 detail=0");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_i32_renders_negative() {
    let msg = compose_msg(
        "start-now={arg0:i32}ms end-now={arg1:i32}ms",
        (-2000i32) as u32,
        (-1i32) as u32,
    );
    assert_eq!(msg, "start-now=-2000ms end-now=-1ms");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_i32_renders_positive_same_as_plain() {
    let msg = compose_msg("delta={arg0:i32}", 42, 0);
    assert_eq!(msg, "delta=42");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_hex_renders_with_prefix() {
    let msg = compose_msg("pc={arg0:hex} lr={arg1:hex}", 0x0800_1234, 0xDEAD_BEEF);
    assert_eq!(msg, "pc=0x8001234 lr=0xdeadbeef");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_hex_zero() {
    let msg = compose_msg("pc={arg0:hex}", 0, 0);
    assert_eq!(msg, "pc=0x0");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_hi16_lo16_split_packed_field() {
    let packed = (7u32 << 16) | 42u32;
    let msg = compose_msg("axis={arg0:hi16} occupancy={arg0:lo16}", packed, 0);
    assert_eq!(msg, "axis=7 occupancy=42");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_mixes_typed_and_plain_placeholders() {
    let msg = compose_msg(
        "func={arg0:hex} dur_cyc={arg1} plain={arg0}",
        0x2000_0100,
        99,
    );
    assert_eq!(msg, "func=0x20000100 dur_cyc=99 plain=536871168");
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_axis_stalled_template() {
    let (name, tmpl) = event_info(SUBSYSTEM_MOTION, EVENT_MOTION_AXIS_STALLED);
    assert_eq!(name, "motion.axis_stalled");
    let packed = (2u32 << 16) | 3u32;
    let msg = compose_msg(tmpl, packed, 500);
    assert_eq!(
        msg,
        "axis retirement stalled with pieces pending axis=2 occupancy=3 stalled_ms=500"
    );
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_axis_stalled_head_template_renders_signed_ms() {
    let (name, tmpl) = event_info(SUBSYSTEM_MOTION, EVENT_MOTION_AXIS_STALLED_HEAD);
    assert_eq!(name, "motion.axis_stalled_head");
    let msg = compose_msg(tmpl, (-2000i32) as u32, 500);
    assert_eq!(
        msg,
        "stalled axis armed piece window vs now start-now=-2000ms end-now=500ms"
    );
}

#[cfg(feature = "host")]
#[test]
fn compose_msg_hard_fault_template_renders_hex() {
    let (name, tmpl) = event_info(SUBSYSTEM_RUNTIME, EVENT_RUNTIME_HARD_FAULT);
    assert_eq!(name, "runtime.hard_fault");
    let msg = compose_msg(tmpl, 0x0800_1234, 0x2000_5678);
    assert_eq!(msg, "cpu hard fault pc=0x8001234 lr=0x20005678");
}
