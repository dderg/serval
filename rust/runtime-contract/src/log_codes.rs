// Subsystem and event code tables for the MCU structured-log endpoint.
//
// Subsystem IDs and event codes are wire-stable u8/u16 discriminants.
// Names and templates are resolved host-side from these tables.
// Compiles for both `no_std` MCU targets and the host.
//
// WIRE-STABLE: do not renumber existing codes. New events append.

#![allow(dead_code)]

pub const SUBSYSTEM_RUNTIME: u8 = 0;
pub const SUBSYSTEM_MOTION: u8 = 1;
pub const SUBSYSTEM_ENDSTOP: u8 = 3;
pub const SUBSYSTEM_DIAG: u8 = 4;

/// Resolve a subsystem id to its `&'static str` name.
///
/// # Examples
///
/// ```
/// use runtime_contract::log_codes::{subsystem_name, SUBSYSTEM_RUNTIME};
/// assert_eq!(subsystem_name(SUBSYSTEM_RUNTIME), "runtime");
/// assert_eq!(subsystem_name(0xFF), "unknown");
/// ```
pub fn subsystem_name(id: u8) -> &'static str {
    match id {
        SUBSYSTEM_RUNTIME => "runtime",
        SUBSYSTEM_MOTION => "motion",
        SUBSYSTEM_ENDSTOP => "endstop",
        SUBSYSTEM_DIAG => "diag",
        _ => "unknown",
    }
}

// Convention: EVENT_<SUBSYSTEM>_<NAME>. Codes are unique within each subsystem
// but may repeat across subsystems — the (subsystem, event) pair is the key.
// Start at 1; 0 is reserved as "no event".

pub const EVENT_RUNTIME_FAULT_LATCHED: u16 = 1;
pub const EVENT_RUNTIME_MCU_READY: u16 = 3;
pub const EVENT_RUNTIME_LOG_DROPS: u16 = 4;
pub const EVENT_RUNTIME_MCU_RESET: u16 = 5;
pub const EVENT_RUNTIME_HARD_FAULT: u16 = 6;
pub const EVENT_RUNTIME_FAULT_STATUS: u16 = 7;
pub const EVENT_RUNTIME_FG_FREEZE: u16 = 8;
pub const EVENT_RUNTIME_RT_PROGRESS: u16 = 9;
pub const EVENT_RUNTIME_LAST_DISPATCH: u16 = 10;
pub const EVENT_RUNTIME_ISR_PHASE: u16 = 11;
pub const EVENT_RUNTIME_BLOCK_SOURCE: u16 = 12;
pub const EVENT_RUNTIME_TIM5_IA: u16 = 13;
pub const EVENT_RUNTIME_DIAG_DUMP: u16 = 14;
pub const EVENT_RUNTIME_FG_TASK: u16 = 17;
pub const EVENT_RUNTIME_FG_MSG: u16 = 18;
pub const EVENT_RUNTIME_FG_DEMUX: u16 = 19;
pub const EVENT_RUNTIME_FG_MSG_HEAD: u16 = 20;
pub const EVENT_RUNTIME_TIMER_TOO_CLOSE: u16 = 21;
pub const EVENT_RUNTIME_TIMER_TOO_CLOSE_LATE: u16 = 22;
pub const EVENT_RUNTIME_PRIOR_RUN: u16 = 23;
pub const EVENT_RUNTIME_PRIOR_LIVE: u16 = 24;
pub const EVENT_RUNTIME_PRIOR_USB_OUT: u16 = 25;
pub const EVENT_RUNTIME_PRIOR_TASK_GAPS: u16 = 26;

pub const EVENT_MOTION_AXIS_STALLED: u16 = 3;
pub const EVENT_MOTION_AXIS_STALLED_HEAD: u16 = 4;
pub const EVENT_MOTION_STEP_LOAD_LATE: u16 = 5;
pub const EVENT_MOTION_STEP_REARM: u16 = 6;
pub const EVENT_MOTION_STEP_REARM_TIGHT: u16 = 7;
pub const EVENT_MOTION_STEP_REARM_LATE: u16 = 8;
pub const EVENT_MOTION_STEP_HALT: u16 = 9;
pub const EVENT_MOTION_STEP_CLOCK_HORIZON: u16 = 10;
pub const EVENT_MOTION_WIRE_PROBE_LATE: u16 = 11;

pub const EVENT_ENDSTOP_TRSYNC_TRIGGER_CMD: u16 = 3;
pub const EVENT_ENDSTOP_TRSYNC_DO_TRIGGER: u16 = 4;

// diag subsystem events (codes mirror MCU DIAG_EV_* tag values 1..=8)
pub const EVENT_DIAG_TIM5_LONG: u16 = 1;
pub const EVENT_DIAG_OTG_LONG: u16 = 2;
pub const EVENT_DIAG_USB_OUT_GAP: u16 = 3;
pub const EVENT_DIAG_USB_IN_GAP: u16 = 4;
pub const EVENT_DIAG_TX_DROP_KAL: u16 = 5;
pub const EVENT_DIAG_TX_DROP_KLP: u16 = 6;
pub const EVENT_DIAG_ENGINE_XITION: u16 = 7;
pub const EVENT_DIAG_RUST_FAULT: u16 = 8;

/// Resolve a `(subsystem, event)` pair to a `(name, template)` tuple.
///
/// `name` is the stable event key (e.g. `"tick.interval_exceeded"`).
/// `template` substitutes `{arg0}` and `{arg1}` with the two numeric args
/// from the `McuLog` frame.
///
/// Returns `("unknown", "")` for unrecognised pairs.
///
/// # Examples
///
/// ```
/// use runtime_contract::log_codes::{event_info, SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRSYNC_TRIGGER_CMD};
///
/// let (name, tmpl) = event_info(SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRSYNC_TRIGGER_CMD);
/// assert_eq!(name, "endstop.trsync_trigger_cmd");
/// assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));
/// ```
pub fn event_info(subsystem: u8, event: u16) -> (&'static str, &'static str) {
    match (subsystem, event) {
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FAULT_LATCHED) => {
            ("runtime.fault_latched", "fault latched, detail={arg0}")
        }
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_MCU_READY) => {
            ("runtime.mcu_ready", "mcu firmware ready, log drain online")
        }
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_LOG_DROPS) => (
            "runtime.log_drops",
            "dropped {arg0} log entries (ring overflow)",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_MCU_RESET) => (
            "runtime.mcu_reset",
            "mcu reset (cause bits={arg0}, iwdg_resets={arg1})",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_HARD_FAULT) => (
            "runtime.hard_fault",
            "cpu hard fault pc={arg0:hex} lr={arg1:hex}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FAULT_STATUS) => (
            "runtime.fault_status",
            "fault status cfsr={arg0:hex} hfsr={arg1:hex}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FG_FREEZE) => (
            "runtime.fg_freeze",
            "foreground freeze pc={arg0:hex} stall_ticks={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_RT_PROGRESS) => (
            "runtime.rt_progress",
            "runtime progress packed={arg0} fault_count={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_LAST_DISPATCH) => (
            "runtime.last_dispatch",
            "last dispatch func={arg0:hex} addr={arg1:hex}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_ISR_PHASE) => {
            ("runtime.isr_phase", "isr phase={arg0} ring_overflow={arg1}")
        }
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_BLOCK_SOURCE) => {
            ("runtime.block_source", "block usb_burst={arg0} cyc")
        }
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_TIM5_IA) => (
            "runtime.tim5_ia",
            "tim5 inter-arrival min={arg0} max={arg1} cyc",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_DIAG_DUMP) => (
            "runtime.diag_dump",
            "live diag dump uptime_us={arg0} ring_seq={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FG_TASK) => (
            "runtime.fg_task",
            "foreground worst task func={arg0:hex} dur_cyc={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FG_MSG) => (
            "runtime.fg_msg",
            "foreground worst msg kind={arg0} dur_cyc={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FG_DEMUX) => (
            "runtime.fg_demux",
            "demux backlog_max={arg0} msgs_max={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FG_MSG_HEAD) => (
            "runtime.fg_msg_head",
            "foreground worst msg head_bytes={arg0} cur_head_bytes={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_TIMER_TOO_CLOSE) => (
            "runtime.timer_too_close",
            "timer too close caller_pc={arg0:hex} timer_func={arg1:hex} count=code",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_TIMER_TOO_CLOSE_LATE) => (
            "runtime.timer_too_close_late",
            "timer too close late_cyc={arg0} count={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_PRIOR_RUN) => (
            "runtime.prior_run",
            "crash replay describes boot {arg0}; {arg1} later hostless boot(s) were skipped to keep it",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_PRIOR_LIVE) => (
            "runtime.prior_live",
            "held run ended with engine_status={arg0} liveness_ok={arg1} (0 = the firmware stopped feeding the iwdg on purpose)",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_PRIOR_USB_OUT) => (
            "runtime.prior_usb_out",
            "held run's worst usb OUT episode: endpoint unarmed {arg0} cyc, bulk-out task gap {arg1} ticks",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_PRIOR_TASK_GAPS) => (
            "runtime.prior_task_gaps",
            "held run's worst foreground gaps: runtime_drain {arg0} ticks, usb bulk-in {arg1} ticks",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_TIM5_LONG) => {
            ("diag.tim5_long", "TIM5 ISR long {arg0} cyc at t={arg1}")
        }
        (SUBSYSTEM_DIAG, EVENT_DIAG_OTG_LONG) => {
            ("diag.otg_long", "OTG ISR long {arg0} cyc at t={arg1}")
        }
        (SUBSYSTEM_DIAG, EVENT_DIAG_USB_OUT_GAP) => (
            "diag.usb_out_gap",
            "USB-OUT gap {arg0} ticks, prev t={arg1}",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_USB_IN_GAP) => {
            ("diag.usb_in_gap", "USB-IN gap {arg0} ticks, prev t={arg1}")
        }
        (SUBSYSTEM_DIAG, EVENT_DIAG_TX_DROP_KAL) => (
            "diag.tx_drop_kalico",
            "kalico TX drop len={arg0} tpos={arg1}",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_TX_DROP_KLP) => (
            "diag.tx_drop_klipper",
            "klipper TX drop max={arg0} tpos={arg1}",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_ENGINE_XITION) => (
            "diag.engine_xition",
            "engine state packed={arg0} samples={arg1}",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_RUST_FAULT) => {
            ("diag.rust_fault", "rust fault err={arg0} detail={arg1}")
        }
        (SUBSYSTEM_MOTION, EVENT_MOTION_AXIS_STALLED) => (
            "motion.axis_stalled",
            "axis retirement stalled with runs pending axis={arg0:hi16} occupancy={arg0:lo16} stalled_ms={arg1}",
        ),
        (SUBSYSTEM_MOTION, EVENT_MOTION_AXIS_STALLED_HEAD) => (
            "motion.axis_stalled_head",
            "front sample-run window vs now start-now={arg0:i32}ms end-now={arg1:i32}ms",
        ),
        (SUBSYSTEM_MOTION, EVENT_MOTION_STEP_LOAD_LATE) => (
            "motion.step_load_late",
            "classic move loaded behind its pending unstep: behind={arg0:i32} cyc min_next={arg1}",
        ),
        (SUBSYSTEM_MOTION, EVENT_MOTION_STEP_REARM) => (
            "motion.step_rearm",
            "idle stepper re-armed count={arg0} min_margin_cyc={arg1:i32}",
        ),
        (SUBSYSTEM_MOTION, EVENT_MOTION_STEP_REARM_TIGHT) => (
            "motion.step_rearm_tight",
            "idle stepper re-arm margin armed={arg0} below_floor={arg1}",
        ),
        (SUBSYSTEM_MOTION, EVENT_MOTION_STEP_REARM_LATE) => (
            "motion.step_rearm_late",
            "idle stepper re-armed behind the mcu clock by margin={arg0:i32} cyc waketime={arg1}",
        ),
        (SUBSYSTEM_MOTION, EVENT_MOTION_STEP_HALT) => (
            "motion.step_halt",
            "classic stepper halted flags={arg0} pending_events={arg1}",
        ),
        (SUBSYSTEM_MOTION, EVENT_MOTION_WIRE_PROBE_LATE) => (
            "motion.wire_probe_late",
            "host->mcu wire probe arrived late by delta={arg0:i32} cyc (claimed clock={arg1})",
        ),
        (SUBSYSTEM_MOTION, EVENT_MOTION_STEP_CLOCK_HORIZON) => (
            "motion.step_clock_horizon",
            "step clock {arg1} is distance={arg0:i32} cyc from the mcu clock, beyond the sync horizon",
        ),
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRSYNC_TRIGGER_CMD) => (
            "endstop.trsync_trigger_cmd",
            "trsync_trigger cmd oid={arg0} reason={arg1}",
        ),
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRSYNC_DO_TRIGGER) => (
            "endstop.trsync_do_trigger",
            "trsync_do_trigger flags={arg0} reason={arg1}",
        ),
        _ => ("unknown", ""),
    }
}

#[cfg(test)]
mod tests;
