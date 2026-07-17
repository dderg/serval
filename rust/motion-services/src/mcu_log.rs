use motion_core::lock_ext::LockExt;
use std::io::Write;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{SyncSender, TrySendError, sync_channel};
use std::sync::{Arc, Mutex};

use serde_json::{Map, Value};
use time::OffsetDateTime;

use host_rt::host_io::runtime_events::McuLogEvent;
use host_rt::passthrough_queue::{McuHandle, PassthroughRouter};
use runtime::error::FaultCode;
use runtime::log_codes::{compose_msg, event_info, subsystem_name};

use crate::logging::context::load_context;
use crate::logging::schema::format_time;
use crate::logging::writer::RotatingJsonlWriter;

fn mcu_level_str(level: u8) -> &'static str {
    match level {
        0 => "trace",
        1 => "debug",
        2 => "warn",
        _ => "error",
    }
}

/// MCU log records are sparse (warns, faults, arrival diagnostics); thousands
/// queued means the writer thread has been stalled for a long time, at which
/// point records are dropped and counted rather than ever blocking the hook's
/// caller.
const LOG_QUEUE_CAPACITY: usize = 4096;
const DROP_REPORT_STRIDE: u64 = 1000;

/// Move the blocking filesystem write onto a dedicated thread. The mcu-log
/// hook runs inline on the MCU transport reactor thread, which also routes
/// every request/response for that MCU — one `write()` wedged on a contended
/// SD card starves PushPieces of its scheduling lead and aborts the host
/// (2026-07-16 trident post-mortem). The thread drains the queue, flushes
/// (fsync at most every `FSYNC_INTERVAL`), and exits when the last sender
/// (the hook) drops.
pub fn spawn_jsonl_writer_thread(
    mut writer: RotatingJsonlWriter,
    source: &str,
) -> SyncSender<String> {
    let (tx, rx) = sync_channel::<String>(LOG_QUEUE_CAPACITY);
    let thread_source = source.to_owned();
    std::thread::Builder::new()
        .name(format!("mcu-log-{source}"))
        .spawn(move || {
            while let Ok(line) = rx.recv() {
                if let Err(err) = writer.write_all(line.as_bytes()) {
                    eprintln!("[mcu-log {thread_source}] JSONL write failed: {err}");
                }
                if let Err(err) = writer.flush() {
                    eprintln!("[mcu-log {thread_source}] JSONL flush failed: {err}");
                }
            }
        })
        .expect("spawn mcu-log writer thread");
    tx
}

pub fn build_mcu_log_hook(
    router: Arc<Mutex<PassthroughRouter>>,
    mcu: McuHandle,
    sink: SyncSender<String>,
    source: String,
) -> impl Fn(McuLogEvent) + Send + Sync + 'static {
    let dropped = AtomicU64::new(0);
    move |e: McuLogEvent| {
        let (time_str, time_estimated) = {
            let guard = router.lock_ok();
            if let Some((dt, estimated)) = guard.wall_time_at_mcu(mcu, e.mcu_tick) {
                (format_time(dt), estimated)
            } else {
                let elapsed = e.host_recv.elapsed();
                let sys = std::time::SystemTime::now()
                    .checked_sub(elapsed)
                    .unwrap_or(std::time::SystemTime::UNIX_EPOCH);
                (format_time(OffsetDateTime::from(sys)), true)
            }
        };

        let subsys_name = subsystem_name(e.subsystem);
        let (event_name, template) = event_info(e.subsystem, e.event);
        let msg = compose_msg(template, e.args[0], e.args[1]);

        let (code_val, code_name_val): (Option<u16>, Option<&'static str>) = if e.code != 0 {
            let name = FaultCode::from_u16(e.code)
                .map(FaultCode::code_name)
                .unwrap_or("unknown");
            (Some(e.code), Some(name))
        } else {
            (None, None)
        };

        let ctx = load_context();

        let mut rec = Map::new();
        rec.insert("_time".into(), Value::String(time_str));
        rec.insert("_msg".into(), Value::String(msg));
        rec.insert(
            "level".into(),
            Value::String(mcu_level_str(e.level).to_owned()),
        );
        rec.insert("source".into(), Value::String(source.clone()));
        rec.insert("subsystem".into(), Value::String(subsys_name.to_owned()));
        rec.insert("event".into(), Value::String(event_name.to_owned()));
        rec.insert("session_id".into(), Value::String(ctx.session_id.clone()));
        rec.insert("print_id".into(), Value::String(ctx.print_id.clone()));
        rec.insert(
            "target".into(),
            Value::String(format!("mcu::{subsys_name}")),
        );
        rec.insert("mcu_tick".into(), Value::from(e.mcu_tick));
        rec.insert("seq".into(), Value::from(e.seq));
        rec.insert("arg0".into(), Value::from(e.args[0]));
        rec.insert("arg1".into(), Value::from(e.args[1]));
        rec.insert("time_estimated".into(), Value::Bool(time_estimated));
        if let Some(code) = code_val {
            rec.insert("code".into(), Value::from(code));
        }
        if let Some(name) = code_name_val {
            rec.insert("code_name".into(), Value::String(name.to_owned()));
        }

        let mut line = serde_json::to_string(&Value::Object(rec))
            .unwrap_or_else(|err| format!("{{\"_msg\":\"mcu-log serialize error: {err}\"}}"));
        line.push('\n');

        match sink.try_send(line) {
            Ok(()) => {
                let n = dropped.swap(0, Ordering::Relaxed);
                if n > 0 {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "mcu_log_drops_recovered",
                        source = %source,
                        dropped = n,
                        "mcu-log queue drained after dropping records"
                    );
                }
            }
            Err(TrySendError::Full(_)) => {
                let n = dropped.fetch_add(1, Ordering::Relaxed) + 1;
                if n == 1 || n % DROP_REPORT_STRIDE == 0 {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "mcu_log_queue_overflow",
                        source = %source,
                        dropped = n,
                        "mcu-log queue full — dropping record; writer thread stalled (slow disk?)"
                    );
                }
            }
            Err(TrySendError::Disconnected(_)) => {
                let n = dropped.fetch_add(1, Ordering::Relaxed) + 1;
                if n == 1 || n % DROP_REPORT_STRIDE == 0 {
                    tracing::error!(
                        subsystem = "mcu-comms",
                        event = "mcu_log_writer_dead",
                        source = %source,
                        dropped = n,
                        "mcu-log writer thread is gone — dropping record"
                    );
                }
            }
        }
    }
}
