//! Structured logging for the EtherCAT endpoint process.
//!
//! The endpoint is a separate process from the klippy-hosted bridge, so it has
//! no subscriber of its own — without this its `tracing` events vanish. This
//! installs a JSON-lines subscriber that appends to `<events_dir>/host-ec.jsonl`
//! with `source = "host-ec"`. Vector's `events/*.jsonl` glob ships the file to
//! VictoriaLogs, so endpoint events are queryable alongside the bridge's
//! (`source:=host-ec`).
//!
//! The emitting thread (which includes the 250 µs FIFO-80 DC cycle) only
//! captures the event's fields and timestamp and hands the record to a worker
//! over a lossy bounded channel — timestamp formatting, JSON serialization and
//! file I/O all happen on the worker thread. The channel is LOSSY: when it
//! fills (wedged SD, log storm) records are dropped rather than blocking the
//! cycle, and the drops are counted. [`emit_dropped_line_report`] turns
//! counter growth into an `obs_log_lines_dropped` warn from a periodic
//! non-RT-critical caller.

use std::io::Write;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::OnceLock;

use crossbeam_channel::{bounded, Sender, TrySendError};
use serde_json::{Map, Value};
use time::format_description::FormatItem;
use time::macros::format_description;
use time::OffsetDateTime;
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::layer::Context;
use tracing_subscriber::prelude::*;
use tracing_subscriber::{EnvFilter, Layer};

const SOURCE: &str = "host-ec";

/// Deep enough to absorb a telemetry beat burst (~10 records) many times
/// over while the worker is stalled on a slow SD write.
const CHANNEL_CAPACITY: usize = 4096;

const TIME_FMT: &[FormatItem<'static>] =
    format_description!("[year]-[month]-[day]T[hour]:[minute]:[second].[subsecond digits:3]Z");

static SESSION: OnceLock<String> = OnceLock::new();
static DROPPED_LINES: AtomicUsize = AtomicUsize::new(0);
static DROP_REPORT: DropReport = DropReport::new();

pub(crate) struct DropReport {
    reported: AtomicUsize,
}

impl DropReport {
    pub(crate) const fn new() -> Self {
        Self {
            reported: AtomicUsize::new(0),
        }
    }

    pub(crate) fn newly_dropped(&self, cumulative: usize) -> Option<usize> {
        let previously_reported = self.reported.swap(cumulative, Ordering::Relaxed);
        (cumulative > previously_reported).then_some(cumulative)
    }
}

/// Report appender drops since the last call. Call from a periodic
/// non-RT-critical path — a counter load plus, only on growth, one warn event.
pub fn emit_dropped_line_report() {
    let Some(dropped_total) = DROP_REPORT.newly_dropped(DROPPED_LINES.load(Ordering::Relaxed))
    else {
        return;
    };
    eprintln!("ec-rt: obs: lossy log channel overflowed — {dropped_total} lines dropped so far");
    tracing::warn!(
        subsystem = "ethercat",
        event = "obs_log_lines_dropped",
        dropped_total = dropped_total as u64,
        "log record channel overflowed; {dropped_total} lines dropped so far (cumulative)"
    );
}

fn session_id() -> &'static str {
    SESSION.get().map_or("ec-unbound", String::as_str)
}

fn level_str(level: &Level) -> &'static str {
    match *level {
        Level::TRACE => "trace",
        Level::DEBUG => "debug",
        Level::INFO => "info",
        Level::WARN => "warn",
        Level::ERROR => "error",
    }
}

#[derive(Default)]
struct FieldVisitor {
    map: Map<String, Value>,
    message: Option<String>,
}

impl Visit for FieldVisitor {
    fn record_str(&mut self, field: &Field, value: &str) {
        if field.name() == "message" {
            self.message = Some(value.to_string());
        } else {
            self.map
                .insert(field.name().to_string(), Value::String(value.to_string()));
        }
    }

    fn record_i64(&mut self, field: &Field, value: i64) {
        self.map
            .insert(field.name().to_string(), Value::from(value));
    }

    fn record_u64(&mut self, field: &Field, value: u64) {
        self.map
            .insert(field.name().to_string(), Value::from(value));
    }

    fn record_f64(&mut self, field: &Field, value: f64) {
        self.map
            .insert(field.name().to_string(), Value::from(value));
    }

    fn record_bool(&mut self, field: &Field, value: bool) {
        self.map
            .insert(field.name().to_string(), Value::Bool(value));
    }

    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        let s = format!("{value:?}");
        if field.name() == "message" {
            self.message = Some(s);
        } else {
            self.map.insert(field.name().to_string(), Value::String(s));
        }
    }
}

/// One captured event, handed from the emitting thread to the format/write
/// worker. Field capture is the only work the emitter does.
struct LogRecord {
    time: OffsetDateTime,
    level: &'static str,
    target: &'static str,
    message: Option<String>,
    fields: Map<String, Value>,
}

fn render_line(record: LogRecord) -> String {
    let LogRecord {
        time,
        level,
        target,
        message,
        mut fields,
    } = record;
    let mut out = Map::new();
    out.insert(
        "_time".into(),
        Value::String(
            time.format(&TIME_FMT)
                .unwrap_or_else(|_| "1970-01-01T00:00:00.000Z".to_string()),
        ),
    );
    out.insert("_msg".into(), Value::String(message.unwrap_or_default()));
    out.insert("level".into(), Value::String(level.into()));
    out.insert("source".into(), Value::String(SOURCE.into()));
    let subsystem = match fields.remove("subsystem") {
        Some(Value::String(s)) => s,
        _ => "ethercat".to_string(),
    };
    out.insert("subsystem".into(), Value::String(subsystem));
    out.insert("session_id".into(), Value::String(session_id().into()));
    out.insert("target".into(), Value::String(target.to_string()));
    for (k, v) in fields {
        out.entry(k).or_insert(v);
    }

    let mut line = serde_json::to_string(&Value::Object(out))
        .unwrap_or_else(|e| format!("{{\"_msg\":\"serialize error: {e}\"}}"));
    line.push('\n');
    line
}

struct JsonlLayer {
    sender: Sender<LogRecord>,
}

impl<S: Subscriber> Layer<S> for JsonlLayer {
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let mut visitor = FieldVisitor::default();
        event.record(&mut visitor);
        let meta = event.metadata();
        let record = LogRecord {
            time: OffsetDateTime::now_utc(),
            level: level_str(meta.level()),
            target: meta.target(),
            message: visitor.message,
            fields: visitor.map,
        };
        match self.sender.try_send(record) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
                DROPPED_LINES.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

fn spawn_writer(file: std::fs::File) -> Sender<LogRecord> {
    let (sender, receiver) = bounded::<LogRecord>(CHANNEL_CAPACITY);
    std::thread::Builder::new()
        .name("obs-writer".into())
        .spawn(move || {
            let mut file = file;
            while let Ok(record) = receiver.recv() {
                let line = render_line(record);
                let _ = file.write_all(line.as_bytes());
            }
        })
        .expect("spawn obs-writer thread");
    sender
}

/// Install the endpoint's JSON-lines subscriber. Best-effort: if the events
/// directory can't be opened (a box without the observability tree) the endpoint
/// still runs, with events unrecorded. Idempotent — a second call is a no-op.
pub fn init(events_dir: &Path, session: String) {
    let _ = SESSION.set(session);
    let path = events_dir.join("host-ec.jsonl");
    let file = match std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
    {
        Ok(f) => f,
        Err(e) => {
            eprintln!("ec-rt: obs: cannot open {}: {e}", path.display());
            return;
        }
    };
    let sender = spawn_writer(file);
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let subscriber = tracing_subscriber::registry()
        .with(filter)
        .with(JsonlLayer { sender });
    if tracing::subscriber::set_global_default(subscriber).is_err() {
        eprintln!("ec-rt: obs: global subscriber already set");
    }
}

#[cfg(test)]
#[path = "obs_tests.rs"]
mod obs_tests;
