//! Structured logging for the EtherCAT endpoint process.
//!
//! The endpoint is a separate process from the klippy-hosted bridge, so it has
//! no subscriber of its own — without this its `tracing` events vanish. This
//! installs a JSON-lines subscriber that appends to `<events_dir>/host-ec.jsonl`
//! with `source = "host-ec"`. Vector's `events/*.jsonl` glob ships the file to
//! VictoriaLogs, so endpoint events are queryable alongside the bridge's
//! (`source:=host-ec`). Writes go through a non-blocking worker thread so the DC
//! cycle never blocks on log I/O; the channel to that worker is LOSSY — when it
//! fills (wedged SD, log storm) lines are dropped rather than blocking the
//! 250 µs cycle, and the drops are counted. [`emit_dropped_line_report`] turns
//! counter growth into an `obs_log_lines_dropped` warn from a periodic
//! non-RT-critical caller.

use std::io::Write;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::OnceLock;

use serde_json::{Map, Value};
use time::format_description::FormatItem;
use time::macros::format_description;
use time::OffsetDateTime;
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_appender::non_blocking::{ErrorCounter, NonBlocking, WorkerGuard};
use tracing_subscriber::layer::Context;
use tracing_subscriber::prelude::*;
use tracing_subscriber::{EnvFilter, Layer};

const SOURCE: &str = "host-ec";

const TIME_FMT: &[FormatItem<'static>] =
    format_description!("[year]-[month]-[day]T[hour]:[minute]:[second].[subsecond digits:3]Z");

static GUARD: OnceLock<WorkerGuard> = OnceLock::new();
static SESSION: OnceLock<String> = OnceLock::new();
static DROPPED_LINES: OnceLock<ErrorCounter> = OnceLock::new();
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
    let Some(counter) = DROPPED_LINES.get() else {
        return;
    };
    let Some(dropped_total) = DROP_REPORT.newly_dropped(counter.dropped_lines()) else {
        return;
    };
    eprintln!("ec-rt: obs: lossy log channel overflowed — {dropped_total} lines dropped so far");
    tracing::warn!(
        subsystem = "ethercat",
        event = "obs_log_lines_dropped",
        dropped_total = dropped_total as u64,
        "log appender channel overflowed; {dropped_total} lines dropped so far (cumulative)"
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

struct JsonlLayer {
    writer: NonBlocking,
}

impl<S: Subscriber> Layer<S> for JsonlLayer {
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let mut visitor = FieldVisitor::default();
        event.record(&mut visitor);
        let meta = event.metadata();

        let mut out = Map::new();
        out.insert(
            "_time".into(),
            Value::String(
                OffsetDateTime::now_utc()
                    .format(&TIME_FMT)
                    .unwrap_or_else(|_| "1970-01-01T00:00:00.000Z".to_string()),
            ),
        );
        out.insert(
            "_msg".into(),
            Value::String(visitor.message.unwrap_or_default()),
        );
        out.insert(
            "level".into(),
            Value::String(level_str(meta.level()).into()),
        );
        out.insert("source".into(), Value::String(SOURCE.into()));
        let subsystem = match visitor.map.remove("subsystem") {
            Some(Value::String(s)) => s,
            _ => "ethercat".to_string(),
        };
        out.insert("subsystem".into(), Value::String(subsystem));
        out.insert("session_id".into(), Value::String(session_id().into()));
        out.insert("target".into(), Value::String(meta.target().to_string()));
        for (k, v) in visitor.map {
            out.entry(k).or_insert(v);
        }

        let mut line = serde_json::to_string(&Value::Object(out))
            .unwrap_or_else(|e| format!("{{\"_msg\":\"serialize error: {e}\"}}"));
        line.push('\n');
        let mut w = self.writer.clone();
        let _ = w.write_all(line.as_bytes());
    }
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
    let (non_blocking, guard) = tracing_appender::non_blocking::NonBlockingBuilder::default()
        .lossy(true)
        .finish(file);
    let error_counter = non_blocking.error_counter();
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let subscriber = tracing_subscriber::registry()
        .with(filter)
        .with(JsonlLayer {
            writer: non_blocking,
        });
    if tracing::subscriber::set_global_default(subscriber).is_err() {
        eprintln!("ec-rt: obs: global subscriber already set");
        return;
    }
    let _ = GUARD.set(guard);
    let _ = DROPPED_LINES.set(error_counter);
}

#[cfg(test)]
#[path = "obs_tests.rs"]
mod obs_tests;
