---
name: query-logs
description: Use when investigating the kalico host logs — finding why homing/a print/an MCU op failed, filtering by session or print or subsystem or level or event type, resolving a numeric fault code, checking whether the logging pipeline itself is healthy, or comparing host-py vs host-rust behavior. Primary tool is `scripts/logq.py` (health/sessions/prints/print/session/tail/schema/resolve); raw LogsQL via `logq.py q` is the escape hatch for bespoke queries.
---

# Querying kalico structured logs (VictoriaLogs / LogsQL)

The kalico host writes structured NDJSON to `~/printer_data/logs/events/*.jsonl`
(durable, plug-pull-safe source of truth). Vector ships it into **VictoriaLogs**,
which indexes **every** field — so instead of grepping a flat log you query by
`session_id` / `print_id` / `subsystem` / `level` / `event` / numeric payload.

## Use `scripts/logq.py` — don't hand-write LogsQL for routine questions

Every subcommand health-checks VL first, so the classic trap ("empty output
= dead VL, silently misread as clean") can't happen — a down pipeline exits
**2** and prints where to find the durable JSONL instead.

```bash
python3 scripts/logq.py <command> [args] [--since 24h]
KALICO_VL=http://ethercatpi5.local:9428 python3 scripts/logq.py health   # remote bench
python3 scripts/logq.py --vl http://ethercatpi5.local:9428 health       # same, via flag
```

- `--vl URL` (or `$KALICO_VL` env) points at a remote bench's VL instead of `127.0.0.1:9428`.
- `--since` is `\d+[smhdw]` (e.g. `24h`, `30m`, `7d`) — every time-scoped subcommand takes it.
- Repeated identical records (same source/subsystem/event/level, or same message) are condensed to one line with a `time_range ×N` suffix — that's normal, not a bug.
- Exit code **2** = pipeline down (VL unreachable); **1** = bad usage (e.g. invalid `--since`); **0** = ran, even if 0 records matched.

Subcommands:

- `health` — VL reachable? heartbeat freshness. Run this first when anything looks off.
  `python3 scripts/logq.py health`
- `sessions [--since 24h] [-n 20]` — recent sessions, busiest/most-recent first.
  `python3 scripts/logq.py sessions --since 24h -n 10`
- `prints [--since 7d] [-n 20]` — recent prints: id, start, duration, outcome, reason.
  `python3 scripts/logq.py prints --since 7d`
- `print <id|last> [--since 7d] [--all]` — **main investigation view** for one print: header (session/print ids, file, start/end, outcome/reason/duration) + condensed warn/error timeline; add `--all` to include info-level records too.
  `python3 scripts/logq.py print last`
  `python3 scripts/logq.py print print-1783354078 --since 24h`
- `session <id> [--since 24h] [--all]` — same investigation view, scoped to a session_id instead of a print.
  `python3 scripts/logq.py session k-1783352398-15015`
- `tail [--since 10m] [--level warn]` — most recent records at or above a level (`trace|debug|info|warn|error`).
  `python3 scripts/logq.py tail --since 10m --level warn`
- `schema [--since 24h]` — **live ground truth**: source/subsystem counts, top events, level counts. Prefer this over any schema notes in docs or this file — schemas drift, this doesn't.
  `python3 scripts/logq.py schema --since 24h`
- `resolve <code> [--since 30d]` — numeric fault code → `code_name` / event / sample message.
  `python3 scripts/logq.py resolve 65228`
- `q '<raw LogsQL>' [--limit 50] [--raw]` — escape hatch for anything the above don't cover; still runs the health check first. `--raw` prints NDJSON verbatim instead of the formatted view.
  `python3 scripts/logq.py q 'subsystem:=homing trigger_mm:>10 _time:24h'`

## The schema (fields you filter on)

Run `python3 scripts/logq.py schema` for current ground truth — the list below is a
starting point and can lag what's actually live.

Core (every record): `_time`, `_msg`, `level` (`trace|debug|info|warn|error`),
`source` — open set, values observed live: `host-py`, `host-rust`, `host-ec`, `mcu`
(each MCU logs under its klippy name, e.g. `mcu`, `bottom`) — `subsystem`,
`session_id`, `target`. Session id shape is `k-<unix>-<pid>` for the main host
process; `host-ec` sessions instead look like `ec-<pid>` and carry no `print_id`.
Optional: `print_id` (empty when idle), `event` (stable key like
`homing.endstop_trip`), `code`/`code_name`, plus typed payload (`axis`,
`trigger_mm`, `gcode_line`, `oid`, …). `_stream = {source, subsystem}`.

Print lifecycle (subsystem `print_stats`, `logq.py prints`/`print` already join
these for you — only reach for raw LogsQL if you need something else):
- `print.start` — field `file`
- `print.pause`
- `print.resume` — field `pause_duration_s`
- `print.end` — `outcome` ∈ `complete|cancelled|error|reset`, `reason`, `duration_s`, `filament_mm`

## Is the pipeline itself healthy?

"VL is down / Vector stalled" is exactly the case that can't self-report through
the logs. First step: `python3 scripts/logq.py health` — it checks VL reachability
and heartbeat freshness (`subsystem=observability event=heartbeat`, emitted ~30s)
in one call and warns if the heartbeat is stale (>60s).

Service-level check (run on the printer; both must be `active`):

```bash
systemctl is-active victorialogs vector
journalctl -u victorialogs -u vector --no-pager | tail -20   # why, if not
```

On the printers the stack lives in `~/observability/` (binaries
`victoria-logs-prod` v1.50.0 + `vector` 0.55.0, unit files, `vector.toml`,
data dirs) with the units symlinked into systemd via
`sudo systemctl enable --now /home/<user>/observability/*.service` —
chosen because the printer sudoers only passwordless-allows `systemctl`.
If the units don't exist at all, the stack was never installed on that
host: deploy per `config/observability/*.service` headers (adapt `User=`
and the `/home/pi/printer_data` path in `vector.toml`).

The on-disk JSONL is the source of truth regardless — if VL is empty but a print
just failed, the records are still in `~/printer_data/logs/events/*.jsonl` and VL
will backfill from Vector's checkpoint once it's healthy.

## How records get here (context)

`structured_log.event(...)` / Rust `tracing` → `events/{host-py,host-rust,...}.jsonl`
(durable) → Vector tails + checkpoints → VL `/insert/jsonline` → `logq.py` queries
`/select/logsql/query`. See `config/observability/vector.toml`.

## Not here: host-process coredumps

This skill covers **structured logs** only; `mcu-diagnostics` covers **MCU**
faults. Neither covers native **host-process coredumps**. When a host process
crashes (Rust `motion-engine`/`ethercat-rt` threads, or the klippy `python`
interpreter), the kernel writes a full ELF core to
`~/printer_data/logs/coredumps/core.<exe>.<pid>.<time>` via `core_pattern` —
**not** `systemd-coredump`, so `coredumpctl` and `/var/lib/systemd/coredump`
are empty and LogsQL will not show them. Read one with
`gdb <matching-binary> <core> -ex bt -ex 'thread apply all bt'` (the release
build keeps symbols). These are large (~40–80 MB each) and accumulate
unbounded, so a crash storm both fills the SD and shows up only as a gap in the
structured stream around the crash `_time`.

## Appendix: raw LogsQL (for queries logq doesn't cover)

Everything below goes through `python3 scripts/logq.py q '<LogsQL>'` — that
keeps the pre-flight health check. Only reach for this when `health`,
`sessions`, `prints`, `print`, `session`, `tail`, `schema`, or `resolve` don't
answer the question directly.

- **Always include a time bound**: `_time:1h`, `_time:24h`, `_time:7d`. Absolute
  range (rarely needed): `_time:[2026-06-01T00:00:00Z, 2026-06-02T00:00:00Z]`.
- VL renders all field values as **strings**, but numeric range filters still
  work at query time.
- LogsQL operators: `field:=value` (exact), `field:in(a,b)` (set), `field:>N` /
  `field:<N` (numeric range), `field:!=""` (present), `"text"` (full-text on
  `_msg`), `| sort by (_time)` / `desc`, `| fields a, b`, `| limit N`, and
  `| stats by (f) count() as n` — **alias aggregates** (`as n`) so you can then
  `| sort by (n)`; `sort by (count(*))` does not parse.

Recipes:

```bash
# --- orient yourself (logq.py sessions / prints cover these; shown for reference) ---
python3 scripts/logq.py q '* _time:1h | stats by (session_id) count() as hits | sort by (hits) desc'
python3 scripts/logq.py q 'print_id:!="" _time:24h | stats by (print_id) min(_time) as first, max(_time) as last'

# --- investigate a failure beyond logq.py session/print ---
python3 scripts/logq.py q 'session_id:=k-1748700131-4412 level:in(warn,error) _time:6h | sort by (_time)'
python3 scripts/logq.py q 'print_id:=print-1748700500 subsystem:=homing _time:24h | sort by (_time)'
python3 scripts/logq.py q 'subsystem:=clocksync _time:1h | sort by (_time) desc' --limit 50

# --- by event type / code ---
python3 scripts/logq.py q 'event:=step_queue_overflow _time:7d | stats by (code_name) count() as n | sort by (n) desc'
python3 scripts/logq.py q 'subsystem:=homing trigger_mm:>10 _time:24h'

# --- host-py vs host-rust ---
python3 scripts/logq.py q 'source:=host-rust level:in(warn,error) _time:1h | sort by (_time)'
python3 scripts/logq.py q 'source:=host-py subsystem:=motion _time:1h | sort by (_time) desc' --limit 100

# --- free-text fallback (searches _msg) ---
python3 scripts/logq.py q '"needs rehome" _time:1h'
```

## Optional: mcp-victorialogs

If you prefer tool-calls over `logq.py`, `mcp-victorialogs` is a first-party
MCP that talks to the **same** VL endpoint (`$KALICO_VL`) — drop it in, no code
from us. It bypasses `logq.py`'s health check, so confirm the pipeline is up
(`logq.py health`) before trusting an empty result.
