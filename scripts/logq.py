#!/usr/bin/env python3
"""CLI for querying the VictoriaLogs structured-log store (see docs/rewrite/observability)."""

import argparse
import datetime
import json
import pathlib
import re
import sys
import tarfile
import urllib.error
import urllib.parse
import urllib.request

DEFAULT_VL_URL = "http://127.0.0.1:9428"

EVENT_PRINT_START = "print.start"
EVENT_PRINT_PAUSE = "print.pause"
EVENT_PRINT_RESUME = "print.resume"
EVENT_PRINT_END = "print.end"
LIFECYCLE_EVENTS = (
    EVENT_PRINT_START,
    EVENT_PRINT_PAUSE,
    EVENT_PRINT_RESUME,
    EVENT_PRINT_END,
)
EVENT_HEARTBEAT = "heartbeat"

LEVEL_LETTERS = {
    "trace": "T",
    "debug": "D",
    "info": "I",
    "warn": "W",
    "error": "E",
}
LEVEL_ORDER = ("trace", "debug", "info", "warn", "error")

KNOWN_FIELDS = frozenset(
    (
        "_time",
        "_msg",
        "_stream",
        "_stream_id",
        "level",
        "source",
        "subsystem",
        "session_id",
        "print_id",
        "target",
        "event",
    )
)

SINCE_RE = re.compile(r"^\d+[smhdw]$")

PIPELINE_DOWN_MESSAGE = (
    "VictoriaLogs at {url} is unreachable ({detail}).\n"
    "The structured logging pipeline is down: no live query is possible.\n"
    "Durable source of truth on the printer: ~/printer_data/logs/events/*.jsonl"
)


class VlUnreachableError(Exception):
    def __init__(self, url, detail):
        super().__init__(detail)
        self.url = url
        self.detail = detail


class UsageError(Exception):
    pass


def validate_since(value):
    if not SINCE_RE.match(value):
        raise UsageError(
            "--since %r is invalid; expected a number followed by s/m/h/d/w "
            "(e.g. 24h, 30m, 7d)" % (value,)
        )
    return value


def levels_at_or_above(level):
    if level not in LEVEL_ORDER:
        raise UsageError(
            "--level %r is invalid; expected one of %s"
            % (level, ", ".join(LEVEL_ORDER))
        )
    idx = LEVEL_ORDER.index(level)
    return LEVEL_ORDER[idx:]


def build_health_query():
    return (
        "subsystem:=observability event:=%s _time:10m | sort by (_time) desc"
        % EVENT_HEARTBEAT
    )


def build_sessions_query(since):
    return (
        "_time:%s | stats by (session_id) count() as hits, "
        "min(_time) as first, max(_time) as last | sort by (last) desc" % since
    )


def build_prints_span_query(since):
    return (
        'print_id:!="" _time:%s | stats by (print_id) min(_time) as first, max(_time) as last'
        % since
    )


def build_prints_end_query(since):
    return (
        "event:=%s _time:%s | fields _time, print_id, outcome, reason, duration_s"
        % (EVENT_PRINT_END, since)
    )


def build_lifecycle_query(scope_field, scope_value, since):
    return "%s:=%s subsystem:=print_stats _time:%s" % (
        scope_field,
        scope_value,
        since,
    )


def build_warn_error_query(scope_field, scope_value, since):
    return "%s:=%s level:in(warn,error) _time:%s" % (
        scope_field,
        scope_value,
        since,
    )


def build_info_query(scope_field, scope_value, since):
    return "%s:=%s level:=info _time:%s" % (scope_field, scope_value, since)


def build_tail_query(since, levels):
    return "level:in(%s) _time:%s" % (",".join(levels), since)


def build_schema_source_query(since):
    return (
        "_time:%s | stats by (source, subsystem) count() as n | sort by (n) desc"
        % since
    )


def build_schema_event_query(since):
    return (
        'event:!="" _time:%s | stats by (source, event, level) count() as n | sort by (n) desc'
        % since
    )


def build_schema_level_query(since):
    return "_time:%s | stats by (level) count() as n" % since


def build_resolve_query(code, since):
    return "code:=%s _time:%s | fields code_name, event, _msg" % (code, since)


BUNDLE_MEMBER_MAX_BYTES = 128 * 1024 * 1024
BUNDLE_TOTAL_EVENT_BYTES = 128 * 1024 * 1024
BUNDLE_LINE_MAX_BYTES = 1024 * 1024
BUNDLE_MEMBER_LIMIT = 64
BUNDLE_ARCHIVE_MEMBER_LIMIT = 1024
BUNDLE_RECORD_LIMIT = 1_000_000
BUNDLE_MANIFEST_MAX_BYTES = 1024 * 1024
BUNDLE_DIAGNOSTIC_RECORD_LIMIT = 20_000
BUNDLE_CONTEXT_RECORD_LIMIT = 500


def load_bundle(bundle_path):
    path = pathlib.Path(bundle_path)
    if not path.is_file():
        raise UsageError("support bundle does not exist: %s" % path)
    records = []
    manifest = None
    malformed = 0
    selected_members = 0
    selected_bytes = 0
    archive_members = 0
    total_records = 0
    lifecycle_records = 0
    context_truncated = False
    try:
        with tarfile.open(path, "r:gz") as archive:
            for member in archive:
                archive_members += 1
                if archive_members > BUNDLE_ARCHIVE_MEMBER_LIMIT:
                    raise UsageError(
                        "support bundle has too many archive members"
                    )
                member_path = pathlib.PurePosixPath(member.name)
                if (
                    member_path.is_absolute()
                    or ".." in member_path.parts
                    or not member.isfile()
                ):
                    continue
                is_manifest = member_path == pathlib.PurePosixPath(
                    "manifest.json"
                )
                is_event_file = (
                    len(member_path.parts) == 2
                    and member_path.parts[0] == "events"
                    and member_path.suffix == ".jsonl"
                )
                if not is_manifest and not is_event_file:
                    continue
                selected_members += 1
                selected_bytes += member.size
                if selected_members > BUNDLE_MEMBER_LIMIT:
                    raise UsageError(
                        "support bundle has too many diagnostic members"
                    )
                if selected_bytes > BUNDLE_TOTAL_EVENT_BYTES:
                    raise UsageError(
                        "support bundle diagnostic data exceeds the uncompressed limit"
                    )
                if member.size > BUNDLE_MEMBER_MAX_BYTES:
                    raise UsageError(
                        "support bundle member is too large: %s" % member.name
                    )
                if is_manifest and member.size > BUNDLE_MANIFEST_MAX_BYTES:
                    raise UsageError(
                        "support bundle manifest exceeds the size limit"
                    )
                source = archive.extractfile(member)
                if source is None:
                    raise UsageError(
                        "support bundle member is unreadable: %s" % member.name
                    )
                if is_manifest:
                    manifest = json.load(source)
                    continue
                while True:
                    raw_line = source.readline(BUNDLE_LINE_MAX_BYTES + 1)
                    if not raw_line:
                        break
                    if len(raw_line) > BUNDLE_LINE_MAX_BYTES:
                        raise UsageError(
                            "support bundle event record exceeds the line limit"
                        )
                    total_records += 1
                    if total_records > BUNDLE_RECORD_LIMIT:
                        raise UsageError(
                            "support bundle exceeds the event record limit"
                        )
                    try:
                        record = json.loads(raw_line)
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        malformed += 1
                        continue
                    if not isinstance(record, dict):
                        malformed += 1
                        continue
                    primary = (
                        record.get("level") in ("warn", "error")
                        or record.get("exception")
                        or "\n" in str(record.get("_msg", ""))
                    )
                    lifecycle = record.get("event") in LIFECYCLE_EVENTS
                    if lifecycle:
                        lifecycle_records += 1
                        if lifecycle_records > BUNDLE_CONTEXT_RECORD_LIMIT:
                            context_truncated = True
                            lifecycle = False
                    if primary or lifecycle:
                        if len(records) >= BUNDLE_DIAGNOSTIC_RECORD_LIMIT:
                            raise UsageError(
                                "support bundle exceeds the diagnostic record limit"
                            )
                        records.append(record)
    except (OSError, tarfile.TarError, json.JSONDecodeError) as exc:
        raise UsageError("support bundle is unreadable: %s" % exc) from exc
    if manifest is None:
        raise UsageError("support bundle has no manifest.json")
    if not isinstance(manifest, dict):
        raise UsageError("support bundle manifest is not an object")
    return manifest, records, malformed, total_records, context_truncated


def validate_bundle_manifest(manifest):
    if not isinstance(manifest, dict):
        raise UsageError("support bundle manifest is not an object")
    warnings = manifest.get("warnings", [])
    if not isinstance(warnings, list) or not all(
        isinstance(warning, str) for warning in warnings
    ):
        raise UsageError("support bundle manifest has invalid warnings")
    event_files = manifest.get("event_files", {})
    if not isinstance(event_files, dict) or not all(
        isinstance(name, str) and isinstance(details, dict)
        for name, details in event_files.items()
    ):
        raise UsageError("support bundle manifest has invalid event_files")


def cmd_bundle(args, _vl_url):
    manifest, diagnostics, malformed, total_records, context_truncated = (
        load_bundle(args.path)
    )
    validate_bundle_manifest(manifest)
    print("support bundle: %s" % args.path)
    print("created: %s" % manifest.get("created_utc", "?"))
    print("cutoff: %s" % manifest.get("cutoff_utc", "?"))
    print("software: %s" % manifest.get("software_version", "?"))
    warnings = list(manifest.get("warnings", []))
    if manifest.get("klippy_log_truncated"):
        warnings.append("klippy.log is truncated")
    for name, details in manifest.get("event_files", {}).items():
        if details.get("scan_truncated"):
            warnings.append("%s scan is truncated" % name)
        count = details.get("malformed", 0)
        if count:
            warnings.append("%s contains %s malformed records" % (name, count))
    if malformed:
        warnings.append(
            "%s bundled event records could not be decoded" % malformed
        )
    if warnings:
        print("evidence: INCOMPLETE")
        for warning in warnings:
            print("warning: %s" % warning)
    else:
        print("evidence: complete for the bundle window")
    session_ids = sorted(
        {
            record.get("session_id")
            for record in diagnostics
            if isinstance(record.get("session_id"), str)
            and record.get("session_id")
            and record.get("session_id") != "__unbound__"
        }
    )
    print("sessions: %s" % (", ".join(session_ids) if session_ids else "none"))
    diagnostics.sort(key=lambda record: str(record.get("_time", "")))
    print("")
    print(
        "diagnostic records: %d selected from %d"
        % (len(diagnostics), total_records)
    )
    if context_truncated:
        print(
            "warning: lifecycle context limited to %d records"
            % BUNDLE_CONTEXT_RECORD_LIMIT
        )
    for line in render_records(diagnostics):
        print(line)
    return 0


def fetch_records(vl_url, query, limit):
    endpoint = vl_url.rstrip("/") + "/select/logsql/query"
    payload = urllib.parse.urlencode(
        {"query": query, "limit": str(limit)}
    ).encode("ascii")
    request = urllib.request.Request(endpoint, data=payload, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            body = response.read().decode("utf-8", errors="replace")
    except (urllib.error.URLError, OSError) as exc:
        raise VlUnreachableError(vl_url, str(exc)) from exc
    records = []
    for line in body.splitlines():
        line = line.strip()
        if not line:
            continue
        records.append(json.loads(line))
    return records


def check_health(vl_url):
    endpoint = vl_url.rstrip("/") + "/health"
    try:
        with urllib.request.urlopen(endpoint, timeout=5) as response:
            body = response.read().decode("utf-8", errors="replace").strip()
    except (urllib.error.URLError, OSError) as exc:
        raise VlUnreachableError(vl_url, str(exc)) from exc
    if body != "OK":
        raise VlUnreachableError(vl_url, "unexpected /health body %r" % (body,))
    return True


def parse_time(record_time):
    normalized = record_time.replace("Z", "+00:00")
    if "." in normalized:
        head, rest = normalized.split(".", 1)
        frac, tz = rest[: rest.index("+")], rest[rest.index("+") :]
        frac = (frac + "000000")[:6]
        normalized = head + "." + frac + tz
    return datetime.datetime.fromisoformat(normalized)


def truncate(value, limit=80):
    text = str(value)
    if len(text) <= limit:
        return text
    return text[: limit - 1] + "…"


def format_extra_fields(record):
    extras = {
        k: v for k, v in record.items() if k not in KNOWN_FIELDS and k != "code"
    }
    parts = []
    for key in sorted(extras):
        parts.append("%s=%s" % (key, truncate(extras[key])))
    return parts


def spans_multiple_days(records):
    days = set()
    for record in records:
        raw_time = record.get("_time")
        if not raw_time:
            continue
        days.add(raw_time[:10])
    return len(days) > 1


def clock_of(dt):
    return "%02d:%02d:%02d.%03d" % (
        dt.hour,
        dt.minute,
        dt.second,
        dt.microsecond // 1000,
    )


def day_prefix_of(dt):
    return "%02d-%02d" % (dt.month, dt.day)


def time_column(raw_time, show_day):
    if not raw_time:
        return "??:??:??.???"
    dt = parse_time(raw_time)
    time_col = clock_of(dt)
    if show_day:
        time_col = "%s %s" % (day_prefix_of(dt), time_col)
    return time_col


def format_record_body(record, event_suffix="", force_extra=None):
    level = record.get("level", "info")
    letter = LEVEL_LETTERS.get(level, "?")
    source = record.get("source", "?")
    subsystem = record.get("subsystem", "?")
    event = record.get("event", "")
    msg = record.get("_msg", "")

    where = "%s/%s" % (source, subsystem)
    if event:
        where += " %s%s" % (event, event_suffix)
    elif event_suffix:
        where += " %s" % event_suffix.strip()

    body = "%s %s | %s" % (letter, where, msg)

    include_extra = force_extra
    if include_extra is None:
        include_extra = level in ("warn", "error") or event in LIFECYCLE_EVENTS
    if include_extra:
        extras = format_extra_fields(record)
        if extras:
            body += " [%s]" % " ".join(extras)
    return body


def format_record_line(record, show_day, force_extra=None):
    time_col = time_column(record.get("_time", ""), show_day)
    return "%s %s" % (
        time_col,
        format_record_body(record, force_extra=force_extra),
    )


def condense_key(record):
    event = record.get("event", "")
    level = record.get("level", "info")
    if event:
        return (
            record.get("source", "?"),
            record.get("subsystem", "?"),
            event,
            level,
            True,
        )
    return (
        record.get("source", "?"),
        record.get("subsystem", "?"),
        record.get("_msg", ""),
        level,
        False,
    )


def condense_records(records):
    groups = []
    for record in records:
        event = record.get("event", "")
        if event in LIFECYCLE_EVENTS:
            groups.append([record])
            continue
        key = condense_key(record)
        if (
            groups
            and groups[-1][0] is not None
            and condense_key(groups[-1][0]) == key
        ):
            groups[-1].append(record)
        else:
            groups.append([record])
    return groups


def format_condensed_group(group, show_day):
    first = group[0]
    if len(group) == 1:
        return format_record_line(first, show_day)

    last = group[-1]
    event = first.get("event", "")
    suffix = " ×%d" % len(group)
    body = format_record_body(first, event_suffix=suffix if event else "")
    if not event:
        pipe_idx = body.index("|")
        body = body[:pipe_idx] + suffix.strip() + " " + body[pipe_idx:]

    first_dt = parse_time(first["_time"]) if first.get("_time") else None
    last_dt = parse_time(last["_time"]) if last.get("_time") else None
    if first_dt is not None and last_dt is not None:
        time_range = "%s–%s" % (clock_of(first_dt), clock_of(last_dt))
        if show_day:
            time_range = "%s %s" % (day_prefix_of(first_dt), time_range)
    else:
        time_range = "??:??:??.???"

    return "%s %s" % (time_range, body)


def render_records(records):
    if not records:
        return ["0 records matched"]
    ordered = sorted(records, key=lambda r: r.get("_time", ""))
    show_day = spans_multiple_days(ordered)
    groups = condense_records(ordered)
    return [format_condensed_group(group, show_day) for group in groups]


def format_stats_record(record):
    parts = []
    for key in sorted(record):
        parts.append("%s=%s" % (key, truncate(record[key])))
    return " ".join(parts)


def render_stats_table(records, columns):
    if not records:
        return ["0 records matched"]
    widths = {c: len(c) for c in columns}
    for record in records:
        for c in columns:
            widths[c] = max(widths[c], len(str(record.get(c, ""))))
    lines = []
    header = "  ".join(c.ljust(widths[c]) for c in columns)
    lines.append(header)
    lines.append("  ".join("-" * widths[c] for c in columns))
    for record in records:
        lines.append(
            "  ".join(str(record.get(c, "")).ljust(widths[c]) for c in columns)
        )
    return lines


def format_duration_s(seconds):
    seconds = int(seconds)
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    if hours:
        return "%dh%dm%ds" % (hours, minutes, seconds)
    if minutes:
        return "%dm%ds" % (minutes, seconds)
    return "%ds" % seconds


def seconds_between(start_iso, end_iso):
    return (parse_time(end_iso) - parse_time(start_iso)).total_seconds()


def build_prints_table(span_records, end_records):
    ends_by_print = {}
    for record in end_records:
        print_id = record.get("print_id", "")
        if print_id:
            ends_by_print[print_id] = record

    rows = []
    for span in span_records:
        print_id = span.get("print_id", "")
        end = ends_by_print.get(print_id)
        if end is not None and end.get("duration_s") not in (None, ""):
            duration = format_duration_s(float(end["duration_s"]))
        else:
            duration = format_duration_s(
                seconds_between(span["first"], span["last"])
            )
        outcome = end.get("outcome", "?") if end is not None else "?"
        reason = truncate(end.get("reason", ""), 40) if end is not None else ""
        rows.append(
            {
                "print_id": print_id,
                "started": span.get("first", ""),
                "duration": duration,
                "outcome": outcome,
                "reason": reason,
            }
        )
    rows.sort(key=lambda r: r["started"], reverse=True)
    return rows


def resolve_last_print_id(span_records):
    if not span_records:
        return None
    newest = max(span_records, key=lambda r: r.get("last", ""))
    return newest.get("print_id")


def dedupe_resolve_records(records):
    seen = {}
    for record in records:
        key = (record.get("code_name", ""), record.get("event", ""))
        if key not in seen:
            seen[key] = record.get("_msg", "")
    return seen


def resolve_vl_url(cli_value):
    import os

    if cli_value:
        return cli_value
    return os.environ.get("KALICO_VL", DEFAULT_VL_URL)


def require_health(vl_url):
    try:
        check_health(vl_url)
    except VlUnreachableError as exc:
        print(
            PIPELINE_DOWN_MESSAGE.format(url=exc.url, detail=exc.detail),
            file=sys.stderr,
        )
        return False
    return True


def cmd_health(args, vl_url):
    if not require_health(vl_url):
        return 2
    print("VL reachable: yes (%s)" % vl_url)
    records = fetch_records(vl_url, build_health_query(), 1)
    if not records:
        print("no heartbeat in 10m — host not logging or Vector stalled")
        return 0
    last = records[0]
    age = (
        datetime.datetime.now(datetime.timezone.utc) - parse_time(last["_time"])
    ).total_seconds()
    print("last heartbeat: %.1fs ago" % age)
    if age > 60:
        print("WARNING: heartbeat is stale (>60s) — host may not be logging")
    return 0


def cmd_sessions(args, vl_url):
    if not require_health(vl_url):
        return 2
    records = fetch_records(vl_url, build_sessions_query(args.since), args.n)
    for r in records:
        r["session_id"] = r.get("session_id", "")
    for line in render_stats_table(
        records, ["session_id", "first", "last", "hits"]
    ):
        print(line)
    return 0


def cmd_prints(args, vl_url):
    if not require_health(vl_url):
        return 2
    span_records = fetch_records(
        vl_url, build_prints_span_query(args.since), 10000
    )
    end_records = fetch_records(
        vl_url, build_prints_end_query(args.since), 10000
    )
    rows = build_prints_table(span_records, end_records)[: args.n]
    for line in render_stats_table(
        rows, ["print_id", "started", "duration", "outcome", "reason"]
    ):
        print(line)
    return 0


def fetch_investigation_records(
    vl_url, scope_field, scope_value, since, include_info
):
    lifecycle = fetch_records(
        vl_url, build_lifecycle_query(scope_field, scope_value, since), 10000
    )
    warn_error = fetch_records(
        vl_url, build_warn_error_query(scope_field, scope_value, since), 10000
    )
    records = lifecycle + warn_error
    if include_info:
        info = fetch_records(
            vl_url, build_info_query(scope_field, scope_value, since), 10000
        )
        records += info
    deduped = {}
    for r in records:
        key = (r.get("_time"), r.get("_msg"), r.get("event"))
        deduped[key] = r
    return list(deduped.values())


def print_investigation_header(records, scope_label, scope_value):
    session_ids = {r.get("session_id") for r in records if r.get("session_id")}
    print_ids = {r.get("print_id") for r in records if r.get("print_id")}
    start = next(
        (r for r in records if r.get("event") == EVENT_PRINT_START), None
    )
    end = next((r for r in records if r.get("event") == EVENT_PRINT_END), None)

    print("%s: %s" % (scope_label, scope_value))
    if session_ids:
        print("session_id(s): %s" % ", ".join(sorted(session_ids)))
    if print_ids and scope_label != "print_id":
        print("print_id(s): %s" % ", ".join(sorted(print_ids)))
    if start is not None:
        print("file: %s" % start.get("file", "?"))
        print("start: %s" % start.get("_time", "?"))
    if end is not None:
        print("end: %s" % end.get("_time", "?"))
        print(
            "outcome=%s reason=%s duration_s=%s"
            % (
                end.get("outcome", "?"),
                end.get("reason", "?"),
                end.get("duration_s", "?"),
            )
        )
    elif scope_label == "print_id":
        print(
            "no print.end recorded — the print never finished cleanly (host died, or still running)"
        )
    print("")


def cmd_print(args, vl_url):
    if not require_health(vl_url):
        return 2
    print_id = args.id
    if print_id == "last":
        span_records = fetch_records(
            vl_url, build_prints_span_query(args.since), 10000
        )
        print_id = resolve_last_print_id(span_records)
        if print_id is None:
            print("0 records matched")
            return 0
    records = fetch_investigation_records(
        vl_url, "print_id", print_id, args.since, args.all
    )
    print_investigation_header(records, "print_id", print_id)
    ordered = sorted(records, key=lambda r: r.get("_time", ""))
    lines = render_records(ordered)
    for line in lines:
        print(line)
    print("")
    print("%d records matched, %d lines shown" % (len(ordered), len(lines)))
    return 0


def cmd_session(args, vl_url):
    if not require_health(vl_url):
        return 2
    records = fetch_investigation_records(
        vl_url, "session_id", args.id, args.since, args.all
    )
    print_investigation_header(records, "session_id", args.id)
    ordered = sorted(records, key=lambda r: r.get("_time", ""))
    lines = render_records(ordered)
    for line in lines:
        print(line)
    print("")
    print("%d records matched, %d lines shown" % (len(ordered), len(lines)))
    return 0


def cmd_tail(args, vl_url):
    if not require_health(vl_url):
        return 2
    levels = levels_at_or_above(args.level)
    records = fetch_records(vl_url, build_tail_query(args.since, levels), 200)
    ordered = sorted(records, key=lambda r: r.get("_time", ""))
    for line in render_records(ordered):
        print(line)
    return 0


def cmd_schema(args, vl_url):
    if not require_health(vl_url):
        return 2
    print(
        "live from %s, window %s — this is the actual schema, docs may lag"
        % (vl_url, args.since)
    )
    print("")
    print("-- source/subsystem counts --")
    source_records = fetch_records(
        vl_url, build_schema_source_query(args.since), 30
    )
    for line in render_stats_table(
        source_records, ["source", "subsystem", "n"]
    ):
        print(line)
    print("")
    print("-- top events --")
    event_records = fetch_records(
        vl_url, build_schema_event_query(args.since), 30
    )
    for line in render_stats_table(
        event_records, ["source", "event", "level", "n"]
    ):
        print(line)
    print("")
    print("-- level counts --")
    level_records = fetch_records(
        vl_url, build_schema_level_query(args.since), 30
    )
    for line in render_stats_table(level_records, ["level", "n"]):
        print(line)
    return 0


def cmd_resolve(args, vl_url):
    if not require_health(vl_url):
        return 2
    records = fetch_records(
        vl_url, build_resolve_query(args.code, args.since), 200
    )
    if not records:
        print(
            "code %s not seen in %s; canonical table: rust/runtime/src/log_codes.rs"
            % (args.code, args.since)
        )
        return 0
    mapping = dedupe_resolve_records(records)
    for (code_name, event), sample_msg in sorted(mapping.items()):
        print("%s / %s: %s" % (code_name or "?", event or "?", sample_msg))
    return 0


def cmd_q(args, vl_url):
    if not require_health(vl_url):
        return 2
    if "_time:" not in args.query:
        print("warning: query has no _time: bound", file=sys.stderr)
    records = fetch_records(vl_url, args.query, args.limit)
    if args.raw:
        if not records:
            print("0 records matched")
            return 0
        for record in records:
            print(json.dumps(record, sort_keys=True))
        return 0
    if not records:
        print("0 records matched")
        return 0
    has_time = all("_time" in r for r in records)
    if has_time:
        ordered = sorted(records, key=lambda r: r.get("_time", ""))
        show_day = spans_multiple_days(ordered)
        for record in ordered:
            print(format_record_line(record, show_day, force_extra=True))
    else:
        for record in records:
            print(format_stats_record(record))
    return 0


def add_since_argument(parser, default):
    parser.add_argument(
        "--since",
        default=default,
        help="relative time bound, e.g. 24h (default %(default)s)",
    )


def build_parser():
    parser = argparse.ArgumentParser(prog="logq.py", description=__doc__)
    parser.add_argument(
        "--vl",
        default=None,
        help="VictoriaLogs base URL (default: $KALICO_VL or %s)"
        % DEFAULT_VL_URL,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    p_health = subparsers.add_parser(
        "health", help="check VL reachability and heartbeat freshness"
    )
    p_health.set_defaults(func=cmd_health)

    p_sessions = subparsers.add_parser("sessions", help="list recent sessions")
    add_since_argument(p_sessions, "24h")
    p_sessions.add_argument(
        "-n", type=int, default=20, help="max rows (default 20)"
    )
    p_sessions.set_defaults(func=cmd_sessions)

    p_prints = subparsers.add_parser("prints", help="list recent prints")
    add_since_argument(p_prints, "7d")
    p_prints.add_argument(
        "-n", type=int, default=20, help="max rows (default 20)"
    )
    p_prints.set_defaults(func=cmd_prints)

    p_print = subparsers.add_parser("print", help="investigate a single print")
    p_print.add_argument("id", help="print_id, or 'last' for the most recent")
    add_since_argument(p_print, "7d")
    p_print.add_argument(
        "--all", action="store_true", help="include info records too"
    )
    p_print.set_defaults(func=cmd_print)

    p_session = subparsers.add_parser(
        "session", help="investigate a single session"
    )
    p_session.add_argument("id", help="session_id")
    add_since_argument(p_session, "24h")
    p_session.add_argument(
        "--all", action="store_true", help="include info records too"
    )
    p_session.set_defaults(func=cmd_session)

    p_tail = subparsers.add_parser(
        "tail", help="recent records at or above a level"
    )
    add_since_argument(p_tail, "10m")
    p_tail.add_argument(
        "--level",
        default="warn",
        choices=LEVEL_ORDER,
        help="minimum level (default warn)",
    )
    p_tail.set_defaults(func=cmd_tail)

    p_schema = subparsers.add_parser("schema", help="discover the live schema")
    add_since_argument(p_schema, "24h")
    p_schema.set_defaults(func=cmd_schema)

    p_resolve = subparsers.add_parser(
        "resolve", help="resolve a numeric code to its name/event"
    )
    p_resolve.add_argument("code", help="numeric code")
    add_since_argument(p_resolve, "30d")
    p_resolve.set_defaults(func=cmd_resolve)

    p_q = subparsers.add_parser("q", help="raw LogsQL escape hatch")
    p_q.add_argument("query", help="raw LogsQL query")
    p_q.add_argument("--limit", type=int, default=50)
    p_q.add_argument(
        "--raw", action="store_true", help="print NDJSON lines verbatim"
    )
    p_q.set_defaults(func=cmd_q)

    p_bundle = subparsers.add_parser(
        "bundle", help="summarize a CREATE_SUPPORT_BUNDLE archive"
    )
    p_bundle.add_argument("path", help="path to serval-support-*.tar.gz")
    p_bundle.set_defaults(func=cmd_bundle)

    return parser


def main(argv):
    parser = build_parser()
    args = parser.parse_args(argv)

    if hasattr(args, "since"):
        try:
            validate_since(args.since)
        except UsageError as exc:
            print("error: %s" % exc, file=sys.stderr)
            return 1

    vl_url = resolve_vl_url(args.vl)

    try:
        return args.func(args, vl_url)
    except VlUnreachableError as exc:
        print(
            PIPELINE_DOWN_MESSAGE.format(url=exc.url, detail=exc.detail),
            file=sys.stderr,
        )
        return 2
    except UsageError as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
