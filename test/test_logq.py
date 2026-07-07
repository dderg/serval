import importlib.util
import pathlib

import pytest

LOGQ_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / "scripts" / "logq.py"
)
PRINT_STATS_PATH = (
    pathlib.Path(__file__).resolve().parent.parent
    / "klippy"
    / "extras"
    / "print_stats.py"
)
LOG_OBSERVABILITY_PATH = (
    pathlib.Path(__file__).resolve().parent.parent
    / "klippy"
    / "extras"
    / "log_observability.py"
)

spec = importlib.util.spec_from_file_location("logq", LOGQ_PATH)
logq = importlib.util.module_from_spec(spec)
spec.loader.exec_module(logq)


def _rec(
    time, level="info", source="host", subsystem="print", event="", msg=""
):
    return {
        "_time": time,
        "level": level,
        "source": source,
        "subsystem": subsystem,
        "event": event,
        "_msg": msg,
    }


class TestLifecycleContractCrossCheck:
    def test_lifecycle_events_appear_in_print_stats_source(self):
        source = PRINT_STATS_PATH.read_text()
        events = set(logq.LIFECYCLE_EVENTS) | {
            logq.EVENT_PRINT_RESUME,
            logq.EVENT_PRINT_PAUSE,
            logq.EVENT_PRINT_START,
            logq.EVENT_PRINT_END,
        }
        for event in events:
            assert '"%s"' % event in source, (
                "lifecycle event %r no longer emitted from print_stats.py "
                "as a string literal — logq.py's LIFECYCLE_EVENTS is now "
                "out of sync with the emitter" % (event,)
            )

    def test_heartbeat_event_appears_in_log_observability_source(self):
        source = LOG_OBSERVABILITY_PATH.read_text()
        assert '"%s"' % logq.EVENT_HEARTBEAT in source


class TestValidateSince:
    @pytest.mark.parametrize("value", ["10m", "24h", "7d", "30s", "2w"])
    def test_accepts_valid_values(self, value):
        assert logq.validate_since(value) == value

    @pytest.mark.parametrize("value", ["bogus", "1x", "-5m", "h", "", "1.5h"])
    def test_rejects_invalid_values(self, value):
        with pytest.raises(logq.UsageError):
            logq.validate_since(value)


class TestLevelsAtOrAbove:
    def test_warn_includes_warn_and_error_only(self):
        assert logq.levels_at_or_above("warn") == ("warn", "error")

    def test_trace_includes_all_five_levels(self):
        assert logq.levels_at_or_above("trace") == (
            "trace",
            "debug",
            "info",
            "warn",
            "error",
        )

    def test_unknown_level_is_rejected(self):
        with pytest.raises(logq.UsageError):
            logq.levels_at_or_above("catastrophic")


class TestQueryBuilders:
    def test_health_query(self):
        query = logq.build_health_query()
        assert "_time:10m" in query
        assert "subsystem:=observability" in query
        assert "event:=heartbeat" in query

    def test_sessions_query(self):
        query = logq.build_sessions_query("24h")
        assert "_time:24h" in query
        assert "stats by (session_id)" in query
        assert "count() as hits" in query
        assert "min(_time) as first" in query
        assert "max(_time) as last" in query

    def test_prints_span_query(self):
        query = logq.build_prints_span_query("7d")
        assert "_time:7d" in query
        assert 'print_id:!=""' in query
        assert "min(_time) as first" in query
        assert "max(_time) as last" in query

    def test_prints_end_query(self):
        query = logq.build_prints_end_query("7d")
        assert "_time:7d" in query
        assert "event:=print.end" in query
        assert "fields _time, print_id, outcome, reason, duration_s" in query

    def test_lifecycle_query(self):
        query = logq.build_lifecycle_query("print_id", "abc123", "7d")
        assert "_time:7d" in query
        assert "print_id:=abc123" in query
        assert "subsystem:=print_stats" in query

    def test_warn_error_query(self):
        query = logq.build_warn_error_query("session_id", "k-1", "24h")
        assert "_time:24h" in query
        assert "session_id:=k-1" in query
        assert "level:in(warn,error)" in query

    def test_resolve_query(self):
        query = logq.build_resolve_query("42", "30d")
        assert "_time:30d" in query
        assert "code:=42" in query
        assert "fields code_name, event, _msg" in query

    def test_aggregate_queries_alias_their_count(self):
        assert "count() as n" in logq.build_schema_source_query("24h")
        assert "count() as n" in logq.build_schema_event_query("24h")
        assert "count() as n" in logq.build_schema_level_query("24h")


class TestCondenseRecords:
    def test_consecutive_identical_records_collapse_to_one_group(self):
        records = [
            _rec("2026-07-06T15:19:28.000Z", event="motion.tick"),
            _rec("2026-07-06T15:19:29.000Z", event="motion.tick"),
            _rec("2026-07-06T15:19:30.000Z", event="motion.tick"),
        ]
        groups = logq.condense_records(records)
        assert len(groups) == 1
        assert len(groups[0]) == 3
        line = logq.format_condensed_group(groups[0], show_day=False)
        assert "×3" in line
        assert "15:19:28.000–15:19:30.000" in line

    def test_non_consecutive_repeats_do_not_collapse(self):
        records = [
            _rec("2026-07-06T15:19:28.000Z", event="motion.tick"),
            _rec("2026-07-06T15:19:29.000Z", event="motion.other"),
            _rec("2026-07-06T15:19:30.000Z", event="motion.tick"),
        ]
        groups = logq.condense_records(records)
        assert len(groups) == 3
        assert all(len(g) == 1 for g in groups)

    def test_empty_event_records_group_by_msg(self):
        records = [
            _rec("2026-07-06T15:19:28.000Z", event="", msg="same message"),
            _rec("2026-07-06T15:19:29.000Z", event="", msg="same message"),
            _rec("2026-07-06T15:19:30.000Z", event="", msg="different message"),
        ]
        groups = logq.condense_records(records)
        assert len(groups) == 2
        assert len(groups[0]) == 2
        assert len(groups[1]) == 1

    def test_lifecycle_events_never_condense(self):
        records = [
            _rec(
                "2026-07-06T15:19:28.000Z",
                event=logq.EVENT_PRINT_START,
                subsystem="print_stats",
            ),
            _rec(
                "2026-07-06T15:19:29.000Z",
                event=logq.EVENT_PRINT_START,
                subsystem="print_stats",
            ),
        ]
        groups = logq.condense_records(records)
        assert len(groups) == 2
        assert all(len(g) == 1 for g in groups)

    def test_ordering_is_preserved_oldest_first(self):
        records = [
            _rec("2026-07-06T15:19:28.000Z", event="a"),
            _rec("2026-07-06T15:19:29.000Z", event="b"),
            _rec("2026-07-06T15:19:30.000Z", event="c"),
        ]
        groups = logq.condense_records(records)
        assert [g[0]["event"] for g in groups] == ["a", "b", "c"]


class TestFormatRecordLine:
    def test_single_record_renders_expected_shape(self):
        record = _rec(
            "2026-07-06T15:19:28.043Z",
            level="warn",
            source="host",
            subsystem="print_stats",
            event="print.pause",
            msg="print paused",
        )
        line = logq.format_record_line(record, show_day=False)
        assert line.startswith(
            "15:19:28.043 W host/print_stats print.pause | print paused"
        )

    def test_extra_fields_are_shown_sorted_for_warn_records(self):
        record = _rec(
            "2026-07-06T15:19:28.043Z",
            level="warn",
            event="",
            msg="something happened",
        )
        record["zeta"] = "z"
        record["alpha"] = "a"
        line = logq.format_record_line(record, show_day=False)
        assert "[alpha=a zeta=z]" in line

    def test_extra_fields_are_omitted_for_plain_info_records(self):
        record = _rec(
            "2026-07-06T15:19:28.043Z",
            level="info",
            event="",
            msg="something happened",
        )
        record["extra"] = "value"
        line = logq.format_record_line(record, show_day=False)
        assert "extra=value" not in line
        assert "[" not in line

    def test_long_values_are_truncated_with_ellipsis(self):
        long_value = "x" * 200
        assert logq.truncate(long_value) == "x" * 79 + "…"
        assert len(logq.truncate(long_value)) == 80

    def test_multi_day_record_sets_get_day_prefix(self):
        records = [
            _rec("2026-07-05T23:00:00.000Z", event="a"),
            _rec("2026-07-06T01:00:00.000Z", event="b"),
        ]
        assert logq.spans_multiple_days(records) is True
        line = logq.format_record_line(records[0], show_day=True)
        assert line.startswith("07-05 23:00:00.000")

    def test_single_day_record_sets_have_no_day_prefix(self):
        records = [
            _rec("2026-07-06T15:19:28.000Z", event="a"),
            _rec("2026-07-06T16:19:28.000Z", event="b"),
        ]
        assert logq.spans_multiple_days(records) is False
        line = logq.format_record_line(records[0], show_day=False)
        assert line.startswith("15:19:28.000")


class TestBuildPrintsTable:
    def test_outcome_and_duration_taken_from_print_end_when_present(self):
        span_records = [
            {
                "print_id": "p1",
                "first": "2026-07-06T10:00:00.000Z",
                "last": "2026-07-06T10:05:00.000Z",
            }
        ]
        end_records = [
            {
                "print_id": "p1",
                "outcome": "complete",
                "reason": "",
                "duration_s": 252,
            }
        ]
        rows = logq.build_prints_table(span_records, end_records)
        assert len(rows) == 1
        assert rows[0]["outcome"] == "complete"
        assert rows[0]["duration"] == "4m12s"

    def test_outcome_and_duration_fall_back_when_no_print_end(self):
        span_records = [
            {
                "print_id": "p2",
                "first": "2026-07-06T10:00:00.000Z",
                "last": "2026-07-06T10:02:00.000Z",
            }
        ]
        rows = logq.build_prints_table(span_records, [])
        assert len(rows) == 1
        assert rows[0]["outcome"] == "?"
        assert rows[0]["duration"] == "2m0s"

    def test_rows_are_sorted_newest_first(self):
        span_records = [
            {
                "print_id": "old",
                "first": "2026-07-06T08:00:00.000Z",
                "last": "2026-07-06T08:01:00.000Z",
            },
            {
                "print_id": "new",
                "first": "2026-07-06T12:00:00.000Z",
                "last": "2026-07-06T12:01:00.000Z",
            },
        ]
        rows = logq.build_prints_table(span_records, [])
        assert [r["print_id"] for r in rows] == ["new", "old"]


class TestFormatDurationS:
    def test_seconds_only(self):
        assert logq.format_duration_s(12) == "12s"

    def test_minutes_and_seconds(self):
        assert logq.format_duration_s(252) == "4m12s"

    def test_hours_minutes_seconds(self):
        assert logq.format_duration_s(3725) == "1h2m5s"


class TestCommandLevelSmoke:
    def _patch_healthy(self, monkeypatch):
        monkeypatch.setattr(logq, "check_health", lambda vl_url: True)

    def test_health_command_reports_reachable(self, monkeypatch, capsys):
        self._patch_healthy(monkeypatch)
        monkeypatch.setattr(
            logq,
            "fetch_records",
            lambda vl_url, query, limit: [
                _rec(
                    "2026-07-06T15:19:28.000Z",
                    source="host",
                    subsystem="observability",
                    event="heartbeat",
                )
            ],
        )
        rc = logq.main(["health"])
        assert rc == 0
        out = capsys.readouterr().out
        assert "VL reachable: yes" in out
        assert "last heartbeat:" in out

    def test_sessions_command_renders_table(self, monkeypatch, capsys):
        self._patch_healthy(monkeypatch)
        monkeypatch.setattr(
            logq,
            "fetch_records",
            lambda vl_url, query, limit: [
                {
                    "session_id": "k-1",
                    "first": "2026-07-06T10:00:00.000Z",
                    "last": "2026-07-06T10:05:00.000Z",
                    "hits": 12,
                }
            ],
        )
        rc = logq.main(["sessions"])
        assert rc == 0
        out = capsys.readouterr().out
        assert "k-1" in out
        assert "hits" in out

    def test_sessions_command_reports_zero_records(self, monkeypatch, capsys):
        self._patch_healthy(monkeypatch)
        monkeypatch.setattr(
            logq, "fetch_records", lambda vl_url, query, limit: []
        )
        rc = logq.main(["sessions"])
        assert rc == 0
        out = capsys.readouterr().out
        assert "0 records matched" in out

    def test_print_last_command_investigates_most_recent_print(
        self, monkeypatch, capsys
    ):
        self._patch_healthy(monkeypatch)

        def fake_fetch(vl_url, query, limit):
            if "stats by (print_id)" in query:
                return [
                    {
                        "print_id": "p1",
                        "first": "2026-07-06T10:00:00.000Z",
                        "last": "2026-07-06T10:05:00.000Z",
                    }
                ]
            if "subsystem:=print_stats" in query:
                return [
                    _rec(
                        "2026-07-06T10:00:00.000Z",
                        source="host",
                        subsystem="print_stats",
                        event=logq.EVENT_PRINT_START,
                    )
                ]
            return []

        monkeypatch.setattr(logq, "fetch_records", fake_fetch)
        rc = logq.main(["print", "last"])
        assert rc == 0
        out = capsys.readouterr().out
        assert "print_id: p1" in out
        assert "records matched" in out

    def test_schema_command_renders_all_three_sections(
        self, monkeypatch, capsys
    ):
        self._patch_healthy(monkeypatch)
        monkeypatch.setattr(
            logq,
            "fetch_records",
            lambda vl_url, query, limit: [{"source": "host", "n": 5}],
        )
        rc = logq.main(["schema"])
        assert rc == 0
        out = capsys.readouterr().out
        assert "-- source/subsystem counts --" in out
        assert "-- top events --" in out
        assert "-- level counts --" in out

    def test_unreachable_pipeline_returns_2_and_prints_down_message(
        self, monkeypatch, capsys
    ):
        def raise_unreachable(vl_url):
            raise logq.VlUnreachableError(vl_url, "connection refused")

        monkeypatch.setattr(logq, "check_health", raise_unreachable)
        rc = logq.main(["health"])
        assert rc == 2
        err = capsys.readouterr().err
        assert "structured logging pipeline is down" in err
