import datetime
import importlib.util
import io
import json
import os
import pathlib
import tarfile

import pytest

SUPPORT_BUNDLE_PATH = (
    pathlib.Path(__file__).resolve().parent.parent
    / "klippy"
    / "extras"
    / "support_bundle.py"
)
spec = importlib.util.spec_from_file_location(
    "support_bundle", SUPPORT_BUNDLE_PATH
)
support_bundle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(support_bundle)


def event(timestamp, name):
    return (
        json.dumps(
            {
                "_time": datetime.datetime.fromtimestamp(
                    timestamp, datetime.timezone.utc
                ).isoformat(),
                "event": name,
            }
        )
        + "\n"
    )


@pytest.mark.parametrize(
    ("value", "seconds"),
    [("1m", 60), ("30m", 1800), ("2h", 7200), ("24h", 86400)],
)
def test_parse_since(value, seconds):
    assert support_bundle.parse_since(value) == seconds


@pytest.mark.parametrize("value", ["", "30", "0m", "25h", "-1m", "1d"])
def test_parse_since_rejects_invalid_window(value):
    with pytest.raises(support_bundle.SupportBundleError):
        support_bundle.parse_since(value)


def test_create_support_bundle_selects_recent_records(tmp_path):
    now = 2_000_000_000.0
    logs = tmp_path / "logs"
    events = logs / "events"
    events.mkdir(parents=True)
    (logs / "klippy.log").write_text("klippy evidence\n", encoding="utf-8")
    (events / "host-rust.jsonl").write_text(
        event(now - 3600, "old") + "not-json\n" + event(now - 60, "fatal"),
        encoding="utf-8",
    )

    archive_path = support_bundle.create_support_bundle(
        logs / "klippy.log",
        events,
        logs,
        1800,
        now=now,
        journal_reader=lambda cutoff: b"journal evidence\n",
        software_version="test-version",
    )

    with tarfile.open(archive_path, "r:gz") as archive:
        assert set(archive.getnames()) == {
            "events",
            "events/host-rust.jsonl",
            "klipper-journal.log",
            "klippy.log",
            "manifest.json",
        }
        selected = archive.extractfile("events/host-rust.jsonl").read()
        assert b'"fatal"' in selected
        assert b'"old"' not in selected
        manifest = json.load(
            io.TextIOWrapper(
                archive.extractfile("manifest.json"), encoding="utf-8"
            )
        )

    assert manifest["software_version"] == "test-version"
    assert manifest["event_files"]["host-rust.jsonl"] == {
        "malformed": 1,
        "selected": 1,
    }
    assert manifest["warnings"] == []


def test_create_support_bundle_includes_only_latest_core_on_request(tmp_path):
    now = 2_000_000_000.0
    logs = tmp_path / "logs"
    events = logs / "events"
    cores = logs / "coredumps"
    events.mkdir(parents=True)
    cores.mkdir()
    (logs / "klippy.log").write_bytes(b"log\n")
    old_core = cores / "core.klippy.old"
    latest_core = cores / "core.klippy.latest"
    old_core.write_bytes(b"old core")
    latest_core.write_bytes(b"latest core")
    os.utime(old_core, (now - 3600, now - 3600))
    os.utime(latest_core, (now - 30, now - 30))

    archive_path = support_bundle.create_support_bundle(
        logs / "klippy.log",
        events,
        logs,
        60,
        now=now,
        journal_reader=lambda cutoff: b"",
        core_dir=cores,
        include_core=True,
    )

    with tarfile.open(archive_path, "r:gz") as archive:
        assert "coredumps/core.klippy.latest" in archive.getnames()
        assert "coredumps/core.klippy.old" not in archive.getnames()
        manifest = json.load(
            io.TextIOWrapper(
                archive.extractfile("manifest.json"), encoding="utf-8"
            )
        )
    assert manifest["latest_core"] == {
        "included": True,
        "name": "core.klippy.latest",
        "size": 11,
    }


def test_create_support_bundle_records_unavailable_journal(tmp_path):
    logs = tmp_path / "logs"
    events = logs / "events"
    events.mkdir(parents=True)
    (logs / "klippy.log").write_bytes(b"log\n")

    def unavailable_journal(cutoff):
        raise support_bundle.SupportBundleError("journal unavailable")

    archive_path = support_bundle.create_support_bundle(
        logs / "klippy.log",
        events,
        logs,
        60,
        now=2_000_000_000.0,
        journal_reader=unavailable_journal,
    )

    with tarfile.open(archive_path, "r:gz") as archive:
        manifest = json.load(
            io.TextIOWrapper(
                archive.extractfile("manifest.json"), encoding="utf-8"
            )
        )
        assert "klipper-journal.log" not in archive.getnames()
    assert manifest["warnings"] == ["journal unavailable"]
