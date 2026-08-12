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


def journal_writer(data):
    def write(cutoff, destination):
        destination.write_bytes(data)
        return False

    return write


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
    config = tmp_path / "config"
    config.mkdir()
    (config / "printer.cfg").write_text("[include macros/*.cfg]\n")
    (config / "macros").mkdir()
    (config / "macros" / "print.cfg").write_text("[gcode_macro PRINT_START]\n")
    (events / "host-rust.jsonl").write_text(
        event(now - 3600, "old") + "not-json\n" + event(now - 60, "fatal"),
        encoding="utf-8",
    )
    os.utime(events / "host-rust.jsonl", (now - 30, now - 30))

    archive_path = support_bundle.create_support_bundle(
        logs / "klippy.log",
        events,
        logs,
        1800,
        now=now,
        journal_writer=journal_writer(b"journal evidence\n"),
        software_version="test-version",
        config_file=config / "printer.cfg",
    )

    with tarfile.open(archive_path, "r:gz") as archive:
        assert set(archive.getnames()) == {
            "events",
            "events/host-rust.jsonl",
            "klipper-journal.log",
            "klippy.log",
            "manifest.json",
            "config",
            "config/macros",
            "config/macros/print.cfg",
            "config/printer.cfg",
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
    assert manifest["event_files"]["host-rust.jsonl"]["malformed"] == 1
    assert manifest["event_files"]["host-rust.jsonl"]["selected"] == 1
    assert not manifest["event_files"]["host-rust.jsonl"]["scan_truncated"]
    assert manifest["config_files"] == [
        "config/macros/print.cfg",
        "config/printer.cfg",
    ]
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
        journal_writer=journal_writer(b""),
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

    def unavailable_journal(cutoff, destination):
        raise support_bundle.SupportBundleError("journal unavailable")

    archive_path = support_bundle.create_support_bundle(
        logs / "klippy.log",
        events,
        logs,
        60,
        now=2_000_000_000.0,
        journal_writer=unavailable_journal,
    )

    with tarfile.open(archive_path, "r:gz") as archive:
        manifest = json.load(
            io.TextIOWrapper(
                archive.extractfile("manifest.json"), encoding="utf-8"
            )
        )
        assert "klipper-journal.log" not in archive.getnames()
    assert manifest["warnings"] == ["journal unavailable"]


def test_event_scan_reads_only_bounded_tail(tmp_path):
    now = 2_000_000_000.0
    source = tmp_path / "host-rust.jsonl"
    destination = tmp_path / "selected.jsonl"
    source.write_text(
        event(now - 60, "discarded") + event(now - 30, "selected"),
        encoding="utf-8",
    )
    last_line_bytes = len(event(now - 30, "selected").encode())

    selected, malformed, truncated, scanned = (
        support_bundle.select_event_records(
            source,
            destination,
            now - 60,
            now,
            last_line_bytes + 1,
        )
    )

    assert selected == 1
    assert malformed == 0
    assert truncated
    assert scanned <= last_line_bytes
    assert b'"selected"' in destination.read_bytes()


def test_journal_capture_stops_at_byte_limit(monkeypatch, tmp_path):
    real_popen = support_bundle.subprocess.Popen

    def producing_process(command, **kwargs):
        return real_popen(
            [
                support_bundle.sys.executable,
                "-c",
                "import sys; sys.stdout.buffer.write(b'x' * 1024)",
            ],
            **kwargs,
        )

    monkeypatch.setattr(support_bundle.subprocess, "Popen", producing_process)
    monkeypatch.setattr(support_bundle, "JOURNAL_BYTES", 16)
    destination = tmp_path / "journal.log"

    assert support_bundle.write_klipper_journal(0, destination)
    assert destination.read_bytes() == b"x" * 16


def test_journal_capture_times_out(monkeypatch, tmp_path):
    real_popen = support_bundle.subprocess.Popen

    def stalled_process(command, **kwargs):
        return real_popen(
            [
                support_bundle.sys.executable,
                "-c",
                "import time; time.sleep(10)",
            ],
            **kwargs,
        )

    monkeypatch.setattr(support_bundle.subprocess, "Popen", stalled_process)
    monkeypatch.setattr(support_bundle, "JOURNAL_TIMEOUT_SECONDS", 0.01)

    with pytest.raises(
        support_bundle.SupportBundleError,
        match="journalctl timed out",
    ):
        support_bundle.write_klipper_journal(0, tmp_path / "journal.log")


def test_gcode_command_starts_worker_without_waiting(monkeypatch, tmp_path):
    responses = []
    commands = []

    class FakeGcode:
        def register_command(self, name, handler, desc=None):
            commands.append(name)

        def respond_info(self, message):
            responses.append(message)

    class FakeReactor:
        def register_async_callback(self, callback):
            callback(0.0)

    class FakePrinter:
        def __init__(self):
            self.gcode = FakeGcode()

        def lookup_object(self, name):
            assert name == "gcode"
            return self.gcode

        def get_reactor(self):
            return FakeReactor()

        def get_start_args(self):
            return {
                "log_file": str(tmp_path / "klippy.log"),
                "log_events_dir": str(tmp_path / "events"),
                "software_version": "test",
                "config_file": str(tmp_path / "printer.cfg"),
            }

    class FakeConfig:
        printer = FakePrinter()

        def get_printer(self):
            return self.printer

    class FakeCommand:
        def get(self, name, default):
            return default

        def get_int(self, name, default, minval, maxval):
            return default

        def respond_info(self, message):
            responses.append(message)

        def error(self, message):
            return RuntimeError(message)

    class FakeWorker:
        returncode = 0

        def communicate(self, timeout):
            return (str(tmp_path / "bundle.tar.gz") + "\n", "")

    class ImmediateThread:
        def __init__(self, target, args, daemon):
            self.target = target
            self.args = args

        def start(self):
            self.target(*self.args)

    popen_calls = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return FakeWorker()

    monkeypatch.setattr(support_bundle.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(support_bundle.threading, "Thread", ImmediateThread)

    bundle = support_bundle.SupportBundle(FakeConfig())
    bundle.cmd_CREATE_SUPPORT_BUNDLE(FakeCommand())

    assert commands == ["CREATE_SUPPORT_BUNDLE"]
    assert popen_calls[0][0][1].endswith("support_bundle.py")
    assert "--worker" in popen_calls[0][0]
    assert responses == [
        "Support bundle collection started",
        f"Support bundle created: {tmp_path / 'bundle.tar.gz'}\n"
        "It may contain printer configuration, file names, and diagnostic data.",
    ]
