import io
import json
import tarfile
import zipfile

import pytest

import serval_bot.bundle_diagnostics as bundle_diagnostics
from serval_bot.bundle_diagnostics import render_bundle_report, support_bundle_urls


def test_finds_uploaded_support_bundle_links_only():
    bundle = "[serval-support-20260810-120000Z.tar.gz](https://github.com/user-attachments/files/123/bundle.tar.gz)"
    unrelated = "[klippy.log](https://github.com/user-attachments/files/124/klippy.log)"

    assert support_bundle_urls(bundle, unrelated) == ["https://github.com/user-attachments/files/123/bundle.tar.gz"]


def test_finds_zip_archive_with_non_bundle_name():
    archive = "[session1-bug-report.zip](https://github.com/user-attachments/files/396/report.zip)"

    assert support_bundle_urls(archive) == ["https://github.com/user-attachments/files/396/report.zip"]


def test_renders_info_level_multiline_shutdown_from_structured_events(tmp_path):
    bundle = tmp_path / "serval-support-test.tar.gz"
    manifest = {
        "created_utc": "2026-08-10T12:00:00+00:00",
        "cutoff_utc": "2026-08-10T11:30:00+00:00",
        "software_version": "test",
        "warnings": [],
        "event_files": {},
    }
    record = {
        "_time": "2026-08-10T11:59:00.000Z",
        "_msg": "MCU 'mcu' shutdown: Timer too close\nDumping config",
        "level": "info",
        "source": "host-py",
        "session_id": "k-1",
    }
    with tarfile.open(bundle, "w:gz") as archive:
        for name, payload in (
            ("manifest.json", json.dumps(manifest).encode()),
            ("events/host-py.jsonl", json.dumps(record).encode() + b"\n"),
        ):
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))

    report = render_bundle_report(bundle)

    assert "evidence: complete" in report
    assert "MCU 'mcu' shutdown: Timer too close" in report


def test_uploaded_bundle_is_inspected_without_user_commands(tmp_path, monkeypatch):
    source = tmp_path / "uploaded.tar.gz"
    manifest = {
        "created_utc": "2026-08-10T12:00:00+00:00",
        "cutoff_utc": "2026-08-10T11:30:00+00:00",
        "software_version": "test",
        "warnings": [],
        "event_files": {},
    }
    record = {
        "_time": "2026-08-10T11:59:00.000Z",
        "_msg": "MCU 'mcu' shutdown: Timer too close\nDumping config",
        "level": "info",
        "source": "host-py",
    }
    with tarfile.open(source, "w:gz") as archive:
        for name, payload in (
            ("manifest.json", json.dumps(manifest).encode()),
            ("events/host-py.jsonl", json.dumps(record).encode() + b"\n"),
        ):
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))

    def copy_uploaded_bundle(_url, destination):
        destination.write_bytes(source.read_bytes())

    monkeypatch.setattr(bundle_diagnostics, "download_support_bundle", copy_uploaded_bundle)
    text = "[serval-support-20260810-120000Z.tar.gz](https://github.com/user-attachments/files/123/bundle.tar.gz)"

    report = bundle_diagnostics.inspect_support_bundles((text,), tmp_path)

    assert "Attachment: https://github.com/user-attachments/" in report
    assert "MCU 'mcu' shutdown: Timer too close" in report


def test_reads_legacy_zip_events_and_imported_configs(tmp_path):
    bundle = tmp_path / "session1-bug-report.zip"
    record = {
        "_time": "2026-08-11T23:00:00.000Z",
        "_msg": "query precedes retained motion history",
        "level": "error",
        "source": "host-rust",
        "subsystem": "motion",
    }
    with zipfile.ZipFile(bundle, "w") as archive:
        archive.writestr("session1-host-rust.jsonl", json.dumps(record) + "\n")
        archive.writestr("printer.cfg", "[include steppers.cfg]\n")
        archive.writestr("steppers.cfg", "[stepper_x]\nstep_pin: PA1\n")

    report = render_bundle_report(bundle)

    assert "legacy archive has no manifest.json" in report
    assert "query precedes retained motion history" in report
    assert "configuration: printer.cfg" in report
    assert "[include steppers.cfg]" in report
    assert "configuration: steppers.cfg" in report


def test_rejects_cumulative_uncompressed_event_overflow(tmp_path, monkeypatch):
    bundle = tmp_path / "serval-support-bomb.tar.gz"
    manifest = json.dumps(
        {
            "created_utc": "2026-08-10T12:00:00+00:00",
            "cutoff_utc": "2026-08-10T11:30:00+00:00",
            "software_version": "test",
            "warnings": [],
            "event_files": {},
        }
    ).encode()
    event = json.dumps(
        {
            "_time": "2026-08-10T11:59:00.000Z",
            "_msg": "ordinary record",
            "level": "info",
        }
    ).encode()
    with tarfile.open(bundle, "w:gz") as archive:
        for name, payload in (
            ("manifest.json", manifest),
            ("events/host-py.jsonl", event),
            ("events/host-rust.jsonl", event),
        ):
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))
    monkeypatch.setattr(
        bundle_diagnostics,
        "MAX_TOTAL_EVENT_BYTES",
        len(manifest) + len(event),
    )

    with pytest.raises(
        bundle_diagnostics.BundleDiagnosticError,
        match="exceeds the uncompressed limit",
    ):
        bundle_diagnostics.load_bundle_records(bundle)


@pytest.mark.parametrize(
    "manifest,error",
    [
        ({"warnings": None, "event_files": {}}, "invalid warnings"),
        ({"warnings": [], "event_files": []}, "invalid event_files"),
        (
            {"warnings": [], "event_files": {"host-py.jsonl": []}},
            "invalid event_files",
        ),
    ],
)
def test_rejects_invalid_manifest_schema(manifest, error):
    with pytest.raises(bundle_diagnostics.BundleDiagnosticError, match=error):
        bundle_diagnostics.validate_manifest(manifest)


def test_invalid_utf8_manifest_is_a_controlled_error(tmp_path):
    bundle = tmp_path / "serval-support-invalid-utf8.tar.gz"
    payload = b'{"warnings":["\xff"],"event_files":{}}'
    with tarfile.open(bundle, "w:gz") as archive:
        info = tarfile.TarInfo("manifest.json")
        info.size = len(payload)
        archive.addfile(info, io.BytesIO(payload))

    with pytest.raises(
        bundle_diagnostics.BundleDiagnosticError,
        match="support bundle is unreadable",
    ):
        bundle_diagnostics.load_bundle_records(bundle)
