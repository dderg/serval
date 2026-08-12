import contextlib
import json
import pathlib
import re
import tarfile
import urllib.parse
import urllib.request
import zipfile

MAX_BUNDLE_BYTES = 256 * 1024 * 1024
MAX_REPORT_BYTES = 200_000
MAX_MEMBER_BYTES = 128 * 1024 * 1024
MAX_TOTAL_EVENT_BYTES = 128 * 1024 * 1024
MAX_EVENT_LINE_BYTES = 1024 * 1024
MAX_DIAGNOSTIC_MEMBERS = 64
MAX_ARCHIVE_MEMBERS = 1024
MAX_EVENT_RECORDS = 1_000_000
MAX_MANIFEST_BYTES = 1024 * 1024
MAX_DIAGNOSTIC_RECORDS = 20_000
MAX_LIFECYCLE_RECORDS = 500
DOWNLOAD_TIMEOUT_SECONDS = 30
_ALLOWED_DOWNLOAD_HOSTS = frozenset(("github.com", "objects.githubusercontent.com"))
_BUNDLE_LINK_RE = re.compile(
    r"\[[^\]]+\.(?:tar\.gz|zip)\]\((https://github\.com/user-attachments/[^)]+)\)",
    re.IGNORECASE,
)


class BundleDiagnosticError(Exception):
    pass


def support_bundle_urls(*texts):
    urls = []
    for text in texts:
        if not isinstance(text, str):
            continue
        for match in _BUNDLE_LINK_RE.finditer(text):
            url = match.group(1)
            if url not in urls:
                urls.append(url)
    return urls


def download_support_bundle(url, destination):
    parsed = urllib.parse.urlparse(url)
    if parsed.scheme != "https" or parsed.hostname != "github.com" or not parsed.path.startswith("/user-attachments/"):
        raise BundleDiagnosticError("unsupported support bundle URL")
    request = urllib.request.Request(url, headers={"User-Agent": "RoboServal/1"})
    try:
        with urllib.request.urlopen(request, timeout=DOWNLOAD_TIMEOUT_SECONDS) as response:
            final = urllib.parse.urlparse(response.geturl())
            if final.scheme != "https" or final.hostname not in _ALLOWED_DOWNLOAD_HOSTS:
                raise BundleDiagnosticError("support bundle redirected to an unsupported host")
            length = response.headers.get("Content-Length")
            if length is not None and int(length) > MAX_BUNDLE_BYTES:
                raise BundleDiagnosticError("support bundle exceeds the download limit")
            written = 0
            with destination.open("wb") as output:
                while True:
                    chunk = response.read(64 * 1024)
                    if not chunk:
                        break
                    written += len(chunk)
                    if written > MAX_BUNDLE_BYTES:
                        raise BundleDiagnosticError("support bundle exceeds the download limit")
                    output.write(chunk)
    except (OSError, ValueError) as exc:
        destination.unlink(missing_ok=True)
        raise BundleDiagnosticError(f"support bundle download failed: {exc}") from exc


@contextlib.contextmanager
def _archive_members(bundle):
    if zipfile.is_zipfile(bundle):
        with zipfile.ZipFile(bundle) as archive:
            yield (
                (
                    member.filename,
                    member.file_size,
                    member.is_dir(),
                    lambda member=member: archive.open(member),
                )
                for member in archive.infolist()
            )
        return
    with tarfile.open(bundle, "r:gz") as archive:
        yield (
            (
                member.name,
                member.size,
                not member.isfile(),
                lambda member=member: archive.extractfile(member),
            )
            for member in archive
        )


def load_bundle_records(bundle):
    manifest = None
    records = []
    configs = []
    malformed = 0
    selected_members = 0
    selected_bytes = 0
    archive_members = 0
    total_records = 0
    lifecycle_records = 0
    context_truncated = False
    lifecycle_events = frozenset(("print.start", "print.pause", "print.resume", "print.end"))
    try:
        with _archive_members(bundle) as members:
            for name, size, is_directory, open_member in members:
                archive_members += 1
                if archive_members > MAX_ARCHIVE_MEMBERS:
                    raise BundleDiagnosticError("support bundle has too many archive members")
                member_path = pathlib.PurePosixPath(name)
                if member_path.is_absolute() or ".." in member_path.parts or is_directory:
                    continue
                is_manifest = member_path.name == "manifest.json"
                is_events = member_path.suffix == ".jsonl"
                is_config = member_path.suffix == ".cfg"
                if not is_manifest and not is_events and not is_config:
                    continue
                selected_members += 1
                selected_bytes += size
                if selected_members > MAX_DIAGNOSTIC_MEMBERS:
                    raise BundleDiagnosticError("support bundle has too many diagnostic members")
                if selected_bytes > MAX_TOTAL_EVENT_BYTES:
                    raise BundleDiagnosticError("support bundle diagnostic data exceeds the uncompressed limit")
                if size > MAX_MEMBER_BYTES:
                    raise BundleDiagnosticError(f"support bundle member is too large: {name}")
                if is_manifest and size > MAX_MANIFEST_BYTES:
                    raise BundleDiagnosticError("support bundle manifest exceeds the size limit")
                source = open_member()
                if source is None:
                    raise BundleDiagnosticError(f"support bundle member is unreadable: {name}")
                with source:
                    if is_manifest:
                        manifest = json.load(source)
                        continue
                    if is_config:
                        configs.append((str(member_path), source.read().decode("utf-8")))
                        continue
                    while True:
                        line = source.readline(MAX_EVENT_LINE_BYTES + 1)
                        if not line:
                            break
                        if len(line) > MAX_EVENT_LINE_BYTES:
                            raise BundleDiagnosticError("support bundle event record exceeds the line limit")
                        total_records += 1
                        if total_records > MAX_EVENT_RECORDS:
                            raise BundleDiagnosticError("support bundle exceeds the event record limit")
                        try:
                            record = json.loads(line)
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
                        lifecycle = record.get("event") in lifecycle_events
                        if lifecycle:
                            lifecycle_records += 1
                            if lifecycle_records > MAX_LIFECYCLE_RECORDS:
                                context_truncated = True
                                lifecycle = False
                        if primary or lifecycle:
                            if len(records) >= MAX_DIAGNOSTIC_RECORDS:
                                raise BundleDiagnosticError("support bundle exceeds the diagnostic record limit")
                            records.append(record)
    except (
        OSError,
        UnicodeDecodeError,
        tarfile.TarError,
        zipfile.BadZipFile,
        json.JSONDecodeError,
    ) as exc:
        raise BundleDiagnosticError(f"support bundle is unreadable: {exc}") from exc
    if manifest is None:
        manifest = {
            "created_utc": "?",
            "cutoff_utc": "?",
            "software_version": "?",
            "event_files": {},
            "warnings": ["legacy archive has no manifest.json"],
        }
    if not isinstance(manifest, dict):
        raise BundleDiagnosticError("support bundle has no valid manifest.json")
    return manifest, records, configs, malformed, total_records, context_truncated


def validate_manifest(manifest):
    if not isinstance(manifest, dict):
        raise BundleDiagnosticError("support bundle has no valid manifest.json")
    warnings = manifest.get("warnings", [])
    if not isinstance(warnings, list) or not all(isinstance(warning, str) for warning in warnings):
        raise BundleDiagnosticError("support bundle manifest has invalid warnings")
    event_files = manifest.get("event_files", {})
    if not isinstance(event_files, dict) or not all(
        isinstance(name, str) and isinstance(details, dict) for name, details in event_files.items()
    ):
        raise BundleDiagnosticError("support bundle manifest has invalid event_files")


def render_bundle_report(bundle):
    manifest, selected, configs, malformed, total_records, context_truncated = load_bundle_records(bundle)
    validate_manifest(manifest)
    lines = [
        f"created: {manifest.get('created_utc', '?')}",
        f"cutoff: {manifest.get('cutoff_utc', '?')}",
        f"software: {manifest.get('software_version', '?')}",
    ]
    warnings = list(manifest.get("warnings", []))
    if manifest.get("klippy_log_truncated"):
        warnings.append("klippy.log is truncated")
    for name, details in manifest.get("event_files", {}).items():
        if details.get("scan_truncated"):
            warnings.append(f"{name} scan is truncated")
        if details.get("malformed"):
            warnings.append(f"{name} contains malformed records")
    if malformed:
        warnings.append(f"{malformed} bundled event records could not be decoded")
    if context_truncated:
        warnings.append(f"lifecycle context limited to {MAX_LIFECYCLE_RECORDS} records")
    lines.append("evidence: INCOMPLETE" if warnings else "evidence: complete")
    lines.extend(f"warning: {warning}" for warning in warnings)
    selected.sort(key=lambda record: str(record.get("_time", "")))
    lines.append(f"diagnostic records: {len(selected)} selected from {total_records}")
    for record in selected:
        lines.append(
            "{time} {level} {source}/{subsystem} {event} | {message}".format(
                time=record.get("_time", "?"),
                level=record.get("level", "info"),
                source=record.get("source", "?"),
                subsystem=record.get("subsystem", "?"),
                event=record.get("event", ""),
                message=record.get("_msg", ""),
            )
        )
        if record.get("exception"):
            lines.append(str(record["exception"]))
    for name, contents in configs:
        lines.append(f"configuration: {name}")
        lines.append(contents)
    report = "\n".join(lines)
    encoded = report.encode("utf-8")
    if len(encoded) <= MAX_REPORT_BYTES:
        return report
    visible = encoded[:MAX_REPORT_BYTES].decode("utf-8", errors="ignore")
    return visible + "\nwarning: diagnostic report truncated at output limit"


def inspect_support_bundles(texts, download_dir):
    reports = []
    for index, url in enumerate(support_bundle_urls(*texts)):
        suffix = ".zip" if url.lower().split("?", 1)[0].endswith(".zip") else ".archive"
        bundle = pathlib.Path(download_dir) / f"support-bundle-{index}{suffix}"
        try:
            download_support_bundle(url, bundle)
            report = render_bundle_report(bundle)
            reports.append(f"Attachment: {url}\n{report}")
        except BundleDiagnosticError as exc:
            reports.append(f"Attachment: {url}\nInspection failed: {exc}")
        finally:
            bundle.unlink(missing_ok=True)
    return "\n\n".join(reports)
