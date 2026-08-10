import argparse
import datetime
import json
import os
import pathlib
import subprocess
import sys
import tarfile
import tempfile
import threading
from selectors import EVENT_READ, DefaultSelector
from time import monotonic

DEFAULT_SINCE = "30m"
MAX_WINDOW_SECONDS = 24 * 60 * 60
SINCE_UNITS = {"m": 60, "h": 60 * 60}
EVENT_SCAN_BYTES = 32 * 1024 * 1024
TOTAL_EVENT_SCAN_BYTES = 128 * 1024 * 1024
KLIPPY_LOG_BYTES = 32 * 1024 * 1024
JOURNAL_BYTES = 16 * 1024 * 1024
JOURNAL_TIMEOUT_SECONDS = 30
WORKER_TIMEOUT_SECONDS = 30 * 60
READ_CHUNK_BYTES = 64 * 1024


class SupportBundleError(Exception):
    pass


def parse_since(value):
    if len(value) < 2 or value[-1] not in SINCE_UNITS:
        raise SupportBundleError(
            "SINCE must use minutes or hours, for example 30m or 2h"
        )
    try:
        amount = int(value[:-1])
    except ValueError as exc:
        raise SupportBundleError(
            "SINCE must use minutes or hours, for example 30m or 2h"
        ) from exc
    seconds = amount * SINCE_UNITS[value[-1]]
    if amount <= 0 or seconds > MAX_WINDOW_SECONDS:
        raise SupportBundleError("SINCE must be between 1m and 24h")
    return seconds


def parse_record_time(value):
    if value.endswith("Z"):
        value = value[:-1] + "+00:00"
    return datetime.datetime.fromisoformat(value).timestamp()


def read_tail(path, max_bytes):
    size = path.stat().st_size
    with path.open("rb") as src:
        if size <= max_bytes:
            return src.read(), False
        src.seek(size - max_bytes)
        src.readline()
        return src.read(), True


def select_event_records(source, destination, cutoff, end, max_bytes):
    data, truncated = read_tail(source, max_bytes)
    selected = 0
    malformed = 0
    with destination.open("wb") as dst:
        for line in data.splitlines(keepends=True):
            try:
                record = json.loads(line)
                timestamp = parse_record_time(record["_time"])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                malformed += 1
                continue
            if cutoff <= timestamp <= end:
                dst.write(line)
                selected += 1
    if not selected:
        destination.unlink()
    return selected, malformed, truncated, len(data)


def write_klipper_journal(cutoff, destination):
    command = [
        "journalctl",
        "-u",
        "klipper",
        "--since",
        f"@{int(cutoff):d}",
        "--no-pager",
        "--output",
        "short-iso-precise",
    ]
    with tempfile.TemporaryFile() as stderr:
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=stderr
        )
        captured = 0
        truncated = False
        deadline = monotonic() + JOURNAL_TIMEOUT_SECONDS
        selector = DefaultSelector()
        selector.register(process.stdout, EVENT_READ)
        try:
            with destination.open("wb") as output:
                while True:
                    remaining_time = deadline - monotonic()
                    if remaining_time <= 0:
                        process.kill()
                        process.wait()
                        raise SupportBundleError(
                            "journalctl timed out after 30 seconds"
                        )
                    if not selector.select(remaining_time):
                        process.kill()
                        process.wait()
                        raise SupportBundleError(
                            "journalctl timed out after 30 seconds"
                        )
                    chunk = os.read(process.stdout.fileno(), READ_CHUNK_BYTES)
                    if not chunk:
                        break
                    remaining_bytes = JOURNAL_BYTES - captured
                    output.write(chunk[:remaining_bytes])
                    captured += min(len(chunk), remaining_bytes)
                    if len(chunk) >= remaining_bytes:
                        truncated = True
                        process.kill()
                        break
            process.wait(timeout=max(1.0, deadline - monotonic()))
        except subprocess.TimeoutExpired as exc:
            process.kill()
            process.wait()
            raise SupportBundleError(
                "journalctl timed out after 30 seconds"
            ) from exc
        finally:
            selector.close()
            process.stdout.close()
        if process.returncode and not truncated:
            stderr.seek(0)
            detail = stderr.read(4096).decode("utf-8", "replace").strip()
            raise SupportBundleError(f"journalctl failed: {detail}")
    return truncated


def event_log_paths(events_dir, cutoff):
    if events_dir is None or not events_dir.is_dir():
        return []
    return sorted(
        (
            path
            for path in events_dir.iterdir()
            if path.is_file()
            and ".jsonl" in path.name
            and path.stat().st_mtime >= cutoff
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )


def latest_core(core_dir, cutoff, end):
    if core_dir is None or not core_dir.is_dir():
        return None
    candidates = [
        path
        for path in core_dir.iterdir()
        if path.is_file() and cutoff <= path.stat().st_mtime <= end
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime, default=None)


def create_support_bundle(
    log_file,
    events_dir,
    output_dir,
    since_seconds,
    now=None,
    journal_writer=write_klipper_journal,
    software_version="unknown",
    core_dir=None,
    include_core=False,
):
    now = (
        datetime.datetime.now(datetime.timezone.utc).timestamp()
        if now is None
        else now
    )
    cutoff = now - since_seconds
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.fromtimestamp(
        now, datetime.timezone.utc
    ).strftime("%Y%m%d-%H%M%SZ")
    final_path = output_dir / f"serval-support-{stamp}.tar.gz"
    if final_path.exists():
        raise SupportBundleError(f"support bundle already exists: {final_path}")

    with tempfile.TemporaryDirectory(
        prefix=".serval-support-", dir=output_dir
    ) as tmp:
        staging = pathlib.Path(tmp)
        events_output = staging / "events"
        events_output.mkdir()
        manifest = {
            "created_utc": datetime.datetime.fromtimestamp(
                now, datetime.timezone.utc
            ).isoformat(),
            "cutoff_utc": datetime.datetime.fromtimestamp(
                cutoff, datetime.timezone.utc
            ).isoformat(),
            "software_version": software_version,
            "event_files": {},
            "event_scan_limit_bytes": TOTAL_EVENT_SCAN_BYTES,
            "warnings": [],
        }
        core = latest_core(core_dir, cutoff, now + 5.0)
        manifest["latest_core"] = (
            None
            if core is None
            else {
                "name": core.name,
                "size": core.stat().st_size,
                "included": include_core,
            }
        )

        remaining_scan_bytes = TOTAL_EVENT_SCAN_BYTES
        for source in event_log_paths(events_dir, cutoff):
            if remaining_scan_bytes <= 0:
                manifest["warnings"].append(
                    "event scan limit reached; use a shorter SINCE window"
                )
                break
            scan_bytes = min(EVENT_SCAN_BYTES, remaining_scan_bytes)
            destination = events_output / source.name
            selected, malformed, truncated, scanned = select_event_records(
                source, destination, cutoff, now + 5.0, scan_bytes
            )
            remaining_scan_bytes -= scanned
            manifest["event_files"][source.name] = {
                "selected": selected,
                "malformed": malformed,
                "scan_truncated": truncated,
                "scanned_bytes": scanned,
            }

        if log_file is not None and log_file.is_file():
            data, truncated = read_tail(log_file, KLIPPY_LOG_BYTES)
            (staging / "klippy.log").write_bytes(data)
            manifest["klippy_log_truncated"] = truncated
        else:
            manifest["warnings"].append("klippy.log is unavailable")

        try:
            journal_truncated = journal_writer(
                cutoff, staging / "klipper-journal.log"
            )
        except (OSError, SupportBundleError) as exc:
            manifest["warnings"].append(str(exc))
        else:
            manifest["journal_truncated"] = journal_truncated

        (staging / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary_archive = staging / "bundle.tar.gz"
        with tarfile.open(temporary_archive, "w:gz") as archive:
            for path in sorted(staging.iterdir()):
                if path != temporary_archive:
                    archive.add(path, arcname=path.name, recursive=True)
            if core is not None and include_core:
                archive.add(core, arcname=f"coredumps/{core.name}")
        os.replace(temporary_archive, final_path)
    return final_path


def worker_command(
    log_file,
    events_dir,
    since_seconds,
    software_version,
    include_core,
):
    command = [
        sys.executable,
        str(pathlib.Path(__file__).resolve()),
        "--worker",
        "--log-file",
        str(log_file),
        "--since-seconds",
        str(since_seconds),
        "--software-version",
        software_version,
    ]
    if events_dir is not None:
        command.extend(("--events-dir", str(events_dir)))
    if include_core:
        command.append("--include-core")
    return command


class SupportBundle:
    cmd_CREATE_SUPPORT_BUNDLE_help = "Create a downloadable recent-log bundle"

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.reactor = self.printer.get_reactor()
        self.worker = None
        self.worker_lock = threading.Lock()
        self.gcode.register_command(
            "CREATE_SUPPORT_BUNDLE",
            self.cmd_CREATE_SUPPORT_BUNDLE,
            desc=self.cmd_CREATE_SUPPORT_BUNDLE_help,
        )

    def cmd_CREATE_SUPPORT_BUNDLE(self, gcmd):
        since_value = gcmd.get("SINCE", DEFAULT_SINCE)
        include_core = gcmd.get_int("INCLUDE_CORE", 0, minval=0, maxval=1)
        try:
            since_seconds = parse_since(since_value)
            start_args = self.printer.get_start_args()
            log_name = start_args.get("log_file")
            if log_name is None:
                raise SupportBundleError(
                    "CREATE_SUPPORT_BUNDLE requires Klippy to run with --logfile"
                )
            log_file = pathlib.Path(log_name)
            events_name = start_args.get("log_events_dir")
            events_dir = pathlib.Path(events_name) if events_name else None
            command = worker_command(
                log_file,
                events_dir,
                since_seconds,
                start_args.get("software_version", "unknown"),
                bool(include_core),
            )
            with self.worker_lock:
                if self.worker is not None:
                    raise SupportBundleError(
                        "support bundle collection is already running"
                    )
                self.worker = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                worker = self.worker
        except (OSError, SupportBundleError) as exc:
            raise gcmd.error(str(exc)) from exc
        gcmd.respond_info("Support bundle collection started")
        threading.Thread(
            target=self._wait_for_worker,
            args=(worker,),
            daemon=True,
        ).start()

    def _wait_for_worker(self, worker):
        try:
            stdout, stderr = worker.communicate(timeout=WORKER_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            worker.kill()
            worker.communicate()
            result = "Support bundle collection timed out after 30 minutes"
        else:
            if worker.returncode:
                detail = (
                    stderr.strip()
                    or f"worker exited with status {worker.returncode}"
                )
                result = f"Support bundle collection failed: {detail}"
            else:
                result = (
                    f"Support bundle created: {stdout.strip()}\n"
                    "It may contain printer configuration, file names, and diagnostic data."
                )
        finally:
            with self.worker_lock:
                if self.worker is worker:
                    self.worker = None
        self.reactor.register_async_callback(
            lambda eventtime, message=result: self.gcode.respond_info(message)
        )


def build_worker_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", action="store_true", required=True)
    parser.add_argument("--log-file", type=pathlib.Path, required=True)
    parser.add_argument("--events-dir", type=pathlib.Path)
    parser.add_argument("--since-seconds", type=int, required=True)
    parser.add_argument("--software-version", required=True)
    parser.add_argument("--include-core", action="store_true")
    return parser


def run_worker(argv):
    args = build_worker_parser().parse_args(argv)
    if not 0 < args.since_seconds <= MAX_WINDOW_SECONDS:
        raise SupportBundleError("worker received an invalid time window")
    bundle = create_support_bundle(
        args.log_file,
        args.events_dir,
        args.log_file.parent,
        args.since_seconds,
        software_version=args.software_version,
        core_dir=args.log_file.parent / "coredumps",
        include_core=args.include_core,
    )
    print(bundle, flush=True)
    return 0


def load_config(config):
    return SupportBundle(config)


if __name__ == "__main__":
    try:
        sys.exit(run_worker(sys.argv[1:]))
    except SupportBundleError as exc:
        print(exc, file=sys.stderr, flush=True)
        sys.exit(1)
