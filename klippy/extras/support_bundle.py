import datetime
import json
import os
import pathlib
import subprocess
import tarfile
import tempfile

DEFAULT_SINCE = "30m"
MAX_WINDOW_SECONDS = 24 * 60 * 60
SINCE_UNITS = {"m": 60, "h": 60 * 60}


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


def select_event_records(source, destination, cutoff, end):
    selected = 0
    malformed = 0
    with source.open("rb") as src, destination.open("wb") as dst:
        for line in src:
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
    return selected, malformed


def read_tail(path, max_bytes):
    size = path.stat().st_size
    with path.open("rb") as src:
        if size <= max_bytes:
            return src.read(), False
        src.seek(size - max_bytes)
        src.readline()
        return src.read(), True


def read_klipper_journal(cutoff):
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
    result = subprocess.run(command, capture_output=True, check=False)
    if result.returncode:
        detail = result.stderr.decode("utf-8", "replace").strip()
        raise SupportBundleError(f"journalctl failed: {detail}")
    return result.stdout


def event_log_paths(events_dir):
    if events_dir is None or not events_dir.is_dir():
        return []
    return sorted(
        path
        for path in events_dir.iterdir()
        if path.is_file() and ".jsonl" in path.name
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
    journal_reader=read_klipper_journal,
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

        for source in event_log_paths(events_dir):
            destination = events_output / source.name
            selected, malformed = select_event_records(
                source, destination, cutoff, now + 5.0
            )
            manifest["event_files"][source.name] = {
                "selected": selected,
                "malformed": malformed,
            }

        if log_file is not None and log_file.is_file():
            data, truncated = read_tail(log_file, 32 * 1024 * 1024)
            (staging / "klippy.log").write_bytes(data)
            manifest["klippy_log_truncated"] = truncated
        else:
            manifest["warnings"].append("klippy.log is unavailable")

        try:
            journal = journal_reader(cutoff)
        except (OSError, SupportBundleError) as exc:
            manifest["warnings"].append(str(exc))
        else:
            (staging / "klipper-journal.log").write_bytes(journal)

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


class SupportBundle:
    cmd_CREATE_SUPPORT_BUNDLE_help = "Create a downloadable recent-log bundle"

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
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
            log_file = pathlib.Path(log_name) if log_name else None
            events_name = start_args.get("log_events_dir")
            events_dir = pathlib.Path(events_name) if events_name else None
            core_dir = log_file.parent / "coredumps" if log_file else None
            if log_file is None:
                raise SupportBundleError(
                    "CREATE_SUPPORT_BUNDLE requires Klippy to run with --logfile"
                )
            bundle = create_support_bundle(
                log_file,
                events_dir,
                log_file.parent,
                since_seconds,
                software_version=start_args.get("software_version", "unknown"),
                core_dir=core_dir,
                include_core=bool(include_core),
            )
        except SupportBundleError as exc:
            raise gcmd.error(str(exc)) from exc
        gcmd.respond_info(
            f"Support bundle created: {bundle}\n"
            "It may contain printer configuration, file names, and diagnostic data."
        )


def load_config(config):
    return SupportBundle(config)
