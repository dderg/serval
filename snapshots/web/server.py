#!/usr/bin/env python3
"""Local web review for motion-planner snapshots.

Serves a before/after comparison for every changed or pending case and an
accept action that rewrites baselines. It can also serve the committed
baselines as a read-only gallery. Stdlib only; run under an interpreter with
the built `_motion_engine` for review mode:

    python snapshots/web/server.py            # then visit the printed URL

Normally you don't run this directly — snapshot-tests.sh starts it for you when
a case needs review. It never opens a browser; it prints a URL to visit.

The planner runs on the host; "after" is the current trajectory, "before" is
the committed baseline. Accept writes the current snapshot under baselines/.
"""

from __future__ import annotations

import argparse
import gzip
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SNAPSHOTS = _REPO_ROOT / "snapshots"
for _p in (str(_SNAPSHOTS), str(_REPO_ROOT / "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import harness  # noqa: E402

_STATIC = Path(__file__).resolve().parent / "static"


class ReviewState:
    """Holds the latest scan of cases."""

    def __init__(self, mode: str, results_dir: Path | None = None):
        self._lock = threading.Lock()
        self.mode = mode
        self.results_dir = results_dir
        self.cases: dict[str, dict] = {}
        self.error: str | None = None
        self.scan()

    def _load_results(self, discovered):
        statuses = json.loads((self.results_dir / "status.json").read_text())
        by_name = {case.name: case for case in discovered}
        for name, status_value in statuses.items():
            case = by_name.get(name)
            if case is None:
                continue
            status = harness.Status(status_value)
            snapshot = None
            if status is not harness.Status.EXACT:
                blob = (
                    self.results_dir / f"{name.replace('/', '__')}.json.gz"
                ).read_bytes()
                snapshot = json.loads(gzip.decompress(blob))
            self.cases[name] = {
                "case": case,
                "snapshot": snapshot,
                "status": status,
            }

    def scan(self):
        with self._lock:
            self.cases = {}
            self.error = None
            try:
                discovered = harness.discover_cases()
            except Exception as exc:
                self.error = f"discover failed: {exc}"
                return
            if self.results_dir is not None:
                try:
                    self._load_results(discovered)
                except (OSError, ValueError, KeyError) as exc:
                    self.error = f"loading run results failed: {exc}"
                    self.cases = {}
                return
            if self.mode == "baselines":
                expected = {case.baseline_path.resolve() for case in discovered}
                found = {
                    path.resolve()
                    for path in harness.BASELINES_DIR.glob(
                        f"**/*{harness.BASELINE_SUFFIX}"
                    )
                }
                orphaned = sorted(found - expected)
                if orphaned:
                    details = "\n".join(
                        str(path.relative_to(harness.BASELINES_DIR))
                        for path in orphaned
                    )
                    self.error = (
                        f"orphaned baselines without matching cases:\n{details}"
                    )
                    return
                for case in discovered:
                    snapshot = harness.baseline_snapshot(case)
                    if snapshot is None:
                        continue
                    self.cases[case.name] = {
                        "case": case,
                        "snapshot": snapshot,
                        "status": "baseline",
                    }
                return
            try:
                for case, snapshot in harness.run_cases_parallel(discovered):
                    status = harness.compare(case, snapshot)
                    self.cases[case.name] = {
                        "case": case,
                        "snapshot": snapshot,
                        "status": status,
                    }
            except (ImportError, ValueError) as exc:
                self.error = str(exc)
                self.cases = {}

    def summary(self) -> dict:
        with self._lock:
            if self.mode == "baselines":
                baselines = [
                    {
                        "name": name,
                        "status": entry["status"],
                        "has_before": False,
                    }
                    for name, entry in self.cases.items()
                ]
                return {
                    "mode": self.mode,
                    "title": "Motion snapshot baselines",
                    "review": baselines,
                    "exact": 0,
                    "baseline_count": len(baselines),
                    "read_only": True,
                    "error": self.error,
                }
            review = [
                {
                    "name": name,
                    "status": entry["status"].value,
                    "has_before": entry["status"] is harness.Status.CHANGED,
                }
                for name, entry in self.cases.items()
                if entry["status"] is not harness.Status.EXACT
            ]
            exact = sum(
                1
                for e in self.cases.values()
                if e["status"] is harness.Status.EXACT
            )
            return {
                "mode": self.mode,
                "title": "Motion snapshot review",
                "review": review,
                "exact": exact,
                "read_only": False,
                "error": self.error,
            }

    def accept(self, names: list[str]) -> list[str]:
        if self.mode == "baselines":
            raise RuntimeError("baseline viewer is read-only")
        with self._lock:
            targets = [
                n
                for n in names
                if n in self.cases
                and self.cases[n]["status"] is not harness.Status.EXACT
            ]
            for n in targets:
                entry = self.cases[n]
                harness.write_baseline(entry["case"], entry["snapshot"])
                entry["status"] = harness.Status.EXACT
        return targets


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, code, body, content_type):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._serve_static("index.html", "text/html; charset=utf-8")
        elif path == "/viewer.html":
            self._serve_static("viewer.html", "text/html; charset=utf-8")
        elif path in ("/playground", "/playground.html"):
            # The playground uses page-relative asset URLs so the static/
            # directory is hostable as-is; here it must live under /static/.
            self.send_response(302)
            self.send_header("Location", "/static/playground.html")
            self.end_headers()
        elif path.startswith("/static/"):
            self._serve_static(path[len("/static/") :], None)
        elif path == "/api/cases":
            self._json(STATE.summary())
        elif path.startswith("/snapshot-data/"):
            self._serve_snapshot_data(path)
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        if self.path != "/api/accept":
            self._json({"error": "not found"}, 404)
            return
        if STATE.mode == "baselines":
            self._json({"error": "baseline viewer is read-only"}, 405)
            return
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length) or b"{}")
        names = body.get("names", [])
        if body.get("all"):
            names = [c["name"] for c in STATE.summary()["review"]]
        accepted = STATE.accept(names)
        summary = STATE.summary()
        self._json({"accepted": accepted, **summary})
        # Nothing left to review -> the session is done; stop serving so the
        # script can re-check and exit.
        if not summary["review"]:
            threading.Thread(target=self.server.shutdown, daemon=True).start()

    def _serve_snapshot_data(self, path):
        parts = path.strip("/").split("/")
        if len(parts) != 2:
            self._json({"error": "bad path"}, 404)
            return
        _, name = parts
        name = unquote(name)
        which = parse_qs(urlsplit(self.path).query).get("which", ["after"])[0]
        if which not in ("before", "after"):
            self._json({"error": "bad which"}, 404)
            return
        with STATE._lock:
            entry = STATE.cases.get(name)
            if entry is None:
                self._json({"error": "not found"}, 404)
                return
            if which == "after":
                snapshot = entry["snapshot"]
            elif STATE.mode == "baselines":
                snapshot = None  # gallery shows committed baselines; no prior
            else:
                snapshot = harness.baseline_snapshot(entry["case"])
        # A missing "before" is an expected state (baselines gallery, NEW
        # case), not an error — 200 null keeps the browser console clean.
        if snapshot is None and which == "before":
            self._json(None)
            return
        if snapshot is None:
            self._json({"error": "no baseline"}, 404)
            return
        self._json(snapshot)

    def _serve_static(self, rel, content_type):
        target = (_STATIC / rel).resolve()
        if _STATIC not in target.parents or not target.is_file():
            self._json({"error": "not found"}, 404)
            return
        if content_type is None:
            content_type = {
                ".js": "text/javascript",
                ".css": "text/css",
                ".html": "text/html; charset=utf-8",
                ".wasm": "application/wasm",
                ".d.ts": "text/plain",
                ".json": "application/json",
            }.get(target.suffix, "application/octet-stream")
        self._send(200, target.read_bytes(), content_type)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument(
        "--mode", choices=("review", "baselines"), default="review"
    )
    parser.add_argument(
        "--results",
        type=Path,
        help="directory of statuses/snapshots written by run.py "
        "--results-dir; review serves these instead of re-running the cases",
    )
    args = parser.parse_args()
    if args.results and args.mode == "baselines":
        parser.error("--results only applies to review mode")

    global STATE
    STATE = ReviewState(args.mode, results_dir=args.results)
    if STATE.error:
        print(f"warning: {STATE.error}", file=sys.stderr)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://{args.host}:{args.port}"
    label = (
        "snapshot baselines" if args.mode == "baselines" else "snapshot review"
    )
    print(f"{label} — visit {url}  (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
