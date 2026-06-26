#!/usr/bin/env python3
"""Local web review for motion-planner snapshots.

Serves a before/after comparison for every changed or pending case and an
accept action that rewrites baselines. It can also serve the committed
baselines as a read-only gallery. Stdlib only; run under an interpreter with
the built `_motion_engine` and matplotlib for review mode, or just matplotlib
for baseline mode:

    python snapshots/web/server.py            # then visit the printed URL

Normally you don't run this directly — snapshot-tests.sh starts it for you when
a case needs review. It never opens a browser; it prints a URL to visit.

The planner runs on the host; "after" is the current trajectory, "before" is
the committed baseline. Accept writes the current snapshot under baselines/.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SNAPSHOTS = _REPO_ROOT / "snapshots"
for _p in (str(_SNAPSHOTS), str(_REPO_ROOT / "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import harness  # noqa: E402
import viz_pipeline  # noqa: E402

_STATIC = Path(__file__).resolve().parent / "static"
_RENDER_LOCK = threading.Lock()


class ReviewState:
    """Holds the latest scan and cached rendered PNG bytes."""

    def __init__(self, mode: str):
        self._lock = threading.Lock()
        self.mode = mode
        self.cases: dict[str, dict] = {}
        self.error: str | None = None
        self.scan()

    def scan(self):
        with self._lock:
            self.cases = {}
            self.error = None
            try:
                discovered = harness.discover_cases()
            except Exception as exc:
                self.error = f"discover failed: {exc}"
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
                        "png": {},
                    }
                return
            for case in discovered:
                try:
                    snapshot = harness.run_case(case)
                except (ImportError, ValueError) as exc:
                    self.error = str(exc)
                    self.cases = {}
                    return
                status = harness.compare(case, snapshot)
                self.cases[case.name] = {
                    "case": case,
                    "snapshot": snapshot,
                    "status": status,
                    "png": {},
                }

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

    def png(self, name: str, which: str) -> bytes | None:
        with self._lock:
            entry = self.cases.get(name)
            if entry is None or which not in ("before", "after"):
                return None
            cached = entry["png"].get(which)
            if cached is not None:
                return cached
            if which == "before":
                snapshot = harness.baseline_snapshot(entry["case"])
                if snapshot is None:
                    return None
            else:
                snapshot = entry["snapshot"]
        data = _render_png(snapshot, name, which)
        with self._lock:
            if name in self.cases:
                self.cases[name]["png"][which] = data
        return data

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
        self.scan()
        return targets


def _render_png(snapshot: dict, name: str, which: str) -> bytes:
    # Flatten "group/stem" so it is a single filename, not a nested path.
    stem = name.replace("/", "__")
    with _RENDER_LOCK:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        out = _RENDER_DIR / f"{stem}_{which}.png"
        viz_pipeline.render(snapshot, _RENDER_DIR, stem, which)
        data = out.read_bytes()
        out.unlink(missing_ok=True)
        plt.close("all")
    return data


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
        elif path.startswith("/static/"):
            self._serve_static(path[len("/static/") :], None)
        elif path == "/api/cases":
            STATE.scan()
            self._json(STATE.summary())
        elif path.startswith("/img/"):
            self._serve_img(path)
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

    def _serve_img(self, path):
        parts = path.strip("/").split("/")
        if len(parts) != 3:
            self._json({"error": "bad image path"}, 404)
            return
        # The case name (parts[1]) is a single URL-encoded segment — a case
        # name like "default/clean_arc" arrives as "default%2Fclean_arc".
        _, name, leaf = parts
        name = unquote(name)
        which = leaf.removesuffix(".png")
        data = STATE.png(name, which)
        if data is None:
            self._json({"error": "no image"}, 404)
            return
        self._send(200, data, "image/png")

    def _serve_snapshot_data(self, path):
        parts = path.strip("/").split("/")
        if len(parts) != 2:
            self._json({"error": "bad path"}, 404)
            return
        _, name = parts
        name = unquote(name)
        with STATE._lock:
            entry = STATE.cases.get(name)
            if entry is None:
                self._json({"error": "not found"}, 404)
                return
            snapshot = entry["snapshot"]
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
    args = parser.parse_args()

    global STATE, _RENDER_DIR
    import tempfile

    with tempfile.TemporaryDirectory(prefix="snapshot-review-") as tmp:
        _RENDER_DIR = Path(tmp)
        STATE = ReviewState(args.mode)
        if STATE.error:
            print(f"warning: {STATE.error}", file=sys.stderr)
        server = ThreadingHTTPServer((args.host, args.port), Handler)
        url = f"http://{args.host}:{args.port}"
        label = (
            "snapshot baselines"
            if args.mode == "baselines"
            else "snapshot review"
        )
        print(f"{label} — visit {url}  (Ctrl-C to stop)")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nstopped")


if __name__ == "__main__":
    main()
