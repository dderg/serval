"""Per-task runtime isolation: deterministic slot UIDs, reaping, worker pool.

Modeled on RoboOMP: every event runs under a deterministic Unix slot UID and
the pool reaps every process still owned by that UID before the slot is
reused. On hosts without slot permissions (non-Linux or non-root) the slot
helpers become no-ops and the pool still bounds concurrency.
"""

from __future__ import annotations

import asyncio
import logging
import os
import platform
import shutil
import signal
import sqlite3
import stat
import time
from collections.abc import Callable
from contextlib import suppress
from pathlib import Path
from typing import Any, Protocol

from serval_bot.database import Database, Event

log = logging.getLogger(__name__)

SLOT_GID = 2000
FIRST_SLOT_UID = 2001
MAX_SLOTS = 8

REAP_DEADLINE_SECONDS = 5.0
_REAP_RESCAN_INTERVAL = 0.05


class PoolFatalError(RuntimeError):
    """The pool cannot continue safely; the affected slot must never be reused."""


class SlotReapError(PoolFatalError):
    """A slot UID could not be verified empty, so its slot must not be reused."""


class HardGraceExceeded(PoolFatalError):
    """The run thread outlived the hard grace, so its slot must not be reused."""


class AgentDirFailure(RuntimeError):
    """The shared agent dir seed source exists but is unsafe or malformed."""


_SCRUBBED_ENV_KEYS: tuple[str, ...] = (
    "GITHUB_TOKEN",
    "GH_TOKEN",
    "GITHUB_WEBHOOK_SECRET",
    "SERVAL_BOT_PROXY_HMAC_KEY",
    "SERVAL_BOT_GITHUB_TOKEN_PATH",
)


class Agent(Protocol):
    def run(self, event: Event, slot_uid: int | None) -> str: ...

    def stop(self, delivery_id: str) -> None: ...

    def merge_review(self, active: Event, duplicate: Event) -> bool: ...


def review_head(event: Event) -> str | None:
    pull_request = event.payload.get("pull_request")
    head = pull_request.get("head") if isinstance(pull_request, dict) else None
    sha = head.get("sha") if isinstance(head, dict) else None
    return sha if isinstance(sha, str) and len(sha) == 40 else None


def slot_uids(max_concurrency: int) -> tuple[int, ...]:
    return tuple(range(FIRST_SLOT_UID, FIRST_SLOT_UID + max_concurrency))


def slot_permissions_active(slot_uid: int | None) -> bool:
    return slot_uid is not None and platform.system() == "Linux" and os.geteuid() == 0


def slot_pids(slot_uid: int, proc_root: Path = Path("/proc")) -> tuple[int, ...]:
    """Return non-zombie process ids owned by the slot UID, read from /proc.

    The slim container image does not ship procps/pkill, so /proc is scanned
    directly; this keeps slot cleanup self-contained. A scan that cannot be
    completed or verified raises SlotReapError: slot emptiness must be proven,
    never assumed.
    """
    try:
        entries = tuple(proc_root.iterdir())
    except OSError as exc:
        raise SlotReapError(f"failed to scan {proc_root} for slot user {slot_uid}: {exc}") from exc
    pids: list[int] = []
    for entry in entries:
        if not entry.name.isdecimal():
            continue
        try:
            status = (entry / "status").read_text(encoding="utf-8")
        except FileNotFoundError:
            continue
        except OSError as exc:
            raise SlotReapError(f"failed to read {entry / 'status'} for slot user {slot_uid}: {exc}") from exc
        state = ""
        uids: tuple[int, ...] = ()
        for line in status.splitlines():
            if line.startswith("State:"):
                parts = line.split(maxsplit=1)
                state = parts[1] if len(parts) == 2 else ""
            elif line.startswith("Uid:"):
                try:
                    uids = tuple(int(part) for part in line.split()[1:5])
                except ValueError as exc:
                    raise SlotReapError(
                        f"unparseable ownership in {entry / 'status'} for slot user {slot_uid}"
                    ) from exc
        if state.startswith("Z") or slot_uid not in uids:
            continue
        pids.append(int(entry.name))
    return tuple(pids)


def reap_slot(
    slot_uid: int | None,
    *,
    deadline_seconds: float = REAP_DEADLINE_SECONDS,
    proc_root: Path = Path("/proc"),
) -> int:
    """SIGKILL every non-zombie process owned by the slot UID; verify emptiness.

    Slot UIDs are reused: a straggler from one event must not observe or
    interfere with the next event on the same slot. The slot is only safe when
    a scan returns no live slot-owned process, so reap scans, kills, waits,
    and rescans to that fixed point: descendants forked during the kill window
    or moved into a new session still share the slot UID and are caught by the
    next scan. Any scan/kill error or a residual pid at the deadline raises
    SlotReapError instead of releasing the slot into the pool.
    """
    if not slot_permissions_active(slot_uid):
        return 0
    assert slot_uid is not None
    deadline = time.monotonic() + deadline_seconds
    reaped = 0
    while True:
        pids = slot_pids(slot_uid, proc_root)
        if not pids:
            return reaped
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                continue
            except OSError as exc:
                raise SlotReapError(f"failed to kill slot user {slot_uid} process {pid}: {exc}") from exc
            reaped += 1
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            residual = slot_pids(slot_uid, proc_root)
            if not residual:
                return reaped
            raise SlotReapError(f"slot user {slot_uid} still owns processes at the reap deadline: {residual}")
        time.sleep(min(_REAP_RESCAN_INTERVAL, remaining))


def slot_subprocess_kwargs(slot_uid: int | None) -> dict[str, Any]:
    """Return subprocess identity kwargs that run a child as the slot.

    `preexec_fn` is avoided: tasks run in threads, and subprocess warns it is
    unsafe there. Python's native user/group/extra_groups parameters perform
    the setuid/setgid switch in the child safely. umask is deliberately not
    set: the entrypoint's 0027 propagates through fork, and the pinned omp-rpc
    RpcClient accepts no umask parameter.
    """
    if not slot_permissions_active(slot_uid):
        return {}
    assert slot_uid is not None
    return {"user": slot_uid, "group": slot_uid, "extra_groups": [SLOT_GID]}


def _provision_private_dirs(session_dir: Path) -> tuple[Path, Path]:
    """Create the per-event HOME/TMP/XDG tree under the session dir.

    Session dirs are not git-cleaned, so the tree survives worktree syncs.
    Returns (home, tmp).
    """
    home = session_dir / ".home"
    tmpdir = session_dir / ".tmp"
    home.mkdir(parents=True, exist_ok=True)
    try:
        st = tmpdir.lstat()
    except FileNotFoundError:
        pass
    else:
        if not stat.S_ISDIR(st.st_mode):
            tmpdir.unlink()
    tmpdir.mkdir(mode=0o700, parents=True, exist_ok=True)

    xdg_root = session_dir / ".xdg"
    for sub in ("data", "state", "cache"):
        (xdg_root / sub).mkdir(parents=True, exist_ok=True)
    (xdg_root / "cache" / "bun-install").mkdir(parents=True, exist_ok=True)
    return home, tmpdir


_AGENT_DIR_SEED_NAMES = ("config.yml", "config.yaml", "config", "auth", "agent.db")


def _provision_agent_dir(session_dir: Path) -> Path:
    """Per-event private omp agent dir, seeded from the shared source.

    The private dir lives under the slot-owned session tree and is wiped and
    re-seeded on every event so no state outlives its slot. Required
    persistent config/auth inputs are copied, with agent.db snapshotted as a
    transactionally consistent SQLite copy; history.db, models, sessions,
    memories, and blobs are never shared. A missing or empty shared source
    seeds nothing; a present but unsafe source fails loudly.
    """
    private = session_dir / ".omp-agent"
    if private.exists():
        shutil.rmtree(private)
    private.mkdir()
    shared = os.environ.get("PI_CODING_AGENT_DIR")
    if shared:
        _seed_agent_dir(private, Path(shared))
    return private


def _seed_agent_dir(private: Path, shared: Path) -> None:
    if not shared.is_dir():
        return
    for name in _AGENT_DIR_SEED_NAMES:
        source = shared / name
        if not source.exists() and not source.is_symlink():
            continue
        if source.is_symlink():
            raise AgentDirFailure(f"unsafe agent dir seed source (symlink): {source}")
        if name == "agent.db":
            if not source.is_file():
                raise AgentDirFailure(f"unsafe agent dir seed source (unexpected type): {source}")
            _snapshot_sqlite(source, private / "agent.db")
        elif source.is_dir():
            if any(source.iterdir()):
                _copy_seed_tree(source, private / name)
        elif source.is_file():
            shutil.copy2(source, private / name)
        else:
            raise AgentDirFailure(f"unsafe agent dir seed source (unexpected type): {source}")


def _snapshot_sqlite(source: Path, destination: Path) -> None:
    """Copy a SQLite database consistently via the backup API.

    The source is opened read-only so the shared auth store can never be
    mutated by seeding, and the backup captures committed WAL content without
    ever copying -wal/-shm sidecars into the private dir. The copy is switched
    back to rollback-journal mode so the private agent dir stays a
    single-file database.
    """
    with (
        sqlite3.connect(f"{source.as_uri()}?mode=ro", uri=True) as src,
        sqlite3.connect(destination) as dst,
    ):
        src.backup(dst)
        dst.execute("PRAGMA journal_mode=DELETE")


def _copy_seed_tree(source: Path, destination: Path) -> None:
    destination.mkdir(parents=True)
    for dirpath, dirnames, filenames in os.walk(source, followlinks=False):
        relative = Path(dirpath).relative_to(source)
        for name in dirnames:
            path = Path(dirpath) / name
            if path.is_symlink():
                raise AgentDirFailure(f"unsafe agent dir seed source (symlink): {path}")
            (destination / relative / name).mkdir(parents=True, exist_ok=True)
        for name in filenames:
            path = Path(dirpath) / name
            if path.is_symlink():
                raise AgentDirFailure(f"unsafe agent dir seed source (symlink): {path}")
            if not path.is_file():
                raise AgentDirFailure(f"unsafe agent dir seed source (unexpected type): {path}")
            shutil.copy2(path, destination / relative / name)


def slot_env(slot_uid: int | None, workspace: Path, session_dir: Path) -> dict[str, str]:
    """Prepare the event's private runtime tree and return the OMP env overlay.

    Provisions per-event HOME/TMP/XDG and a private omp agent dir under the
    session dir, hands the event-owned trees to the slot identity, and returns
    an overlay with workspace-private paths plus credential keys blanked so
    the agent subprocess cannot printenv them; RpcClient merges it over
    os.environ. The agent dir is seeded with required config/auth inputs from
    the shared PI_CODING_AGENT_DIR source before the chown, and the overlay
    redirects PI_CODING_AGENT_DIR to the private copy.
    """
    home, tmpdir = _provision_private_dirs(session_dir)
    agent_dir = _provision_agent_dir(session_dir)
    chown_event_paths(workspace, session_dir, slot_uid)
    xdg_root = session_dir / ".xdg"
    env = dict.fromkeys(_SCRUBBED_ENV_KEYS, "")
    env.update(
        {
            "HOME": str(home),
            "TMPDIR": str(tmpdir),
            "TMP": str(tmpdir),
            "TEMP": str(tmpdir),
            "XDG_DATA_HOME": str(xdg_root / "data"),
            "XDG_STATE_HOME": str(xdg_root / "state"),
            "XDG_CACHE_HOME": str(xdg_root / "cache"),
            "BUN_INSTALL_CACHE_DIR": str(xdg_root / "cache" / "bun-install"),
            "PI_CODING_AGENT_DIR": str(agent_dir),
        }
    )
    return env


def chown_event_paths(workspace: Path, session_dir: Path, slot_uid: int | None) -> None:
    """Recursively hand both event-owned trees to the slot identity.

    Owner and group become the slot's own UID/GID with owner-only modes
    (directories 0700, files keep their owner bits), so the slot can write its
    workspace and session while sibling workspaces/sessions and the shared Git
    pool stay write-inaccessible to it. Root re-syncs by recreating repository
    metadata from the trusted pool, so no group sharing is required. The
    session parent chain is exposed (0755) so the slot can traverse to its
    private tree.
    """
    if not slot_permissions_active(slot_uid):
        return
    _provision_private_dirs(session_dir)
    for ancestor in (session_dir.parent, session_dir.parent.parent):
        if ancestor.name:
            ancestor.chmod(0o755)
    for root in (workspace, session_dir):
        if not root.exists():
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            for name in dirnames + filenames:
                path = Path(dirpath) / name
                try:
                    st = path.lstat()
                except FileNotFoundError:
                    continue
                if stat.S_ISLNK(st.st_mode):
                    continue
                os.chown(path, slot_uid, slot_uid)
                if stat.S_ISDIR(st.st_mode):
                    path.chmod(0o700)
                else:
                    path.chmod(st.st_mode & 0o700)
        os.chown(root, slot_uid, slot_uid)
        root.chmod(0o700)


class SlotPool:
    def __init__(self, max_concurrency: int) -> None:
        self._slot_uids = slot_uids(max_concurrency)
        self._available: asyncio.Queue[int] = asyncio.Queue()
        for slot_uid in self._slot_uids:
            self._available.put_nowait(slot_uid)
        self._checked_out: set[int] = set()

    @property
    def slot_uids(self) -> tuple[int, ...]:
        return self._slot_uids

    async def acquire(self) -> int:
        slot_uid = await self._available.get()
        self._checked_out.add(slot_uid)
        return slot_uid

    def release(self, slot_uid: int) -> None:
        if slot_uid not in self._checked_out:
            raise ValueError(f"slot UID was not acquired: {slot_uid}")
        self._checked_out.remove(slot_uid)
        self._available.put_nowait(slot_uid)


class WorkerPool:
    """One worker loop per slot UID, draining the durable event queue.

    Each loop claims only after the previous event's thread has drained and
    its slot has been reaped, so no thread or process can overlap the next
    claimed event or a reused slot.
    """

    def __init__(
        self,
        database: Database,
        agent: Agent,
        *,
        timeout_seconds: int,
        hard_grace_seconds: int,
        max_concurrency: int,
        max_retries: int = 0,
        retry_delay_seconds: Callable[[int], float] | None = None,
    ) -> None:
        if not 1 <= max_concurrency <= MAX_SLOTS:
            raise ValueError(f"max_concurrency must be between 1 and {MAX_SLOTS}")
        self._database = database
        self._agent = agent
        self._timeout_seconds = timeout_seconds
        self._hard_grace_seconds = hard_grace_seconds
        self._pool = SlotPool(max_concurrency)
        self._max_retries = max_retries
        self._retry_delay_seconds = retry_delay_seconds or (lambda _: 0.0)
        self._stop = asyncio.Event()
        self._wake = [asyncio.Event() for _ in self._pool.slot_uids]
        self._active_reviews: dict[tuple[str, int], tuple[Event, asyncio.Task[None]]] = {}
        self._superseded: dict[str, str] = {}

    def wake(self) -> None:
        for event in self._wake:
            event.set()

    def stop(self) -> None:
        self._stop.set()
        self.wake()

    def reconcile_review_heads(self, repo: str, review_heads: dict[int, str]) -> None:
        skipped = self._database.skip_stale_reviews(repo, review_heads)
        for key, (event, task) in tuple(self._active_reviews.items()):
            if key[0] != repo:
                continue
            current_head = review_heads.get(key[1])
            if "review_request" not in event.payload:
                continue
            event_head = review_head(event)
            if current_head is not None and current_head == event_head:
                continue
            self._superseded[event.delivery_id] = (
                "review assignment removed" if current_head is None else "superseded by newer pull request head"
            )
            task.cancel()
        if skipped:
            self.wake()

    def _remove_active_review(self, review_key: tuple[str, int] | None, event: Event) -> None:
        if review_key is None:
            return
        active = self._active_reviews.get(review_key)
        if active is not None and active[0].delivery_id == event.delivery_id:
            self._active_reviews.pop(review_key)

    async def run(self) -> None:
        recovered = self._database.reset_running()
        if recovered:
            log.info("recovered running events", extra={"count": recovered})
        await asyncio.gather(*(asyncio.to_thread(reap_slot, uid) for uid in self._pool.slot_uids))
        log.info(
            "worker pool online",
            extra={"max_concurrency": len(self._pool.slot_uids), "slot_uids": list(self._pool.slot_uids)},
        )
        loops = [
            asyncio.create_task(self._worker_loop(slot_uid, wake), name=f"serval-worker-{slot_uid}")
            for slot_uid, wake in zip(self._pool.slot_uids, self._wake, strict=True)
        ]
        try:
            await asyncio.gather(*loops)
        except PoolFatalError as exc:
            log.error("worker pool disabled", extra={"error": f"{type(exc).__name__}: {exc}"})
            for loop in loops:
                loop.cancel()
            with suppress(BaseException):
                await asyncio.gather(*loops)
            raise
        except BaseException:
            for loop in loops:
                loop.cancel()
            with suppress(BaseException):
                await asyncio.gather(*loops)
            raise

    async def _worker_loop(self, slot_uid: int, wake: asyncio.Event) -> None:
        while not self._stop.is_set():
            wake.clear()
            await self.merge_queued_duplicates()
            event = self._database.claim()
            if event is None:
                retry_delay = self._database.next_retry_delay_seconds()
                idle_timeout = 1.0 if retry_delay is None else min(1.0, retry_delay)
                with suppress(TimeoutError):
                    await asyncio.wait_for(wake.wait(), timeout=idle_timeout)
                continue
            slot_uid = await self._pool.acquire()
            try:
                await self._run_event(event, slot_uid)
            except PoolFatalError:
                raise
            else:
                self._pool.release(slot_uid)

    async def merge_queued_duplicates(self) -> None:
        for (repo, issue_number), (active, _task) in tuple(self._active_reviews.items()):
            active_head = review_head(active)
            if active_head is None:
                continue
            for duplicate in self._database.queued_reviews(repo, issue_number):
                if review_head(duplicate) != active_head:
                    continue
                if not self._database.reserve_queued(duplicate.delivery_id):
                    continue
                merged = False
                try:
                    merged = await asyncio.to_thread(self._agent.merge_review, active, duplicate)
                finally:
                    if merged:
                        self._database.finish(
                            duplicate.delivery_id, "done", f"merged into running review {active.delivery_id}"
                        )
                        log.info(
                            "duplicate review merged into running session",
                            extra={
                                "delivery_id": duplicate.delivery_id,
                                "merged_into": active.delivery_id,
                                "repo": repo,
                                "issue": issue_number,
                            },
                        )
                    else:
                        self._database.requeue(duplicate.delivery_id)

    async def _run_event(self, event: Event, slot_uid: int) -> None:
        started = time.monotonic()
        task = asyncio.current_task()
        if task is None:
            raise RuntimeError("review worker has no asyncio task")
        review_key: tuple[str, int] | None = None
        if event.event_type == "pull_request_review.requested":
            review_key = (event.repo, event.issue_number)
            self._active_reviews[review_key] = (event, task)
        self._log_event("event claimed", event, slot_uid, started)
        future = asyncio.ensure_future(asyncio.to_thread(self._agent.run, event, slot_uid))
        self._log_event("event started", event, slot_uid, started)
        outcome: str | None = None
        error: str | None = None
        cancelled = False
        try:
            try:
                await asyncio.wait_for(asyncio.shield(future), self._timeout_seconds)
                outcome = "success"
            except TimeoutError:
                outcome = "timeout"
                error = f"task exceeded {self._timeout_seconds}s deadline"
            except PoolFatalError:
                raise
            except Exception as exc:
                outcome = "failure"
                error = f"{type(exc).__name__}: {exc}"[:8000]
        except asyncio.CancelledError:
            cancelled = True

        if cancelled and outcome is None:
            await self._stop_agent(event)
            exceeded = await self._drain(event, slot_uid, future)
            fatal = self._fatal_failure(event, future, exceeded)
            if fatal is not None:
                raise fatal
            reaped = reap_slot(slot_uid)
            if event.delivery_id in self._superseded:
                reason = self._superseded.pop(event.delivery_id)
                self._database.finish(event.delivery_id, "skipped", reason)
                self._remove_active_review(review_key, event)
                self._log_event("event superseded", event, slot_uid, started, extra={"reaped": reaped})
                return
            self._remove_active_review(review_key, event)
            self._log_event("event stopped by shutdown", event, slot_uid, started, extra={"reaped": reaped})
            raise asyncio.CancelledError

        if outcome == "timeout":
            await self._stop_agent(event)
            exceeded = await self._drain(event, slot_uid, future)
            fatal = self._fatal_failure(event, future, exceeded)
            if fatal is not None:
                raise fatal
            reaped = reap_slot(slot_uid)
            if self._schedule_retry(event, error, slot_uid, started, reaped):
                if cancelled:
                    raise asyncio.CancelledError
                return
            self._database.finish(event.delivery_id, "failed", error)
            self._log_event("event timed out", event, slot_uid, started, extra={"error": error, "reaped": reaped})
            self._remove_active_review(review_key, event)
            if cancelled:
                raise asyncio.CancelledError
            return

        if outcome == "failure":
            await self._drain(event, slot_uid, future)
            reaped = reap_slot(slot_uid)
            if self._schedule_retry(event, error, slot_uid, started, reaped):
                if cancelled:
                    raise asyncio.CancelledError
                return
            self._database.finish(event.delivery_id, "failed", error)
            self._log_event("event failed", event, slot_uid, started, extra={"error": error, "reaped": reaped})
            self._remove_active_review(review_key, event)
            if cancelled:
                raise asyncio.CancelledError
            return

        if outcome == "success":
            self._database.finish(event.delivery_id, "done")
            reaped = reap_slot(slot_uid)
            self._log_event("event succeeded", event, slot_uid, started, extra={"reaped": reaped})
            self._remove_active_review(review_key, event)
            if cancelled:
                raise asyncio.CancelledError
            return
        raise RuntimeError(f"unreachable outcome for {event.delivery_id}")

    def _schedule_retry(
        self,
        event: Event,
        error: str | None,
        slot_uid: int,
        started: float,
        reaped: int,
    ) -> bool:
        if error is None or not 0 < event.attempts <= self._max_retries:
            return False
        delay = self._retry_delay_seconds(event.attempts)
        if not self._database.schedule_retry(event.delivery_id, delay, error):
            return False
        self._log_event(
            "event retry scheduled",
            event,
            slot_uid,
            started,
            extra={
                "error": error,
                "reaped": reaped,
                "attempt": event.attempts,
                "max_retries": self._max_retries,
                "retry_in_seconds": round(delay, 1),
            },
        )
        return True

    async def _stop_agent(self, event: Event) -> None:
        """Call agent.stop off-loop; wait for it even under pool cancellation.

        A stop that fails may leave the RPC process group alive, so the
        failure propagates instead of being logged away: the slot stays
        withheld and the pool dies loudly.
        """
        stop_future = asyncio.ensure_future(asyncio.to_thread(self._agent.stop, event.delivery_id))
        while True:
            try:
                await asyncio.shield(stop_future)
                return
            except asyncio.CancelledError:
                continue
            except Exception:
                log.exception("agent stop raised", extra={"delivery_id": event.delivery_id})
                raise

    def _fatal_failure(self, event: Event, future: asyncio.Future[Any], exceeded: bool) -> PoolFatalError | None:
        """Return the fatal failure to raise, or None when the slot is safe.

        Either the run thread outlived the hard grace (an in-flight host-tool
        execution or run body never drained) or the run thread itself died
        with a fatal error. Both permanently withhold the slot.
        """
        if exceeded:
            return HardGraceExceeded(
                f"worker thread did not exit within {self._hard_grace_seconds}s hard grace: {event.delivery_id}"
            )
        exception = future.exception()
        if isinstance(exception, PoolFatalError):
            return exception
        return None

    async def _drain(self, event: Event, slot_uid: int, future: asyncio.Future[Any]) -> bool:
        """Wait for the run thread to exit, bounded by the hard grace.

        The run thread lives in the executor and outlives pool cancellation,
        so drain keeps waiting even while the pool task is being torn down:
        the slot must not be reused while the thread still runs. Returns True
        when the thread outlived the hard grace.
        """
        exceeded = False
        while not future.done():
            try:
                await asyncio.wait_for(asyncio.shield(future), self._hard_grace_seconds)
            except TimeoutError:
                exceeded = True
                break
            except asyncio.CancelledError:
                continue
            except Exception:
                break
        return exceeded

    def _log_event(self, message: str, event: Event, slot_uid: int, started: float, **extra: Any) -> None:
        log.info(
            message,
            extra={
                "delivery_id": event.delivery_id,
                "repo": event.repo,
                "issue": event.issue_number,
                "slot": slot_uid,
                "elapsed_ms": int((time.monotonic() - started) * 1000),
                **extra,
            },
        )
