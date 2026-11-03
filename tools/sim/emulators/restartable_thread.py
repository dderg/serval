import threading


class RestartableThread:
    """One worker thread that can be asked to run again while it is still
    finishing a stop.

    "Is it still running?" and "it has decided to stop" must be one atomic
    fact. Asking `Thread.is_alive()` is not: a thread that has already left
    its loop is alive until its cleanup ends, so a restart in that window saw
    a live thread, started none, and the old one then exited, leaving nothing
    running. Here the worker's loop asks `keep_running`, which under the lock
    either confirms it or clears the slot, and `ensure_running` starts a new
    thread exactly when the slot is clear.
    """

    def __init__(self, target, name: str):
        self._target = target
        self._name = name
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None

    def ensure_running(self) -> None:
        with self._lock:
            if self._thread is not None:
                return
            self._thread = threading.Thread(
                target=self._target, name=self._name, daemon=True
            )
            self._thread.start()

    def keep_running(self, wanted: bool) -> bool:
        with self._lock:
            if wanted:
                return True
            self._thread = None
            return False

    def join(self, timeout: float) -> None:
        with self._lock:
            thread = self._thread
        if thread is not None:
            thread.join(timeout=timeout)
