import contextlib

_SENTINEL = object()


class FakeError(Exception):
    pass


class FakeCommandError(Exception):
    pass


class FakeConfigError(Exception):
    pass


class FakeReactor:
    NOW = 0.0
    NEVER = 9999999999999999.0

    def __init__(self, now=0.0, tick=0.0):
        self.now = now
        self.tick = tick
        self.pauses = []
        self.timers = []
        self.callbacks = []

    def monotonic(self):
        self.now += self.tick
        return self.now

    def pause(self, waketime):
        self.pauses.append(waketime)
        self.now = max(self.now, waketime)
        return self.now

    def register_timer(self, callback, waketime=NEVER):
        timer_handler = [callback, waketime]
        self.timers.append(timer_handler)
        return timer_handler

    def unregister_timer(self, timer_handler):
        self.timers.remove(timer_handler)

    def update_timer(self, timer_handler, waketime):
        timer_handler[1] = waketime

    def register_callback(self, callback, waketime=NOW):
        self.callbacks.append(callback)
        return None

    def register_async_callback(self, callback, waketime=NOW):
        return None

    def register_fd(self, fd, read_callback, write_callback=None):
        return object()

    def unregister_fd(self, fd_handler):
        return None

    def mutex(self, is_locked=False):
        return contextlib.nullcontext()

    def run(self):
        return None

    def get_gc_stats(self):
        return (0, 0, 0)


class FakePrinter:
    command_error = FakeCommandError
    config_error = FakeConfigError

    def __init__(self, objects=None, reactor=None, shutdown=False):
        self.objects = dict(objects) if objects else {}
        self.reactor = reactor
        self.event_handlers = {}
        self.events = []
        self.shutdown_reasons = []
        self._shutdown = shutdown

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is _SENTINEL:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default

    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        prefix = module + " "
        objs = [
            (n, self.objects[n]) for n in self.objects if n.startswith(prefix)
        ]
        if module in self.objects:
            return [(module, self.objects[module])] + objs
        return objs

    def load_object(self, config, name, default=_SENTINEL):
        return self.lookup_object(name, default)

    def get_reactor(self):
        if self.reactor is None:
            self.reactor = FakeReactor()
        return self.reactor

    def register_event_handler(self, event, callback):
        self.event_handlers[event] = callback

    def send_event(self, event, *params):
        self.events.append(event)
        callback = self.event_handlers.get(event)
        if callback is None:
            return []
        return [callback(*params)]

    def is_shutdown(self):
        return self._shutdown

    def invoke_shutdown(self, msg):
        self._shutdown = True
        self.shutdown_reasons.append(msg)

    def get_start_args(self):
        return {}
