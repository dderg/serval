from klippy.extras import stepper_enable


class FakeEnableLine:
    def __init__(self):
        self.enabled_at = []
        self.disabled_at = []

    def motor_enable(self, print_time):
        self.enabled_at.append(print_time)

    def energize(self, print_time):
        self.enabled_at.append(print_time)
        return None

    def motor_disable(self, print_time):
        self.disabled_at.append(print_time)


class FakeToolhead:
    def __init__(self):
        self.events = []
        self._t = 100.0

    def dwell(self, delay):
        self.events.append(("dwell", delay))
        self._t += delay

    def get_last_move_time(self):
        return self._t

    def resync_parked_servos(self):
        self.events.append(("resync", self._t))
        self._t += 1.0


class FakePrinter:
    def __init__(self, toolhead):
        self._toolhead = toolhead

    def lookup_object(self, name):
        assert name == "toolhead"
        return self._toolhead


class FakeStepperEnable:
    motor_debug_enable = stepper_enable.PrinterStepperEnable.motor_debug_enable
    motor_enable_group = stepper_enable.PrinterStepperEnable.motor_enable_group

    def __init__(self, toolhead, names):
        self.printer = FakePrinter(toolhead)
        self.enable_lines = {n: FakeEnableLine() for n in names}


def test_debug_enable_resyncs_before_energize():
    th = FakeToolhead()
    se = FakeStepperEnable(th, ["servo_z"])
    se.motor_debug_enable("servo_z", True)
    kinds = [e[0] for e in th.events]
    assert "resync" in kinds
    enabled_at = se.enable_lines["servo_z"].enabled_at
    assert enabled_at, "motor was energized"
    resync_t = next(e[1] for e in th.events if e[0] == "resync")
    assert enabled_at[0] > resync_t


def test_debug_disable_does_not_resync():
    th = FakeToolhead()
    se = FakeStepperEnable(th, ["servo_z"])
    se.motor_debug_enable("servo_z", False)
    assert all(e[0] != "resync" for e in th.events)
    assert se.enable_lines["servo_z"].disabled_at


def test_group_enable_resyncs_before_energize():
    th = FakeToolhead()
    se = FakeStepperEnable(th, ["motor_a", "servo_z"])
    se.motor_enable_group(["motor_a", "servo_z"])
    assert any(e[0] == "resync" for e in th.events)
    resync_t = next(e[1] for e in th.events if e[0] == "resync")
    for el in se.enable_lines.values():
        assert el.enabled_at[0] > resync_t
