from klippy import motion

EST_NOW = 22.0
MOTION_LEAD = 0.25


class RecordingMcu:
    non_critical_disconnected = False

    def __init__(self):
        self.calibrated_through = []

    def get_engine_handle(self):
        return 0

    def estimated_print_time(self, eventtime):
        return EST_NOW

    def check_active(self, print_time, eventtime):
        self.calibrated_through.append(print_time)


class PlannerEngine:
    def __init__(self, frontier):
        self.frontier = frontier

    def frontier_print_time(self, mcu_handle):
        return self.frontier

    def pump_backlog(self):
        return 0


class Reactor:
    def monotonic(self):
        return EST_NOW


class StatsMotion:
    stats = motion.Motion.stats
    _schedule_floor = motion.Motion._schedule_floor
    _schedulable_mcus = motion.Motion._schedulable_mcus

    def __init__(self, frontier):
        self.mcu = RecordingMcu()
        self.scanner = RecordingMcu()
        self.all_mcus = [self.mcu, self.scanner]
        self.engine = PlannerEngine(frontier)
        self.reactor = Reactor()
        self.motion_lead = MOTION_LEAD
        self._planner_ready = True
        self.print_stall = 0


def test_a_planned_homing_move_does_not_anchor_secondary_clocks_minutes_ahead():
    homing_move_end = EST_NOW + 76.0
    m = StatsMotion(frontier=homing_move_end)
    m.stats(EST_NOW)
    assert m.scanner.calibrated_through == [EST_NOW + MOTION_LEAD]


def test_an_idle_planner_calibrates_at_its_past_frontier():
    m = StatsMotion(frontier=EST_NOW - 5.0)
    m.stats(EST_NOW)
    assert m.scanner.calibrated_through == [EST_NOW - 5.0]
