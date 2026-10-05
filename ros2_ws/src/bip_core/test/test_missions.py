from bip_core.missions import (MissionProgress, drive_mission_step,
                               expand_mission)


class FakeSkills:
    """Duck-typed stand-in for the ROS skills layer.
    nav succeeds after `ticks_to_succeed` polls; can be forced to fail."""

    def __init__(self, ticks_to_succeed=3, fail_targets=None):
        self.nav_state = 'idle'
        self.nav_message = ''
        self.ticks_to_succeed = ticks_to_succeed
        self.fail_targets = fail_targets or set()
        self._countdown = 0
        self._target = None
        self.goals_sent = []

    def navigate(self, x, y, yaw=0.0):
        self._target = (x, y)
        self.goals_sent.append((x, y))
        self.nav_state = 'running'
        self._countdown = self.ticks_to_succeed
        return True

    def nav_busy(self):
        if self.nav_state == 'running':
            self._countdown -= 1
            if self._countdown <= 0:
                if self._target in self.fail_targets:
                    self.nav_state = 'failed'
                    self.nav_message = 'no_path'
                else:
                    self.nav_state = 'succeeded'
        return self.nav_state in ('pending', 'running')

    def reset_nav(self):
        self.nav_state = 'idle'


MISSIONS = [
    {'name': 'p1', 'type': 'patrol', 'waypoints': [[1, 1], [2, 2]], 'loops': 1},
    {'name': 'i1', 'type': 'inspect', 'target': [3, 3], 'duration': 0.05},
    {'name': 'd1', 'type': 'deliver', 'pickup': [4, 4], 'dropoff': [5, 5]},
]


def run_to_completion(skills, prog, max_ticks=5000):
    t = 0.0
    for _ in range(max_ticks):
        r = drive_mission_step(skills, prog, t)
        t += 0.1
        if r == 'done':
            return True
    return False


def test_expand_mission_shapes():
    assert expand_mission(MISSIONS[0]) == [('goto', 1.0, 1.0), ('goto', 2.0, 2.0)]
    assert expand_mission(MISSIONS[1])[0] == ('goto', 3.0, 3.0)
    assert expand_mission(MISSIONS[1])[1][0] == 'wait'
    assert len(expand_mission(MISSIONS[2])) == 4


def test_all_missions_complete():
    skills = FakeSkills(ticks_to_succeed=3)
    prog = MissionProgress(list(MISSIONS))
    assert run_to_completion(skills, prog)
    assert prog.completed == ['p1', 'i1', 'd1']
    assert prog.failed == []
    # every goto target was actually commanded
    assert (1.0, 1.0) in skills.goals_sent and (5.0, 5.0) in skills.goals_sent


def test_nav_failure_retries_then_skips_mission():
    # waypoint (2,2) always fails -> p1 skipped after retries, others complete
    skills = FakeSkills(ticks_to_succeed=2, fail_targets={(2.0, 2.0)})
    prog = MissionProgress(list(MISSIONS))
    assert run_to_completion(skills, prog)
    assert prog.completed == ['i1', 'd1']
    assert prog.failed[0]['name'] == 'p1'
    assert 'nav_failed' in prog.failed[0]['reason']
    # exactly 1 initial try + MAX_NAV_RETRIES retries for the bad target
    assert skills.goals_sent.count((2.0, 2.0)) == 3


def test_progress_survives_interruption():
    """Simulates a battery preemption: nav canceled mid-step, then resumed —
    progress must resume at the same step, not restart the mission."""
    skills = FakeSkills(ticks_to_succeed=3)
    prog = MissionProgress(list(MISSIONS))
    # complete first waypoint
    t = 0.0
    while prog.step_idx == 0:
        drive_mission_step(skills, prog, t)
        t += 0.1
    # preemption: executive cancels nav (canceled state), does other things
    skills.nav_state = 'canceled'
    step_before = (prog.mission_idx, prog.step_idx)
    # resume
    assert run_to_completion(skills, prog)
    assert prog.completed == ['p1', 'i1', 'd1']
    assert step_before == (0, 1)  # it had NOT lost step-0 progress
