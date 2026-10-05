"""ROS-free mission expansion, progress tracking, and step driving.

The `skills` object passed to drive_mission_step only needs:
  nav_state (str), nav_message (str), nav_busy(), navigate(x, y), reset_nav()
which makes this fully unit-testable with a fake.
"""
import math

import yaml

MAX_NAV_RETRIES = 2


# ---------------------------------------------------------------------------
def expand_mission(m):
    """Turn a mission dict into a flat list of steps.
    Step = ('goto', x, y) or ('wait', seconds)."""
    t = m['type']
    steps = []
    if t == 'patrol':
        for _ in range(int(m.get('loops', 1))):
            for (x, y) in m['waypoints']:
                steps.append(('goto', float(x), float(y)))
    elif t == 'inspect':
        x, y = m['target']
        steps.append(('goto', float(x), float(y)))
        steps.append(('wait', float(m.get('duration', 3.0))))
    elif t == 'deliver':
        px, py = m['pickup']
        dx, dy = m['dropoff']
        steps += [('goto', float(px), float(py)), ('wait', 1.5),
                  ('goto', float(dx), float(dy)), ('wait', 1.5)]
    else:
        raise ValueError(f'unknown mission type: {t}')
    return steps


def load_missions(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return data.get('missions', [])




# ---------------------------------------------------------------------------
class MissionProgress:
    """Tracks progress through the scheduled mission list (shared by all
    engines so recovery/resume logic is identical)."""

    def __init__(self, missions):
        self.missions = missions
        self.mission_idx = 0
        self.step_idx = 0
        self._steps = expand_mission(missions[0]) if missions else []
        self.wait_until = None
        self.retries = 0
        self.completed = []
        self.failed = []

    def done(self):
        return self.mission_idx >= len(self.missions)

    def current_mission(self):
        return None if self.done() else self.missions[self.mission_idx]

    def current_step(self):
        if self.done() or self.step_idx >= len(self._steps):
            return None
        return self._steps[self.step_idx]

    def advance_step(self):
        self.step_idx += 1
        self.retries = 0
        self.wait_until = None
        if self.step_idx >= len(self._steps):
            self.completed.append(self.current_mission()['name'])
            self._next_mission()

    def skip_mission(self, reason):
        m = self.current_mission()
        if m is not None:
            self.failed.append({'name': m['name'], 'reason': reason})
        self._next_mission()

    def _next_mission(self):
        self.mission_idx += 1
        self.step_idx = 0
        self.retries = 0
        self.wait_until = None
        if not self.done():
            self._steps = expand_mission(self.missions[self.mission_idx])

    def summary(self):
        return {'mission': None if self.done()
                else self.current_mission()['name'],
                'step': self.step_idx,
                'completed': self.completed,
                'failed': self.failed}


def drive_mission_step(skills, prog, now):
    """Advance the current mission by one poll. Returns:
    'running' | 'done' | 'failed_step'  (identical logic for every engine)."""
    if prog.done():
        return 'done'
    step = prog.current_step()
    if step is None:
        return 'done' if prog.done() else 'running'

    kind = step[0]
    if kind == 'wait':
        if prog.wait_until is None:
            prog.wait_until = now + step[1]
        if now >= prog.wait_until:
            prog.advance_step()
        return 'running'

    # goto
    if skills.nav_state in ('idle', 'canceled'):
        if not skills.navigate(step[1], step[2]):
            skills.nav_state = 'idle'   # server not ready yet -> retry next tick
        return 'running'
    if skills.nav_busy():
        return 'running'
    if skills.nav_state == 'succeeded':
        skills.reset_nav()
        skills.nav_state = 'idle'
        prog.advance_step()
        return 'running'
    if skills.nav_state == 'failed':
        skills.nav_state = 'idle'
        prog.retries += 1
        if prog.retries > MAX_NAV_RETRIES:
            prog.skip_mission(f'nav_failed:{skills.nav_message}')
            return 'failed_step'
        return 'running'
    return 'running'


def math_dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])
