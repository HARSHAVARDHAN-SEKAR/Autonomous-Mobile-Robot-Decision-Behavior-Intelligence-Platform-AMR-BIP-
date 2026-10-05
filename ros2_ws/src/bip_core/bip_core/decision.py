"""FSM engine, utility-based arbitration, and priority mission scheduler."""


# --------------------------------------------------------------------------
# Finite State Machine
# --------------------------------------------------------------------------
class StateMachine:
    """States are dicts of callables. on_tick returns the next state name
    (transition) or None (stay)."""

    def __init__(self, initial):
        self.states = {}
        self.current = initial
        self._entered = False

    def add_state(self, name, on_tick, on_enter=None, on_exit=None):
        self.states[name] = {"tick": on_tick, "enter": on_enter, "exit": on_exit}

    def force(self, name):
        self._transition(name)

    def _transition(self, name):
        if name not in self.states:
            raise KeyError(f"unknown state: {name}")
        cur = self.states[self.current]
        if cur["exit"]:
            cur["exit"]()
        self.current = name
        self._entered = False

    def tick(self):
        st = self.states[self.current]
        if not self._entered:
            if st["enter"]:
                st["enter"]()
            self._entered = True
        nxt = st["tick"]()
        if nxt is not None and nxt != self.current:
            self._transition(nxt)
        return self.current


# --------------------------------------------------------------------------
# Utility-based arbitration
# --------------------------------------------------------------------------
class UtilityArbiter:
    """Pick the highest-utility activity, with hysteresis to avoid chatter."""

    def __init__(self, hysteresis=0.08):
        self.hysteresis = hysteresis
        self.current = None

    def choose(self, scores):
        if not scores:
            return None
        best = max(scores, key=scores.get)
        if self.current is not None and self.current in scores:
            if scores[self.current] + self.hysteresis >= scores[best]:
                best = self.current
        self.current = best
        return best


def battery_urgency(pct, low=25.0, critical=10.0):
    """0 when full -> 1 near critical (piecewise linear)."""
    if pct >= low:
        return max(0.0, (100.0 - pct) / 100.0 * 0.2)
    if pct <= critical:
        return 1.0
    return 0.2 + 0.8 * (low - pct) / (low - critical)


# --------------------------------------------------------------------------
# Mission scheduler
# --------------------------------------------------------------------------
def schedule(missions, now=0.0):
    """Order missions by (priority desc, deadline asc, insertion order).
    Missions past their deadline are dropped (returned separately)."""
    valid, expired = [], []
    for i, m in enumerate(missions):
        dl = m.get("deadline")
        if dl is not None and dl < now:
            expired.append(m)
        else:
            valid.append((i, m))
    valid.sort(key=lambda im: (-im[1].get("priority", 0),
                               im[1].get("deadline", float("inf")),
                               im[0]))
    return [m for _, m in valid], expired
