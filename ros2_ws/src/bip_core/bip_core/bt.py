"""Minimal behavior-tree engine: SUCCESS / FAILURE / RUNNING semantics.

- Sequence: memory sequence (resumes at running child).
- Fallback: reactive selector (re-evaluates from the first child every tick,
  which is what makes battery/emergency preemption work).
- Condition: wraps a bool callable.
- Action: wraps a callable returning a Status each tick.
"""
from enum import Enum


class Status(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class Node:
    def __init__(self, name):
        self.name = name

    def tick(self):  # pragma: no cover - abstract
        raise NotImplementedError

    def reset(self):
        pass


class Condition(Node):
    def __init__(self, name, fn):
        super().__init__(name)
        self.fn = fn

    def tick(self):
        return Status.SUCCESS if self.fn() else Status.FAILURE


class Action(Node):
    def __init__(self, name, fn, on_reset=None):
        super().__init__(name)
        self.fn = fn
        self.on_reset = on_reset

    def tick(self):
        return self.fn()

    def reset(self):
        if self.on_reset:
            self.on_reset()


class Sequence(Node):
    """Memory sequence: remembers which child was RUNNING."""

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.idx = 0

    def tick(self):
        while self.idx < len(self.children):
            s = self.children[self.idx].tick()
            if s == Status.RUNNING:
                return Status.RUNNING
            if s == Status.FAILURE:
                self.reset()
                return Status.FAILURE
            self.idx += 1
        self.reset()
        return Status.SUCCESS

    def reset(self):
        self.idx = 0
        for c in self.children:
            c.reset()


class Fallback(Node):
    """Reactive fallback: always re-ticks from the first child."""

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.last_running = None

    def tick(self):
        for i, c in enumerate(self.children):
            s = c.tick()
            if s != Status.FAILURE:
                # a higher-priority branch took over -> reset preempted branch
                if s == Status.RUNNING and self.last_running is not None \
                        and self.last_running != i:
                    self.children[self.last_running].reset()
                self.last_running = i if s == Status.RUNNING else None
                return s
        self.last_running = None
        return Status.FAILURE

    def reset(self):
        self.last_running = None
        for c in self.children:
            c.reset()
