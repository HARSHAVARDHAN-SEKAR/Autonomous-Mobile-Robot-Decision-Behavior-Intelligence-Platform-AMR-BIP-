import math

from bip_core.grid import OccupancyGrid, grid_from_config
from bip_core import astar
from bip_core.bt import Status, Sequence, Fallback, Condition, Action
from bip_core.decision import StateMachine, UtilityArbiter, battery_urgency, schedule


# ---------------------------------------------------------------- grid + A*
def make_env():
    cfg = {
        "bounds": {"x_min": -6.0, "x_max": 6.0, "y_min": -4.0, "y_max": 4.0},
        "resolution": 0.1,
        "obstacles": [
            {"x": 0.0, "y": 4.0, "size_x": 12.4, "size_y": 0.2},   # top wall
            {"x": 0.0, "y": -4.0, "size_x": 12.4, "size_y": 0.2},  # bottom wall
            {"x": -6.0, "y": 0.0, "size_x": 0.2, "size_y": 8.4},   # left wall
            {"x": 6.0, "y": 0.0, "size_x": 0.2, "size_y": 8.4},    # right wall
            {"x": 0.0, "y": -2.5, "size_x": 0.2, "size_y": 3.0},   # inner wall A
            {"x": 2.5, "y": 2.0, "size_x": 0.2, "size_y": 4.0},    # inner wall B
        ],
    }
    return grid_from_config(cfg)


def test_grid_roundtrip():
    g = make_env()
    cx, cy = g.world_to_cell(1.23, -2.31)
    wx, wy = g.cell_to_world(cx, cy)
    assert abs(wx - 1.23) < g.res and abs(wy - (-2.31)) < g.res


def test_astar_around_wall():
    g = make_env()
    occ = g.inflated(0.3)
    start = g.world_to_cell(4.0, -3.0)
    goal = g.world_to_cell(-4.0, -3.0)
    path = astar.plan(occ, start, goal)
    assert path is not None
    # must detour above the inner wall A (which spans y in [-4, -1])
    ys = [g.cell_to_world(*c)[1] for c in path]
    assert max(ys) > -1.0
    sm = astar.smooth(occ, path)
    assert len(sm) <= len(path) and sm[0] == path[0] and sm[-1] == path[-1]


def test_astar_blocked():
    g = OccupancyGrid(0, 2, 0, 2, 0.1)
    g.add_rect(1.0, 1.0, 0.2, 2.4)  # full vertical wall
    occ = g.inflated(0.0)
    p = astar.plan(occ, g.world_to_cell(0.2, 1.0), g.world_to_cell(1.8, 1.0))
    assert p is None


def test_astar_goal_in_inflation_recovers():
    g = make_env()
    occ = g.inflated(0.3)
    # goal right next to a wall -> inside inflation zone -> nearest-free rescue
    path = astar.plan(occ, g.world_to_cell(0.0, 0.0), g.world_to_cell(5.85, 0.0))
    assert path is not None


# ---------------------------------------------------------------- behavior tree
def test_sequence_memory_and_failure():
    log = []
    calls = {"n": 0}

    def slow_action():
        calls["n"] += 1
        return Status.SUCCESS if calls["n"] >= 3 else Status.RUNNING

    seq = Sequence("s", [
        Action("first", lambda: (log.append("first"), Status.SUCCESS)[1]),
        Action("slow", slow_action),
        Action("last", lambda: (log.append("last"), Status.SUCCESS)[1]),
    ])
    assert seq.tick() == Status.RUNNING
    assert seq.tick() == Status.RUNNING
    assert seq.tick() == Status.SUCCESS
    assert log == ["first", "last"]  # memory: 'first' not re-ticked

    bad = Sequence("b", [Condition("no", lambda: False),
                         Action("never", lambda: Status.SUCCESS)])
    assert bad.tick() == Status.FAILURE


def test_fallback_preemption_resets_lower_branch():
    state = {"emergency": False, "reset_count": 0}

    def mission():
        return Status.RUNNING

    root = Fallback("root", [
        Sequence("em", [Condition("em?", lambda: state["emergency"]),
                        Action("stop", lambda: Status.RUNNING)]),
        Action("mission", mission,
               on_reset=lambda: state.__setitem__("reset_count",
                                                  state["reset_count"] + 1)),
    ])
    assert root.tick() == Status.RUNNING          # mission running
    state["emergency"] = True
    assert root.tick() == Status.RUNNING          # emergency preempts
    assert state["reset_count"] == 1              # mission branch was reset


# ---------------------------------------------------------------- FSM
def test_fsm_transitions_and_hooks():
    trace = []
    sm = StateMachine("IDLE")
    sm.add_state("IDLE", on_tick=lambda: "WORK", on_enter=lambda: trace.append("+I"),
                 on_exit=lambda: trace.append("-I"))
    sm.add_state("WORK", on_tick=lambda: None, on_enter=lambda: trace.append("+W"))
    assert sm.tick() == "WORK"
    assert sm.tick() == "WORK"
    assert trace == ["+I", "-I", "+W"]


# ---------------------------------------------------------------- utility
def test_utility_hysteresis():
    arb = UtilityArbiter(hysteresis=0.1)
    assert arb.choose({"mission": 0.5, "charge": 0.2}) == "mission"
    # charge is barely better -> hysteresis keeps mission
    assert arb.choose({"mission": 0.5, "charge": 0.55}) == "mission"
    # decisively better -> switch
    assert arb.choose({"mission": 0.5, "charge": 0.9}) == "charge"


def test_battery_urgency_monotonic():
    vals = [battery_urgency(p) for p in (100, 60, 30, 25, 20, 15, 10, 5)]
    assert all(b >= a - 1e-9 for a, b in zip(vals, vals[1:]))
    assert battery_urgency(5) == 1.0


# ---------------------------------------------------------------- scheduler
def test_schedule_priority_deadline():
    ms = [
        {"name": "low", "priority": 1},
        {"name": "urgent", "priority": 5, "deadline": 100},
        {"name": "urgent_late", "priority": 5, "deadline": 50},
        {"name": "expired", "priority": 9, "deadline": 5},
    ]
    ordered, expired = schedule(ms, now=10.0)
    assert [m["name"] for m in ordered] == ["urgent_late", "urgent", "low"]
    assert [m["name"] for m in expired] == ["expired"]
