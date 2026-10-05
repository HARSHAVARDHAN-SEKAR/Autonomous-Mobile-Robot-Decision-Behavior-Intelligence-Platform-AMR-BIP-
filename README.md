# Autonomous Mobile Robot Decision & Behavior Intelligence Platform (AMR-BIP)

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![CI](https://github.com/HARSHAVARDHAN-SEKAR/amr-bip/actions/workflows/ci.yml/badge.svg)](https://github.com/HARSHAVARDHAN-SEKAR/amr-bip/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

AMR-BIP is a ROS 2 research and portfolio platform for studying the **executive
layer** of an autonomous mobile robot: mission scheduling, behavior selection,
preemption, recovery, and fault handling.

The same robot, mission set, navigation stack, battery model, and skills API
are controlled by three alternative decision architectures:

- **Behavior Tree (BT)**
- **Finite State Machine (FSM)**
- **Utility-based arbitration**

This keeps the experimental variable at the decision layer instead of changing
the rest of the autonomy stack between runs.

The navigation subsystem is intentionally **Nav2-free**: A* global planning and
pure-pursuit control are implemented explicitly and exposed through a
cancellable ROS 2 action so that executive preemption and recovery behavior are
easy to inspect.

```text
┌──────────────────────────────────────────────────────────────┐
│ Decision engines (select one)                                │
│  bt_executive  │  fsm_executive  │  utility_executive       │
├──────────────────── shared Skills API ───────────────────────┤
│ navigate · cancel · charge · battery · emergency · status    │
├──────────────────────────────────────────────────────────────┤
│ bip_navigation  A* + smoothing + pure pursuit + replanning   │
│ bip_battery     speed-dependent drain + dock charging         │
│ bip_chaos       battery / emergency / path-block fault input  │
├──────────────────────────────────────────────────────────────┤
│ Gazebo Classic 11 · diff-drive robot · 2D lidar · odometry   │
├──────────────────────────────────────────────────────────────┤
│ bip_benchmark   controlled reset/start + CSV/JSON metrics     │
└──────────────────────────────────────────────────────────────┘
```

## Packages

| Package | Purpose |
|---|---|
| `bip_core` | ROS-free occupancy grid, A*, BT primitives, FSM, utility arbiter, mission scheduler and mission progress logic |
| `bip_interfaces` | `GoToPose` action and custom battery/obstacle services |
| `bip_description` | Robot xacro, Gazebo world, RViz config and simulation bringup |
| `bip_navigation` | Cancellable `GoToPose` action server with A*, path smoothing, pure pursuit, emergency handling, scan blocking and virtual-map replanning |
| `bip_battery` | Speed-dependent battery drain and dock-only charging simulation |
| `bip_executive` | BT, FSM and utility decision engines using the same skills and mission driver |
| `bip_chaos` | Controlled fault injection for battery, emergency stop and virtual path blocking |
| `bip_benchmark` | Deterministic benchmark start/reset, fault scheduling, status traces and per-fault recovery metrics |

## Behaviors demonstrated

- Priority/deadline-based mission scheduling
- Mission preemption and resume at the same mission step
- Low-battery return-to-dock, charging and mission continuation
- Emergency stop at the navigation layer plus executive hold/resume
- Virtual path blockage followed by **immediate active-goal replanning**
- Scan-obstacle wait/replan with bounded retries and clean navigation failure
- Cancellable ROS 2 actions for executive-driven preemption
- Controlled BT vs FSM vs Utility experiments with common initial conditions

## Deterministic benchmark design

For benchmark runs, launch an executive with `wait_for_start:=true`. The
benchmark runner waits until the executive reports `WAITING_START`, then:

1. resets the Gazebo world pose using `/reset_world`;
2. stops battery charging;
3. restores battery to 100%;
4. clears all virtual planner obstacles;
5. clears emergency state;
6. waits for the configured settling interval; and
7. releases `/benchmark/start` and starts the experiment clock.

Fault schedules are therefore measured from the same mission-start event for
all three executive architectures.

The navigation server also publishes `/nav/events`. A virtual obstacle changes
the map revision and causes the **currently active goal** to replan immediately;
this makes the blocked-path benchmark a real behavioral disturbance rather than
an obstacle that only affects a future goal.

Per-fault JSON records contain fields such as:

```json
{
  "id": "block_1",
  "fault_type": "block",
  "scheduled_s": 10.0,
  "injected_s": 10.03,
  "recovered_s": 10.21,
  "recovery_time_s": 0.18,
  "outcome": "replanned"
}
```

## Quick start

For the full terminal-by-terminal workflow, smoke tests and benchmark protocol,
see **[RUN_GUIDE.md](RUN_GUIDE.md)**.

```bash
docker build -t amr-bip:humble -f docker/Dockerfile .
xhost +local:docker
docker compose up -d

docker exec -it amr-bip bash
cd /ws
colcon build --symlink-install
source install/setup.bash
```

Then launch in separate container terminals:

```bash
ros2 launch bip_description sim.launch.py
ros2 launch bip_navigation stack.launch.py
ros2 launch bip_executive executive.launch.py engine:=bt
```

Normal demonstrations start immediately. For controlled benchmarks:

```bash
ros2 launch bip_executive executive.launch.py engine:=bt wait_for_start:=true
```

and then in another terminal:

```bash
ros2 run bip_benchmark benchmark_node --ros-args \
  -p scenario:=full_chaos \
  -p out_dir:=/ws/results
```

Scenarios: `baseline`, `battery_drop`, `emergency`, `blocked_path`, and
`full_chaos`.

**Use a fresh executive process for every engine/scenario run.** The benchmark
resets the shared robot/world subsystems, while a newly launched executive
provides a fresh mission-progress state.

## Testing

The ROS-free core can be tested without ROS:

```bash
pytest ros2_ws/src/bip_core/test -q
```

The current suite contains **14 unit tests** covering A* behavior, inflation,
BT semantics, FSM hooks, utility hysteresis, scheduling, mission completion,
retry/skip behavior and mission progress across preemption.

Inside ROS 2:

```bash
cd /ws
colcon test --packages-select bip_core
colcon test-result --verbose
```

GitHub Actions additionally performs a ROS 2 Humble `colcon build` and a static
map/mission reachability check.

## Safety and experiment notes

- Final yaw alignment checks cancellation and emergency state continuously;
  an emergency can no longer be reported as a successful goal.
- A pose goal is not declared successful if final orientation fails to reach
  tolerance within the configured yaw-alignment timeout.
- `/nav/clear_obstacles` and `/chaos/clear_obstacles` prevent virtual obstacles
  from leaking between experiments.
- A benchmark run is marked unsuccessful when the mission set terminates with
  one or more failed/skipped missions.

## Project direction

AMR-BIP currently focuses on **executive architecture comparison**. Planned
extensions for world-state reasoning, risk assessment, explainable decisions
and adaptive arbitration are documented in **[ROADMAP.md](ROADMAP.md)**.

## Author

**Harshavardhan Coimbatore Sekar**  
GitHub: **@HARSHAVARDHAN-SEKAR**

## License

MIT — see [LICENSE](LICENSE).
