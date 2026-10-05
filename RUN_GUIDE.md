# AMR-BIP Run Guide

This guide uses Docker so the ROS 2 Humble + Gazebo Classic environment is
reproducible and does not modify the host ROS installation.

## Requirements

- Linux desktop
- Docker Engine
- Docker Compose plugin
- X server for Gazebo/RViz GUI use

---

## 1. Build the Docker image

From the repository root:

```bash
docker build -t amr-bip:humble -f docker/Dockerfile .
```

## 2. Allow GUI access

```bash
xhost +local:docker
```

## 3. Start the container

```bash
docker compose up -d
```

Equivalent plain Docker command:

```bash
docker run -d --name amr-bip --network host --ipc host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v $(pwd)/ros2_ws:/ws -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  amr-bip:humble sleep infinity
```

## 4. Build the ROS 2 workspace

```bash
docker exec -it amr-bip bash
source /opt/ros/humble/setup.bash
cd /ws
colcon build --symlink-install
source install/setup.bash
```

Expected result: all 8 packages build successfully.

---

# Normal interactive demo

## 5. Terminal 1 — simulation

```bash
docker exec -it amr-bip bash
ros2 launch bip_description sim.launch.py
```

Headless:

```bash
ros2 launch bip_description sim.launch.py headless:=true
```

The robot starts at approximately `(-4.0, 3.0)` in the configured Gazebo
world.

## 6. Terminal 2 — navigation, battery and chaos stack

```bash
docker exec -it amr-bip bash
ros2 launch bip_navigation stack.launch.py
```

Expected logs include:

```text
nav_server ready
battery ready
chaos node ready
```

## 7. Terminal 3 — choose one decision engine

```bash
docker exec -it amr-bip bash
ros2 launch bip_executive executive.launch.py engine:=bt
```

Alternative engines:

```bash
ros2 launch bip_executive executive.launch.py engine:=fsm
ros2 launch bip_executive executive.launch.py engine:=utility
```

Normal demo mode starts the mission queue immediately.

## 8. Terminal 4 — observe the executive

```bash
docker exec -it amr-bip bash
ros2 topic echo /executive/status
```

Other useful interfaces:

```bash
ros2 topic echo /battery --field percentage
ros2 topic echo /nav/events
ros2 topic echo /planned_path
```

RViz:

```bash
rviz2 -d /ws/src/bip_description/rviz/bip.rviz
```

---

# Manual smoke tests

## 9.1 Direct navigation

Stop the executive first, then:

```bash
ros2 action send_goal /go_to_pose bip_interfaces/action/GoToPose \
  "{x: 4.0, y: -3.0, yaw: 0.0}" --feedback
```

Expected: decreasing `distance_remaining` and a successful result after both
position and final yaw satisfy tolerance.

## 9.2 Battery preemption

With an executive running:

```bash
ros2 service call /chaos/set_battery bip_interfaces/srv/SetBattery \
  "{level: 18.0}"
```

Expected behavior: mission navigation is preempted, robot drives to the dock,
charging begins, battery reaches the resume threshold, and mission progress
continues from the interrupted step.

## 9.3 Emergency stop

```bash
ros2 service call /chaos/emergency std_srvs/srv/SetBool "{data: true}"
```

Release:

```bash
ros2 service call /chaos/emergency std_srvs/srv/SetBool "{data: false}"
```

The navigation action must not report success if emergency occurs during final
yaw alignment.

## 9.4 Controlled virtual path blockage

```bash
ros2 service call /chaos/block_path bip_interfaces/srv/AddObstacle \
  "{x: 1.2, y: -2.0, size_x: 1.0, size_y: 1.0}"
```

Expected: the navigation map revision changes, the **active goal replans
immediately**, `/planned_path` updates and `/nav/events` publishes a
`replan_succeeded` event if an alternative path exists.

Clear injected obstacles:

```bash
ros2 service call /chaos/clear_obstacles std_srvs/srv/Trigger "{}"
```

## 9.5 Core unit tests

```bash
cd /ws
colcon test --packages-select bip_core
colcon test-result --verbose
```

Expected: 14 tests pass.

---

# Controlled BT vs FSM vs Utility benchmark

## 10. Why the start gate matters

Do **not** benchmark an executive that has already been running missions. Fault
times must be relative to the same mission-start event for every architecture.

AMR-BIP provides a benchmark gate for this purpose.

## 11. Keep simulation and stack running

Terminal 1:

```bash
ros2 launch bip_description sim.launch.py headless:=true
```

Terminal 2:

```bash
ros2 launch bip_navigation stack.launch.py
```

## 12. Launch a fresh executive in waiting mode

For BT:

```bash
ros2 launch bip_executive executive.launch.py \
  engine:=bt wait_for_start:=true
```

The status topic should report:

```text
WAITING_START
```

Do the same with `engine:=fsm` or `engine:=utility` for later runs.

## 13. Start one benchmark scenario

In another terminal:

```bash
ros2 run bip_benchmark benchmark_node --ros-args \
  -p scenario:=blocked_path \
  -p out_dir:=/ws/results
```

Before releasing the executive, the benchmark automatically:

1. calls Gazebo `/reset_world`;
2. stops charging;
3. sets battery to 100%;
4. clears virtual obstacles;
5. clears emergency state;
6. waits 2 seconds for settling; and
7. publishes `/benchmark/start:=true` and starts `t=0`.

Available scenarios:

```text
baseline
battery_drop
emergency
blocked_path
full_chaos
```

Useful optional parameters:

```bash
-p timeout_s:=600.0
-p settle_s:=2.0
-p initial_battery_pct:=100.0
-p reset_world:=true
```

## 14. One fresh executive process per run

After a benchmark finishes, terminate that executive process and launch a new
one with `wait_for_start:=true` before the next scenario or engine.

This is required because world/battery/obstacle state can be reset externally,
but mission-progress state belongs to the executive process itself.

Recommended experiment matrix:

```text
BT       × baseline / battery_drop / emergency / blocked_path / full_chaos
FSM      × baseline / battery_drop / emergency / blocked_path / full_chaos
Utility  × baseline / battery_drop / emergency / blocked_path / full_chaos
```

## 15. Result files

Each run produces:

```text
/ws/results/<engine>_<scenario>.csv
/ws/results/<engine>_<scenario>_summary.json
```

CSV fields:

```text
t,state,battery,charging,emergency,nav,mission,step,n_completed,n_failed
```

Summary JSON includes:

- run success/failure
- termination reason
- total mission time
- missions completed/failed
- executive state-switch count
- controlled-reset metadata
- per-fault injection and recovery records

Recovery outcomes currently include:

- `replanned` for blocked-path map recovery
- `charged_and_resumed` for low-battery recovery
- `released` for emergency-hold recovery
- `unresolved`, `service_unavailable` or injection error states when recovery
  is not observed

---

# Clean shutdown

```bash
docker compose down
```

Optional clean rebuild:

```bash
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
```

---

# Troubleshooting

| Symptom | Check |
|---|---|
| Gazebo GUI does not open | Run `xhost +local:docker`; check `echo $DISPLAY` inside the container |
| `goal rejected: server busy` | An older goal is still active; let the executive cancel it or stop the previous client |
| Robot does not move | Verify `/odom`, `/scan`, `nav_server ready`, and that emergency is false |
| Benchmark waits forever | Executive must be launched with `wait_for_start:=true`; stack and Gazebo reset services must be running |
| Benchmark startup error mentions `reset_world` | Ensure Gazebo Classic + `gazebo_ros` are running, or set `-p reset_world:=false` only when you intentionally manage pose reset yourself |
| Charging request returns `not_at_dock` | Battery node requires the robot to be physically inside dock radius |
| Slow simulation | Use `headless:=true` and RViz instead of Gazebo GUI |
