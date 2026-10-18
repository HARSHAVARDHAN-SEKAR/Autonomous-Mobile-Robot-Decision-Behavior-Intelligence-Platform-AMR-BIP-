# AMR-BIP — Clean-Clone Validation and Genuine Gazebo Video

This is a verification protocol, **not** a claim that the tests have passed.
Record your exact commit (`git rev-parse HEAD`), test outputs and run date.

## 1. Clean clone and host preparation (Ubuntu with Docker + X11)

```bash
git clone https://github.com/HARSHAVARDHAN-SEKAR/Autonomous-Mobile-Robot-Decision-Behavior-Intelligence-Platform-AMR-BIP-.git
cd Autonomous-Mobile-Robot-Decision-Behavior-Intelligence-Platform-AMR-BIP-
git rev-parse HEAD
git status --short
```

If applying a separately provided fix script, run it **after cloning**,
review `git diff`, and use the modified commit ID only after committing.

```bash
docker build -t amr-bip:humble -f docker/Dockerfile .
# For Gazebo GUI on X11; restrict/revoke this permission after recording.
xhost +local:docker
docker compose up -d --no-build
docker compose ps
```

The host runs `docker` commands. **Inside the container** Docker may not be
installed. Open as many host terminals as needed, each with:

```bash
docker exec -it amr-bip bash
source /opt/ros/humble/setup.bash
cd /ws
colcon build --symlink-install
source install/setup.bash
```

Build only once; in later container terminals source both setup scripts:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
```

Run tests before starting simulation:

```bash
cd /ws
colcon test --packages-select bip_core
colcon test-result --verbose
```

## 2. Start a real Gazebo simulation (three container terminals)

Terminal A (GUI):

```bash
ros2 launch bip_description sim.launch.py
```

Verify a **successful robot spawn**. Terminal B:

```bash
ros2 launch bip_navigation stack.launch.py
```

Verify the nav, battery and chaos nodes are ready. Before starting any
executive, test direct navigation in Terminal C:

```bash
ros2 action send_goal /go_to_pose bip_interfaces/action/GoToPose \
  "{x: 4.0, y: -3.0, yaw: 0.0}" --feedback
```

PASS: feedback decreases, robot moves in Gazebo and action result reports
success. Capture that result. If failure occurs, record `/nav/events`,
`/odom`, `/scan`, and the action result; **do not claim goal completion**.

## 3. Dock-navigation check (known prior failure)

Keep the executive stopped. To clear stale virtual obstacles:

```bash
ros2 service call /nav/clear_obstacles std_srvs/srv/Trigger "{}"
```

From the current robot pose, request the dock:

```bash
ros2 action send_goal /go_to_pose bip_interfaces/action/GoToPose \
  "{x: -5.2, y: 3.2, yaw: 0.0}" --feedback
```

PASS: action success **and** the robot physically reaches the configured
0.6 m dock radius. Then test the charging service:

```bash
ros2 service call /battery/set bip_interfaces/srv/SetBattery "{level: 18.0}"
ros2 service call /battery/start_charging std_srvs/srv/Trigger "{}"
ros2 topic echo /battery --field percentage
```

PASS: service returns success and published percentage increases. These are
manual skill-level tests, not yet proof of executive preemption/resume.

Note: `/reset_world` can be used to reset simulation state when Gazebo is up,
but if the desired pose is not restored, restart simulation before benchmarks.

## 4. Full executive video test

After a clean simulation launch and battery reset to 100%:

```bash
ros2 service call /battery/set bip_interfaces/srv/SetBattery "{level: 100.0}"
ros2 service call /nav/clear_obstacles std_srvs/srv/Trigger "{}"
ros2 launch bip_executive executive.launch.py engine:=bt
```

In a separate sourced container terminal:

```bash
ros2 topic echo /executive/status
```

For a complete run, verify `state=DONE` and the status contains all missions
completed with an empty `failed` list. A moving robot alone is not a full
mission-completion test. Stop the executive before testing another engine.

## 5. Fair benchmark comparison (after skill-level navigation is working)

Start Gazebo and stack once. For **each** engine/scenario, launch a NEW
executive process:

```bash
ros2 launch bip_executive executive.launch.py engine:=bt wait_for_start:=true
```

In another sourced terminal:

```bash
ros2 run bip_benchmark benchmark_node --ros-args \
  -p scenario:=baseline -p out_dir:=/ws/results
```

Start with baseline; next test `battery_drop`, `emergency`, `blocked_path`,
`full_chaos`. Repeat separately for engines `fsm` and `utility`. Check:

- Fresh waiting executive at start and a successful reset.
- Result `success=true`, `missions_failed=0` for any run advertised as complete.
- Every injected fault is acknowledged; reject `unresolved`,
  `service_unavailable`, `injection_failed` as successful recovery.
- Treat a `replanned` event as planner replanning, **not** proof that the robot
  subsequently navigated successfully past the obstacle.
- Record any timeout or dock-navigation failure rather than hiding it.

Save result JSON, CSV, execution logs, and commit SHA for every run. Use
results from multiple runs/seeds before claims about one engine being better.

## 6. Record a genuine screen capture on your Ubuntu HOST

OBS Studio is a good option when installed (use the application on the host,
not inside Docker). Configure Display Capture or Window Capture at 1920x1080,
30 FPS, recording to MKV; optionally remux to MP4 in OBS afterwards. Do not
use stock footage or AI-generated simulation as proof.

Suggested 60–90 second sequence:

1. 0–10 s: actual Gazebo Classic world and spawned AMR-BIP robot.
2. 10–35 s: robot starts moving on a verified goal; optionally show RViz
   `/planned_path` and `/scan` if available.
3. 35–65 s: real executive status + visible movement in the same recording.
4. 65–90 s: actual completed action/result or a fault reaction that you
   independently verified; never edit out a failure to imply success.

Record a separate longer, uncut verification video for the repository if
possible. Add the video, exact launch commands, and logs to README only after
confirming they are genuine and synchronized to the same run.

## 7. Publish gates

- [ ] Clean clone builds with Docker and ROS 2 Humble.
- [ ] Core tests pass, GitHub Actions passes for the resulting commit.
- [ ] Direct navigation returns success in Gazebo.
- [ ] BT mission/behavior shown in the video matches the logs.
- [ ] Dock charging/recovery is either verified or clearly described as WIP.
- [ ] Benchmark results are actual files from verified runs, not examples.
- [ ] No claims about hardware, Nav2, Isaac Sim or benchmark superiority
      without independent evidence.

Shutdown from host: `docker compose down`. Revoke GUI permission after capture:
`xhost -local:docker`.
