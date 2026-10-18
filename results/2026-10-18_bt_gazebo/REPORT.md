# AMR-BIP | Gazebo BT Autonomy Validation

**Run date:** 18 October 2026  
**Evidence type:** One observed ROS 2 Humble / Gazebo Classic 11 Behavior Tree status capture  
**Robot/control:** Differential-drive AMR; lidar + odometry; A* global planner; smoothed path and pure-pursuit controller, **not Nav2**  
**Observation window:** 88.47 s, 880 JSON status samples (~10 Hz)

## Executive summary

This run **completed `patrol_perimeter` and `inspect_east_room`** before the captured interval. During the captured interval, `deliver_package` progressed to step 2, was interrupted by an automatic low-battery recharge sequence, and resumed **at the same mission step** after charging. The capture ends with delivery step 2 still navigating; **there is no recorded final delivery completion or `DONE` state**. The status trace contains no failed missions during this observed interval.

A separate direct `GoToPose` test to `(4.5, 3.0)` returned `SUCCEEDED` with **52.2659 s** total time. This was a distinct test and is not an end-to-end mission timing metric.

## Quantitative evidence

| Observation | From recorded trace |
|---|---|
| ROS 2 status samples | 880 |
| Capture length | 88.47 s |
| Battery at first observed delivery status | 35.9% |
| Battery when charging diversion began | 24.9% |
| Battery at dock/charge request | 16.3% |
| Battery when mission resumed | 80.3% |
| Battery at end of capture | 77.3% |
| Completed missions observed in status | `patrol_perimeter`, `inspect_east_room` |
| Mission preserved across recharge | `deliver_package`, step 2 |
| End of capture | `MISSION`, delivery step 2, `nav=running`; not `DONE` |
| Failure array during sampled interval | Empty (`[]`) |

## Timeline (elapsed time from first status)

| At (s) | State transition | Battery | Delivery step |
|---:|---|---:|---:|
| 0.0 | `MISSION` | 35.9% | 0 |
| 14.7 | `MISSION` | 30.2% | 1 |
| 16.4 | `MISSION` | 30.1% | 2 |
| 27.6 | `CHARGE_IDLE` | 24.9% | 2 |
| 27.8 | `CHARGE_NAVIGATING` | 24.9% | 2 |
| 49.6 | `CHARGE_REQUESTING` | 16.3% | 2 |
| 49.8 | `CHARGE_CHARGING` | 16.3% | 2 |
| 50.2 | `CHARGE_CHARGING` (charging) | 17.3% | 2 |
| 81.9 | `MISSION` (charging) | 80.3% | 2 |
| 82.4 | `MISSION` | 80.2% | 2 |

## Plots

![Battery profile](figures/battery_profile.png)

![Behavior Tree executive state timeline](figures/executive_state_timeline.png)

![Delivery step resume](figures/mission_resume.png)

These figures show a **simulated** battery model, not a hardware battery discharge curve.

## Code and runtime changes under evaluation

The successful isolated inspection goal followed a planning clearance experiment: `robot_radius=0.30 m` and `planning_clearance_margin=0.20 m` (total inflated planning clearance **0.50 m**). The LiDAR obstacle stopping threshold was not reduced as part of this experiment. This margin is an **experimental configuration**, not yet a statistically validated universal setting. An earlier run at the narrower clearance had blocked goals; runs differ in initial conditions and should not be used as a controlled comparison.

## Other observed validations (separate test sessions)

- `colcon build --symlink-install`: eight ROS 2 packages built.
- `colcon test --packages-select bip_core`: **22 tests, zero failures**, reported in terminal output.
- Static waypoint connectivity: **7 / 7** route-pair checks found a path in an offline map check. This is not evidence of collision-free real-time execution.
- Direct `GoToPose` `(4.5, 3.0, 0.0)`: **SUCCEEDED**, message `reached`, **52.265911 s**.

## Evidence and reproducibility

- `bt_status.jsonl`: raw JSON payloads extracted without changing data fields from the `ros2 topic echo /executive/status --field data` terminal capture.
- `bt_status.csv`: CSV projection with relative timestamps for analysis.
- `summary.json`: structured phase transitions and validation caveats.
- `figures/*.png`: plots generated only from `bt_status.jsonl`, with `tools/plot_bt_evidence.py`.
- `direct_goal_result.txt`: short excerpt of the independent action result.

Regenerate plots from repository root:

```bash
python3 -m pip install matplotlib numpy
python3 tools/plot_bt_evidence.py \
  --input results/2026-10-18_bt_gazebo/bt_status.jsonl \
  --out results/2026-10-18_bt_gazebo/figures
```

## Limitations / remaining tests

1. The **final `deliver_package` success and executive `DONE` state were not captured**. Do not claim full mission completion yet.
2. One BT trace is not a BT vs FSM vs Utility benchmark. Record controlled multiple seeds, matched resets and repeated runs before comparing performance.
3. Battery percentages and charge rates originate from a simulator; do not interpret them as measured power or battery hardware performance.
4. Runtime tests have not established repeated-run navigation success, emergency-stop recovery or real-robot safety.
5. Video has not yet been recorded or attached; the video folder must receive a genuine OBS capture before publishing.

## Recording follow-up

See `docs/AMR_BIP_RECORDING_GUIDE.md` for terminal setup, OBS workflow and the exact Git commands. Include the complete final result from a fresh capture if available.
