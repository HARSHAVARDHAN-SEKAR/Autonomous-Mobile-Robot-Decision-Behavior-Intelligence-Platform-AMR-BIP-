# AMR-BIP — Complete Behavior Tree Baseline Validation

## Experiment

- Date: 18 October 2026
- Environment: ROS 2 Humble, Gazebo Classic 11, Docker
- Executive: Behavior Tree (BT)
- Scenario: baseline, without scheduled fault injection
- Initial simulated battery: 100%
- Controlled start: enabled
- Gazebo world reset: enabled
- Benchmark timeout: 1200 seconds

## Recorded results

| Metric | Value |
|---|---|
| Benchmark success | True |
| Termination reason | mission_set_finished |
| Duration | 278.1 seconds |
| Missions completed | 3 |
| Missions failed | 0 |
| Executive state switches | 5 |
| Battery at final completion | Approximately 64.6% |
| Final executive state | DONE |

Completed missions:

1. patrol_perimeter
2. inspect_east_room
3. deliver_package

The benchmark detected completion from the mission-status
record immediately before the executive published DONE.

## Evidence files

- bt_baseline.csv — benchmark time-series metrics
- bt_baseline_summary.json — benchmark outcome
- bt_baseline_status.jsonl — ROS 2 status records from
  benchmark start through the first DONE state

The status JSONL is extracted from the original local
ROS 2 terminal capture. Records after the first DONE
state are excluded.

## Limitations

This is one successful controlled simulation run, not a
repeated-trial reliability result.

The baseline scenario did not schedule fault injections.
Physical robot performance was not evaluated.

The BT, FSM and Utility architectures have not yet been
compared experimentally using repeated controlled trials.

The previously published battery-recovery video documents
a separate low-battery simulation experiment.
