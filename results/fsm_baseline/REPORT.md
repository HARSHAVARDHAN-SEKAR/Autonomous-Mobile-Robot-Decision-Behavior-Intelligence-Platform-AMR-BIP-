# AMR-BIP — FSM Baseline Mission Validation

## Experiment

- Date: 18 October 2026
- Environment: ROS 2 Humble, Gazebo Classic 11, Docker
- Executive: Finite State Machine
- Scenario: baseline
- Initial simulated battery: 100%
- Controlled world reset: enabled
- Benchmark timeout: 1200 seconds

## Results

| Metric | Recorded result |
|---|---|
| Success | True |
| Termination | mission_set_finished |
| Duration | 279.6 seconds |
| Missions completed | 3 |
| Missions failed | 0 |
| State switches | 4 |
| Final executive state | DONE |

Completed missions:

1. patrol_perimeter
2. inspect_east_room
3. deliver_package

## Evidence

- fsm_baseline.csv
- fsm_baseline_summary.json
- fsm_baseline_status.jsonl

## Limitations

One controlled Gazebo simulation run. No scheduled
fault injection, repeated-trial reliability analysis,
or physical robot validation.

No architectural performance superiority is claimed.
