# AMR-BIP v1.0.0 Release Notes

This release is prepared for the first public GitHub publication.

## Reliability fixes

- Final-yaw alignment now respects action cancellation and emergency stop.
- Pose goals no longer report success when final yaw times out.
- Virtual obstacle injection increments a map revision and replans the active goal immediately.
- Added virtual-obstacle reset services to prevent cross-experiment contamination.

## Benchmark fixes

- Added optional executive `wait_for_start` gate.
- Added deterministic pre-run reset of Gazebo world pose, charging state, battery, virtual obstacles and emergency state.
- Benchmark clock starts only when the executive is released.
- Replaced the single recovery tracker with independent per-fault records.
- Added navigation event telemetry for blocked-path recovery measurement.
- Benchmark success is false when the mission set ends with failed/skipped missions.

## Repository quality

- Replaced placeholder maintainer metadata with the project author's GitHub identity.
- Completed ROS 2 package dependency declarations.
- Updated CI with syntax, XML, metadata, unit-test, map-reachability and ROS build checks.
- Added GitHub publishing guide and development roadmap.
