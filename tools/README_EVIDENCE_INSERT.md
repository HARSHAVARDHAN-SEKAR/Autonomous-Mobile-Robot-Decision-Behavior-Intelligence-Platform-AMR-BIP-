## Verified Gazebo Behavior Tree evidence (18 October 2026)

[Technical report (PDF)](results/2026-10-18_bt_gazebo/AMR_BIP_BT_Validation_Report.pdf) | [Detailed report + methodology](results/2026-10-18_bt_gazebo/REPORT.md) | [Battery plot](results/2026-10-18_bt_gazebo/figures/battery_profile.png) | [Executive state timeline](results/2026-10-18_bt_gazebo/figures/executive_state_timeline.png) | [Recording guide](docs/AMR_BIP_RECORDING_GUIDE.md)

In ROS 2 Humble / Gazebo Classic 11, a Behavior Tree run completed patrol and inspection, then interrupted `deliver_package` to navigate autonomously to the charging dock. The robot charged from 16.3% to about 80.3% (simulated battery model) and resumed delivery at the same mission step. A separate `GoToPose` inspection goal completed in 52.27 s; 22 ROS-free core unit tests passed. The captured BT status trace ends **before final delivery completion**. Neither a three-mission success nor comparative BT/FSM/Utility benchmark is claimed. Source JSONL and reproducible plots are committed under `results/2026-10-18_bt_gazebo/`.

