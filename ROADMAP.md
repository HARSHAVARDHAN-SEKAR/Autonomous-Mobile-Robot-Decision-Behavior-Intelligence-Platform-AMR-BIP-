# AMR-BIP Roadmap

AMR-BIP v1 deliberately focuses on a controlled comparison of executive
architectures. Future work extends the platform upward toward richer robot
intelligence without replacing the safety-critical deterministic core.

## v1 — Executive architecture benchmark

- [x] Behavior Tree executive
- [x] Finite State Machine executive
- [x] Utility-based executive
- [x] Shared mission driver and robot skills
- [x] A* + pure-pursuit navigation
- [x] Battery/docking behavior
- [x] Emergency preemption
- [x] Virtual path-block fault injection
- [x] Deterministic benchmark start/reset
- [x] Per-fault recovery metrics
- [x] Docker + GitHub Actions + ROS-free unit tests

## v2 — Situation and risk reasoning

- [ ] Robot/world state aggregator
- [ ] Localization confidence state
- [ ] Collision-risk estimate
- [ ] Terrain/system-health risk channels
- [ ] Explicit safety supervisor independent of mission policy

## v3 — Explainable decisions

- [ ] `DecisionExplanation` interface
- [ ] Candidate behavior scores
- [ ] Selected-behavior reason codes
- [ ] Decision latency and confidence logging
- [ ] Timeline/dashboard visualization

## v4 — Adaptive arbitration

- [ ] Context-aware utility weight adaptation
- [ ] Offline learning from benchmark traces
- [ ] Safe parameter adaptation bounded by deterministic safety rules
- [ ] Comparison against fixed BT/FSM/utility baselines

## Evaluation extensions

- [ ] Repeat each scenario across multiple random seeds
- [ ] Report mean, standard deviation and confidence intervals
- [ ] Add CPU/memory utilization
- [ ] Add decision latency and navigation recovery latency
- [ ] Export comparison plots from benchmark summaries
