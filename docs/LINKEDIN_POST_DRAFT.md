**LinkedIn post draft - publish after recording real video**

I have been developing AMR-BIP: an Autonomous Mobile Robot Decision & Behavior Intelligence Platform built with ROS 2 Humble, Docker and Gazebo Classic.

The goal is to examine how different decision-making approaches coordinate an autonomous mobile robot. The platform includes Behavior Tree, Finite State Machine and utility-based executives, sharing the same A* + pure-pursuit navigation stack (without Nav2), simulated differential-drive robot, 2D LiDAR and battery model.

In my latest Behavior Tree simulation, the robot completed its patrol and inspection missions. While executing a delivery task, the battery dropped below the charging threshold; the executive preempted the task, routed the robot back to its dock, charged it from 16.3% to approximately 80.3% and resumed delivery at the same step.

I also validated the previously blocked inspection waypoint with an independent navigation action: goal reached in 52.27 seconds. The ROS-free core test suite passed 22 tests.

I recorded the results, ROS 2 status evidence and reproducible battery/state plots in the GitHub repository. The full BT/FSM/Utility comparison and repeated-run robustness evaluation are next; the captured delivery trace does not yet establish final delivery completion.

[Add actual GitHub repository link]
[Attach your genuine Gazebo recording]

#ROS2 #Robotics #AutonomousMobileRobots #Gazebo #BehaviorTrees #Docker #AutonomousNavigation
