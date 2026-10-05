# GitHub Publishing Guide

Recommended repository name:

```text
amr-bip
```

Recommended GitHub description:

```text
ROS 2 decision & behavior intelligence platform comparing Behavior Trees, FSMs and utility arbitration for autonomous mobile robots under controlled faults.
```

Recommended topics:

```text
ros2
robotics
autonomous-mobile-robot
behavior-tree
finite-state-machine
utility-based-ai
robot-navigation
gazebo
docker
fault-injection
autonomy
robotics-simulation
```

## Create the repository

On GitHub, create a new empty repository under:

```text
HARSHAVARDHAN-SEKAR/amr-bip
```

Do not initialize it with a README, `.gitignore`, or license because this
project already contains them.

## Push the extracted project

Run from the directory containing this README:

```bash
git init
git branch -M main
git add .
git status
git commit -m "Initial release of AMR-BIP"
git remote add origin https://github.com/HARSHAVARDHAN-SEKAR/amr-bip.git
git push -u origin main
```

Before committing, verify that `git status` does not include generated ROS
folders such as:

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
```

After pushing, open the Actions tab and confirm the `ci` workflow is green
before treating the repository as the public release.
