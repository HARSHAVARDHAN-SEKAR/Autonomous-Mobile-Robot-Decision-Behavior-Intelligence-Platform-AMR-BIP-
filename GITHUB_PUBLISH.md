# AMR-BIP — Publishing and release workflow

This repository is **already public** at:

https://github.com/HARSHAVARDHAN-SEKAR/Autonomous-Mobile-Robot-Decision-Behavior-Intelligence-Platform-AMR-BIP-

Do not follow the old `git init` or `remote add` instructions: they used an
obsolete repo name (`amr-bip`) and could push to the wrong remote.

## Clone the actual project

```bash
git clone https://github.com/HARSHAVARDHAN-SEKAR/Autonomous-Mobile-Robot-Decision-Behavior-Intelligence-Platform-AMR-BIP-.git
cd Autonomous-Mobile-Robot-Decision-Behavior-Intelligence-Platform-AMR-BIP-
git remote -v
git rev-parse HEAD
```

For updates, create a branch, run the checks in `VALIDATION_AND_VIDEO.md`,
review `git diff`, and open a pull request. Do not commit generated
`ros2_ws/build`, `ros2_ws/install`, or `ros2_ws/log` folders.

For a LinkedIn post, link the real repository, cite the run date and Git SHA,
and describe only scenarios proven by captured logs and video.
