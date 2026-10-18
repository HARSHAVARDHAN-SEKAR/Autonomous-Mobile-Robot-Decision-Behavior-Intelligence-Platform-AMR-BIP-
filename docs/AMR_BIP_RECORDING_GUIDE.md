# AMR-BIP | Real Gazebo video: terminal-by-terminal

**Never substitute a generated demonstration or unrelated footage.** Record the actual robot and terminal state from your Ubuntu desktop.

## Preparation on Ubuntu host

```bash
cd ~/Downloads/Autonomous-Mobile-Robot-Decision-Behavior-Intelligence-Platform-AMR-BIP-
docker compose up -d --no-build
xhost +si:localuser:root
```

Launch **one** instance of each ROS stack. If already running, do not duplicate.

### Terminal 1 - simulator server

```bash
docker exec -it amr-bip bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch bip_description sim.launch.py headless:=true
```

### Terminal 2 - Gazebo GUI (software rendering)

```bash
docker exec -it amr-bip bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
gzclient --verbose
```

### Terminal 3 - Nav + battery + chaos

```bash
docker exec -it amr-bip bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch bip_navigation stack.launch.py
```

Check that this prints `Planning inflation: robot radius 0.30 m + margin 0.20 m = 0.50 m`.

### Terminal 4 - reset & evidence capture (two Ubuntu windows as necessary)

Inside Docker, after sourcing:

```bash
ros2 service call /reset_world std_srvs/srv/Empty '{}'
ros2 service call /nav/clear_obstacles std_srvs/srv/Trigger '{}'
ros2 service call /battery/stop_charging std_srvs/srv/Trigger '{}'
ros2 service call /battery/set bip_interfaces/srv/SetBattery '{level: 100.0}'
```

For recording a fresh run, **on the Ubuntu HOST**, in a separate terminal, run the command below *before* launching BT. The pipe and tee are on the host, so the output is saved into your Git repo instead of the container's `/ws` mount:

```bash
mkdir -p results/2026-10-18_bt_gazebo/capture

docker exec amr-bip bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic echo /executive/status --field data' \
  | tee results/2026-10-18_bt_gazebo/capture/bt_live_status.txt
```

Stop with `Ctrl+C` after completion; use `ps` if a nested `docker exec` remains running. NOTE: opening a topic monitor before the topic exists may print a warning until the node launches; harmless.

### Terminal 5 - launch BT only once

```bash
docker exec -it amr-bip bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch bip_executive executive.launch.py engine:=bt
```

### Terminal 6 - clean on-screen status

```bash
docker exec -it amr-bip bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
python3 /ws/src/bip_benchmark/scripts/bt_live_status_monitor.py
```

The monitor only prints state changes, mission transitions and periodic battery values. The original ROS status is still the authoritative source.

## Use OBS on Ubuntu desktop

1. Install OBS Studio if missing: `sudo apt update && sudo apt install obs-studio` (host, not Docker). Launch `obs`.
2. Add a **Screen Capture** source for the desktop or **Window Capture** for Gazebo. Include the clean terminal monitor side-by-side or as a small readable inset.
3. In OBS Settings -> Video, choose 1920x1080, 30 fps if performance allows. If running software-rendered Gazebo, prefer stable recording to high framerate.
4. In Settings -> Output -> Recording, use MKV as a crash-safe recording container. Remux to MP4 afterward via OBS File -> Remux Recordings if needed.
5. Hit **Start Recording** *before* launching the BT executive. Capture patrol, room inspection, battery transition to dock, charging, mission resume and a verifiable final result (if achieved).
6. Stop recording. Keep the raw file private if it includes username or home directory; crop or blur before publishing.
7. Cut a **45-90 second** highlight from genuine recorded footage. If rendering a long full simulation, speed up only with an on-screen `xN speed` label. Don't fabricate frame evidence.

Suggested truthful overlay headings: `ROS 2 Humble | Gazebo Classic 11`, `A* + pure pursuit (not Nav2)`, `Low battery -> docking -> charging -> resume`, `Simulation evidence, not physical robot test`.

## Check true outcome after recording

With executive running, use:

```bash
ros2 topic echo /executive/status --once --field data
```

A successful 3-mission claim is permitted only if this reports `state="DONE"`, all three mission names under `completed`, and `failed=[]`. If status disappears after shutdown, use the **saved capture log** to verify. When the battery has charged, do not manually set it during an autonomy demo, or clearly disclose interventions.

## Add final status / video to repo (Ubuntu host)

```bash
mkdir -p results/2026-10-18_bt_gazebo/video
cp ~/Videos/your-real-gazebo-recording.mp4 results/2026-10-18_bt_gazebo/video/amr_bip_bt_demo.mp4
```

If the video is large, use a GitHub Release or YouTube link instead of committing multi-hundred-MB videos. GitHub blocks files >100 MB, and large binaries can degrade the repository. Prefer a compressed 45-90s MP4 in repo if comfortably small.

Optionally compress the video with installed ffmpeg:

```bash
ffmpeg -i input.mkv -c:v libx264 -preset medium -crf 25 -pix_fmt yuv420p -c:a aac -b:a 128k final_demo.mp4
```

## A useful video sequence (honest footage only)

- 0:00-0:07 project title overlay, Gazebo robot + ROS2 status
- 0:07-0:20 patrol and autonomous inspection (actual robot motion)
- 0:20-0:34 low-battery status -> `CHARGE_NAVIGATING`
- 0:34-0:48 physical docking in Gazebo, `CHARGE_CHARGING`, battery increasing
- 0:48-1:02 resumed `deliver_package` step 2; complete only if the new run verifies it
- 1:02-1:10 charts and GitHub link; annotate metrics from the report, not invented results
