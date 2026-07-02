# Session 6 — desktop_slam.launch.py missing, stale symlink (root cause)

**Session ID:** `a873b233-fc45-42f0-85a3-ea11d1e593ce`
**Time:** 15:31:04 – 15:58:33 local

## Issue 1: launch file not found
```
ros2 launch mdetect_robot desktop_slam.launch.py
file 'desktop_slam.launch.py' was not found in the share directory of package 'mdetect_robot'
```
`desktop_slam.launch.py` doesn't exist in this package. Available launch files at the time: `robot_bringup.launch.py`, `workstation_navigation.launch.py`, `workstation_mapping.launch.py`. Assistant recommended `workstation_mapping.launch.py` (confirmed via grep it includes `slam_toolbox`'s `online_async_launch.py`).

## Issue 2: package not found at all (fresh terminal)
Running `ros2 launch mdetect_robot workstation_mapping.launch.py` then failed with `Package 'mdetect_robot' not found`, searching only unrelated workspaces. Diagnosed as a terminal that hadn't sourced `~/ros2_ws/install/setup.bash`. Also noted (but did not yet fix) that `~/.bashrc` had several redundant/duplicated ROS sourcing blocks accumulated from repeated bootstrap runs.

## Issue 3: real root cause
Once the user got it working and asked to update the project/README, deeper investigation (`git diff`, `readlink -f ~/ros2_ws/src/mdetect_robot`, comparing timestamps of installed vs. repo launch files) found the actual bug: `~/ros2_ws/src/mdetect_robot` had become a **real directory**, not a symlink to the repo — so `colcon build` was compiling and installing a stale/older copy of the package that predated the current `desktop_slam.launch.py` / `robot.launch.py` naming. That's why the old `workstation_mapping.launch.py` name worked while the actually-current `desktop_slam.launch.py` didn't. The user had already fixed it by hand (moved the stale copy aside, symlinked to the repo, rebuilt) before reporting back "now it work."

## Fix (made permanent)
- `scripts/bootstrap_robot_stack.sh`: now auto-detects a real (non-symlink) directory at `$WORKSPACE_DIR/src/mdetect_robot` and self-heals by moving it to a timestamped `.stale-backup` and symlinking the correct path back to the repo, instead of just warning.
- `README.md` (§18 Troubleshooting): new entry describing this failure mode and that re-running the bootstrap script now self-heals it.
- Verified `bash -n` on the edited script, confirmed nothing else in the repo referenced the old `workstation_*`/`robot_bringup` launch names, and cleaned up the leftover stale-backup directory on this machine.

## End of session
User confirmed the SLAM/mapping workflow was fully working. Assistant offered to commit the accumulated fixes (bootstrap self-heal, README troubleshooting, udev rules, new `teleop_keyboard.py`) — these were committed later the same day (`4e38d32`, `a6e67f9`, `d252b1d`, `32a2ecb`). Next documented steps for the user per the README: drive the robot to build the map, save it with `nav2_map_server map_saver_cli`, then switch to `desktop_navigation.launch.py map:=...` for autonomous navigation.
