# Deep dive — Missing/stale directories (colcon duplicate package + shadowed workspace symlink)

**Parent session:** [2026-07-02-05-apt-source-rewrite-udev-teleop-estop.md](2026-07-02-05-apt-source-rewrite-udev-teleop-estop.md) (`8e28bfbd-1823-45d1-9f7d-1a534664ed92`), 15:15–15:41 local, plus root-cause confirmation in [2026-07-02-06-desktop-slam-launch-stale-symlink.md](2026-07-02-06-desktop-slam-launch-stale-symlink.md) (`a873b233`).
**Status: both fixed and confirmed working; the workspace symlink fix was made permanent/self-healing in `bootstrap_robot_stack.sh`.**

There were two distinct "directory" problems this session, both in `~/ros2_ws/src/`, hit back-to-back.

## Problem A — stray backup directory causing a colcon duplicate-name error
```
[0.332s] ERROR:colcon:colcon build: Duplicate package names not supported:
- mdetect_robot:
  - package_backups/mdetect_robot.20260616_231415
  - src/mdetect_robot
```
`colcon build` scans the whole workspace and found two directories both declaring the ROS2 package name `mdetect_robot`: the real one at `src/mdetect_robot`, and a leftover timestamped backup at `package_backups/mdetect_robot.20260616_231415` — almost certainly auto-created by an old version of an install script from June 16, before symlinking existed.

**Fix (non-destructive):** rather than deleting the backup, dropped a `COLCON_IGNORE` marker file into `package_backups/mdetect_robot.20260616_231415/` — colcon skips any directory containing that marker, so the backup stays on disk untouched but is no longer scanned. Confirmed with a search that no other stray timestamped backup directories existed elsewhere in the workspace. Rebuild succeeded afterward.

## Problem B — `~/ros2_ws/src/mdetect_robot` was a real directory, not a symlink to the repo
This was the deeper, more consequential issue and took two separate investigation passes across sessions to fully root-cause.

### First symptom (this session): "No executable found"
After adding `teleop_keyboard.py` to the repo and rebuilding, `ros2 run mdetect_robot teleop_keyboard ...` reported `No executable found` despite `colcon build --symlink-install --packages-select mdetect_robot` reporting success. Investigation:
1. Inspected installed console-script entry_points metadata for the built package.
2. Checked the egg-info location, the live `setup.py`, and — critically — where `~/ros2_ws/src/mdetect_robot` actually pointed.
3. Diffed the stale workspace copy against the repo copy.

**Root cause found:** `~/ros2_ws/src/mdetect_robot` was **not a symlink**. It was a real, disconnected directory dating from June 16 that had never been updated since — so every `colcon build` had been silently compiling and installing that stale snapshot instead of the actual repo at `~/AutonomousRobot/ros2_ws/src/mdetect_robot`. The stale copy only contained `serial_bridge` and `waypoint_runner`; it predated `cmd_mux`, `coin_d6_lidar`, `waypoint_cli`, and (of course) the brand-new `teleop_keyboard`.

The stale copy also contained files that don't exist in the repo at all:
- `robot_bringup.launch.py`
- `workstation_mapping.launch.py`
- `workstation_navigation.launch.py`
- `collision_monitor.yaml`
- `waypoint_runner.py`

The assistant flagged these as unknown-status (possibly still-needed features, possibly abandoned design) and used `AskUserQuestion` to confirm how to proceed before touching anything, rather than guessing.

### Fix applied (with user confirmation)
```bash
mv ~/ros2_ws/src/mdetect_robot ~/mdetect_robot.stale-backup   # preserve, don't delete
ln -s ~/AutonomousRobot/ros2_ws/src/mdetect_robot ~/ros2_ws/src/mdetect_robot
cd ~/ros2_ws && rm -rf build install log   # colcon's cached artifacts were keyed to the stale source
colcon build --symlink-install
```
Verified: `ros2 pkg executables mdetect_robot` then correctly listed `teleop_keyboard` alongside the other nodes.

### Second occurrence (Session 6): `desktop_slam.launch.py` not found
The same underlying bug resurfaced from a different angle in the next session: `ros2 launch mdetect_robot desktop_slam.launch.py` failed with "file not found," while the old, no-longer-current name `workstation_mapping.launch.py` worked. Root cause was identical — a real (non-symlink) `~/ros2_ws/src/mdetect_robot` shadowing the repo with older launch-file names — confirmed via `readlink -f`, `git diff`, and comparing install-vs-repo file timestamps. The user had already fixed it by hand before reporting back; the assistant then made the fix **permanent and self-healing**.

### Permanent fix — `scripts/bootstrap_robot_stack.sh`
```bash
mkdir -p "$WORKSPACE_DIR/src"
if [ -e "$WORKSPACE_DIR/src/mdetect_robot" ] && [ ! -L "$WORKSPACE_DIR/src/mdetect_robot" ]; then
  # A real (non-symlink) copy here silently shadows this repo: colcon builds
  # and installs whatever stale launch files/code happen to be in that copy,
  # `git pull` in the repo never touches it, and `ros2 launch` errors about
  # missing files that clearly exist in the repo. Move it aside once instead
  # of leaving the user to debug that mismatch by hand.
  STALE_BACKUP="$WORKSPACE_DIR/src/mdetect_robot.stale-backup.$(date +%Y%m%d%H%M%S)"
  echo "WARNING: $WORKSPACE_DIR/src/mdetect_robot is a real directory, not a symlink" >&2
  echo "back to this repo, so it was going stale. Moving it to $STALE_BACKUP" >&2
  echo "and linking $WORKSPACE_DIR/src/mdetect_robot -> $REPO_ROOT/ros2_ws/src/mdetect_robot instead." >&2
  mv "$WORKSPACE_DIR/src/mdetect_robot" "$STALE_BACKUP"
  ln -s "$REPO_ROOT/ros2_ws/src/mdetect_robot" "$WORKSPACE_DIR/src/mdetect_robot"
fi
```
(see `scripts/bootstrap_robot_stack.sh:97-109`). Every future run of `bash scripts/bootstrap_robot_stack.sh <desktop|pi>` now detects this exact condition automatically, timestamp-backs-up the stale copy, and restores the correct symlink — no manual diffing/moving required again.

`README.md` §18 ("Troubleshooting") documents the symptom, root cause, and self-healing fix under *"`ros2 launch mdetect_robot <file>.launch.py` says the file was not found, even though it exists in this repo"* ([README.md:499-519](../../README.md#L499-L519)), including the tell: `ros2 pkg prefix mdetect_robot` failing means the symlink is missing entirely; an *old* launch-file name working while the current one doesn't means it's a stale shadow copy.

## Still open
`~/mdetect_robot.stale-backup` on the workstation has not been reviewed. If any of `robot_bringup.launch.py`, `workstation_mapping.launch.py`, `workstation_navigation.launch.py`, `collision_monitor.yaml`, or `waypoint_runner.py` represent work worth keeping, they need to be manually merged into the repo before that backup directory is deleted — otherwise that logic is lost for good.
