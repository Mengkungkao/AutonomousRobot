# Deep dive — Custom teleop_keyboard node

**Parent session:** [2026-07-02-05-apt-source-rewrite-udev-teleop-estop.md](2026-07-02-05-apt-source-rewrite-udev-teleop-estop.md) (`8e28bfbd-1823-45d1-9f7d-1a534664ed92`), 15:28–15:49 local
**Status: confirmed working end-to-end** ("great it work now") — the node itself worked on the first correct build; two unrelated environment bugs (stale symlink, latched e-stop) blocked it along the way and both were root-caused and fixed.

## Request
Custom key bindings, different from the stock `teleop_twist_keyboard`:
```
W forward
S backward
A rotate LEft
D rotate Right
,/. : increase/decrease max speeds by 10%
-/+ : increase/decrease only linear speed by 10%
[/] : increase/decrease only angular speed by 10%
```

## Implementation
Read the installed stock node for reference (`/opt/ros/humble/lib/python3.10/site-packages/teleop_twist_keyboard.py`) to reuse its raw-terminal keystroke-reading and publishing loop rather than reinventing it, then wrote a new node:

- **New file:** `ros2_ws/src/mdetect_robot/mdetect_robot/teleop_keyboard.py`
- **Registered as a console script** in `ros2_ws/src/mdetect_robot/setup.py` (entry_points), so it's runnable as `ros2 run mdetect_robot teleop_keyboard`
- **Final key mapping:** `w`/`s` forward/backward, `a`/`d` rotate left/right, `,`/`.` scale both linear+angular speed ±10%, `-`/`+` scale linear speed only, `[`/`]` scale angular speed only, any other key stops the robot
- Byte-compiled (`python3 -m py_compile`) to confirm no syntax errors before moving on
- `README.md` updated in two places: the run instructions (§ around line 336) and the package file-tree listing (line 81) to list the new file

## How to run it
```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select mdetect_robot
source install/setup.bash
ros2 run mdetect_robot teleop_keyboard --ros-args --remap cmd_vel:=/cmd_vel_teleop
```
Requirements: must be run in a real (non-piped, non-backgrounded) terminal since it reads raw stdin keystrokes; `cmd_mux` must be running to forward `/cmd_vel_teleop` onward (the launch files start it automatically).

## Bug 1 hit while testing: "No executable found"
After the first rebuild, `ros2 run mdetect_robot teleop_keyboard ...` reported `No executable found` even though the build reported success. This was **not a teleop bug** — see the companion write-up [2026-07-02-05c-missing-directories-full-detail.md](2026-07-02-05c-missing-directories-full-detail.md) for the full root-cause (a stale non-symlinked `~/ros2_ws/src/mdetect_robot` directory was shadowing the repo). Once that was fixed and the workspace rebuilt clean, `ros2 pkg executables mdetect_robot` correctly listed `teleop_keyboard`.

## Bug 2 hit while testing: commands didn't move the robot
After the executable was found and ran, keypresses still didn't move the robot ("the command is not work with the robot communication, like the old command vel"). Diagnosis, in order:
1. Read `cmd_mux.py` to understand the `/cmd_vel_teleop` → `/cmd_vel_out` forwarding logic.
2. Checked ROS domain, running nodes, and topic list on the machine.
3. Wrote a small throwaway ROS2 listener subscribed to both `/cmd_vel_teleop` and `/cmd_vel_out`.
4. Simulated a `w` keypress by driving `teleop_keyboard` inside a pty (since it needs a real terminal for raw stdin) and watched the topics — confirmed `linear.x=0.5` flowed cleanly from `/cmd_vel_teleop` through `cmd_mux` to `/cmd_vel_out`. **The node and the topic pipeline were fine.**
5. Checked QoS/pub-sub wiring, serial device presence, `serial_bridge` config, and finally `/diagnostics`:
   ```
   $ ros2 topic echo /diagnostics --once
   status:
   - name: mDetect Arduino base controller
     message: Emergency stop latched
     values:
     - estop_latched: 'True'
     - watchdog_stopped: 'False'
   ```
   The Arduino base controller's **emergency stop was latched**, so `serial_bridge` was silently refusing to forward *any* velocity command to the Arduino — old teleop, new teleop, or a manual `ros2 topic pub`, it made no difference. This was unrelated to the new node; something (bumper contact, low battery, a prior e-stop service call, or a physical hardware button) had tripped it earlier.

## Fix for bug 2
```bash
ros2 service call /base/clear_emergency_stop std_srvs/srv/Trigger {}
```
Confirmed cleared by re-checking diagnostics — status changed to `"Base controller operational"`. Teleop then worked immediately; user confirmed with "great it work now."

## Recap for future reference
If teleop (or any `cmd_vel` source) stops moving the robot again:
1. Check whether it's actually reaching the hardware: `ros2 topic echo /diagnostics | grep -A6 "mDetect Arduino"`.
2. If it shows `Emergency stop latched`, clear it: `ros2 service call /base/clear_emergency_stop std_srvs/srv/Trigger {}`.
3. Investigate *why* it latched (bumper, low battery, stray service call) — this was never identified and is called out as an open follow-up in the day's index.
