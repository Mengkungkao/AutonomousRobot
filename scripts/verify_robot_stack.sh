#!/usr/bin/env bash
set -euo pipefail

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  # ROS 2's generated setup.bash references unset vars like COLCON_TRACE,
  # which trips `set -u`; relax it just for the sourcing.
  set +u
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
  set -u
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

echo "== ROS 2 environment =="
printenv | grep -E '^(ROS|AMENT|COLCON)' | sort || true

echo
echo "== Package discovery =="
if ros2 pkg prefix mdetect_robot >/dev/null 2>&1; then
  echo "OK   mdetect_robot found: $(ros2 pkg prefix mdetect_robot)"
else
  echo "FAIL mdetect_robot not found by ros2 pkg prefix. Did you colcon build and source install/setup.bash?"
fi

echo
echo "== Python syntax check =="
if [ -d "$HOME/ros2_ws/src/mdetect_robot" ]; then
  python3 -m compileall -q "$HOME/ros2_ws/src/mdetect_robot" && echo "OK   no Python syntax errors in mdetect_robot"
else
  echo "SKIP $HOME/ros2_ws/src/mdetect_robot not found"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo
  echo "ros2 CLI not on PATH; cannot check the live graph. Install/source ROS 2 first."
  exit 0
fi

NODE_LIST="$(ros2 node list 2>/dev/null || true)"
TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
SERVICE_LIST="$(ros2 service list 2>/dev/null || true)"

if [ -z "$NODE_LIST" ]; then
  echo
  echo "== Live ROS graph =="
  echo "No nodes visible yet. Start the stack first, e.g. on the robot:"
  echo "  ros2 launch mdetect_robot robot.launch.py"
  echo "and/or on the workstation:"
  echo "  ros2 launch mdetect_robot desktop_slam.launch.py"
  exit 0
fi

check() {
  local kind="$1" name="$2" list="$3"
  if grep -qxF "$name" <<<"$list"; then
    echo "OK   $kind $name"
  else
    echo "..   $kind $name not present (fine if that part of the stack isn't launched right now)"
  fi
}

echo
echo "== Expected robot-side nodes (from robot.launch.py) =="
check node /mdetect_serial_bridge "$NODE_LIST"
check node /coin_d6_lidar "$NODE_LIST"
check node /mdetect_cmd_mux "$NODE_LIST"
check node /robot_state_publisher "$NODE_LIST"

echo
echo "== Expected robot-side topics =="
check topic /odom "$TOPIC_LIST"
check topic /imu/data "$TOPIC_LIST"
check topic /joint_states "$TOPIC_LIST"
check topic /scan "$TOPIC_LIST"
check topic /diagnostics "$TOPIC_LIST"
check topic /cmd_vel "$TOPIC_LIST"
check topic /cmd_vel_out "$TOPIC_LIST"
check topic /cmd_vel_teleop "$TOPIC_LIST"
check topic /cmd_vel_manual "$TOPIC_LIST"
check topic /tf "$TOPIC_LIST"

echo
echo "== Expected robot-side services (mdetect_serial_bridge) =="
check service /base/emergency_stop "$SERVICE_LIST"
check service /base/clear_emergency_stop "$SERVICE_LIST"
check service /base/reset_odometry "$SERVICE_LIST"
check service /base/zero_yaw "$SERVICE_LIST"
check service /base/calibrate_imu "$SERVICE_LIST"

echo
echo "== Nav2/SLAM nodes (only present when a desktop_*.launch.py is running) =="
check node /controller_server "$NODE_LIST"
check node /planner_server "$NODE_LIST"
check node /bt_navigator "$NODE_LIST"
check node /slam_toolbox "$NODE_LIST"
check node /amcl "$NODE_LIST"

echo
echo "== Full graph dump =="
echo "-- nodes --"; echo "$NODE_LIST"
echo "-- topics --"; echo "$TOPIC_LIST"
echo "-- services --"; echo "$SERVICE_LIST"
