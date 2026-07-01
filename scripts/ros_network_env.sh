#!/usr/bin/env bash
# Source this file on BOTH the Raspberry Pi and Ubuntu workstation.
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
# For networks that block multicast, run a Fast DDS discovery server on the Pi
# and uncomment this on both machines:
# export ROS_DISCOVERY_SERVER=192.168.1.50:11811
