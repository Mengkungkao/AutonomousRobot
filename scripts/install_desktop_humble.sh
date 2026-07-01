#!/usr/bin/env bash
set -euo pipefail
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-yaml chrony
