#!/usr/bin/env bash
set -euo pipefail
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-robot-state-publisher ros-humble-xacro \
  python3-colcon-common-extensions python3-serial python3-yaml chrony
sudo usermod -aG dialout "$USER"
echo "Log out and back in so the dialout group change takes effect."
