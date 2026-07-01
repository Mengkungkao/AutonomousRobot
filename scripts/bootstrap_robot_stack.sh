#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-desktop}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

case "$MODE" in
  desktop)
    echo "Installing ROS 2 desktop stack for Ubuntu machine"
    INSTALL_PACKAGES=(
      "ros-${ROS_DISTRO}-desktop"
      "ros-${ROS_DISTRO}-navigation2"
      "ros-${ROS_DISTRO}-nav2-bringup"
      "ros-${ROS_DISTRO}-slam-toolbox"
      "ros-${ROS_DISTRO}-teleop-twist-keyboard"
      "python3-colcon-common-extensions"
      "python3-yaml"
      "python3-serial"
      "chrony"
    )
    ;;
  pi)
    echo "Installing ROS 2 base stack for Raspberry Pi"
    INSTALL_PACKAGES=(
      "ros-${ROS_DISTRO}-ros-base"
      "ros-${ROS_DISTRO}-robot-state-publisher"
      "ros-${ROS_DISTRO}-xacro"
      "python3-colcon-common-extensions"
      "python3-yaml"
      "python3-serial"
      "chrony"
    )
    ;;
  *)
    echo "Usage: $0 [desktop|pi]" >&2
    exit 1
    ;;
esac

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required" >&2
  exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "This installer expects Ubuntu/Debian apt" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release ca-certificates

if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  sudo mkdir -p /usr/share/keyrings
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

sudo apt-get update
sudo apt-get install -y "${INSTALL_PACKAGES[@]}" python3-rosdep python3-rosinstall-generator python3-vcstool build-essential

sudo rosdep init >/dev/null 2>&1 || true
rosdep update >/dev/null 2>&1 || true

sudo usermod -aG dialout "$USER" >/dev/null 2>&1 || true

mkdir -p "$WORKSPACE_DIR/src"
if [ -e "$WORKSPACE_DIR/src/mdetect_robot" ] && [ ! -L "$WORKSPACE_DIR/src/mdetect_robot" ]; then
  echo "WARNING: $WORKSPACE_DIR/src/mdetect_robot already exists and is a real directory," >&2
  echo "not a symlink back to this repo. It will not be updated by future git pulls." >&2
  echo "Remove it and re-run this script to link it to $REPO_ROOT/ros2_ws/src/mdetect_robot" >&2
elif [ ! -e "$WORKSPACE_DIR/src/mdetect_robot" ]; then
  ln -s "$REPO_ROOT/ros2_ws/src/mdetect_robot" "$WORKSPACE_DIR/src/mdetect_robot"
fi

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  cd "$WORKSPACE_DIR"
  rosdep install --from-paths src --ignore-src -r -y || true
  colcon build --symlink-install
fi

if [ "$MODE" = "pi" ]; then
  if grep -q "_HERE" "$REPO_ROOT/scripts/99-mdetect-robot.rules.example"; then
    echo "Skipping udev rule install: scripts/99-mdetect-robot.rules.example still has"
    echo "  ARDUINO_SERIAL_HERE / LIDAR_SERIAL_HERE placeholders. Find the real serial"
    echo "  numbers with 'udevadm info -a -n /dev/ttyACM0 | grep \"{serial}\"' (and ttyUSB0"
    echo "  for the LiDAR), edit the file, then re-run this script."
  else
    sudo install -m 0644 "$REPO_ROOT/scripts/99-mdetect-robot.rules.example" /etc/udev/rules.d/99-mdetect-robot.rules
    sudo udevadm control --reload-rules >/dev/null 2>&1 || true
    sudo udevadm trigger >/dev/null 2>&1 || true
  fi
fi

BASHRC_SNIPPET="""
# mDetect ROS 2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE_DIR}/install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
"""

if ! grep -Fq "mDetect ROS 2 environment" "$HOME/.bashrc" 2>/dev/null; then
  printf '\n%s\n' "$BASHRC_SNIPPET" >> "$HOME/.bashrc"
fi

echo "Bootstrap completed. Open a new shell or run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
