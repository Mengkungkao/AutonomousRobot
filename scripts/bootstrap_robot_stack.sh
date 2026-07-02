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

# Legacy migration: this script used to hand-roll the ROS 2 apt source (curl the
# key to /usr/share/keyrings, write a one-line ros2.list). That conflicts with
# the official ros2-apt-source package's deb822 ros2.sources file the moment
# both exist, because apt refuses two different Signed-By values for the same
# repo. Remove any leftover ros2.list from a previous run of this script before
# it can conflict with the official source installed below.
if [ -f /etc/apt/sources.list.d/ros2.list ] && grep -q "packages.ros.org" /etc/apt/sources.list.d/ros2.list 2>/dev/null; then
  echo "Removing legacy /etc/apt/sources.list.d/ros2.list (superseded by the official ros2-apt-source package)"
  sudo rm -f /etc/apt/sources.list.d/ros2.list
  sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
fi

OTHER_ROS_SOURCES="$(grep -rl "packages.ros.org" /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null | grep -vx "/etc/apt/sources.list.d/ros2.sources" || true)"
if [ -n "$OTHER_ROS_SOURCES" ]; then
  echo "Found packages.ros.org source(s) this script does not manage:" >&2
  echo "$OTHER_ROS_SOURCES" >&2
  echo "Remove the duplicate(s) to avoid a Signed-By conflict, then re-run." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y curl ca-certificates

if [ ! -f /etc/apt/sources.list.d/ros2.sources ]; then
  ROS_CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
  ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -m1 '"tag_name"' | cut -d'"' -f4)"
  if [ -z "$ROS_APT_SOURCE_VERSION" ]; then
    echo "Could not determine the current ros2-apt-source release from GitHub" >&2
    exit 1
  fi
  ROS_APT_DEB="$(mktemp --suffix=.deb)"
  curl -fL -o "$ROS_APT_DEB" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${ROS_CODENAME}_all.deb"
  sudo dpkg -i "$ROS_APT_DEB"
  rm -f "$ROS_APT_DEB"
fi

sudo apt-get update
sudo apt-get install -y "${INSTALL_PACKAGES[@]}" python3-rosdep python3-rosinstall-generator python3-vcstool build-essential

sudo rosdep init >/dev/null 2>&1 || true
rosdep update >/dev/null 2>&1 || true

sudo usermod -aG dialout "$USER" >/dev/null 2>&1 || true

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
