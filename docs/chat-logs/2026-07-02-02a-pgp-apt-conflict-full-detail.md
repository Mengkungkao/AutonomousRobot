# Deep dive — ROS2 apt "Signed-By" / PGP key conflict

**Occurred across two sessions:**
- First (symptom-level) fix: [2026-07-02-02-apt-conflict-pi-package-not-found.md](2026-07-02-02-apt-conflict-pi-package-not-found.md) (`ed2d182d`), 13:53–13:57 local
- Root-cause fix: [2026-07-02-05-apt-source-rewrite-udev-teleop-estop.md](2026-07-02-05-apt-source-rewrite-udev-teleop-estop.md) (`8e28bfbd`), 14:29–15:12 local
**Status: permanently fixed** — root cause resolved by rewriting the installer; confirmed clean apt state on the workstation with no recurrence risk from this script going forward.

## The error
```
E: Conflicting values set for option Signed-By regarding source http://packages.ros.org/ros2/ubuntu/ jammy:
   /usr/share/keyrings/ros-archive-keyring.gpg != -----BEGIN PGP PUBLIC KEY BLOCK-----
   mQINBFzvJpYBEADY8l1YvO7iYW5gUESyzsTGnMvVUmlV3XarBaJz9bGRmgPXh7jc
   ...
   -----END PGP PUBLIC KEY BLOCK-----
E: The list of sources could not be read.
```
This blocks `apt update` (and therefore `apt install`) entirely — every apt operation fails, not just ROS2 package installs.

## What it means
Two different files under `/etc/apt/sources.list.d/` both declared the same apt repo (`http://packages.ros.org/ros2/ubuntu jammy`), but told apt to trust it via two different mechanisms:
- **`ros2.sources`** — modern DEB822-format file, installed by the official `ros2-apt-source` `.deb` package. Its `Signed-By` field embeds the full ASCII-armored PGP public key block *inline*.
- **`ros2.list`** — legacy one-line format, historically produced by curling `ros.key` to `/usr/share/keyrings/ros-archive-keyring.gpg` and writing a `deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ...` line. Its `Signed-By` is a *file path*, not inline key content.

apt requires exactly one `Signed-By` declaration per repo; seeing a file path in one source and inline key text in another for the identical repo URL is treated as an unresolvable conflict, so it refuses to read *any* sources until it's fixed.

## Pass 1 — symptom fix (Session `ed2d182d`, then recurred, fixed properly in Session `8e28bfbd`)
Initial diagnosis found `bootstrap_robot_stack.sh`'s own duplicate-source detection was buggy:
> "The script's duplicate-detection is actually buggy: it only errors out if `ros2.list` is *not* among the matches — so once `ros2.list` exists (from this run), it stops complaining about `ros2.sources` being there too, even though that's exactly what's causing the conflict."

First fix (Session `ed2d182d`) moved the detection check to run *before* the script's own first `apt-get update` (previously the update itself failed before the check could even run). Immediate unblock required manual intervention since the assistant has no `sudo`:
```bash
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
```
This worked initially, but the file kept coming back — see Pass 2.

## Pass 2 — finding the real root cause (Session `8e28bfbd`)
The conflict recurred later the same day (`ros2.list` mtime showed it had been recreated). Rather than repeat the manual `rm`, the assistant investigated *why* it kept coming back:
- Checked shell history and file timestamps for what recreated `ros2.list`.
- Searched for any cron job, systemd timer, or background process writing the file — found none.
- Searched the filesystem more broadly and found remnants in Trash of an **abandoned prior project** (`mdetect_ros2_v2_turtlebot3`, deleted 22–30 June) that had hit this exact same conflict before and had already engineered the correct fix: an installer patch (`install_ros2_repo.sh`) that uses the official `ros2-apt-source` package instead of the legacy manual-key method. That fix never carried over when the current `AutonomousRobot` project was started fresh.

**Actual root cause:** `scripts/bootstrap_robot_stack.sh` was still using the legacy method (curl the key, write `ros2.list`) on a machine that *already* had `/etc/apt/sources.list.d/ros2.sources` installed the official way (via `ros2-apt-source`, back in April). Any time the legacy method ran again — via this script, or by hand-following ROS's classic install-key docs, or muscle memory from the old project — `ros2.list` would reappear and break apt again. Every `sudo rm` was only ever patching the symptom; the script itself was the thing that kept threatening to recreate the conflict.

## Fix — rewrote `bootstrap_robot_stack.sh` to stop using the legacy method entirely
Current logic (`scripts/bootstrap_robot_stack.sh:53-90`):
```bash
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
```

What changed vs. before:
1. **Auto-cleans** any leftover legacy `ros2.list` (and its keyring file) from a previous run of *this script*, before it can conflict with anything.
2. **Fails loudly** (`exit 1`) if it finds any *other*, unmanaged `packages.ros.org` source file it didn't create — e.g. if someone hand-pastes the classic ROS docs commands again — rather than silently letting apt hit the conflict later.
3. **Installs the ROS2 apt source exclusively via the official `ros2-apt-source` `.deb`** (fetched from its GitHub releases, matched to the OS codename) only if `ros2.sources` doesn't already exist — and never touches `ros2.list` again afterward.

Verified clean on the workstation immediately after: only `ros2.sources` present, `ros2.list` gone, `apt update` succeeds.

## README documentation
`README.md` §18 "Troubleshooting" ([README.md:478-497](../../README.md#L478-L497)) documents the final, correct explanation and fix:
- What the error means (two `Signed-By` declarations for one repo).
- That re-running `bash scripts/bootstrap_robot_stack.sh <desktop|pi>` now self-heals it.
- If it still fails, how to find any *other* unmanaged conflicting source file: `grep -rl "packages.ros.org" /etc/apt/sources.list /etc/apt/sources.list.d/` — keep `ros2.sources`, delete anything else.
- An explicit warning not to manually paste ROS's classic curl-key/`ros2.list` install steps on this machine, since `ros2-apt-source` already fully configures the repo and mixing the two methods is what causes the error.

## Why this one is worth remembering
The first fix (Pass 1) *looked* successful — apt worked again — but was incomplete: it treated a recreated file as the problem instead of asking why it kept recreating. The durable fix required tracing the conflict back to an architectural mismatch (two competing apt-source install methods) inherited from an abandoned sibling project, and eliminating the legacy method from the script entirely rather than continuing to clean up after it.
