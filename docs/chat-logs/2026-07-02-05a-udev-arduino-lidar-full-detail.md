# Deep dive — Stable udev symlinks for Arduino + LiDAR

**Parent session:** [2026-07-02-05-apt-source-rewrite-udev-teleop-estop.md](2026-07-02-05-apt-source-rewrite-udev-teleop-estop.md) (`8e28bfbd-1823-45d1-9f7d-1a534664ed92`), 14:48–15:07 local
**Status: confirmed working, currently in production on this robot's Pi.**

## Symptom
Launching the full stack (`ros2 launch mdetect_robot robot.launch.py`) produced continuous warnings and never connected to either serial device:

```
[coin_d6_lidar-3] [WARN] [Errno 2] could not open port /dev/coin_d6: No such file or directory: '/dev/coin_d6'
[serial_bridge-2] [INFO] mDetect serial bridge ready: /dev/arduino_mdetect @ 500000 baud, cmd_vel=/cmd_vel_out, odom=/odom
[serial_bridge-2] [WARN] Cannot open Arduino serial port /dev/arduino_mdetect: No such file or directory
```

## Root cause
`mdetect_robot` nodes are hardcoded to expect stable device names (`/dev/arduino_mdetect`, `/dev/coin_d6`), which only exist if a udev rule creates them. That rule (`scripts/99-mdetect-robot.rules.example`) had never been installed on the Pi — `bootstrap_robot_stack.sh` deliberately skips auto-installing it while the file still contains placeholder text (checked via `grep -q "_HERE"`), and no one had filled in the real values yet.

Immediate workaround offered (not the final fix): launch with explicit ports:
```bash
ros2 launch mdetect_robot robot.launch.py \
  arduino_port:=/dev/ttyUSB1 \
  lidar_port:=/dev/ttyUSB0
```

## Diagnosis walk (why the naive approach didn't work)
Standard practice is to key a udev rule on `ATTRS{serial}` (the device's unique serial number), found via:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -1
udevadm info -a -n /dev/ttyUSB1 | grep '{serial}' | head -1
```
Both commands returned the **same** value: `ATTRS{serial}=="0000:01:00.0"`. That string is actually the PCI/USB host controller's serial, not the individual adapter's — a dead end for telling the two devices apart.

Full property dump was pulled instead:
```bash
udevadm info -q property -n /dev/ttyUSB0
udevadm info -q property -n /dev/ttyUSB1
```

This revealed both adapters are **identical CH340 USB-serial chips** — `ID_VENDOR_ID=1a86`, `ID_MODEL_ID=7523`, `ID_SERIAL=1a86_USB_Serial` on both — with no unique serial EEPROM burned in at the factory. Any rule matching on vendor/model/serial would match both devices identically and be useless.

The dump did show a per-device field that *does* differ: `ID_PATH`, which encodes the physical USB port location:

| Device | Node | `ID_PATH` |
|---|---|---|
| LiDAR | `/dev/ttyUSB0` | `platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0` |
| Arduino | `/dev/ttyUSB1` | `platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0` |

This is stable across reboots and reconnects — but **not** across moving an adapter to a different physical USB port on the Pi (the path number, e.g. `1.3` vs `1.4`, would change).

## The fix
Installed `/etc/udev/rules.d/99-mdetect-robot.rules` on the Pi with two `ID_PATH`-keyed rules:

```udev
SUBSYSTEM=="tty", ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0", SYMLINK+="arduino_mdetect", MODE="0660", GROUP="dialout"
SUBSYSTEM=="tty", ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0", SYMLINK+="coin_d6", MODE="0660", GROUP="dialout"
```

Applied with:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Verification (confirmed working)
```
$ ls -l /dev/arduino_mdetect /dev/coin_d6
lrwxrwxrwx 1 root root 7 Jul  2 15:04 /dev/arduino_mdetect -> ttyUSB1
lrwxrwxrwx 1 root root 7 Jul  2 15:04 /dev/coin_d6 -> ttyUSB0
```
Both symlinks resolved to the correct physical devices. User then ran the launch with no explicit port args and confirmed both nodes connected cleanly:
```bash
ros2 launch mdetect_robot robot.launch.py
```

## Made permanent / repo changes
1. **`scripts/99-mdetect-robot.rules.example`** — rewritten (twice) to:
   - Ship the actual working `ID_PATH` rule for this robot (both lines above), so `bootstrap_robot_stack.sh pi` auto-installs it on future provisions/reprovisions without manual editing.
   - Document in comments why `ATTRS{serial}` doesn't work here (identical CH340 chips, no EEPROM), the exact `udevadm` command to re-derive `ID_PATH` if an adapter is ever moved to a different port, and the current confirmed port mapping.
   - A second edit was needed because the first version's comment still contained the literal string `_HERE` (in unrelated wording), which would have falsely tripped the bootstrap script's placeholder check (`grep -q "_HERE"`) and silently skipped auto-install again. Verified after the fix with a re-check that no `_HERE` remained.
2. **`README.md` §8 "Stable Arduino and LiDAR port names"** — rewritten to reflect the real device mapping (LiDAR=`ttyUSB0`, Arduino=`ttyUSB1`), explain the CH340/no-EEPROM limitation, give the `ID_PATH` re-derivation command, and give both the automatic (`bootstrap_robot_stack.sh pi`) and manual (`cp` + `udevadm control --reload-rules` + `udevadm trigger`) install paths. Current text is at [README.md:201-243](../../README.md#L201-L243).

## Caveat / what would break this
If either the Arduino or the LiDAR is ever unplugged and reconnected to a **different physical USB port** on the Pi, its `ID_PATH` changes and the symlink will stop appearing. Fix in that case: re-run `udevadm info -q property -n /dev/ttyUSBn | grep ID_PATH=` for the new port, update the two `ID_PATH` values in `scripts/99-mdetect-robot.rules.example`, and reinstall the rule (automatically via bootstrap, or manually as above). This is documented inline in both the rules file and the README so it's discoverable next time it happens.
