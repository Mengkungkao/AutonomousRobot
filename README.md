# mDetect ROS2 Robot Stack

This project changes the robot from a Pygame waypoint controller into a TurtleBot3-style ROS2 system.

## 0. Get the code

Clone this repository on both the Raspberry Pi and the Ubuntu workstation (or copy it over some other way, e.g. `scp`/USB drive) before running any of the steps below:

```bash
git clone <this-repository-url> AutonomousRobot
cd AutonomousRobot
```

Everything below assumes your current directory is the repository root.

## 1. System architecture

```text
Ubuntu workstation
  RViz2
  SLAM Toolbox / AMCL
  Nav2 global planner (A*)
  Nav2 Regulated Pure Pursuit controller
  Global and local costmaps
  Predefined waypoint client
            |
            | ROS2 DDS over Ethernet/Wi-Fi
            v
Raspberry Pi running Ubuntu Server + ROS2 Humble
  robot_state_publisher
  COIN-D6 -> /scan
  cmd_mux: priority mux over /cmd_vel (Nav2), /cmd_vel_teleop, /cmd_vel_manual,
           plus a front obstacle stop gate driven by /scan -> /cmd_vel_out
  serial bridge: /cmd_vel_out -> Arduino
  Arduino telemetry -> /odom, /imu/data, /joint_states and TF
            |
            | USB serial, 500000 baud
            v
Arduino Uno
  Four quadrature encoders
  MPU6050 yaw and yaw rate
  Four independent wheel-speed PID loops
  Forward, reverse, left, right and stop control
  500 ms communication watchdog
  Latched emergency stop
```

SSH is used to open a terminal and manage the Raspberry Pi. ROS2 topics are transported directly by DDS across the network; they are not carried inside the SSH session.

Nav2's `navigation_launch.py` always publishes its final velocity command on the
plain `/cmd_vel` topic (its internal `velocity_smoother` remap to that name is
hardcoded and cannot be renamed from an outer launch file without namespacing
the whole Nav2 stack). Because of that, `cmd_mux` reads `/cmd_vel` directly as
its Nav2 input and republishes the arbitrated, safety-gated result on
`/cmd_vel_out`, which is the topic the serial bridge actually subscribes to.
Manual test commands and teleop are unaffected: they already use
`/cmd_vel_manual` and `/cmd_vel_teleop` (see section 12).

## 2. Important coordinate convention

The updated odometry follows ROS conventions:

- `+X`: forward
- `+Y`: left
- positive yaw: counter-clockwise/left turn
- TF chain while mapping: `map -> odom -> base_footprint -> base_link -> laser`

## 3. Folder structure

```text
arduino_ros2_base_controller/
  arduino_ros2_base_controller.ino   the sketch to upload to the Arduino Uno
  QGPMaker_MotorShield.*             motor shield driver
  QGPMaker_Encoder.h                 quadrature encoder driver
  Adafruit_MS_PWMServoDriver.*       PCA9685 PWM driver used by the shield
  PinChangeInterrupt*.*              pin-change interrupt library for the encoders
ros2_ws/src/mdetect_robot/
  mdetect_robot/serial_bridge.py    Arduino <-> ROS2
  mdetect_robot/coin_d6_lidar.py    COIN-D6 -> LaserScan
  mdetect_robot/cmd_mux.py          teleop/manual/Nav2 priority and safety gate
  mdetect_robot/waypoint_cli.py     predefined Nav2 routes
  launch/robot.launch.py            Raspberry Pi launch
  launch/desktop_slam.launch.py     RViz + SLAM + Nav2
  launch/desktop_navigation.launch.py saved map + AMCL + Nav2
  config/base.yaml                  serial bridge and cmd_mux parameters
  config/lidar.yaml                 COIN-D6 driver parameters
  config/nav2_params.yaml           costmaps, A* and pure pursuit
  config/slam_toolbox.yaml
  config/waypoints.yaml
  urdf/mdetect_robot.urdf.xacro
scripts/
  bootstrap_robot_stack.sh          one-shot installer (desktop or pi mode)
  verify_robot_stack.sh             checks the stack is wired up correctly
  ros_network_env.sh                source this to set ROS2 DDS network env vars
  99-mdetect-robot.rules.example    udev rules template for stable serial ports
```

## 4. Arduino serial protocol

The Pi normally sends:

```text
VEL,<linear_mm_s>,<angular_deg_s>
```

Examples:

```text
VEL,100,0
VEL,0,45
VEL,0,-45
VEL,-100,0
STOP
ESTOP
CLEAR_ESTOP
RESET_ODOM
ZERO_YAW
CAL_IMU
PING
```

Testing aliases are also supported:

```text
FORWARD,100
REVERSE,100
LEFT,45
RIGHT,45
```

Arduino publishes one telemetry line at 20 Hz:

```text
T,time_ms,x_mm,y_mm,yaw_deg,vx_mm_s,wz_deg_s,
  tick1,tick2,tick3,tick4,
  wheel_speed1,wheel_speed2,wheel_speed3,wheel_speed4,
  pwm1,pwm2,pwm3,pwm4,estop,watchdog
```

## 5. Motor PID behaviour

For straight motion, all four wheels receive the same speed setpoint. Each motor has its own PID state and PWM output. This is intentional: equal speed does not usually require equal PWM because motor torque, gearbox friction and wheel loading are different.

Initial gains are:

```text
Kp = 0.25
Ki = 0.034
Kd = 0.003
```

Change all four motors at runtime:

```text
PID,0.25,0.034,0.003
```

Change one motor, where the motor number is 1 to 4:

```text
PIDM,4,0.28,0.040,0.003
```

## 6. Install the Arduino software

1. Install **MPU6050_tockn** from the Arduino Library Manager (Sketch > Include Library > Manage Libraries). The other libraries this sketch needs (QGPMaker motor shield/encoder driver, Adafruit PWM driver, PinChangeInterrupt) are already included as source files inside `arduino_ros2_base_controller/`, so nothing else needs to be installed for those.
2. Open `arduino_ros2_base_controller/arduino_ros2_base_controller.ino` in the Arduino IDE (opening the `.ino` file also loads the other `.cpp`/`.h` files in the same folder as sketch tabs).
3. Select Arduino Uno and upload.
4. Keep the robot still during startup IMU calibration.

The sketch assumes motor order:

```text
M1 front-left
M2 front-right
M3 rear-right
M4 rear-left
```

## 7. Install ROS2 on the Raspberry Pi

Use Ubuntu Server 22.04 with ROS2 Humble. From the repository root:

```bash
bash scripts/bootstrap_robot_stack.sh pi
```

This single script (safe to re-run) will:

- add the ROS2 apt repository and key if they are not already configured
- install `ros-humble-ros-base`, `robot_state_publisher`, `xacro` and the Python serial/YAML/colcon tooling
- add your user to the `dialout` group (needed for serial port access)
- symlink `ros2_ws/src/mdetect_robot` from this repo into `~/ros2_ws/src/`, so future `git pull`s in this repo are picked up without re-copying anything
- run `rosdep install` and `colcon build --symlink-install`
- install the udev rules from step 8 automatically, once you have replaced the placeholders in `scripts/99-mdetect-robot.rules.example`
- append the ROS2 environment (domain ID, `ROS_LOCALHOST_ONLY=0`) to `~/.bashrc`

Log out and back in afterwards so the `dialout` group change takes effect.

## 8. Stable Arduino and LiDAR port names

Inspect serial device identities:

```bash
ls -l /dev/serial/by-id/
udevadm info -a -n /dev/ttyACM0 | grep '{serial}' | head -1
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -1
```

Edit `scripts/99-mdetect-robot.rules.example` and replace the two serial placeholders with the real values, then either re-run `bash scripts/bootstrap_robot_stack.sh pi` (it will now install the rule since the placeholders are gone) or install it directly:

```bash
sudo cp scripts/99-mdetect-robot.rules.example /etc/udev/rules.d/99-mdetect-robot.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Expected device links:

```text
/dev/arduino_mdetect
/dev/coin_d6
```

The launch arguments can be used instead if the links are not configured:

```bash
ros2 launch mdetect_robot robot.launch.py \
  arduino_port:=/dev/ttyUSB1 \
  lidar_port:=/dev/ttyUSB0
```

## 9. Start the robot-side ROS2 backbone

`bootstrap_robot_stack.sh` already added ROS2 sourcing and the network env vars to `~/.bashrc`, so a fresh login shell has everything needed. In that shell, on the Raspberry Pi:

```bash
ros2 launch mdetect_robot robot.launch.py
```

In a second shell, verify the stack came up correctly:

```bash
bash scripts/verify_robot_stack.sh
```

This checks that `mdetect_serial_bridge`, `coin_d6_lidar` and `mdetect_cmd_mux` are running, and that `/odom`, `/imu/data`, `/scan`, `/joint_states`, `/cmd_vel_out` and the `/base/*` services are all present. You can also inspect things manually:

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /imu/data
ros2 topic echo /scan
ros2 run tf2_tools view_frames
```

## 10. Configure ROS2 networking

Both the Pi and workstation must:

- be on the same local network
- use the same `ROS_DOMAIN_ID`
- use `ROS_LOCALHOST_ONLY=0`
- allow ROS2/DDS UDP traffic through the firewall

On both machines:

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

Test from the workstation:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
```

When Wi-Fi blocks multicast, run a Fast DDS discovery server on the Pi:

```bash
fastdds discovery --server-id 0 -l 192.168.1.50 -p 11811
```

Then on both machines:

```bash
export ROS_DISCOVERY_SERVER=192.168.1.50:11811
```

## 11. Install and build on the Ubuntu workstation

From the repository root on the workstation:

```bash
bash scripts/bootstrap_robot_stack.sh desktop
```

This installs `ros-humble-desktop`, Nav2, SLAM Toolbox and `teleop_twist_keyboard`, symlinks `ros2_ws/src/mdetect_robot` into `~/ros2_ws/src/`, builds the workspace, and appends the ROS2 environment to `~/.bashrc` (same script as section 7, just with the `desktop` argument). Open a new shell afterwards, or source the two `setup.bash` files it printed at the end.

Run `bash scripts/verify_robot_stack.sh` here too to confirm the package built and (once `robot.launch.py` is running on the Pi) that the robot's topics and services are visible over the network.

## 12. Manual movement tests

Raise the wheels off the floor for the first test.

Forward:

```bash
ros2 topic pub -r 10 /cmd_vel_manual geometry_msgs/msg/Twist \
"{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Rotate left:

```bash
ros2 topic pub -r 10 /cmd_vel_manual geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Stop the publisher with `Ctrl+C`. The source timeout and Arduino watchdog will stop the robot.

Keyboard teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel_teleop
```

Emergency stop:

```bash
ros2 service call /base/emergency_stop std_srvs/srv/Trigger {}
```

Clear the latched stop:

```bash
ros2 service call /base/clear_emergency_stop std_srvs/srv/Trigger {}
```

Reset odometry:

```bash
ros2 service call /base/reset_odometry std_srvs/srv/Trigger {}
```

Zero the yaw reference and recalibrate the IMU (keep the robot still for the latter):

```bash
ros2 service call /base/zero_yaw std_srvs/srv/Trigger {}
ros2 service call /base/calibrate_imu std_srvs/srv/Trigger {}
```

## 13. Mapping with RViz, SLAM and Nav2

Keep `robot.launch.py` running on the Pi. On the workstation:

```bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch mdetect_robot desktop_slam.launch.py
```

The workstation runs:

- RViz2
- SLAM Toolbox
- Nav2 local and global costmaps
- NavFn A* global path planning
- Regulated Pure Pursuit path following
- Nav2 waypoint follower

Use RViz **2D Goal Pose** to test navigation while mapping.

Save the map:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/mdetect
```

## 14. Navigation using a saved map

```bash
ros2 launch mdetect_robot desktop_navigation.launch.py \
  map:=$HOME/maps/mdetect.yaml
```

Set the initial pose in RViz with **2D Pose Estimate**, then send a goal with **2D Goal Pose**.

## 15. Predefined waypoints from the workstation

Edit:

```text
ros2_ws/src/mdetect_robot/config/waypoints.yaml
```

Then rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Start the interactive route selector:

```bash
ros2 run mdetect_robot waypoint_cli
```

The client sends the selected route to Nav2's `/follow_waypoints` action. Nav2 calculates the path and sends velocity commands through the Pi to Arduino. Arduino does not store or solve map waypoints.

## 16. Required commissioning checks

### Encoder direction

Command slow forward motion. All four wheel speed values in Arduino telemetry must be positive. If one is negative, change that entry in:

```cpp
const int8_t ENCODER_SIGN[4]
```

### IMU direction

Rotate the robot left by hand. `/imu/data` yaw and `/odom` yaw should increase. If they decrease, change:

```cpp
const float IMU_YAW_SIGN = -1.0f;
```

### Straight-line PID

Drive 1 m at 0.10 m/s and compare all four wheel-speed telemetry values. Increase `Kp` slightly if a wheel reacts too slowly. Increase `Ki` slowly if it remains below target. Keep `Kd` small because encoder speed is noisy.

### Distance calibration

Measure actual travel over 1 m. Correct `WHEEL_DIAMETER_MM` or `COUNTS_PER_REV` if odometry distance is consistently scaled.

### Track width

Command an in-place 360-degree turn. Correct `TRACK_WIDTH_MM` if encoder-derived turning behaviour does not match the physical robot. Heading published to ROS still comes from the IMU.

### LiDAR alignment

A wall directly in front must appear at angle zero in `/scan`. Adjust the `laser_joint` yaw in the URDF if the scan is rotated in RViz.

## 17. Safety notes

- Test with wheels raised before floor testing.
- Keep a physical battery disconnect or hardware emergency-stop switch.
- The software emergency stop is latched in Arduino and must be explicitly cleared.
- The LiDAR stop gate stops forward commands inside 300 mm, but it does not replace a certified safety system.
- Tune the robot footprint, inflation radius and velocity limits before autonomous operation near people.
