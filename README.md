# mDetect ROS2 Robot Stack

This project changes the robot from a Pygame waypoint controller into a TurtleBot3-style ROS2 system.

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
  velocity priority + front obstacle stop gate
  /cmd_vel -> serial bridge
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

## 2. Important coordinate convention

The updated odometry follows ROS conventions:

- `+X`: forward
- `+Y`: left
- positive yaw: counter-clockwise/left turn
- TF chain while mapping: `map -> odom -> base_footprint -> base_link -> laser`

## 3. Folder structure

```text
arduino/
  mdetect_ros2_low_level/mdetect_ros2_low_level.ino
  libraries/                         custom QGPMaker and pin interrupt files
ros2_ws/src/mdetect_robot/
  mdetect_robot/serial_bridge.py    Arduino <-> ROS2
  mdetect_robot/coin_d6_lidar.py    COIN-D6 -> LaserScan
  mdetect_robot/cmd_mux.py          teleop/manual/Nav2 priority and safety gate
  mdetect_robot/waypoint_cli.py     predefined Nav2 routes
  launch/robot.launch.py            Raspberry Pi launch
  launch/desktop_slam.launch.py     RViz + SLAM + Nav2
  launch/desktop_navigation.launch.py saved map + AMCL + Nav2
  config/nav2_params.yaml           costmaps, A* and pure pursuit
  config/slam_toolbox.yaml
  config/waypoints.yaml
  urdf/mdetect_robot.urdf.xacro
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

1. Copy the folders under `arduino/libraries/` into the Arduino libraries directory.
2. Install **MPU6050_tockn** from Arduino Library Manager.
3. Open `arduino/mdetect_ros2_low_level/mdetect_ros2_low_level.ino`.
4. Select Arduino Uno and upload.
5. Keep the robot still during startup IMU calibration.

The sketch assumes motor order:

```text
M1 front-left
M2 front-right
M3 rear-right
M4 rear-left
```

## 7. Install ROS2 on the Raspberry Pi

Use Ubuntu Server 22.04 with ROS2 Humble.

```bash
cd mdetect_ros2_full_stack
bash scripts/install_pi_humble.sh
```

Copy the ROS2 package into the Pi workspace:

```bash
mkdir -p ~/ros2_ws/src
cp -r ros2_ws/src/mdetect_robot ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Add the user to the serial group if the script has not already done it:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in after changing groups.

## 8. Stable Arduino and LiDAR port names

Inspect serial device identities:

```bash
ls -l /dev/serial/by-id/
udevadm info -a -n /dev/ttyACM0 | grep '{serial}' | head -1
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -1
```

Edit `scripts/99-mdetect-robot.rules.example`, replace the serial placeholders, and install it:

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

On the Raspberry Pi:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 launch mdetect_robot robot.launch.py
```

Check the topics:

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

```bash
bash scripts/install_desktop_humble.sh
mkdir -p ~/ros2_ws/src
cp -r ros2_ws/src/mdetect_robot ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

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
