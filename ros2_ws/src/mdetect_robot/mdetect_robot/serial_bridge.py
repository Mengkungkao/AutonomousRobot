#!/usr/bin/env python3
"""ROS2 serial bridge for the mDetect Arduino low-level controller."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError as exc:  # pragma: no cover - runtime dependency check
    raise RuntimeError('pyserial is required: sudo apt install python3-serial') from exc


@dataclass
class Telemetry:
    """One decoded 'T,...' telemetry line from the Arduino: pose/velocity in
    Arduino units (mm, degrees), per-wheel encoder/speed/PWM arrays, and the
    e-stop / watchdog status flags."""
    arduino_ms: int
    x_mm: float
    y_mm: float
    yaw_deg: float
    vx_mm_s: float
    wz_deg_s: float
    ticks: list[int]
    wheel_speed_mm_s: list[float]
    wheel_pwm: list[float]
    estop: bool
    watchdog: bool


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    # Planar robot: only rotation about Z, so the quaternion is
    # (0, 0, sin(yaw/2), cos(yaw/2)).
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


class SerialBridge(Node):
    """Bridges the ROS 2 graph and the Arduino base controller over USB serial.

    Downstream (ROS -> Arduino): rate-limited 'VEL,<mm/s>,<deg/s>' commands from
    the muxed /cmd_vel_out topic, plus Trigger services mapped to one-shot
    commands (ESTOP, RESET_ODOM, ...).
    Upstream (Arduino -> ROS): 'T,...' telemetry lines republished as Odometry
    (+ optional odom->base TF), Imu, JointState, and periodic diagnostics.
    """

    def __init__(self) -> None:
        super().__init__('mdetect_serial_bridge')

        # Connection, topic naming, frame, and kinematic parameters. The
        # velocity limits clamp incoming commands before they reach the
        # Arduino; wheel geometry converts encoder ticks to joint states.
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 500000)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_out')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('reconnect_period_s', 2.0)
        self.declare_parameter('max_linear_m_s', 0.25)
        self.declare_parameter('max_angular_rad_s', 2.1)
        self.declare_parameter('wheel_diameter_m', 0.0805)
        self.declare_parameter('counts_per_rev', 4320.0)
        self.declare_parameter('serial_stale_s', 0.5)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.max_linear = float(self.get_parameter('max_linear_m_s').value)
        self.max_angular = float(self.get_parameter('max_angular_rad_s').value)
        self.wheel_radius = float(self.get_parameter('wheel_diameter_m').value) * 0.5
        self.counts_per_rev = float(self.get_parameter('counts_per_rev').value)
        self.serial_stale_s = float(self.get_parameter('serial_stale_s').value)
        self.reconnect_period_s = float(self.get_parameter('reconnect_period_s').value)

        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 20)
        self.imu_pub = self.create_publisher(Imu, imu_topic, 20)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 20)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, cmd_topic, self.cmd_vel_callback, 20)

        # One-shot base operations exposed as std_srvs/Trigger services; each
        # simply forwards a text command to the Arduino.
        self.create_service(Trigger, '/base/emergency_stop', self.handle_estop)
        self.create_service(Trigger, '/base/clear_emergency_stop', self.handle_clear_estop)
        self.create_service(Trigger, '/base/reset_odometry', self.handle_reset_odom)
        self.create_service(Trigger, '/base/zero_yaw', self.handle_zero_yaw)
        self.create_service(Trigger, '/base/calibrate_imu', self.handle_calibrate_imu)

        # Serial link state. 'rx_buffer' accumulates bytes until a full
        # newline-terminated line arrives. 'estop_requested' is the local latch
        # set by the service call; 'estop_latched_reported' mirrors what the
        # Arduino says in telemetry. 'watchdog_reported' starts True so
        # diagnostics warn until real telemetry proves otherwise.
        self.serial_port: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.rx_buffer = bytearray()
        self.last_connect_attempt = 0.0
        self.last_telemetry_monotonic = 0.0
        self.last_rx_text = ''
        self.last_cmd = Twist()
        self.estop_latched_reported = False
        self.estop_requested = False
        self.watchdog_reported = True

        # Three periodic jobs: stream velocity commands at command_rate_hz,
        # poll the serial port at 200 Hz, and publish diagnostics at 1 Hz.
        command_rate = max(1.0, float(self.get_parameter('command_rate_hz').value))
        self.create_timer(1.0 / command_rate, self.command_timer)
        self.create_timer(0.005, self.serial_read_timer)
        self.create_timer(1.0, self.diagnostics_timer)

        self.get_logger().info(
            f'mDetect serial bridge ready: {self.port} @ {self.baud} baud, '
            f'cmd_vel={cmd_topic}, odom={odom_topic}'
        )

    def connect_serial(self) -> bool:
        # Open the Arduino port if it is not already open. Reconnect attempts
        # are throttled to one every reconnect_period_s so a missing device
        # does not spam errors.
        if self.serial_port is not None and self.serial_port.is_open:
            return True

        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_period_s:
            return False
        self.last_connect_attempt = now

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.0,
                write_timeout=0.1,
            )
            # Give the board a moment after the port opens (opening the port
            # can reset the Arduino), discard stale bytes, then probe it.
            time.sleep(0.2)
            self.serial_port.reset_input_buffer()
            self.rx_buffer.clear()
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            self.send_line('PING')
            return True
        except (serial.SerialException, OSError) as exc:
            self.serial_port = None
            self.get_logger().warning(f'Cannot open Arduino serial port {self.port}: {exc}')
            return False

    def disconnect_serial(self, reason: str) -> None:
        # Close and forget the port after an I/O failure; connect_serial()
        # will retry on the next timer tick.
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.get_logger().error(f'Arduino serial disconnected: {reason}')

    def send_line(self, command: str) -> bool:
        # Send one newline-terminated ASCII command to the Arduino. The lock
        # serializes writes between the command timer and service callbacks.
        if not self.connect_serial():
            return False
        assert self.serial_port is not None
        try:
            payload = (command.strip() + '\n').encode('ascii', errors='strict')
            with self.serial_lock:
                self.serial_port.write(payload)
                self.serial_port.flush()
            return True
        except (serial.SerialException, OSError, UnicodeError) as exc:
            self.disconnect_serial(str(exc))
            return False

    def cmd_vel_callback(self, msg: Twist) -> None:
        # Cache the latest command, clamped to the configured limits; the
        # command timer streams it to the Arduino at a fixed rate.
        self.last_cmd.linear.x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        self.last_cmd.angular.z = max(-self.max_angular, min(self.max_angular, msg.angular.z))

    def command_timer(self) -> None:
        # Continuously resend the last command (the Arduino has a watchdog that
        # stops the motors if VEL lines stop arriving). Suppressed entirely
        # while an e-stop is requested or reported.
        if self.estop_requested or self.estop_latched_reported:
            return
        # Convert ROS units (m/s, rad/s) to Arduino units (mm/s, deg/s).
        linear_mm_s = self.last_cmd.linear.x * 1000.0
        angular_deg_s = math.degrees(self.last_cmd.angular.z)
        self.send_line(f'VEL,{linear_mm_s:.2f},{angular_deg_s:.2f}')

    def serial_read_timer(self) -> None:
        # Drain available bytes and dispatch each complete line. Non-blocking:
        # reads only what is already waiting.
        if not self.connect_serial():
            return
        assert self.serial_port is not None

        try:
            waiting = self.serial_port.in_waiting
            if waiting <= 0:
                return
            chunk = self.serial_port.read(min(waiting, 4096))
            if not chunk:
                return
            self.rx_buffer.extend(chunk)

            while b'\n' in self.rx_buffer:
                raw_line, _, remainder = self.rx_buffer.partition(b'\n')
                self.rx_buffer[:] = remainder
                line = raw_line.decode('ascii', errors='ignore').strip()
                if line:
                    self.handle_serial_line(line)
        except (serial.SerialException, OSError) as exc:
            self.disconnect_serial(str(exc))

    def handle_serial_line(self, line: str) -> None:
        # Route one line from the Arduino: 'T,...' is telemetry (parsed and
        # republished); ERR/INFO/READY are logged; ACK/PONG are debug noise.
        self.last_rx_text = line
        if line.startswith('T,'):
            telemetry = self.parse_telemetry(line)
            if telemetry is not None:
                self.last_telemetry_monotonic = time.monotonic()
                self.estop_latched_reported = telemetry.estop
                self.watchdog_reported = telemetry.watchdog
                self.publish_telemetry(telemetry)
            return

        if line.startswith(('ERR,', 'INFO,', 'READY,')):
            self.get_logger().info(f'Arduino: {line}')
        elif line.startswith(('ACK,', 'PONG')):
            self.get_logger().debug(f'Arduino: {line}')

    def parse_telemetry(self, line: str) -> Optional[Telemetry]:
        # Decode a comma-separated telemetry line into a Telemetry record;
        # returns None (with a warning) on any malformed field.
        fields = line.split(',')
        # T + 6 base fields + 4 ticks + 4 wheel speeds + 4 PWM + 2 flags = 21
        if len(fields) != 21:
            self.get_logger().warning(f'Invalid telemetry field count {len(fields)}: {line[:120]}')
            return None
        try:
            return Telemetry(
                arduino_ms=int(fields[1]),
                x_mm=float(fields[2]),
                y_mm=float(fields[3]),
                yaw_deg=float(fields[4]),
                vx_mm_s=float(fields[5]),
                wz_deg_s=float(fields[6]),
                ticks=[int(v) for v in fields[7:11]],
                wheel_speed_mm_s=[float(v) for v in fields[11:15]],
                wheel_pwm=[float(v) for v in fields[15:19]],
                estop=bool(int(fields[19])),
                watchdog=bool(int(fields[20])),
            )
        except ValueError as exc:
            self.get_logger().warning(f'Cannot parse telemetry: {exc}: {line[:120]}')
            return None

    def publish_telemetry(self, data: Telemetry) -> None:
        # Fan one telemetry sample out to all ROS consumers: Odometry,
        # optional odom->base TF, Imu, and JointState — all with one stamp.
        stamp = self.get_clock().now().to_msg()
        yaw_rad = math.radians(data.yaw_deg)
        q = yaw_to_quaternion(yaw_rad)

        # Odometry: convert Arduino mm/deg units to m/rad. Covariance is
        # meaningful only for the planar dimensions (x, y, yaw); the rest are
        # set huge so downstream fusion ignores them.
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = data.x_mm / 1000.0
        odom.pose.pose.position.y = data.y_mm / 1000.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = data.vx_mm_s / 1000.0
        odom.twist.twist.angular.z = math.radians(data.wz_deg_s)
        odom.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03,
        ]
        odom.twist.covariance = [
            0.03, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.03, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        self.odom_pub.publish(odom)

        # Broadcast odom->base_footprint unless something else (e.g. an EKF)
        # owns that transform.
        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = q
            self.tf_broadcaster.sendTransform(transform)

        # IMU message carries only yaw orientation and Z angular rate; the
        # -1 in linear_acceleration_covariance[0] marks acceleration as
        # unavailable per the sensor_msgs/Imu convention.
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation = q
        imu.angular_velocity.z = math.radians(data.wz_deg_s)
        imu.orientation_covariance = [9999.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.03]
        imu.angular_velocity_covariance = [9999.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.05]
        imu.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        # Joint states for the four wheels: encoder ticks -> wheel angle (rad),
        # rim speed (mm/s) -> angular velocity (rad/s).
        joint = JointState()
        joint.header.stamp = stamp
        # Order matches the Arduino motor numbering: 1 FR, 2 FL, 3 RL, 4 RR.
        joint.name = [
            'front_right_wheel_joint',
            'front_left_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]
        radians_per_tick = 2.0 * math.pi / self.counts_per_rev
        joint.position = [tick * radians_per_tick for tick in data.ticks]
        joint.velocity = [
            (speed_mm_s / 1000.0) / self.wheel_radius
            for speed_mm_s in data.wheel_speed_mm_s
        ]
        self.joint_pub.publish(joint)

    def diagnostics_timer(self) -> None:
        # Publish a 1 Hz health summary. Severity, worst first: port closed >
        # stale telemetry > e-stop latched > watchdog tripped > OK.
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'mDetect Arduino base controller'
        status.hardware_id = self.port

        connected = self.serial_port is not None and self.serial_port.is_open
        age = time.monotonic() - self.last_telemetry_monotonic if self.last_telemetry_monotonic else math.inf

        if not connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Serial port disconnected'
        elif age > self.serial_stale_s:
            status.level = DiagnosticStatus.ERROR
            status.message = f'Telemetry stale ({age:.2f} s)'
        elif self.estop_latched_reported:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Emergency stop latched'
        elif self.watchdog_reported:
            status.level = DiagnosticStatus.WARN
            status.message = 'Arduino command watchdog stopped the motors'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Base controller operational'

        status.values = [
            KeyValue(key='port', value=self.port),
            KeyValue(key='baud', value=str(self.baud)),
            KeyValue(key='telemetry_age_s', value='inf' if math.isinf(age) else f'{age:.3f}'),
            KeyValue(key='estop_latched', value=str(self.estop_latched_reported)),
            KeyValue(key='watchdog_stopped', value=str(self.watchdog_reported)),
            KeyValue(key='last_rx', value=self.last_rx_text[:120]),
        ]
        msg.status.append(status)
        self.diag_pub.publish(msg)

    def trigger_command(self, response: Trigger.Response, command: str, message: str) -> Trigger.Response:
        # Shared helper for all Trigger services: send the command and report
        # success/failure back to the caller.
        response.success = self.send_line(command)
        response.message = message if response.success else 'Arduino serial port is not connected'
        return response

    def handle_estop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        # Zero the cached command and latch the local e-stop flag (which also
        # stops the VEL stream) before telling the Arduino to stop.
        self.last_cmd = Twist()
        self.estop_requested = True
        return self.trigger_command(response, 'ESTOP', 'Emergency stop command sent and latched')

    def handle_clear_estop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        # Only release the local latches if the clear command actually reached
        # the Arduino.
        self.last_cmd = Twist()
        result = self.trigger_command(response, 'CLEAR_ESTOP', 'Emergency stop clear command sent')
        if result.success:
            self.estop_requested = False
            self.estop_latched_reported = False
        return result

    def handle_reset_odom(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.last_cmd = Twist()
        return self.trigger_command(response, 'RESET_ODOM', 'Odometry reset command sent')

    def handle_zero_yaw(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.last_cmd = Twist()
        return self.trigger_command(response, 'ZERO_YAW', 'Yaw zero command sent')

    def handle_calibrate_imu(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.last_cmd = Twist()
        return self.trigger_command(
            response,
            'CAL_IMU',
            'IMU calibration started; keep the robot completely still',
        )

    def destroy_node(self) -> bool:
        # Best effort on shutdown: command the motors to stop, then close the
        # port. Errors are ignored because the node is going away regardless.
        try:
            self.send_line('STOP')
        except Exception:
            pass
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
