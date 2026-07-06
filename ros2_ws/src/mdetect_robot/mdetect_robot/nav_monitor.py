#!/usr/bin/env python3
"""Live health monitor for the Nav2 -> mux -> Arduino pipeline.

Run it (usually on the desktop, next to Nav2) while navigating:

    ros2 run mdetect_robot nav_monitor

Every second it prints one status line covering the whole chain —
sensor rates, TF localization, Nav2 output, mux output, base health —
and, when something is wrong, a WARN line naming the broken link. The
goal is that "Nav2 doesn't work" becomes "line X says AMCL never
localized" (or "mux is zeroing commands", "e-stop latched", ...).
"""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'unknown', GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING', GoalStatus.STATUS_CANCELING: 'canceling',
    GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class TopicProbe:
    """Message counter + freshness tracker for one monitored topic."""

    def __init__(self):
        self.count = 0          # messages since the last report
        self.last_time = None   # monotonic stamp of the newest message
        self.last_msg = None

    def hit(self, msg):
        self.count += 1
        self.last_time = time.monotonic()
        self.last_msg = msg

    def rate(self, period_s):
        rate = self.count / period_s
        self.count = 0
        return rate

    def age(self):
        return math.inf if self.last_time is None else time.monotonic() - self.last_time


class NavMonitor(Node):
    def __init__(self):
        super().__init__('nav_monitor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan = TopicProbe()
        self.odom = TopicProbe()
        self.nav_cmd = TopicProbe()     # /cmd_vel   = Nav2 (velocity smoother) output
        self.mux_out = TopicProbe()     # /cmd_vel_out = what the Arduino bridge consumes
        self.amcl = TopicProbe()
        self.base_status = None         # latest DiagnosticStatus from the serial bridge

        self.create_subscription(LaserScan, '/scan', self.scan.hit, 10)
        self.create_subscription(Odometry, '/odom', self.odom.hit, 20)
        self.create_subscription(Twist, '/cmd_vel', self.nav_cmd.hit, 20)
        self.create_subscription(Twist, '/cmd_vel_out', self.mux_out.hit, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl.hit, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)

        # Nav2 action status: which goal/behavior is actually driving the
        # robot right now. Spin/backup EXECUTING means the motion is a
        # failure RECOVERY, not path following. Action status topics are
        # latched (transient_local), so match that QoS.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.action_status = {}   # action name -> latest GoalStatus code
        for action in ('navigate_to_pose', 'follow_path', 'spin', 'backup', 'drive_on_heading', 'wait'):
            self.create_subscription(
                GoalStatusArray, f'/{action}/_action/status',
                lambda msg, a=action: self.on_action_status(a, msg), latched)

        self.period = 1.0
        self.create_timer(self.period, self.report)
        self.get_logger().info('nav_monitor started — one status line per second, WARNs name the broken link')

    def on_diagnostics(self, msg):
        for status in msg.status:
            if 'Arduino' in status.name:
                self.base_status = status

    def on_action_status(self, action, msg):
        if msg.status_list:
            self.action_status[action] = msg.status_list[-1].status

    def tf_age(self, target, source):
        # Age in seconds of the latest target<-source transform, or None if
        # the chain has never been available.
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            stamp = Time.from_msg(tf.header.stamp)
            return max(0.0, (self.get_clock().now() - stamp).nanoseconds * 1e-9)
        except Exception:
            return None

    @staticmethod
    def fmt_twist(probe):
        if probe.last_msg is None:
            return 'never'
        m = probe.last_msg
        txt = f'{m.linear.x:+.2f}/{m.angular.z:+.2f}'
        # Flag values that are just the memory of a stopped stream, so a
        # frozen last command is not mistaken for an active one.
        age = probe.age()
        if age > 1.0:
            txt += f' ({age:.0f}s old)'
        return txt

    @staticmethod
    def moving(probe):
        m = probe.last_msg
        return m is not None and (abs(m.linear.x) > 0.005 or abs(m.angular.z) > 0.01)

    def report(self):
        problems = []
        scan_rate = self.scan.rate(self.period)
        odom_rate = self.odom.rate(self.period)
        self.nav_cmd.rate(self.period)
        self.mux_out.rate(self.period)

        map_age = self.tf_age('map', 'base_footprint')
        odom_tf_age = self.tf_age('odom', 'base_footprint')

        # --- sensor / robot-side links -------------------------------------
        if self.scan.age() > 1.5:
            problems.append('/scan missing or stale -> LiDAR driver down; costmaps and mux stop-gate are blind')
        if self.odom.age() > 1.5:
            problems.append('/odom missing or stale -> serial bridge or Arduino link down; Nav2 cannot track motion')
        if odom_tf_age is None:
            problems.append('TF odom->base_footprint never seen -> serial bridge not publishing its TF')
        # --- localization ---------------------------------------------------
        if map_age is None:
            problems.append('TF map->base_footprint never seen -> AMCL not localized: set the initial pose (RViz "2D Pose Estimate")')
        elif map_age > 3.0:
            problems.append(f'TF map->base_footprint stale ({map_age:.1f} s) -> AMCL stopped updating (check /scan and particle cloud)')
        # --- command chain ---------------------------------------------------
        if self.moving(self.nav_cmd) and self.nav_cmd.age() < 0.5:
            if self.mux_out.age() > 0.5:
                problems.append('Nav2 is commanding motion but /cmd_vel_out is silent -> cmd_mux not running')
            elif not self.moving(self.mux_out):
                problems.append('Nav2 is commanding motion but mux outputs zero -> mux front-stop gate or a higher-priority stale source (see mux log)')
        if self.base_status is not None and self.base_status.level != DiagnosticStatus.OK:
            problems.append(f'base controller: {self.base_status.message}')
        # --- direction check: commanded vs measured motion ------------------
        # If the base consistently moves opposite to the command, the reversal
        # is in the command chain (bridge/firmware). If commands and odometry
        # agree but the robot still heads the wrong way on the map, the
        # reversal is in perception (laser mounting / localization) instead.
        cmd = self.mux_out.last_msg
        meas = None if self.odom.last_msg is None else self.odom.last_msg.twist.twist
        if (cmd is not None and meas is not None
                and self.mux_out.age() < 0.5 and self.odom.age() < 0.5):
            if abs(cmd.linear.x) > 0.05 and abs(meas.linear.x) > 0.02 \
                    and cmd.linear.x * meas.linear.x < 0:
                problems.append(
                    f'REVERSED LINEAR: commanded {cmd.linear.x:+.2f} m/s but odom measures '
                    f'{meas.linear.x:+.2f} m/s -> command chain (bridge/firmware) inverts forward')
            if abs(cmd.angular.z) > 0.2 and abs(meas.angular.z) > 0.05 \
                    and cmd.angular.z * meas.angular.z < 0:
                problems.append(
                    f'REVERSED ANGULAR: commanded {cmd.angular.z:+.2f} rad/s but odom measures '
                    f'{meas.angular.z:+.2f} rad/s -> rotation sign inverted (LEFT_SIDE vs IMU_YAW_SIGN)')

        # Recovery behaviors executing = the goal is failing; name the motion
        # so reversing/spinning is not mistaken for a sign bug.
        for recovery in ('spin', 'backup', 'drive_on_heading', 'wait'):
            if self.action_status.get(recovery) == GoalStatus.STATUS_EXECUTING:
                problems.append(
                    f'{recovery.upper()} RECOVERY is driving the robot — the navigation goal is '
                    'failing and Nav2 is trying to recover (this motion is intentional)')

        map_txt = 'MISSING' if map_age is None else f'{map_age:.1f}s'
        odom_tf_txt = 'MISSING' if odom_tf_age is None else 'OK'
        amcl_txt = 'never' if self.amcl.last_msg is None else f'{self.amcl.age():.0f}s ago'
        base_txt = 'unknown' if self.base_status is None else self.base_status.message
        goal_txt = ' | '.join(
            f'{a}:{STATUS_NAMES.get(s, s)}' for a, s in sorted(self.action_status.items())
            if a in ('navigate_to_pose', 'follow_path')) or 'goal: none yet'

        self.get_logger().info(
            f'scan {scan_rate:.0f}Hz | odom {odom_rate:.0f}Hz | TF odom->base {odom_tf_txt} | '
            f'TF map->base {map_txt} | amcl_pose {amcl_txt} | '
            f'nav_cmd {self.fmt_twist(self.nav_cmd)} | mux_out {self.fmt_twist(self.mux_out)} | '
            f'{goal_txt} | base: {base_txt}')
        for p in problems:
            self.get_logger().warning(p)


def main(args=None):
    rclpy.init(args=args)
    node = NavMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy._rclpy_pybind11.RCLError:
        # Benign race: SIGINT can invalidate the context while spin() is
        # rebuilding its wait set.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
