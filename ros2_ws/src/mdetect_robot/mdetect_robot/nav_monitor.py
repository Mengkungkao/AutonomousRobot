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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


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

        self.period = 1.0
        self.create_timer(self.period, self.report)
        self.get_logger().info('nav_monitor started — one status line per second, WARNs name the broken link')

    def on_diagnostics(self, msg):
        for status in msg.status:
            if 'Arduino' in status.name:
                self.base_status = status

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
        return f'{m.linear.x:+.2f}/{m.angular.z:+.2f}'

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

        map_txt = 'MISSING' if map_age is None else f'{map_age:.1f}s'
        odom_tf_txt = 'MISSING' if odom_tf_age is None else 'OK'
        amcl_txt = 'never' if self.amcl.last_msg is None else f'{self.amcl.age():.0f}s ago'
        base_txt = 'unknown' if self.base_status is None else self.base_status.message

        self.get_logger().info(
            f'scan {scan_rate:.0f}Hz | odom {odom_rate:.0f}Hz | TF odom->base {odom_tf_txt} | '
            f'TF map->base {map_txt} | amcl_pose {amcl_txt} | '
            f'nav_cmd {self.fmt_twist(self.nav_cmd)} | mux_out {self.fmt_twist(self.mux_out)} | base: {base_txt}')
        for p in problems:
            self.get_logger().warning(p)


def main(args=None):
    rclpy.init(args=args)
    node = NavMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
