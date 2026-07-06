#!/usr/bin/env python3
"""Priority velocity mux plus independent front-LiDAR stop gate.

Nav2's navigation_launch.py hardcodes its velocity_smoother output remap as
('cmd_vel_smoothed', 'cmd_vel'). That remap uses relative topic names, so it
always resolves to the plain '/cmd_vel' topic and cannot be redirected from an
outer launch file's SetRemap (a SetRemap only overrides rules that share its
'from' key; it has no effect on this unrelated 'cmd_vel_smoothed' rule). So
Nav2's real, final output topic is always '/cmd_vel' - never '/cmd_vel_nav'.
This node therefore reads Nav2 output on '/cmd_vel' directly and republishes
the arbitrated result on '/cmd_vel_out', which is what the Arduino serial
bridge subscribes to.
"""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class CmdMux(Node):
    """Arbitrates between three velocity sources (teleop > manual > Nav2) and
    overrides the winner with a zero Twist when the front LiDAR sector reports
    an obstacle closer than the stop distance."""

    def __init__(self):
        super().__init__('mdetect_cmd_mux')
        # Tunable parameters:
        #   front_stop_distance  - obstacle range (m) below which forward motion is blocked
        #   front_half_angle_deg - half-width of the forward LiDAR sector to check
        #   source_timeout       - seconds after which a velocity source is considered stale
        self.declare_parameter('front_stop_distance',0.30); self.declare_parameter('front_half_angle_deg',45.0); self.declare_parameter('source_timeout',0.4)
        self.stop=float(self.get_parameter('front_stop_distance').value); self.half=math.radians(float(self.get_parameter('front_half_angle_deg').value)); self.timeout=float(self.get_parameter('source_timeout').value)
        # Each source holds (last Twist, monotonic receive time); 0. means "never received".
        # 'blocked' is the latest verdict of the front-sector obstacle check.
        self.nav=(Twist(),0.); self.tele=(Twist(),0.); self.manual=(Twist(),0.); self.blocked=False
        # Operational state kept for logging: nearest front range, scan arrival
        # time, and the last source/blocked values so transitions log once.
        self.nearest=float('inf'); self.last_scan_t=0.; self.last_source='(none yet)'
        # Arbitrated output consumed by the Arduino serial bridge.
        self.pub=self.create_publisher(Twist,'/cmd_vel_out',20)
        # Inputs: Nav2 on /cmd_vel (see module docstring), keyboard teleop,
        # manual/scripted commands, and the LiDAR scan for the stop gate.
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.setsrc('nav',m),20); self.create_subscription(Twist,'/cmd_vel_teleop',lambda m:self.setsrc('tele',m),20); self.create_subscription(Twist,'/cmd_vel_manual',lambda m:self.setsrc('manual',m),20); self.create_subscription(LaserScan,'/scan',self.scan,10)
        # Publish the arbitrated command at 20 Hz.
        self.create_timer(.05,self.tick)

    # Record the newest message and its arrival time for source 'n'
    # ('nav'/'tele'/'manual') so tick() can judge freshness.
    def setsrc(self,n,m): setattr(self,n,(m,time.monotonic()))

    def scan(self,m):
        # Find the nearest valid return within +/- half-angle of straight ahead
        # and update the blocked flag used by tick().
        nearest=float('inf')
        for i,r in enumerate(m.ranges):
            # Beam angle for range index i (LaserScan ranges are evenly spaced).
            a=m.angle_min+i*m.angle_increment
            # Only consider beams inside the front sector; skip inf/NaN returns.
            if abs(a)<=self.half and math.isfinite(r): nearest=min(nearest,r)
        self.nearest=nearest; self.last_scan_t=time.monotonic()
        blocked=nearest<self.stop
        if blocked!=self.blocked:
            if blocked: self.get_logger().warning(f'FRONT STOP engaged: obstacle at {nearest:.2f} m < {self.stop:.2f} m — forward commands will be zeroed')
            else: self.get_logger().info(f'front stop released: nearest front range {nearest:.2f} m')
        self.blocked=blocked

    def tick(self):
        # Pick the highest-priority source that published within the timeout:
        # teleop first, then manual, then Nav2. If all are stale, output stays
        # a zero Twist, which stops the robot.
        now=time.monotonic(); out=Twist(); source='none (all stale)'
        for name in ('tele','manual','nav'):
            msg,t=getattr(self,name)
            if now-t<=self.timeout: out=msg; source=name; break
        # Log once whenever the winning source changes, including falling back
        # to zero because every source went stale mid-drive.
        if source!=self.last_source:
            self.get_logger().info(f'velocity source: {self.last_source} -> {source}')
            self.last_source=source
        # Safety gate: while an obstacle is inside the stop distance, any command
        # with forward velocity is replaced by a full stop. Commands that only
        # reverse or rotate in place (linear.x <= 0) pass through unchanged.
        if self.blocked and out.linear.x>0:
            self.get_logger().warning(
                f'front stop gate zeroing {source} command lin={out.linear.x:.2f} (nearest {self.nearest:.2f} m)',
                throttle_duration_sec=1.0)
            out=Twist()
        # The gate decides from LiDAR data; warn if that data stops flowing.
        if self.last_scan_t and now-self.last_scan_t>1.0:
            self.get_logger().warning(f'/scan stale for {now-self.last_scan_t:.1f} s — front stop gate using old data', throttle_duration_sec=2.0)
        # 1 Hz heartbeat while anything nonzero is moving through the mux.
        if abs(out.linear.x)>0.005 or abs(out.angular.z)>0.01:
            self.get_logger().info(
                f'mux: {source} lin={out.linear.x:+.2f} m/s ang={out.angular.z:+.2f} rad/s '
                f'front={self.nearest:.2f} m blocked={self.blocked}',
                throttle_duration_sec=1.0)
        self.pub.publish(out)

def main(args=None):
    # Standard rclpy lifecycle: init, spin until Ctrl-C, then clean shutdown.
    rclpy.init(args=args); n=CmdMux()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node(); rclpy.shutdown()
