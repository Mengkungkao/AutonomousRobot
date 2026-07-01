#!/usr/bin/env python3
"""Priority velocity mux plus independent front-LiDAR stop gate."""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class CmdMux(Node):
    def __init__(self):
        super().__init__('mdetect_cmd_mux')
        self.declare_parameter('front_stop_distance',0.30); self.declare_parameter('front_half_angle_deg',45.0); self.declare_parameter('source_timeout',0.4)
        self.stop=float(self.get_parameter('front_stop_distance').value); self.half=math.radians(float(self.get_parameter('front_half_angle_deg').value)); self.timeout=float(self.get_parameter('source_timeout').value)
        self.nav=(Twist(),0.); self.tele=(Twist(),0.); self.manual=(Twist(),0.); self.blocked=False
        self.pub=self.create_publisher(Twist,'/cmd_vel',20)
        self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.setsrc('nav',m),20); self.create_subscription(Twist,'/cmd_vel_teleop',lambda m:self.setsrc('tele',m),20); self.create_subscription(Twist,'/cmd_vel_manual',lambda m:self.setsrc('manual',m),20); self.create_subscription(LaserScan,'/scan',self.scan,10)
        self.create_timer(.05,self.tick)
    def setsrc(self,n,m): setattr(self,n,(m,time.monotonic()))
    def scan(self,m):
        nearest=float('inf')
        for i,r in enumerate(m.ranges):
            a=m.angle_min+i*m.angle_increment
            if abs(a)<=self.half and math.isfinite(r): nearest=min(nearest,r)
        self.blocked=nearest<self.stop
    def tick(self):
        now=time.monotonic(); out=Twist()
        for name in ('tele','manual','nav'):
            msg,t=getattr(self,name)
            if now-t<=self.timeout: out=msg; break
        if self.blocked and out.linear.x>0: out=Twist()
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args); n=CmdMux()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node(); rclpy.shutdown()
