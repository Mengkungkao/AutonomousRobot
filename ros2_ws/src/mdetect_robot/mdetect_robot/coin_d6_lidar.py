#!/usr/bin/env python3
"""COIN-D6 serial driver publishing sensor_msgs/LaserScan."""
from __future__ import annotations
import math, time
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

START_CMD = bytes([0xAA,0x55,0xF0,0x0F])
STOP_CMD = bytes([0xAA,0x55,0xF5,0x0A])
HEADER = bytes([0xAA,0x55])

def u16(lo,hi): return lo | (hi<<8)
def angle_raw(v): return ((v>>1)/64.0)%360.0

class CoinD6(Node):
    def __init__(self):
        super().__init__('coin_d6_lidar')
        for n,v in [('port','/dev/ttyUSB0'),('baud',230400),('frame_id','laser'),('range_min',0.05),('range_max',8.0),('angle_increment_deg',1.0),('distance_scale',1.0),('distance_offset_m',0.0),('publish_rate_hz',10.0)]: self.declare_parameter(n,v)
        self.port=str(self.get_parameter('port').value); self.baud=int(self.get_parameter('baud').value)
        self.frame=str(self.get_parameter('frame_id').value); self.rmin=float(self.get_parameter('range_min').value); self.rmax=float(self.get_parameter('range_max').value)
        self.inc=math.radians(float(self.get_parameter('angle_increment_deg').value)); self.scale=float(self.get_parameter('distance_scale').value); self.offset=float(self.get_parameter('distance_offset_m').value)
        self.pub=self.create_publisher(LaserScan,'/scan',10); self.ser:Optional[serial.Serial]=None; self.buf=bytearray(); self.last_try=0.; self.points={}
        self.create_timer(0.005,self.read_timer); self.create_timer(1.0/max(1.0,float(self.get_parameter('publish_rate_hz').value)),self.publish)
    def connect(self):
        if self.ser and self.ser.is_open:return True
        if time.monotonic()-self.last_try<2:return False
        self.last_try=time.monotonic()
        try:
            self.ser=serial.Serial(self.port,self.baud,timeout=0); time.sleep(.2); self.ser.reset_input_buffer(); self.ser.write(START_CMD); self.ser.flush(); self.get_logger().info(f'COIN-D6 connected: {self.port}'); return True
        except Exception as e: self.ser=None; self.get_logger().warning(str(e)); return False
    def packet(self):
        while True:
            i=self.buf.find(HEADER)
            if i<0:
                if len(self.buf)>1: del self.buf[:-1]
                return None
            if i: del self.buf[:i]
            if len(self.buf)<10:return None
            n=self.buf[3]
            if n<1 or n>160: del self.buf[0]; continue
            size=10+3*n
            if len(self.buf)<size:return None
            p=bytes(self.buf[:size]); del self.buf[:size]
            if not(p[4]&1) or not(p[6]&1):continue
            return p
    def parse(self,p):
        n=p[3]; a0=angle_raw(u16(p[4],p[5])); a1=angle_raw(u16(p[6],p[7]));
        if a1<a0:a1+=360
        step=0 if n<=1 else (a1-a0)/(n-1)
        now=time.monotonic()
        for i in range(n):
            o=10+3*i; dist_mm=p[o+2]*64+(p[o+1]>>2); intensity=((p[o+1]&3)*64)+(p[o]>>2)
            d=dist_mm*.001*self.scale+self.offset
            if self.rmin<=d<=self.rmax:
                dev=(a0+i*step)%360; ros=math.atan2(math.sin(math.radians(-dev)),math.cos(math.radians(-dev)))
                self.points[int(round((ros+math.pi)/self.inc))]=(d,float(intensity),now)
    def read_timer(self):
        if not self.connect():return
        try:
            data=self.ser.read(4096)
            if data:self.buf.extend(data)
            while True:
                p=self.packet()
                if p is None:break
                self.parse(p)
        except Exception as e:
            self.get_logger().error(str(e));
            try:self.ser.close()
            except:pass
            self.ser=None
    def publish(self):
        bins=int(round(2*math.pi/self.inc))+1; msg=LaserScan(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id=self.frame
        msg.angle_min=-math.pi; msg.angle_max=math.pi; msg.angle_increment=self.inc; msg.scan_time=0.1; msg.time_increment=0.; msg.range_min=self.rmin; msg.range_max=self.rmax
        msg.ranges=[float('inf')]*bins; msg.intensities=[0.0]*bins; now=time.monotonic()
        for i,(d,q,t) in list(self.points.items()):
            if now-t<0.5 and 0<=i<bins: msg.ranges[i]=d; msg.intensities[i]=q
        self.pub.publish(msg)
    def destroy_node(self):
        if self.ser:
            try:self.ser.write(STOP_CMD); self.ser.close()
            except:pass
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args); n=CoinD6()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node(); rclpy.shutdown()
