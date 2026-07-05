#!/usr/bin/env python3
"""COIN-D6 serial driver publishing sensor_msgs/LaserScan."""
from __future__ import annotations
import math, time
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

# COIN-D6 serial protocol constants: commands to start/stop the motor and
# measurement stream, and the 2-byte sync header that begins every data packet.
START_CMD = bytes([0xAA,0x55,0xF0,0x0F])
STOP_CMD = bytes([0xAA,0x55,0xF5,0x0A])
HEADER = bytes([0xAA,0x55])

# Combine two bytes into a little-endian unsigned 16-bit value.
def u16(lo,hi): return lo | (hi<<8)
# Decode a raw angle field to degrees: bit 0 is a validity flag, the remaining
# bits are the angle in 1/64-degree units; wrap into [0, 360).
def angle_raw(v): return ((v>>1)/64.0)%360.0

class CoinD6(Node):
    """Reads COIN-D6 measurement packets from a serial port, accumulates them
    into fixed angular bins covering a full revolution, and publishes the
    assembled 360-degree LaserScan at a fixed rate."""

    def __init__(self):
        super().__init__('coin_d6_lidar')
        # Serial/geometry parameters with defaults; distance_scale and
        # distance_offset_m allow per-unit range calibration (d = raw*scale + offset).
        for n,v in [('port','/dev/ttyUSB0'),('baud',230400),('frame_id','laser'),('range_min',0.05),('range_max',8.0),('angle_increment_deg',1.0),('distance_scale',1.0),('distance_offset_m',0.0),('publish_rate_hz',10.0)]: self.declare_parameter(n,v)
        self.port=str(self.get_parameter('port').value); self.baud=int(self.get_parameter('baud').value)
        self.frame=str(self.get_parameter('frame_id').value); self.rmin=float(self.get_parameter('range_min').value); self.rmax=float(self.get_parameter('range_max').value)
        self.inc=math.radians(float(self.get_parameter('angle_increment_deg').value)); self.scale=float(self.get_parameter('distance_scale').value); self.offset=float(self.get_parameter('distance_offset_m').value)
        # State: 'buf' collects raw serial bytes for packet framing; 'points'
        # maps angular bin index -> (distance, intensity, receive time);
        # 'last_try' rate-limits reconnect attempts.
        self.pub=self.create_publisher(LaserScan,'/scan',10); self.ser:Optional[serial.Serial]=None; self.buf=bytearray(); self.last_try=0.; self.points={}
        # Fast timer (200 Hz) drains the serial port; slower timer publishes scans.
        self.create_timer(0.005,self.read_timer); self.create_timer(1.0/max(1.0,float(self.get_parameter('publish_rate_hz').value)),self.publish)

    def connect(self):
        # Open the serial port if needed and command the sensor to start
        # streaming. Retries are throttled to one attempt every 2 seconds.
        if self.ser and self.ser.is_open:return True
        if time.monotonic()-self.last_try<2:return False
        self.last_try=time.monotonic()
        try:
            self.ser=serial.Serial(self.port,self.baud,timeout=0); time.sleep(.2); self.ser.reset_input_buffer(); self.ser.write(START_CMD); self.ser.flush(); self.get_logger().info(f'COIN-D6 connected: {self.port}'); return True
        except Exception as e: self.ser=None; self.get_logger().warning(str(e)); return False

    def packet(self):
        # Extract one complete, plausible packet from the byte buffer, or return
        # None if more data is needed. Packet layout: AA 55 header, byte 3 =
        # sample count n, bytes 4-7 = start/end angle, then 3 bytes per sample
        # starting at offset 10 (total size 10 + 3*n).
        while True:
            i=self.buf.find(HEADER)
            if i<0:
                # No header found: keep only the last byte (it could be the
                # first half of a header split across reads).
                if len(self.buf)>1: del self.buf[:-1]
                return None
            # Discard garbage bytes before the header.
            if i: del self.buf[:i]
            if len(self.buf)<10:return None
            n=self.buf[3]
            # Implausible sample count means we synced on a false header;
            # drop one byte and rescan.
            if n<1 or n>160: del self.buf[0]; continue
            size=10+3*n
            if len(self.buf)<size:return None
            p=bytes(self.buf[:size]); del self.buf[:size]
            # Both angle fields must have their validity bit set, else the
            # packet is rejected.
            if not(p[4]&1) or not(p[6]&1):continue
            return p

    def parse(self,p):
        # Decode all samples in a packet and store them into angular bins.
        # Sample angles are interpolated linearly between the packet's start
        # angle a0 and end angle a1 (unwrapped across the 0/360 boundary).
        n=p[3]; a0=angle_raw(u16(p[4],p[5])); a1=angle_raw(u16(p[6],p[7]));
        if a1<a0:a1+=360
        step=0 if n<=1 else (a1-a0)/(n-1)
        now=time.monotonic()
        for i in range(n):
            # Each 3-byte sample packs distance (14 bits, mm) and intensity
            # (8 bits) across the bytes: dist = b2*64 + b1>>2,
            # intensity = (b1 & 3)*64 + b0>>2.
            o=10+3*i; dist_mm=p[o+2]*64+(p[o+1]>>2); intensity=((p[o+1]&3)*64)+(p[o]>>2)
            # Apply calibration and convert mm -> m.
            d=dist_mm*.001*self.scale+self.offset
            if self.rmin<=d<=self.rmax:
                # Convert the device's clockwise degree angle to a ROS-convention
                # angle in [-pi, pi] (negate, then normalize via atan2), and
                # store the sample in the bin index used by publish().
                dev=(a0+i*step)%360; ros=math.atan2(math.sin(math.radians(-dev)),math.cos(math.radians(-dev)))
                self.points[int(round((ros+math.pi)/self.inc))]=(d,float(intensity),now)

    def read_timer(self):
        # Periodic serial pump: read whatever bytes are available and process
        # every complete packet in the buffer. On any I/O error, drop the
        # connection so connect() can re-establish it.
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
        # Assemble a full-circle LaserScan from the binned points. Bins with no
        # recent sample stay at +inf ("no return"); samples older than 0.5 s are
        # treated as stale and excluded.
        bins=int(round(2*math.pi/self.inc))+1; msg=LaserScan(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id=self.frame
        msg.angle_min=-math.pi; msg.angle_max=math.pi; msg.angle_increment=self.inc; msg.scan_time=0.1; msg.time_increment=0.; msg.range_min=self.rmin; msg.range_max=self.rmax
        msg.ranges=[float('inf')]*bins; msg.intensities=[0.0]*bins; now=time.monotonic()
        for i,(d,q,t) in list(self.points.items()):
            if now-t<0.5 and 0<=i<bins: msg.ranges[i]=d; msg.intensities[i]=q
        self.pub.publish(msg)

    def destroy_node(self):
        # Best effort: tell the sensor to stop spinning before closing the port.
        if self.ser:
            try:self.ser.write(STOP_CMD); self.ser.close()
            except:pass
        return super().destroy_node()

def main(args=None):
    # Standard rclpy lifecycle: init, spin until Ctrl-C, then clean shutdown.
    rclpy.init(args=args); n=CoinD6()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node(); rclpy.shutdown()
