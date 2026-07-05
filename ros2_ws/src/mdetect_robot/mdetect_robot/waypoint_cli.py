#!/usr/bin/env python3
"""Interactive desktop client for predefined Nav2 waypoint routes."""
import math, os, yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from ament_index_python.packages import get_package_share_directory

class WaypointCLI(Node):
    """Lists routes from the package's waypoints.yaml, lets the user pick one,
    and sends it to Nav2's FollowWaypoints action, blocking until it finishes."""

    def __init__(self):
        super().__init__('mdetect_waypoint_cli'); self.client=ActionClient(self,FollowWaypoints,'/follow_waypoints')

    def pose(self,x,y,yaw):
        # Build a map-frame PoseStamped from (x, y, yaw-in-degrees). Yaw is
        # encoded as a pure-Z quaternion: z = sin(yaw/2), w = cos(yaw/2).
        p=PoseStamped(); p.header.frame_id='map'; p.header.stamp=self.get_clock().now().to_msg(); p.pose.position.x=float(x); p.pose.position.y=float(y); r=math.radians(float(yaw)); p.pose.orientation.z=math.sin(r/2); p.pose.orientation.w=math.cos(r/2); return p

    def run(self):
        # Load the route definitions installed with the package. Each route is
        # a named list of [x, y, yaw] entries.
        path=os.path.join(get_package_share_directory('mdetect_robot'),'config','waypoints.yaml'); data=yaml.safe_load(open(path))['routes']; names=list(data)
        # Show a numbered menu and convert the chosen route into poses.
        print('\nPredefined routes:'); [print(f'  {i+1}. {n}') for i,n in enumerate(names)]
        choice=input('Select route number: ').strip(); name=names[int(choice)-1]; poses=[self.pose(*v) for v in data[name]]
        # Send the goal and wait for Nav2 to accept it.
        print('Waiting for Nav2 FollowWaypoints action...'); self.client.wait_for_server(); goal=FollowWaypoints.Goal(); goal.poses=poses
        future=self.client.send_goal_async(goal); rclpy.spin_until_future_complete(self,future); handle=future.result()
        if not handle.accepted: print('Route rejected'); return
        # Block until the whole route has been driven.
        print(f'Route {name} accepted with {len(poses)} waypoints'); result=handle.get_result_async(); rclpy.spin_until_future_complete(self,result); print('Route finished')

def main(args=None):
    # One-shot CLI: run the interactive flow once, then shut down cleanly.
    rclpy.init(args=args); n=WaypointCLI()
    try:n.run()
    finally:n.destroy_node(); rclpy.shutdown()
