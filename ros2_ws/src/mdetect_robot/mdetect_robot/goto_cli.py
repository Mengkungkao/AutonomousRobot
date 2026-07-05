#!/usr/bin/env python3
"""Desktop client that sends the robot to one target pose via Nav2.

Targets use the same units as the Arduino base: x_mm, y_mm, heading in
degrees (CCW-positive, 0 deg = map +X). Example: 1000,1000,90 drives to
(1.0 m, 1.0 m) facing +Y in the map frame.

Run while desktop_slam.launch.py or desktop_navigation.launch.py is up:

    ros2 run mdetect_robot goto_cli               # interactive prompt
    ros2 run mdetect_robot goto_cli 1000 1000 90  # one-shot target
"""
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def parse_target(text: str) -> list[float]:
    # Accept "1000,1000,90", "(1000, 1000, 90)" or "1000 1000 90"; heading
    # defaults to 0 deg when omitted.
    parts = [p for p in text.strip().strip('()').replace(',', ' ').split() if p]
    if len(parts) not in (2, 3):
        raise ValueError('expected x_mm,y_mm[,heading_deg] e.g. 1000,1000,90')
    try:
        values = [float(p) for p in parts]
    except ValueError:
        raise ValueError(f'not numbers: {text.strip()!r}') from None
    if len(values) == 2:
        values.append(0.0)
    return values


class GotoCLI(Node):
    """Sends NavigateToPose goals built from (x_mm, y_mm, heading_deg) input
    and blocks until Nav2 finishes, printing distance-remaining feedback."""

    def __init__(self) -> None:
        super().__init__('mdetect_goto_cli')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.last_feedback_print = 0.0

    def pose_from_mm_deg(self, x_mm: float, y_mm: float, heading_deg: float) -> PoseStamped:
        # Map-frame pose in metres; heading encoded as a pure-Z quaternion.
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x_mm / 1000.0
        p.pose.position.y = y_mm / 1000.0
        r = math.radians(heading_deg)
        p.pose.orientation.z = math.sin(r / 2.0)
        p.pose.orientation.w = math.cos(r / 2.0)
        return p

    def feedback(self, msg) -> None:
        # Throttle Nav2's ~10 Hz feedback stream to one line per second.
        now = time.monotonic()
        if now - self.last_feedback_print >= 1.0:
            self.last_feedback_print = now
            print(f'  {msg.feedback.distance_remaining:.2f} m remaining')

    def navigate(self, x_mm: float, y_mm: float, heading_deg: float) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_from_mm_deg(x_mm, y_mm, heading_deg)
        print(f'Target ({x_mm:.0f}, {y_mm:.0f}) mm @ {heading_deg:.0f} deg '
              f'-> map ({goal.pose.pose.position.x:.3f}, {goal.pose.pose.position.y:.3f}) m')

        send = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('Goal rejected by Nav2')
            return False

        result_future = handle.get_result_async()
        self.last_feedback_print = time.monotonic()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            # Ctrl+C during a drive cancels this goal but keeps the CLI alive.
            print('\nCancelling goal...')
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel)
            print('Goal cancelled')
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print('Target reached')
            return True
        name = {
            GoalStatus.STATUS_ABORTED: 'aborted (no valid path or stuck?)',
            GoalStatus.STATUS_CANCELED: 'cancelled',
        }.get(status, f'ended with status {status}')
        print(f'Navigation {name}')
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GotoCLI()
    argv = remove_ros_args(sys.argv)[1:]
    try:
        print('Waiting for Nav2 NavigateToPose action...')
        node.client.wait_for_server()
        if argv:
            # One-shot mode: target given on the command line.
            node.navigate(*parse_target(' '.join(argv)))
            return
        # Interactive mode: keep accepting targets until quit.
        print("Enter target as x_mm,y_mm,heading_deg (e.g. 1000,1000,90).")
        print("Ctrl+C during a drive cancels it; 'q' quits.")
        while True:
            try:
                line = input('goto> ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            if line.lower() in ('q', 'quit', 'exit'):
                break
            try:
                target = parse_target(line)
            except ValueError as exc:
                print(f'  Invalid target: {exc}')
                continue
            node.navigate(*target)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
