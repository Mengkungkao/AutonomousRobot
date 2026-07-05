# Keyboard teleoperation node (adapted from the standard ROS
# teleop_twist_keyboard): reads single keypresses from the terminal and
# publishes Twist (or TwistStamped) velocity commands on 'cmd_vel'.
#
# While a movement key is held, the current command is re-published
# continuously (at ~20 Hz). This matters because the downstream cmd_mux
# treats a source as stale after 'source_timeout' (0.4 s): keyboard
# autorepeat only starts after an initial delay (~0.5-0.7 s), so a
# publish-per-keypress teleop leaves a gap on every keypress during which
# the mux falls back to a lower-priority source (e.g. Nav2), which can
# briefly drive the robot in a different direction than the key pressed.
# Once no key has arrived for 'key_timeout' seconds, a final zero command
# is published and the node goes silent so lower-priority sources can
# take over again.
import sys
import threading
import time

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy

# Raw single-key input needs platform-specific terminal handling:
# msvcrt on Windows, termios/tty raw mode + select on POSIX.
if sys.platform == 'win32':
    import msvcrt
else:
    import select
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around (hold a key to move, release to stop):
        w
   a         d
        s

w : forward
s : backward
a : rotate left
d : rotate right

anything else : stop immediately

CTRL-C to quit
"""

# Key -> (x, y, z, yaw) direction multipliers. This differential-drive robot
# only uses x (forward/backward) and yaw (rotate left/right).
moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
}


def getKey(settings, timeout):
    # Wait up to 'timeout' seconds for one keypress and return it without
    # requiring Enter; returns '' if no key arrived in time. On POSIX the
    # terminal is put in raw mode just for the read, then restored.
    if sys.platform == 'win32':
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if msvcrt.kbhit():
                # getwch() returns a string on Windows
                return msvcrt.getwch()
            time.sleep(0.01)
        return ''
    tty.setraw(sys.stdin.fileno())
    try:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1) if ready else ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# Snapshot the terminal attributes so they can be restored on exit
# (raw mode would otherwise leave the shell in a broken state).
def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# Format the current speed settings for display.
def vels(speed, turn):
    return 'currently:\tspeed %.2f\tturn %.2f ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_keyboard')

    # Parameters: 'stamped' selects TwistStamped output, 'frame_id' fills the
    # header when stamped, 'speed'/'turn' are the linear (m/s) and angular
    # (rad/s) magnitudes, 'key_timeout' is how long after the last keypress
    # the command is considered released (must exceed the keyboard's
    # autorepeat initial delay, typically 0.5-0.7 s).
    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter('stamped', False, read_only_descriptor).value
    frame_id = node.declare_parameter('frame_id', '', read_only_descriptor).value
    speed = node.declare_parameter('speed', 0.5, read_only_descriptor).value
    turn = node.declare_parameter('turn', 1.0, read_only_descriptor).value
    key_timeout = node.declare_parameter('key_timeout', 0.75, read_only_descriptor).value

    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    # Spin in a background thread so the main thread can block on keyboard
    # input while ROS callbacks (e.g. parameter services) stay responsive.
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    # Current direction multipliers (set by keypresses). 'active' is True
    # while a command (including an explicit stop) is being streamed;
    # 'last_key_time' drives the key-release timeout.
    x = 0.0
    th = 0.0
    active = False
    last_key_time = 0.0

    # 'twist' aliases the Twist portion of the outgoing message so the loop
    # below works identically for stamped and unstamped variants.
    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        # Main loop: poll the keyboard at ~20 Hz and re-publish the current
        # command on every iteration while it is active.
        while True:
            key = getKey(settings, 0.05)
            now = time.monotonic()
            if key == '\x03':
                break
            if key in moveBindings.keys():
                # Movement key: set direction multipliers.
                x = moveBindings[key][0]
                th = moveBindings[key][3]
                active = True
                last_key_time = now
            elif key:
                # Any other key stops the robot immediately. The zero command
                # is streamed until the timeout so the mux latches the stop.
                x = 0.0
                th = 0.0
                active = True
                last_key_time = now
            elif active and now - last_key_time > key_timeout:
                # Key released: publish one final stop, then go silent so
                # lower-priority sources (e.g. Nav2) can take over.
                x = 0.0
                th = 0.0
                active = False
            elif not active:
                continue

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        # Always publish a final zero Twist so the robot stops, then restore
        # the terminal and shut ROS down.
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
