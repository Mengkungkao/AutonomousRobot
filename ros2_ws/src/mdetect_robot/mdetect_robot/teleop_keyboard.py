# Keyboard teleoperation node (adapted from the standard ROS
# teleop_twist_keyboard): reads single keypresses from the terminal and
# publishes Twist (or TwistStamped) velocity commands on 'cmd_vel'.
import sys
import threading

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy

# Raw single-key input needs platform-specific terminal handling:
# msvcrt on Windows, termios/tty raw mode on POSIX.
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
        w
   a         d
        s

w : forward
s : backward
a : rotate left
d : rotate right

anything else : stop

,/. : increase/decrease max speeds by 10%
-/+ : increase/decrease only linear speed by 10%
[/] : increase/decrease only angular speed by 10%

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

# Key -> (linear factor, angular factor) applied multiplicatively to the
# current speed settings.
speedBindings = {
    ',': (1.1, 1.1),
    '.': (.9, .9),
    '-': (.9, 1),
    '+': (1.1, 1),
    '[': (1, .9),
    ']': (1, 1.1),
}


def getKey(settings):
    # Block until one key is pressed and return it without requiring Enter.
    # On POSIX the terminal is put in raw mode just for the read, then restored.
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
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
    # header when stamped, 'speed'/'turn' are the initial linear (m/s) and
    # angular (rad/s) magnitudes.
    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter('stamped', False, read_only_descriptor).value
    frame_id = node.declare_parameter('frame_id', '', read_only_descriptor).value
    speed = node.declare_parameter('speed', 0.5, read_only_descriptor).value
    turn = node.declare_parameter('turn', 1.0, read_only_descriptor).value

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

    # Current direction multipliers (set by keypresses) and a counter used to
    # re-print the help text every 15 speed changes.
    x = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

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
        # Main loop: one keypress -> one published command.
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                # Movement key: set direction multipliers.
                x = moveBindings[key][0]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                # Speed key: scale the current magnitudes and show them.
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Any other key stops the robot; Ctrl-C (\x03) exits.
                x = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = 0.0
            twist.linear.z = z * speed
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