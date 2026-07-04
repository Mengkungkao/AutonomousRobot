import sys
import threading

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy

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

Speed is fixed to the 'speed' and 'turn' parameters.

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %.2f\tturn %.2f ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_keyboard')

    # parameters
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

    # Publish straight to the mux's dedicated teleop input. Publishing on
    # 'cmd_vel' (the old default) put keyboard commands on the same topic
    # Nav2 uses, so Nav2's own rotation commands could immediately override
    # a/d keypresses and spin the robot the wrong way.
    pub = node.create_publisher(TwistMsg, 'cmd_vel_teleop', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = 0.0
    z = 0.0
    th = 0.0

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
        while True:
            key = getKey(settings)
            if key == '\x03':
                break
            # Accept the WSAD keys regardless of Shift/Caps Lock so 'A'/'D'
            # never fall through to the stop branch.
            key = key.lower()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][3]
            else:
                x = 0.0
                th = 0.0

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