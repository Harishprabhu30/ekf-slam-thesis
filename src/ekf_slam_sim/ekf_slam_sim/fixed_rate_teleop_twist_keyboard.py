#!/usr/bin/env python3

import sys
import threading
import rclpy
import geometry_msgs.msg
import rcl_interfaces.msg
import time

import termios
import tty
import select

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. Works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

# --- Terminal handling ---
def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def getKey(timeout):
    dr, dw, de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def vels(speed, turn):
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f} "

# --- Main ---
def main():

    settings = saveTerminalSettings()
    tty.setcbreak(sys.stdin.fileno())

    rclpy.init()
    node = rclpy.create_node('fixed_rate_teleop_twist_keyboard')

    read_only = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter('stamped', False, read_only).value
    frame_id = node.declare_parameter('frame_id', '', read_only).value
    speed = node.declare_parameter('speed', 0.5, read_only).value
    turn = node.declare_parameter('turn', 1.0, read_only).value

    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    TwistMsg = geometry_msgs.msg.TwistStamped if stamped else geometry_msgs.msg.Twist
    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    # -------------------------
    # LOGIC CHANGE: shared state
    # -------------------------
    state = {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'th': 0.0,
        'last_key_time': 0.0
    }

    rate_hz = 30.0
    timeout = 0.5

    print(msg)
    print(vels(speed, turn))

    # -------------------------
    # LOGIC CHANGE: Timer publishing
    # -------------------------
    def timer_callback():

        curr_time = node.get_clock().now().nanoseconds / 1e9

        twist_msg = TwistMsg()
        twist = twist_msg.twist if stamped else twist_msg

        # timeout check using ROS clock
        if curr_time - state['last_key_time'] > timeout:
            state['x'] = state['y'] = state['z'] = state['th'] = 0.0

        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()
            twist_msg.header.frame_id = frame_id

        twist.linear.x = state['x'] * speed
        twist.linear.y = state['y'] * speed
        twist.linear.z = state['z'] * speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = state['th'] * turn

        pub.publish(twist_msg)

    # ROS timer replaces manual publish loop
    timer = node.create_timer(1.0 / rate_hz, timer_callback)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    try:
        while rclpy.ok():

            key = getKey(0.1)

            if key:

                # update last key time using ROS clock
                state['last_key_time'] = node.get_clock().now().nanoseconds / 1e9

                if key in moveBindings:

                    state['x'], state['y'], state['z'], state['th'] = moveBindings[key]

                elif key in speedBindings:

                    speed *= speedBindings[key][0]
                    turn *= speedBindings[key][1]

                    print(vels(speed, turn))

                elif key == '\x03':
                    break

                else:
                    state['x'] = state['y'] = state['z'] = state['th'] = 0.0

    finally:

        restoreTerminalSettings(settings)

        node.destroy_node()
        rclpy.shutdown()
        spinner.join()


if __name__ == '__main__':
    main()
