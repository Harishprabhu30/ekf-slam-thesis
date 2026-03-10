# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#!/usr/bin/env python3
"""
Fixed-rate teleop_twist_keyboard for ROS2 (Linux)
- Publishes Twist/TwistStamped at fixed rate
- Timeout: stops robot if no key pressed for 0.5s
- Non-blocking keyboard input
"""

import sys
import time
import threading
import rclpy
import geometry_msgs.msg
import rcl_interfaces.msg

# Linux keyboard handling
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
    """Return a key if pressed within timeout seconds, else None"""
    dr, dw, de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def vels(speed, turn):
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f} "

# --- Main ---
def main():
    settings = saveTerminalSettings()
    tty.setcbreak(sys.stdin.fileno())  # Linux: set raw mode for non-blocking input

    rclpy.init()
    node = rclpy.create_node('fixed_rate_teleop_twist_keyboard')

    # parameters
    read_only = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter('stamped', False, read_only).value
    frame_id = node.declare_parameter('frame_id', '', read_only).value
    speed = node.declare_parameter('speed', 0.5, read_only).value
    turn = node.declare_parameter('turn', 1.0, read_only).value

    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    TwistMsg = geometry_msgs.msg.TwistStamped if stamped else geometry_msgs.msg.Twist
    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    # Fixed rate and timeout
    rate_hz = 20.0
    timeout = 0.5  # seconds
    last_key_time = time.time()

    x = y = z = th = 0.0
    twist_msg = TwistMsg()
    twist = twist_msg.twist if stamped else twist_msg

    print(msg)
    print(vels(speed, turn))

    try:
        while rclpy.ok():
            key = getKey(1.0 / rate_hz)  # non-blocking read
            if key:
                last_key_time = time.time()
                if key in moveBindings:
                    x, y, z, th = moveBindings[key]
                elif key in speedBindings:
                    speed *= speedBindings[key][0]
                    turn *= speedBindings[key][1]
                    print(vels(speed, turn))
                elif key == '\x03':  # CTRL-C
                    break
                else:
                    x = y = z = th = 0.0
            # Stop if timeout exceeded
            if time.time() - last_key_time > timeout:
                x = y = z = th = 0.0

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()
                twist_msg.header.frame_id = frame_id

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn

            pub.publish(twist_msg)
    finally:
        # Stop robot before exit
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0
        twist.angular.x = twist.angular.y = twist.angular.z = 0.0
        pub.publish(twist_msg)

        restoreTerminalSettings(settings)
        rclpy.shutdown()
        spinner.join()

if __name__ == '__main__':
    main()
