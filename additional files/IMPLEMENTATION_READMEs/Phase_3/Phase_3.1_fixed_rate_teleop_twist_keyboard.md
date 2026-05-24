# Fixed-Rate Teleoperation Node

The standard ROS 2 `teleop_twist_keyboard` node publishes velocity commands
only when keyboard events occur. This produces sparse and irregular
`cmd_vel` streams, which can lead to inconsistent trajectory replay in
simulation environments with varying frame rates.

To ensure deterministic trajectory recording and replay, the teleoperation
node was modified to publish velocity commands at a fixed rate of **20 Hz**.

## Key changes:
- Keyboard input is read asynchronously.
- The most recent velocity command is republished at a constant rate.
- Publishing is independent of typing speed.
- Compatible with both `Twist` and `TwistStamped` message types.

This guarantees a consistent `cmd_vel` stream during trajectory recording
and replay, improving experiment reproducibility.

**Modified from ROS 2 Humble `teleop_twist_keyboard.py`**  
Original BSD license applies:  
[https://github.com/ros/teleop_twist_keyboard](https://github.com/ros/teleop_twist_keyboard)


to add in main readme:

The standard ROS 2 teleoperation node publishes velocity commands only
when keyboard events occur. This results in sparse cmd_vel streams that
may replay inconsistently when simulator timing changes.

To ensure deterministic trajectory reproduction, a modified teleoperation
node was implemented that republishes the most recent velocity command
at a fixed rate of 20 Hz. This decouples keyboard input from command
publication and produces a consistent control stream during trajectory
recording and replay.
