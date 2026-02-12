from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ekf_slam_sim",
            executable="trajectory_player",
            name="trajectory_player",
            output="screen",
            parameters=[{
                "use_sim_time": True,          # will be ignored by node unless declared; sim time is usually global
                "cmd_vel_topic": "/cmd_vel",
                "profile": "square",           # straight10 | spin2x | square
                "v_straight": 0.25,
                "w_turn": 0.6,
                "straight_distance_m": 10.0,
                "square_side_m": 2.0,
                "num_full_turns": 2,
                "enable_caster_settle": True,
                "settle_stop_s": 0.8,
                "settle_creep_s": 0.5,
                "settle_creep_v": 0.05,
                "rate_hz": 30.0,
                "start_delay_s": 1.0,
                "end_zero_hold_s": 1.5,
            }],
        )
    ])

