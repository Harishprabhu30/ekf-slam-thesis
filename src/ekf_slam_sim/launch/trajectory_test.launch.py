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
                # Time sync - this works with node's declaration
                "use_sim_time": True,
                
                # Topics
                "cmd_vel_topic": "/cmd_vel",
                
                # Profile selection
                "profile": "square",           # straight10 | spin2x | square
                
                # Motion parameters
                "v_straight": 0.5,              # m/s
                "w_turn": 0.5,                   # rad/s
                
                # Geometry
                "straight_distance_m": 10.0,     # for straight10 profile
                "square_side_m": 2.0,             # for square profile
                "num_full_turns": 1,               # for spin2x profile
                
                # REMOVED all caster settle parameters - no longer needed
                # "enable_caster_settle": False,   # DELETED
                # "settle_stop_s": 0.8,            # DELETED
                # "settle_creep_s": 0.5,            # DELETED
                # "settle_creep_v": 0.05,           # DELETED
                
                # Timing
                "rate_hz": 60.0,
                "start_delay_s": 1.0,
                "end_zero_hold_s": 1.5,
            }],
        )
    ])
