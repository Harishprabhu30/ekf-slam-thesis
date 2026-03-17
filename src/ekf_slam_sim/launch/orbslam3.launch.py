from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    thesis_pkg = get_package_share_directory("ekf_slam_sim")
    orb_pkg = get_package_share_directory("orbslam3")

    use_sim_time = LaunchConfiguration("use_sim_time")

    settings_file = os.path.join(
        thesis_pkg,
        "config",
        "orbslam3_stereo.yaml"
    )

    vocab_file = os.path.expanduser(
    "/home/vgtu/Downloads/Harish_Thesis/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        Node(
            package="orbslam3",
            executable="stereo",
            name="orbslam3_stereo",
            output="screen",

            parameters=[{
                "use_sim_time": use_sim_time
            }],

            arguments=[
                vocab_file,
                settings_file,
                "false"
            ],

            remappings=[
                ("/camera/left/image_raw", "/camera/left/image_raw"),
                ("/camera/right/image_raw", "/camera/right/image_raw")
            ]
        ),
    ])
