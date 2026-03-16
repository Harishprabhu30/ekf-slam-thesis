from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("ekf_slam_sim")
    odom_params = os.path.join(pkg_share, "config", "odom_params.yaml")
    ekf_params = os.path.join(pkg_share, "config", "ekf_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        Node(
            package="ekf_slam_sim",
            executable="imu_covariance_fixer",
            name="imu_covariance_fixer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "in_topic": "/imu_raw",
                "out_topic": "/imu",
                "gyro_cov_xy": 0.04,
                "gyro_cov_z": 0.0004,
                "acc_cov_xy": 0.1,
                "acc_cov_z": 0.1,
                "disable_orientation": True,
            }],
        ),

        Node(
            package="ekf_slam_sim",
            executable="encoder_odom_publisher",
            name="encoder_odom_publisher",
            output="screen",
            parameters=[
                odom_params,
                {
                    "use_sim_time": use_sim_time,
                    "publish_tf": False
                },
            ],
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                ekf_params,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
