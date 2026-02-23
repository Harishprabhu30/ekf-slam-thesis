import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ekf_slam_sim")
    odom_params = os.path.join(pkg_share, "config", "odom_params.yaml")
    ekf_params = os.path.join(pkg_share, "config", "ekf_params.yaml")

    imu_cov_fixer = Node(
        package="ekf_slam_sim",
        executable="imu_covariance_fixer",
        name="imu_covariance_fixer",
        output="screen",
        parameters=[{
            "in_topic": "/imu_raw",
            "out_topic": "/imu",
            "gyro_cov_xy": 0.04,
            "gyro_cov_z": 0.0004,
            "acc_cov_xy": 0.1,
            "acc_cov_z": 0.1,
            "disable_orientation": True,
        }],
    )

    # Wheel odom does NOT own TF in EKF mode, but still publishes /odom topic
    encoder_odom = Node(
        package="ekf_slam_sim",
        executable="encoder_odom_publisher",
        name="encoder_odom_publisher",
        output="screen",
        parameters=[
            odom_params,
            {"publish_tf": False},
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
    )

    return LaunchDescription([
        imu_cov_fixer,
        encoder_odom,
        ekf_node,
    ])
