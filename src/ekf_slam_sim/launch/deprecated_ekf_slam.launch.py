import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ekf_slam_sim")
    odom_params = os.path.join(pkg_share, "config", "odom_params.yaml")

    encoder_odom = Node(
        package="ekf_slam_sim",
        executable="encoder_odom_publisher",
        name="encoder_odom_publisher",
        output="screen",
        parameters=[odom_params],
    )

    # Optional: if you want World as RViz fixed frame, publish static World->odom
    # Comment it out if you don't need it.
    world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_static",
        arguments=["0", "0", "0", "0", "0", "0", "World", "odom"],
        output="screen",
    )

    return LaunchDescription([
        encoder_odom,
        # world_to_odom,
    ])

