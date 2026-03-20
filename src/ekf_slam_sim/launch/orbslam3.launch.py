import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    thesis_pkg = get_package_share_directory("ekf_slam_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")

    odom_params = os.path.join(thesis_pkg, "config", "odom_params.yaml")
    settings_file = os.path.join(thesis_pkg, "config", "orbslam3_stereo.yaml")

    vocab_file = os.path.expanduser(
        "/home/vgtu/Downloads/Harish_Thesis/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    )

    # ========================= ADDED: IMU fixer kept same as wheel-only mode =========================
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
            "use_sim_time": use_sim_time,
        }],
    )
    # ================================================================================================

    # ========================= ADDED: odom node publishes odom -> chassis_link TF =========================
    encoder_odom = Node(
        package="ekf_slam_sim",
        executable="encoder_odom_publisher",
        name="encoder_odom_publisher",
        output="screen",
        parameters=[
            odom_params,
            {
                "publish_tf": True,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    # =====================================================================================================

    # CHANGED: ORB node now launched together with odom TF source for live RViz map visualization
    orb_node = Node(
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
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        imu_cov_fixer,
        encoder_odom,
        orb_node,
    ])
