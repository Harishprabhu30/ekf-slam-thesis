import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    thesis_pkg = get_package_share_directory("ekf_slam_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")

    cuvslam_params = os.path.join(thesis_pkg, "config", "cuvslam_params.yaml")

    # ========================= ADDED: optical child frames for thesis-safe visual SLAM =========================
    camera_left_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_left_optical_tf_pub",
        output="screen",
        arguments=[
            "0", "0", "0",
            "-1.57079632679", "0", "-1.57079632679",
            "camera_left", "camera_left_optical"
        ],
    )

    camera_right_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_right_optical_tf_pub",
        output="screen",
        arguments=[
            "0", "0", "0",
            "-1.57079632679", "0", "-1.57079632679",
            "camera_right", "camera_right_optical"
        ],
    )
    # ==========================================================================================================

    # ========================= cuVSLAM as standalone estimator =========================
    cuvslam_node = ComposableNode(
        name="visual_slam_node",
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        parameters=[
            cuvslam_params,
            {
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("visual_slam/image_0", "/camera/left/image_raw"),
            ("visual_slam/image_1", "/camera/right/image_raw"),
            ("visual_slam/camera_info_0", "/camera/left/camera_info"),
            ("visual_slam/camera_info_1", "/camera/right/camera_info"),
        ],
    )

    cuvslam_container = ComposableNodeContainer(
        name="visual_slam_launch_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[cuvslam_node],
        output="screen",
    )
    # ==================================================================================

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        camera_left_optical_tf,
        camera_right_optical_tf,
        cuvslam_container,
    ])
