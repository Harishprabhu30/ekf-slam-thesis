cd ~/ros2_ws/src

# Create Python package (choose one, Python recommended for rapid prototyping)
ros2 pkg create ekf_slam_sim --build-type ament_python --dependencies rclpy sensor_msgs nav_msgs geometry_msgs tf2_ros

# OR for C++ package
ros2 pkg create ekf_slam_sim_cpp --build-type ament_cmake --dependencies rclcpp sensor_msgs nav_msgs geometry_msgs tf2

# Create directory structure
cd ekf_slam_sim
mkdir launch config scripts rviz