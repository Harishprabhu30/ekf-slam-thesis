# Create and build workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash

# Add this to your ~/.bashrc for persistence
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc