# Check status (do this often)
git status

# Add specific files
git add src/ekf_slam_sim/scripts/data_logger.py
git add launch/warehouse_sim.launch.py

# Commit with descriptive message
git commit -m "feat: add synchronized data logger with rosbag2 support"

# Create branches for features
git checkout -b feature/ekf_implementation
git checkout -b experiment/sensor_fusion
git checkout main  # Return to main

# View commit history
git log --oneline --graph --all