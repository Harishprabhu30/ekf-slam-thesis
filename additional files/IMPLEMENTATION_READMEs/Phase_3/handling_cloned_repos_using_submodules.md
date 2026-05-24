# Documentation: How we handled cloned repos using submodules

## Project structure

You have:
ros2_ws/
└─ src/
   ├─ orb_slam3_ros2        # Your main ROS2 package
   ├─ ORB_SLAM3             # Submodule for the ORB_SLAM3 library
   └─ Pangolin              # Submodule for Pangolin visualization library

## Step 1: Adding a cloned repository as a submodule

Suppose you want to add Pangolin as a submodule in orb_slam3_ros2 workspace:
```bash
cd ~/Downloads/Harish_Thesis/ros2_ws
git submodule add https://github.com/stevenlovegrove/Pangolin.git src/Pangolin
git commit -m "Add Pangolin as a git submodule"
git push origin main
```

This records the submodule URL and commit in .gitmodules.

If you also need ORB_SLAM3:

```bash
git submodule add https://github.com/ORB-SLAM3/ORB_SLAM3.git src/ORB_SLAM3
git commit -m "Add ORB_SLAM3 as a submodule"
git push origin main
```

# Step 2: Cloning your main repository with submodules

If someone clones your repo:

```bash
git clone https://github.com/Harishprabhu30/ekf-slam-thesis.git ros2_ws
cd ros2_ws
git submodule update --init --recursive
```

This will automatically clone Pangolin, ORB_SLAM3, and any nested submodules (pybind11, vcpkg) at the exact commit tracked by your main repository.

# Step 3: Updating submodules after a merge

Suppose you merged `orb-slam-stereo` branch into `main` and the merge changed submodule commits.

After switching to `main`:

```bash
git checkout main
git pull origin main
git submodule update --init --recursive
```

- This ensures all submodules are at the commits your merged branch expects.

- No need to manually cd into each submodule unless you want the latest upstream commits.

# Step 4: Pulling new upstream changes for submodules

If you want the latest upstream commits for a submodule:

```bash
cd src/Pangolin
git fetch origin
git checkout main        # or specific branch
git pull origin main
cd ~/Downloads/Harish_Thesis/ros2_ws
git add src/Pangolin
git commit -m "Update Pangolin to latest"
git push origin main
```

After this, everyone who does:

```bash
git pull origin main
git submodule update --init --recursive
```

will get the updated submodule commit.
