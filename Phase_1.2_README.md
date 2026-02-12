# Hybrid SLAM Baseline — Isaac Sim 5.0 + ROS 2
## Development Timeline (So Far)

| Stage | Milestone | Status |
|------|----------|--------|
| 1 | Robot imported with physics + ground plane | ✅ Completed |
| 2 | Sensor pruning (single RGB, 2D LiDAR, IMU, encoders) | ✅ Completed |
| 3 | ROS 2 bridge + sensor topics verified | ✅ Completed |
| 4 | TF plumbing and frame validation | ✅ Completed |
| 5 | Joint states (wheel encoders) from physics | ✅ Completed |
| 6 | Removal of World → chassis_link (ground-truth TF) | ✅ Completed |
| 7 | Wheel-encoder odometry (`/odom`) implemented | ✅ Completed |
| 8 | Correct TF chain: `odom → chassis_link → sensors` | ✅ Completed |
| 9 | Odometry verified with TF + motion | ✅ Completed |
| 10 | Wheel parameter calibration | ⏭️ Next |

Everything above this line is **locked** before moving forward.

---

## Current System Architecture

### Enabled Sensors
- Monocular RGB camera
- 2D LiDAR
- Chassis IMU
- Wheel encoders (via joint states)

Disabled:
- Camera IMU
- Redundant cameras
- Any simulator pose / ground-truth outputs

---

## ROS 2 Topics (Verified)

Active topics:
- `/clock`
- `/joint_states`
- `/imu`
- `/laser_scan`
- `/rgb`
- `/camera_info`
- `/odom`
- `/tf`

---

## TF Tree — Final Baseline State

The current TF tree (validated with `tf2_tools view_frames`) is:

odom
└── chassis_link
    ├── camera_link
    ├── imu_link
    └── lidar_link


Key guarantees:
- **No `World → chassis_link` transform**
- No dual-parent TF conflicts
- No simulator ground-truth pose leakage
- Odometry is the **only** source of base motion

Sensor mount transforms are published dynamically on `/tf`
(due to `/tf_static` QoS incompatibility in Isaac Sim).

---

## Wheel Encoders via Joint States

Wheel encoders are obtained from physics-based joint angles:

Verified `/joint_states` example:
```yaml
name:
- joint_wheel_left
- joint_wheel_right
position:
- 2.5935
- 2.6150

**Important:**

- Keyboard teleoperation → DifferentialController → wheel joints
- Reading joint angles ≡ real encoder feedback
- No pose, no world state, no simulator odometry used

**Odometry Implementation**

- Wheel-only differential drive odometry
- **Inputs:** `/joint_states`
- **Outputs:**
  - `/odom` (`nav_msgs/Odometry`)
  - TF: `odom → chassis_link`
- IMU is not fused yet (reserved for EKF stage)

This provides a clean baseline for later comparison.

## Package Structure

ros2_ws/
└── src/ekf_slam_sim/
    ├── ekf_slam_sim/
    │   ├── __init__.py
    │   └── encoder_odom_publisher.py
    ├── launch/
    │   └── warehouse_sim.launch.py
    ├── config/
    │   └── odom_params.yaml
    ├── rviz/
    │   └── warehouse_config.rviz
    ├── setup.py
    ├── setup.cfg
    └── package.xml

## Build Instructions

From the workspace root:

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

### Verify executable is registered:
ros2 pkg executables ekf_slam_sim

Expected: encoder_odom_publisher

## Running the System
* Option 1 — Launch file (recommended)
ros2 launch ekf_slam_sim warehouse_sim.launch.py

* Option 2 — Run node directly
ros2 run ekf_slam_sim encoder_odom_publisher --ros-args --params-file src/ekf_slam_sim/config/odom_params.yaml

## Verification Checklist (Must Pass)
1. Odometry topic
ros2 topic hz /odom
ros2 topic echo /odom --once

2. TF transform exists
ros2 run tf2_ros tf2_echo odom chassis_link

Should update smoothly while driving.

3. TF tree validation
ros2 run tf2_tools view_frames

## Confirm:

odom → chassis_link → sensors (imu, lidar, camera)

## Debugging Experience & Lessons Learned
TF frames not showing odom while saving TF frames (view_frames), the odom frame initially did not appear.

### Root cause:

* The odometry node was implemented correctly

* But the package / launch file was not running

### Fix:

* Properly launched the package

* Re-ran view_frames

* odom → chassis_link appeared correctly

### Lesson:

TF frames only exist if the publishing node is actively running.

-----------------------------------------------------------------------
## Next Step (Confirmed)

### Wheel Parameter Calibration

Before EKF:

* Wheel radius calibration (straight-line test)

* Wheel separation calibration (in-place rotation test)

This ensures:

* Accurate distance estimation

* Correct yaw integration

* Reliable EKF baseline

Once calibrated:

* Rosbags will be recorded

* EKF fusion (wheel + IMU) will begin.
