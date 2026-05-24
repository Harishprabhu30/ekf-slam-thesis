# Hybrid SLAM Baseline Setup (Isaac Sim 5.0 + ROS 2)
## In this phase 1.1: /joint_states to pub the joint velocity values through ROS2 Omnigraph was created and prepared tf tree for odometry flow by separating /world (parent) from chassis_link (child) -> chassis_link (parent) and child (sensors). 
---

## Current Status (LOCKED)

### 1. Sensors Enabled (Minimal Set)

The robot is pruned to only essential sensors:

- Monocular RGB camera  
- 2D LiDAR  
- Chassis IMU  
- Wheel encoders (via joint states)

Disabled:

- Camera IMU  
- Redundant cameras  
- Simulator pose/state outputs  

---

### 2. ROS 2 Topics Publishing

Verified active topics:

- `/clock`
- `/imu`
- `/laser_scan`
- `/rgb`
- `/camera_info`
- `/joint_states`
- `/tf`

Planned later:

- `/odom` (wheel encoder odometry)
- EKF fused outputs

---

### 3. Joint States (Wheel Encoders)

Wheel encoder data is obtained **only** from joint angles under physics simulation.

Verified `/joint_states` example:

```yaml
name:
- joint_wheel_left
- joint_wheel_right
position:
- -0.0207
- -0.0207

**Key point:**

- Keyboard teleoperation → DifferentialController → wheel joints
- Reading joint angles ≡ real encoder feedback
- No base pose or world state is accessed

---

## 4. TF Tree — Final Baseline State (Before Odometry)

Current TF tree (validated with `tf2_tools view_frames`):

```text
chassis_link
 ├── camera_link
 ├── imu_link
 └── lidar_link

**Important decisions:**

- World → chassis_link transform is **DISABLED**
- Only sensor mount transforms are published
- All sensor transforms are published on `/tf` (not `/tf_static`) due to QoS incompatibility in Isaac Sim

This ensures:

- No ground-truth base pose leakage
- Clean insertion point for wheel odometry

---

## Problems Faced & Resolutions

### 1. TF Conflict: Dual Parent Problem

**Problem:**

- Isaac Sim was publishing `World → chassis_link`
- Planned odometry would publish `odom → chassis_link`
- This caused a TF tree violation (two parents for base frame)

**Resolution:**

- Disabled the OmniGraph TF publisher responsible for `World → chassis_link`
- Verified removal using:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo World chassis_link

### 2. `/tf_static` QoS Incompatibility

**Problem:**

- Isaac publishes `/tf_static` with `VOLATILE` durability
- ROS tools expect `TRANSIENT_LOCAL`
- Result: dropped or invisible static transforms

**Resolution:**

- Sensor mount transforms published dynamically on `/tf`
- Transforms are constant but sent periodically
- Documented clearly for thesis transparency

---

### 3. Avoiding Ground-Truth Leakage

**Risk Identified:**

- Isaac Sim can expose perfect pose via TF or simulator APIs

**Mitigation:**

- No simulator pose topics used
- No `World → chassis_link` transform
- Odometry will be computed strictly from wheel joint deltas

---

## What This Enables Next

With this baseline locked, we can safely proceed to:

- Wheel-encoder odometry (`/odom`)
- TF insertion: `odom → chassis_link`
- EKF baseline (wheel + IMU fusion)
- Later: SLAM, loop closure, and AI conditioning

This ensures clean experimental separation and reproducibility.

---

## Reproducibility Notes

- All TF states validated using `tf2_tools view_frames`
- Joint states verified via `/joint_states`
- Configuration snapshots saved for thesis evidence


