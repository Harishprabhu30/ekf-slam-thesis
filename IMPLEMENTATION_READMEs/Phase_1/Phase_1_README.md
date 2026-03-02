# Edge-Efficient Hybrid SLAM in NVIDIA Isaac Sim (ROS 2)

## Master’s Thesis Project

---

## 1. Thesis Focus and Motivation

This thesis investigates a **hybrid SLAM architecture** that combines **classical probabilistic state estimation** with **lightweight learning-based conditioning**, explicitly designed for **edge-computing constraints**.

The core philosophy is:

> **AI as a conditioning module, not a replacement for SLAM.**

Instead of end-to-end learning or vision-heavy SLAM, the system preserves:
- interpretability,
- physical consistency,
- and real-time guarantees,

while using minimal neural components to **adaptively improve robustness** under challenging conditions (illumination variation, dynamics, sensor degradation).

The system is implemented and evaluated in **NVIDIA Isaac Sim 5.0** with **ROS 2**, targeting performance comparable to **Jetson Nano–class hardware (≈15–20 FPS)**.

---

## 2. System Overview

### 2.1 Core SLAM Backbone (Classical)

- **Motion model**:  
  Differential-drive kinematics using:
  - wheel encoders  
  - chassis-mounted IMU  

- **Measurement model**:  
  - single 2D LiDAR  
  - scan matching–based correction  

- **Estimator**:  
  - EKF-based SLAM / EKF + scan matching  

This backbone provides:
- metric scale,
- pose estimation,
- map consistency,
- and full probabilistic interpretability.

---

### 2.2 Learning Modules (Planned, not yet activated)

Two **minimal AI modules** will later be introduced:

1. **Vision Conditioning Module**
   - MobileNetV2-based encoder–decoder
   - Inputs: monocular RGB
   - Outputs:
     - illumination-corrected image
     - static structure mask
   - Supervision: **LiDAR self-supervision**
   - Purpose: improve feature reliability under lighting and dynamic clutter

2. **Adaptive Noise Estimation Module**
   - Lightweight MLP
   - Inputs:
     - illumination quality
     - motion context
     - innovation residuals
     - feature confidence
   - Outputs:
     - adaptive EKF **Q/R covariance scaling**
   - Purpose: context-aware uncertainty tuning

**Important:**  
Neither module replaces pose estimation. They *condition* the EKF.

---

## 3. Robot and Simulation Setup

### 3.1 Platform

- **Robot**: Nova Carter (differential drive)
- **Simulator**: NVIDIA Isaac Sim 5.0
- **Physics**: enabled
- **Control**: OmniGraph Differential Drive controller (keyboard input)

The controller is used **only for actuation**.  
All estimation is performed via sensor feedback.

---

### 3.2 Sensor Selection (Minimal, Interpretable)

To avoid sensor redundancy and scope creep, the system intentionally uses a **minimal sensor suite**:

| Sensor | Status | Purpose |
|------|------|--------|
| Wheel encoders | Enabled | Motion prediction |
| Chassis IMU | Enabled | Orientation + dynamics |
| 2D LiDAR | Enabled | Metric correction |
| RGB camera (pinhole) | Enabled | Conditioning only |
| Stereo / depth | Disabled | Out of scope |
| 3D LiDAR | Disabled | Out of scope |
| Camera IMU | Disabled | Avoid VIO ambiguity |

---

## 4. Camera Configuration Rationale

- **Type**: Monocular RGB
- **Projection**: Pinhole
- **Usage**:  
  - *not used for pose estimation*
  - used only for perception conditioning

Reasons:
- avoids fisheye distortion complexity
- avoids stereo depth dependency
- prevents reviewers asking “why not visual SLAM?”
- clean separation between perception and state estimation

---

## 5. ROS 2 Integration Architecture

### 5.1 Published Topics

| Topic   | Message Type |
|----|----|
| `/clock`| rosgraph_msgs/Clock |
| `/laser_scan` | sensor_msgs/LaserScan |
| `/imu` | sensor_msgs/Imu |
| `/rgb` | sensor_msgs/Image |
| `/camera_info` | sensor_msgs/CameraInfo |
| `/tf` | tf2_msgs/TFMessage |
| `/tf_static` | tf2_msgs/TFMessage |

---

### 5.2 TF Frame Convention (Locked)

The following TF frames are now fixed and must not be changed:

| Frame | Description |
|----|----|
| `World` | Global simulation frame |
| `chassis_link` | Robot base frame |
| `imu_link` | IMU frame |
| `lidar_link` | 2D LiDAR frame |
| `camera_link` | RGB camera frame |

---

### 5.3 TF Publishing Strategy

To avoid TF ambiguity and QoS issues:

- **Dynamic articulation TF** (`World → chassis_link → wheels`)  
  → published on `/tf`

- **Rigid sensor extrinsics** (`chassis_link → imu/camera/lidar`)  
  → published via **ROS2 Publish Raw Transform Tree**  
  → currently on `/tf_static` (with continuous publishing)

This separation mirrors real robotic systems and keeps the EKF clean.

---

## 6. Current Status (Checkpoint)

### ✅ Completed

- Physics-enabled robot placement
- Differential drive control working
- Sensor pruning completed
- Camera configured as monocular pinhole
- ROS 2 bridge operational
- Topics publishing correctly
- IMU timestamps synchronized with simulation time
- TF tree validated:
  - dynamic robot links
  - static sensor mounts
- 2D LiDAR scan validated:
  - 360° coverage
  - stable scan rate (~10 Hz)
  - correct frame alignment

### ⚠️ Known Notes (Accepted)

- `LaserScan.time_increment = inf`  
  → acceptable unless scan matcher explicitly rejects it

- Sensor mount transforms currently identity  
  → will be replaced with real offsets later

---

## 7. Next Planned Steps

1. **LiDAR motion sanity validation**
   - confirm scan consistency during translation/rotation

2. **Odometry publishing**
   - encoder + IMU based odom
   - no controller “ground truth” leakage

3. **EKF baseline**
   - prediction: wheel + IMU
   - correction: LiDAR scan matching
   - no learning, no vision

4. **Baseline evaluation**
   - accuracy (ATE)
   - FPS
   - robustness under degraded lighting

Only after the baseline is stable will learning modules be introduced.

---

## 8. Design Principles (Examiner-Facing)

- No end-to-end learning
- No black-box SLAM
- Explicit uncertainty modeling
- Real-time first
- AI improves reliability, not replaces physics

*Last updated: TF and LiDAR pipeline stabilized*
