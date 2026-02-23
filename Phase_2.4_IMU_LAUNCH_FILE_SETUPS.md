# EKF Baseline Bring-up Log (Wheel Odom + IMU ωz) — Locked Progress Notes

## Goal

Bring up a lightweight, interpretable EKF baseline in Isaac Sim + ROS 2 under a strict scientific constraint:

- **Immutable:** robot, sensors, TF architecture, physics, world, trajectory, sim timestep  
- **Variable:** estimator stack only (wheel-only vs EKF vs later SLAM)  

No simulator ground truth TF. No world frame. No map frame.

## Locked TF Policy

### Allowed TF chain (LOCKED)

'''bash
odom → chassis_link → sensors

### Rules (LOCKED)

- No world frame usage  
- No simulator ground-truth transforms  
- Only one node may publish `odom → chassis_link` at a time  
- Sensor TF publishers are allowed only under `chassis_link`

## TF Ownership Modes (LOCKED)

| Mode               | odom → chassis_link Publisher | Wheel odom TF | Wheel odom Topic |
|-------------------|------------------------------|---------------|-----------------|
| Wheel baseline     | encoder_odom_publisher        | ON            | always ON       |
| EKF baseline       | ekf_filter_node               | OFF           | always ON       |
| SLAM mode (future) | SLAM node                     | OFF           | always ON       |

## Key Problems Encountered and How They Were Solved

### 1) TF Conflicts (Root Issue)

- **Symptom:** TF chain instability / conflicting transforms  
- **Cause:** Multiple sources publishing transforms that should be owned by one estimator  
- **Fix:** Enforce strict TF ownership using a parameter toggle in wheel odom and EKF TF publishing  

✅ **Result:** EKF mode publishes only EKF TF (`odom → chassis_link`)

### 2) Low Sensor Rates / Unstable Sim Performance

- **Symptom:** IMU and other topics dropped to ~8 Hz in heavy environments; FPS dropped ~9  
- **Fixes applied (LOCKED for baseline):**  
  - Use ground plane with physics material + friction enabled  
  - Eco rendering  
  - Reflections OFF  
  - Translucency OFF  
  - Avoid heavy warehouse scene during baseline bring-up  

✅ **Result:** Stable sensor update rates in the ~15–24 Hz range (IMU ~18 Hz stable)

### 3) IMU Covariance Invalid (Zeros)

- **Symptom:** `/imu_raw` had zero covariance → EKF can behave poorly / ignore measurement weighting  
- **Fix:** Created IMU covariance fixer node  
  - **Input:** `/imu_raw`  
  - **Output:** `/imu` (validated stream used by EKF)  
  - **Orientation explicitly disabled** (ROS convention): `orientation_covariance[0] = -1`  

✅ **Result:** EKF uses `/imu` with meaningful covariance and correct yaw-rate weighting

## Final Locked Implementation

### A) Wheel Odometry (LOCKED)

Differential drive odometry from `/joint_states`.

**Features:**

- Midpoint yaw integration  
- Wrap-safe wheel delta:  
  ```text
  dphi = atan2(sin(dphi), cos(dphi))
  
**Publishes:**

- /odom always

- TF odom → chassis_link only if publish_tf:=true

### Calibrated Geometry (LOCKED)

| Parameter        | Value       |
|-----------------|------------|
| wheel_radius     | 0.14 m     |
| wheel_separation | 0.4132 m   |

### B) IMU Validation + Fixer Node (LOCKED)

- **Validated topic:** `/imu` (not `/imu_raw`)  
- **Fusion policy:** use only `angular_velocity.z (ωz)`

**Covariance policy used:**

| Field                           | Value                        |
|---------------------------------|------------------------------|
| angular_velocity_covariance (x,y)| 0.04                         |
| angular_velocity_covariance (z)  | 0.0004                       |
| linear_acceleration_covariance   | 0.1 diagonal                 |
| orientation_covariance           | `[-1, 0, 0; ...]` (orientation disabled) |

### C) EKF Baseline (LOCKED)

**robot_localization EKF input selection:**

- **odom0:** `/odom` → fuse `x`, `y`, `vx`  
- **imu0:** `/imu` → fuse `vyaw (ωz)` only  

**Publishes:** `/odometry/filtered` and TF `odom → chassis_link`  

**Notes:**  
- No IMU orientation fusion  
- No IMU acceleration fusion

## Experiment Launch Structure (Current)

### Wheel-only Launch (Baseline)

- IMU covariance fixer runs  
- Wheel odom runs with `publish_tf:=true`  

### EKF Launch (Baseline)

- IMU covariance fixer runs  
- Wheel odom runs with `publish_tf:=false`  
- EKF runs with `publish_tf:=true`

## Validation Commands Used 

### IMU Sanity Checks

### Frame + time
```bash
ros2 topic echo /imu_raw --once --field header

### Rate Stability

```bash
ros2 topic hz /imu_raw
ros2 topic hz /imu

### Sign Check for ωz

```bash
ros2 topic echo /imu_raw --field angular_velocity.z

### Interpretation Used

| Teleop Key | Visual Rotation | ωz Expected |
|------------|----------------|-------------|
| Left / CCW | robot turns left | positive |
| Right / CW | robot turns right | negative |

### Covariance Check

```bash
ros2 topic echo /imu --once --field angular_velocity_covariance
ros2 topic echo /imu --once --field orientation_covariance

### TF Ownership and Chain Checks

**Confirm `odom → chassis_link` exists**

```bash
ros2 topic echo /tf --once | grep -E "frame_id: odom|child_frame_id: chassis_link"

### Confirm Sensor TF under `chassis_link`

```bash
ros2 topic echo /tf --once | grep -E "frame_id: chassis_link|child_frame_id: imu_link"

### Check TF Publishers (Ownership Debugging)

```bash
ros2 topic info /tf -v

### Expected TF Publishers in EKF Mode

- `_Graph_ROS_TF_*` (sensors)  
- `ekf_filter_node`  
- ❌ `encoder_odom_publisher` must **NOT** appear after cleanup  

### TF Stability Check

```bash
ros2 run tf2_ros tf2_echo odom chassis_link

## Confusions That Were Resolved

**“Why was `/tf` rate higher than `/odom`?”**  
Because `/tf` includes transforms published by multiple nodes (sensor TF graph + estimator TF).  
So `ros2 topic hz /tf` measures aggregated TF traffic, not only `odom → chassis`.

**“Why did `encoder_odom` show as a `/tf` publisher even when `publish_tf=false`?”**  
Because a node can create a publisher endpoint without actively publishing transforms.  
✅ **Final cleanup:** TF broadcaster is only created when `publish_tf=true`, so it no longer appears as a `/tf` publisher in EKF mode.

**“Why does `ros2 topic hz` sometimes warn ‘not published yet’?”**  
It often prints that when started before the first message arrives. If it then reports rates, it’s normal.

## Current Status Snapshot

**Completed (LOCKED)**

| Item                                           | Status       |
|-----------------------------------------------|-------------|
| TF policy and ownership enforcement           | ✅ Locked    |
| Wheel odometry validated                       | ✅ Locked    |
| IMU topic stable (~18 Hz)                      | ✅ Locked    |
| IMU covariance fixed and validated            | ✅ Locked    |
| EKF configured (odom x,y,vx + imu ωz)         | ✅ Locked    |
| EKF publishes `/odometry/filtered` + TF odom→chassis | ✅ Locked |
| Encoder TF disabled in EKF mode               | ✅ Locked    |

## Next Tasks (Immediate)

| Task                                | Why                                        |
|------------------------------------|-------------------------------------------|
| Record reference trajectory bag (`/cmd_vel`, `/clock`) | Replay identical motion for all estimators |
| Record wheel-only run bag           | Baseline comparison dataset               |
| Record EKF run bag                  | Baseline dataset                           |
| Create plots (yaw drift, ωz tracking) | Thesis baseline metrics                    |
