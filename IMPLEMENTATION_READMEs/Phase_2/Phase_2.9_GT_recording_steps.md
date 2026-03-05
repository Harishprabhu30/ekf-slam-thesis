# Ground Truth Recording Protocol

## 1. Objective

The purpose of this protocol is to record simulator ground truth (GT) trajectories for evaluation of odometry and SLAM estimators in a controlled and reproducible environment.

Ground truth data must satisfy the following constraints:

- It must represent the true robot pose from the simulator physics engine.
- It must never influence the estimator pipeline.
- It must be recorded independently from estimator execution.
- It must only be used during offline evaluation and benchmarking.

This separation ensures that the evaluation remains unbiased and prevents ground truth leakage into the estimator state.

## 2. Experimental Architecture

The experiment is conducted using:

- NVIDIA Isaac Sim 5.0
- ROS2
- Nova Carter differential drive robot

Ground truth is obtained from the simulator physics state and published to ROS2 as an odometry message.

The architecture follows strict separation:

```
Simulator Physics
       │
       ▼
Isaac Compute Odometry Node
       │
       ▼
ROS2 Odometry Publisher
       │
       ▼
/gt/odom   (recorded only)
```

**Important rule:**
**Ground truth is never connected to the estimator TF tree or EKF inputs.**

## 3. Frame and TF Policy

To prevent estimator contamination, the following rules are enforced.

### Allowed TF structure for estimation

```
odom → chassis_link → sensors
```

### Ground truth policy

Ground truth is published as an isolated odometry topic:

```
/gt/odom
```

Frame configuration:

```
frame_id: gt_world
child_frame_id: chassis_link
```

**Critical constraint:**

- **GT frames must never appear in `/tf`**
- **No GT transforms may be broadcast**

Verification is performed using:

```
ros2 topic echo /tf
ros2 run tf2_tools view_frames
```

The TF tree must **not include `gt_world`**.

## 4. Clock Policy

The simulation uses a single authoritative clock source.

```
/clock → Isaac Sim
```

### Rules

- Isaac Sim is the only clock publisher
- rosbag replay is executed **without `--clock`**
- `/clock` is **not recorded** in command bags

### Verification

```
ros2 topic info /clock -v
```

Publisher count **must equal 1**.

5. Ground Truth Publisher Graph

Ground truth is produced using the Isaac Sim action graph with the following nodes:

On Playback Tick

Isaac Compute Odometry

Read Simulation Time

ROS2 Context

ROS2 Odometry Publisher

Connections:

```
OnPlaybackTick → IsaacComputeOdometry
OnPlaybackTick → ROS2PublishOdometry

IsaacComputeOdometry outputs:
    position → ROS2PublishOdometry.position
    orientation → ROS2PublishOdometry.orientation
    linear_velocity → ROS2PublishOdometry.linear_velocity
    angular_velocity → ROS2PublishOdometry.angular_velocity

ReadSimulationTime → ROS2PublishOdometry.timestamp
ROS2Context → ROS2PublishOdometry.context
```

### Parameters

```
chassis_frame_id = chassis_link
odom_frame_id = gt_world
topic = /gt/odom
```

## 6. Pre-Recording Sanity Checks

Before recording, the following checks must pass.

### 6.1 Clock Validation

```
ros2 topic info /clock -v
```

**Expected:**

```
Publisher count: 1
Node: Isaac Sim
```

### 6.2 TF Tree Validation

```
ros2 run tf2_tools view_frames
```

**Expected tree:**

```
chassis_link
 ├── camera_link
 ├── imu_link
 └── lidar_link
```

GT frames must **not appear**.

### 6.3 Ground Truth Topic Check

```
ros2 topic info /gt/odom -v
```

**Expected:**

```
Type: nav_msgs/msg/Odometry
Publisher count: 1
```

### 6.4 TF Leakage Check

```
ros2 topic echo /tf --once
```

**Verify:**

- `gt_world` does **not appear**

## 7. Ground Truth Recording Procedure

Ground truth is recorded while replaying the canonical trajectory command bag.

**Example command:**

```
ros2 bag record -o run_gt_eval_v1 \
  /gt/odom \
  /traj_phase
```

The `/traj_phase` topic records phase transitions of the trajectory:

```
0 → stop
1 → straight
2 → square
3 → CW rotation
4 → arc
```

## 8. Dataset Validation

After recording, the dataset is verified using:

```
ros2 bag info run_gt_eval_v1
```

**Expected properties:**

- Duration ≈ 270 seconds
- `/gt/odom` message count ≈ 4500
- `/traj_phase` message count = 5

**Example:**

```
Topic: /gt/odom      | Count: 4627
Topic: /traj_phase    | Count: 5
```

## 9. Issues Encountered and Resolutions

Several implementation issues were encountered during GT recording.

### Issue 1 — Missing Phase Event

**Problem:**  
Initial recording missed one phase transition.

**Resolution:**  
Ground truth bag was re-recorded ensuring all `/traj_phase` events were captured.

### Issue 2 — Timestamp Misalignment

**Problem:**  
Early experiments attempted to synchronize trajectories using motion onset detection.  
Phase events occurred slightly before motion started, leading to negative timestamps and segmentation collapse.

**Resolution:**  
Trajectory time was anchored using:

```
t0 = timestamp of phase = 1
```

This guarantees correct phase segmentation.

### Issue 3 — TF Leakage Risk

**Problem:**  
Initial design considered publishing GT transforms.  
GT transforms appearing in `/tf` would contaminate estimator pipelines.

**Resolution:**  
Ground truth is published only as:

```
/gt/odom
```

with **no TF broadcasting**.

## 10. Final Locked Ground Truth Dataset

Final dataset used for evaluation:

`run_gt_eval_v1`

**Contents:**

```
/gt/odom
/traj_phase
```

**Dataset properties:**

- Duration ≈ 270 s
- Physics-derived pose
- No TF leakage
- Independent from estimator execution

This dataset serves as the **reference trajectory** for all subsequent estimator evaluations.

## 11. Role in Evaluation Pipeline

The ground truth dataset is used exclusively during **offline analysis** to compute:

- Absolute Trajectory Error (ATE)  
- Relative Pose Error (RPE)  
- Phase-wise trajectory drift  
- Yaw error evolution

**Ground truth is never used by the estimator during runtime.**

** GT BASELINE WHEEL ONLY AND EKF ANALYSIS INTERRETATIONS : SEE FILE GT_Baseline_Interpretation.md **
















