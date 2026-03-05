# Evaluation Pipeline

## 1. Purpose

This document describes the complete evaluation pipeline used to analyze the performance of odometry estimators against simulator ground truth.

The pipeline converts raw ROS2 bag recordings into:

- Synchronized trajectories
- Aligned trajectories
- Trajectory error metrics
- Phase-wise performance statistics
- Diagnostic plots

The evaluation framework ensures **reproducibility**, **fair comparison**, and **strict separation** between estimation and ground truth data.

## 2. Evaluation Data Sources

The evaluation pipeline uses three datasets.

### 2.1 Ground Truth Dataset

**Dataset:** `run_gt_eval_v1`

**Topics:**

```
/gt/odom
/traj_phase
```

**Purpose:**

- Provides simulator-derived ground truth pose  
- Provides phase transition timestamps for trajectory segmentation

### 2.2 Wheel Odometry Baseline

**Dataset:** `run_wheel_only_v2`

**Topics used:**

```
/odom
```

**Characteristics:**

- Differential drive wheel encoder integration  
- No sensor fusion  
- Represents the **baseline odometry estimator**

### 2.3 EKF Baseline

**Dataset:** `run_ekf_baseline_v2`

**Topics used:**

```
/odometry/filtered
```

**Characteristics:**

- Wheel + IMU fusion  
- Implemented using `robot_localization` EKF  
- Represents the **sensor fusion baseline**

## 3. Evaluation Pipeline Overview

The evaluation pipeline consists of six stages:

```
ROS2 Bags
   │
   ▼
Trajectory Extraction
   │
   ▼
Time Synchronization
   │
   ▼
Trajectory Alignment (SE2)
   │
   ▼
Metric Computation
   │
   ▼
Phase Segmentation
   │
   ▼
Visualization and Summary Tables
```

Each stage is implemented using Python scripts.

## 4. Trajectory Extraction

The first step converts ROS2 bag messages into trajectory CSV files.

**Script:** `extract_metrics.py`

**Inputs:**

```
/odom
/odometry/filtered
/gt/odom
```

**Output files:** `analysis/results_gt/`

```
gt_traj.csv
wheel_traj.csv
ekf_traj.csv
traj_phase_events.csv
```

Each trajectory file contains:

- `t_ns`  
- `x`  
- `y`  
- `yaw`

**Yaw extraction from quaternion orientation:**

\[
\theta = \text{atan2}\Big( 2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2) \Big)
\]

Timestamps are stored in **nanoseconds** to maintain precision.

## 5. Time Synchronization

Different estimators start at slightly different timestamps.  
To allow fair comparison, trajectories must be synchronized to a **common timeline**.

### Synchronization Strategy

The timeline is anchored using the timestamp of the first trajectory phase.

```
phase = 1  → straight motion
```

**Synchronization reference:**

```
t0 = timestamp of phase 1 event
```

For each trajectory:

\[
t_{\text{aligned}} = t_{\text{raw}} - t_0
\]

This produces `t_s`, which represents **time since trajectory start**.

## 6. Trajectory Alignment

Even with synchronized timestamps, estimators may start with different coordinate offsets.  
Trajectories are therefore aligned using a **2D rigid transformation (SE(2))**.

### The alignment corrects:

- Translation offset  
- Rotation offset

**Transformation equation:**

\[
p_{\text{aligned}} = R \, p_{\text{est}} + t
\]

Where:

- \(R\) = 2D rotation matrix  
- \(t\) = translation vector

Alignment parameters are computed using the **initial overlapping segment** between:

- GT trajectory  
- Estimator trajectory

The aligned estimator trajectory is stored as:

```
est_x_al
est_y_al
est_yaw_al
```

**Resulting files:**

```
wheel_synced_aligned.csv
ekf_synced_aligned.csv
```

## 7. Error Metrics

After synchronization and alignment, **error metrics** are computed.

### 7.1 Absolute Trajectory Error (ATE)

ATE measures the distance between the estimator pose and ground truth.

\[
\text{ATE}_i = (x_i^{\text{gt}} - x_i^{\text{est}})^2 + (y_i^{\text{gt}} - y_i^{\text{est}})^2
\]

**Reported metrics:**

- `ATE_RMSE`  
- `ATE_mean`  
- `ATE_max`

### 7.2 Yaw Error

Yaw error measures **heading deviation**.

\[
\theta_{\text{err}} = \text{wrap}(\theta_{\text{gt}} - \theta_{\text{est}})
\]

Where:

```
wrap(θ) ∈ [-π, π]
```

**Reported metrics:**

- `yaw_abs_mean`  
- `yaw_abs_max`

### 7.3 Relative Pose Error (RPE)

RPE measures **drift over short time intervals**.

For a time interval \(\Delta t\):

\[
\text{RPE}_i = \left\| (p_{i+\Delta t}^{\text{est}} - p_i^{\text{est}}) - (p_{i+\Delta t}^{\text{gt}} - p_i^{\text{gt}}) \right\|
\]

**Evaluation parameters:**

```
Δt = 1 second
```

**Reported metrics:**

- `rpe_trans_rmse`  
- `rpe_yaw_rmse`

## 8. Phase Segmentation

The trajectory is divided into **phases** based on `/traj_phase` events.

**Phase definitions:**

```
0 → stop
1 → straight
2 → square
3 → CW rotation
4 → arc
```

Phase segments are computed as:

```
[phase_i_timestamp, phase_(i+1)_timestamp)
```

Metrics are computed **separately for each phase**, enabling detailed analysis of estimator behavior under different motion patterns.

## 9. Visualization

The pipeline produces several **diagnostic plots**.

### 9.1 Trajectory Overlay

- GT vs Wheel vs EKF  
- Used to visually inspect drift behavior

### 9.2 ATE vs Time

- Shows how positional error evolves during the trajectory  
- Phase boundaries are annotated to identify which motion pattern causes error growth

### 9.3 Yaw Error vs Time

- Shows heading drift over time  
- Helps identify rotational bias and integration drift

## 10. Final Summary Table

All metrics are aggregated into a **final evaluation table**.

**Example:**

```
Estimator    Phase        ATE_RMSE    Yaw_mean
----------------------------------------------
wheel        straight     0.54 m      0.079 rad
ekf          straight     0.62 m      0.093 rad
wheel        square       0.41 m      0.128 rad
ekf          square       0.46 m      0.145 rad
wheel        arc          0.88 m      0.26 rad
ekf          arc          0.92 m      0.17 rad
```

**Global metrics** are also computed across phases:

```
GLOBAL (1–4)
```

*Note:* The stop phase is excluded from global metrics to avoid bias from stationary samples.

## 11. Reproducibility

All evaluation scripts are stored in:

```
analysis/gt_evaluation_scripts/
```

The evaluation environment is isolated using:

```
analysis_env
```

**Dependencies include:**

- `numpy`  
- `pandas`  
- `matplotlib`  
- `rosbag2_py`  
- `rclpy`

This ensures the evaluation pipeline can be **reproduced exactly**.

## 12. Output Artifacts

The pipeline generates the following outputs.

### Trajectory Data

```
gt_traj.csv
wheel_traj.csv
ekf_traj.csv
```

### Aligned Trajectories

```
wheel_synced_aligned.csv
ekf_synced_aligned.csv
```

### Phase Metrics

```
phase_metrics.csv
```

### Final Summary

```
final_summary_table.csv
```

### Visualization

```
trajectory_overlay.png
ate_vs_time.png
yaw_error_vs_time.png
```

** GT BASELINE WHEEL ONLY AND EKF ANALYSIS INTERRETATIONS : SEE FILE GT_Baseline_Interpretation.md **

