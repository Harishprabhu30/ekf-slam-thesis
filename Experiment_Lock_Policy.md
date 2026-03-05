# Experiment Lock Policy

## 1. Purpose

This document defines the **fixed experimental configuration** used throughout the thesis.  
The purpose is to ensure **fair and reproducible comparison** between different state estimation methods.

- All estimators must be evaluated under **identical simulation conditions**  
- Only the **estimator algorithm itself** may change

This prevents unintended experimental bias and ensures that **performance differences arise solely from the estimation method**.

## 2. Locked Simulation Environment

The simulation environment is **fixed for all experiments**.

**Environment configuration:**

- **Simulator:** NVIDIA Isaac Sim 5.0  
- **World file:** ground plane environment  
- **Rendering:** eco mode  
- **Reflections:** disabled  
- **Translucency:** disabled  
- **Physics:** enabled

These settings maintain **stable simulation frame rates** while ensuring **deterministic physics behavior**.

> **Note:** No changes to the simulation environment are allowed after this lock.

## 3. Robot Model

**Robot platform:**

- **Robot:** Nova Carter (Differential Drive)

**Robot parameters remain fixed:**

- Wheel radius  
- Wheel separation  
- Joint configuration  
- Robot URDF

The robot model **must not be modified** between experiments.

## 4. Sensor Configuration

The following sensors are mounted on the robot:

- Wheel encoders  
- IMU  
- RGB camera  
- LiDAR

**Sensor placement** relative to `chassis_link` is fixed.  
Sensor noise models and update rates **remain unchanged** across experiments.

**TF structure for sensors:**

```
chassis_link
 ├── camera_link
 ├── imu_link
 └── lidar_link
```

> **Important:** Sensor transforms must **not change** during estimator comparisons.

## 5. TF Architecture

The **estimation TF tree** follows a strict policy.

**Allowed TF chain:**

```
odom → chassis_link → sensors
```

**Rules:**

- Only **one node** may publish `odom → chassis_link`  
- Sensor frames are **static relative to `chassis_link`**  
- The world frame from the simulator is **never used** in the estimator pipeline  
- Ground truth frames must **never appear in `/tf`**

## 6. Clock Policy

The **simulation clock** is controlled exclusively by Isaac Sim.

```
/clock → published by Isaac Sim
```

**Rules:**

- Single clock publisher  
- Rosbag replay **without `--clock`**  
- Clock topic **not recorded** in command bags

> Clock source must remain **unchanged across experiments**.

## 7. Trajectory Dataset

All estimators must be evaluated using the **same reference trajectory**.

**Canonical trajectory bag:** `traj_cmd_clean`

**Topics:**

```
/cmd_vel
/traj_phase
```

The trajectory consists of the following **motion phases**:

```
0 – stop
1 – straight motion
2 – square trajectory
3 – clockwise rotation
4 – arc trajectory
```

> Trajectory bags must **not be modified** between experiments.

## 8. Ground Truth Dataset

Ground truth data is recorded independently using the protocol described in:

```
GT_RECORDING_PROTOCOL.md
```

**Dataset used for evaluation:** `run_gt_eval_v1`

**Topics:**

```
/gt/odom
/traj_phase
```

> Ground truth data is used **only for offline evaluation** and must **never be used by any estimator node during runtime**.

## 9. Evaluation Pipeline

All estimator results are processed using the **same evaluation pipeline**.

**Pipeline stages:**

1. Trajectory extraction  
2. Time synchronization  
3. SE(2) trajectory alignment  
4. Metric computation  
5. Phase segmentation  
6. Visualization

**Evaluation scripts are stored in:**

```
analysis/gt_evaluation_scripts/
```

**Evaluation environment:**

```
analysis_env  (Python virtual environment)
```

> Evaluation methods must remain **unchanged** when comparing estimators.

## 10. Allowed Experimental Changes

Between experiments, **only the estimator stack** may change.

**Examples:**

- Wheel odometry baseline  
- EKF sensor fusion  
- ORB-SLAM  
- cuVSLAM  
- Hybrid SLAM approaches

> All other system components must remain **fixed**.

## 11. Summary

The experimental configuration described in this document ensures that:

- All estimators operate under **identical conditions**  
- Results are **directly comparable**  
- Experimental bias is **avoided**  
- The evaluation remains **reproducible**

> This **locked configuration** provides the baseline for all future SLAM experiments conducted in this thesis.


## Git Tag: `v1.0-gt-baseline-lock`

This tag marks the **first locked version** of the experimental setup and evaluation pipeline.  

All simulation parameters, robot and sensor configurations, ground truth datasets, trajectory datasets, and evaluation scripts are **frozen in this state**.  

This ensures that all results obtained from this point onward are **directly comparable** and reproducible.  

Future estimator development and SLAM experiments will reference this version as the **baseline configuration**.
