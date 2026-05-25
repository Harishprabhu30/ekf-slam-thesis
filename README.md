# Controlled SLAM Evaluation Pipeline Using ROS 2 and NVIDIA Isaac Sim

This repository contains the implementation, experiment scripts, configuration files, and analysis tools used for a controlled simulation-based evaluation of localization and SLAM methods for autonomous mobile robots.

The project was developed as part of the MSc thesis **“Investigation of SLAM Methods for Autonomous Robots”** and the related IEEE paper on controlled simulation-based SLAM evaluation. The main focus is not only to run SLAM algorithms, but to compare them fairly under controlled and repeatable simulation conditions.

The final evaluation pipeline compares:

- Wheel odometry
- Extended Kalman filter (EKF)-based wheel–IMU fusion
- ORB-SLAM3 stereo visual SLAM

The evaluation is performed using ROS 2, NVIDIA Isaac Sim, recorded ROS 2 datasets, estimator isolation, simulator ground truth for offline evaluation only, and a Python-based trajectory analysis pipeline.

---

## 1. Project Overview

The repository provides a complete workflow for:

1. Creating controlled simulation scenes in NVIDIA Isaac Sim
2. Recording sensor data from a simulated Nova Carter differential-drive robot
3. Replaying the same recorded dataset for separate estimators
4. Running each estimator independently
5. Recording estimator outputs
6. Extracting trajectories from ROS 2 bags
7. Synchronizing estimator and ground-truth trajectories
8. Aligning trajectories using SE(2) without scale correction
9. Computing quantitative metrics
10. Generating tables and figures for thesis and paper reporting

The key design idea is estimator isolation. Only one estimator is active during a run, and simulator ground truth is never used as an estimator input. Ground truth is used only during offline post-processing for alignment and metric computation.

---

## 2. Main Research Purpose

The project investigates how different localization and SLAM methods behave under controlled simulation conditions.

The final V6 lighting robustness study evaluates the methods under:

- Bright lighting
- Dim lighting
- Low-light conditions

Each lighting condition contains three repeated trials. Since exact frame-level trajectory reproduction was not reliable in Isaac Sim, each trial is evaluated against its own simulator ground truth. Results are reported using mean and standard deviation across trials.

The main evaluation metrics include:

- Absolute Trajectory Error (ATE) RMSE
- Normalized ATE
- Relative Pose Error (RPE)
- Drift rate
- Mean absolute yaw error
- Pose output ratio

---

## 3. Important Evaluation Policies

### 3.1 Ground Truth Policy

Simulator ground truth is published through `/gt/odom`, but it is used only for offline evaluation.

It is not used by:

- Wheel odometry
- EKF
- ORB-SLAM3
- TF estimation
- Any online estimator input

This avoids ground-truth leakage.

### 3.2 Estimator Isolation

Each estimator is run separately.

Only one estimator is active during a replay run. This avoids hidden TF interaction between estimators and prevents one estimator from affecting another through shared transforms.

### 3.3 Alignment Policy

Trajectory comparison uses SE(2) alignment:

- Translation correction
- Planar yaw correction
- No scale correction

No scale correction is applied because stereo ORB-SLAM3 is expected to produce metric-scale output.

### 3.4 V6 Trial Policy

The repeated V6 trials follow the same route protocol, but the trajectories are not assumed to be frame-identical. Therefore, each trial is evaluated against the ground truth recorded in that same trial.

---

## 4. Repository Structure

```text
.
├── additional files
│   ├── Experiment_Insights
│   ├── IMPLEMENTATION_READMEs
│   ├── Phase_1_ros2_ws
│   ├── Problem_random_shift_rosbag2_2026_02_09-17_47_56
│   ├── random_shift_solvedrosbag2_2026_02_09-18_13_52
│   ├── rosbag2_2026_02_06-18_24_31_without_odom
│   └── tf_tree_versions_pdf
│
├── analysis
│   ├── baseline_analysis_scripts
│   ├── gt_evaluation_scripts
│   ├── results_v2_lighting_sweep
│   ├── results_v6_lighting_sweep
│   ├── results_v6_orb_ablation
│   ├── light_inspection_bright_v2.json
│   └── physics_inspection_ekf_slam2_v2.json
│
├── bags
│   ├── experiments
│   ├── trajectories
│   ├── traj_with_cam_records
│   └── traj_without_cam_records
│
├── images
│   ├── bright_dim_lowlight_env.png
│   ├── nova_carter_with_sensors.png
│   └── lighting condition screenshots
│
├── rviz_images
│   └── RViz screenshots
│
├── scripts
│   ├── record_master_dataset.sh
│   ├── record_master_dataset_v2_manual.sh
│   ├── run_experiment.sh
│   ├── run_experiment_v2.sh
│   └── deprecated scripts
│
├── src
│   ├── ekf_slam_sim
│   ├── isaac_ros_common
│   ├── ORB_SLAM3
│   ├── orb_slam3_ros2
│   └── Pangolin
│
├── tools
│   ├── trajectory replay tools
│   ├── photometric variant tools
│   └── pose timeline extraction tools
│
├── video_records
│   └── ORB-SLAM3 visualization videos
│
├── ekf-slam2-bright.usd
├── ekf-slam2-dim.usd
├── ekf-slam2-lowlight.usd
├── ekf-slam2.usd
├── README.md
└── LICENSE
```

# 5. Important Folders

## `src/`

Contains the ROS 2 packages and third-party SLAM dependencies.

### Main Packages

#### `ekf_slam_sim`

Contains the custom ROS 2 nodes, launch files, configuration files, wheel odometry, EKF setup, simulation integration, and experiment support.

#### `orb_slam3_ros2`

ROS 2 wrapper used to run ORB-SLAM3 and publish pose output.

#### `ORB_SLAM3`

ORB-SLAM3 source code.

#### `Pangolin`

Dependency used by ORB-SLAM3.

#### `isaac_ros_common`

NVIDIA Isaac ROS common utilities.

---

## `scripts/`

Contains shell scripts for recording datasets and running estimator experiments.

### Important Scripts

```bash
scripts/run_experiment_v2.sh
scripts/record_master_dataset.sh
scripts/record_master_dataset_v2_manual.sh
scripts/run_experiment.sh
```

---

## Notes

The V6 lighting experiments were recorded using the manual fixed-rate recording workflow.

## `analysis/results_v6_lighting_sweep/`

This is the main final analysis folder for the lighting robustness study.

## Important Contents

```text
analysis/results_v6_lighting_sweep/
├── aligned
├── extracted
├── metrics
├── final_reporting
├── final_reporting_t03
├── paper_figures_t03
├── paper_tables_t03
├── plots
├── scripts
├── tables
└── v6_run_registry.csv
```


## Key Files

```text
metrics/v6_all_trial_metrics.csv
tables/v6_mean_std_by_lighting_estimator.csv
final_reporting_t03/table1_main_v6_lighting_mean_std.csv
final_reporting_t03/table2_orbslam3_default_vs_tuned_t03.csv
```

## `analysis/results_v6_lighting_sweep/scripts/`

Contains the final analysis scripts.

---
# 6. Dataset Availability

Large ROS 2 bag files are not included directly in this repository because of their size.

## The Repository Includes

- Scripts
- Configuration files
- Analysis code
- Result tables
- Generated figures
- Experiment manifests
- Metadata files describing recorded bags

The full recorded ROS 2 bags can be shared upon request.

---

# 7. System Requirements

The project was developed and tested with:

- Ubuntu Linux
- ROS 2 Humble
- NVIDIA Isaac Sim 5.0
- Python 3
- OpenCV
- NumPy
- pandas
- matplotlib
- ORB-SLAM3 dependencies
- Pangolin

Recommended GPU support is needed for running Isaac Sim smoothly.

---

# 8. Building the Workspace

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If ORB-SLAM3 or Pangolin needs to be rebuilt separately, build those dependencies first according to their normal build process, then rebuild the ROS 2 workspace.

---
# 9. Running Isaac Sim Scenes

## Main Isaac Sim Scene Files

```text
ekf-slam2.usd
ekf-slam2-bright.usd
ekf-slam2-dim.usd
ekf-slam2-lowlight.usd
```
---

# 10. Recording a Master Dataset

For the final manual fixed-rate recording workflow:

```bash
bash scripts/record_master_dataset_v2_manual.sh
```

This records sensor and ground-truth topics such as:

```text
/cmd_vel
/traj_phase
/camera/left/image_raw
/camera/right/image_raw
/camera/left/camera_info
/camera/right/camera_info
/imu_raw
/joint_states
/tf
/gt/odom
/clock
```

## Notes

The `/gt/odom` topic is recorded only for offline evaluation.

# 11. Running Estimator Experiments

After a master bag has been recorded, estimators are run separately.

## Typical Estimator Modes

```bash
bash scripts/run_experiment.sh wheel
bash scripts/run_experiment.sh ekf
bash scripts/run_experiment.sh orbslam3
```

Or, for the V2/V6 workflow:

```bash
bash scripts/run_experiment_v2.sh wheel
bash scripts/run_experiment_v2.sh ekf
bash scripts/run_experiment_v2.sh orbslam3
```

## Notes

Before running these scripts, check and update the bag paths and output paths inside the script according to the dataset being evaluated.

---

# 12. Running the V6 Lighting Analysis

The final V6 analysis is stored under:

```text
analysis/results_v6_lighting_sweep/
```

## Run the Full Batch Pipeline

```bash
python3 analysis/results_v6_lighting_sweep/scripts/04_run_v6_batch_pipeline.py
```

## This Performs

- Trajectory extraction
- Motion-onset synchronization
- SE(2) alignment
- Metric computation
- Result saving

## 13 ORB-SLAM3 Stereo Visual SLAM

ORB-SLAM3 is evaluated in stereo mode using simulated stereo camera streams.

### Camera Configuration Used in the Final Experiments

| Parameter   | Value      |
|-------------|------------|
| Resolution  | 640 × 480 |
| Frame rate  | 15 Hz     |

### Main Selected ORB-SLAM3 Extractor Configuration

```yaml
ORBextractor.nFeatures: 1800
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 10
ORBextractor.minThFAST: 4
```

### Ablation Configuration (Default ORB-SLAM3 Settings)

A separate ablation compares this tuned configuration against a default configuration using:

```yaml
ORBextractor.nFeatures: 1000
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```
---

# 14. Evaluated Methods


## 14.1 Wheel Odometry

Wheel odometry integrates left and right wheel encoder information using a differential-drive motion model.

It is simple and efficient, but it accumulates drift due to:

- Wheel slip
- Calibration error
- Surface interaction
- Heading error accumulation

---

## 14.2 EKF-Based Wheel–IMU Fusion

The EKF fuses wheel odometry with IMU yaw-rate information in a planar configuration.

It can improve short-term consistency and smoothness, but it does not fully remove long-term drift because no absolute global correction source is used.

---

# 15. V6 Lighting Sweep

The final lighting sweep evaluates three lighting conditions:

| Condition  | RectLight 1 | RectLight 2 | DomeLight |
|------------|-------------|-------------|------------|
| Bright     | 15000       | 15000       | 1000       |
| Dim        | 3000        | 3000        | 100        |
| Low-light  | 70          | 70          | 10         |

## Repeated Trials

Each lighting condition contains three repeated trials:

```text
t01
t02
t03
```

## Route Protocol

The route protocol includes:

- Initial rest
- Square motion
- Straight motion
- Clockwise rotation
- Curved motion

---

# 16. Metrics

The evaluation computes the following metrics.

## Absolute Trajectory Error

ATE measures global position difference between the aligned estimated trajectory and ground truth.

## Normalized ATE

Normalized ATE divides ATE RMSE by the ground-truth path length of the same trial.

## Relative Pose Error

RPE measures local motion consistency over a fixed time interval.

## Drift Rate

Drift rate measures final accumulated position error relative to total travelled distance.

## Yaw Error

Yaw error measures heading difference between the estimated and ground-truth yaw angles.

## Pose Output Ratio

Pose output ratio compares the number of estimator pose messages with the number of ground-truth pose messages. It is used as a diagnostic quantity, not as an accuracy metric.

---

# 17. Key Results Summary

The final V6 lighting sweep showed that wheel odometry and EKF-based wheel–IMU fusion have similar drift behaviour because neither method receives a global correction source.

ORB-SLAM3 showed stronger condition-dependent behaviour. The results should not be interpreted as a simple monotonic lighting trend. Instead, ORB-SLAM3 performance depends on the combined effect of:

- Lighting
- Feature availability
- Route execution
- Tracking stability
- Extractor configuration

The ORB-SLAM3 parameter ablation showed that the tuned extractor configuration improved the selected low-light run, but it was not uniformly better in bright and dim scenes.

---

# 18. Reproducibility Notes

To reproduce the evaluation, keep the following policies fixed:

- Use one active estimator per replay.
- Do not use `/gt/odom` as estimator input.
- Use `/gt/odom` only during offline evaluation.
- Evaluate each repeated trial against its own ground truth.
- Use SE(2) alignment without scale correction.
- Keep estimator configuration fixed during the main lighting sweep.
- Report mean and standard deviation across repeated trials.
- Treat pose output ratio as diagnostic, not as accuracy.

---

# 19. Known Limitations

The current final evaluation has the following limitations:

- The lighting sweep uses three trials per condition.
- The repeated trajectories follow the same route protocol but are not frame-identical.
- The study does not include a stereo-inertial baseline such as ORB-SLAM3 stereo-inertial, OpenVINS, or VINS-Fusion.
- Large ROS 2 bag files are not stored directly in the repository.
- The evaluation is simulation-based and should be extended with real-world experiments.

---

# 20. Future Work

Future work may include:

- Adding stereo-inertial ORB-SLAM3
- Adding OpenVINS or VINS-Fusion
- Increasing the number of repeated trials
- Testing dynamic lighting conditions
- Testing surface friction variation
- Adding sensor noise sweeps
- Adding LiDAR or cuVSLAM baselines
- Extending the evaluation to real robot datasets

---

# 21. License

This repository is released under the MIT License.

---

## 22. Contact

For questions about this project, implementation details, or research collaboration, please contact:

**Harish Prabhu**  
MSc Artificial Intelligence Systems  
Vilnius Gediminas Technical University  
Vilnius, Lithuania  

- GitHub: [Harishprabhu30](https://github.com/Harishprabhu30)
- ORCID: [0009-0009-2301-7474](https://orcid.org/0009-0009-2301-7474)
- LinkedIn: https://www.linkedin.com/in/harishprabhu3007/
- Email: harishprabhu3007@gmail.com
