Baseline Recording Protocol (Wheel-only + EKF)

## Context

We need repeatable baseline datasets for:

- Wheel-only odometry (mechanical baseline)
- EKF baseline (robot_localization fusion)

All experiments must use:

- identical robot (Nova Carter), world, physics, sensors, noise, TF policy
- identical command trajectory
- no simulator ground truth usage

A critical requirement is a stable, single time authority (/clock) to prevent TF2 and EKF failures.

Final Clock Decision (Why “Live Sim Clock” Wins)

## Problem observed

When replaying bags with `--clock` or when `/clock` existed in the command bag, the system sometimes produced non-monotonic time (time jumping backwards). This caused:

- `tf2_buffer: Detected jump back in time. Clearing TF buffer.`
- EKF not publishing `/odometry/filtered` (no time progression / filter pauses)
- nondeterministic results between runs

Root cause: multiple `/clock` publishers or mixed clock content recorded in command bags.

## Final decision

Isaac Sim is the single time authority.  
We do not replay `/clock` from rosbag.  
We record and replay only the command inputs (`/cmd_vel` + `/traj_phase`).

This removes time conflicts and makes EKF stable.

Locked Time Policy

✅ During ALL baseline runs:

- `/clock` must be published by Isaac Sim only
- rosbag playback must **not** publish `/clock`
- never use `ros2 bag play ... --clock`

Expected `/clock` state:

Run: ros2 topic info /clock -v

Correct output must show:

- Publisher count: 1
- Node name: _Graph_ROS_Clock_PublishClock
- (QoS reliability can be RELIABLE — fine)

No rosbag2_player should appear as a `/clock` publisher.

---

# Command Bag (Reference Trajectory)

## Purpose

A canonical input trajectory for fairness across estimators:

- same `/cmd_vel`
- same phase markers for segmentation

## Recorded topics

Only:

- `/cmd_vel` (geometry_msgs/Twist)
- `/traj_phase` (std_msgs/Int32)

## Command bag recording
ros2 bag record -o traj_cmd_clean /cmd_vel /traj_phase

## Replay (used for wheel-only and EKF)
ros2 bag play traj_cmd_clean

Important

- Replay is done without `--clock`
- Isaac must be running and publishing `/clock` continuously

---

# Baseline Output Bags

## 1) Wheel-only baseline bag

Wheel odometry node publishes `/odom`, and owns TF `odom → chassis_link`.

Record:

ros2 bag record -o run_wheel_only_v2 \
  /clock /tf /tf_static \
  /odom \
  /imu /imu_raw \
  /joint_states \
  /traj_phase
  
Then replay the command bag:

ros2 bag play traj_cmd_clean

## Wheel-only correctness checks

TF ownership:

- only wheel node publishes `odom → chassis_link` (`publish_tf:=true`)

Confirm TF:

ros2 run tf2_ros tf2_echo odom chassis_link

- Confirm /odom is nonzero and updating:

ros2 topic hz /odom

## 2) EKF baseline bag

EKF publishes `/odometry/filtered` and owns TF `odom → chassis_link`.  
Wheel TF must be disabled in EKF mode.

Record:

ros2 bag record -o run_ekf_baseline_v2 \
  /clock /tf /tf_static \
  /odom /odometry/filtered \
  /imu /imu_raw \
  /joint_states \
  /traj_phase
  
  
Then replay the command bag:

ros2 bag play traj_cmd_clean

## EKF correctness checks

- EKF output rate exists: 

ros2 topic hz /odometry/filtered

- TF ownership:

	- EKF publishes odom → chassis_link

	- wheel node publish_tf:=false

- Confirm TF:

ros2 run tf2_ros tf2_echo odom chassis_link

- Confirm EKF is publishing:

ros2 topic echo /odometry/filtered --once

## Bag Validation Checklist (After Recording)

Run:

ros2 bag info <bag_name>

# Bag Content Requirements

## Command bag (`traj_cmd_clean`) must contain:

- `/cmd_vel` count > 0
- `/traj_phase` count > 0
- must NOT contain `/clock`

---

## Wheel-only bag must contain:

- `/odom` count > 0
- `/tf` count > 0
- `/clock` count > 0
- `/imu` and `/imu_raw` counts > 0
- `/joint_states` count > 0

---

## EKF bag must contain:

- `/odometry/filtered` count > 0 ✅ **required**
- `/tf` count > 0
- `/clock` count > 0
- `/odom`, `/imu`, `/joint_states` counts > 0

If `/odometry/filtered` is 0 → EKF was not running properly or time wasn’t advancing.

---

# What to Avoid (Hard Rules)

- Do not record `/clock` in the command bag.
- Do not run `ros2 bag play ... --clock`.
- Do not allow multiple `/clock` publishers at once.
- Do not change world, physics, sensors, timestep, trajectory between estimators.
- Do not use simulator ground truth or a world frame.

## Notes on /cmd_vel Frequency

Teleop publishes /cmd_vel mainly on key changes, so /cmd_vel counts may be low (tens of messages). That’s expected because the robot controller continues applying the last command until changed. This is acceptable but must be documented.

# Quick “Golden Run” Order (Recommended)

## Wheel-only:

1. Start Isaac (clock ON)
2. Launch wheel-only stack
3. Start `ros2 bag record ...`
4. `ros2 bag play traj_cmd_clean`
5. Stop recorder after playback ends
6. Validate with `ros2 bag info`

## EKF:

1. Start Isaac (clock ON)
2. Launch EKF stack (wheel TF OFF)
3. Start `ros2 bag record ...`
4. `ros2 bag play traj_cmd_clean`
5. Stop recorder after playback ends
6. Validate with `ros2 bag info` (check `/odometry/filtered` count)
