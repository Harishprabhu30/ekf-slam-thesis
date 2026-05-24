# EKF-SLAM Experimental Framework Ready

## 1. Architectural Lock-In

The experimental framework has been finalized under strict reproducibility constraints:

### Immutable Components (Frozen)

- Evaluation USD world file
- Nova Carter differential-drive robot model
- Physics parameters
- Sensor placement and noise configuration
- TF architecture (`odom → chassis_link → sensors`)
- Simulation timestep
- Rendering optimization settings
- Reference control trajectory (to be recorded once)

Only the estimator stack will vary between experiments.

## 2. TF Architecture Finalized

### Allowed TF Chain
odom → chassis_link → sensors

Ownership Modes:
Mode	TF Owner
Wheel-only	encoder_odom_publisher
EKF	ekf_filter_node
Future SLAM	SLAM node

Rules enforced:

No world frame

No simulator ground-truth transforms

Single publisher for odom → chassis_link

Wheel TF toggled via publish_tf parameter

TF validation confirmed using:

tf2_echo odom chassis_link
ros2 topic echo /tf --once

## 3. Differential Drive Command Architecture Refactored

### Previous Issue

Isaac keyboard teleop directly drove wheel joints.

No ROS `/cmd_vel` topic was exposed.

Motion was not reproducible or recordable.

### Solution Implemented

Disabled Isaac internal keyboard teleop.

Created new DiffDrive OmniGraph.

Added **ROS2 Subscribe Twist** node.

Connected:

- `Twist.linear.x` → `desiredLinearVelocity`
- `Twist.angular.z` → `desiredAngularVelocity`

Connected **ROS2 Context** to subscriber.

Verified correct execution tick wiring.

### Now motion flow is:
ROS teleop → /cmd_vel → OmniGraph → DiffDrive → Wheel joints

### This Enables

- Recording of canonical command input  
- Deterministic replay for all estimator runs  

## Structure of Recorded Trajectory

- 10s stationary (filter stabilization)  
- Straight segment (table-length)  
- Square loop around table  
- In-place spin  
- Arc movements  
- 10s stationary  

### Trajectory Characteristics

- Contains translation-only segments  
- Contains rotation-only segments  
- Contains combined linear + angular motion  
- Returns near starting area (loop closure proxy possible)  

### Topics Recorded

- `/cmd_vel`  
- `/clock`  
- `/traj_phase`  

### Replay Verified

- Robot motion reproducible  
- `/cmd_vel` shows non-zero values during motion  
- Phase labeling working correctly  

This bag is now frozen as the canonical control-input trajectory.

## 5. Phase Labeling System Implemented

A lightweight ROS2 node was created to publish:

- `/traj_phase` (`std_msgs/Int32`)

### Phase Definitions

| ID | Motion   |
|----|---------|
| 0  | Stop    |
| 1  | Straight|
| 2  | Square  |
| 3  | Spin    |
| 4  | Arc     |

Purpose:

Enables segment-wise evaluation

Allows per-motion metric computation

Improves thesis clarity and scientific rigor

This node is only active during command recording, not during evaluation replay.

## 6. System Validation Checks Completed

Verified:

- `/cmd_vel` publishes correctly  
- DiffDrive responds 1:1 to ROS commands  
- No duplicate wheel controllers active  
- Replay reproduces motion  
- No ground-truth frames introduced  
- TF ownership behaves correctly  

## Current System State

- **Infrastructure:** Stable and Locked  
- **Command trajectory:** Recorded and Verified  
- **Reproducibility:** Confirmed  
- **Estimator stack:** Ready for baseline evaluation  

## Next Phase

Proceed to controlled baseline recordings:

- Wheel-only estimator run  
- EKF baseline run  

### Offline Metric Computation

- Drift proxy  
- ωz tracking  
- Smoothness metrics  
- Phase-wise evaluation  

