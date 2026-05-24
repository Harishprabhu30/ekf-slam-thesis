# Isaac Sim 5.0: Straight 10m Trajectory Issue

## Issue Summary
A deterministic straight 10m trajectory was executed using a custom ROS2 `TrajectoryPlayer` node in Isaac Sim 5.0. Despite correct node implementation and command publishing, the robot only traveled ~2.2m instead of the commanded 10m.

---

## Initial Observations
- `/cmd_vel` published correct 0.7 m/s at 60 Hz
- `/odom` confirmed velocity command was received (~0.7 m/s)
- Segment duration correctly calculated as 14.2857s (10m / 0.7 m/s)
- Robot motion plateaued at ~2.2m despite continued command publishing

---

## Diagnostic Process

### 1. Code Review
The `TrajectoryPlayer` node was verified correct:

- ✅ Correct time conversion (nanoseconds → seconds)
- ✅ Accurate segment duration calculation
- ✅ Proper timer implementation at 60 Hz
- ✅ Clean segment transition logic

### 2. Joint State Analysis
Monitoring `/joint_states` revealed critical evidence:

```bash
ros2 topic echo /joint_states --field velocity

**Key finding:** Joint velocities showed inconsistency:

- **Drive wheels:** ~5.015 rad/s  
- **Caster/swing joints:** ~9.305 rad/s (**86% faster**)
- Pattern persisted regardless of configuration changes.

### 3. Physics Configuration Attempts

| Change Attempted | Result |
|---|---|
| Damping adjustments (20 → 500) | No significant improvement |
| Max joint velocity increase | No effect |
| Friction tuning | Minimal impact |
| Caster/swing joint damping | Pattern persisted |

---

### 4. Breakthrough Discoveries

**Drive Damping Role:**  
Setting drive damping to **0 → no motion**; increasing damping → **increased motion**.  
**Conclusion:** Drive damping acts as **motor gain** in Isaac Sim's velocity control.

**Passive Joint Configuration:**  
Caster and swing joints had **Drive tab values** (should be passive). They were acting as **energy sinks**.

**Time Synchronization:**  
Node was using **wall clock time** while simulation ran at a different rate.  
**14.3s of real time ≠ 14.3s of simulation time.**

---

### Root Causes Identified

- **Time Source Mismatch:** Node not synchronized with simulation time  
- **Insufficient Drive Gain:** Drive damping too low for required torque  
- **Misconfigured Passive Joints:** Caster/swing joints had active control  
- **Energy Dissipation:** Passive joints spinning at high velocities wasted energy  

---

### Solution Implementation

#### 1. Code Fix: Enable Simulation Time
# Critical fix in TrajectoryPlayer __init__
self.set_parameters([rclpy.parameter.Parameter('use_sim_time', 
                                               rclpy.Parameter.Type.BOOL, 
                                               True)])

### 2. Joint Configuration in Isaac Sim

#### Drive Wheels (actively controlled)

**joint_wheel_left/right:**
- **Drive tab damping:** 300–500 *(motor gain)*
- **Revolute joint damping:** 20 *(mechanical)*
- **Max force/torque:** 5000

---

#### Swing Joints (caster swivel — passive)

**joint_swing_left/right:**
- **Drive tab:** **NONE** *(critical fix)*
- **Revolute joint damping:** 100
- **Physics friction:** 0.2

---

#### Caster Wheels (passive)

**joint_caster_left/right:**
- **Drive tab:** **NONE**
- **Revolute joint damping:** 30

### 3. Final Tuning

**Progressive damping adjustment until 10m achieved:**

| Damping | Distance Traveled |
|---|---|
| 200 | ~5m |
| 350 | ~7.5m |
| 500 | 10m ✓ |

---

### Validation

#### Test Command
```bash
ros2 run ekf_slam_sim trajectory_player --ros-args \
  -p profile:=straight10 \
  -p cmd_vel_topic:=/cmd_vel \
  -p v_straight:=0.7 \
  -p straight_distance_m:=10.0

### Results

| Metric | Expected | Actual |
|---|---|---|
| **Linear velocity** | 0.7 m/s | 0.699–0.700 m/s |
| **Segment duration** | 14.2857 s | 14.29 s |
| **Distance traveled** | 10.0 m | 10.0 m |
| **Final position** | 10.0 m | 9.98–10.02 m |

**Verification Commands:**
```bash
# Monitor velocity
ros2 topic echo /odom --field twist.twist.linear.x

# Track position
ros2 topic echo /odom --field pose.pose.position.x

# Verify joint behavior
ros2 topic echo /joint_states --field velocity

# Check simulation time sync
ros2 topic echo /clock


