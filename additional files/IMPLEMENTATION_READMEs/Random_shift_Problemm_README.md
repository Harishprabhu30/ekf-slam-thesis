## Odometry Stability Issue & Resolution (Critical Debugging Phase)

After successfully publishing wheel-encoder odometry and establishing the correct TF chain:

odom → chassis_link → (camera_link, imu_link, lidar_link)


a **non-physical motion artifact** was observed in RViz during straight-line driving.

---

## Observed Problem

### Symptoms
- In RViz, the robot:
  - moved forward smoothly for a short distance
  - then suddenly **shifted backward** onto the same trajectory
  - sometimes shifted sideways by ~1 grid block
  - then continued forward again
- This pattern repeated during long straight drives.
- Isaac Sim motion itself was **smooth and continuous**.

This immediately indicated:
> The issue was **not physics**, **not the controller**, and **not RViz** — but odometry computation.

---

## Initial Hypotheses Considered (and Eliminated)

- ❌ TF dual-parent issue → already fixed earlier
- ❌ RViz fixed-frame misconfiguration → verified correct
- ❌ Sim-time vs wall-time mismatch → verified consistent
- ❌ Sensor noise → not applicable (pure encoder odom)
- ❌ ROS bag replay artifact → disproved (see below)

---

## Critical Debugging Insight: Rosbag Replay

To isolate whether the issue was runtime-related or algorithmic:

1. A **rosbag** was recorded containing:
   - `/joint_states`
   - `/odom`
   - `/tf`

2. The bag was replayed offline.

### Observation
- The **exact same odometry jumps and snap-backs** appeared during rosbag playback.
- No simulator, no keyboard input, no live TF involved.

### Conclusion
> The bug was **purely inside the odometry integration code**.

This ruled out:
- Isaac Sim timing
- Controller behavior
- ROS transport or RViz visualization

This was the decisive moment confirming a **deterministic algorithmic flaw**.

---

## Root Causes Identified (Code-Level)

### 1. Wheel Angle Wrap-Around (Primary Cause)

Wheel joint positions from `/joint_states` are angular values and can cross the ±π boundary.

Original code computed wheel deltas as:
```python
dphi = curr_angle - prev_angle

When a wheel crossed from +π → -π (or vice versa):

dphi ≈ ±2π

This produced a huge fake wheel displacement

Resulting in a sudden odometry “teleport”

**Fix Applied**

Wheel deltas are now normalized:

```python
dphi = atan2(sin(dphi), cos(dphi))

This constrains deltas to:

[-π, π]

and completely eliminates wrap-induced jumps.

**2. Wheel Radius Mismatch (Secondary Cause)**

The simulator wheel radius was 0.14 m, but the odom code initially used a different value.

**Effects:**

Incorrect distance scaling

Exaggerated drift and integration error

**Fix Applied**

Wheel radius moved entirely to parameters

Set to match simulator geometry exactly

No hardcoded real-world geometry remains in code

**3. Integration Accuracy Improvement**

To reduce accumulated integration error:

Midpoint yaw integration was retained:

yaw_mid = yaw + 0.5 * dyaw
This improves numerical stability, especially during curved motion.

**Validation After Fixes**

+ After applying the fixes:
+ 
+ RViz motion became smooth and continuous
+ No backward jumps
+ No sideways block shifts
+ Straight-line motion remained straight
+ Turning behavior was stable
+ Rosbag replay reproduced the correct, smooth odometry
+ 
+ This confirmed:
+ 
+ The odometry implementation is now physically consistent and deterministic.

**Current Locked State**

At this point, the following are verified and locked:

- Wheel-encoder odometry is stable
- TF tree is correct and ROS-compliant
- No simulator ground-truth leakage
- Rosbag replay matches live behavior
- Odometry suitable as EKF baseline input
- No further architectural changes are required at this level.

**Next Steps (Confirmed)**

1. **Wheel Radius Calibration (Validation)**

   Although the radius matches simulator geometry, a grid-based straight-line test will be used to:

   - validate distance accuracy
   - document calibration procedure for thesis completeness

2. **Wheel Separation Calibration**

   - In-place rotation tests
   - Correct yaw scaling
   - Finalize differential-drive geometry

3. **EKF Baseline**

   - Fuse wheel odometry + IMU
   - Establish classical EKF performance baseline
   - Only after this, introduce AI-based conditioning modules

**Key Lesson Learned**

If an error appears both live and in rosbag replay, the bug is always in the algorithm.

This debugging step saved significant time and prevented misattribution of the issue to simulation or visualization tools.

