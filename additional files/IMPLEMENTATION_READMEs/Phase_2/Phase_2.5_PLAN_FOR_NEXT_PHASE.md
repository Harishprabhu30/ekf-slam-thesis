## What’s Next After Bag Recording

**Next milestone (immediately after you have the two bags):**  
Baseline evaluation + plots (Phase 4 in your pipeline, but only for wheel vs EKF first)

You will compute and plot:

| Metric              | From Which Topics                                                                 | Why It Matters                                                                 |
|--------------------|---------------------------------------------------------------------------------|-------------------------------------------------------------------------------|
| yaw(t) comparison   | `/odom` vs `/odometry/filtered` (or TF)                                         | Shows EKF yaw stability improvement                                           |
| ωz(t) comparison    | `/odom.twist.angular.z` vs `/imu.angular_velocity.z` vs EKF `/odometry/filtered.twist.angular.z` | Shows how EKF tracks IMU while smoothing wheel noise                         |
| drift proxy         | End pose difference after same replay                                            | Shows cumulative integration differences                                       |
| smoothness          | Yaw derivative / jerk proxy                                                      | Shows EKF reduces jitter                                                      |

> That becomes your baseline results chapter content.

Only after you can show “wheel vs EKF baseline” clearly, you move to visual localization.

## Do We Go to LiDAR and Camera Next?

**Yes — but in the right order.**

**Priority order moving forward:**

1. Record bags (wheel-only and EKF) using the same reference trajectory  
2. Generate baseline plots + short analysis (this is thesis evidence)  
3. Camera pipeline bring-up (ORB-SLAM / cuVSLAM needs camera first)  
4. Then optional:  
   - LiDAR (if you plan to use a LiDAR SLAM baseline or to support loop closure / robustness comparisons)  

> For your locked plan, visual SLAM is Phase 3, so **camera comes before LiDAR**.  
> The next “engineering” work after bagging is usually **camera setup + validation**, not more EKF.

## What About “Start with robot_localization Package for EKF”?

- You already did. That’s what `ekf_filter_node` is.  
- The remaining EKF work is **not** “making EKF work”—it’s:  
  - Light tuning  
  - Documenting tuning effect (`Q` vs `R`)  
  - Showing baseline improvements with plots
  
  ## Concrete Forward Plan (No Extra Theory)

### Step 1 — Bags (You’re Ready)

- `bags/reference/ref_traj`  
- `bags/ekf_runs/wheel_only`  
- `bags/ekf_runs/ekf`  

### Step 2 — Plot Script (Next Deliverable)

Create a Python script (offline) that reads rosbag and outputs:

- `yaw(t)` wheel vs EKF  
- `ωz(t)` IMU vs wheel vs EKF  
- End pose summary table  

> (We’ll do this right after you record the bags.)

### Step 3 — Camera Validation Checklist (Before SLAM)

Before ORB/cuVSLAM, camera must pass:

- Stable FPS (`ros2 topic hz /rgb/image_raw`)  
- Valid timestamps (monotonic, sim time)  
- `camera_info` published  
- Calibration matrices non-zero  
- Correct `frame_id` and TF exists under `chassis_link`  

### Step 4 — Visual Localization Experiments

- ORB-SLAM launch  
- cuVSLAM launch  

> Both replay the same `ref_traj` bag and record comparable bags.


