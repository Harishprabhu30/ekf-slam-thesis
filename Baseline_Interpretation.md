# Baseline Objective

The goal of the baseline experiment was:

- To quantify the difference between wheel-only odometry and EKF fusion (wheel + IMU) under identical simulation and trajectory conditions.

**Key constraints:**

- No world frame  
- No ground-truth correction  
- No vision / LiDAR  
- Identical trajectory  
- Identical physics  
- Only estimator stack changes

This ensures a controlled estimator-only comparison.

# The Baseline Shows (Quantitatively)

## A) Translational Drift (Position)

**Overall drift:**

- Wheel: ~4.85 m  
- EKF: ~4.85 m  
- **Improvement:** ≈ 0%

**Phase-wise drift:**

- Straight: EKF slightly worse  
- Square: EKF slightly worse  
- Arc: EKF slightly worse  
- Turn-in-place: nearly identical

### Interpretation

- The EKF does **not** reduce translational drift.  
- This is not a failure — it is expected.

**Why?**

- Both wheel and IMU are relative sensors.  
- EKF fuses two relative sources.  
- No absolute position measurement exists.  
- Slip bias and integration error accumulate identically.

**Conclusion:**

- The EKF improves state estimation consistency but cannot eliminate accumulated drift without exteroceptive correction.  
- This is theoretically correct and validates experimental integrity.

## B) Yaw Behavior

**Overall Δyaw difference:**

- Slight EKF improvement (~4%)

**Phase 4 (arc motion):**

- EKF produces slightly smaller final yaw deviation

### Interpretation

- The IMU contributes useful angular velocity information, especially in:
  - sustained turns
  - arc motion
  - high angular acceleration phases

**However:**

- Long-term yaw bias still accumulates  
- No global heading correction exists

**Conclusion:**

- EKF stabilizes short-term heading evolution but does not anchor global orientation  
- This is theoretically consistent

## C) Angular Velocity Variance

- **Omega variance:** Wheel ≈ EKF (very similar)  
- **Implication:** Average angular noise magnitude remains comparable. EKF does not drastically change angular velocity amplitude.  
- **Note:** Variance alone does not tell the full story.

---

## D) Jerk Variance (Critical Finding)

**Jerk variance (ALL):**

- Wheel: 0.705  
- EKF: 0.044 → ~94% reduction

**Phase 4 (arc):**

- Wheel: 1.769  
- EKF: 0.074

**Interpretation:**

- **Wheel-only:**  
  - Discrete encoder updates  
  - Sharp transitions at turn start/stop  
  - High-frequency angular acceleration spikes

- **EKF:**  
  - Blends IMU gyro  
  - Applies covariance-weighted smoothing  
  - Produces physically plausible angular acceleration

**Conclusion:**

- EKF significantly improves dynamic rotational smoothness  
- This is a major result

# What the Plots Reveal

## Yaw vs Time

**Observations:**

- Strong overlap overall  
- EKF smoother at transition points  
- No divergence instability  
- Clean wrap-around at ±π

**Conclusion:**

- Filter is stable  
- No divergence  
- Good consistency between sensors

---

## Angular Velocity vs Time

**Observations:**

- Wheel shows spikes at motion transitions  
- EKF smooths rise and decay  
- EKF reduces oscillations during arc

**Conclusion:**

- EKF reduces high-frequency rotational noise  
- Particularly beneficial during mixed motion

---

## Trajectory Overlay

**Observations:**

- Nearly identical large-scale path  
- Slight curvature differences in arc  
- Similar endpoint position

**Conclusion:**

- EKF does not significantly change geometric path  
- Improvement is dynamic, not geometric

# 4. Strengths of the Baseline

- ✔ **Controlled environment**  
  Only estimator changed.

- ✔ **Phase segmentation**  
  Analysis performed per motion primitive:  
  - Straight  
  - Square  
  - Turn  
  - Arc  
  This is much stronger than a single global metric.

- ✔ **Both visual + statistical validation**  
  Plots support numeric metrics.

- ✔ **Theoretical consistency**  
  Results match what control and estimation theory predict.
  
# 5. Weaknesses / Limitations of Current Baseline

This is important — critical self-evaluation.

1) **No ground truth comparison**

- Measured drift relative to start  
- Did **not** measure absolute pose error vs simulated ground truth  
- Thus, only relative consistency is evaluated, not absolute accuracy

2) **No covariance tuning study**

- Current EKF uses fixed Q/R  
- Did not test:  
  - Sensitivity to IMU noise  
  - Sensitivity to wheel noise  
  - Effect of covariance inflation

3) **No slip modeling**

- Wheel slip not explicitly modeled  
- Straight drift remains unchanged

4) **No bias estimation**

- IMU bias not explicitly modeled/adapted  
- Long-term heading drift still accumulates.

# 6. Critical Insight

Your EKF currently acts as:

- A **dynamic stabilizer**, not a drift corrector

This is correct behavior for:

- Wheel + IMU fusion without landmarks

This becomes your justification for:

- Vision SLAM  
- cuVSLAM  
- ORB-SLAM  
- Adaptive landmark covariance

# 7. Future Improvements

Now I define structured next steps.

---

### Step 1 — Covariance Study (Low Effort, High Value)

**Experiment:**

- Increase IMU R  
- Decrease IMU R  
- Increase wheel Q  
- Decrease wheel Q

**Measure:**

- Jerk variance  
- Drift  
- Yaw stability

**Goal:**  
Understand sensitivity and optimal balance. This strengthens your EKF backbone.

---

### Step 2 — Add Bias State

**Extend EKF state vector:**

- Current: `[x, y, θ]`  
- Upgrade to: `[x, y, θ, b_gyro]`

**Action:** Estimate gyro bias online

**Expected result:** Reduced long-term yaw accumulation

---

### Step 3 — Loop Closure Metric

For square trajectory:

- Measure: `||pose_final - pose_start||`  

**Purpose:**  
True loop-closure drift; better than global drift metric

---

### Step 4 — Introduce Exteroceptive SLAM

**Test:**

- ORB-SLAM  
- cuVSLAM

**Compare:**

- Drift reduction  
- Loop closure accuracy  
- Computational cost  
- Stability under arc motion

---

### Step 5 — Hybrid Adaptive EKF (Thesis Inno contri plan)

**Insight from baseline:**

- EKF works well dynamically  
- EKF does not correct drift

**Hybrid approach:**

- EKF backbone for smooth dynamics  
- Vision landmark updates  
- Adaptive covariance scaling  
  - Increase measurement weight during dynamic motion  
  - Reduce during stable motion

# Summary

**Baseline demonstrates:**

- EKF significantly improves rotational smoothness (~94% jerk reduction)  
- EKF does **not** reduce translational drift in the absence of absolute references  
- Behavior matches theoretical expectations of relative sensor fusion

# IMPORTANT OUTPUT FILES

## RAW EXTRACTED DATA

- `analysis/script/results/wheel_baseline.csv`  
- `analysis/script/results/ekf_baseline.csv`

## PHASE-LABELED DATA

- `analysis/script/results/wheel_labeled_final.csv`  
- `analysis/script/results/ekf_labeled_final.csv`

## NORMALIZED (FOR PLOTTING)

- `analysis/script/results/wheel_labeled_final_norm.csv`  
- `analysis/script/results/ekf_labeled_final_norm.csv`

## METRICS TABLES

- `analysis/script/results/metrics_phasewise.csv`  
- `analysis/script/results/improvement_table.csv`

## PLOTS (VISUAL COMPARISON)

- `analysis/script/results/trajectory_compare.png`  
- `analysis/script/results/yaw_compare.png`  
- `analysis/script/results/omega_compare.png`
