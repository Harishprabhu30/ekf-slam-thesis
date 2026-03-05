# Baseline Ground Truth Analysis and Interpretation

## 1. Relationship to Previous Baseline Analysis

Prior to introducing simulator ground truth, an internal baseline comparison was conducted between **wheel odometry** and **EKF-based odometry**. That evaluation focused on the **relative behaviour** of the estimators, including:

- Trajectory smoothness  
- Rotational jerk reduction  
- Short-term motion consistency

However, the previous evaluation did **not include a true reference trajectory**, so it could not determine whether either estimator was closer to the actual robot motion. The earlier results primarily reflected **internal estimator behaviour** rather than absolute accuracy.

The introduction of **simulator-derived ground truth** enables **direct measurement of trajectory error**, allowing evaluation of:

- Absolute Trajectory Error (ATE)  
- Yaw error relative to the true robot heading  
- Phase-wise drift accumulation  
- Influence of motion patterns on estimator performance

The ground truth evaluation therefore **complements the previous baseline study** by providing an **absolute accuracy benchmark**.

## 2. Global Performance Comparison

Global trajectory accuracy was evaluated using **Absolute Trajectory Error (ATE)** over the complete motion sequence (excluding the stop phase).

| Estimator        | ATE RMSE | Mean Yaw Error |
|-----------------|-----------|----------------|
| Wheel Odometry   | 0.63 m    | 0.148 rad      |
| EKF Fusion       | 0.69 m    | 0.131 rad      |

The results show that **EKF fusion slightly reduces yaw error** but does not reduce overall positional error.

This behaviour is consistent with the theoretical limitations of **relative sensor fusion**. The EKF integrates wheel encoder measurements and IMU angular velocity, both providing relative motion estimates. While fusion reduces short-term noise and improves rotational smoothness, it cannot eliminate accumulated bias without an **external reference**.

Consequently, the EKF estimator produces a trajectory that is **slightly smoother** but not significantly closer to the true path.


## 3. Phase-Wise Behaviour

To better understand estimator performance, the trajectory was divided into phases corresponding to different motion patterns:

```
0 – Stop
1 – Straight motion
2 – Square trajectory
3 – Clockwise rotation
4 – Circular arc
```

Phase-wise metrics reveal how drift develops under different motion conditions.

### 3.1 Straight Motion

During straight motion, the robot moves with **minimal curvature**.

**Observed behaviour:**

- Both estimators maintain relatively small yaw errors  
- Positional drift remains limited  
- Wheel odometry slightly outperforms EKF in both ATE and yaw error

This occurs because straight motion does not amplify heading bias. Small angular errors have **minimal impact** on the resulting position estimate.

### 3.2 Square Trajectory

The square trajectory introduces **periodic 90° turns**.

**Observations:**

- Positional error remains relatively moderate  
- Heading error increases at the corners due to discrete rotations  
- EKF fusion does not significantly outperform wheel odometry

Because turns occur intermittently, yaw bias has limited time to accumulate into translational error.

### 3.3 Pure Rotation

During the clockwise rotation phase, the robot **rotates in place** without translational motion.

**Results show:**

- EKF reduces mean yaw error compared to wheel odometry  
- Positional ATE increases slightly

This occurs because rotational motion **magnifies coordinate frame misalignment**. Even small orientation bias causes apparent translation when positions are expressed in a global frame.

### 3.4 Circular Arc

The arc trajectory produces the **largest trajectory error**.

**Key observations:**

- ATE increases significantly during this phase  
- Yaw error grows continuously during the arc motion  
- EKF reduces yaw magnitude slightly but does not prevent positional drift

The reason is that **curved motion converts heading bias into lateral displacement**. The relationship between rotational bias and translational error can be approximated as:

\[
e_{\text{position}} \approx R \cdot \theta_{\text{bias}}
\]

Where:

- \(R\) = curvature radius  
- \(\theta_{\text{bias}}\) = accumulated yaw error

Even small angular deviations therefore lead to **large positional errors** during curved trajectories.

## 4. Error Evolution Over Time

Time-based plots illustrate how trajectory error develops.

**ATE versus time plot shows:**

- Low error growth during straight segments  
- Step-like increases during rotation phases  
- Rapid error accumulation during arc motion

These observations confirm that **trajectory curvature strongly influences drift growth**.

The **yaw error plot** demonstrates a gradual increase in heading deviation during the arc segment, directly corresponding to the positional drift observed in the ATE plot.

## 5. EKF vs Wheel Odometry Interpretation

The EKF estimator fuses wheel encoder measurements with IMU angular velocity to improve motion estimation, primarily affecting **short-term rotational stability**.

**Evaluation results show:**

- EKF reduces instantaneous yaw noise  
- EKF produces smoother orientation estimates  
- EKF does **not eliminate systematic bias**

Since both wheel encoders and IMU provide relative measurements, the estimator cannot correct accumulated drift without **external reference observations**.  
This explains why the EKF trajectory remains similar to the wheel odometry trajectory when compared to ground truth.

## 6. Key Findings

The baseline evaluation reveals several important characteristics:

- Rotational bias is the dominant contributor to trajectory drift  
- Curved motion significantly amplifies positional error  
- Sensor fusion alone does not eliminate accumulated drift  
- Relative sensors cannot provide absolute position correction  
- Arc trajectories are the most challenging motion pattern for odometry-based estimation

These findings are consistent with established SLAM literature.

## 7. Implications for Future Experiments

The limitations observed motivate the integration of **exteroceptive sensors** capable of providing absolute spatial constraints.

Future experiments will incorporate **visual SLAM methods** to reduce accumulated drift.  
The same evaluation pipeline will be reused to compare:

- Wheel odometry baseline  
- EKF fusion baseline  
- Visual SLAM estimators

This will allow **direct measurement of drift reduction** and trajectory accuracy improvements.

## 8. Summary

The ground truth evaluation confirms that:

- EKF fusion improves **short-term rotational stability**  
- EKF does **not significantly reduce long-term positional drift** with only relative sensors  
- Trajectory curvature plays a **critical role in drift accumulation**, with circular motion producing the largest deviations from ground truth

These results establish a **clear baseline** for evaluating more advanced SLAM techniques in subsequent research.











