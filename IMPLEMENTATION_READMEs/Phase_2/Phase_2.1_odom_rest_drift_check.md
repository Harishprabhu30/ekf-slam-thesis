# Odometry Noise Analysis in ROS 2 / Isaac Sim

This document explains the behavior of odometry readings when the robot is stationary in simulation. It provides an interpretation of small numerical fluctuations and guidelines for monitoring real movement.

---

## 1. Observed Data

Using the following command to monitor `/odom`:

```bash
ros2 topic echo /odom | awk '/position:/ {getline; x=$2; getline; y=$2; print "x="x,"y="y}'

Sample output while the robot was stationary:

x=-2.7874e-05 y=-1.8281e-09
x=-4.8779e-05 y=-3.8848e-09
x=-5.5748e-05 y=-4.5704e-09
x=-6.9685e-05 y=-6.8555e-09
x=-0.0001045 y=-1.2111e-08
...

Once the robot started moving, values jumped to the 0.01 m range.

## 2. Interpretation

**Small fluctuations at rest:**

- **X-values:** on the order of `1e-5` to `1e-4` m (~0.01 mm to 0.1 mm)  
- **Y-values:** even smaller, `1e-9` to `1e-8` m  
- These represent **numerical noise** in the simulator, not real motion.

**Causes of noise:**

- **Physics solver:** Tiny adjustments due to collision/contact resolution.  
- **Wheel odometry integration:** Even zero velocities accumulate floating-point errors.  
- **Simulation settling:** Initial contact corrections when the robot is placed.

**Movement detection:**

- Significant motion (> 1 mm) is clearly visible once the robot drives.  
- Values below 0.001 m can be considered **noise**.

---

## 3. Guidelines for Monitoring Odometry

- **Ignore sub-millimeter fluctuations** while stationary.  
- **Optional threshold filter** to only monitor meaningful movement:

```bash
ros2 topic echo /odom | awk '/position:/ {getline; x=$2; getline; y=$2; if (sqrt(x*x+y*y)>0.001) print "x="x,"y="y}'

- This prints positions only when displacement exceeds 1 mm.

- For yaw (orientation) monitoring without an IMU:

- Without wheel slippage, small changes are negligible.

- Python scripts can convert quaternion → yaw if required, but x/y are sufficient for stationary drift checks.

**Summary**

- Tiny x/y fluctuations while stationary are normal odometry noise in Isaac Sim. 

- They provide a baseline to understand the minimum detectable movement.

- Real drift or commanded motion is easily distinguishable once it exceeds ~1 mm.
