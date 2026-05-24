vgtu@vgtu-B450M-DS3H:~/Downloads/Harish_Thesis/ros2_ws$ ros2 topic info -v /cmd_vel
Type: geometry_msgs/msg/Twist

Publisher count: 1

Node name: trajectory_player
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: PUBLISHER
GID: 01.0f.66.fc.30.35.9d.21.00.00.00.00.00.00.11.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: _Graphs_ros2_cmd_vel_converter_ros2_subscribe_twist
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: SUBSCRIPTION
GID: 01.0f.66.fc.26.30.70.23.04.00.09.00.00.00.03.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

# How to Run — Warehouse + Trajectory Validation Baseline

This section documents the exact procedure to reproduce the current
warehouse + wheel-odometry validation setup.

No ground truth is used at any stage.

---

# 1️⃣ Prerequisites

- Isaac Sim 5.0
- ROS 2 sourced
- `ekf_slam_sim` built and installed
- Warehouse Physics World loaded
- Action Graph wired:

  ROS2 Subscribe Twist  
  → Differential Controller  
  → Articulation Controller  
  (Exec connected from Playback Tick)

- Encoder odometry node available

---

# 2️⃣ Build & Source

From workspace root:

```bash
colcon build --packages-select ekf_slam_sim
source install/setup.bash

# 3️⃣ Launch Order

## **Step A — Start Isaac Sim**

- **Load** the *Warehouse Physics World*

- **Ensure the following:**
  - ✅ Physics enabled  
  - ✅ Collisions working  
  - ✅ Floor material applied  

- ▶️ Press **Play**

## **Step B — Start Encoder Odometry Node**

Run:

```bash
ros2 run ekf_slam_sim encoder_odom_publisher

**Verify Topics**

```bash
ros2 topic list

**Expected topics include:
```
/joint_states
/odom
/tf
/cmd_vel (after trajectory node starts)

## **Step C — Run Trajectory Player**

### **Straight 10 m**

```bash
ros2 run ekf_slam_sim trajectory_player --ros-args -p profile:=straight10

```bash
ros2 run ekf_slam_sim trajectory_player --ros-args -p profile:=spin2x

```bash
ros2 run ekf_slam_sim trajectory_player --ros-args -p profile:=square

# **4. Topic Verification**

## **Check Command Flow**

```bash
ros2 topic info /cmd_vel

**Expected:**

- Publishers: 1

- Subscribers: 1

# **5. Rosbag Recording (Wheel-Only Baseline)**

```bash
ros2 bag record \
/clock \
/cmd_vel \
/joint_states \
/imu \
/odom \
/tf

Run full trajectory.
Stop bag after motion completes.

This bag becomes: Wheel-only baseline dataset

# **6️⃣ Sanity Checks**

## **A. Idle Stability**

When the robot is stationary:

- `/odom` drift ≈ numerical noise scale (~1e-5 m)

---

## **B. Collision Check**

Drive the robot into a wall:

- Robot must stop  
- No penetration through obstacles  

---

## **C. Repeatability**

Re-run the same trajectory:

- Segment timing remains consistent  
- Similar `/odom` traces observed  

# **7️⃣ Important Experimental Notes**

- Motion is **open-loop time-based**
- No simulator pose is read
- No world frame is used

---

## **Evaluation Uses Only:**

- `/odom`
- *(later)* `/odometry/filtered`

---

## **This Guarantees:**

- No ground-truth leakage  
- Reproducible control inputs  
- Fair comparison between estimation methods  

