#!/usr/bin/env bash
set -euo pipefail

RUN_ID="${1:-}"
MODE="${2:-}"
TRAJ_ID="${3:-traj_cmd_clean_v4_cam}"

if [[ -z "$RUN_ID" || -z "$MODE" ]]; then
  echo "Usage: $0 <run_id> <mode> [traj_id]"
  echo "Modes: wheel | ekf | gt | visual"
  exit 1
fi

# Reset ROS 2 Daemon to clear any previous !rclpy.ok() crashes
ros2 daemon stop && ros2 daemon start

OUT_DIR="bags/experiments/${RUN_ID}"
mkdir -p "${OUT_DIR}"

DATE_LOCAL="$(date --iso-8601=seconds)"
MANIFEST="${OUT_DIR}/manifest.yaml"

# Check for nested or flat bag folder structure
if [[ -d "bags/trajectories/${TRAJ_ID}/${TRAJ_ID}" ]]; then
  MASTER_BAG="bags/trajectories/${TRAJ_ID}/${TRAJ_ID}"
else
  MASTER_BAG="bags/trajectories/${TRAJ_ID}"
fi

# -------- Select topics by mode --------
case "$MODE" in
  wheel)  TOPICS=(/odom /tf /traj_phase) ;;
  ekf)    TOPICS=(/joint_states /imu_raw /odom /odometry/filtered /tf /traj_phase) ;;
  gt)     TOPICS=(/gt/odom /traj_phase) ;;
  visual) TOPICS=(/camera/left/image_raw /camera/left/camera_info /camera/right/image_raw /camera/right/camera_info /imu_raw /odom /tf /traj_phase) ;;
  *) echo "Unknown mode: $MODE"; exit 1 ;;
esac

# -------- Write manifest --------
cat > "${MANIFEST}" <<EOF
run_type: experiment
run_id: ${RUN_ID}
trajectory_id: ${TRAJ_ID}
estimator: ${MODE}
ros:
  use_sim_time: true
date_time_local: "${DATE_LOCAL}"
EOF

# -------- Start recorder --------
echo "[run_experiment] Cleaning old data and starting recorder..."
# Manually delete old folder because Humble record doesn't support --force-overwrite
rm -rf "${OUT_DIR}/${RUN_ID}"

ros2 bag record "${TOPICS[@]}" -o "${OUT_DIR}/${RUN_ID}" &
REC_PID=$!

# Give recorder 2 seconds to initialize subscriptions
sleep 2

# Cleanup function for manual script interruption
cleanup() {
  echo "[run_experiment] Interrupt received, killing recorder..."
  kill -INT "${REC_PID}" 2>/dev/null || true
  exit
}
trap cleanup SIGINT SIGTERM

# -------- Replay master trajectory --------
# Define only the topics needed for EKF to stay light
EKF_REPLAY_TOPICS="/joint_states /imu_raw /odom /tf /gt/odom /traj_phase"


#echo "[run_experiment] Replaying trajectory: ${TRAJ_ID}"
# Running in foreground (no '&') ensures the script waits for the bag to finish
# Added --disable-keyboard-controls to clean up the 'stdin' error logs
#ros2 bag "${play_MASTER}" --BAG --clock-disable-keyboard

# Replay ONLY these topics

echo "[run_experiment] Replaying sensor data only..."
#ros2 bag play "${MASTER_BAG}" --clock --topics ${EKF_REPLAY_TOPICS} --disable-keyboard-controls
ros2 bag play "${MASTER_BAG}" --disable-keyboard-controls


# -------- Finish and Save --------
echo "[run_experiment] Replay finished. Stopping recorder..."
# Sending SIGINT to the recorder allows it to close the database file properly
kill -INT "${REC_PID}"
wait "${REC_PID}" || true

echo "[run_experiment] SUCCESS: Experiment bag saved in ${OUT_DIR}/${RUN_ID}"

