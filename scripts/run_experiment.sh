#!/usr/bin/env bash
set -euo pipefail

RUN_ID="${1:-}"
MODE="${2:-}"
TRAJ_ID="${3:-traj_cmd_clean_v5_cam}"

if [[ -z "$RUN_ID" || -z "$MODE" ]]; then
  echo "Usage: $0 <run_id> <mode> [traj_id]"
  echo "Modes: wheel | ekf | gt | visual | orbslam3 | cuvslam"
  exit 1
fi

# Reset ROS 2 daemon
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

OUT_DIR="bags/experiments/${RUN_ID}"
mkdir -p "${OUT_DIR}"

DATE_LOCAL="$(date --iso-8601=seconds)"
MANIFEST="${OUT_DIR}/manifest.yaml"

# Resolve master bag path
if [[ -d "bags/trajectories/${TRAJ_ID}/${TRAJ_ID}" ]]; then
  MASTER_BAG="bags/trajectories/${TRAJ_ID}/${TRAJ_ID}"
elif [[ -d "bags/trajectories/${TRAJ_ID}" ]]; then
  MASTER_BAG="bags/trajectories/${TRAJ_ID}"
else
  echo "[run_experiment] ERROR: master bag not found for trajectory: ${TRAJ_ID}"
  exit 1
fi

# Topics to record per mode
case "$MODE" in
  wheel)
    TOPICS=(/gt/odom /odom /tf /traj_phase)
    ;;
  ekf)
    TOPICS=(/gt/odom /joint_states /imu_raw /odom /odometry/filtered /tf /traj_phase)
    ;;
  gt)
    TOPICS=(/gt/odom /traj_phase)
    ;;
  visual)
    TOPICS=(/gt/odom /camera/left/image_raw /camera/left/camera_info /camera/right/image_raw /camera/right/camera_info /imu_raw /odom /tf /traj_phase)
    ;;
  orbslam3)
    TOPICS=(/orbslam3/pose /odom /gt/odom /traj_phase /tf)
    ;;
  cuvslam)
    TOPICS=(/visual_slam/tracking/odometry /visual_slam/tracking/slam_path /gt/odom /traj_phase /tf)
    ;;
  *)
    echo "[run_experiment] ERROR: unknown mode: $MODE"
    exit 1
    ;;
esac

# Write manifest
cat > "${MANIFEST}" <<EOF
run_type: experiment
run_id: ${RUN_ID}
trajectory_id: ${TRAJ_ID}
estimator: ${MODE}
ros:
  use_sim_time: true
date_time_local: "${DATE_LOCAL}"
EOF

# Remove previous output bag
rm -rf "${OUT_DIR}/${RUN_ID}"

# Cleanup on interrupt
cleanup() {
  echo "[run_experiment] Interrupt received. Stopping recorder..."
  kill -INT "${REC_PID}" 2>/dev/null || true
  wait "${REC_PID}" 2>/dev/null || true
  exit 1
}
trap cleanup SIGINT SIGTERM

echo "[run_experiment] Mode        : ${MODE}"
echo "[run_experiment] Trajectory  : ${TRAJ_ID}"
echo "[run_experiment] Master bag  : ${MASTER_BAG}"
echo "[run_experiment] Output bag  : ${OUT_DIR}/${RUN_ID}"
echo "[run_experiment] Starting recorder..."

ros2 bag record "${TOPICS[@]}" -o "${OUT_DIR}/${RUN_ID}" &
REC_PID=$!

# Give recorder time to subscribe
sleep 2

# Special handling for ORB-SLAM3 (wait for node to publish)
if [[ "$MODE" == "orbslam3" ]]; then
  echo "[run_experiment] Waiting for /orbslam3/pose topic..."
  until ros2 topic list | grep -q "/orbslam3/pose"; do
    sleep 1
  done
  echo "[run_experiment] ORB-SLAM3 detected."
fi

# Special handling for cuVSLAM (wait for node to publish)
if [[ "$MODE" == "cuvslam" ]]; then
  echo "[run_experiment] Waiting for /visual_slam/tracking/odometry topic..."
  until ros2 topic list | grep -q "/visual_slam/tracking/odometry"; do
    sleep 1
  done
  echo "[run_experiment] cuVSLAM detected."
fi

echo "[run_experiment] Replaying master dataset..."
ros2 bag play "${MASTER_BAG}" --disable-keyboard-controls

echo "[run_experiment] Replay finished. Stopping recorder..."
kill -INT "${REC_PID}"
wait "${REC_PID}" || true

echo "[run_experiment] SUCCESS: experiment bag saved to ${OUT_DIR}/${RUN_ID}"
