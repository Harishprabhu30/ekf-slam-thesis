#!/usr/bin/env bash
set -euo pipefail

RUN_ID="${1:-}"
MODE="${2:-}"
TRAJ_ID="${3:-traj_cmd_clean_v2_cam}"

if [[ -z "$RUN_ID" || -z "$MODE" ]]; then
  echo "Usage: $0 <run_id> <mode> [traj_id]"
  echo "Modes: wheel | ekf | gt | visual"
  echo "Example: $0 run_wheel_only_v3_cam wheel traj_cmd_clean_v2_cam"
  exit 1
fi

TRAJ_DIR="bags/trajectories/${TRAJ_ID}/${TRAJ_ID}"

if [[ ! -d "bags/trajectories/${TRAJ_ID}" ]]; then
  echo "[run_experiment] ERROR: trajectory directory not found: bags/trajectories/${TRAJ_ID}"
  exit 1
fi

OUT_DIR="bags/experiments/${RUN_ID}"
mkdir -p "${OUT_DIR}"

DATE_LOCAL="$(date --iso-8601=seconds)"

MANIFEST="${OUT_DIR}/manifest.yaml"

# -------- Topic selection by mode --------
case "$MODE" in
  wheel)
    TOPICS=(
      /odom
      /traj_phase
    )
    ;;
  ekf)
    TOPICS=(
      /odometry/filtered
      /odom
      /imu_raw
      /traj_phase
    )
    ;;
  gt)
    TOPICS=(
      /gt/odom
      /traj_phase
    )
    ;;
  visual)
    TOPICS=(
      /camera/left/image_raw
      /camera/left/camera_info
      /camera/right/image_raw
      /camera/right/camera_info
      /imu_raw
      /odom
      /traj_phase
    )
    ;;
  *)
    echo "Unknown mode: $MODE"
    exit 1
    ;;
esac

# -------- Write manifest --------
cat > "${MANIFEST}" <<EOF
run_type: experiment
run_id: ${RUN_ID}
trajectory_id: ${TRAJ_ID}
env_id: simple_room_v1
camera_config: stereo_640x480
isaac_sim: "5.0"
estimator: ${MODE}
ros:
  use_sim_time: true
topics_recorded:
EOF

for topic in "${TOPICS[@]}"; do
  echo "  - ${topic}" >> "${MANIFEST}"
done

cat >> "${MANIFEST}" <<EOF
clock_policy: "Isaac only; /clock not recorded; no --clock on replay"
date_time_local: "${DATE_LOCAL}"
notes: ""
EOF

echo "[run_experiment] Manifest written: ${MANIFEST}"
echo "[run_experiment] Mode: ${MODE}"
echo "[run_experiment] Output directory: ${OUT_DIR}"

# -------- Start recording --------
echo "[run_experiment] Starting recorder..."
ros2 bag record "${TOPICS[@]}" -o "${OUT_DIR}/${RUN_ID}" &
REC_PID=$!

cleanup() {
  echo "[run_experiment] Stopping recorder..."
  kill "${REC_PID}" 2>/dev/null || true
}
trap cleanup EXIT

# -------- Wait for simulation clock --------
echo "[run_experiment] Waiting for /clock..."
ros2 topic echo /clock --once >/dev/null 2>/dev/null

# -------- Wait for required topics --------
echo "[run_experiment] Waiting for required topics..."
for topic in "${TOPICS[@]}"; do
  echo "  checking $topic"
  ros2 topic echo "$topic" --once >/dev/null 2>/dev/null || true
done

# -------- Verify sim time advancing --------
echo "[run_experiment] Verifying sim time advances..."
T1="$(ros2 topic echo /clock --once 2>/dev/null | awk '/sec:/{print $2; exit}')"
T2="$(ros2 topic echo /clock --once 2>/dev/null | awk '/sec:/{print $2; exit}')"

if [[ "${T1}" == "${T2}" ]]; then
  echo "[run_experiment] WARNING: /clock did not advance. If sim is paused, press Play."
fi

# -------- Replay trajectory --------
echo "[run_experiment] Replaying trajectory: ${TRAJ_ID}"

ros2 bag play "bags/trajectories/${TRAJ_ID}/${TRAJ_ID}" \
  || ros2 bag play "bags/trajectories/${TRAJ_ID}"

echo "[run_experiment] Trajectory replay finished."
echo "[run_experiment] Done."
