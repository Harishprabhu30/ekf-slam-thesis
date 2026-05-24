#!/usr/bin/env bash
set -euo pipefail

RUN_ID="${1:-}"
MODE="${2:-}"
TRAJ_ID="${3:-traj_cmd_v6_bright}"
EXPERIMENT_GROUP="${4:-}"

if [[ -z "$RUN_ID" || -z "$MODE" ]]; then
  echo "Usage:"
  echo "  $0 <run_id> <mode> [traj_id] [experiment_group]"
  echo ""
  echo "Modes:"
  echo "  wheel | ekf | gt | visual | orbslam3"
  echo ""
  echo "Examples:"
  echo "  $0 run_orbslam3_v6_bright orbslam3 traj_cmd_v6_bright"
  echo "  $0 run_ekf_v6_dim_t02 ekf traj_cmd_v6_dim_t02"
  echo "  $0 run_orbslam3_default_v6_bright_t01 orbslam3 traj_cmd_v6_bright orbslam3_ablation/default"
  exit 1
fi

# ------------------------------------------------------------
# Infer lighting condition from trajectory/run name
# ------------------------------------------------------------
infer_lighting() {
  local text="$1"

  if [[ "$text" == *"lowlight"* ]]; then
    echo "lowlight"
  elif [[ "$text" == *"bright"* ]]; then
    echo "bright"
  elif [[ "$text" == *"dim"* ]]; then
    echo "dim"
  else
    echo "unknown"
  fi
}

LIGHTING="$(infer_lighting "${TRAJ_ID}")"

if [[ "${LIGHTING}" == "unknown" ]]; then
  LIGHTING="$(infer_lighting "${RUN_ID}")"
fi

if [[ "${LIGHTING}" == "unknown" ]]; then
  echo "[run_experiment_v2] ERROR: could not infer lighting from TRAJ_ID or RUN_ID."
  echo "TRAJ_ID=${TRAJ_ID}"
  echo "RUN_ID=${RUN_ID}"
  echo "Expected one of: bright | dim | lowlight"
  exit 1
fi

# ------------------------------------------------------------
# Map estimator mode to folder name
# ------------------------------------------------------------
case "$MODE" in
  wheel)
    MODE_DIR="wheel_only"
    TOPICS=(/gt/odom /odom /tf /traj_phase)
    ;;
  ekf)
    MODE_DIR="ekf"
    TOPICS=(/gt/odom /joint_states /imu_raw /odom /odometry/filtered /tf /traj_phase)
    ;;
  gt)
    MODE_DIR="gt"
    TOPICS=(/gt/odom /traj_phase)
    ;;
  visual)
    MODE_DIR="visual"
    TOPICS=(/gt/odom /camera/left/image_raw /camera/left/camera_info /camera/right/image_raw /camera/right/camera_info /imu_raw /odom /tf /traj_phase)
    ;;
  orbslam3)
    MODE_DIR="orbslam3"
    TOPICS=(/orbslam3/pose /odom /gt/odom /traj_phase /tf)
    ;;
  *)
    echo "[run_experiment_v2] ERROR: unknown mode: ${MODE}"
    echo "Allowed: wheel | ekf | gt | visual | orbslam3"
    exit 1
    ;;
esac

# ------------------------------------------------------------
# Resolve master bag path
# New preferred structure:
#   bags/trajectories/<lighting>/<traj_id>/<traj_id>
#
# Old fallback structure:
#   bags/trajectories/<traj_id>/<traj_id>
#   bags/trajectories/<traj_id>
# ------------------------------------------------------------
if [[ -d "bags/trajectories/${LIGHTING}/${TRAJ_ID}/${TRAJ_ID}" ]]; then
  MASTER_BAG="bags/trajectories/${LIGHTING}/${TRAJ_ID}/${TRAJ_ID}"
elif [[ -d "bags/trajectories/${LIGHTING}/${TRAJ_ID}" ]]; then
  MASTER_BAG="bags/trajectories/${LIGHTING}/${TRAJ_ID}"
elif [[ -d "bags/trajectories/${TRAJ_ID}/${TRAJ_ID}" ]]; then
  MASTER_BAG="bags/trajectories/${TRAJ_ID}/${TRAJ_ID}"
elif [[ -d "bags/trajectories/${TRAJ_ID}" ]]; then
  MASTER_BAG="bags/trajectories/${TRAJ_ID}"
else
  echo "[run_experiment_v2] ERROR: master bag not found."
  echo "Tried:"
  echo "  bags/trajectories/${LIGHTING}/${TRAJ_ID}/${TRAJ_ID}"
  echo "  bags/trajectories/${LIGHTING}/${TRAJ_ID}"
  echo "  bags/trajectories/${TRAJ_ID}/${TRAJ_ID}"
  echo "  bags/trajectories/${TRAJ_ID}"
  exit 1
fi

# ------------------------------------------------------------
# Output folder
#
# Normal:
#   bags/experiments/<mode_dir>/<lighting>/<run_id>/<run_id>
#
# Optional ablation/custom group:
#   bags/experiments/<experiment_group>/<lighting>/<run_id>/<run_id>
# Example:
#   experiment_group=orbslam3_ablation/default
# ------------------------------------------------------------
if [[ -n "${EXPERIMENT_GROUP}" ]]; then
  OUT_DIR="bags/experiments/${EXPERIMENT_GROUP}/${LIGHTING}/${RUN_ID}"
else
  OUT_DIR="bags/experiments/${MODE_DIR}/${LIGHTING}/${RUN_ID}"
fi

OUT_BAG="${OUT_DIR}/${RUN_ID}"
MANIFEST="${OUT_DIR}/manifest.yaml"

mkdir -p "${OUT_DIR}"

DATE_LOCAL="$(date --iso-8601=seconds)"

# ------------------------------------------------------------
# Reset ROS 2 daemon
# ------------------------------------------------------------
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

# ------------------------------------------------------------
# Write manifest
# ------------------------------------------------------------
cat > "${MANIFEST}" <<EOF
run_type: experiment
run_id: ${RUN_ID}
trajectory_id: ${TRAJ_ID}
lighting_condition: ${LIGHTING}
estimator: ${MODE}
mode_dir: ${MODE_DIR}
experiment_group: ${EXPERIMENT_GROUP}
master_bag: ${MASTER_BAG}
output_bag: ${OUT_BAG}
ros:
  use_sim_time: true
date_time_local: "${DATE_LOCAL}"
topics_recorded:
$(printf "  - %s\n" "${TOPICS[@]}")
EOF

# ------------------------------------------------------------
# Remove previous output bag
# ------------------------------------------------------------
rm -rf "${OUT_BAG}"

REC_PID=""

cleanup() {
  echo "[run_experiment_v2] Interrupt received. Cleaning up..."

  if [[ -n "${REC_PID}" ]]; then
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi

  exit 1
}

trap cleanup SIGINT SIGTERM

echo "============================================================"
echo "[run_experiment_v2] Experiment"
echo "============================================================"
echo "Mode             : ${MODE}"
echo "Mode folder      : ${MODE_DIR}"
echo "Lighting         : ${LIGHTING}"
echo "Trajectory       : ${TRAJ_ID}"
echo "Master bag       : ${MASTER_BAG}"
echo "Experiment group : ${EXPERIMENT_GROUP}"
echo "Output bag       : ${OUT_BAG}"
echo "Manifest         : ${MANIFEST}"
echo "============================================================"

echo "[run_experiment_v2] Starting recorder..."

ros2 bag record "${TOPICS[@]}" -o "${OUT_BAG}" &
REC_PID=$!

# Give recorder time to subscribe
sleep 2

if [[ "$MODE" == "orbslam3" ]]; then
  echo "[run_experiment_v2] Waiting for /orbslam3/pose topic..."
  until ros2 topic list | grep -q "/orbslam3/pose"; do
    sleep 1
  done
  echo "[run_experiment_v2] ORB-SLAM3 detected."
fi

echo "[run_experiment_v2] Replaying master dataset..."
ros2 bag play "${MASTER_BAG}" --disable-keyboard-controls

echo "[run_experiment_v2] Replay finished. Stopping recorder..."
kill -INT "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" || true

echo "============================================================"
echo "[run_experiment_v2] SUCCESS"
echo "Saved experiment bag:"
echo "  ${OUT_BAG}"
echo "============================================================"
