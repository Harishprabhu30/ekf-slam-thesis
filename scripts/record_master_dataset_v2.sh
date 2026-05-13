#!/usr/bin/env bash
set -euo pipefail

TRAJ_NAME="${1:-}"
LIGHTING_CONDITION="${2:-}"
USD_SCENE="${3:-}"
CMD_SOURCE_BAG="${4:-bags/trajectories/traj_cmd_clean_v5_cam/traj_cmd_clean_v5_cam}"

if [[ -z "$TRAJ_NAME" || -z "$LIGHTING_CONDITION" || -z "$USD_SCENE" ]]; then
  echo "Usage:"
  echo "  $0 <traj_name> <lighting_condition> <usd_scene> [cmd_source_bag]"
  echo ""
  echo "Examples:"
  echo "  $0 traj_cmd_v6_bright bright ekf-slam2-bright.usd"
  echo "  $0 traj_cmd_v6_dim dim ekf-slam2-dim.usd"
  echo "  $0 traj_cmd_v6_lowlight lowlight ekf-slam2-lowlight.usd"
  echo ""
  echo "Optional command source bag default:"
  echo "  bags/trajectories/traj_cmd_clean_v5_cam/traj_cmd_clean_v5_cam"
  exit 1
fi

if [[ ! -d "$CMD_SOURCE_BAG" ]]; then
  echo "[record_v2] ERROR: command source bag not found:"
  echo "  $CMD_SOURCE_BAG"
  exit 1
fi

OUT_DIR="bags/trajectories/${TRAJ_NAME}"
OUT_BAG="${OUT_DIR}/${TRAJ_NAME}"
MANIFEST="${OUT_DIR}/manifest.yaml"

mkdir -p "${OUT_DIR}"

DATE_LOCAL="$(date --iso-8601=seconds)"

TOPICS=(
/cmd_vel
/traj_phase
/camera/left/camera_info
/camera/left/image_raw
/camera/right/camera_info
/camera/right/image_raw
/imu_raw
/laser_scan
/joint_states
/tf
/gt/odom
/clock
)

case "$LIGHTING_CONDITION" in
  bright)
    RECTLIGHT_1=15000
    RECTLIGHT_2=15000
    DOMELIGHT=1000
    ;;
  dim)
    RECTLIGHT_1=3000
    RECTLIGHT_2=3000
    DOMELIGHT=100
    ;;
  lowlight)
    RECTLIGHT_1=70
    RECTLIGHT_2=70
    DOMELIGHT=10
    ;;
  *)
    echo "[record_v2] ERROR: unknown lighting condition: ${LIGHTING_CONDITION}"
    echo "Allowed: bright | dim | lowlight"
    exit 1
    ;;
esac

cat > "${MANIFEST}" <<EOF
run_type: trajectory
trajectory_id: ${TRAJ_NAME}

env_id: simple_room_v2_lighting_sweep
usd_scene: ${USD_SCENE}
lighting_condition: ${LIGHTING_CONDITION}

command_source:
  source_bag: ${CMD_SOURCE_BAG}
  topics_replayed:
    - /cmd_vel
    - /traj_phase
  clock_replayed: false
  purpose: "Reuse V5 command stream and phase labels to avoid manual teleoperation differences."

lighting_config:
  rectlight_1_intensity: ${RECTLIGHT_1}
  rectlight_2_intensity: ${RECTLIGHT_2}
  domelight_intensity: ${DOMELIGHT}
  exposure: 0
  color: white
  note: "Same light positions across bright, dim, and lowlight. Only intensity changes."

camera_config: stereo_640x480
isaac_sim: "5.0"

ros:
  use_sim_time: true

topics_recorded:
$(printf "  - %s\n" "${TOPICS[@]}")

clock_policy: "New Isaac Sim /clock recorded in V6 bag. V5 /clock is not replayed."

date_time_local: "${DATE_LOCAL}"

description: >
  V2 master ROS 2 dataset recorded from Isaac Sim under a controlled lighting
  condition. The robot is driven by replaying only /cmd_vel and /traj_phase
  from the V5 master bag, while camera, IMU, joint states, TF, ground truth,
  laser scan, and clock are generated from the current Isaac Sim scene.

notes: >
  Robot, sensors, room layout, physics settings, trajectory command input,
  TF structure, and camera parameters are kept unchanged across lighting
  conditions.
EOF

echo "============================================================"
echo "[record_v2] V2 master dataset recording"
echo "============================================================"
echo "[record_v2] Trajectory name     : ${TRAJ_NAME}"
echo "[record_v2] Lighting condition  : ${LIGHTING_CONDITION}"
echo "[record_v2] USD scene           : ${USD_SCENE}"
echo "[record_v2] Command source bag  : ${CMD_SOURCE_BAG}"
echo "[record_v2] Output bag          : ${OUT_BAG}"
echo "[record_v2] Manifest            : ${MANIFEST}"
echo "============================================================"
echo ""
echo "[record_v2] IMPORTANT:"
echo "  1. Open the correct USD scene in Isaac Sim."
echo "  2. Reset robot/simulation to the start pose."
echo "  3. Press Play in Isaac Sim before continuing."
echo "  4. This script will replay only /cmd_vel and /traj_phase from V5."
echo "  5. It will NOT replay V5 /clock, /tf, sensors, or /gt/odom."
echo ""

read -rp "[record_v2] Press ENTER when Isaac Sim is playing and ready..."

rm -rf "${OUT_BAG}"

REC_PID=""
cleanup() {
  echo ""
  echo "[record_v2] Cleanup requested."

  if [[ -n "${REC_PID}" ]]; then
    echo "[record_v2] Stopping recorder..."
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi

  exit 1
}
trap cleanup SIGINT SIGTERM

echo "[record_v2] Starting recorder..."
ros2 bag record "${TOPICS[@]}" -o "${OUT_BAG}" &
REC_PID=$!

echo "[record_v2] Waiting for recorder subscriptions..."
sleep 3

echo "[record_v2] Replaying V5 command source topics only:"
echo "  /cmd_vel"
echo "  /traj_phase"
echo ""

ros2 bag play "${CMD_SOURCE_BAG}" \
  --topics /cmd_vel /traj_phase \
  --disable-keyboard-controls

echo ""
echo "[record_v2] Command replay finished."
echo "[record_v2] Stopping recorder..."

kill -INT "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" || true

echo ""
echo "============================================================"
echo "[record_v2] SUCCESS"
echo "[record_v2] Saved V2 master bag:"
echo "  ${OUT_BAG}"
echo "============================================================"
echo ""
echo "[record_v2] Check bag with:"
echo "  ros2 bag info ${OUT_BAG}"
