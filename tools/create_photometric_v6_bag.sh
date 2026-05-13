#!/usr/bin/env bash
set -euo pipefail

VARIANT="${1:-}"
SCALE="${2:-}"
GAMMA="${3:-}"
SOURCE_BAG="${4:-bags/trajectories/traj_cmd_clean_v5_cam/traj_cmd_clean_v5_cam}"

if [[ -z "$VARIANT" || -z "$SCALE" || -z "$GAMMA" ]]; then
  echo "Usage:"
  echo "  $0 <variant_name> <scale> <gamma> [source_bag]"
  echo ""
  echo "Examples:"
  echo "  $0 traj_cmd_v6_photo_bright 1.00 1.00"
  echo "  $0 traj_cmd_v6_photo_dim 0.45 1.30"
  echo "  $0 traj_cmd_v6_photo_lowlight 0.12 1.80"
  exit 1
fi

OUT_DIR="bags/trajectories/${VARIANT}"
OUT_BAG="${OUT_DIR}/${VARIANT}"
MANIFEST="${OUT_DIR}/manifest.yaml"

mkdir -p "${OUT_DIR}"
rm -rf "${OUT_BAG}"

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

DATE_LOCAL="$(date --iso-8601=seconds)"

cat > "${MANIFEST}" <<EOF
run_type: trajectory
trajectory_id: ${VARIANT}

env_id: simple_room_v2_photometric_lighting_sweep
source_bag: ${SOURCE_BAG}

photometric_transform:
  scale: ${SCALE}
  gamma: ${GAMMA}
  transformed_topics:
    - /camera/left/image_raw
    - /camera/right/image_raw

camera_config: stereo_640x480
isaac_sim: "5.0"

ros:
  use_sim_time: true

topics_recorded:
$(printf "  - %s\n" "${TOPICS[@]}")

date_time_local: "${DATE_LOCAL}"

description: >
  Derived V2 lighting-robustness dataset generated from one master trajectory.
  All non-image topics are preserved from the source bag. Stereo image streams
  are photometrically transformed to create controlled bright, dim, and low-light
  visual conditions while keeping the trajectory, timing, IMU, wheel, TF, and
  ground-truth data identical.
EOF

cleanup() {
  echo "[photo_v6] Cleaning up..."

  if [[ -n "${REC_PID:-}" ]]; then
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi

  if [[ -n "${NODE_PID:-}" ]]; then
    kill -INT "${NODE_PID}" 2>/dev/null || true
    wait "${NODE_PID}" 2>/dev/null || true
  fi
}
trap cleanup SIGINT SIGTERM

echo "[photo_v6] Variant     : ${VARIANT}"
echo "[photo_v6] Source bag  : ${SOURCE_BAG}"
echo "[photo_v6] Output bag  : ${OUT_BAG}"
echo "[photo_v6] Scale/Gamma : ${SCALE} / ${GAMMA}"

python3 tools/photometric_variant_node.py \
  --scale "${SCALE}" \
  --gamma "${GAMMA}" &
NODE_PID=$!

sleep 2

ros2 bag record "${TOPICS[@]}" -o "${OUT_BAG}" &
REC_PID=$!

sleep 2

ros2 bag play "${SOURCE_BAG}" \
  --remap /camera/left/image_raw:=/camera/left/image_raw_src \
          /camera/right/image_raw:=/camera/right/image_raw_src \
  --disable-keyboard-controls

echo "[photo_v6] Replay finished. Stopping recorder and node..."

kill -INT "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" || true

kill -INT "${NODE_PID}" 2>/dev/null || true
wait "${NODE_PID}" || true

echo "[photo_v6] SUCCESS: ${OUT_BAG}"
echo "[photo_v6] Check with:"
echo "  ros2 bag info ${OUT_BAG}"
