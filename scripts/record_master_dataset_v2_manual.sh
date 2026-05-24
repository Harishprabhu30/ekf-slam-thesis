#!/usr/bin/env bash
set -euo pipefail

TRAJ_NAME="${1:-}"
LIGHTING_CONDITION="${2:-}"
USD_SCENE="${3:-}"

if [[ -z "$TRAJ_NAME" || -z "$LIGHTING_CONDITION" || -z "$USD_SCENE" ]]; then
  echo "Usage:"
  echo "  $0 <traj_name> <lighting_condition> <usd_scene>"
  echo ""
  echo "Examples:"
  echo "  $0 traj_cmd_v6_bright bright ekf-slam2-bright.usd"
  echo "  $0 traj_cmd_v6_dim dim ekf-slam2-dim.usd"
  echo "  $0 traj_cmd_v6_lowlight lowlight ekf-slam2-lowlight.usd"
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
    echo "[record_v2_manual] ERROR: unknown lighting condition: ${LIGHTING_CONDITION}"
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

lighting_config:
  rectlight_1_intensity: ${RECTLIGHT_1}
  rectlight_2_intensity: ${RECTLIGHT_2}
  domelight_intensity: ${DOMELIGHT}
  exposure: 0
  color: white

trajectory_protocol:
  source: fixed_rate_manual_teleop
  note: >
    Same route protocol as V5 was followed manually using fixed-rate teleop.
    Exact V5 trajectory reproduction was tested but not reliable, so each V2 run
    is evaluated against its own simulator ground truth.

camera_config: stereo_640x480
isaac_sim: "5.0"

ros:
  use_sim_time: true

topics_recorded:
$(printf "  - %s\n" "${TOPICS[@]}")

clock_policy: "Isaac /clock recorded in bag"

date_time_local: "${DATE_LOCAL}"

description: >
  V2 master ROS 2 dataset recorded from Isaac Sim under a controlled lighting
  condition. Robot, sensors, room layout, physics settings, estimator parameters,
  TF policy, and route protocol are kept consistent across bright, dim, and low-light runs.

notes: >
  This is a lighting robustness extension, not a frame-exact repeat of V5.
EOF

echo "============================================================"
echo "[record_v2_manual] V2 manual master recording"
echo "============================================================"
echo "Trajectory name    : ${TRAJ_NAME}"
echo "Lighting condition : ${LIGHTING_CONDITION}"
echo "USD scene          : ${USD_SCENE}"
echo "Output bag         : ${OUT_BAG}"
echo "Manifest           : ${MANIFEST}"
echo "============================================================"

rm -rf "${OUT_BAG}"
# remove --use-sim-time  if producing error
ros2 bag record --use-sim-time "${TOPICS[@]}" -o "${OUT_BAG}"
