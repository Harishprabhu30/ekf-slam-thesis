#!/usr/bin/env bash
set -euo pipefail

TRAJ_NAME="${1:-}"
if [[ -z "$TRAJ_NAME" ]]; then
echo "Usage: $0 <traj_name>"
exit 1
fi

OUT_DIR="bags/trajectories/${TRAJ_NAME}"
mkdir -p "${OUT_DIR}"

DATE_LOCAL="$(date --iso-8601=seconds)"

MANIFEST="${OUT_DIR}/manifest.yaml"

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

cat > "${MANIFEST}" <<EOF
run_type: trajectory
trajectory_id: ${TRAJ_NAME}

env_id: simple_room_v1
camera_config: stereo_640x480
isaac_sim: "5.0"
source: teleop

ros:
use_sim_time: true

topics_recorded:
$(printf "  - %s\n" "${TOPICS[@]}")

clock_policy: "Isaac clock recorded"

date_time_local: "${DATE_LOCAL}"

description: >
Master ROS2 dataset recorded from Isaac Sim using teleoperation.
This trajectory bag serves as the canonical dataset for offline
evaluation of multiple localization and SLAM estimators, including
wheel odometry, EKF-based localization, visual SLAM (e.g., ORB-SLAM3),
cuVSLAM, visual-inertial odometry, and other sensor fusion pipelines.

notes: ""
EOF

echo "[record_traj] Manifest written: ${MANIFEST}"
echo "[record_traj] Recording trajectory bag to: ${OUT_DIR}"

ros2 bag record "${TOPICS[@]}" -o "${OUT_DIR}/${TRAJ_NAME}"

