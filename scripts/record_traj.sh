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
  - /cmd_vel
  - /traj_phase
clock_policy: "Isaac only; /clock not recorded"
date_time_local: "${DATE_LOCAL}"
notes: ""
EOF

echo "[record_traj] Manifest written: ${MANIFEST}"
echo "[record_traj] Recording trajectory bag to: ${OUT_DIR}"

# Record only the canonical command topics
ros2 bag record /cmd_vel /traj_phase -o "${OUT_DIR}/${TRAJ_NAME}"
