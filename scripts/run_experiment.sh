#!/bin/bash

RUN_NAME=$1

if [ -z "$RUN_NAME" ]; then
  echo "Usage: ./run_experiment.sh <run_name>"
  exit 1
fi

echo "Running experiment: $RUN_NAME"

mkdir -p bags/experiments

sleep 2

echo "Starting recording..."

ros2 bag record \
/odom \
/imu_raw \
/camera/left/image_raw \
/camera/left/camera_info \
/camera/right/image_raw \
/camera/right/camera_info \
/traj_phase \
-o bags/experiments/$RUN_NAME &

REC_PID=$!

sleep 3

echo "Replaying trajectory..."

ros2 bag play bags/trajectories/traj_cmd_clean_v2_cam

echo "Trajectory finished"

sleep 2

kill $REC_PID

echo "Experiment completed"
