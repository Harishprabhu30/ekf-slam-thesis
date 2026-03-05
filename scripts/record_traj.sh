#!/bin/bash

RUN_NAME=$1

if [ -z "$RUN_NAME" ]; then
  echo "Usage: ./record_traj.sh <traj_name>"
  exit 1
fi

echo "Recording trajectory: $RUN_NAME"

mkdir -p bags/trajectories

sleep 2

echo "Starting bag recording..."

ros2 bag record \
/cmd_vel \
/traj_phase \
-o bags/trajectories/$RUN_NAME

echo "Recording stopped."
