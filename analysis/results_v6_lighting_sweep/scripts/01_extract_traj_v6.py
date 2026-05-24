import os
import math
import argparse
import numpy as np
import pandas as pd

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped

ORB_MAPPING = os.getenv("ORB_MAPPING", "B")
DEBUG_ORB = False

TOPIC_BY_MODE = {
    "gt": "/gt/odom",
    "wheel": "/odom",
    "ekf": "/odometry/filtered",
    "orb": "/orbslam3/pose",
}

def quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

def quat_to_rotmat_xyzw(qx, qy, qz, qw):
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)]
    ], dtype=float)

def orb_pose_to_planar(msg):
    px = msg.pose.position.x
    py = msg.pose.position.y
    pz = msg.pose.position.z
    q = msg.pose.orientation

    Rwc = quat_to_rotmat_xyzw(q.x, q.y, q.z, q.w)
    fwd_w = Rwc @ np.array([0.0, 0.0, 1.0])

    if ORB_MAPPING == "A":
        x_planar = pz
        y_planar = px
        yaw_planar = math.atan2(fwd_w[0], fwd_w[2])
    elif ORB_MAPPING == "B":
        x_planar = pz
        y_planar = -px
        yaw_planar = math.atan2(-fwd_w[0], fwd_w[2])
    elif ORB_MAPPING == "C":
        x_planar = -pz
        y_planar = px
        yaw_planar = math.atan2(fwd_w[0], -fwd_w[2])
    elif ORB_MAPPING == "D":
        x_planar = -pz
        y_planar = -px
        yaw_planar = math.atan2(-fwd_w[0], -fwd_w[2])
    else:
        raise ValueError(f"Invalid ORB_MAPPING: {ORB_MAPPING}")

    if DEBUG_ORB:
        print(
            f"[ORB DEBUG] px={px:.3f} py={py:.3f} pz={pz:.3f} | "
            f"x={x_planar:.3f} y={y_planar:.3f} yaw={yaw_planar:.3f}"
        )

    return x_planar, y_planar, yaw_planar

def open_reader(bag_path: str):
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader

def extract_trajectory(bag_path: str, mode: str) -> pd.DataFrame:
    if mode not in TOPIC_BY_MODE:
        raise ValueError(f"mode must be one of {list(TOPIC_BY_MODE.keys())}")

    target_topic = TOPIC_BY_MODE[mode]
    reader = open_reader(bag_path)

    rows = []

    while reader.has_next():
        topic, raw, t_record = reader.read_next()

        if topic != target_topic:
            continue

        if mode == "orb":
            msg = deserialize_message(raw, PoseStamped)
            x, y, yaw = orb_pose_to_planar(msg)
        else:
            msg = deserialize_message(raw, Odometry)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = quat_to_yaw(msg.pose.pose.orientation)

        rows.append([int(t_record), float(x), float(y), float(yaw)])

    df = pd.DataFrame(rows, columns=["t_ns", "x", "y", "yaw"])
    df = df.sort_values("t_ns").reset_index(drop=True)
    return df

def extract_phase_events(bag_path: str) -> pd.DataFrame:
    reader = open_reader(bag_path)
    rows = []

    while reader.has_next():
        topic, raw, t_record = reader.read_next()

        if topic != "/traj_phase":
            continue

        msg = deserialize_message(raw, Int32)
        rows.append([int(t_record), int(msg.data)])

    df = pd.DataFrame(rows, columns=["t_ns", "phase"])
    df = df.sort_values("t_ns").reset_index(drop=True)
    return df

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_path")
    ap.add_argument("--mode", required=True, choices=list(TOPIC_BY_MODE.keys()))
    ap.add_argument("--out_csv", required=True)
    ap.add_argument("--phase_out_csv", default=None)
    args = ap.parse_args()

    traj = extract_trajectory(args.bag_path, args.mode)

    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    traj.to_csv(args.out_csv, index=False)

    print(f"[extract] mode={args.mode}")
    print(f"[extract] bag={args.bag_path}")
    print(f"[extract] rows={len(traj)}")
    print(f"[extract] saved={args.out_csv}")

    if args.phase_out_csv:
        phases = extract_phase_events(args.bag_path)
        os.makedirs(os.path.dirname(args.phase_out_csv), exist_ok=True)
        phases.to_csv(args.phase_out_csv, index=False)
        print(f"[extract] phase rows={len(phases)}")
        print(f"[extract] phase saved={args.phase_out_csv}")

if __name__ == "__main__":
    main()
