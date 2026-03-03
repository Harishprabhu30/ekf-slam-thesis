# analysis/gt_evaluation_scripts/extract_traj.py

import os
import math
import pandas as pd

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


def quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


TOPIC_BY_MODE = {
    "gt": "/gt/odom",
    "wheel": "/odom",
    "ekf": "/odometry/filtered",
}


def extract_trajectory(bag_path: str, mode: str) -> pd.DataFrame:
    if mode not in TOPIC_BY_MODE:
        raise ValueError(f"mode must be one of {list(TOPIC_BY_MODE.keys())}")

    target_topic = TOPIC_BY_MODE[mode]

    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    data = []

    while reader.has_next():
        topic, raw, t_record = reader.read_next()

        if topic != target_topic:
            continue

        msg = deserialize_message(raw, Odometry)

        # Prefer message header stamp
        #if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        #    t_ns = stamp_to_ns(msg.header.stamp)
        #else:
        t_ns = int(t_record)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        data.append([t_ns, x, y, yaw])

    df = pd.DataFrame(data, columns=["t_ns", "x", "y", "yaw"])
    df = df.sort_values("t_ns").reset_index(drop=True)
    return df


def extract_phase_events(bag_path: str) -> pd.DataFrame:
    """Extract sparse phase change events (Int32) from /traj_phase."""
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    events = []
    while reader.has_next():
        topic, raw, t_record = reader.read_next()
        if topic != "/traj_phase":
            continue
        msg = deserialize_message(raw, Int32)
        # /traj_phase Int32 doesn't have header, so use record time
        events.append([int(t_record), int(msg.data)])

    df = pd.DataFrame(events, columns=["t_ns", "phase"])
    df = df.sort_values("t_ns").reset_index(drop=True)
    return df


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("bag_path")
    ap.add_argument("--mode", choices=["gt", "wheel", "ekf"], required=True)
    ap.add_argument("--out_dir", default="analysis/results_gt")
    ap.add_argument("--also_phase", action="store_true",
                    help="Also extract /traj_phase events (useful for GT bag).")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    traj = extract_trajectory(args.bag_path, args.mode)
    traj_path = os.path.join(args.out_dir, f"{args.mode}_traj.csv")
    traj.to_csv(traj_path, index=False)
    print(f"Saved trajectory: {traj_path} ({len(traj)} rows)")

    if args.also_phase:
        phase = extract_phase_events(args.bag_path)
        # phase_path = os.path.join(args.out_dir, "traj_phase_events.csv")
        phase_path = os.path.join(args.out_dir, f"{args.mode}_traj_phase_events.csv")
        phase.to_csv(phase_path, index=False)
        print(f"Saved phase events: {phase_path} ({len(phase)} rows)")
