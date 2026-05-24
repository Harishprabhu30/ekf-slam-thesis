#!/usr/bin/env python3

import argparse
import csv
import math

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def nearest(items, t_ns):
    return min(items, key=lambda x: abs(x[0] - t_ns))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--camera-topic", default="/camera/left/image_raw")
    args = parser.parse_args()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    gt = []
    cam_times = []

    while reader.has_next():
        topic, data, bag_t = reader.read_next()

        if topic == "/gt/odom":
            msg = deserialize_message(data, Odometry)
            t = stamp_to_ns(msg.header.stamp)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = yaw_from_quat(q)
            gt.append((t, p.x, p.y, p.z, yaw))

        elif topic == args.camera_topic:
            msg = deserialize_message(data, Image)
            t = stamp_to_ns(msg.header.stamp)
            cam_times.append(t)

    if not gt:
        raise RuntimeError("No /gt/odom found.")

    if not cam_times:
        raise RuntimeError(f"No camera timestamps found on {args.camera_topic}")

    gt.sort(key=lambda x: x[0])
    cam_times.sort()

    first_t = cam_times[0]
    rows = []

    for i, t in enumerate(cam_times):
        g = nearest(gt, t)
        _, x, y, z, yaw = g

        rows.append({
            "idx": i,
            "t_rel_ns": t - first_t,
            "t_abs_ns": t,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
        })

    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["idx", "t_rel_ns", "t_abs_ns", "x", "y", "z", "yaw"],
        )
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved: {args.out}")
    print(f"Rows: {len(rows)}")
    print(f"Duration: {rows[-1]['t_rel_ns'] * 1e-9:.3f} s")


if __name__ == "__main__":
    main()

