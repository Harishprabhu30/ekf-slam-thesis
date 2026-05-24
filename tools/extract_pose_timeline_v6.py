#!/usr/bin/env python3

import argparse
import csv
import math

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def nearest_by_time(items, t_ns):
    if not items:
        return None

    best = min(items, key=lambda x: abs(x[0] - t_ns))
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument(
        "--camera-topic",
        default="/camera/left/image_raw",
        help="Use this topic timestamps as the render timeline."
    )

    args = parser.parse_args()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )

    gt_samples = []
    image_times = []
    phase_samples = []

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()

        if topic == "/gt/odom":
            msg = deserialize_message(data, Odometry)
            t_ns = stamp_to_ns(msg.header.stamp)

            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = yaw_from_quat(q)

            gt_samples.append((t_ns, p.x, p.y, p.z, yaw))

        elif topic == args.camera_topic:
            msg = deserialize_message(data, Image)
            t_ns = stamp_to_ns(msg.header.stamp)
            image_times.append(t_ns)

        elif topic == "/traj_phase":
            msg = deserialize_message(data, Int32)
            phase_samples.append((bag_time_ns, msg.data))

    if not gt_samples:
        raise RuntimeError("No /gt/odom samples found.")

    if not image_times:
        raise RuntimeError(f"No image timestamps found on {args.camera_topic}")

    gt_samples.sort(key=lambda x: x[0])
    image_times.sort()

    # Phase messages may have sparse bag-time stamps. If exact mapping is not clean,
    # nearest phase by bag time is still enough for phase labeling.
    phase_samples.sort(key=lambda x: x[0])

    first_t = image_times[0]

    rows = []

    for i, img_t in enumerate(image_times):
        gt = nearest_by_time(gt_samples, img_t)

        if gt is None:
            continue

        _, x, y, z, yaw = gt

        # Use nearest phase by relative position in time if available.
        # If not available, default to -1 and later analysis can still use overall metrics.
        phase = -1

        if phase_samples:
            # phase_samples are bag time, not header time, so this is approximate.
            # For robust phase labels, we mainly rely on the original sparse phase sequence.
            phase = nearest_by_time(phase_samples, img_t)[1]

        rows.append(
            {
                "idx": i,
                "t_rel_ns": img_t - first_t,
                "t_abs_ns": img_t,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "phase": phase,
            }
        )

    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "idx",
                "t_rel_ns",
                "t_abs_ns",
                "x",
                "y",
                "z",
                "yaw",
                "phase",
            ]
        )
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved pose timeline: {args.out}")
    print(f"Rows: {len(rows)}")
    print(f"Duration: {rows[-1]['t_rel_ns'] * 1e-9:.3f} s")


if __name__ == "__main__":
    main()
