# extract_metrics.py

import os
import math
import numpy as np
import pandas as pd
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

def extract_bag(bag_path, mode="wheel"):

    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    data = []

    while reader.has_next():
        topic, raw, t = reader.read_next()

        if mode == "wheel" and topic == "/odom":
            msg = deserialize_message(raw, Odometry)

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = quat_to_yaw(msg.pose.pose.orientation)
            omega = msg.twist.twist.angular.z

            data.append([t, x, y, yaw, omega, None])

        elif mode == "ekf" and topic == "/odometry/filtered":
            msg = deserialize_message(raw, Odometry)

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = quat_to_yaw(msg.pose.pose.orientation)
            omega = msg.twist.twist.angular.z

            data.append([t, x, y, yaw, omega, None])

    df = pd.DataFrame(data, columns=[
        "timestamp", "x", "y", "yaw", "omega_z", "phase"
    ])

    return df

if __name__ == "__main__":
    import sys

    bag_path = sys.argv[1]
    mode = sys.argv[2]

    df = extract_bag(bag_path, mode)

    # Ensure output directory exists
    os.makedirs("analysis/results", exist_ok=True)

    out_file = f"analysis/results/{mode}_baseline.csv"
    df.to_csv(out_file, index=False)

    print(f"Saved to {out_file}")
