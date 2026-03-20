# analysis/gt_evaluation_scripts/inspect_orb_raw.py

import os
import pandas as pd
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

# Directory to save CSVs
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb_raw")


def extract_orb_raw(bag_path: str) -> pd.DataFrame:
    """
    Extract raw ORB pose components from /orbslam3/pose:
    t_ns, px, py, pz, qx, qy, qz, qw
    """
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    data = []

    while reader.has_next():
        topic, raw, t_record = reader.read_next()
        if topic != "/orbslam3/pose":
            continue

        msg = deserialize_message(raw, PoseStamped)
        t_ns = int(t_record)

        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        data.append([t_ns, px, py, pz, qx, qy, qz, qw])

    df = pd.DataFrame(data, columns=["t_ns", "px", "py", "pz", "qx", "qy", "qz", "qw"])
    df = df.sort_values("t_ns").reset_index(drop=True)
    return df


def save_and_describe(df: pd.DataFrame, bag_path: str):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    bag_name = os.path.basename(bag_path.rstrip("/"))
    csv_path = os.path.join(RESULTS_DIR, f"{bag_name}_orb_raw.csv")
    df.to_csv(csv_path, index=False)
    print(f"\nCSV saved to: {csv_path}")
    print(f"Rows: {len(df)} | Columns: {list(df.columns)}\n")
    print("Raw position summary (min/max/mean/std):")
    print(df[["px", "py", "pz"]].describe())


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python inspect_orb_raw.py <bag_path>")
        sys.exit(1)

    bag_path = sys.argv[1]
    df = extract_orb_raw(bag_path)
    save_and_describe(df, bag_path)
