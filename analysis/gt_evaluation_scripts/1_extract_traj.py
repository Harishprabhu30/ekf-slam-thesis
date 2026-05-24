# analysis/gt_evaluation_scripts/1_extract_traj.py

import os
import math
import pandas as pd
import numpy as np

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped

# ========================= CONFIG =========================
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v6_orb/lowlight")
ORB_MAPPING = "B"        # Options: "A", "B", "C", "D"
DEBUG_ORB = False         # True to print px,py,pz + planar values
SAVE_ORB_RAW = False      # Save raw ORB components for inspection
# ==========================================================

def quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

# ========================= ORB planar conversion helpers =========================
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

    # Select mapping
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
        raise ValueError("Invalid ORB_MAPPING")

    if DEBUG_ORB:
        print(f"[ORB DEBUG] px={px:.3f} py={py:.3f} pz={pz:.3f} | "
              f"x_planar={x_planar:.3f} y_planar={y_planar:.3f} yaw={yaw_planar:.3f}")

    return x_planar, y_planar, yaw_planar, px, py, pz, q.x, q.y, q.z, q.w
# ============================================================================

TOPIC_BY_MODE = {
    "gt": "/gt/odom",
    "wheel": "/odom",
    "ekf": "/odometry/filtered",
    "orb": "/orbslam3/pose",
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

        if mode == "orb":
            msg = deserialize_message(raw, PoseStamped)
            t_ns = int(t_record)
            x, y, yaw, px, py, pz, qx, qy, qz, qw = orb_pose_to_planar(msg)
            row = [t_ns, x, y, yaw]
            if SAVE_ORB_RAW:
                row += [px, py, pz, qx, qy, qz, qw]
        else:
            msg = deserialize_message(raw, Odometry)
            t_ns = int(t_record)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = quat_to_yaw(msg.pose.pose.orientation)
            row = [t_ns, x, y, yaw]

        data.append(row)

    # Build DataFrame
    if mode == "orb" and SAVE_ORB_RAW:
        cols = ["t_ns","x","y","yaw","px","py","pz","qx","qy","qz","qw"]
    else:
        cols = ["t_ns","x","y","yaw"]

    df = pd.DataFrame(data, columns=cols)
    df = df.sort_values("t_ns").reset_index(drop=True)
    return df

# ========================= Extract phase events =========================
def extract_phase_events(bag_path: str) -> pd.DataFrame:
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
        events.append([int(t_record), int(msg.data)])

    df = pd.DataFrame(events, columns=["t_ns", "phase"])
    df = df.sort_values("t_ns").reset_index(drop=True)
    return df
# ============================================================================

def save_and_print_csv(df: pd.DataFrame, bag_path: str, mode: str):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    bag_name = os.path.basename(bag_path.rstrip("/"))
    csv_path = os.path.join(RESULTS_DIR, f"{mode}_traj_lowlight.csv")
    df.to_csv(csv_path, index=False)

    print(f"\nCSV saved to: {csv_path}")
    print(f"Rows: {len(df)} | Columns: {list(df.columns)}")
    print("\nFirst 5 rows:")
    print(df.head())

    return csv_path

# ========================= PATH LENGTH / SCALE CHECK =========================
def path_len(df: pd.DataFrame) -> float:
    dx = np.diff(df["x"].values)
    dy = np.diff(df["y"].values)
    return np.sum(np.sqrt(dx*dx + dy*dy))

def check_orb_scale(df_gt, df_orb):
    L_gt = path_len(df_gt)
    L_orb = path_len(df_orb)
    ratio = L_orb / L_gt if L_gt > 0 else 0
    print(f"\n[PATH LENGTH CHECK] GT={L_gt:.3f} m | ORB={L_orb:.3f} m | ratio={ratio:.3f}")
    if ratio < 0.8:
        print("[WARNING] ORB trajectory is smaller than GT — likely metric-scale issue!")
    return ratio

# ========================= MAIN =========================
if __name__ == "__main__":
    import sys

    also_phase = False
    if "--also_phase" in sys.argv:
        also_phase = True
        sys.argv.remove("--also_phase")

    if len(sys.argv) == 3:
        bag_path = sys.argv[1]
        mode = sys.argv[2]
    elif len(sys.argv) == 4 and sys.argv[2] == "--mode":
        bag_path = sys.argv[1]
        mode = sys.argv[3]
    else:
        print("Usage:")
        print("  python 1_extract_traj.py <bag_path> <mode>")
        print("  python 1_extract_traj.py <bag_path> --mode <mode> [--also_phase]")
        sys.exit(1)

    traj_df = extract_trajectory(bag_path, mode)
    save_and_print_csv(traj_df, bag_path, mode)

    if also_phase:
        phase_df = extract_phase_events(bag_path)
        os.makedirs(RESULTS_DIR, exist_ok=True)
        phase_path = os.path.join(RESULTS_DIR, f"{mode}_traj_phase_events.csv")
        phase_df.to_csv(phase_path, index=False)
        print(f"\nSaved phase events: {phase_path}")
        print(f"Rows: {len(phase_df)} | First 5 rows:\n{phase_df.head()}")

    # ========================= SCALE CHECK =========================
    if mode == "orb":
        # Attempt to load GT for scale check
        gt_csv = os.path.join(RESULTS_DIR, "gt_traj_lowlight.csv")
        if os.path.exists(gt_csv):
            df_gt = pd.read_csv(gt_csv)
            check_orb_scale(df_gt, traj_df)
        else:
            print("[SCALE CHECK] GT CSV not found — run GT extraction first.")
