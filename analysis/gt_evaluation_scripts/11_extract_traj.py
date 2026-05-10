# analysis/gt_evaluation_scripts/11_extract_traj.py

import os
import math
import pandas as pd
import numpy as np

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

# ========================= CONFIG =========================
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb")
ORB_MAPPING = "B"        # Options: "A", "B", "C", "D"
DEBUG_ORB = False
SAVE_ORB_RAW = False

CUVSLAM_MAP_PARENT = "map"
CUVSLAM_MAP_CHILD = "odom"
CUVSLAM_ODOM_PARENT = "odom"
CUVSLAM_ODOM_CHILD = "chassis_link"
# ==========================================================


def quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def yaw_to_rot2(yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=float)


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


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
    "cuvslam": "/visual_slam/tracking/odometry",   # local odom/debug
    "cuvslam_tf": "/tf",                           # composed TF debug
    "cuvslam_path": "/visual_slam/tracking/slam_path",  # preferred global SLAM path
}


def compose_se2(x1, y1, yaw1, x2, y2, yaw2):
    R1 = yaw_to_rot2(yaw1)
    t2_in_a = R1 @ np.array([x2, y2], dtype=float)
    x = x1 + t2_in_a[0]
    y = y1 + t2_in_a[1]
    yaw = wrap_pi(yaw1 + yaw2)
    return x, y, yaw


def extract_trajectory(bag_path: str, mode: str) -> pd.DataFrame:
    if mode not in TOPIC_BY_MODE:
        raise ValueError(f"mode must be one of {list(TOPIC_BY_MODE.keys())}")

    target_topic = TOPIC_BY_MODE[mode]
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    data = []

    latest_map_to_odom = None

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
            data.append(row)

        elif mode == "cuvslam_path":
            msg = deserialize_message(raw, Path)
            if len(msg.poses) == 0:
                continue

            last_pose = msg.poses[-1]
            t_ns = int(t_record)  # keep same time basis as your other modes
            x = last_pose.pose.position.x
            y = last_pose.pose.position.y
            yaw = quat_to_yaw(last_pose.pose.orientation)
            data.append([t_ns, x, y, yaw])

        elif mode == "cuvslam_tf":
            msg = deserialize_message(raw, TFMessage)

            for tf in msg.transforms:
                parent = tf.header.frame_id
                child = tf.child_frame_id

                x = tf.transform.translation.x
                y = tf.transform.translation.y
                yaw = quat_to_yaw(tf.transform.rotation)

                if parent == CUVSLAM_MAP_PARENT and child == CUVSLAM_MAP_CHILD:
                    latest_map_to_odom = (x, y, yaw)

                elif parent == CUVSLAM_ODOM_PARENT and child == CUVSLAM_ODOM_CHILD:
                    if latest_map_to_odom is None:
                        continue

                    x_map_odom, y_map_odom, yaw_map_odom = latest_map_to_odom
                    x_mb, y_mb, yaw_mb = compose_se2(
                        x_map_odom, y_map_odom, yaw_map_odom,
                        x, y, yaw
                    )

                    t_ns = int(t_record)
                    data.append([t_ns, x_mb, y_mb, yaw_mb])

        else:
            msg = deserialize_message(raw, Odometry)
            t_ns = int(t_record)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = quat_to_yaw(msg.pose.pose.orientation)
            data.append([t_ns, x, y, yaw])

    if mode == "orb" and SAVE_ORB_RAW:
        cols = ["t_ns", "x", "y", "yaw", "px", "py", "pz", "qx", "qy", "qz", "qw"]
    else:
        cols = ["t_ns", "x", "y", "yaw"]

    df = pd.DataFrame(data, columns=cols)
    df = df.sort_values("t_ns").drop_duplicates(subset=["t_ns"]).reset_index(drop=True)
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
    csv_path = os.path.join(RESULTS_DIR, f"{mode}_traj.csv")
    df.to_csv(csv_path, index=False)

    print(f"\nCSV saved to: {csv_path}")
    print(f"Rows: {len(df)} | Columns: {list(df.columns)}")
    print("\nFirst 5 rows:")
    print(df.head())

    return csv_path


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
        print("  python 11_extract_traj.py <bag_path> <mode>")
        print("  python 11_extract_traj.py <bag_path> --mode <mode> [--also_phase]")
        print("Modes: gt | wheel | ekf | orb | cuvslam | cuvslam_tf | cuvslam_path")
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

    if mode == "orb":
        gt_csv = os.path.join(RESULTS_DIR, "gt_traj.csv")
        if os.path.exists(gt_csv):
            df_gt = pd.read_csv(gt_csv)
            check_orb_scale(df_gt, traj_df)
        else:
            print("[SCALE CHECK] GT CSV not found — run GT extraction first.")
