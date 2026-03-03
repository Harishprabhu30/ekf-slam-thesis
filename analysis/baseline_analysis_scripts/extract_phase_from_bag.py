import sys
import pandas as pd

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message

# We'll support common phase message types:
from std_msgs.msg import Int32, UInt8, String

def extract_phase(bag_path: str, topic_name: str, out_csv: str):
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic_name not in topic_types:
        raise RuntimeError(f"Topic {topic_name} not found in bag. Available: {list(topic_types.keys())}")

    msg_type = topic_types[topic_name]

    rows = []
    while reader.has_next():
        topic, raw, t = reader.read_next()
        if topic != topic_name:
            continue

        if msg_type == "std_msgs/msg/Int32":
            msg = deserialize_message(raw, Int32)
            phase = int(msg.data)
        elif msg_type == "std_msgs/msg/UInt8":
            msg = deserialize_message(raw, UInt8)
            phase = int(msg.data)
        elif msg_type == "std_msgs/msg/String":
            msg = deserialize_message(raw, String)
            phase = str(msg.data)
        else:
            raise RuntimeError(f"Unsupported traj_phase type: {msg_type} (add it in script)")

        rows.append((int(t), phase))

    df = pd.DataFrame(rows, columns=["timestamp", "phase"]).drop_duplicates()
    df = df.sort_values("timestamp").reset_index(drop=True)

    if len(df) == 0:
        raise RuntimeError("No /traj_phase messages found (df is empty).")

    # normalize to t_rel
    t0 = int(df["timestamp"].iloc[0])
    df["t_rel"] = (df["timestamp"] - t0) * 1e-9

    df.to_csv(out_csv, index=False)
    print("Saved:", out_csv)
    print("Type:", msg_type)
    print("Rows:", len(df))
    print("Phase counts:\n", df["phase"].value_counts())

if __name__ == "__main__":
    # usage:
    # python3 extract_phase_from_bag.py <bag_path> <topic> <out_csv>
    extract_phase(sys.argv[1], sys.argv[2], sys.argv[3])
