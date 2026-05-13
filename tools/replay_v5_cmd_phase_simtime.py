#!/usr/bin/env python3

import argparse
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from rosgraph_msgs.msg import Clock


Event = Tuple[int, str, object]


def load_events(bag_path: str) -> List[Event]:
    reader = SequentialReader()

    storage_options = StorageOptions(
        uri=bag_path,
        storage_id="sqlite3"
    )

    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)

    events = []

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()

        if topic == "/cmd_vel":
            msg = deserialize_message(data, Twist)
            events.append((timestamp_ns, topic, msg))

        elif topic == "/traj_phase":
            msg = deserialize_message(data, Int32)
            events.append((timestamp_ns, topic, msg))

    if not events:
        raise RuntimeError(
            "No /cmd_vel or /traj_phase messages found in source bag."
        )

    events.sort(key=lambda x: x[0])

    first_t = events[0][0]
    normalized = [
        (t - first_t, topic, msg)
        for t, topic, msg in events
    ]

    return normalized


class SimTimeCommandReplayer(Node):
    def __init__(self, events: List[Event]):
        super().__init__("simtime_v5_cmd_phase_replayer")

        self.events = events
        self.index = 0

        self.current_clock_ns = None
        self.start_clock_ns = None
        self.done = False

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.phase_pub = self.create_publisher(
            Int32,
            "/traj_phase",
            10
        )

        clock_qos = QoSProfile(depth=10)
        clock_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.clock_sub = self.create_subscription(
            Clock,
            "/clock",
            self.clock_cb,
            clock_qos
        )

        self.timer = self.create_timer(
            0.002,
            self.tick
        )

        duration_s = self.events[-1][0] * 1e-9

        self.get_logger().info(
            f"Loaded {len(self.events)} events. "
            f"Command duration = {duration_s:.3f} s sim time."
        )

        self.get_logger().info(
            "Waiting for /clock from Isaac Sim..."
        )

    def clock_cb(self, msg: Clock):
        self.current_clock_ns = (
            msg.clock.sec * 1_000_000_000
            + msg.clock.nanosec
        )

        if self.start_clock_ns is None:
            self.start_clock_ns = self.current_clock_ns
            self.get_logger().info(
                f"Started replay at sim clock ns = {self.start_clock_ns}"
            )

    def tick(self):
        if self.done:
            return

        if self.current_clock_ns is None or self.start_clock_ns is None:
            return

        elapsed_ns = self.current_clock_ns - self.start_clock_ns

        # If simulation was reset after starting, stop safely.
        if elapsed_ns < 0:
            self.get_logger().warn(
                "Detected negative elapsed sim time. "
                "Was simulation reset during replay?"
            )
            return

        while (
            self.index < len(self.events)
            and self.events[self.index][0] <= elapsed_ns
        ):
            _, topic, msg = self.events[self.index]

            if topic == "/cmd_vel":
                self.cmd_pub.publish(msg)

            elif topic == "/traj_phase":
                self.phase_pub.publish(msg)

            self.index += 1

        if self.index >= len(self.events):
            self.get_logger().info(
                "Finished replaying V5 /cmd_vel and /traj_phase."
            )

            # Publish zero velocity once for safety.
            zero = Twist()
            self.cmd_pub.publish(zero)

            self.done = True
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--bag",
        required=True,
        help="Path to V5 source bag directory."
    )

    args = parser.parse_args()

    events = load_events(args.bag)

    rclpy.init()
    node = SimTimeCommandReplayer(events)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
