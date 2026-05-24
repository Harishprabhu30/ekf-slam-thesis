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


CmdEvent = Tuple[int, Twist]
PhaseEvent = Tuple[int, Int32]


def twist_key(msg: Twist):
    return (
        round(msg.linear.x, 4),
        round(msg.linear.y, 4),
        round(msg.linear.z, 4),
        round(msg.angular.x, 4),
        round(msg.angular.y, 4),
        round(msg.angular.z, 4),
    )


def load_events(bag_path: str):
    reader = SequentialReader()

    reader.open(
        StorageOptions(uri=bag_path, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    raw_cmd_events = []
    raw_phase_events = []

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()

        if topic == "/cmd_vel":
            msg = deserialize_message(data, Twist)
            raw_cmd_events.append((timestamp_ns, msg))

        elif topic == "/traj_phase":
            msg = deserialize_message(data, Int32)
            raw_phase_events.append((timestamp_ns, msg))

    if not raw_cmd_events:
        raise RuntimeError("No /cmd_vel messages found in source bag.")

    first_t = min(
        [t for t, _ in raw_cmd_events]
        + [t for t, _ in raw_phase_events]
    )

    cmd_events = [
        (t - first_t, msg)
        for t, msg in raw_cmd_events
    ]

    phase_events = [
        (t - first_t, msg)
        for t, msg in raw_phase_events
    ]

    cmd_events.sort(key=lambda x: x[0])
    phase_events.sort(key=lambda x: x[0])

    return cmd_events, phase_events


def print_bag_summary(cmd_events: List[CmdEvent],
                      phase_events: List[PhaseEvent]):
    duration_s = cmd_events[-1][0] * 1e-9

    nonzero_linear = sum(
        1 for _, m in cmd_events
        if abs(m.linear.x) > 1e-6
    )

    nonzero_angular = sum(
        1 for _, m in cmd_events
        if abs(m.angular.z) > 1e-6
    )

    print("\n================ V5 COMMAND SUMMARY ================")
    print(f"cmd_vel messages      : {len(cmd_events)}")
    print(f"traj_phase messages   : {len(phase_events)}")
    print(f"duration              : {duration_s:.3f} s")
    print(f"nonzero linear.x cmds : {nonzero_linear}")
    print(f"nonzero angular.z cmds: {nonzero_angular}")

    print("\nFirst command changes:")
    last = None
    changes = 0

    for t_ns, msg in cmd_events:
        key = twist_key(msg)

        if key != last:
            print(
                f"t={t_ns * 1e-9:8.3f}s | "
                f"lin.x={msg.linear.x: .3f} | "
                f"ang.z={msg.angular.z: .3f}"
            )
            last = key
            changes += 1

        if changes >= 30:
            break

    print("====================================================\n")


class SimTimeHoldReplayer(Node):
    def __init__(self,
                 cmd_events: List[CmdEvent],
                 phase_events: List[PhaseEvent],
                 publish_hz: float):
        super().__init__("simtime_v5_cmd_phase_hold_replayer")

        self.cmd_events = cmd_events
        self.phase_events = phase_events

        self.cmd_index = 0
        self.phase_index = 0

        self.current_cmd = Twist()
        self.last_cmd_key = None

        self.current_clock_ns = None
        self.start_clock_ns = None

        self.publish_period_wall = 1.0 / publish_hz

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.phase_pub = self.create_publisher(
            Int32,
            "/traj_phase",
            10,
        )

        clock_qos = QoSProfile(depth=10)
        clock_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.clock_sub = self.create_subscription(
            Clock,
            "/clock",
            self.clock_cb,
            clock_qos,
        )

        # Publish latest command continuously in wall-time,
        # while command selection follows simulation time.
        self.timer = self.create_timer(
            self.publish_period_wall,
            self.tick,
        )

        self.end_time_ns = self.cmd_events[-1][0]
        self.done = False

        self.get_logger().info(
            f"Loaded {len(self.cmd_events)} cmd_vel events and "
            f"{len(self.phase_events)} phase events."
        )

        self.get_logger().info(
            f"Continuous command publish rate: {publish_hz:.1f} Hz wall-time."
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
                f"Replay started at sim clock ns = {self.start_clock_ns}"
            )

    def tick(self):
        if self.done:
            return

        if self.current_clock_ns is None or self.start_clock_ns is None:
            return

        elapsed_ns = self.current_clock_ns - self.start_clock_ns

        if elapsed_ns < 0:
            self.get_logger().warn(
                "Negative elapsed sim time detected. "
                "Reset simulation and restart this node."
            )
            return

        # Advance scheduled cmd_vel according to simulation time
        while (
            self.cmd_index < len(self.cmd_events)
            and self.cmd_events[self.cmd_index][0] <= elapsed_ns
        ):
            _, self.current_cmd = self.cmd_events[self.cmd_index]

            key = twist_key(self.current_cmd)
            if key != self.last_cmd_key:
                self.get_logger().info(
                    f"cmd change @ {elapsed_ns * 1e-9:.2f}s sim | "
                    f"lin.x={self.current_cmd.linear.x:.3f}, "
                    f"ang.z={self.current_cmd.angular.z:.3f}"
                )
                self.last_cmd_key = key

            self.cmd_index += 1

        # Advance phase labels according to simulation time
        while (
            self.phase_index < len(self.phase_events)
            and self.phase_events[self.phase_index][0] <= elapsed_ns
        ):
            _, phase_msg = self.phase_events[self.phase_index]
            self.phase_pub.publish(phase_msg)

            self.get_logger().info(
                f"phase change @ {elapsed_ns * 1e-9:.2f}s sim | "
                f"phase={phase_msg.data}"
            )

            self.phase_index += 1

        # Continuously publish the latest command
        self.cmd_pub.publish(self.current_cmd)

        # Finish after command stream duration
        if elapsed_ns > self.end_time_ns + int(1.0e9):
            self.get_logger().info(
                "Finished V5 command replay. Publishing zero velocity."
            )

            zero = Twist()
            self.cmd_pub.publish(zero)

            self.done = True
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--bag",
        required=True,
        help="Path to V5 source bag directory.",
    )

    parser.add_argument(
        "--publish-hz",
        type=float,
        default=30.0,
        help="Wall-time rate to continuously publish latest /cmd_vel.",
    )

    args = parser.parse_args()

    cmd_events, phase_events = load_events(args.bag)

    print_bag_summary(cmd_events, phase_events)

    rclpy.init()
    node = SimTimeHoldReplayer(
        cmd_events=cmd_events,
        phase_events=phase_events,
        publish_hz=args.publish_hz,
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
