#!/usr/bin/env python3

import argparse
import bisect
from typing import List, Tuple, Optional

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


def clock_msg_to_ns(msg: Clock) -> int:
    return msg.clock.sec * 1_000_000_000 + msg.clock.nanosec


def twist_key(msg: Twist):
    return (
        round(msg.linear.x, 4),
        round(msg.linear.y, 4),
        round(msg.linear.z, 4),
        round(msg.angular.x, 4),
        round(msg.angular.y, 4),
        round(msg.angular.z, 4),
    )


def map_to_source_clock(
    event_bag_time_ns: int,
    clock_bag_times: List[int],
    clock_sim_times: List[int],
) -> Optional[int]:
    """
    Maps a rosbag storage timestamp to the nearest previous V5 /clock value.
    """
    idx = bisect.bisect_right(clock_bag_times, event_bag_time_ns) - 1

    if idx < 0:
        return None

    return clock_sim_times[idx]


def load_events_using_source_clock(bag_path: str):
    reader = SequentialReader()

    reader.open(
        StorageOptions(uri=bag_path, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    raw_cmd = []
    raw_phase = []
    clock_bag_times = []
    clock_sim_times = []

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()

        if topic == "/clock":
            msg = deserialize_message(data, Clock)
            clock_bag_times.append(bag_time_ns)
            clock_sim_times.append(clock_msg_to_ns(msg))

        elif topic == "/cmd_vel":
            msg = deserialize_message(data, Twist)
            raw_cmd.append((bag_time_ns, msg))

        elif topic == "/traj_phase":
            msg = deserialize_message(data, Int32)
            raw_phase.append((bag_time_ns, msg))

    if not clock_bag_times:
        raise RuntimeError("No /clock messages found in source V5 bag.")

    if not raw_cmd:
        raise RuntimeError("No /cmd_vel messages found in source V5 bag.")

    cmd_events = []
    phase_events = []

    missing_cmd = 0
    missing_phase = 0

    for bag_t, msg in raw_cmd:
        source_clock_ns = map_to_source_clock(
            bag_t,
            clock_bag_times,
            clock_sim_times,
        )

        if source_clock_ns is None:
            missing_cmd += 1
            continue

        cmd_events.append((source_clock_ns, msg))

    for bag_t, msg in raw_phase:
        source_clock_ns = map_to_source_clock(
            bag_t,
            clock_bag_times,
            clock_sim_times,
        )

        if source_clock_ns is None:
            missing_phase += 1
            continue

        phase_events.append((source_clock_ns, msg))

    if not cmd_events:
        raise RuntimeError("No /cmd_vel events could be mapped to V5 /clock.")

    cmd_events.sort(key=lambda x: x[0])
    phase_events.sort(key=lambda x: x[0])

    # Normalize schedule to the first command/phase clock time
    first_source_clock_ns = min(
        [t for t, _ in cmd_events]
        + [t for t, _ in phase_events]
    )

    cmd_events_norm = [
        (t - first_source_clock_ns, msg)
        for t, msg in cmd_events
    ]

    phase_events_norm = [
        (t - first_source_clock_ns, msg)
        for t, msg in phase_events
    ]

    source_clock_duration_s = (
        clock_sim_times[-1] - clock_sim_times[0]
    ) * 1e-9

    return (
        cmd_events_norm,
        phase_events_norm,
        source_clock_duration_s,
        missing_cmd,
        missing_phase,
    )


def print_summary(
    cmd_events: List[CmdEvent],
    phase_events: List[PhaseEvent],
    source_clock_duration_s: float,
    missing_cmd: int,
    missing_phase: int,
):
    duration_s = cmd_events[-1][0] * 1e-9

    nonzero_linear = sum(
        1 for _, m in cmd_events
        if abs(m.linear.x) > 1e-6
    )

    nonzero_angular = sum(
        1 for _, m in cmd_events
        if abs(m.angular.z) > 1e-6
    )

    print("\n=========== V5 SOURCE-CLOCK COMMAND SUMMARY ===========")
    print(f"cmd_vel messages mapped       : {len(cmd_events)}")
    print(f"traj_phase messages mapped    : {len(phase_events)}")
    print(f"missing cmd mappings          : {missing_cmd}")
    print(f"missing phase mappings        : {missing_phase}")
    print(f"source /clock duration        : {source_clock_duration_s:.3f} s")
    print(f"command schedule duration     : {duration_s:.3f} s")
    print(f"nonzero linear.x cmds         : {nonzero_linear}")
    print(f"nonzero angular.z cmds        : {nonzero_angular}")

    print("\nFirst command changes using V5 /clock timing:")
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

    print("========================================================\n")


class SourceClockHoldReplayer(Node):
    def __init__(
        self,
        cmd_events: List[CmdEvent],
        phase_events: List[PhaseEvent],
        publish_hz: float,
    ):
        super().__init__("v5_source_clock_cmd_phase_replayer")

        self.cmd_events = cmd_events
        self.phase_events = phase_events

        self.cmd_index = 0
        self.phase_index = 0

        self.current_cmd = Twist()
        self.last_cmd_key = None

        self.current_clock_ns = None
        self.start_clock_ns = None

        self.end_time_ns = self.cmd_events[-1][0]
        self.done = False

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

        self.timer = self.create_timer(
            1.0 / publish_hz,
            self.tick,
        )

        self.get_logger().info(
            f"Loaded {len(self.cmd_events)} source-clock cmd events and "
            f"{len(self.phase_events)} phase events."
        )

        self.get_logger().info(
            f"Publishing latest command continuously at {publish_hz:.1f} Hz wall-time."
        )

        self.get_logger().info(
            "Waiting for current Isaac Sim /clock..."
        )

    def clock_cb(self, msg: Clock):
        self.current_clock_ns = clock_msg_to_ns(msg)

        if self.start_clock_ns is None:
            self.start_clock_ns = self.current_clock_ns
            self.get_logger().info(
                f"Replay started at current sim clock ns = {self.start_clock_ns}"
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
                "Reset simulation and restart the replayer."
            )
            return

        while (
            self.cmd_index < len(self.cmd_events)
            and self.cmd_events[self.cmd_index][0] <= elapsed_ns
        ):
            _, self.current_cmd = self.cmd_events[self.cmd_index]

            key = twist_key(self.current_cmd)

            if key != self.last_cmd_key:
                self.get_logger().info(
                    f"cmd change @ {elapsed_ns * 1e-9:.2f}s current sim | "
                    f"lin.x={self.current_cmd.linear.x:.3f}, "
                    f"ang.z={self.current_cmd.angular.z:.3f}"
                )
                self.last_cmd_key = key

            self.cmd_index += 1

        while (
            self.phase_index < len(self.phase_events)
            and self.phase_events[self.phase_index][0] <= elapsed_ns
        ):
            _, phase_msg = self.phase_events[self.phase_index]
            self.phase_pub.publish(phase_msg)

            self.get_logger().info(
                f"phase change @ {elapsed_ns * 1e-9:.2f}s current sim | "
                f"phase={phase_msg.data}"
            )

            self.phase_index += 1

        self.cmd_pub.publish(self.current_cmd)

        if elapsed_ns > self.end_time_ns + int(1.0e9):
            self.get_logger().info(
                "Finished V5 source-clock command replay. Publishing zero velocity."
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

    (
        cmd_events,
        phase_events,
        source_clock_duration_s,
        missing_cmd,
        missing_phase,
    ) = load_events_using_source_clock(args.bag)

    print_summary(
        cmd_events,
        phase_events,
        source_clock_duration_s,
        missing_cmd,
        missing_phase,
    )

    rclpy.init()

    node = SourceClockHoldReplayer(
        cmd_events=cmd_events,
        phase_events=phase_events,
        publish_hz=args.publish_hz,
    )

    rclpy.spin(node)


if __name__ == "__main__":
    main()
