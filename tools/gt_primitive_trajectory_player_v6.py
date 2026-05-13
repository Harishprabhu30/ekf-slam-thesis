#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int32


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def sign(v):
    return 1.0 if v >= 0.0 else -1.0


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GTPrimitiveTrajectoryPlayer(Node):
    def __init__(self, args):
        super().__init__("gt_primitive_trajectory_player_v6")

        self.args = args

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.phase_pub = self.create_publisher(Int32, "/traj_phase", 10)

        clock_qos = QoSProfile(depth=10)
        clock_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.clock_sub = self.create_subscription(
            Clock,
            "/clock",
            self.clock_cb,
            clock_qos,
        )

        self.gt_sub = self.create_subscription(
            Odometry,
            "/gt/odom",
            self.gt_cb,
            10,
        )

        self.timer = self.create_timer(
            1.0 / args.publish_hz,
            self.control_loop,
        )

        self.current_clock_ns = None
        self.x = None
        self.y = None
        self.raw_yaw = None
        self.prev_raw_yaw = None
        self.yaw_unwrapped = None

        self.primitive_index = -1
        self.primitive = None
        self.primitive_start_ns = None
        self.primitive_start_x = None
        self.primitive_start_y = None
        self.primitive_start_yaw = None

        self.current_phase = None
        self.last_cmd = Twist()

        self.primitives = self.build_primitives()

        self.csv_file = None
        self.csv_writer = None

        if args.csv:
            Path(args.csv).parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = open(args.csv, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "t_sim",
                    "primitive_index",
                    "primitive_type",
                    "phase",
                    "phase_name",
                    "x",
                    "y",
                    "yaw",
                    "linear_x",
                    "angular_z",
                    "progress",
                    "target",
                ]
            )

        self.get_logger().info("GT primitive V6 trajectory player started.")
        self.get_logger().info(
            "Ground truth is used only for trajectory execution during dataset generation."
        )
        self.get_logger().info(f"Total primitives: {len(self.primitives)}")

    def build_primitives(self):
        p = []

        def rest(duration, phase, phase_name):
            p.append(
                {
                    "type": "rest",
                    "duration": duration,
                    "phase": phase,
                    "phase_name": phase_name,
                }
            )

        def settle(phase, phase_name):
            p.append(
                {
                    "type": "settle",
                    "duration": self.args.settle_time,
                    "phase": phase,
                    "phase_name": phase_name,
                }
            )

        def straight(distance, phase, phase_name):
            p.append(
                {
                    "type": "straight",
                    "distance": distance,
                    "phase": phase,
                    "phase_name": phase_name,
                }
            )

        def turn(angle, phase, phase_name):
            p.append(
                {
                    "type": "turn",
                    "angle": angle,
                    "phase": phase,
                    "phase_name": phase_name,
                }
            )

        def circle(phase, phase_name):
            p.append(
                {
                    "type": "circle",
                    "angle": -2.0 * math.pi,
                    "phase": phase,
                    "phase_name": phase_name,
                }
            )

        # Phase 0: initial rest
        rest(self.args.rest_time, 0, "rest")

        # Phase 1: square path
        for _ in range(4):
            straight(self.args.square_side, 1, "square")
            settle(1, "square")
            turn(-math.pi / 2.0, 1, "square")
            settle(1, "square")

        # Phase 2: long straight out and back
        straight(self.args.straight_length, 2, "straight")
        settle(2, "straight")
        turn(-math.pi, 2, "straight")
        settle(2, "straight")
        straight(self.args.straight_length, 2, "straight")
        settle(2, "straight")
        turn(-math.pi, 2, "straight")
        settle(2, "straight")

        # Phase 3: clockwise rotation at rest
        turn(-2.0 * math.pi, 3, "cw_rotation")
        settle(3, "cw_rotation")

        # Phase 4: circular / curved path
        circle(4, "circular")
        settle(4, "circular")

        # Phase 5: final stop
        rest(self.args.final_stop_time, 5, "stop")

        return p

    def clock_cb(self, msg):
        self.current_clock_ns = (
            msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        )

    def gt_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y
        self.raw_yaw = yaw_from_quat(q)

        if self.yaw_unwrapped is None:
            self.yaw_unwrapped = self.raw_yaw
            self.prev_raw_yaw = self.raw_yaw
        else:
            dyaw = normalize_angle(self.raw_yaw - self.prev_raw_yaw)
            self.yaw_unwrapped += dyaw
            self.prev_raw_yaw = self.raw_yaw

    def sim_time_s(self):
        if self.current_clock_ns is None:
            return 0.0
        return self.current_clock_ns * 1e-9

    def primitive_elapsed_s(self):
        if self.current_clock_ns is None or self.primitive_start_ns is None:
            return 0.0
        return (self.current_clock_ns - self.primitive_start_ns) * 1e-9

    def publish_zero(self):
        cmd = Twist()
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

    def publish_phase_if_changed(self, phase):
        if self.current_phase == phase:
            return

        self.current_phase = phase

        msg = Int32()
        msg.data = int(phase)
        self.phase_pub.publish(msg)

        self.get_logger().info(f"Published /traj_phase = {phase}")

    def start_next_primitive(self):
        self.primitive_index += 1

        if self.primitive_index >= len(self.primitives):
            self.get_logger().info("V6 trajectory completed.")
            self.publish_zero()

            if self.csv_file:
                self.csv_file.close()

            rclpy.shutdown()
            return

        self.primitive = self.primitives[self.primitive_index]
        self.primitive_start_ns = self.current_clock_ns
        self.primitive_start_x = self.x
        self.primitive_start_y = self.y
        self.primitive_start_yaw = self.yaw_unwrapped

        self.publish_phase_if_changed(self.primitive["phase"])

        self.get_logger().info(
            f"Primitive {self.primitive_index + 1}/{len(self.primitives)}: "
            f"{self.primitive['type']} | "
            f"phase={self.primitive['phase']} "
            f"({self.primitive['phase_name']})"
        )

    def log_csv(self, progress, target):
        if self.csv_writer is None:
            return

        if self.x is None or self.yaw_unwrapped is None:
            return

        self.csv_writer.writerow(
            [
                f"{self.sim_time_s():.6f}",
                self.primitive_index,
                self.primitive["type"] if self.primitive else "none",
                self.primitive["phase"] if self.primitive else -1,
                self.primitive["phase_name"] if self.primitive else "none",
                f"{self.x:.6f}",
                f"{self.y:.6f}",
                f"{self.yaw_unwrapped:.6f}",
                f"{self.last_cmd.linear.x:.6f}",
                f"{self.last_cmd.angular.z:.6f}",
                f"{progress:.6f}",
                f"{target:.6f}",
            ]
        )

    def control_rest_or_settle(self):
        duration = self.primitive["duration"]
        elapsed = self.primitive_elapsed_s()

        self.publish_zero()
        self.log_csv(elapsed, duration)

        if elapsed >= duration:
            self.start_next_primitive()

    def control_straight(self):
        target_dist = self.primitive["distance"]

        dx = self.x - self.primitive_start_x
        dy = self.y - self.primitive_start_y
        travelled = math.hypot(dx, dy)

        remaining = target_dist - travelled

        if remaining <= self.args.distance_tolerance:
            self.get_logger().info(
                f"Straight complete: travelled={travelled:.3f} m, "
                f"target={target_dist:.3f} m"
            )
            self.publish_zero()
            self.start_next_primitive()
            return

        # Hold the yaw at the yaw when the straight primitive started.
        yaw_error = self.primitive_start_yaw - self.yaw_unwrapped

        cmd = Twist()

        # Slow down near the target to reduce overshoot.
        if remaining < self.args.slowdown_distance:
            cmd.linear.x = clamp(
                self.args.k_dist * remaining,
                self.args.min_linear,
                self.args.straight_speed,
            )
        else:
            cmd.linear.x = self.args.straight_speed

        cmd.angular.z = clamp(
            self.args.k_yaw_hold * yaw_error,
            -self.args.max_yaw_correction,
            self.args.max_yaw_correction,
        )

        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)
        self.log_csv(travelled, target_dist)

    def control_turn(self):
        target_angle = self.primitive["angle"]
        target_yaw = self.primitive_start_yaw + target_angle

        remaining = target_yaw - self.yaw_unwrapped

        if abs(remaining) <= self.args.yaw_tolerance:
            self.get_logger().info(
                f"Turn complete: target_angle={target_angle:.3f} rad"
            )
            self.publish_zero()
            self.start_next_primitive()
            return

        cmd = Twist()
        angular = self.args.k_turn * remaining
        angular = clamp(
            angular,
            -self.args.turn_speed,
            self.args.turn_speed,
        )

        if abs(angular) < self.args.min_angular:
            angular = sign(angular) * self.args.min_angular

        cmd.angular.z = angular

        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

        turned = self.yaw_unwrapped - self.primitive_start_yaw
        self.log_csv(turned, target_angle)

    def control_circle(self):
        target_angle = self.primitive["angle"]
        target_yaw = self.primitive_start_yaw + target_angle

        remaining = target_yaw - self.yaw_unwrapped

        if abs(remaining) <= self.args.yaw_tolerance:
            self.get_logger().info("Circular phase complete.")
            self.publish_zero()
            self.start_next_primitive()
            return

        cmd = Twist()
        cmd.linear.x = self.args.circle_linear
        cmd.angular.z = self.args.circle_angular

        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

        turned = self.yaw_unwrapped - self.primitive_start_yaw
        self.log_csv(turned, target_angle)

    def control_loop(self):
        if self.current_clock_ns is None:
            return

        if self.x is None or self.yaw_unwrapped is None:
            self.publish_zero()
            return

        if self.primitive is None:
            self.start_next_primitive()
            return

        primitive_type = self.primitive["type"]

        if primitive_type in ["rest", "settle"]:
            self.control_rest_or_settle()
        elif primitive_type == "straight":
            self.control_straight()
        elif primitive_type == "turn":
            self.control_turn()
        elif primitive_type == "circle":
            self.control_circle()
        else:
            self.get_logger().error(f"Unknown primitive type: {primitive_type}")
            self.publish_zero()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--square-side", type=float, default=3.0)
    parser.add_argument("--straight-length", type=float, default=3.5)

    parser.add_argument("--rest-time", type=float, default=5.0)
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--final-stop-time", type=float, default=5.0)

    parser.add_argument("--straight-speed", type=float, default=0.25)
    parser.add_argument("--turn-speed", type=float, default=0.35)
    parser.add_argument("--circle-linear", type=float, default=0.20)
    parser.add_argument("--circle-angular", type=float, default=-0.20)

    parser.add_argument("--distance-tolerance", type=float, default=0.06)
    parser.add_argument("--yaw-tolerance", type=float, default=0.04)

    parser.add_argument("--slowdown-distance", type=float, default=0.35)
    parser.add_argument("--min-linear", type=float, default=0.06)
    parser.add_argument("--min-angular", type=float, default=0.08)

    parser.add_argument("--k-dist", type=float, default=0.7)
    parser.add_argument("--k-turn", type=float, default=0.9)
    parser.add_argument("--k-yaw-hold", type=float, default=1.2)
    parser.add_argument("--max-yaw-correction", type=float, default=0.12)

    parser.add_argument("--publish-hz", type=float, default=20.0)

    parser.add_argument("--csv", type=str, default="")

    args = parser.parse_args()

    rclpy.init()
    node = GTPrimitiveTrajectoryPlayer(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
