#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int32


LocalPoint = Tuple[float, float]


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GTWaypointTrajectoryPlayer(Node):
    def __init__(self, args):
        super().__init__("gt_waypoint_trajectory_player_v6")

        self.args = args

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.phase_pub = self.create_publisher(Int32, "/traj_phase", 10)

        clock_qos = QoSProfile(depth=10)
        clock_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.clock_sub = self.create_subscription(
            Clock,
            "/clock",
            self.clock_cb,
            clock_qos
        )

        self.gt_sub = self.create_subscription(
            Odometry,
            "/gt/odom",
            self.gt_cb,
            10
        )

        self.timer = self.create_timer(
            1.0 / args.publish_hz,
            self.control_loop
        )

        self.current_clock_ns: Optional[int] = None
        self.start_clock_ns: Optional[int] = None
        self.phase_start_ns: Optional[int] = None

        self.x = None
        self.y = None
        self.yaw = None

        self.init_x = None
        self.init_y = None
        self.init_yaw = None

        self.prev_yaw = None
        self.unwrapped_yaw = None

        self.phase = 0
        self.phase_name = "rest"
        self.targets: List[LocalPoint] = []
        self.target_index = 0

        self.rotation_target_yaw = None

        self.csv_writer = None
        self.csv_file = None

        if args.csv:
            Path(args.csv).parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = open(args.csv, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["t_sim", "phase", "phase_name", "x", "y", "yaw"]
            )

        self.get_logger().info("GT-assisted V6 trajectory player started.")
        self.get_logger().info(
            "Ground truth is used only for trajectory execution during dataset recording."
        )
        self.get_logger().info(
            f"Parameters: square_side={args.square_side}, "
            f"straight_length={args.straight_length}, "
            f"circle_radius={args.circle_radius}, "
            f"max_linear={args.max_linear}, max_angular={args.max_angular}"
        )

    def clock_cb(self, msg: Clock):
        self.current_clock_ns = (
            msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        )

        if self.start_clock_ns is None:
            self.start_clock_ns = self.current_clock_ns
            self.phase_start_ns = self.current_clock_ns

    def gt_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y
        raw_yaw = yaw_from_quat(q)

        if self.init_x is None:
            self.init_x = self.x
            self.init_y = self.y
            self.init_yaw = raw_yaw
            self.prev_yaw = raw_yaw
            self.unwrapped_yaw = raw_yaw

            self.get_logger().info(
                f"Initial pose locked: x={self.init_x:.3f}, "
                f"y={self.init_y:.3f}, yaw={self.init_yaw:.3f}"
            )

        else:
            dyaw = normalize_angle(raw_yaw - self.prev_yaw)
            self.unwrapped_yaw += dyaw
            self.prev_yaw = raw_yaw

        self.yaw = raw_yaw

    def sim_elapsed_s(self):
        if self.current_clock_ns is None or self.start_clock_ns is None:
            return 0.0
        return (self.current_clock_ns - self.start_clock_ns) * 1e-9

    def phase_elapsed_s(self):
        if self.current_clock_ns is None or self.phase_start_ns is None:
            return 0.0
        return (self.current_clock_ns - self.phase_start_ns) * 1e-9

    def publish_phase(self):
        msg = Int32()
        msg.data = int(self.phase)
        self.phase_pub.publish(msg)

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def local_to_world(self, lx, ly):
        c = math.cos(self.init_yaw)
        s = math.sin(self.init_yaw)

        wx = self.init_x + c * lx - s * ly
        wy = self.init_y + s * lx + c * ly

        return wx, wy

    def transition(self, phase, name, targets=None):
        self.phase = phase
        self.phase_name = name
        self.phase_start_ns = self.current_clock_ns
        self.target_index = 0
        self.targets = targets or []

        self.get_logger().info(
            f"Transition to phase {phase}: {name}"
        )

        self.publish_phase()

    def start_square(self):
        s = self.args.square_side

        # Clockwise square in the robot's initial local frame.
        targets = [
            (s, 0.0),
            (s, -s),
            (0.0, -s),
            (0.0, 0.0),
        ]

        self.transition(1, "square", targets)

    def start_straight(self):
        L = self.args.straight_length

        # Forward and return to origin.
        targets = [
            (L, 0.0),
            (0.0, 0.0),
        ]

        self.transition(2, "straight", targets)

    def start_cw_rotation(self):
        self.transition(3, "cw_rotation", [])

        self.rotation_target_yaw = (
            self.unwrapped_yaw - self.args.rotation_angle
        )

        self.get_logger().info(
            f"Rotation target yaw: {self.rotation_target_yaw:.3f}"
        )

    def start_circular(self):
        r = self.args.circle_radius
        n = self.args.circle_points

        # Circle starts near origin, center is to the robot's right in local frame.
        center_x = 0.0
        center_y = -r

        targets = []

        for i in range(1, n + 1):
            theta = (math.pi / 2.0) - (2.0 * math.pi * i / n)
            lx = center_x + r * math.cos(theta)
            ly = center_y + r * math.sin(theta)
            targets.append((lx, ly))

        self.transition(4, "circular", targets)

    def drive_to_current_target(self):
        if self.target_index >= len(self.targets):
            return True

        lx, ly = self.targets[self.target_index]
        tx, ty = self.local_to_world(lx, ly)

        dx = tx - self.x
        dy = ty - self.y

        dist = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.yaw)

        if dist < self.args.position_tolerance:
            self.get_logger().info(
                f"Reached target {self.target_index + 1}/{len(self.targets)} "
                f"in phase {self.phase_name}: local=({lx:.2f}, {ly:.2f})"
            )
            self.target_index += 1
            self.publish_zero()
            return self.target_index >= len(self.targets)

        cmd = Twist()

        # Rotate-in-place first if heading error is large.
        if abs(heading_error) > self.args.heading_gate:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(
                self.args.k_yaw * heading_error,
                -self.args.max_angular,
                self.args.max_angular,
            )
        else:
            cmd.linear.x = clamp(
                self.args.k_linear * dist,
                0.0,
                self.args.max_linear,
            )
            cmd.angular.z = clamp(
                self.args.k_yaw * heading_error,
                -self.args.max_angular,
                self.args.max_angular,
            )

        self.cmd_pub.publish(cmd)
        return False

    def run_rotation(self):
        yaw_error = normalize_angle(
            self.rotation_target_yaw - self.unwrapped_yaw
        )

        if abs(yaw_error) < self.args.yaw_tolerance:
            self.get_logger().info("Completed clockwise rotation.")
            self.publish_zero()
            return True

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = clamp(
            self.args.k_yaw * yaw_error,
            -self.args.max_angular,
            self.args.max_angular,
        )

        self.cmd_pub.publish(cmd)
        return False

    def log_csv(self):
        if self.csv_writer is None:
            return

        if self.x is None or self.current_clock_ns is None:
            return

        self.csv_writer.writerow(
            [
                f"{self.sim_elapsed_s():.6f}",
                self.phase,
                self.phase_name,
                f"{self.x:.6f}",
                f"{self.y:.6f}",
                f"{self.yaw:.6f}",
            ]
        )

    def control_loop(self):
        if self.current_clock_ns is None:
            return

        if self.x is None:
            self.publish_zero()
            return

        self.publish_phase()
        self.log_csv()

        # Phase 0: rest
        if self.phase == 0:
            self.publish_zero()

            if self.phase_elapsed_s() >= self.args.rest_time:
                self.start_square()

            return

        # Phase 1: square
        if self.phase == 1:
            done = self.drive_to_current_target()

            if done:
                self.start_straight()

            return

        # Phase 2: straight
        if self.phase == 2:
            done = self.drive_to_current_target()

            if done:
                self.start_cw_rotation()

            return

        # Phase 3: clockwise rotation
        if self.phase == 3:
            done = self.run_rotation()

            if done:
                self.start_circular()

            return

        # Phase 4: circular
        if self.phase == 4:
            done = self.drive_to_current_target()

            if done:
                self.transition(5, "stop", [])
                self.publish_zero()

            return

        # Phase 5: stop
        if self.phase == 5:
            self.publish_zero()

            if self.phase_elapsed_s() >= self.args.final_stop_time:
                self.get_logger().info("V6 trajectory completed.")
                self.publish_zero()

                if self.csv_file:
                    self.csv_file.close()

                rclpy.shutdown()

            return


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--square-side", type=float, default=0.8)
    parser.add_argument("--straight-length", type=float, default=1.0)
    parser.add_argument("--circle-radius", type=float, default=0.45)
    parser.add_argument("--circle-points", type=int, default=24)

    parser.add_argument("--rotation-angle", type=float, default=2.0 * math.pi)

    parser.add_argument("--rest-time", type=float, default=5.0)
    parser.add_argument("--final-stop-time", type=float, default=3.0)

    parser.add_argument("--max-linear", type=float, default=0.25)
    parser.add_argument("--max-angular", type=float, default=0.5)

    parser.add_argument("--k-linear", type=float, default=0.8)
    parser.add_argument("--k-yaw", type=float, default=1.4)

    parser.add_argument("--position-tolerance", type=float, default=0.08)
    parser.add_argument("--yaw-tolerance", type=float, default=0.04)
    parser.add_argument("--heading-gate", type=float, default=0.35)

    parser.add_argument("--publish-hz", type=float, default=20.0)

    parser.add_argument("--csv", type=str, default="")

    args = parser.parse_args()

    rclpy.init()
    node = GTWaypointTrajectoryPlayer(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
