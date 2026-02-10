#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def wrap_to_pi(x: float) -> float:
    return math.atan2(math.sin(x), math.cos(x))

class SeparationBagAnalyzer(Node):
    def __init__(self):
        super().__init__("separation_bag_analyzer")

        # ===== USER SETTINGS =====
        self.r = 0.13937   # <-- put your locked wheel radius here
        self.N_turns = 2   # <-- set how many full turns you executed in this bag (2 or 3)
        self.left_joint = "joint_wheel_left"
        self.right_joint = "joint_wheel_right"

        # Motion detection
        self.min_wheel_rate = 0.02  # rad/s threshold for "moving"
        # =========================

        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 50)

        self.idx_L = None
        self.idx_R = None

        self.prev_phi_L = None
        self.prev_phi_R = None
        self.prev_t = None

        self.sum_dphi_L = 0.0
        self.sum_dphi_R = 0.0

        self.moving_started = False
        self.moving_stopped = False

    def cb(self, msg: JointState):
        # Map indices once
        if self.idx_L is None or self.idx_R is None:
            try:
                self.idx_L = msg.name.index(self.left_joint)
                self.idx_R = msg.name.index(self.right_joint)
                self.get_logger().info(f"Found joints: L idx={self.idx_L}, R idx={self.idx_R}")
            except ValueError:
                self.get_logger().warn("Wheel joint names not found in this JointState msg yet.")
                return

        phi_L = msg.position[self.idx_L]
        phi_R = msg.position[self.idx_R]

        # Timestamp
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        if self.prev_phi_L is None:
            self.prev_phi_L, self.prev_phi_R = phi_L, phi_R
            self.prev_t = t
            return

        dt = t - self.prev_t
        if dt <= 0.0:
            self.prev_phi_L, self.prev_phi_R = phi_L, phi_R
            self.prev_t = t
            return

        dphi_L = wrap_to_pi(phi_L - self.prev_phi_L)
        dphi_R = wrap_to_pi(phi_R - self.prev_phi_R)

        w_L = abs(dphi_L / dt)
        w_R = abs(dphi_R / dt)
        moving_now = (w_L > self.min_wheel_rate) or (w_R > self.min_wheel_rate)

        if moving_now and not self.moving_started:
            self.moving_started = True
            self.get_logger().info("Motion detected: START accumulating wheel deltas.")

        if self.moving_started and not self.moving_stopped:
            if moving_now:
                self.sum_dphi_L += dphi_L
                self.sum_dphi_R += dphi_R
            else:
                self.moving_stopped = True
                self.report_and_shutdown()

        self.prev_phi_L, self.prev_phi_R = phi_L, phi_R
        self.prev_t = t

    def report_and_shutdown(self):
        diff = (self.sum_dphi_R - self.sum_dphi_L)
        sum_lr = (self.sum_dphi_R + self.sum_dphi_L)

        dtheta_true = 2.0 * math.pi * float(self.N_turns)
        if abs(diff) < 1e-6:
            self.get_logger().error("Wheel rotation difference ~0. Did you rotate in place?")
            rclpy.shutdown()
            return

        b_hat = (self.r * abs(diff)) / dtheta_true

        # Extra sanity: in-place rotation should have near-zero translation component
        # For ideal in-place: Δφ_L ≈ -Δφ_R  -> sum close to 0 (not perfect in practice)
        self.get_logger().info("==== SEPARATION CALIBRATION RESULT ====")
        self.get_logger().info(f"r (locked) = {self.r:.6f} m")
        self.get_logger().info(f"N_turns = {self.N_turns:d} -> Δθ_true = {dtheta_true:.6f} rad")
        self.get_logger().info(f"Δφ_L = {self.sum_dphi_L:.6f} rad")
        self.get_logger().info(f"Δφ_R = {self.sum_dphi_R:.6f} rad")
        self.get_logger().info(f"Δφ_R - Δφ_L = {diff:.6f} rad")
        self.get_logger().info(f"Δφ_R + Δφ_L = {sum_lr:.6f} rad (should be near 0 for pure in-place)")
        self.get_logger().info(f"b_hat = {b_hat:.6f} m")
        self.get_logger().info("======================================")

        rclpy.shutdown()

def main():
    rclpy.init()
    node = SeparationBagAnalyzer()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

