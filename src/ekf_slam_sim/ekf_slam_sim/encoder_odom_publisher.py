#!/usr/bin/env python3
import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class EncoderOdomPublisher(Node):
    """
    Wheel-encoder-only differential drive odometry.

    Inputs:
      - /joint_states (sensor_msgs/JointState), must include left/right wheel joint positions in radians.

    Outputs:
      - /odom (nav_msgs/Odometry)
      - TF: odom -> chassis_link (or configured base frame)

    Notes:
      - Uses sim time when use_sim_time is true.
      - Normalizes wheel angle deltas to [-pi, pi] to prevent wrap-around jumps.
      - Geometry is configured via parameters (no hardcoded "real" values in code).
      - TF publishing can be toggled via publish_tf parameter (for EKF/SLAM ownership mode).
    """

    def __init__(self):
        super().__init__("encoder_odom_publisher")

        # -------------------------
        # Parameters
        # -------------------------
        self.left_joint = self.declare_parameter("left_wheel_joint", "joint_wheel_left").value
        self.right_joint = self.declare_parameter("right_wheel_joint", "joint_wheel_right").value

        # Safe placeholders; overwrite in config/odom_params.yaml
        self.wheel_radius = float(self.declare_parameter("wheel_radius", 0.14).value)
        self.wheel_sep = float(self.declare_parameter("wheel_separation", 0.4132).value)

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "chassis_link").value

        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.joint_states_topic = self.declare_parameter("joint_states_topic", "/joint_states").value

        # If true, warns when wheel delta is near pi (could indicate low joint_states rate / aliasing)
        self.warn_on_large_delta = bool(self.declare_parameter("warn_on_large_delta", True).value)

        # Allow runtime parameter update (useful during debugging)
        self.add_on_set_parameters_callback(self._on_set_params)

        # -------------------------
        # State
        # -------------------------
        self.prev_pos: Dict[str, float] = {}
        self.prev_stamp: Optional[rclpy.time.Time] = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # -------------------------
        # ROS publishers/subscribers
        # -------------------------
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # CLEANUP (TF endpoint hygiene):
        # Create the TF broadcaster ONLY when publish_tf is true at startup.
        # This prevents the node from appearing as a /tf publisher when TF ownership is disabled
        # (avoids confusion during EKF launch validation).
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(JointState, self.joint_states_topic, self.on_joint_states, 50)

        self.get_logger().info(
            "EncoderOdomPublisher started "
            f"(left={self.left_joint}, right={self.right_joint}, "
            f"wheel_radius={self.wheel_radius:.5f} m, wheel_separation={self.wheel_sep:.5f} m, "
            f"odom_frame={self.odom_frame}, base_frame={self.base_frame}, "
            f"publish_tf={self.publish_tf}, joint_states_topic={self.joint_states_topic})"
        )
        self.get_logger().info(
            "Note: Set wheel_radius and wheel_separation in config/odom_params.yaml "
            "after calibration; code defaults are placeholders."
        )

    def _on_set_params(self, params):
        for p in params:
            if p.name == "publish_tf":
                new_val = bool(p.value)
                self.publish_tf = new_val

                # CLEANUP (runtime TF enable):
                # If TF was disabled at startup and gets enabled at runtime, create the broadcaster then.
                if self.publish_tf and self.tf_broadcaster is None:
                    self.tf_broadcaster = TransformBroadcaster(self)

                self.get_logger().info(f"publish_tf set to: {self.publish_tf}")
        return SetParametersResult(successful=True)

    @staticmethod
    def _normalize_delta(dphi: float) -> float:
        # Keeps delta within [-pi, pi] to avoid wrap-around jumps (pi -> -pi)
        return math.atan2(math.sin(dphi), math.cos(dphi))

    def on_joint_states(self, msg: JointState):
        # Build name->position map for this message
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}

        if self.left_joint not in name_to_pos or self.right_joint not in name_to_pos:
            # Wait until wheel joints appear
            return

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        # Initialize on first message
        if self.prev_stamp is None:
            self.prev_stamp = stamp
            self.prev_pos[self.left_joint] = name_to_pos[self.left_joint]
            self.prev_pos[self.right_joint] = name_to_pos[self.right_joint]
            return

        dt = (stamp - self.prev_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # Wheel angle deltas (rad)
        raw_dphi_l = name_to_pos[self.left_joint] - self.prev_pos[self.left_joint]
        raw_dphi_r = name_to_pos[self.right_joint] - self.prev_pos[self.right_joint]

        # Normalize to avoid wrap-around jumps
        dphi_l = self._normalize_delta(raw_dphi_l)
        dphi_r = self._normalize_delta(raw_dphi_r)

        # Optional warning if deltas are suspiciously large (near pi)
        if self.warn_on_large_delta:
            near_pi = math.pi * 0.95
            if abs(dphi_l) > near_pi or abs(dphi_r) > near_pi:
                self.get_logger().warn(
                    f"Large wheel delta near pi detected (dphi_l={dphi_l:.3f}, dphi_r={dphi_r:.3f}). "
                    "This may indicate low joint_states rate or very high wheel speed."
                )

        # Update previous
        self.prev_stamp = stamp
        self.prev_pos[self.left_joint] = name_to_pos[self.left_joint]
        self.prev_pos[self.right_joint] = name_to_pos[self.right_joint]

        # Wheel travel (m)
        ds_l = self.wheel_radius * dphi_l
        ds_r = self.wheel_radius * dphi_r

        # Differential drive kinematics
        ds = 0.5 * (ds_r + ds_l)
        dyaw = (ds_r - ds_l) / self.wheel_sep

        # Integrate pose using midpoint rotation
        yaw_mid = self.yaw + 0.5 * dyaw
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw = norm_angle(self.yaw + dyaw)

        # Velocities
        vx = ds / dt
        wz = dyaw / dt

        # Debug logging
        self.get_logger().debug(
            f"raw_dphi_l={raw_dphi_l:.4f}, raw_dphi_r={raw_dphi_r:.4f}, "
            f"dphi_l={dphi_l:.4f}, dphi_r={dphi_r:.4f}, "
            f"ds_l={ds_l:.4f}, ds_r={ds_r:.4f}, ds={ds:.4f}, dyaw={dyaw:.4f}, "
            f"dt={dt:.4f}, vx={vx:.4f}, wz={wz:.4f}, "
            f"x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}"
        )

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz

        # Covariance placeholders (tune later; keep non-zero for downstream tools)
        odom.pose.covariance[0] = 1e-3    # x
        odom.pose.covariance[7] = 1e-3    # y
        odom.pose.covariance[35] = 1e-2   # yaw
        odom.twist.covariance[0] = 1e-2   # vx
        odom.twist.covariance[35] = 1e-1  # wz

        self.odom_pub.publish(odom)

        # Publish TF: odom -> base_frame (only if we own TF)
        if self.publish_tf and self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = yaw_to_quat(self.yaw)
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = EncoderOdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
