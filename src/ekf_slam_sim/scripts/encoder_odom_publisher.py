import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

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
	# keeping yaw bounded for numeric stability
	return math.atan2(math.sin(a), math.cos(a))
	
class EncoderOdomPublisher(Node):
    def __init__(self):
        super().__init__("encoder_odom_publisher")

        # Parameters
        self.left_joint = self.declare_parameter("left_wheel_joint", "joint_wheel_left").value
        self.right_joint = self.declare_parameter("right_wheel_joint", "joint_wheel_right").value
        self.wheel_radius = float(self.declare_parameter("wheel_radius", 0.10).value)
        self.wheel_sep = float(self.declare_parameter("wheel_separation", 0.46).value)

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "chassis_link").value

        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value

        # State
        self.prev_pos: Dict[str, float] = {}
        self.prev_stamp: Optional[rclpy.time.Time] = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 50)

        self.get_logger().info(
            f"EncoderOdomPublisher started. left={self.left_joint}, right={self.right_joint}, "
            f"r={self.wheel_radius:.3f} m, b={self.wheel_sep:.3f} m, base={self.base_frame}, odom={self.odom_frame}"
        )

    def on_joint_states(self, msg: JointState):
        # Build a name->position map for this message
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}

        if self.left_joint not in name_to_pos or self.right_joint not in name_to_pos:
            # Not fatal: just wait until joints appear
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
        dphi_l = name_to_pos[self.left_joint] - self.prev_pos[self.left_joint]
        dphi_r = name_to_pos[self.right_joint] - self.prev_pos[self.right_joint]

        # Update previous
        self.prev_stamp = stamp
        self.prev_pos[self.left_joint] = name_to_pos[self.left_joint]
        self.prev_pos[self.right_joint] = name_to_pos[self.right_joint]

        # Convert to wheel travel (m)
        ds_l = self.wheel_radius * dphi_l
        ds_r = self.wheel_radius * dphi_r

        # Differential drive kinematics
        ds = 0.5 * (ds_r + ds_l)
        dyaw = (ds_r - ds_l) / self.wheel_sep

        # Integrate using midpoint rotation (better than naive Euler)
        yaw_mid = self.yaw + 0.5 * dyaw
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw = norm_angle(self.yaw + dyaw)

        # Velocities
        vx = ds / dt
        wz = dyaw / dt

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

        # Minimal covariance placeholders (you’ll refine later)
        # Keep small non-zero to avoid downstream "all zeros" assumptions.
        odom.pose.covariance[0] = 1e-3   # x
        odom.pose.covariance[7] = 1e-3   # y
        odom.pose.covariance[35] = 1e-2  # yaw
        odom.twist.covariance[0] = 1e-2  # vx
        odom.twist.covariance[35] = 1e-1 # wz

        self.odom_pub.publish(odom)

        # Publish TF: odom -> chassis_link
        if self.publish_tf:
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
