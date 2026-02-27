#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCovFixer(Node):
    def __init__(self):
        super().__init__('imu_covariance_fixer')

        self.declare_parameter('in_topic', '/imu_raw')
        self.declare_parameter('out_topic', '/imu')

        # Gyro variances (rad/s)^2
        self.declare_parameter('gyro_cov_xy', 4e-2)   # e.g., (0.2 rad/s)^2
        self.declare_parameter('gyro_cov_z',  4e-4)   # e.g., (0.02 rad/s)^2

        # Acc variances (m/s^2)^2
        self.declare_parameter('acc_cov_xy', 1e-1)
        self.declare_parameter('acc_cov_z',  1e-1)

        # If true, mark orientation as "not provided" (recommended for our thesis EKF baseline)
        self.declare_parameter('disable_orientation', True)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        self.pub = self.create_publisher(Imu, out_topic, 10)
        self.sub = self.create_subscription(Imu, in_topic, self.cb, 10)

    def cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header

        # Pass-through raw values
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        gyro_cov_xy = float(self.get_parameter('gyro_cov_xy').value)
        gyro_cov_z  = float(self.get_parameter('gyro_cov_z').value)
        acc_cov_xy  = float(self.get_parameter('acc_cov_xy').value)
        acc_cov_z   = float(self.get_parameter('acc_cov_z').value)
        disable_ori = bool(self.get_parameter('disable_orientation').value)

        # Angular velocity covariance (diag)
        out.angular_velocity_covariance = [0.0]*9
        out.angular_velocity_covariance[0] = gyro_cov_xy  # wx
        out.angular_velocity_covariance[4] = gyro_cov_xy  # wy
        out.angular_velocity_covariance[8] = gyro_cov_z   # wz (trusted most)

        # Linear acceleration covariance (diag)
        out.linear_acceleration_covariance = [0.0]*9
        out.linear_acceleration_covariance[0] = acc_cov_xy
        out.linear_acceleration_covariance[4] = acc_cov_xy
        out.linear_acceleration_covariance[8] = acc_cov_z

        # Orientation covariance
        if disable_ori:
            # ROS convention: orientation not provided
            out.orientation_covariance = [-1.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0]
        else:
            # If you ever decide to provide it, you MUST set a real covariance.
            # For now, keep disabled.
            out.orientation_covariance = [-1.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0]

        self.pub.publish(out)

def main():
    rclpy.init()
    node = ImuCovFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
