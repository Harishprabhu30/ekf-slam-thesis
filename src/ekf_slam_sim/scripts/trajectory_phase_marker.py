#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class TrajectoryPhaseMarker(Node):

    def __init__(self):
        super().__init__('trajectory_phase_marker')
        self.publisher = self.create_publisher(Int32, '/traj_phase', 10)
        self.phase = 0

        self.get_logger().info("Phase Marker Started.")
        self.get_logger().info("Press:")
        self.get_logger().info("0 = STOP")
        self.get_logger().info("1 = STRAIGHT")
        self.get_logger().info("2 = SQUARE")
        self.get_logger().info("3 = SPIN")
        self.get_logger().info("4 = ARC")

    def publish_phase(self, phase_value):
        msg = Int32()
        msg.data = phase_value
        self.publisher.publish(msg)
        self.get_logger().info(f"Phase set to {phase_value}")

def main():
    rclpy.init()
    node = TrajectoryPhaseMarker()

    try:
        while True:
            key = input("Enter phase (0-4): ")

            if key in ['0','1','2','3','4']:
                node.publish_phase(int(key))
            else:
                print("Invalid input.")

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
