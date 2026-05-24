#!/usr/bin/env python3

import argparse
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PhotometricVariantNode(Node):
    def __init__(self, scale, gamma):
        super().__init__("photometric_variant_node")

        self.scale = scale
        self.gamma = gamma
        self.bridge = CvBridge()

        self.left_pub = self.create_publisher(
            Image,
            "/camera/left/image_raw",
            10
        )

        self.right_pub = self.create_publisher(
            Image,
            "/camera/right/image_raw",
            10
        )

        self.left_sub = self.create_subscription(
            Image,
            "/camera/left/image_raw_src",
            self.left_cb,
            10
        )

        self.right_sub = self.create_subscription(
            Image,
            "/camera/right/image_raw_src",
            self.right_cb,
            10
        )

        self.get_logger().info(
            f"Photometric variant node started: scale={scale}, gamma={gamma}"
        )

    def apply_transform(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        arr = img.astype(np.float32) / 255.0

        # Brightness scaling + gamma transform.
        arr = np.clip(arr * self.scale, 0.0, 1.0)
        arr = np.power(arr, self.gamma)

        out = np.clip(arr * 255.0, 0, 255).astype(np.uint8)

        out_msg = self.bridge.cv2_to_imgmsg(out, encoding=msg.encoding)
        out_msg.header = msg.header

        return out_msg

    def left_cb(self, msg):
        self.left_pub.publish(self.apply_transform(msg))

    def right_cb(self, msg):
        self.right_pub.publish(self.apply_transform(msg))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scale", type=float, required=True)
    parser.add_argument("--gamma", type=float, required=True)

    args = parser.parse_args()

    rclpy.init()
    node = PhotometricVariantNode(args.scale, args.gamma)
    rclpy.spin(node)


if __name__ == "__main__":
    main()

