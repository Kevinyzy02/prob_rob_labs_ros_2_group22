#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node


heartbeat_period = 0.1

class Lab_6Assignment_1(Node):

    def __init__(self):
        super().__init__('lab_6_assignment_1')
        self.declare_parameter("map_file", "config/lab6_landmarks.yaml")
        map_file_param = self.get_parameter("map_file").get_parameter_value().string_value
        
        this_dir = os.path.dirname(os.path.abspath(__file__))   # .../prob_rob_labs/src/lab_6_assignment_1
        src_dir = os.path.dirname(this_dir)                     # .../prob_rob_labs/src
        pkg_root = os.path.dirname(src_dir)                     # .../prob_rob_labs

        if os.path.isabs(map_file_param):
            map_file = map_file_param
        else:
            map_file = os.path.join(pkg_root, map_file_param)

        if not os.path.exists(map_file):
            self.get_logger().fatal(f"Map file does not exist: {map_file}")
            raise SystemExit(1)

        self.get_logger().info(f"Loading landmark map from: {map_file}")
        with open(map_file, "r") as f:
            data = yaml.safe_load(f)

        self.landmarks = data.get("landmarks", {})

        if not self.landmarks:
            self.get_logger().warn("No landmarks found in map file!")

        for color, info in self.landmarks.items():
            x = info.get("x")
            y = info.get("y")
            w = info.get("width")
            h = info.get("height")
            frame_id = info.get("frame_id", "map")

            self.get_logger().info(
                f"Landmark '{color}': "
                f"x={x}, y={y}, width={w}, height={h}, frame_id={frame_id}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Lab_6Assignment_1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
