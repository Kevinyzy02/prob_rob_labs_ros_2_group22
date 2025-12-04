#!/usr/bin/env python3

import os
import math
import yaml
import numpy as np 

import rclpy
from rclpy.node import Node

from prob_rob_msgs.msg import Point2DArrayStamped 

class Lab_6Assignment_2(Node):

    def __init__(self)-> None:
        super().__init__('lab_6_assignment_2')
        self.declare_parameter("map_file", "config/lab6_landmarks.yaml")
        map_file_param = self.get_parameter("map_file").get_parameter_value().string_value
        
        this_dir = os.path.dirname(os.path.abspath(__file__))   # .../prob_rob_labs/src/lab_6_assignment_2
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
            self.get_logger().info(
                f"Landmark '{color}': x={info.get('x')}, y={info.get('y')}, "
                f"width={info.get('width')}, height={self.landmark_heights[color]}, "
                f"frame_id={info.get('frame_id', 'map')}"
            )

        self.fx = self.fy = self.cx = self.cy = None
        self.create_subscription(CameraInfo, "/camera/camera_info", self.cam_cb, 10)

        self.measurements = {}   # store last measurement per color
        self.subscribers = []

        for color in self.landmarks.keys():
            topic = f"/vision_{color}/corners"   # SAME pattern as Lab 5

            self.get_logger().info(f"Subscribing to {topic} for color '{color}'")

            sub = self.create_subscription(
                Point2DArrayStamped,
                topic,
                lambda msg, c=color: self.corners_cb(msg, c),
                10,
            )
            self.subscribers.append(sub)

        self.get_logger().info("[Lab 6 A2] Landmark measurement node started.")


    def cam_cb(self, msg: CameraInfo):
        P = msg.p
        self.fx, self.fy = P[0], P[5]
        self.cx, self.cy = P[2], P[6]

  
    def compute_range_bearing_and_variance(self, msg: Point2DArrayStamped, color: str):
        """
        Uses your Lab 5 geometry:
            - compute h_pix from 4 corner points
            - estimate bearing theta and distance d
        Then adds a simple variance model (you can refine later for EKF).
        """

        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None

        if not msg.points:
            return None

        xs = np.array([p.x for p in msg.points])
        ys = np.array([p.y for p in msg.points])

        y_min, y_max = np.min(ys), np.max(ys)
        x_min, x_max = np.min(xs), np.max(xs)
        h_pix = y_max - y_min

        if h_pix <= 1.0:
            return None

        
        x_p = (x_min + x_max) / 2.0
        theta = math.atan((self.cx - x_p) / self.fx)  # bearing
        H = self.landmark_heights.get(color, self.default_height)
        d = (H * self.fy) / (h_pix * math.cos(theta))  # distance


        var_r = (H / (h_pix + 1e-6))**2 * 0.05      # distance variance ~ f(h_pix)
        var_b = (1.0 / (h_pix + 1e-6))**2 * 0.01    # bearing variance ~ f(h_pix)

        return d, theta, var_r, var_b, h_pix


    def corners_cb(self, msg: Point2DArrayStamped, color: str):
        result = self.compute_range_bearing_and_variance(msg, color)
        if result is None:
            return

        d, theta, var_r, var_b, h_pix = result
        count = len(msg.points)

        self.measurements[color] = {
            "range": d,
            "bearing": theta,
            "var_r": var_r,
            "var_b": var_b,
            "h_pix": h_pix,
            "corners": count,
            "stamp": msg.header.stamp,
        }

        self.get_logger().info(
            f"[{color.upper()}] d={d:.2f} m, θ={math.degrees(theta):.2f}°, "
            f"h_pix={h_pix:.1f}, corners={count}, var_r={var_r:.4f}, var_θ={var_b:.4f}"
        )



def main(args=None) -> None:
    rclpy.init(args=args)
    node = Lab_6Assignment_2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
