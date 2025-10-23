#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import CameraInfo
from prob_rob_msgs.msg import Point2DArrayStamped


class Lab_5Assignment_2(Node):

    def __init__(self):
        super().__init__('lab_5_assignment_2')

        self.landmark_height = 0.5
        self.colors = ['cyan', 'green', 'magenta', 'red', 'yellow']
        self.fx = self.fy = self.cx = self.cy = None

        self.latest_detections = {}

        self.create_subscription(CameraInfo, '/camera/camera_info', self.cam_cb, 10)
        for color in self.colors:
            topic = f'/vision_{color}/corners'
            self.create_subscription(
                Point2DArrayStamped,
                topic,
                lambda msg, c=color: self.corners_cb(msg, c),
                10,
            )

        self.pub_color = self.create_publisher(String, '/landmark_active/color', 10)
        self.pub_distance = self.create_publisher(Float64, '/landmark_active/distance', 10)
        self.pub_bearing = self.create_publisher(Float64, '/landmark_active/bearing', 10)

        self.create_timer(0.1, self.publish_strongest_detection)

        self.get_logger().info(" Landmark distance/bearing node started.")

    def cam_cb(self, msg: CameraInfo):
        P = msg.p
        self.fx, self.fy = P[0], P[5]
        self.cx, self.cy = P[2], P[6]


    def corners_cb(self, msg: Point2DArrayStamped, color: str):
        if self.fx is None or not msg.points:
            return

        xs = np.array([p.x for p in msg.points])
        ys = np.array([p.y for p in msg.points])
        y_min, y_max = np.min(ys), np.max(ys)
        x_min, x_max = np.min(xs), np.max(xs)
        h_pix = y_max - y_min
        if h_pix <= 1.0:
            return

        x_p = (x_min + x_max) / 2.0
        theta = math.atan((self.cx - x_p) / self.fx)
        d = (self.landmark_height * self.fy) / (h_pix * math.cos(theta))

        count = len(msg.points)
        self.latest_detections[color] = (count, d, theta, h_pix)

    def publish_strongest_detection(self):
        if not self.latest_detections:
            return

        best_color, best_data = max(self.latest_detections.items(), key=lambda kv: kv[1][0])
        count, d, theta, h_pix = best_data

        if count < 4:
            return

        self.pub_color.publish(String(data=best_color))
        self.pub_distance.publish(Float64(data=d))
        self.pub_bearing.publish(Float64(data=theta))

        self.get_logger().info(
            f"[{best_color.upper()}]  d={d:.2f} m, θ={math.degrees(theta):.2f}°, "
            f"h_pix={h_pix:.1f}, corners={count}"
        )

        self.latest_detections.clear()


def main():
    rclpy.init()
    node = Lab_5Assignment_2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
