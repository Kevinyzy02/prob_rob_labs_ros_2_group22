#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped  
from prob_rob_msgs.msg import Point2DArrayStamped
import numpy as np


class Lab_5Assignment_1(Node):
    def __init__(self):
        super().__init__('lab_5_assignment_1')

        self.sub = self.create_subscription(Point2DArrayStamped, '/vision_cyan/corners', self.corners_callback, 10)

        self.pub_center = self.create_publisher(PointStamped, 'axis_center', 10)

    def corners_callback(self, msg):
        if not msg.points:
            self.get_logger().warn('No corner points received.')
            return

        xs = np.array([p.x for p in msg.points])
        ys = np.array([p.y for p in msg.points])

        x_min, x_max = float(np.min(xs)), float(np.max(xs))
        y_min, y_max = float(np.min(ys)), float(np.max(ys))
        x_center = (x_min + x_max) / 2.0
        y_center = (y_min + y_max) / 2.0

        center = (x_center, y_center)

        self.get_logger().info(
            f"Vertical axis line: center={center}, height={y_max - y_min:.1f}"
        )

        self.publish_point(center, msg.header, self.pub_center)

    def publish_point(self, xy, header, pub):
        pt = PointStamped()
        pt.header = header
        pt.point.x, pt.point.y = xy[0], xy[1]
        pt.point.z = 0.0
        pub.publish(pt)


def main():
    rclpy.init()
    node = Lab_5Assignment_1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
