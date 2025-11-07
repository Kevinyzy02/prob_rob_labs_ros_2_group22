#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Pose, PoseStamped


class Lab5Assignment3(Node):

    def __init__(self):
        super().__init__('lab_5_assignment_3')

        self.landmarks_map = {
            "red":     (8.5,   -5.0),
            "green":   (8.5,    5.0),
            "yellow": (-11.5,   5.0),
            "magenta":(-11.5,  -5.0),
            "cyan":    (0.0,    0.0),
        }

        self.active_color = None
        self.d_meas = None
        self.theta_meas = None

        self.create_subscription(String,  "/landmark_active/color",    self.cb_color,   10)
        self.create_subscription(Float64, "/landmark_active/distance", self.cb_dmeas,   10)
        self.create_subscription(Float64, "/landmark_active/bearing",  self.cb_tmeas,   10)
        self.create_subscription(PoseStamped,    "/tb3/ground_truth/pose",    self.cb_gt,      10)

        self.pub_meas_d     = self.create_publisher(Float64, "/measurement/measured_distance", 10)
        self.pub_meas_theta = self.create_publisher(Float64, "/measurement/measured_bearing", 10)
        self.pub_err_d      = self.create_publisher(Float64, "/measurement_error/distance",    10)
        self.pub_err_theta  = self.create_publisher(Float64, "/measurement_error/bearing",     10)

        self.get_logger().info("Assignment 3 Error Characterization Node Running")

    def cb_color(self, msg):   self.active_color = msg.data
    def cb_dmeas(self, msg):   self.d_meas = msg.data
    def cb_tmeas(self, msg):   self.theta_meas = msg.data

    def cb_gt(self, msg: PoseStamped):
        if self.active_color is None or self.d_meas is None or self.theta_meas is None:
            return

        if self.active_color not in self.landmarks_map:
            return

        pose = msg.pose
        rx = pose.position.x
        ry = pose.position.y
        yaw = self.yaw_from_quat(pose.orientation)

        lx, ly = self.landmarks_map[self.active_color]

        dx = lx - rx
        dy = ly - ry

        x_bl =  math.cos(-yaw)*dx - math.sin(-yaw)*dy
        y_bl =  math.sin(-yaw)*dx + math.cos(-yaw)*dy

        z_cam = x_bl
        x_cam = -y_bl

        d_true = math.sqrt(z_cam**2 + x_cam**2)
        theta_true = -math.atan2(x_cam, z_cam)

        err_d = self.d_meas - d_true
        err_theta = self.theta_meas - theta_true

        self.pub_meas_d.publish(Float64(data=self.d_meas))
        self.pub_meas_theta.publish(Float64(data=self.theta_meas))
        self.pub_err_d.publish(Float64(data=err_d))
        self.pub_err_theta.publish(Float64(data=err_theta))

        self.get_logger().info(
            f"[{self.active_color}] "
            f"d_meas={self.d_meas:.2f}, d_true={d_true:.2f}, err_d={err_d:+.3f}  |  "
            f"theta_meas={math.degrees(self.theta_meas):.1f}°, "
            f"theta_true={math.degrees(theta_true):.1f}°, "
            f"err_theta={math.degrees(err_theta):+.2f}°"
        )


    @staticmethod
    def yaw_from_quat(q):
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        return math.atan2(siny, cosy)


def main():
    rclpy.init()
    node = Lab5Assignment3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
