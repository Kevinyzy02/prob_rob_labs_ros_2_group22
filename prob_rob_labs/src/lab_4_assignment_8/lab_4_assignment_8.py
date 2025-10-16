#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64


class Lab4Assignment8(Node):

    def __init__(self):
        super().__init__('lab_4_assignment_8')

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.pub_gt_pose = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.pub_gt_twist = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)

        self.sub_gt = self.create_subscription(PoseStamped, '/tb3/ground_truth/pose', self.gt_cb, 10)
        self.sub_ekf = self.create_subscription(Odometry, '/ekf_odom', self.ekf_cb, 10)

        self.pub_pos_err = self.create_publisher(Float64, '/ekf_error/position', 10)
        self.pub_ang_err = self.create_publisher(Float64, '/ekf_error/orientation', 10)

        self.gt_pose = None
        self.ekf_pose = None

        self.timer = self.create_timer(0.5, self.report_error)
        self.get_logger().info('Assignment 8 node initialized.')

    def odom_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.pub_gt_pose.publish(ps)

        ts = TwistStamped()
        ts.header = msg.header
        ts.twist = msg.twist.twist
        self.pub_gt_twist.publish(ts)

    def gt_cb(self, msg: PoseStamped):
        self.gt_pose = msg

    def ekf_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.ekf_pose = ps

    def report_error(self):
        if self.gt_pose is None or self.ekf_pose is None:
            return

        x_gt, y_gt = self.gt_pose.pose.position.x, self.gt_pose.pose.position.y
        x_ekf, y_ekf = self.ekf_pose.pose.position.x, self.ekf_pose.pose.position.y
        pos_err = math.hypot(x_gt - x_ekf, y_gt - y_ekf)

        yaw_gt = self.get_yaw_from_quat(self.gt_pose.pose.orientation)
        yaw_ekf = self.get_yaw_from_quat(self.ekf_pose.pose.orientation)
        yaw_err = self.wrap_angle(yaw_gt - yaw_ekf)

        self.pub_pos_err.publish(Float64(data=pos_err))
        self.pub_ang_err.publish(Float64(data=yaw_err))

        self.get_logger().info(f"Position error = {pos_err:.3f} m | Angular error = {math.degrees(yaw_err):.2f}°")

    @staticmethod
    def get_yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = Lab4Assignment8()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Assignment 8 node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
