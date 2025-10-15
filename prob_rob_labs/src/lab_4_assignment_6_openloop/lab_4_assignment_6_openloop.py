#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler


class OpenLoopOdometry(Node):
    def __init__(self):
        super().__init__('lab_4_assignment_6_odometry')

        self.use_sim_time = True

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        self.v = 0.0
        self.w = 0.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        self.get_logger().info('✅ Open-loop odometry node (clock-based) initialized.')

    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_odometry(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        if dt <= 0.0:
            return

        self.last_time = now

        v = self.v
        w = self.w

        if abs(w) < 1e-6:
            dx = v * math.cos(self.theta) * dt
            dy = v * math.sin(self.theta) * dt
            dtheta = 0.0
        else:
            dx = (v / w) * (math.sin(self.theta + w * dt) - math.sin(self.theta))
            dy = (v / w) * (-math.cos(self.theta + w * dt) + math.cos(self.theta))
            dtheta = w * dt

        self.x += dx
        self.y += dy
        self.theta = (self.theta + dtheta + math.pi) % (2 * math.pi) - math.pi  # wrap to [-π, π]

        q = quaternion_from_euler(0.0, 0.0, self.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        self.odom_pub.publish(odom_msg)

        self.get_logger().info(
            f"x={self.x:.3f}, y={self.y:.3f}, θ={math.degrees(self.theta):.1f}°, "
            f"v={v:.3f}, w={w:.3f}, dt={dt:.3f}s"
        )

    def run_timer(self):
        self.timer = self.create_timer(1.0 / 10.0, self.update_odometry)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopOdometry()
    node.run_timer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down open-loop odometry node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
