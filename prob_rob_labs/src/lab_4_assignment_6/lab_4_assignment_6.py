#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf_transformations import quaternion_from_euler


class EkfOdomNode(Node):
    def __init__(self):
        super().__init__('lab_4_assignment_6')

        self.r_w = 0.033 
        self.R = 0.1435 / 2.0 
        self.tau_v = 0.3591   
        self.tau_w = 0.446
        self.Gv = 1.0
        self.Gw = 1.0

        self.x = np.zeros((5, 1), dtype=float)
        self.P = np.diag([1e-3, 1e-3, 1e-3, 1e-2, 1e-2])

        self.q_base = np.array([1e-4, 1e-4, 1e-4, 1e-3, 5e-3])

        self.R_meas = np.diag([
            (0.02)**2, 
            (0.02)**2, 
            (0.005)**2 
        ])

        self.u_v = 0.0
        self.u_w = 0.0

        self.last_time = None

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.imu_sub = Subscriber(self, Imu, '/imu')
        self.odom_sub = Subscriber(self, Odometry, '/odom')

        self.sync = ApproximateTimeSynchronizer(
            [self.imu_sub, self.odom_sub], queue_size=20, slop=0.08)
        self.sync.registerCallback(self.sync_cb)

        self.pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        self.get_logger().info('EKF odometry node initialized.')

    def cmd_cb(self, msg: Twist):
        self.u_v = float(msg.linear.x)
        self.u_w = float(msg.angular.z)

    def sync_cb(self, imu_msg: Imu, odom_msg: Odometry):
        t = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = t
            return

        dt = t - self.last_time
        if dt <= 0.0:
            return
        self.last_time = t

        self.predict(dt)

        self.update(imu_msg, odom_msg)

        self.publish_odom(t)

    def predict(self, dt):
        x, y, th, v, w = self.x.flatten()

        a_v = 0.1 ** (dt / self.tau_v)
        a_w = 0.1 ** (dt / self.tau_w)

        nx = x + v * math.cos(th) * dt
        ny = y + v * math.sin(th) * dt
        nth = wrap_pi(th + w * dt)
        nv = a_v * v + self.Gv * (1 - a_v) * self.u_v
        nw = a_w * w + self.Gw * (1 - a_w) * self.u_w
        self.x = np.array([[nx], [ny], [nth], [nv], [nw]])

        F = np.array([
            [1, 0, -v * math.sin(th) * dt, math.cos(th) * dt, 0],
            [0, 1,  v * math.cos(th) * dt, math.sin(th) * dt, 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, a_v, 0],
            [0, 0, 0, 0, a_w]
        ], dtype=float)

        B = np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [self.Gv * (1 - a_v), 0],
            [0, self.Gw * (1 - a_w)]
        ], dtype=float)

        Q = np.diag(self.q_base * max(dt, 1e-3))

        self.P = F @ self.P @ F.T + Q

    def update(self, imu_msg: Imu, odom_msg: Odometry):

        v_body = float(odom_msg.twist.twist.linear.x)
        w_body = float(odom_msg.twist.twist.angular.z)

        v_enc_r = (v_body + self.R * w_body) / self.r_w
        v_enc_l = (v_body - self.R * w_body) / self.r_w

        w_gyro = float(imu_msg.angular_velocity.z)

        z = np.array([[v_enc_r],
                    [v_enc_l],
                    [w_gyro]])

        C = np.array([
            [0, 0, 0, 1 / self.r_w,  self.R / self.r_w],
            [0, 0, 0, 1 / self.r_w, -self.R / self.r_w],
            [0, 0, 0, 0,            1.0]
        ], dtype=float)

        z_hat = C @ self.x

        y = z - z_hat
        S = C @ self.P @ C.T + self.R_meas
        K = self.P @ C.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2, 0] = wrap_pi(self.x[2, 0])

        I = np.eye(5)
        self.P = (I - K @ C) @ self.P

    def publish_odom(self, t_sec):
        odom = Odometry()
        odom.header.stamp.sec = int(t_sec)
        odom.header.stamp.nanosec = int((t_sec - int(t_sec)) * 1e9)
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        x, y, th, v, w = self.x.flatten()
        q = quaternion_from_euler(0.0, 0.0, th)

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)

        P = self.P
        cov = [0.0] * 36
        cov[0]  = float(P[0, 0])   # var(x)
        cov[7]  = float(P[1, 1])   # var(y)
        cov[35] = float(P[2, 2])   # var(theta)
        cov[14] = float(P[3, 3])   # var(v)   
        cov[21] = float(P[4, 4])   # var(w) 
        odom.pose.covariance = cov

        self.pub.publish(odom)

def wrap_pi(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = EkfOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('EKF node shut down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
