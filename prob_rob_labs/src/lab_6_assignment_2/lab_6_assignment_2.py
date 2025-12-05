#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import yaml
import math
import numpy as np
import os
from functools import partial

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from prob_rob_msgs.msg import Point2DArrayStamped

class Lab_6Assignment_2(Node):
    def __init__(self):
        super().__init__('lab_6_assignment_2')

        self.declare_parameter('map_file', 'lab6_landmarks.yaml')
        
        self.declare_parameter('alpha1', 0.1)  # rot -> rot
        self.declare_parameter('alpha2', 0.01) # trans -> rot
        self.declare_parameter('alpha3', 0.1)  # trans -> trans
        self.declare_parameter('alpha4', 0.01) # rot -> trans

        self.mu = np.zeros((3, 1))
        
        self.sigma = np.diag([0.1, 0.1, 0.1])

        self.last_time = None

        self.map_data = {}
        self.camera_params = {
            'fx': None,
            'fy': None,
            'cx': None,
            'cy': None,
            'received': False
        }
        
        self.load_map()

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)
        
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.landmark_subs = []
        for color, data in self.map_data.items():
            topic_name = data.get('topic', f"/vision_{color}/corners")
            self.get_logger().info(f"Subscribing to {color} landmark at {topic_name}")
            
            sub = self.create_subscription(
                Point2DArrayStamped,
                topic_name,
                partial(self.landmark_callback, landmark_color=color),
                10
            )
            self.landmark_subs.append(sub)

        self.get_logger().info("EKF Localization Node (Assignment 2) Initialized")

    def load_map(self):
        map_path = self.get_parameter('map_file').get_parameter_value().string_value
        
        try:
            if not os.path.exists(map_path) and not map_path.startswith('/'):
                 if os.path.exists(os.path.join(os.getcwd(), map_path)):
                     map_path = os.path.join(os.getcwd(), map_path)

            with open(map_path, 'r') as file:
                data = yaml.safe_load(file)
            
            landmarks = data.get('landmarks', [])
            if isinstance(landmarks, list):
                for item in landmarks:
                    if isinstance(item, dict) and 'color' in item:
                        self.map_data[item['color']] = item
            
            self.get_logger().info(f"Loaded {len(self.map_data)} landmarks from {map_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load map file: {e}")

    def camera_info_callback(self, msg):
        if not self.camera_params['received']:
            P = msg.p
            self.camera_params['fx'] = P[0]
            self.camera_params['fy'] = P[5]
            self.camera_params['cx'] = P[2]
            self.camera_params['cy'] = P[6]
            self.camera_params['received'] = True
            self.get_logger().info(f"Camera Params Received: fx={P[0]}, fy={P[5]}")

    def calculate_variance(self, dist, bearing):
        sigma_dist = 0.01 * dist**2 + 0.005
        sigma_bearing = 0.001 * dist**2 + 0.003
        return sigma_dist, sigma_bearing

    def odom_callback(self, msg: Odometry):
        current_time = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        
        if self.last_time is None:
            self.last_time = current_time
            return # Wait for next tick to have valid dt

        dt = current_time - self.last_time
        self.last_time = current_time

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        theta = self.mu[2, 0]

        if abs(w) < 1e-5:
            dx = v * dt * math.cos(theta)
            dy = v * dt * math.sin(theta)
            dtheta = 0.0
        else:
            dx = (-v/w) * math.sin(theta) + (v/w) * math.sin(theta + w*dt)
            dy = (v/w) * math.cos(theta) - (v/w) * math.cos(theta + w*dt)
            dtheta = w * dt

        self.mu[0, 0] += dx
        self.mu[1, 0] += dy
        self.mu[2, 0] += dtheta
        self.mu[2, 0] = math.atan2(math.sin(self.mu[2, 0]), math.cos(self.mu[2, 0]))

        G = np.eye(3)
        if abs(w) > 1e-5:
            G[0, 2] = (-v/w) * math.cos(theta) + (v/w) * math.cos(theta + w*dt)
            G[1, 2] = (-v/w) * math.sin(theta) + (v/w) * math.sin(theta + w*dt)
        else:
            G[0, 2] = -v * dt * math.sin(theta)
            G[1, 2] = v * dt * math.cos(theta)

        a1 = self.get_parameter('alpha1').value
        a2 = self.get_parameter('alpha2').value
        a3 = self.get_parameter('alpha3').value
        a4 = self.get_parameter('alpha4').value

        sigma_v2 = (a3 * v**2) + (a4 * w**2)
        sigma_w2 = (a1 * w**2) + (a2 * v**2)
        
        R = np.zeros((3, 3))
        R[0, 0] = sigma_v2 * abs(math.cos(theta)) + 1e-6
        R[1, 1] = sigma_v2 * abs(math.sin(theta)) + 1e-6
        R[2, 2] = sigma_w2 + 1e-6

        self.sigma = G @ self.sigma @ G.T + R
        
        self.publish_pose(msg.header.stamp)

    def landmark_callback(self, msg: Point2DArrayStamped, landmark_color: str):

        if not self.camera_params['received'] or not msg.points:
            return

        xs = np.array([p.x for p in msg.points])
        ys = np.array([p.y for p in msg.points])
        
        if len(msg.points) < 4: return
        y_min, y_max = np.min(ys), np.max(ys)
        x_min, x_max = np.min(xs), np.max(xs)
        h_pix = y_max - y_min
        if h_pix <= 1.0: return

        if landmark_color not in self.map_data: return
        lm_info = self.map_data[landmark_color]
        
        mx, my = lm_info['x'], lm_info['y']
        real_height = lm_info.get('height', 0.5)

        x_p = (x_min + x_max) / 2.0
        fx = self.camera_params['fx']
        cx = self.camera_params['cx']
        fy = self.camera_params['fy']
        
        z_theta = math.atan((cx - x_p) / fx)
        z_dist = (real_height * fy) / (h_pix * math.cos(z_theta))
        z_obs = np.array([[z_dist], [z_theta]])


        rx, ry, rtheta = self.mu[0,0], self.mu[1,0], self.mu[2,0]
        dx = mx - rx
        dy = my - ry
        q = dx**2 + dy**2
        dist_pred = math.sqrt(q)

        if dist_pred < 0.1:
            self.get_logger().warn(f"Skipping update for {landmark_color}: Robot too close to landmark estimate (Singularity).")
            return

        bearing_pred = math.atan2(dy, dx) - rtheta
        bearing_pred = math.atan2(math.sin(bearing_pred), math.cos(bearing_pred))
        z_pred = np.array([[dist_pred], [bearing_pred]])

        H = np.zeros((2, 3))
        H[0, 0] = -dx / dist_pred
        H[0, 1] = -dy / dist_pred
        H[0, 2] = 0
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1

        var_dist, var_theta = self.calculate_variance(z_dist, z_theta)
        Q = np.diag([var_dist, var_theta])

        S = H @ self.sigma @ H.T + Q
        try:
            K = self.sigma @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        innovation = z_obs - z_pred
        innovation[1, 0] = math.atan2(math.sin(innovation[1, 0]), math.cos(innovation[1, 0]))
        
        self.mu = self.mu + K @ innovation
        self.mu[2, 0] = math.atan2(math.sin(self.mu[2, 0]), math.cos(self.mu[2, 0]))
        self.sigma = (np.eye(3) - K @ H) @ self.sigma

        self.get_logger().info(
            f"Update [{landmark_color}]: "
            f"Pos=({self.mu[0,0]:.2f}, {self.mu[1,0]:.2f}) "
            f"Inn_Dist={float(innovation[0]):.2f}"
        )

    def publish_pose(self, timestamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = "odom"
        
        msg.pose.pose.position.x = float(self.mu[0, 0])
        msg.pose.pose.position.y = float(self.mu[1, 0])
        
        theta = self.mu[2, 0]
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        cov = np.zeros(36)
        cov[0]  = self.sigma[0, 0] # xx
        cov[1]  = self.sigma[0, 1] # xy
        cov[5]  = self.sigma[0, 2] # xth
        cov[6]  = self.sigma[1, 0] # yx
        cov[7]  = self.sigma[1, 1] # yy
        cov[11] = self.sigma[1, 2] # yth
        cov[30] = self.sigma[2, 0] # thx
        cov[31] = self.sigma[2, 1] # thy
        cov[35] = self.sigma[2, 2] # thth
        
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Lab_6Assignment_2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()