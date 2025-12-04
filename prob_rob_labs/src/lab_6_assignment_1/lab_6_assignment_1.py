#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import math
import numpy as np
from functools import partial

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String, Float64

from prob_rob_msgs.msg import Point2DArrayStamped

class Lab_6Assignment_1(Node):
    def __init__(self):
        super().__init__('lab_6_assignment_1')

        self.declare_parameter('map_file', 'lab6_landmarks.yaml')
        
        self.map_data = {}
        self.camera_params = {
            'fx': None,
            'fy': None,
            'cx': None,
            'cy': None,
            'received': False
        }
        
        self.load_map()

        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        self.landmark_subs = []
        for color, data in self.map_data.items():
            topic_name = data['topic']
            self.get_logger().info(f"Subscribing to {color} landmark at {topic_name}")
            
            sub = self.create_subscription(
                Point2DArrayStamped,
                topic_name,
                partial(self.landmark_callback, landmark_color=color),
                10
            )
            self.landmark_subs.append(sub)

        self.get_logger().info("EKF Localization Node Initialized")

    def load_map(self):
        map_path = self.get_parameter('map_file').get_parameter_value().string_value
        
        try:
            with open(map_path, 'r') as file:
                data = yaml.safe_load(file)
            for item in data['landmarks']:
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

    def landmark_callback(self, msg: Point2DArrayStamped, landmark_color: str):

        if not self.camera_params['received'] or not msg.points:
            return

        xs = np.array([p.x for p in msg.points])
        ys = np.array([p.y for p in msg.points])
        
        if len(msg.points) < 4:
            return

        y_min, y_max = np.min(ys), np.max(ys)
        x_min, x_max = np.min(xs), np.max(xs)
        
        h_pix = y_max - y_min
        
        if h_pix <= 1.0:
            return

        lm_info = self.map_data[landmark_color]
        real_height = lm_info['height']

        x_p = (x_min + x_max) / 2.0
        
        fx = self.camera_params['fx']
        cx = self.camera_params['cx']
        
        theta = math.atan((cx - x_p) / fx)

        fy = self.camera_params['fy']
        
        d = (real_height * fy) / (h_pix * math.cos(theta))

        var_dist, var_theta = self.calculate_variance(d, theta)

        self.get_logger().info(
            f"[{landmark_color.upper()}] "
            f"d={d:.2f}m, "
            f"theta={math.degrees(theta):.2f}deg, "
            f"h_pix={h_pix:.1f}, "
            f"var_dist={var_dist:.1f}, "
            f"var_theta={var_theta:.1f}, "
        )
        

def main(args=None):
    rclpy.init(args=args)
    node = Lab_6Assignment_1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()