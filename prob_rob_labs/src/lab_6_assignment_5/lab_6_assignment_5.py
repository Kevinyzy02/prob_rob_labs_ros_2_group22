#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
import math

class Lab6Assignment5(Node):
    def __init__(self):
        super().__init__('lab_6_assignment_5')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.ekf_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/ekf_pose', 
            self.ekf_callback, 
            10
        )


        self.map_odom_trans = TransformStamped()
        self.map_odom_trans.header.frame_id = 'map'
        self.map_odom_trans.child_frame_id = 'odom'
        self.map_odom_trans.transform.rotation.w = 1.0

        self.create_timer(1.0 / 30.0, self.publish_transform)

        self.get_logger().info("Map->Odom TF Broadcaster (Assignment 5) Initialized")

    def ekf_callback(self, msg):
        tx_ekf = msg.pose.pose.position.x
        ty_ekf = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta_ekf = self.get_yaw(q)

        T_map_base = self.pose_to_matrix(tx_ekf, ty_ekf, theta_ekf)

        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 
                'base_footprint', 
                rclpy.time.Time()
            )
            
            tx_odom = trans.transform.translation.x
            ty_odom = trans.transform.translation.y
            q_odom = trans.transform.rotation
            theta_odom = self.get_yaw(q_odom)

            T_odom_base = self.pose_to_matrix(tx_odom, ty_odom, theta_odom)

        except Exception as e:
            self.get_logger().warn(f"Could not look up odom->base transform: {e}")
            return

        T_odom_base_inv = np.linalg.inv(T_odom_base)
        T_map_odom = np.dot(T_map_base, T_odom_base_inv)

        x, y, theta = self.matrix_to_pose(T_map_odom)
        
        self.map_odom_trans.transform.translation.x = x
        self.map_odom_trans.transform.translation.y = y
        self.map_odom_trans.transform.translation.z = 0.0
        
        self.map_odom_trans.transform.rotation.z = math.sin(theta / 2.0)
        self.map_odom_trans.transform.rotation.w = math.cos(theta / 2.0)

    def publish_transform(self):
        self.map_odom_trans.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_odom_trans)

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_to_matrix(self, x, y, theta):
        c = math.cos(theta)
        s = math.sin(theta)
        return np.array([
            [c, -s, x],
            [s,  c, y],
            [0,  0, 1]
        ])

    def matrix_to_pose(self, T):
        x = T[0, 2]
        y = T[1, 2]
        theta = math.atan2(T[1, 0], T[0, 0])
        return x, y, theta

def main(args=None):
    rclpy.init(args=args)
    node = Lab6Assignment5()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()