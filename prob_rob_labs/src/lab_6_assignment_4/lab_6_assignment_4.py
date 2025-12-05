#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import message_filters
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float64

class Lab6Assignment4(Node):

    def __init__(self):
        super().__init__('lab_6_assignment_4')


        self.sub_ekf = message_filters.Subscriber(self, PoseWithCovarianceStamped, '/ekf_pose')
        
        self.sub_gt = message_filters.Subscriber(self, PoseStamped, '/tb3/ground_truth/pose')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ekf, self.sub_gt], 
            queue_size=10, 
            slop=0.5
        )
        self.ts.registerCallback(self.sync_cb)

        self.pub_pos_err = self.create_publisher(Float64, '/ekf_error/position', 10)
        self.pub_ang_err = self.create_publisher(Float64, '/ekf_error/orientation', 10)

        self.get_logger().info('EKF Evaluation Node (Assignment 4) Initialized.')

    def sync_cb(self, ekf_msg, gt_msg):
        
        x_gt, y_gt = gt_msg.pose.position.x, gt_msg.pose.position.y
        x_ekf, y_ekf = ekf_msg.pose.pose.position.x, ekf_msg.pose.pose.position.y
        
        pos_err = math.hypot(x_gt - x_ekf, y_gt - y_ekf)

        yaw_gt = self.get_yaw_from_quat(gt_msg.pose.orientation)
        yaw_ekf = self.get_yaw_from_quat(ekf_msg.pose.pose.orientation)
        
        yaw_err = self.wrap_angle(yaw_gt - yaw_ekf)

        self.pub_pos_err.publish(Float64(data=pos_err))
        self.pub_ang_err.publish(Float64(data=yaw_err))

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
    node = Lab6Assignment4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Evaluation node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()