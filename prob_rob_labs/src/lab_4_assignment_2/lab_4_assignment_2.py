#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist

class Lab_4Assignment2(Node):
    def __init__(self):
        super().__init__('lab_4_assignment_2')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('linear_x', 0.2)
        self.declare_parameter('angular_z', 0.0)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.linear_x = self.get_parameter('linear_x').get_parameter_value().double_value
        self.angular_z = self.get_parameter('angular_z').get_parameter_value().double_value


        self.pose_pub = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub = self.create_subscription(LinkStates, '/gazebo/link_states',
                                            self.link_states_callback, 10)

        self.target_link = 'waffle_pi::base_footprint'

        self.vel_timer = self.create_timer(0.1, self.publish_velocity)

        self.get_logger().info(
            f'Started lab_4_assignment_2: frame={self.frame_id}, '
            f'linear_x={self.linear_x:.2f}, angular_z={self.angular_z:.2f}'
        )

    def link_states_callback(self, msg: LinkStates):
        try:
            idx = msg.name.index(self.target_link)
        except ValueError:
            self.get_logger().warn_once(f"{self.target_link} not found yet in /gazebo/link_states.")
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose = msg.pose[idx]
        self.pose_pub.publish(pose_msg)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist = msg.twist[idx]
        self.twist_pub.publish(twist_msg)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Lab_4Assignment2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()