#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('lab_4_assignment_1')

        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pose_pub = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)

        self.sub = self.create_subscription(LinkStates, '/gazebo/link_states',
                                            self.link_states_callback, 10)

        self.target_link = 'waffle_pi::base_footprint'
        self.get_logger().info(f'Publishing ground truth for {self.target_link}')

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

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()