#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
class Tb3StepTest(Node):
   
    def __init__(self):
        super().__init__('tb3_step_test')

        self.declare_parameter('test_axis', 'angular')
        self.declare_parameter('step_value', 0.3)      
        self.declare_parameter('zero_time', 2.0)      
        self.declare_parameter('hold_time', 6.0)        
        self.declare_parameter('rate_hz', 20.0)        

        self.axis = self.get_parameter('test_axis').get_parameter_value().string_value
        self.step_value = self.get_parameter('step_value').get_parameter_value().double_value
        self.zero_time = self.get_parameter('zero_time').get_parameter_value().double_value
        self.hold_time = self.get_parameter('hold_time').get_parameter_value().double_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)

        self.dt = 1.0 / max(self.rate_hz, 1.0)
        self.elapsed = 0.0
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"[StepTest] axis={self.axis} step={self.step_value} "
            f"zero_time={self.zero_time}s hold_time={self.hold_time}s rate={self.rate_hz}Hz"
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

    def tick(self):
        self.elapsed += self.dt
        cmd = Twist()

        if self.elapsed < self.zero_time:
            pass

        elif self.elapsed < self.zero_time + 10 * self.hold_time:
            for i in range(1, 10):
                if self.elapsed < self.zero_time + i * self.hold_time:
                    if self.axis == 'linear':
                        cmd.linear.x = i * self.step_value
                    else:
                        cmd.angular.z = i * self.step_value
                    break 
        else:
            if self.axis == 'linear':
                cmd.linear.x = 0
            else:
                cmd.angular.z = 0

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Tb3StepTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()